# Keep all GUI code in the main loop: if you split it in multithreading it won't work
# Command file read / execute / delete is performed globally, not at SM level

# Relevant methods:
# GUI_main_event_loop: main GUI loop
# GUI_main_SM_commands_execute: reads the command file,
#   executes it according to the SM calling the required transition with machine.model.trigger,
#   key point, here the new state is triggered: sm_in_loop.sm_gui.machine.model.trigger(trigger)
#   entering the new state, and on_enter_each_state is automatically called back

# This class is a state machine collector, each SM has its own tab, the GUI is the same for all SM
# All GUIMain methods that interact with the GUI have to stay in the main loop,
# main_GUI_SM_commands_execute is in a parallel thread and so it can not call directly the MainGUI,
# otherwise you will get "RuntimeError: main thread is not in main loop"
# so it will send events via window.write_event_value

import threading
import traceback

import PySimpleGUI as sg # GUI library
from PIL import Image
import time
import io
import transitions
from pywinauto import keyboard
from FLBaseClass import FLBaseClass
from FLCommands import FLCommands
from FLConstants import UI_LOGORRHOIC, TIME_WAIT_READ_NEXT_COMMAND, ERROR, COMMAND_SHUTDOWN, APP_NAME, \
    IMAGE_TRANSPARENT_PATH, MESSAGE_GUI_STARTUP, AUTO_START_TRANSITION_NAME, USER_ROLE, SOCKET_SERVER_IS_USED, \
    SM_STATE_SEPARATOR, SM_STATE_SEPARATOR_FOR_NLU
from FLGUISM import FLGUISM
from FLOS import screen_coordinates_for_popup, screen_coordinates_for_GUI_main
from pynput import keyboard

from fluently.FLSMStateRecorder import FLSMStateRecorder
from fluently.FLSocketServer import Server

message_GUI = MESSAGE_GUI_STARTUP # So this variable it can also be accessed by FLModel

class FLGUIMain(FLBaseClass):

    def __init__(self, logger):
        super().__init__(logger)
        self.init_SM_done = False
        self.window_main = None
        self.tabgroup_main_GUI = [] # The SM tabs container
        self.GM_GUI_dictionary = {}
        self.GUI_main_needs_update = False
        self.commands = FLCommands(logger)
        self.log_message_last = ""
        self.image_graph = None
        self.image_help = None
        self.message = message_GUI
        self.tab_names_list = []
        self.state_recorder = FLSMStateRecorder()
        if SOCKET_SERVER_IS_USED:
            self.server_socket = Server()
            self.current_state = self.current_state_set("")

    # Pressing the arrow left button allows to select the tab on the left, same with right
    def button_arrows_press(self, keyboard_key):
        # Which tab has been pressed?
        tab_key_active = self.window_main['-TAB_GROUP-'].Get()
#        tab_key_active = tab_key_active.replace("-", "")

        previous_index = None
        next_index = None
        found = False

        keys = list(self.GM_GUI_dictionary.keys())
        for i, key in enumerate(keys):
            if key == tab_key_active:
                if i > 0:
                    previous_index = i - 1
                if i < len(keys) - 1:
                    next_index = i + 1
                break

        try:
            if keyboard_key == keyboard.Key.left:
                self.window_main['-TAB_GROUP-'].Widget.select(previous_index)
                # Extract the current tab name
                selected_tab_name = self.window_main['-TAB_GROUP-'].Widget.tab(previous_index, "text")
            elif keyboard_key == keyboard.Key.right:
                self.window_main['-TAB_GROUP-'].Widget.select(next_index)
        except AttributeError:
            pass

    # Start the keyboard listener in a separate thread
    def button_arrows_listener_start(self):
        with keyboard.Listener(on_press=self.button_arrows_press) as listener:
            listener.join()

    def current_state_set(self, state):
        self.current_state = state
        self.server_socket.message_set(state)

    # Infinite method, it updates the GUI when external commands are processed and update is asked by them
    # and serves the GUI events of all tabs
    def GUI_main_event_loop(self, title=APP_NAME):
        image_GUI_state_name = {} # Remember the name of the state when image_GUI_path has been displayed
        image_GUI_keep = {} # This variable enables displaying of image_GUI_path, when image_GUI_path has been reset to ""
        width_max = height_max = 0
        # For all SM in dictionary, create the corresponding tab and
        # find out how big the app window has to be, in order to fit to the biggest SM image

        # Reverse the order (yaml is reversed now, too), so that at startup EBuddy and Helper are in sync, regarding the topic sm_state
        for key in reversed(list(self.GM_GUI_dictionary)):
        #for key in self.GM_GUI_dictionary: # Original order
            sm_in_loop = self.GM_GUI_dictionary[key]
            # Save graph machine to file, for later GUI update
            sm_in_loop.graph_save_to_file(interactive_mode=False)
            # Extract image image_graph_width and image_graph_height from the graph image
            #sm_gui_file_name = SM_GRAPH_DEFAULT_SIZE + key + ".png"
            # Calculate height_max, width_max for the main GUI, prepare the image for the GUI
            with open(sm_in_loop.sm_graph_path, "rb") as image_file:
                # Without this conversion, scrolling does not work
                self.image_graph = Image.open(image_file)
                bio = io.BytesIO() # creates an in-memory byte buffer
                self.image_graph.save(bio, format="PNG") # saves the image to the bio buffer in PNG format
                image_graph_bytes = bio.getvalue()
                image_graph_width, image_graph_height = self.image_graph.size
                if image_graph_width > width_max: width_max = image_graph_width
                if image_graph_height > height_max: height_max = image_graph_height
                height_max = height_max * 1.4 # Keep some room vertically

                # This image is needed as placeholder, otherwise any later scrolling won't work
                self.image_help = Image.open(IMAGE_TRANSPARENT_PATH)
                bio = io.BytesIO()  # creates an in-memory byte buffer
                self.image_help.save(bio, format="PNG")  # saves the image to the bio buffer in PNG format
                image_help_bytes = bio.getvalue()
                image_help_width, image_help_height = self.image_help.size

            tab_name = sm_in_loop.sm_gui.name
            self.tab_names_list.append(tab_name)
            tab_key = f"-{tab_name}-"
            if (USER_ROLE == 'Beginner'):
                sm_in_loop.tab_layout = [
                    [sg.Column([[sg.Image(data=image_graph_bytes, key=f'-{tab_name}_IMAGE_GRAPH-')]], scrollable=True, size=(image_graph_width, image_graph_height))],
                    [sg.Button("AutoPilot", key=f"-{tab_name}_AUTOPILOT-"),
                     sg.Button("Settings", key=f"-{tab_name}_SEND_COMMANDS-"),
                     # sg.Button("Jump", key=file"-{tab_name}_JUMP-"),
                     # sg.Button("Handle States", key=file'-{tab_name}_HANDLE_STATES-'),
                     # sg.Button("Handle Transitions",key=file"-{tab_name}_HANDLE_TRANSITIONS-"),
                     # sg.Button("Load SM From File", key=file"-{tab_name}_LOAD_SM_FROM_FILE-"),
                     # sg.Button("Save SM To File", key=file"-{tab_name}_SAVE_SM_TO_FILE-"),
                     sg.Button("Save SM To Image", key=f"-{tab_name}_SAVE_SM_TO_IMAGE-"),
                     # sg.Button("Show Cockpit", key=file"-{tab_name}_SHOW_COCKPIT-"),
                     sg.Text("", font=("Helvetica", 24)), sg.Text("", key=f"-{tab_name}_HINT-", font=("Helvetica", 24))],
                    #[sg.Column([[sg.Image(data=image_help_bytes, key=f'-{tab_name}_IMAGE_HELP-')]], key=f'-{tab_name}_COLUMN-', scrollable=True, size=(image_help_width, image_help_height))]
                    [sg.Column([[sg.Image(data=image_help_bytes, key=f'-{tab_name}_IMAGE_HELP-')]], key=f'-{tab_name}_COLUMN-', scrollable=True)]
                ]
            else:
                sm_in_loop.tab_layout = [
                    [sg.Column([[sg.Image(data=image_graph_bytes, key=f'-{tab_name}_IMAGE_GRAPH-')]], scrollable=True, size=(image_graph_width, image_graph_height))],
                    [sg.Button("Send Commands", key=f"-{tab_name}_SEND_COMMANDS-"),
                     sg.Button("AutoPilot", key=f"-{tab_name}_AUTOPILOT-"),
                     sg.Button("Jump", key=f"-{tab_name}_JUMP-"),
                     sg.Button("Handle States", key=f'-{tab_name}_HANDLE_STATES-'),
                     sg.Button("Handle Transitions",key=f"-{tab_name}_HANDLE_TRANSITIONS-"),
                     sg.Button("Load SM From File", key=f"-{tab_name}_LOAD_SM_FROM_FILE-"),
                     sg.Button("Save SM To File", key=f"-{tab_name}_SAVE_SM_TO_FILE-"),
                     sg.Button("Save SM To Image", key=f"-{tab_name}_SAVE_SM_TO_IMAGE-"),
                     sg.Button("Show Cockpit", key=f"-{tab_name}_SHOW_COCKPIT-"),
                     sg.Text("", font=("Helvetica", 24)), sg.Text("", key=f"-{tab_name}_HINT-", font=("Helvetica", 24))],
                    [sg.Column([[sg.Image(data=image_help_bytes, key=f'-{tab_name}_IMAGE_HELP-')]], key=f'-{tab_name}_COLUMN-', scrollable=True, size=(image_help_width, image_help_height))]
                ]

            # Apply user role to tab
            #if ((USER_ROLE == 'Beginner' and 'Preview' in tab_name) or
            if (USER_ROLE == 'Beginner' and 'Setup' in tab_name):
                USER_ROLE_CURRENT_TAB = 'viewer'
            else: USER_ROLE_CURRENT_TAB = 'admin'

            # Create tab
            sm_in_loop.window_sm = sg.Tab(f"{tab_name}", sm_in_loop.tab_layout, key= tab_key, disabled=(USER_ROLE_CURRENT_TAB == 'viewer'))
            self.tabgroup_main_GUI.append([sm_in_loop.window_sm])

        self.layout_main_GUI = [
            [sg.Text("", font=("Helvetica", 24)), sg.Text("", key="-MESSAGE-", font=("Helvetica", 24))],
            [sg.TabGroup(self.tabgroup_main_GUI, key="-TAB_GROUP-", enable_events=True)]
        ]
        # Get the screen resolution
        screen_info = sg.Window.get_screen_size()
        x, y = screen_coordinates_for_GUI_main()
        # Create the window
        self.window_main = sg.Window(title, self.layout_main_GUI, resizable=True, location=(x, y), finalize=True)
        self.window_main.Maximize()
        self.print(f"Main GUI is up and entered the event loop")

        # Pass window_main to all SM, so that they can send it any window events for update requests
        for key in self.GM_GUI_dictionary:
            sm_in_loop = self.GM_GUI_dictionary[key]
            sm_in_loop.window_main_set(self.window_main)
            sm_in_loop.tab_names_list = self.tab_names_list
            tab_name = sm_in_loop.sm_gui.name
            self.window_main[f"-{tab_name}_SAVE_SM_TO_IMAGE-"].update(visible=False)

        # Start the keyboard listener thread
        listener_thread = threading.Thread(target=self.button_arrows_listener_start, daemon=True)
        listener_thread.start()

        # Main infinite window event loop
        while True:
            self.gui_main_loop_is_active = True

            # Method .read shows the window content and immediately starts the event loop,
            # Method .read is blocking, since a GUI or external event arises
            # Method .read must happen before .update, so if this line is missing, a later crash on update will happen
            # event_name of the jump button pressed: -TabCreateMesh(F1)_JUMP-, event_values = '-TabCreateMesh(F1)-'
            # event_name can be risen with something like self.window_main.write_event_value(('-THREAD_READ_COMMANDS-',), self.sm_gui.name)
            event_name, event_values = self.window_main.read()

            # event_name can be returned as a string or a tuple!
            if isinstance(event_name, tuple): # Usually it is a string, not a tuple
                event_name = event_name[0] # Convert '-THREAD_READ_COMMANDS-' to string

            tab_name = ""
            widget_name = ""
            if event_name == sg.WINDOW_CLOSED:
                break # End this loop
            else:
                # Find the tab name to be processed from the event SM_JSON_data
                event_values_list = list(event_values.values())
                if event_name == '-THREAD_READ_COMMANDS-':
                    tab_name = event_values_list[1] #  0 is -SM_Name-, 1 is SM_Name
                    # It is not a GUI event
                    widget_name = ""
                else:
                    # User pressed the tab: event_name == '-TAB_GROUP-'
                    # User pressed a widget within a tab: event_name == '-TabCreateMesh(F1)_JUMP-' for example
                    tab_name = event_values_list[0]
                    # Remove the hyphens "- -" around the name
                    tab_name = tab_name.replace("-", "")
                    # widget_name is defined later, since first we need to know the sm_gui.name
                sm_in_loop = self.GM_GUI_dictionary[tab_name]
                self.print(f'Selected tab: {tab_name}')
                if not event_name == '-THREAD_READ_COMMANDS-':
                    # When jump button is pressed, event_name is -TabCreateMesh(F1)_JUMP-, event_values = '-TabCreateMesh(F1)-'
                    widget_name = event_name.replace(tab_name, '')
                    widget_name = widget_name[:1] + sm_in_loop.sm_gui.name + widget_name[1:]
                # graph_save_to_file is performed here
                sm_in_loop.GUI_tab_event_loop(widget_name)

                # Update tab
                try:
                    if tab_name in image_GUI_state_name and sm_in_loop.sm_gui.machine.model.state != \
                            image_GUI_state_name[tab_name]:
                        image_GUI_keep[tab_name] = False
                        del image_GUI_state_name[tab_name]
                    # Update the SM graph image
                    sm_in_loop.graph_mutex_acquire()
                    self.window_main[f'-{sm_in_loop.sm_gui.name}_IMAGE_GRAPH-'].update(sm_in_loop.sm_graph_path)
                    sm_in_loop.graph_mutex_release()
                    # Update the SM GUI image, if needed
                    if sm_in_loop.sm_gui.machine.model.image_GUI_path != "":
                        self.print(f'Calling Image.open {sm_in_loop.sm_gui.machine.model.image_GUI_path}')
                        self.image_help = Image.open(sm_in_loop.sm_gui.machine.model.image_GUI_path)
                        bio = io.BytesIO() # Convert the image to a byte array
                        self.image_help.save(bio, format="PNG") # saves the image to the bio buffer in PNG format
                        image_help_bytes = bio.getvalue()
                        self.window_main[f'-{sm_in_loop.sm_gui.name}_IMAGE_HELP-'].update(image_help_bytes, visible=True)
                        self.window_main[f'-{sm_in_loop.sm_gui.name}_COLUMN-'].update(visible=True)
                        # Reset variable, so next state will start with no image_GUI_path
                        sm_in_loop.sm_gui.machine.model.image_GUI_path = ""
                        image_GUI_keep[tab_name] = True
                        image_GUI_state_name[tab_name] = sm_in_loop.sm_gui.machine.model.state
                    else:
                        if tab_name in image_GUI_keep and not image_GUI_keep[tab_name] or tab_name not in image_GUI_keep:
                            self.window_main[f'-{sm_in_loop.sm_gui.name}_IMAGE_HELP-'].update(visible=False)
                            self.window_main[f'-{sm_in_loop.sm_gui.name}_COLUMN-'].update(visible=False)

                    # Update the autopilot button
                    if sm_in_loop.autopilot_is_on:
                        self.window_main[f"-{sm_in_loop.sm_gui.name}_AUTOPILOT-"].update(button_color=("white", "green"))
                    else:
                        self.window_main[f"-{sm_in_loop.sm_gui.name}_AUTOPILOT-"].update(button_color=sg.DEFAULT_BUTTON_COLOR)
                    # Update the hint
                    GUI_command = "Command: " + sm_in_loop.hint
                    self.window_main[f"-{sm_in_loop.sm_gui.name}_HINT-"].update(GUI_command)
                    GUI_message = message_GUI # message_GUI = "Message " + message_GUI gives error
                    GUI_message = "Message " + GUI_message
                    self.window_main["-MESSAGE-"].update(GUI_message)
                    self.window_main.refresh()
                except Exception as error:
                    with io.StringIO() as buf:
                        traceback.print_exc(file=buf)
                        error_message = buf.getvalue()
                        self.print(f'Error {error}')
                        self.print(f'Error traceback: {error_message}')
                        pass

        self.gui_main_loop_is_active = False
        self.window_main.close()
        self.print(f'Main window closed now on event {event_name}')

    # Thread with a neverending loop: it reads the command file, executes it, updates the SM via write_event_value
    def GUI_main_SM_commands_execute(self):
        global trigger
        triggers_available = []
        triggers_available_new = []
        time.sleep(2) # Wait for the GUI to create the object
        SM_names_all = list(self.GM_GUI_dictionary.keys())

        # Read commands from file system & activate the corresponding transition
        while True:
            # Read SM or autopilot_is_on_from_GUI commands
            try:
                while True:
                    # This part must always be executed, otherwise there is an error somewhere inside!
                    # 1) Init all SM's at startup inserting fake commands into the queue (and redo this loop until all SM are initialised)
                    # 2) Read file commands and insert them into the commands' queue (SM command mode), according to the context flags
                    self.init_SM_done = self.commands.commands_read_and_insert_into_queue(SM_names_all, self.init_SM_done)
                    # Read queue commands (autopilot_is_on_from_GUI mode)
                    command_to_be_executed = self.commands.queue.element_remove()
                    do_update_state = True
                    if command_to_be_executed != None:
                        # There is a command to be executed
                        self.print(f"command_to_be_executed: SM name {command_to_be_executed.SM_name}, command: {command_to_be_executed.command}, autopilot is {command_to_be_executed.autopilot_is_on}")
                        break # Execute the command
                    else:
                        # There is no command to be executed, thus check if some SM is in autopilot mode, and auto-generate a command
                        for key in self.GM_GUI_dictionary:
                            sm_in_loop:FLGUISM = self.GM_GUI_dictionary[key]
                            sm_in_loop.commands_available = self.triggers_available(sm_in_loop, sm_in_loop.sm_gui.machine.model.state)
                            # If autopilot_is_on_from_GUI mode is active, insert into queue the autopilot_is_on_from_GUI commands
                            if sm_in_loop.autopilot_is_on == True: # YAML file defines which SM has autopilot_is_on_from_GUI
                                if sm_in_loop.commands_available != []:
                                    # Take the first command, but only if there is at least one
                                    my_command = sm_in_loop.commands_available[0]
                                    self.commands.queue.element_insert(sm_in_loop.sm_gui.name, my_command, sm_in_loop.autopilot_is_on)
                            else:
                                if self.init_SM_done and sm_in_loop.commands_available and sm_in_loop.commands_available[0] == AUTO_START_TRANSITION_NAME:
                                    self.commands.queue.element_insert(sm_in_loop.sm_gui.name, AUTO_START_TRANSITION_NAME,
                                                                       sm_in_loop.autopilot_is_on)

                        # Since this loop is continuous and not event triggered, guarantee a "sleep" time within the loop
                        time.sleep(TIME_WAIT_READ_NEXT_COMMAND)

                # Check that the required SM and command exist, then execute the command, update the SM
                sm_in_loop = self.GM_GUI_dictionary.get(command_to_be_executed.SM_name) # Search the SM in the dictionary

                # Autopilot can be set by a command file, do it if needed
                if command_to_be_executed is not None and sm_in_loop is not None:
                    if command_to_be_executed.autopilot_is_on != sm_in_loop.autopilot_is_on:
                        sm_in_loop.autopilot_is_on_set(command_to_be_executed.autopilot_is_on)

                # Execute the command: if the received command is an available trigger, set its condition free
                if(sm_in_loop != None and command_to_be_executed.command != ""):
                    while True: # xxx Can this while be removed?
                        tab_group = self.window_main['-TAB_GROUP-']
                        index_max = len(tab_group.Widget.tabs())
                        update_tab = False
                        # GoToTab: check if the command is a tab switch
                        if "GotoTab" in command_to_be_executed.command:
                            update_tab = True
                            # Search for the tab from which the tab switch command has been said
                            tab_searched = command_to_be_executed.SM_name
                            # Iterate through all tabs
                            for index in range(index_max):
                                tab_name = tab_group.Widget.tab(index, "text") # Read the tab name
                                if tab_name == tab_searched:
                                    index_new = index
                                    if "Right" in command_to_be_executed.command:
                                        index_new += 1
                                    elif "Left" in command_to_be_executed.command:
                                        index_new -= 1
                                    # Correct the index to loop within the bounds
                                    if index_new > index_max:
                                        index_new = 1
                                    elif index_new < 0:
                                        index_new = index_max
                        elif command_to_be_executed.command in self.tab_names_list:
                            update_tab = True
                            tab_searched = command_to_be_executed.command
                            # Iterate through all tabs
                            for index in range(index_max):
                                tab_name = tab_group.Widget.tab(index, "text") # Read the tab name
                                if tab_name == tab_searched:
                                    index_new = index

                        if update_tab:
                            tab_group.Widget.select(index_new)
                            tab_name_new = tab_group.Widget.tab(index_new, "text") # When this command is done, GUI is updated immediately
                            self.window_main.refresh() # Let's do it anyway

                            # xxx Integrate those two
                            my_state = self.state_recorder.return_state(tab_name_new) # Recall the state of this SM
                            self.current_state_set(tab_name_new + SM_STATE_SEPARATOR + my_state)
                            self.state_recorder.store_state(tab_name_new, my_state)

                            # Tell NLU about the new selected tab
                            # Message format: TabName cat State name, for example InstructionsReady
                            ROS2_message = tab_name_new + SM_STATE_SEPARATOR_FOR_NLU + self.state_recorder.return_state(tab_name_new)
                            self.server_socket.send(ROS2_message)
                            do_update_state = False
                            update_tab = False
                            break  # Exit the loop since the tab has been found

                        triggers_available = self.triggers_available(sm_in_loop, sm_in_loop.sm_gui.machine.model.state)
                        # Check if the command received can be found within the available commands
                        if command_to_be_executed.command in triggers_available:
                            # Set transition "semaphore" to green: activate the corresponding transition condition flag, for example CmdStartArtecStudio = True
                            sm_in_loop.sm_gui.transition_condition_set(command_to_be_executed.command, True)
                            break
                        # A bounce command has been received, as Shutdown
                        # command_to_be_executed.command.__contains__(COMMAND_SHUTDOWN):
                        elif command_to_be_executed.command in sm_in_loop.sm_gui.machine.model.states_bounce:
                            break # Goto transitions part below
                        elif "GoToTab" in command_to_be_executed.command:
                            break
                        else:
                            # Command can not be executed
                            sm_in_loop.print(f"Command {command_to_be_executed.command} can not be executed and will be archived")
                            #sm_in_loop.command_delete()
                            self.commands.command_file_archive()
                            break

                    # Evaluate the transition
                    state_before_transitioning = sm_in_loop.sm_gui.machine.model.state
                    sm_in_loop.print(f"State {state_before_transitioning} before trying to transition with command {command_to_be_executed.command}")
                    # Store this state for a potential future state rollback
                    sm_in_loop.sm_gui.machine.model.state_before_transitioning = sm_in_loop.sm_gui.machine.model.state

                    # Is it a Bounce command?
                    #if (command_to_be_executed.command.__contains__(COMMAND_SHUTDOWN)):
                    if command_to_be_executed.command in sm_in_loop.sm_gui.machine.model.states_bounce:
                        # Force the shutdown state without using the transition mechanism, so every state can transition into it
                        #sm_in_loop.sm_gui.machine.model.state = "ShutdownIsActive"
                        # In a bounce state, command name and state name coincide
                        sm_in_loop.sm_gui.machine.model.bounce_to_state(command_to_be_executed.command)
                    # Is it a normal command?
                    else:
                        for trigger in triggers_available:
                            # My transitions are not called "to_" as all default transitions added by the library
                            if not trigger.__contains__('to_'):
                                try:
                                    if(sm_in_loop.sm_gui.machine.model.transition_semaphores[trigger] == True):
                                        # Store trigger value
                                        sm_in_loop.sm_gui.machine.model.trigger_last_last = sm_in_loop.sm_gui.machine.model.trigger_last
                                        sm_in_loop.sm_gui.machine.model.trigger_last = trigger
                                        # Fire the transition trigger
                                        sm_in_loop.print(f"Firing transition {trigger}")
                                        sm_in_loop.sm_gui.machine.model.GUI_update_requested = True
                                        # This call immediately fires the callback
                                        # Unknown problems adding a queue object within FLModel lead to the error 'Queue' object has no attribute 'info',
                                        # the transition fired but the callback was not called anymore: horror!
                                        sm_in_loop.sm_gui.machine.model.trigger(trigger)
                                        # Whatever code after the last line above will be ignored, since that line always triggers a wrong exception!!
                                        # Shutdown received, special case
                                        # This line is probably never executed
                                        sm_in_loop.print(f"Transition {trigger} fired without exceptions")
                                        sm_in_loop.sm_gui.transition_condition_set(command_to_be_executed.command, False)
                                except transitions.core.MachineError as error:
                                    sm_in_loop.print(f"On context trigger {trigger}, a transitions.core.MachineError EXCEPTION0 occurred: {error}", ERROR)
                                except Exception as error:
                                    error_message = str(error)
                                    if "'FLModel' object has no attribute 'transitions'" in error_message:
                                        sm_in_loop.print(f"On context trigger {trigger}, an EXCEPTION1 occurred: {error} => Is this library EXCEPTION a bug?", ERROR)
                                    elif "'FLModel' object has no attribute 'log_file_path'" in error_message:
                                        sm_in_loop.print(f"On context trigger {trigger}, an EXCEPTION2 occurred: {error} => Is this library EXCEPTION a bug?", ERROR)
                                    elif "not enough values to unpack (expected 2, got 0)" in error_message:
                                        sm_in_loop.print(
                                            f"On context trigger {trigger}, an EXCEPTION2B occurred: {error} => Is this library EXCEPTION a bug?", ERROR)
                                    else:
                                        sm_in_loop.print(f"On context trigger {trigger}, a generic EXCEPTION occurred: {error}", ERROR)

                    # State has changed, write to log
                    sm_in_loop.sm_gui.machine.model.state_after_transitioning = sm_in_loop.sm_gui.machine.model.state
                    sm_in_loop.print(f"State after transitioning: {sm_in_loop.sm_gui.machine.model.state_after_transitioning}")
                    self.state_recorder.store_state(sm_in_loop.sm_gui.name, sm_in_loop.sm_gui.machine.model.state_after_transitioning)

                    # Communicate via ROS2 the new EBuddy state to Rocco NLU
                    if SOCKET_SERVER_IS_USED:
                        if do_update_state == True:
                            # We have to remember which sm it was since Rocco 0.1 does not tell this to me
                            self.current_state_set(sm_in_loop.sm_gui.name + SM_STATE_SEPARATOR + sm_in_loop.sm_gui.machine.model.state_after_transitioning)
                            # Message format: TabName space State name, for example InstructionsReady # 1.0 version was TabName cat State name
                            ROS2_message = sm_in_loop.sm_gui.name + " " + sm_in_loop.sm_gui.machine.model.state_after_transitioning
                            self.server_socket.send(ROS2_message)

                    # Find out which commands are available now, update the hints
                    # We implemented a try catch since sometimes (not always) we get strange and not existing errors
                    try:
                        sm_in_loop.commands_available = self.triggers_available(sm_in_loop, sm_in_loop.sm_gui.machine.model.state)
                        sm_in_loop.hint = ' '.join(sm_in_loop.commands_available)
                    except Exception as error:
                        sm_in_loop.print(f"On context trigger {trigger}, an EXCEPTION4 occurred: {error}")

                    # If a shutdown command is received, leave the while loop
                    if command_to_be_executed.command.__contains__(COMMAND_SHUTDOWN):
                        # No GUI update is performed, send exit event to the main GUI
                        self.window_main.write_event_value(sg.WINDOW_CLOSED, 'GUI_main_SM_commands_execute')
                        sm_in_loop.print(f"Command \"{COMMAND_SHUTDOWN}\" received and executed")
                        break

                # Check whether main GUI update has been asked by some callbacks
                # If a SM has to be updated, the whole GUI is updated
                for key in self.GM_GUI_dictionary:
                    sm_in_loop = self.GM_GUI_dictionary[key]
                    if sm_in_loop.sm_gui.machine.model.GUI_update_requested:
                        # Update SM GUI, reset variable
                        sm_in_loop.print(f"Handling GUI_update_requested_by_callback")
                        sm_in_loop.graph_save_to_file(interactive_mode=False)
                        # Check if main window is up and running and can receive events
                        if self.window_main != None:
                            self.GUI_main_update_a_SM(sm_in_loop) #now
                            sm_in_loop.sm_gui.machine.model.GUI_update_requested = False # Reset variable
            except Exception as e:
                print(f"An error occurred: {e}") #nnn
                sm_in_loop.print(f"An error occurred: {e}")

        self.print(f"Endless loop ended")

    # For all SM's do graph_save_to_file and call GUI update
    def GUI_main_update_a_SM(self, sm_in_loop):
        # Save graph machine to file for later viewing
        sm_in_loop.graph_save_to_file(interactive_mode=False)
        if self.window_main != None: # Send events only if the window is up
            #self.window_main.write_event_value(('-THREAD_READ_COMMANDS-',), sm_in_loop.sm_gui.name)
            key = '-THREAD_READ_COMMANDS-'
            value = sm_in_loop.sm_gui.name
            self.window_main.write_event_value(key, value)
            self.print(f"self.window_main.write_event_value, key {key} ,value {value}")

    def GUI_popup_ok(self, text):
        # Define the layout for the popup
        layout = [
            [sg.Text(text)],
            [sg.Button("OK", key="-OK-", bind_return_key=True), sg.Button("Cancel")]
        ]
        x, y = screen_coordinates_for_popup()
        # Create the window
        window = sg.Window("EBuddy Confirmation", layout, finalize=True, location=(x, y))
        # Set focus to the OK button
        window["-OK-"].set_focus()

        # Event loop to handle user interaction
        while True:
            event, _ = window.read()

            if event == sg.WIN_CLOSED or event == "Cancel":
                window.close()
                return "Cancel"
            elif event == "-OK-":
                window.close()
                return "OK"

    # def print(self, *args, **kwargs):
    #     caller_frame = inspect.stack()[1]
    #     caller_info = inspect.getframeinfo(caller_frame[0])
    #     log_message_no_caller = [file"{arg}" for arg in args]
    #     log_message = [file"{caller_info.function}() - {arg}" for arg in args]
    #
    #     if (log_message_no_caller != self.log_message_last):
    #         # print to log file
    #         self.log_file.write(*log_message)
    #         # Standard print call
    #         builtins.print(*log_message, **kwargs)
    #         self.log_message_last = log_message_no_caller

    def triggers_available(self, sm_in_loop, state):
        # Get triggers (available commands) from the current state
        # get_triggers returns ALL potential possible triggers from the current state (also multiple "to_"),
        # thus I have to filter out only the real available commands from this state
        triggers_available = []
        triggers_all = sm_in_loop.sm_gui.machine.get_triggers(sm_in_loop.sm_gui.machine.model.state)
        if UI_LOGORRHOIC: sm_in_loop.print(f"All potentially available triggers: {triggers_all}")
        for trigger in triggers_all:
            # My transitions are not called "to_" as all default transitions added by the library
            if not trigger.__contains__('to_'):
                try:
                    triggers_available.append(trigger)
                    if UI_LOGORRHOIC: sm_in_loop.print(f"Available triggers: {trigger}")
                except Exception as error:
                    sm_in_loop.print(f"Exception executing triggers_available.append(trigger): {error}", ERROR)
        return triggers_available

if __name__ == '__main__':
    gm_main = FLGUIMain()