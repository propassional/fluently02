# Relevant methods:
# sm_in_loop.sm_gui.name contiene il nome dei tabs mostrati in EBuddy
# sm_in_loop.sm_gui.[...]model.name contiene il nome del file json
# Useful GUI methods: sg.popup("SM loaded from: " + sm_file_name)
# GUI_main_event_loop reads the command file, then according to the SM it triggers on_enter_each_state
# button_load_SM_from_file
import threading
import time
import keyboard

# This class handles with the graphic user interface of the FL (Fluently project)
# Relevant methods: GUI_SM_commands_execute obtains the list of all potential transitions from the current state

# When some GUI code in main and some other is in another thread, the event loop is not called: absurd!
# The problem is related to the fact some other gui code is running in multithreading
# Remember that GUI_GUI_SM_commands_execute runs into another thread
# Keep all your GUI code in the main thread!

# https://pysimplegui.com/
# https://github.com/pytransitions
# https://github.com/pytransitions/transitions
# pysimplegui Help: call this method: sg.main_sdk_help()
# Ver5 has a private license, and will expire 08.05.25, renewable
# Ver5 is not for business, I signed for this agreement with propassional@yahoo.com
# Ver4 is completely free and available, but it is marked by them as "pirate" or something like that

# Callbacks
# https://github.com/pytransitions/transitions?tab=readme-ov-file#callback-execution-order
# If you want to log every trigger or event (even if no transition occurs or an exception is raised), you can use the finalize_event callback1.
# This callback will execute regardless of whether a transition took place or an exception occurred.
# If you need to return values from callbacks, keep in mind that callback return values are typically ignored (unless they are transition_semaphores).
# Instead, you can assign variables to your stateful object (such as the model or machine) or pass a referenced object as a parameter2.
# This way, you can achieve the desired feedback without directly returning values from the callback.

import PySimpleGUI as sg
import glob
import os
import shutil
import sys
import pygetwindow
from PIL import Image, ImageTk
import matplotlib.pyplot as plt # If imported, my FLCommand class does not show its window: I don't see the correlation!!
import pyautogui # primarily designed for automating mouse and keyboard actions
from matplotlib.widgets import RectangleSelector
from transitions.extensions import GraphMachine
import io

import FLOS
from FLBaseClass import FLBaseClass
from FLCV2 import FLCV2
from FLCommands import FLCommands
from FLConstants import *
import FLPersistenceJSON
from FLCockpit import FLCockpit
from FLModel import FLModel
from FLOS import screen_coordinates_for_popup
from FLPersistenceJSON import SM_save_to_json

class FLGUISM(FLBaseClass):

    def __init__(self, sm_name, logger):
        super().__init__(logger)
        self.logger = logger
        # GUI part
        # Path to the uncropped SM graph image
        self.sm_graph_default_size_path = SM_GRAPH_DEFAULT_SIZE + sm_name + ".png"
        # Path to the default size SM graph image
        self.sm_graph_cropped_size_path = SM_GRAPH_CROPPED + sm_name + ".png"
        # Path to the current SM graph image, cropped or not depending on the scroll mode
        if SCROLL_IS_ACTIVE:
            self.sm_graph_path = SM_GRAPH_DEFAULT_SIZE + sm_name + ".png"
        else:
            self.sm_graph_path = SM_GRAPH_CROPPED + sm_name + ".png"
        self.window_main = None # For sending gui update events with write_event_value
        self.layout = []
        self.sm_gui = [] # Created in SM_connect
        self.gui_main_loop_is_active = False
        # SM part
        self.autopilot_is_on = False
        self.state_history = []
        self.transitions = {}
        self.hint = ""
        self.commands_available = []
        self.tab_names_list = [] # Copied from FLGUIMain
        # Threading part
        self.thread_escape_stop_flag = None # For stopping the escape thread
        self.thread_escape_handle = None
        self.graph_mutex = threading.Lock()
        self.tab_layout = None
        self.window_sm = None

    def autopilot_is_on_set(self, autopilot_is_on_in):
        self.autopilot_is_on = autopilot_is_on_in
        self.print(f"autopilot_mode is now {self.autopilot_is_on}")
        self.GUI_update()

    def autopilot_toggle(self):
        # There is no need to warn the main window that the autopilot mode changed,
        # since the main window is able to find it out by inquiring self.autopilot_is_on_from_GUI of all SM's
        # Toggle autopilot_is_on_from_GUI button and mode
        if self.autopilot_is_on:
            # Set autopilot from on to off
            self.autopilot_is_on_set(False)
            # Allows the thread to terminate
            if self.thread_escape_stop_flag:
                self.thread_escape_stop_flag.set()
        else:
            # Set autopilot from off to on
            result_is_good = self.autopilot_insert()
            if result_is_good:
                self.autopilot_is_on_set(True)

    def autopilot_insert(self):
        # Enable resetting autopilot via escape button
        # Reading the esc button shall last as the autopilot mode itself
        self.thread_escape_stop_flag = threading.Event()
        self.thread_escape_handle = threading.Thread(target=button_escape_thread,
                                                     args=(self.thread_escape_stop_flag, self))
        self.thread_escape_handle.start()
        return True

    def button_close(self):
        print("User has clicked the close button")
        # You can add any cleanup code here before closing the window
        self.gui_main_loop_is_active = False
        self.root.destroy()

    def button_handle_transitions(self):
        self.GUI_transitions_handle()
        #self.GUI_main_update()

    def button_handle_states(self):
        # Read the text box contents
        self.GUI_states_handle()
        #self.GUI_main_update()

    def button_load_SM_from_file(self, sm_file_name, code_path):
        if not os.path.exists(sm_file_name):
            sm_file_name = self.SM_file_open_browser(JSON_FILES, file_extension="*.json")
        if sm_file_name != '':
            self.sm_gui.machine = FLPersistenceJSON.SMGraph_load_from_json(sm_file_name, code_path, self.logger)
            self.sm_gui.machine.model.transition_semaphores = self.sm_gui.conditions_create()
            if UI_LOGORRHOIC:
                sg.popup("SM loaded from: " + sm_file_name)
            return sm_file_name
        else:
            return ""

    def get_image_files(self, directory):
        supported_extensions = ('.jpg', '.jpeg', '.png', '.gif', '.bmp')
        return [f for f in os.listdir(directory) if f.lower().endswith(supported_extensions)]

    def test_add_button(self, window_layout):
        window_layout.append([sg.Button('Show Image', key=f'-BUTTON-')])
        #window_layout = [[sg.Text('hello world tab1')]]
        pass
    def test_add_buttons(self, window):
        image_directory = r'D:\Banfi\Github\Fluently\Releases\2024_07_09_EBuddy\Data\Input\App\ArtecStudio\Hints'
        image_files = self.get_image_files(image_directory)
        if not image_files:
            sg.popup('No images found in the directory')
            return

        window.extend_layout(window, [
            [sg.Button('Show First Image')],
            [sg.Image(key='-IMAGE-')]
        ])

    def button_save_SM_to_image(self):
        self.graph_save_to_file(interactive_mode=True)

    def button_save_SM_to_file(self):
        # self.SM_file_save_browser(PICKLE_FILES, ".pickle") # Error
        self.SM_file_save_browser(JSON_FILES, ".json")

    def button_send_commands(self):
        # When this code is in main, at the beginning of everything, the event loop is called: normal
        # When this code is in main, at the end of everything, the event loop is not called: absurd!
        # command_send_GUI = FLCommand()
        # command_send_GUI_thread = threading.Thread(target=command_send_GUI.GUI_command_send)
        # command_send_GUI_thread.start()

        # Because of the above problem, I open this window as sub-window
        command_send_GUI = FLCommands()
        command_send_GUI.GUI_command_send()
        #self.GUI_main_update()

    def button_show_cockpit(self):
        cockpit_GUI = FLCockpit(self)
        cockpit_GUI.show(self.sm_gui.machine.model.transition_semaphores, self.gui_main_loop_is_active,
                         self.log_file.log_file_name)

    # Move the newest *.png image from operator to SM_IMAGES, calling it as the transition_name
    def graph_machine_delete_state(self, state_to_remove=""):
        extracted_states = [state.name for state in self.sm_gui.machine.states.values()]

        # Extract all transitions from SM
        extracted_transitions = []
        # for transition in machine_source.events.values():
        for transition in self.sm_gui.machine.events.values():
            for model_transition in transition.transitions.values():
                for dest in model_transition:
                    extracted_transitions.append({
                        'trigger': transition.name,
                        'source': dest.source,  # dest.source.name,
                        'dest': dest.dest  # dest.dest.name
                    })

        # Remove the state
        # Specify a default value (e.g., None) to avoid KeyError if the key is not present
        #  Removes the first occurrence in the list, without knowing its index
        # Second variant
        #del self.machine.states[state_to_remove]
        extracted_states.remove(state_to_remove)
        self.print(f"Removed state: {str(state_to_remove)}")

        # Remove all state's transitions
        for transition in extracted_transitions.copy():
            if state_to_remove in transition['source'] or state_to_remove in transition['dest']:
                extracted_transitions.remove(transition)
                self.print(f"Removed transition: {str(transition)}")

        # Build a new state machine using the extracted states and transitions
        model = FLModel()
        if len(extracted_states) > 0:
            extracted_states_initial = extracted_states[0]
        else:
            extracted_states_initial = ''

        graph_machine_updated = GraphMachine(model=model, states=extracted_states,
                                             transitions=extracted_transitions,
                                             initial=extracted_states_initial, show_conditions=True)
        #graph_machine_updated.get_graph().draw(SM_GRAPH_UPDATED, format='png', prog='dot')
        #graph_machine_updated.get_graph().draw(SM_GRAPH_DEFAULT_SIZE, format='png', prog='dot')
        self.graph_save_to_file(interactive_mode=False)
        self.sm_gui.machine = graph_machine_updated

    def graph_mutex_acquire(self):
        self.graph_mutex.acquire()
        if UI_LOGORRHOIC: self.print(f"Mutex acquired")

    def graph_mutex_release(self):
        self.graph_mutex.release()
        if UI_LOGORRHOIC: self.print(f"Mutex released")

    def graph_save_to_file(self, interactive_mode=False):
        # Problem: save (for displaying) big machines
        # Proposed approach (experimented but not integrated into my code since it always creates empty png): save SM into .dot format
        # https://stackoverflow.com/questions/13417411/laying-out-a-large-graph-with-graphviz
        # Download text file of a huge SM, rename it to .dot
        # In terminal: sfdp -x -Goverlap=scale -Tpng SM_JSON_data.dot > SM_JSON_data.pngwith-graphviz
        # gm_default.machine.get_graph().save('state_machine.dot')
        # This stores the big image into png: dot -Tpng state_machine.dot -o state_machine.png
        # (graph,) = pydot.graph_from_dot_file('your_file.dot')
        # graph.write_png('your_file.png')

        self.graph_mutex_acquire()
        # Graphical layouts can be set/overwritten in this method
        # https://graphviz.org/docs/layouts/
        # By default rankdir is set to 'LR', which means that the graph is laid out from left to right, 'TB' from top to bottom
        # Create the graph
        graph = self.sm_gui.machine.get_graph()
        # Good for small SM, but many overlaps on big SM
        graph.graph_attr['rankdir'] = 'TB' # 'LR' vs 'TB', Neato & fdp & sfdp does not support rankdir
        #self.sm_gui.machine.get_graph().draw(SM_GRAPH_DEFAULT_SIZE, format='png', prog='sfdp', args='-Goverlap=scale')

        # Save uncropped graph image, used for creating the cropped image, if needed
        my_format = 'png'
        my_prog = 'dot'

        # Save the SM graph to file
        # self.sm_gui.machine.get_graph().draw(self.SM_graph_default_path, format=my_format, prog=my_prog) # Works well for LR graph, rankdir does not change it
        # message = file"SM graph saved as {self.SM_graph_default_path}"
        try:
            # Save the graph file
            self.sm_gui.machine.get_graph().draw(self.sm_graph_default_size_path, format=my_format, prog=my_prog) # Works well for LR graph, rankdir does not change it
            message = f"Saved current state {self.sm_gui.machine.model.state} as image {self.sm_graph_default_size_path}"
            if interactive_mode: sg.popup(message)
            self.print(message)
        except Exception as e:
            self.print(f"graph_save_to_file: an error occurred while saving graph: {str(e)}", ERROR)

        if not SCROLL_IS_ACTIVE:
            # Save cropped imaged
            my_FLCV2 = FLCV2(self.logger)
            my_FLCV2.image_around_active_state_save(self.sm_graph_default_size_path, self.sm_graph_cropped_size_path)

        self.graph_mutex_release()

        #self.sm_gui.machine.get_graph().draw(SM_GRAPH_DEFAULT_SIZE, format='png', prog='neato') # Works best, but it has some overlap
        #self.sm_gui.machine.get_graph().draw(SM_GRAPH_DEFAULT_SIZE, format='png', prog='neato', args='-Goverlap=voronoi')  # Works best
        #self.sm_gui.machine.get_graph().draw(SM_GRAPH_DEFAULT_SIZE, format='png', prog='neato', args='-Goverlap=prism')  # Many overlaps
        #self.sm_gui.machine.get_graph().draw(SM_GRAPH_DEFAULT_SIZE, format='png', prog='neato', args='-Goverlap=scale')  # Many overlaps
        #self.sm_gui.machine.get_graph().draw(SM_GRAPH_DEFAULT_SIZE, format='png', prog='neato', args='-Goverlap=compress') # Many overlaps
        #self.sm_gui.machine.get_graph().draw(SM_GRAPH_DEFAULT_SIZE, format='png', prog='neato', args='-Goverlap=vpsc') # Many overlaps
        #self.sm_gui.machine.get_graph().draw(SM_GRAPH_DEFAULT_SIZE, format='png', prog='sfdp', args='-Goverlap=voronoi')  # Works, but has some text overlap

        # Other alternatives
        # Draw just the region of interest: (previous state, active state and all reachable states)
        #   self.sm_gui.machine.get_graph(show_roi=True).draw(SM_GRAPH_DEFAULT_SIZE, format='png', prog='dot')  # Works
        #self.sm_gui.machine.get_graph().draw(SM_GRAPH_DEFAULT_SIZE, format='png', prog='twopi') # Works, but bad graphics
        #self.sm_gui.machine.get_graph().draw(SM_GRAPH_DEFAULT_SIZE, format='png', prog='fdp') # Works, but bad graphics
        #self.sm_gui.machine.get_graph().draw(SM_GRAPH_DEFAULT_SIZE, format='png', prog='sfdp') # Works, but some text overlap
        #self.sm_gui.machine.get_graph().draw(SM_GRAPH_DEFAULT_SIZE, format='png', prog='patchwork') # Works, but bad graphics
        #self.sm_gui.machine.get_graph().draw(SM_GRAPH_DEFAULT_SIZE, format='png', prog='osage') # Works well for states, but transitions overlapping

    def GUI_tab_event_loop(self, event):
        # Event loop
        while True:
            if event == sg.WINDOW_CLOSED:
                break
            elif event == f"-{self.sm_gui.name}_AUTOPILOT-":
                self.autopilot_toggle() # Toggle autopilot mode
                break
            elif event == f"-{self.sm_gui.name}_JUMP-":
                self.jump()
                break
            elif event == f"-{self.sm_gui.name}_HANDLE_TRANSITIONS-":
                self.GUI_transitions_handle()
                break #self.GUI_main_update()
            elif event == f"-{self.sm_gui.name}_HANDLE_STATES-":
                # Read the text box contents
                self.GUI_states_handle()
                break #self.GUI_main_update()
            elif event == f"-{self.sm_gui.name}_LOAD_SM_FROM_FILE-":
                sm_file_name = self.SM_file_open_browser(JSON_FILES, file_extension="*.json")
                if sm_file_name != '':
                    self.sm_gui.machine = FLPersistenceJSON.SMGraph_load_from_json(sm_file_name)
                    # self.sm_gui.machine = pickle.load(sm_file_handle)
                    break #self.GUI_main_update()
                    if UI_LOGORRHOIC:
                        sg.popup("SM loaded from default pickle file: " + SM_FILE_PICKLE_DEFAULT)
            elif event == f"-{self.sm_gui.name}_SAVE_SM_TO_IMAGE-":
                self.graph_save_to_file(interactive_mode=True)
                break
            elif event == f"-{self.sm_gui.name}_SAVE_SM_TO_FILE-":
                # self.SM_file_save_browser(PICKLE_FILES, ".pickle") # Error
                self.SM_file_save_browser(JSON_FILES, ".json")
                break
            elif event == f"-{self.sm_gui.name}_SEND_COMMANDS-":
                # Extract bounce states list
                self.states_bounce_list = list(self.sm_gui.machine.model.states_bounce.values())
                self.states_bounce_names_list = []
                for i in range(len(self.states_bounce_list)):
                    self.states_bounce_names_list.append(self.states_bounce_list[i].name)

                # Create all possible commands
                self.commands_available_all = []
                # Add commands that available from the current state in the current SM
                for command in self.commands_available:
                    self.commands_available_all.append(command)
                # Add bounce commands
                for command in self.states_bounce_names_list:
                    self.commands_available_all.append(command)
                # Filter tabs commands
                tab_names_list_filtered = self.tab_names_list.copy() # Can not be self, since this list should not be saved
                # Remove the current tab, since it is already active
                tab_names_list_filtered.remove(self.sm_gui.name) #nnn
                # Append together all possible commands
                command_send_GUI = FLCommands(self.logger)
                command_send_GUI.GUI_command_send(self.sm_gui.name, self.commands_available_all, tab_names_list_filtered)
                break
            elif event == f"-{self.sm_gui.name}_SHOW_COCKPIT-":
                cockpit_GUI = FLCockpit(self)
                cockpit_GUI.show(self.sm_gui.machine.model.transition_semaphores, self.gui_main_loop_is_active,
                                 self.logger)
                break
            else:
                # Do nothing, since it was not a GUI event, but update the graph file
                self.graph_save_to_file()
                break

    def GUI_SM_create(self):
        # Pattern: create a minimal layout & window before the window event while loop,
        # update its SM_JSON_data within the while loop, close it afterwards
        machine_states_array = list(self.sm_gui.machine.states)

        # Save graph machine to file for later viewing
        self.graph_save_to_file(interactive_mode=False)

        # Extract image width, height from graph image, the uncropped one
        #with open(SM_GRAPH_DEFAULT_SIZE + self.sm_gui.name + ".png", "rb") as image_file:
        with open(self.sm_graph_path, "rb") as image_file:
            self.image = Image.open(image_file)
            # Convert the image to a byte array
            bio = io.BytesIO()
            self.image.save(bio, format="PNG")
            image_bytes = bio.getvalue()
            width, height = self.image.size
            height = height * 1.4  # Keep some room vertically

        # self.layout = [
        #     # IMPORTANT: sg.Image("") must be EMPTY!, otherwise the first image is always displayed withing the GUI,
        #     # despite of the call of a update, it stays always there, and so the GUI can not be updated correctly later!
        #     #[sg.Image(""), sg.Image(key="-SM_DIAGRAM-")],
        #     [sg.Column([[sg.Image(SM_JSON_data=image_bytes, key="-SM_DIAGRAM-")]], scrollable=True, vertical_scroll_only=False, size=(width, height))],
        #     [sg.Button("Handle States"), sg.Button("Handle Transitions"), sg.Button("Load SM From File"),
        #      sg.Button("Save SM To File"), sg.Button("Save SM To Image"), sg.Button("Send Commands"), sg.Button("Show Cockpit"),
        #      sg.Text("Hint:", font=("Helvetica", 24)), sg.Text("", key="-HINT-", font=("Helvetica", 24))]
        # ]
        #
        # self.window_main = sg.Window(title, self.layout, resizable=True)


    def GUI_states_handle(self):
        # Implementation alternatives:
        # v) Extract state machine, so that states and transitions can be deleted, then create a new SM
        # x) Store sm to configuration files (JSON or YAML), edit them, load file into SM
        # x) Exclude deleted states? HierarchicalMachine allows defining substates either with the keyword children or states. If both are present, only children will be considered.
        # x) Use multiple states machines for avoid to delete states? or is it possible to disactivate a state?
        # v) Using remap will copy events and transitions: ok, but delete its parts stays unsolved
        # v) from copy import deepcopy: ok, but delete its parts stays unsolved
        # v) Use another library: bad, since all three main free libs have no delete states nor transitions
        #    https://pypi.org/project/statesman/ positive: last release a couple of months ago, UML, negative: no graphs

        machine_states_array = list(self.sm_gui.machine.states)
        layout_states = [
            #[sg.Text("Macro State name:"), sg.InputText("", key="-MACRO_STATE_NAME-"), sg.Button("Create Macro State")],
            #[sg.Text("Previous State name:"), sg.Combo(values=machine_states_array, size=(30, 5), key='-COMBO_MACRO_STATE-', enable_events=True)],
            [sg.HorizontalSeparator()],
            [sg.InputText("", size=(30, 5), key="-STATE_NAME-"), sg.Button("Create State")],
            [sg.Combo(values=machine_states_array, size=(30, 5), key='-COMBO_DELETE_STATE-', enable_events=True), sg.Button("Delete State", key='-BTN_DELETE_STATE-') ],
        ]
        x, y = screen_coordinates_for_popup()
        window_states = sg.Window("Handle States", layout_states, finalize=True, location=(x, y))
        window_states['-BTN_DELETE_STATE-'].Widget.configure(state=sg.tk.DISABLED)

        while True:
            # .read shows the window content and immediately starts the event loop,
            # so only "if event" code parts will be executed
            # .read must happen before .update, so if this line is missing, a later crash on update will happen
            event, values = window_states.read()

            # Read all window input fields
            try:
                state_name = values["-STATE_NAME-"]
            except Exception as e:
                state_name = []
            try:
                combo_delete_option = values['-COMBO_DELETE_STATE-']
            except Exception as e:
                combo_delete_option = []

            if event == sg.WINDOW_CLOSED or event == "Cancel" or event == 'Exit':
                break

            elif event == "Create State":
                if state_name != "":
                    # Check if state is already defined
                    if state_name in self.sm_gui.machine.states.values():
                        # All these dots are needed otherwise the window title is not shown
                        sg.popup("The state " + state_name + " is already defined.........", title='Warning')
                    else:
                        # A new state can be defined
                        self.sm_gui.machine.add_states([state_name])
                        self.print(f"Added state: {state_name}")
                        # Delete the text input field
                        state_name = ""
                        window_states["-STATE_NAME-"].update(state_name)
                        machine_states_array = list(self.sm_gui.machine.states)
                        window_states['-COMBO_DELETE_STATE-'].update(values=machine_states_array)
                        break #self.GUI_main_update()
                else:
                    sg.popup('State field is empty, please insert a state name.............', title='Warning')

            # Not used for the moment
            elif event == "Create Macro State":
                if combo_delete_option == "":
                    sg.popup("Select the previous state", title='Warning')
                else:
                    macro_state_name = values["-MACRO_STATE_NAME-"]
                    if macro_state_name != "":
                        # Check if state is already defined
                        if macro_state_name in self.sm_gui.machine.states.values():
                            # All these dots are needed otherwise the window title is not shown
                            sg.popup("The macro state " + macro_state_name + " is already defined.........", title='Warning')
                        else:
                            # New macro state is not yet defined
                            # User asks sw to insert Recording state (selects IsOff/IsOn), selects a transition from stateN-1 (cmd/?):
                            # sw adds two states (RecordingIsOn, RecordingDo), and four transitions (CmdRecording, RecordingDone 2x, !statedone
                            state_do = macro_state_name + "Do"
                            state_is = macro_state_name + "IsOn"
                            self.sm_gui.machine.add_states(state_do)
                            self.sm_gui.machine.add_states(state_is)
                            transition_done = macro_state_name + 'Done'
                            combo_delete_option = values['-COMBO_MACRO_STATE-']
                            state_previous = combo_delete_option
                            self.sm_gui.machine.add_transition("Cmd" + macro_state_name, state_previous, state_do)
                            self.sm_gui.machine.add_transition(transition_done, state_previous, state_is)
                            self.sm_gui.machine.add_transition("!" + transition_done, state_do, state_previous)
                            self.sm_gui.machine.add_transition(transition_done, state_do, state_is)
                            # Delete the text input field
                            macro_state_name = ""
                            window_states["-MACRO_STATE_NAME-"].update(macro_state_name)
                            machine_states_array = list(self.sm_gui.machine.states)
                            window_states["-COMBO_MACRO_STATE-"].update(machine_states_array)
                            window_states['-COMBO_DELETE_STATE-'].update(values=machine_states_array)
                            break #self.GUI_main_update()
                    else:
                        sg.popup('Please insert a macro state name.................', title='Warning')

            elif event == "-BTN_DELETE_STATE-":
                # Check if state has transitions is not done, since state and its transitions are deleted

                # Delete the selected state
                state_to_remove = combo_delete_option

                # If you want to add a state, this method exists: self.machine.add_states(['Start'])
                # If you want to remove a state, unfortunately this method does not exist: self.sm_gui.machine.remove_state(state_to_remove)
                # This alternative neither works: del self.sm_gui.machine.states[state_to_remove]
                # Thus, I have to extract from SM all states and transitions, delete that state, and create a new graph machine:
                self.graph_machine_delete_state(state_to_remove)

                machine_states_array = list(self.sm_gui.machine.states)
                window_states['-COMBO_DELETE_STATE-'].update(values=machine_states_array)
                break #self.GUI_main_update()

            elif event == '-COMBO_DELETE_STATE-':
                combo_delete_option = values['-COMBO_DELETE_STATE-']
                # Since the option has been chosen, enable the button
                window_states['-BTN_DELETE_STATE-'].Widget.configure(state=sg.tk.NORMAL)
                break  # self.GUI_main_update()

        window_states.close()
    # Precondition: sm_gui.machine has already been created

    def GUI_transitions_handle(self):
        button_create_image = "Create Cmd Image"
        button_create_text = "Create Cmd Text"
        text_to_be_typed = ''
        machine_states_array = list(self.sm_gui.machine.states)
        layout_transitions = [
            [sg.Text("Transition Name:"), sg.InputText("", key="-TRANSITION_NAME-")],
            [sg.Text("Predecessor State:"), sg.Combo(values=machine_states_array, size=(30, 5), key='-DROPDOWN_PREDE-', enable_events=True)],
            [sg.Text("Successor State:"), sg.Combo(values=machine_states_array, size=(30, 5), key='-DROPDOWN_SUC-', enable_events=True)],
            [sg.Button(button_create_image, key='-BUTTON_CREATE_IMAGE-')],
            [sg.Button(button_create_text, key='-BUTTON_CREATE_TEXT-'), sg.Text('Text to be typed:'), sg.InputText(text_to_be_typed, key="-TEXT_TO_BE_TYPED-")],
            [sg.Button("Create Transition", key='-BUTTON_CREATE_TRANSITION-'), sg.Button("Delete Transition")],
        ]
        x, y = screen_coordinates_for_popup()
        window_transitions = sg.Window("Handle Transitions", layout_transitions, finalize=True, location=(x, y))

        button_color_default = sg.theme_button_color()
        button_color_inverted = (button_color_default[1], button_color_default[0])
        window_transitions['-BUTTON_CREATE_TEXT-'].update(button_color=button_color_default)

        # button_create_text_color = ('white', 'blue')  # (text color, background color)
        # window_transitions['-BUTTON_CREATE_TEXT-'].update(button_color=button_create_text_color)

        while True:
            # .read shows the window content and immediately starts the event loop,
            # so only "if event" code parts will be executed
            # .read must happen before .update, so if this line is missing, a later crash on update will happen
            event, values = window_transitions.read()

            if event == sg.WINDOW_CLOSED:
                break
            elif event == '-DROPDOWN_PREDE-':
                predecessor_selected_option = values['-DROPDOWN_PREDE-']
            elif event == '-DROPDOWN_SUC-':
                successor_selected_option = values['-DROPDOWN_SUC-']
            elif event == '-BUTTON_CREATE_TEXT-':
                # Check if transition name is defined
                transition_name = values["-TRANSITION_NAME-"]
                if (transition_name != ""):
                    text_to_be_typed = values["-TEXT_TO_BE_TYPED-"]
                    if(text_to_be_typed != ""):
                        # Create a text file, with the same name of the transition, containing the text to be used
                        self.text_to_be_typed_create(transition_name, text_to_be_typed)
                        window_transitions["-TEXT_TO_BE_TYPED-"].update(text_to_be_typed)
                        # Show that the button has been pushed
                        window_transitions['-BUTTON_CREATE_TEXT-'].update(button_color=button_color_inverted)
                    else:
                        sg.popup('Warning: please insert text to be typed before!', title='Text missing')
                else:
                    sg.popup('Warning: please insert transition name before!', title='Transition name missing')
            elif event == '-BUTTON_CREATE_IMAGE-':
                # Check if transition name is defined
                transition_name = values["-TRANSITION_NAME-"]
                if (transition_name != ""):
                    self.image_command_create(transition_name)
                    # Show that the button has been pushed
                    window_transitions['-BUTTON_CREATE_IMAGE-'].update(button_color=button_color_inverted)
                else:
                    sg.popup('Warning: please insert transition name before!', title='Transition name missing')
            elif event == '-BUTTON_CREATE_TRANSITION-':
                # Read the text box contents
                transition_name = values["-TRANSITION_NAME-"]
                if (transition_name != ""
                        and predecessor_selected_option != ""
                        and successor_selected_option != ""):
                    self.sm_gui.machine.add_transition(transition_name, predecessor_selected_option,
                                                       successor_selected_option)
                    self.print(f'Added transition called {transition_name}, predecessor {predecessor_selected_option}, successor {successor_selected_option}')
                    # self.graph_save_to_file_default()
                    # window_transitions.window["-TRANSITION_NAME-"].update("")
                    # self.window["-SM_DIAGRAM-"].update(values=SM_GRAPH_DEFAULT_SIZE)
                    # self.window.refresh()
                    break
                else:
                    error_msg = 'Transition SM_JSON_data is missing'
                    error_title = 'please complete missing transition information'
                    self.print(f'Error: {error_title}, {error_msg}')
                    sg.popup(f'Error: {error_title}, {error_msg}', title=error_title)
        window_transitions.close()

    def GUI_update(self):
        # This command is not enough for triggering gui update, since execute stays in read file (none) and execute the queue (empty)...
        self.sm_gui.machine.model.GUI_update_requested = True
        # ...thus trigger this GUI window event
        #self.window_main.write_event_value(('-THREAD_READ_COMMANDS-',), self.sm_gui.name)
        self.window_main.write_event_value('-THREAD_READ_COMMANDS-', self.sm_gui.name)

    def image_command_create(self, transition_name):
        screenshot = pyautogui.screenshot()
        screenshot.save(SCREENSHOT_DEFAULT)
        screenshot_path = SCREENSHOT_DEFAULT
        # img = Image.open(screenshot_path)
        try:
            # Open the image
            img = Image.open(screenshot_path)

            # Display the image
            plt.imshow(img)
            plt.title("Select a rectangular region")
            plt.axis("on")  # Turn off axis ticks
            plt.show()

            # Create a dialog box for entering the region
            # root = tk.Tk()
            # root.title("Enter the region")
            # left_label = tk.Label(root, text="Left:")
            # left_label.grid(row=0, column=0)
            # left_entry = tk.Entry(root)
            # left_entry.grid(row=0, column=1)
            # upper_label = tk.Label(root, text="Upper:")
            # upper_label.grid(row=1, column=0)
            # upper_entry = tk.Entry(root)
            # upper_entry.grid(row=1, column=1)
            # right_label = tk.Label(root, text="Right:")
            # right_label.grid(row=2, column=0)
            # right_entry = tk.Entry(root)
            # right_entry.grid(row=2, column=1)
            # lower_label = tk.Label(root, text="Lower:")
            # lower_label.grid(row=3, column=0)
            # lower_entry = tk.Entry(root)
            # lower_entry.grid(row=3, column=1)
            # ok_button = tk.Button(root, text="OK", command=get_region)
            # ok_button.grid(row=4, column=0, columnspan=2)
            # root.mainloop()

            # This works only if the command image is of one type, in this case, the artec studio type
            # Check the new implementation with link updated by SMB
            # Check the newest file called *.png in C:\Users\operator, overwrite its name with transition_name, then move it to dir2
            # return self.image_command_move(transition_name)

        except KeyboardInterrupt:
            # Handle the case where the user presses Ctrl+C to exit the program
            print("Program terminated by user.")
            sys.exit(0)

    def image_command_move(self, transition_name):
        # Define the source directory and target directory
        source_dir = r'C:\Users\operator'
        new_image_name_target = SM_COMMAND_IMAGES

        # Get a list of all PNG files in the source directory
        png_files = glob.glob(os.path.join(source_dir, '*.png'))

        # Sort the files by modification time (newest first)
        png_files.sort(key=os.path.getmtime, reverse=True)

        if png_files:
            # Get the newest PNG file
            newest_png = png_files[0]
            # Create the new filename (transition_name.png)
            new_image_name = f'{transition_name}.png'
            new_image_name_complete = os.path.join(source_dir, new_image_name)
            # Rename the image
            os.rename(newest_png, new_image_name_complete)
            # Move the renamed file to the target directory
            new_image_name_target = os.path.join(new_image_name_target, new_image_name)
            shutil.move(new_image_name_complete, new_image_name_target)
            self.print(f"File '{new_image_name}' moved to '{new_image_name_target}' successfully.")
            return new_image_name_target
        else:
            warning = f"No PNG files found in {source_dir}"
            self.print(warning)
            return warning

    def jump(self):
        states = []
        for item in self.sm_gui.machine.states: # self.sm_gui.machine.states is a Dictionary
            states.append(item)
        states = sorted(states)
        states_len = len(states)

        # Define the layout of the GUI
        layout = [
            [sg.Text("Select a state to jump into(forced transition)")],
            [sg.Listbox(values=states, size=(20, states_len), key="-STATE-", enable_events=True)],
            [sg.Button("OK"), sg.Button("Cancel")]
        ]

        # Create the window
        window = sg.Window("State Selector", layout)
        while True:
            event, values = window.read()

            if event == sg.WINDOW_CLOSED or event == "Cancel":
                selected_state = None
                break
            elif event == "OK":
                selected_state = values["-STATE-"][0]
                break

        window.close()

        if selected_state:
            self.print(f"State selected for autopilot mode: {selected_state}")
            # Force transition
            self.sm_gui.machine.model.jump_to_state(selected_state)
            self.GUI_update()

    def on_enter_state(self):
        self.model.log_file_path.write("Current state: " + self.state)
        self.log_file.write("Current transitions: " + str(self.transitions))
        self.log_file.write("Current state history: " + str(self.state_history))

        # if len(self.state_history) >= 2:
        #     self.transitions[self.state] = self.state_history[-2]
        # self.state_history.append(self.state)
        if len(self.state_history) >= 1:
            self.transitions[self.state] = self.state_history[-1]
        self.state_history.append(self.state)
        self.log_file.write("Updated transitions: " + str(self.transitions))
        self.log_file.write("Updated state history: " + str(self.state_history))

    def on_select(eclick, erelease, ax):
        x1, y1 = eclick.xdata, eclick.ydata
        x2, y2 = erelease.xdata, erelease.ydata
        rect_width = abs(x2 - x1)
        rect_height = abs(y2 - y1)
        rect_x = min(x1, x2)
        rect_y = min(y1, y2)
        rect = plt.Rectangle((rect_x, rect_y), rect_width, rect_height, linewidth=1, edgecolor='r', facecolor='none')
        ax.add_patch(rect)
        plt.draw()
        plt.savefig('selected_region.png')
        plt.show()

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

    def SM_connect(self, sm):
        self.sm_gui = sm

    def SM_file_open_browser(self, path=r'D:\\', file_extension =""):

        file_types = [("", file_extension)]

        layout = [[sg.Text("Please select a SM json file:")],
                  [sg.In(path, key='file_input_path')],
                  [sg.FileBrowse(initial_folder=path, file_types=file_types, target='file_input_path'),
                   sg.Submit(), sg.Cancel()]]
        x, y = screen_coordinates_for_popup()
        window = sg.Window('File Browser', layout, location=(x, y))

        # Event loop
        while True:
            event, values = window.read()
            if event in (None, 'Cancel'):
                break
            if event == 'Submit':
                self.print(f"You selected {values['file_input_path']}")
                window.close()
                return values['file_input_path']
        window.close()
        return ""

    def SM_file_save_browser(self, path=r'D:\\', file_extension=""):
        current_datetime = datetime.now()
        formatted_timestamp = current_datetime.strftime("%Y_%m_%d__%H_%M_%S")
        file_name = "SM_" + formatted_timestamp + file_extension

        layout = [[sg.Text("Please select a directory:")],
                  [sg.In(path, key='dir_path'), sg.Button("Browse") ],
                  [sg.Text("Please enter a file name:")],
                  [sg.In(file_name, key='file_name')],
                  [sg.Button("SaveSM"), sg.Button("CreateDir"), sg.Cancel()]]
        x, y = screen_coordinates_for_popup()
        window = sg.Window('File Browser', layout, location=(x, y))

        # Event loop
        while True:
            event, values = window.read()
            if event in (None, 'Cancel'):
                break
            if event == 'Browse':
                selected_folder = sg.popup_get_folder('Select a folder', no_window=True, initial_folder=path)
                if selected_folder:
                    window['dir_path'].update(selected_folder)
                break
            if event == 'CreateDir':
                current_datetime = datetime.now()
                formatted_timestamp = current_datetime.strftime("%Y_%m_%d__%H_%M_%S")
                dir_name = os.path.join(SM_FILES, formatted_timestamp)
                os.makedirs(dir_name, exist_ok=True)
                break
            if event == 'SaveSM':
                sm_file_name = os.path.join(values['dir_path'], values['file_name'])
                try:
                    #pickle_file.write(self.sm_gui.machine)
                    #pickle.dump(self.sm_gui.machine, sm_file)
                    SM_save_to_json(self.sm_gui.machine, sm_file_name)
                except Exception as e:
                    self.print(f"An error occurred: {str(e)}", ERROR)
                    window.close()
                break

        window.close()

    def screenshot_do_temp(self):
        screenshot = pyautogui.screenshot()
        screenshot.save(SCREENSHOT_DEFAULT)
        screenshot_path = SCREENSHOT_DEFAULT
        img = Image.open(screenshot_path)

        # Display the image
        fig, ax = plt.subplots()
        ax.imshow(img)
        ax.set_title("Select a rectangular region")
        ax.axis("on")  # Turn off axis ticks

        # Create the rectangle selector
        rect_selector = RectangleSelector(ax, onselect, drawtype='box', useblit=True, button=[1], minspanx=5,
                                          minspany=5, spancoords='pixels')

    def screenshot_do(self):
        screenshot = pyautogui.screenshot()
        screenshot.save(SCREENSHOT_DEFAULT)
        screenshot_path = SCREENSHOT_DEFAULT
        img = Image.open(screenshot_path)

        # Display the image
        plt.imshow(img)
        plt.title("Select a rectangular region, save image")
        plt.axis("on")  # Turn off axis ticks
        plt.show()

        # Get user input for the rectangular region (left, upper, right, lower)
        # left, upper, right, lower = map(int, input("Enter the region (left, upper, right, lower): ").split())
        #
        # print(file"The entered region is: {left=}, {upper=}, {right=}, {lower=}")
        #
        # # Crop the image
        # cropped_img = img.crop((left, upper, right, lower))
        # # Save the cropped image
        # cropped_img.save(SCREENSHOT_CROPPED, SM_FILES)
        # print("Cropped image saved as " + SCREENSHOT_CROPPED)

    def screenshot_do_new(self):
        screenshot = pyautogui.screenshot()
        screenshot.save(SCREENSHOT_DEFAULT)
        screenshot_path = SCREENSHOT_DEFAULT
        img = Image.open(screenshot_path)

        # Display the image
        fig, ax = plt.subplots()
        ax.imshow(img)
        ax.set_title("Select a rectangular region")
        ax.axis("on")  # Turn off axis ticks

        # Create the rectangle selector
        # rect_selector = RectangleSelector(ax, lambda eclick, erelease: onselect(eclick, erelease, ax), drawtype='box',
        #                                  useblit=True, button=[1], minspanx=5, minspany=5, spancoords='pixels')

        rect_selector = RectangleSelector(ax, self.on_select, useblit=True, interactive=True, button=[1], minspanx=5,
                                          minspany=5, spancoords='pixels')
        plt.show()

    def text_to_be_typed_create(self, file_name, text_to_be_typed):
        file_name += ".txt"
        file_path = os.path.join(SM_FILES_TEXTS, file_name)

        # Delete the existing file if it exists
        if os.path.exists(file_path):
            FLOS.file_remove(file_path)
            self.print(f"Deleted existing file '{file_name}'")

        # Create a new file
        with open(file_path, "w") as f:
            f.write(text_to_be_typed)
            self.print(f"File '{file_name}' created in directory '{SM_FILES_TEXTS}' with content: '{text_to_be_typed}'")
    def window_main_set(self, window_main):
        self.window_main = window_main

def button_escape_thread(stop_flag, self):
    # If AutoPilot button is not disengaged manually
    while not stop_flag.is_set():
        try:
            time.sleep(0.1)
            if keyboard.is_pressed('esc'):
                print("Escape key pressed: autopilot is now disengaged")
                self.autopilot_is_on_set(False)
                self.GUI_has_to_be_updated = True
        except Exception as e:
            print(f"Error: {e}")
            self.autopilot_is_on_set(False)
    print("escape_button_thread is stopping...")

def region_get():
    # Get user input for the rectangular region (left, upper, right, lower)
    left = int(left_var.get())
    upper = int(upper_var.get())
    right = int(right_var.get())
    lower = int(lower_var.get())

    # Close the dialog box
    root.destroy()

    # Print the entered region
    print(f"The entered region is: {left=}, {upper=}, {right=}, {lower=}")

def screenshot_do_Artec():
    # Get the window title or program ID (modify this based on your app)
    window_title = "Artec Studio 16 Professional"
    # Find the window by title
    my_window = pygetwindow.getWindowsWithTitle(window_title)

    # Activate the window (bring it to the foreground)
    if len(my_window) != 0:
        my_window = pygetwindow.getWindowsWithTitle(window_title)[0]
        my_window.activate() # pysimplegui ver5

    # Capture a screenshot of the specified window
    screenshot = pyautogui.screenshot()
    # Now you have the screenshot in the 'screenshot' variable

def window_ok_cancel(text):
    layout = [
        [sg.Text(text)],
        [sg.Button("OK"), sg.Button("Cancel")]
    ]
    x, y = screen_coordinates_for_popup()
    window_ok = sg.Window("OK/Cancel Window", layout, modal=True, finalize=True, location=(x, y))

    while True:
        event, _ = window_ok.read()

        if event in (sg.WINDOW_CLOSED, "Cancel"):
            result = "Cancel"
            break
        elif event == "OK":
            result = "OK"
            break

    window_ok.close()
    return result

def z_listbox(self):
    states = ['A', 'B', 'C']

    layout = [[sg.Text('Select a state:')],
              [sg.Listbox(states, size=(15, len(states)), key='-STATE-')],
              [sg.Button('Ok')]]

    window = sg.Window('Window Title', layout)

    while True:
        event, values = window.read()
        if event == sg.WIN_CLOSED:
            break
        if event == 'Ok':
            selected_state = values['-STATE-'][0]
            sg.popup(f'You selected {selected_state}!')

    window.close()

if __name__ == '__main__':
    screenshot_do_Artec()