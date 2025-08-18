# The commands are files that are sent from the user or a SM to another SM (or itself) for obtaining a specific behavior
# File contents structure: SM_name\nCommand\nAutopilotMode, where the third parameter is true or false

import shutil
import threading
import os
import PySimpleGUI as sg

import FLOS
from FLBaseClass import FLBaseClass
from FLConstants import COMMAND_FOR_EBUDDY, UI_LOGORRHOIC, ERROR, AUTO_START_TRANSITION_NAME, COMMAND_FILE_NR_OF_ARGS, \
    COMMAND_HISTORY_SAVE, COMMAND_FOR_EBUDDY_HISTORY, SCROLL_IS_ACTIVE
from FLOS import screen_coordinates_for_popup
from FLQueue import FLQueue

class FLCommands(FLBaseClass):
    def __init__(self, logger):
        super().__init__(logger)
        self.queue = FLQueue(logger)

    def command_file_read(self):
        try:
            command_files = [file for file in os.listdir(COMMAND_FOR_EBUDDY) if file.endswith('.txt')]
            if not command_files:
                if (UI_LOGORRHOIC):
                    self.print(f"No .txt files found in the directory: {COMMAND_FOR_EBUDDY}")
                return ""

            # Read the command from the content of the file (not the name)
            for file_name in command_files:
                self.command_file_path = os.path.join(COMMAND_FOR_EBUDDY, file_name)
                with open(self.command_file_path, 'r') as command_file:
                    content = command_file.read()
                    contentFiltered = content.replace("[whisper]", "")
                    if UI_LOGORRHOIC:
                        self.print(f"\nFile: {file_name}\nContents:\n{contentFiltered}")
                    return contentFiltered
        except FileNotFoundError:
            self.print(f"Error: Directory '{COMMAND_FOR_EBUDDY}' not found.", ERROR)
        except IOError as e:
            self.print(f"Error reading files in directory '{COMMAND_FOR_EBUDDY}': {e}", ERROR)
        except Exception as e:
            self.print(f"An unexpected error occurred: {e}", ERROR)
        if command_file:
            command_file.close()

    # Command file read / execute / delete is performed globally
    def command_file_archive(self, attribute):
        # Remove the file
        try:
            if hasattr(self, 'command_file_path'):
                if COMMAND_HISTORY_SAVE:
                    file_name = os.path.basename(self.command_file_path)
                    file_name = FLOS.datetime_now_get_string() + "_" + file_name
                    destination = os.path.join(COMMAND_FOR_EBUDDY_HISTORY, file_name)

                    # Split the file path into directory and file name
                    dir_path, file_name = destination.rsplit('\\', 1)
                    # Split the file name into name and extension
                    name, ext = file_name.rsplit('.', 1)
                    # Insert '_readme' into the file name
                    new_file_name = f"{name}{attribute}.{ext}"
                    # Combine the directory path and new file name
                    new_file_path = f"{dir_path}\\{new_file_name}"

                    shutil.move(self.command_file_path, new_file_path)
                    self.print(f"File archived to {new_file_path}")
                else:
                    FLOS.file_remove(self.command_file_path)
                    self.print(f"File deleted/removed {self.command_file_path}")

        except FileNotFoundError:
            self.print(f"File {self.command_file_path} not found.")
        except Exception as e:
            self.print(f"Error removing file: {e}")

    def command_str_to_bool(self, value):
        if isinstance(value, str):
            if value.lower() == 'true':
                return True
            elif value.lower() == 'false':
                return False
        return None  # or raise an exception if you prefer

    def commands_read_and_insert_into_queue(self, SM_names_all_ptr, init_SM_done):
        # Init all SM's, then read the command files, if there are some, or do autopilot_is_on_from_GUI
        if init_SM_done:
            # Read command files #nnn
            cmd_content = self.command_file_read()
            # Check if file exists
            if cmd_content != "":
                cmd_contents = cmd_content.split(r"\n")
                cmd_contents_len = len(cmd_contents)
                if cmd_contents_len >= COMMAND_FILE_NR_OF_ARGS-1 and cmd_contents_len <= COMMAND_FILE_NR_OF_ARGS:
                    SM_name = cmd_contents[0]
                    command = cmd_contents[1]
                    # Check if the command file also contains the third argument
                    if cmd_contents_len == COMMAND_FILE_NR_OF_ARGS:
                        autopilot_is_on = self.command_str_to_bool(cmd_contents[2])
                    else:
                        # If command file does not define the autopilot mode
                        autopilot_is_on = None

                    # Insert the command into the command queue
                    self.queue.element_insert(SM_name, command, autopilot_is_on)
                    self.print(f"commands_read: command appended to the queue: {SM_name}, {command}, {autopilot_is_on}")
                    self.command_file_archive("_queued")
                else:
                    sg.popup("main_GUI_SM_commands_execute: wrong command format")
                    self.print(
                        f"main_GUI_SM_commands_execute: command {cmd_content} can not be executed and will be deleted")
                    self.command_file_archive("_bad_format")
            return True # init_SM_done stays True as at the beginning of this method
            # elif self.autopilot_is_on_from_GUI:
            #     cmd_to_be_executed_loc = self.GM_GUI_dictionary[SM_to_be_executed_loc].sm_gui.hint
        else:
            # Do SM initialisation, inserting the transition "AutoStart" into the queue
            if len(SM_names_all_ptr) > 0:


                #xxx nnn
                # Coherence between initialisation and tab displaying: I want initialisation to start from last tab, so at the end the first tab is initialised and displayed

                #SM_names_all_ptr_reversed = SM_names_all_ptr[::-1]
                #self.queue.element_insert(SM_names_all_ptr_reversed[0], AUTO_START_TRANSITION_NAME, False)

                self.queue.element_insert(SM_names_all_ptr[0], AUTO_START_TRANSITION_NAME, False)
                #self.queue_append_command(SM_names_all_ptr[0], "AutoStart", False)
                # Remove last processed SM from the SM list, so that at the end all SM's will be initialised, one after the other, in sequential loops
                SM_names_all_ptr.pop(0)



                return False # init_SM_done is False
            else:
                return True # init_SM_done is True
    # Read all command files or do SM init, delete command file

    def GUI_command_send(self, sm_gui_name, commands_available, tab_names_list):
        global SCROLL_IS_ACTIVE
        all_commands_available = commands_available + tab_names_list
        all_commands_available.append("SCROLL_IS_ACTIVE")
        window_title = "Command Sender"
        layout = [
            #[sg.Button(file_content, key=file_content)] for file_content in file_contents_set_AtoZ  # Works
            [sg.Button(file_content, key=file_content) for file_content in all_commands_available],
            [sg.Text("", key='-FILE_SAVED-')]
        ]
        x, y = screen_coordinates_for_popup()
        window_command = sg.Window(window_title, layout, resizable=True, location=(x, y))

        while True:
            file_written = False
            event, values = window_command.read()

            if event == sg.WINDOW_CLOSED:
                break
            elif event == "SCROLL_IS_ACTIVE":
                SCROLL_IS_ACTIVE = not SCROLL_IS_ACTIVE
                break
            # Chosen command is a tab name?
            elif event not in tab_names_list:
                # Command is a command of the current sm
                sm_name = sm_gui_name
            else:
                # Command is a tab switch
                sm_name = event

            # xxx This file seems to be deleted by EBuddy, at the moment I don't understand how this happens
            path_sink = os.path.join(COMMAND_FOR_EBUDDY, sm_name + "_" + event + ".txt")
            try:
                with open(path_sink, "w") as sink_file:
                    sink_file.write(sm_name + r"\n" + event)
                    # This will be done automatically by exiting the with statement: sink_file.close()
                    window_command['-FILE_SAVED-'].update(path_sink)
                    file_written = True
            except FileNotFoundError:
                sg.popup(f"GUI_command_send: file '{event}' not found in source directory.", title="Error")
            except Exception as e:
                sg.popup(f"GUI_command_send: an error occurred while copying '{event}': {str(e)}", title="Error")

            # Quite strange, I have to write the file twice
            path_sink = os.path.join(COMMAND_FOR_EBUDDY, sm_name + "_" + event + ".txt")
            try:
                with open(path_sink, "w") as sink_file:
                    sink_file.write(sm_name + r"\n" + event)
                    # This will be done automatically by exiting the with statement: sink_file.close()
                    window_command['-FILE_SAVED-'].update(path_sink)
                    file_written = True
            except FileNotFoundError:
                sg.popup(f"GUI_command_send: file '{event}' not found in source directory.", title="Error")
            except Exception as e:
                sg.popup(f"GUI_command_send: an error occurred while copying '{event}': {str(e)}", title="Error")

            if file_written:
                break
            else:
                sg.popup(f"GUI_command_send: a command name has no correct command file associated with it", title="Error")
                break
        # Close the window
        window_command.close()

if __name__ == '__main__':
    command_send_GUI = FLCommands()
    command_send_GUI_thread = threading.Thread(target=command_send_GUI.GUI_command_send)
    command_send_GUI_thread.start()

    pass
