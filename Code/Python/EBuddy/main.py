# See EBuddy docs in _main.txt

# Kill all threads of this app: kill all python apps
# Relevant methods: search them with ctrl-shift-f
# GUI_main_event_loop
# GUI_main_SM_commands_execute

import os
# OMP: Error #15: Initializing libiomp5md.dll, but found libomp140.x86_64.dll already initialized.
# OMP: Hint This means that multiple copies of the OpenMP runtime have been linked into the program. That is dangerous, since it can degrade performance or cause incorrect results. The best thing to do is to ensure that only a single OpenMP runtime is linked into the process, e.g. by avoiding static linking of the OpenMP runtime in any library. As an unsafe, unsupported, undocumented workaround you can set the environment variable KMP_DUPLICATE_LIB_OK=TRUE to allow the program to continue to execute, but that may cause crashes or silently produce incorrect results. For more information, please see http://www.intel.com/software/products/support/.
# Potentially unsafe
os.environ['KMP_DUPLICATE_LIB_OK'] = 'TRUE'

# Legenda:
# Legenda of the labels you find within the code:
# "!!" Strange things happen here
# "xx" to be done

# Relevant methods:
# SM_update: reads the commands, then updates the state machine, then calls GUI_main_update
# GUI_main_loop: handles main GUI
# GUI_main_update: saves the SM image, loads it into the GUI
# transitions: transition_condition_set changes self.machine.model.transition_semaphores
# graph_save_to_file_default() saves the graph machine .png
# states_and_transitions_add
# FLModel: after_state_change='on_enter_each_state': state tasks

import threading
import sys

# https://docs.pysimplegui.com/en/latest/ seems that PySimpleGUI needs to be disinstalled, and reinstalled from a new server, in reality this version is over
# I downloaded the dirs from https://github.com/markreading/PySimpleGUI_4_60_5 and installed in C:\Users\operator\.conda\envs\ml_env\Lib\site-packages
import PySimpleGUI as sg

from fluently.FLConstants import LINK_TO_COMMAND_IMAGES, SM_COMMAND_IMAGES, YAML_MAIN_PATH, YAML_MAIN_PATH
from fluently.FLGUIMain import FLGUIMain
from fluently.FLGraphMachine import FLGraphMachine

import fluently.FLOS
from fluently.FLGUISM import FLGUISM
from fluently.FLLogger import FLLogger


sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))


if __name__ == '__main__':
    # Do not delete me
    # # Extract JSON from the hard coded SM of GA
    # gm_from_code = FLGraphMachine("GA")  # Create gm == Graph Machine
    # gm_from_code.states_and_transitions_add()  # Load hardcoded states and transitions
    # gm_from_code_GUI = FLGUIMain(gm_from_code, '')  # Create main GUI
    # SM_save_to_json(gm_from_code.machine, r"D:\Banfi\Github\Fluently\Releases\2024_04_09_Teachix\Data\Input\JSON\ArtecStudio_2024_05_01__11_21_48.json")

    # Create the main GUI object, the container of all SM
    main_log_file = FLLogger(r"MainGUI")
    main_GUI = FLGUIMain(main_log_file)

    # Setup environment, according to the current code release
    FLOS.shortcut_update(LINK_TO_COMMAND_IMAGES, SM_COMMAND_IMAGES, "Link to command images")
    FLOS.commands_delete(main_GUI)
    FLOS.screenshots_delete(main_GUI)

    loaded_data = FLOS.yaml_load()

    # Now `loaded_data` contains the list of dictionaries with all SM paths and names
    no_name_index = 0
    for entry in loaded_data:
        print(f"SM path: {entry['SM_path']}, SM name: {entry['SM_name']}")
        # Create an empty Graph Machine
        if entry['SM_name'] == "":
            entry['SM_name'] = "SMWithNoName" + str(no_name_index)
        log_file = FLLogger(entry['SM_name'])
        gm_GUI = FLGUISM(entry['SM_name'], log_file)
        # The SM of this default GM will be overwritten later, on loading the SM from json
        gm = FLGraphMachine(entry['SM_name'], "", log_file)
        gm_GUI.SM_connect(gm)
        if not os.path.exists(entry['SM_path']):
            sg.popup(f"State Machine \"{ entry['SM_name'] }\" has a wrong path, SM could not be loaded, please change it in {YAML_MAIN_PATH}), \nApp will be closed now")
            sys.exit(0) # Close main app
        else:
            # Load SM machine from JSON
            #main_GUI.GM_GUI_dictionary[entry['SM_name']].button_load_SM_from_file(entry['SM_path'], entry['path_code'])
            gm_GUI.button_load_SM_from_file(entry['SM_path'], "")
            pass
        if entry['autopilot']:
            gm_GUI.autopilot_is_on = True
        if entry['app_name']:
            gm_GUI.sm_gui.machine.model.app_name = entry['app_name']
        main_GUI.GM_GUI_dictionary[entry['SM_name']] = gm_GUI
        # Create the SM GUI and load the SM contents from json
        main_GUI.GM_GUI_dictionary[entry['SM_name']].GUI_SM_create()

    # Check if Artec Studio is open, and ask user to kill it
    # if FLOS.artec_studio_is_open():
    #     message_to_user = "An opened instance of Artec Studio has been found, Teachix needs to kill it\n" + "Please save and close it, otherwise SM_JSON_data loss is possible\n\nKill Artec Studio (Ok) or Close Teachix (Cancel)?"
    #     res = main_GUI.GUI_popup_ok(message_to_user)
    #     #res = window_ok_cancel(message_to_user)
    #     if res == "Cancel":
    #         sys.exit(0) # Close main app
    #     else:
    #         FLOS.artec_studio_kill()

    # Following threads run concurrently in separate threads using the same object: this should be ok (!!)

    # Launch an infinite parallel thread for listening to voice commands and updating the SM GUI
    gm_sm_update_thread = threading.Thread(target=main_GUI.GUI_main_SM_commands_execute)
    gm_sm_update_thread.start()

    # I am forced to start GUI into the main loop,
    # since starting the neverending window GUI loop in a separate thread returns “RuntimeError:
    # main thread is not in main loop,”
    # when another pysimplegui window is previously opened and closed in the main loop
    main_GUI.GUI_main_event_loop()