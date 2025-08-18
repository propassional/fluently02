import PySimpleGUI as sg
import subprocess

import FLConstants
from FLConstants import COMMAND_FOR_EBUDDY, COMMAND_FOR_EBUDDY_REPOSITORY, SCROLL_IS_ACTIVE

class FLCockpit:
    def __init__(self, gui_main):
        self.gui_main = gui_main
        pass

    def show(self, transition_conditions, gui_main_loop_is_active, logger):

        # Get a list of filenames from the source directory
        #transition_conditions = self.gui_main.sm_gui.machine.model.transition_semaphores
        #separator = sg.HorizontalSeparator()

        # Why does this code does not work, if copy pasted into the layout?!!
        transition_values = [sg.Text(f"{key}: {value}") for key, value in transition_conditions.items()]

        layout = [
            [sg.Text('Transitions:')],
            transition_values,
            [sg.HorizontalSeparator(color='red')],
            [sg.Text('Command read thread is active:'), sg.Text('', key='-MAIN_LOOP_IS_ACTIVE-')],
            [sg.Button("Open Command Explorers", key="-OPEN_EXPLORERS-")],
            [sg.Button("Open Log File", key="-OPEN_LOG_FILE-")],
            [sg.Button("Toggle Scroll Mode", key="-TOGGLE_SCROLL_MODE-")]
        ]

        window_cockpit = sg.Window('Cockpit Viewer', layout, finalize=True)

        # Event loop to handle button presses
        while True:
            window_cockpit['-MAIN_LOOP_IS_ACTIVE-'].update(gui_main_loop_is_active)
            event, values = window_cockpit.read()

            if event == sg.WINDOW_CLOSED:
                break
            elif event == "-OPEN_EXPLORERS-":
                path = COMMAND_FOR_EBUDDY
                subprocess.Popen(fr'explorer /select,{path}') # xx Why does it open the upper dir?
                path = COMMAND_FOR_EBUDDY_REPOSITORY
                subprocess.Popen(fr'explorer /select,{path}')
            elif event == "-OPEN_LOG_FILE-":
                #subprocess.Popen([r'start\notepad++', log_file_path]) # It does not work
                #subprocess.Popen(["notepad++", log_file_path]) # It does not work
                # subprocess.Popen([r"C:\Program Files\Notepad++\notepad++.exe\notepad++", log_file_path]) # It does not work
                subprocess.Popen(["notepad", logger.log_file_path])
            elif event == "-TOGGLE_SCROLL_MODE-":
                FLConstants.SCROLL_IS_ACTIVE = not FLConstants.SCROLL_IS_ACTIVE
                pass


        # Close the window
        window_cockpit.close()

if __name__ == '__main__':
    my_dict = {
        'Name': 'John Doe',
        'Age': 30,
        'Email': 'john.doe@example.com',
        'Country': 'USA',
        # ... other fields ...
    }
    cockpit = FLCockpit(my_dict)
    cockpit.show(my_dict, True)
    pass
