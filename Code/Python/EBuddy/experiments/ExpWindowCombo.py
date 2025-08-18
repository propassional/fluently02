import os

import PySimpleGUI as sg

from FLConstants import SM_FILES

# Problem0: select Option2, then select Update: this leads to total crash "Process finished with exit code -1073741819 (0xC0000005)"!!
# Problem1: ['NewOption1', 'NewOption2', 'NewOption3'] are shown on a single row, and not in 3 rows as expected
# Problem2: The new options do not override the old ones
# Problem3: No new option can be selected
def window_combo0():
    combo_values = ['Opzione 1', 'Opzione 2', 'Opzione 3']
    layout = [
        [sg.Text('Seleziona un opzione dal menu a tendina:')],
        #[sg.Combo(['Opzione 1', 'Opzione 2', 'Opzione 3'], key='-DROPDOWN-', enable_events=True)],
        [sg.Combo(combo_values, key='-DROPDOWN-', enable_events=True)],
        [sg.Text('Opzione selezionata:'), sg.Text('', key='-OUTPUT-')],
        [sg.Button('Esci'), sg.Button("UpdateGUI"),]
    ]

    window = sg.Window('Menu a Tendina con PySimpleGUI', layout, finalize=True)

    while True:
        event, values = window.read()

        if event == sg.WINDOW_CLOSED or event == 'Esci':
            break
        elif event == '-DROPDOWN-':
            selected_option = values['-DROPDOWN-']
            window['-OUTPUT-'].update(selected_option)
        elif event == "UpdateGUI":
            # Try to reset the combo values
            combo_values = []
            window['-DROPDOWN-'].update(combo_values)

            combo_values = ['NewOption1', 'NewOption2', 'NewOption3']
            window['-DROPDOWN-'].update(combo_values)

    window.close()

def window_combo1():

    # Define the layout of the window
    combo_values = ["Option 1", "Option 2", "Option 3"]
    layout = [[sg.Text("Select an option:"), sg.Combo(combo_values, key="-COMBO-")], [sg.Button("Update Combo"), sg.Button("Exit")]]

    # Create the window
    window = sg.Window("Window Title", layout)

    # Event loop
    while True:
        event, values = window.read()
        if event == "Exit" or event == sg.WIN_CLOSED:
            break
        elif event == "Update Combo":
            # Replace all previous options with a new option

            #new_option = "New Option"
            #window["-COMBO-"].update(values=[new_option])

            # combo_values_new = ["Option 1", "Option 2", "Option 3"]
            # window["-COMBO-"].update(values=[combo_values_new])

            # Non va
            #combo_values.append("Option 4")
            #window["-COMBO-"].update(values=[combo_values])

            combo_values_new = ['New Option 1', 'New Option 2', 'New Option 3']
            window["-COMBO-"].update(values=combo_values_new)

    # Close the window
    window.close()

def window_combo2():
    combo_values = ["Option 1", "Option 2", "Option 3"]
    layout = [[sg.Combo(values=combo_values, key='-COMBO-')],
              [sg.Button('Submit'), sg.Button('Update')]]

    window = sg.Window('Combo Example', layout)

    while True:
        event, values = window.read()
        if event == sg.WINDOW_CLOSED:
            break
        elif event == 'Submit':
            print(f"You selected {values['-COMBO-']}")
        elif event == 'Update':
            combo_values_new = ['New Option 1', 'New Option 2', 'New Option 3']
            window['-COMBO-'].update(values=combo_values_new)
            # This works
            # combo_values_new = ['New Option 1', 'New Option 2', 'New Option 3']
            # window['-COMBO-'].update(values=combo_values_new)
            # This works
            #window['-COMBO-'].update(values=['New Option 1', 'New Option 2', 'New Option 3'])

    window.close()

# This variant works!
def window_combo_works():

    layout = [[sg.Combo(['Option 1', 'Option 2', 'Option 3'], key='-COMBO-')],
              [sg.Button('Submit'), sg.Button('Update')]]

    window = sg.Window('Combo Example', layout)

    while True:
        event, values = window.read()
        if event == sg.WINDOW_CLOSED:
            break
        elif event == 'Submit':
            print(f"You selected {values['-COMBO-']}")
        elif event == 'Update':
            # This works
            combo_values_new = ['New Option 1', 'New Option 2', 'New Option 3']
            window['-COMBO-'].update(values=combo_values_new)
            # This works
            #window['-COMBO-'].update(values=['New Option 1', 'New Option 2', 'New Option 3'])

    window.close()

def window_file_browser1():
    # Specify your default path
    default_path = SM_FILES

    # Layout
    layout = [[sg.Text("Please select a file:")],
              [sg.In(default_path)],
              [sg.FileBrowse(initial_folder=default_path), sg.Submit(), sg.Cancel()]]

    # Create the window
    window = sg.Window('File Browser', layout)

    # Event loop
    while True:
        event, values = window.read()
        if event in (None, 'Cancel'):
            break
        if event == 'Submit':
            print(f"You selected {values[0]}")

    window.close()

def save_file1(file_path):
    print(f"Processing file: {file_path}")

    layout = [
        [sg.Text("Please select a directory:")],
        [sg.In(size=(25, 1), enable_events=True, key="-FOLDER-"), sg.FolderBrowse()],
        [sg.Text("Please enter a file name:")],
        [sg.In(size=(25, 1), key="-FILE-")],
        [sg.Button("Submit"), sg.Button("Cancel")]
    ]

    window = sg.Window('File Browser', layout)

    while True:
        event, values = window.read()
        if event == sg.WINDOW_CLOSED or event == "Cancel":
            break
        if event == "Submit":
            folder_path = values["-FOLDER-"]
            file_name = values["-FILE-"]
            file_path = os.path.join(folder_path, file_name)

    window.close()

def save_file2(path=r'D:\\', file_extension=""):
        current_datetime = datetime.now()
        formatted_timestamp = current_datetime.strftime("%Y_%m_%d__%H_%M_%S")
        file_name = "SM_" + formatted_timestamp + file_extension

        file_types = [("", file_extension)]

        layout = [[sg.Text("Please select a directory:")],
                  [sg.In(path, key='dir_path'), sg.FolderBrowse(initial_folder=path, target='dir_path')],
                  [sg.Text("Please enter a file name:")],
                  [sg.In(file_name, key='file_name')],
                  [sg.Submit(), sg.Cancel()]]

        window = sg.Window('File Browser', layout)

        # Event loop
        while True:
            event, values = window.read()
            if event in (None, 'Cancel'):
                break
            if event == 'Submit':
                file_path = os.path.join(values['dir_path'], values['file_name'])
                print(f"You selected {file_path}")
                window.close()
                return file_path
        window.close()

from datetime import datetime

def get_formatted_timestamp():
    # Get the current date and time
    current_datetime = datetime.now()

    # Format the date and time as year_month_day__hour_minute_second
    formatted_timestamp = current_datetime.strftime("%Y_%m_%d__%H_%M_%S")

    return formatted_timestamp

def button_grey_out():
    layout = [
        [sg.Button('Click me', key='-BUTTON-', initial_state=sg.tk.DISABLED)],
        [sg.Button('Exit')]
    ]

    window = sg.Window('Grey Out Button Example', layout, finalize=True)

    # Event loop
    while True:
        event, values = window.read()

        if event == sg.WINDOW_CLOSED or event == 'Exit':
            break
        elif event == '-BUTTON-':
            # Grey out the button
            button_element = window['-BUTTON-']
            button_element.Widget.configure(state=sg.tk.DISABLED)

    window.close()

def button_grey_out2():
    layout = [
        [sg.Button('Click me', key='-BUTTON-')],
        [sg.Button('Enable', key='-ENABLE-'), sg.Button('Exit')]
    ]

    window = sg.Window('Enable/Disable Button Example', layout, finalize=True)

    # Set the initial state of the button to disabled
    button_element = window['-BUTTON-']
    button_element.Widget.configure(state=sg.tk.DISABLED)

    # Event loop
    while True:
        event, values = window.read()

        if event == sg.WINDOW_CLOSED or event == 'Exit':
            break
        elif event == '-BUTTON-':
            # Do something when 'Click me' button is clicked
            pass
        elif event == '-ENABLE-':
            # Enable the button
            button_element.Widget.configure(state=sg.tk.NORMAL)

    window.close()