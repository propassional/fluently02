# https://docs.pysimplegui.com/en/latest/documentation/module/dynamic_layouts/
# Once you have used a layout to create a Window, you cannot change that layout directly.
# You also cannot use it in another Window. Once "used" a layout, the list of lists variable, can't be tampered with

import PySimpleGUI as sg
import threading
import time

# Function to add a button to the first tab
def add_button(window):
    time.sleep(2)  # Simulate some delay
    window.write_event_value('-ADD_BUTTON-', 'Add Button')

# Define the layout of the tabs
tab1_layout = [[]]
tab2_layout = [[]]

# Define the layout of the window
layout = [
    [sg.TabGroup([[sg.Tab('Tab 1', tab1_layout, key='-TAB1-'), sg.Tab('Tab 2', tab2_layout, key='-TAB2-')]], key='-TABGROUP-')]
]

# Create the window
window = sg.Window('Two Tabs Example', layout)

# Start the thread to add a button to the first tab
threading.Thread(target=add_button, args=(window,), daemon=True).start()

# Event loop to process events and get values of inputs
while True:
    event, values = window.read()
    if event == sg.WIN_CLOSED:
        break
    elif event == '-ADD_BUTTON-':
        # This works only if window has not been created, afterwards as here, it does not work
        # window_layout.append([sg.Button('Show Image', key=file'-BUTTON-')])
        # In this case, only dynamic layout helps
        window.extend_layout(window['-TAB1-'], [[sg.Button('New Button')]])

window.close()
