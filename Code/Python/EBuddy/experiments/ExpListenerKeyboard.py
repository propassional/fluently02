import PySimpleGUI as sg
import threading
from pynput import keyboard

# Define the layout for the two tabs
tab1_layout = [[sg.Text('This is Tab 1')]]
tab2_layout = [[sg.Text('This is Tab 2')]]

# Define the layout for the window
layout = [[sg.TabGroup([[sg.Tab('Tab 1', tab1_layout, key='-TAB1-'), sg.Tab('Tab 2', tab2_layout, key='-TAB2-')]], key='-TABGROUP-')]]

# Create the window
window = sg.Window('Tab Switcher', layout)

# Function to handle key presses
def arrows_on_press_left_and_right(key):
    try:
        if key == keyboard.Key.left:
            window['-TABGROUP-'].Widget.select(0)
        elif key == keyboard.Key.right:
            window['-TABGROUP-'].Widget.select(1)
    except AttributeError:
        pass

# Function to start the keyboard listener in a separate thread
def arrows_listener_start():
    with keyboard.Listener(on_press=arrows_on_press_left_and_right) as listener:
        listener.join()

# Start the keyboard listener thread
listener_thread = threading.Thread(target=arrows_listener_start, daemon=True)
listener_thread.start()

# Event loop to keep the window open
while True:
    event, values = window.read()
    if event == sg.WIN_CLOSED:
        break

# Close the window
window.close()
