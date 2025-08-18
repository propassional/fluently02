import PySimpleGUI as sg
import random
import string
import threading
import time

# Generate random text of 500 characters
random_text = ''.join(random.choices(string.ascii_letters + string.digits + string.punctuation + ' ', k=500))

# Define the layout
layout = [
    [sg.Multiline(random_text, size=(100, 10), horizontal_scroll=True, key='-TEXT-', disabled=True)]
]

# Create the window
window = sg.Window('Random Text Display', layout, resizable=True, finalize=True)

def scroll_text():
    text_element = window['-TEXT-']
    while True:
        for i in range(len(random_text)):
            text_element.Widget.xview_moveto(i / len(random_text))
            time.sleep(0.1)  # Adjust the speed as needed
            if not window.read(timeout=0)[0]:
                return

# Start the scrolling thread
thread = threading.Thread(target=scroll_text, daemon=True)
thread.start()

# Event loop
while True:
    event, values = window.read()
    if event == sg.WINDOW_CLOSED:
        break

window.close()
