import PySimpleGUI as sg
from PIL import Image, ImageTk
import os

class HelloWorld:
    def __init__(self):
        self.layout = [[sg.Text('Hello World')]]
        self.window = sg.Window('Hello World Window', self.layout, finalize=True)

    def show(self):
        while True:
            event, values = self.window.read()
            if event == sg.WIN_CLOSED:
                break
        self.window.close()

def open_image(file_path, window):
    if file_path:
        image = Image.open(file_path)
        photo = ImageTk.PhotoImage(image)
        window['-IMAGE-'].update(data=photo)

def get_image_files(directory):
    supported_extensions = ('.jpg', '.jpeg', '.png', '.gif', '.bmp')
    return [f for f in os.listdir(directory) if f.lower().endswith(supported_extensions)]

def add_buttons(window, image_directory):
    image_files = get_image_files(image_directory)
    if not image_files:
        sg.popup('No images found in the directory')
        return

    window.extend_layout(window, [
        [sg.Button('Show First Image')],
        [sg.Image(key='-IMAGE-')]
    ])

    while True:
        event, values = window.read(timeout=100)
        if event == sg.WIN_CLOSED:
            break
        elif event == 'Show First Image':
            open_image(os.path.join(image_directory, image_files[0]), window)

# Create an instance of the HelloWorld class and show the initial window
hello_world = HelloWorld()

# Specify the image directory
image_directory = r'D:\Banfi\Github\Fluently\Releases\2024_07_09_EBuddy\Data\Input\App\ArtecStudio\Hints'

# Call the external method to add buttons and display images
add_buttons(hello_world.window, image_directory)

# Show the window
hello_world.show()
