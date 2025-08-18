import PySimpleGUI as sg
from PIL import Image, ImageTk
import io

class ImageViewerScrollable:
    def __init__(self):
        # Load an image file
        with open("SMFluentlyGACompleteMacro.png", "rb") as image_file:
            image = Image.open(image_file)
            # Convert the image to a byte array
            bio = io.BytesIO()
            image.save(bio, format="PNG")
            image_bytes = bio.getvalue()

        # Create the GUI layout
        layout = [
            [sg.Column([[sg.Image(data=image_bytes, key="-IMAGE-")]], scrollable=True, vertical_scroll_only=False, size=(400, 400))],
        ]

        # Create the window
        window = sg.Window("Scrollable Image Viewer", layout)

        # Event loop
        while True:
            event, values = window.read()
            if event == sg.WINDOW_CLOSED:
                break

        window.close()

if __name__ == '__main__':
    GUI = ImageViewerScrollable()
