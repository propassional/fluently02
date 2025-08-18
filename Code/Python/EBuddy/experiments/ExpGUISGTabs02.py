# Similar to version01, but this one performs auto tab switch every 3 seconds

import PySimpleGUI as sg
import io
from PIL import Image
import time

def load_image(file_path):
    with open(file_path, "rb") as image_file:
        image = Image.open(image_file)
        # Convert the image to a byte array
        bio = io.BytesIO()
        image.save(bio, format="PNG")
        image_bytes = bio.getvalue()
        width, height = image.size
        height = height * 1.4  # Keep some room vertically
        return image_bytes, width, height

if __name__ == '__main__':
    path_base = r"D:\Banfi\Github\Fluently\Releases\2024_07_09_EBuddy\Data\Output\Screenshot\_old01"
    file_paths = [path_base + r"\SMGraphDefault_ArtecStudio__Preview__2024_08_05__14_57_32.png",
                  path_base + r"\SMGraphDefault_ArtecStudio__Scan__2024_08_05__14_57_32.png",
                  path_base + r"\SMGraphDefault_BiState.png"]
    tab_names = ["ArtecStudio Interactive", "Cobot Interactive", "Processing Auto", "Processing Interactive", "Repairing Auto"]

    # Sort tabs alphabetically
    sorted_tabs = sorted(zip(tab_names, file_paths), key=lambda x: x[0])
    tab_names, file_paths = zip(*sorted_tabs)

    tabgroup = []

    for i in range(0, len(file_paths)):
        image_bytes, width, height = load_image(file_paths[i])
        tab_layout = [
            [sg.Column([[sg.Image(data=image_bytes, key='-IMAGE-')]], scrollable=True, vertical_scroll_only=False,
                       size=(width, height))],
        ]
        tab = sg.Tab(f"{tab_names[i]}", tab_layout, key=f"-{tab_names[i]}-")
        tabgroup.append(tab)

    layout_main = [
        [sg.TabGroup([tabgroup], key="TabGroup")]
    ]

    window_main = sg.Window("Main Window", layout_main, return_keyboard_events=True)

    while True:
        event, values = window_main.read(timeout=100)
        if event == sg.WIN_CLOSED:
            break

        for i in range(len(tab_names)):
            window_main['TabGroup'].Widget.select(i)
            window_main.refresh()
            time.sleep(3)

    window_main.close()