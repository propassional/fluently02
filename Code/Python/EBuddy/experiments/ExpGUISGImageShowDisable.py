import PySimpleGUI as sg

# Function to update the column based on image presence
def update_image_column(window, image_path, tab_name, show_image, scrollable_states):
    scrollable_states[tab_name] = show_image
    if show_image:
        window[f'-{tab_name}_IMAGE_HELP-'].update(filename=image_path, visible=True)
        window[f'-{tab_name}_COLUMN-'].update(visible=True)
    else:
        window[f'-{tab_name}_IMAGE_HELP-'].update(visible=False)
        window[f'-{tab_name}_COLUMN-'].update(visible=False)

# Example layout
tab_name = 'TAB1'
image_help_width, image_help_height = 400, 300  # Adjust size as needed
layout = [
    [sg.Column([[sg.Image(key=f'-{tab_name}_IMAGE_HELP-')]], key=f'-{tab_name}_COLUMN-', scrollable=True, size=(image_help_width, image_help_height))],
    [sg.Button('Toggle Image', key='-TOGGLE-')] # With scrollable=False the scrollbars never appear
]

window = sg.Window('Image Help Example', layout, finalize=True)

# Dictionary to keep track of scrollable state for each tab
scrollable_states = {tab_name: True}

# Example usage
image_path = r'D:\test.png'
show_image = scrollable_states[tab_name]  # Initial state to show the image and scrollbars
update_image_column(window, image_path, tab_name, show_image, scrollable_states)

while True:
    event, values = window.read()
    if event == sg.WIN_CLOSED:
        break
    elif event == '-TOGGLE-':
        show_image = not scrollable_states[tab_name]
        update_image_column(window, image_path, tab_name, show_image, scrollable_states)
        window.refresh()

window.close()