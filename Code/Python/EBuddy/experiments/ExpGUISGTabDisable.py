import PySimpleGUI as sg

# Define user roles
#user_role = 'viewer'  # Change this to 'admin' to see the difference
user_role = 'admin'  # Change this to 'viewer' to see the difference

# Define the layout with tabs
layout = [
    [sg.TabGroup([
        [sg.Tab('Tab 1', [[sg.Text('Content of Tab 1')]], key='-TAB1-', disabled=(user_role == 'viewer')),
         sg.Tab('Tab 2', [[sg.Text('Content of Tab 2')]], key='-TAB2-')]
    ])]
]

# Create the window
window = sg.Window('Multi-User System', layout)

# Event loop
while True:
    event, values = window.read()
    if event == sg.WIN_CLOSED:
        break

window.close()
