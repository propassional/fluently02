import PySimpleGUI as sg

# Sample dictionary (replace with your actual dictionary)
my_dict = {
    'Name': 'John Doe',
    'Age': 30,
    'Email': 'john.doe@example.com',
    'Country': 'USA',
    # ... other fields ...
}

# Create layout for the window
layout = [
    [sg.Text(f'{SM_found}: {value}')] for SM_found, value in my_dict.items() # Vertical layout
]

# Create the window
window = sg.Window('Dictionary Viewer', layout, finalize=True)

# Event loop
while True:
    event, _ = window.read()

    if event == sg.WINDOW_CLOSED:
        break

window.close()
