import PySimpleGUI as sg

layout = [
    [sg.TabGroup([[sg.Tab('Tab 1', [[sg.Text('This is tab 1')]]),
                   sg.Tab('Tab 2', [[sg.Text('This is tab 2')]])]], key='-TABGROUP-', enable_events=True)],
    [sg.Text('Active Tab:'), sg.Text('', key='-OUTPUT-')]
]

window = sg.Window('Tab Example', layout)

while True:
    event, values = window.read()
    if event == sg.WINDOW_CLOSED:
        break
    if event == '-TABGROUP-':
        active_tab = values['-TABGROUP-']
        window['-OUTPUT-'].update(active_tab)

window.close()