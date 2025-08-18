import PySimpleGUI as sg
import time

def main():

    layout = [[sg.Text("\nScanner is not connected, please connect it", text_color="white", font=("Helvetica", 20))]]
    window = sg.Window('', layout, finalize=True)
    time_in_sec = 10
    sg.popup_quick_message(f'E-Buddy error message (disappears in {time_in_sec} seconds)\nScanner is not connected, please connect it', font=("Helvetica", 20))
    time.sleep(time_in_sec)
    window.close()
    
    # This is needed to verify that the code has been called correctly
    return True