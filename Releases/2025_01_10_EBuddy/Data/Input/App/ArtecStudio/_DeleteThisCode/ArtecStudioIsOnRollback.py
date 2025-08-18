import PySimpleGUI as sg
import time

def main():

    layout = [[sg.Text("", text_color="white", font=("Helvetica", 20))]] # This text is not displayed, quite strange!
    window = sg.Window('', layout, finalize=True)
    time_in_sec = 10
    sg.popup_quick_message(f'E-Buddy error message (disappears in {time_in_sec} seconds)\nArtec Studio App could not be opened \nThis transition will be rollbacked now', font=("Helvetica", 20))
    time.sleep(time_in_sec)
    window.close()
    
    # This is needed to verify that the code has been called correctly
    return True