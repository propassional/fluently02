import PySimpleGUI as sg
import threading
import time

def close_window_thread(window):
    time.sleep(5)  # Simulate some work
    # Send whatever event to the window, and the window will process it
    #window.write_event_value('-CLOSE WINDOW-', '')
    # Send a sg standard event to close the window
    window.write_event_value(sg.WINDOW_CLOSED, '')

def main():
    layout = [[sg.Text('Hello World')]]
    window = sg.Window('Hello World', layout)

    # Start the parallel thread to close the window
    threading.Thread(target=close_window_thread, args=(window,), daemon=True).start()

    while True:
        event, values = window.read()
        if event == sg.WINDOW_CLOSED:
                break

    window.close()

if __name__ == '__main__':
    main()