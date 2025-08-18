import time
import pygetwindow

def main():

    # Wait until the Artec Studio is up: 12 seconds seems a lot, but with 11 seconds at least once it did not work
    time.sleep(12) # 1 seconds returns False, 10 seconds returns True

    # Get the window title or program ID (modify this based on your app)
    window_title = "Artec Studio 16 Professional"
    # Find the window by title
    my_window = pygetwindow.getWindowsWithTitle(window_title)

    if len(my_window) != 0:
        return True
    else:
        return False

