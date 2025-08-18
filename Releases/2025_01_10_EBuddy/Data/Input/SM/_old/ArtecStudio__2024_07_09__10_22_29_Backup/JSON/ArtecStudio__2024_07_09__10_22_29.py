import time
import pygetwindow
import sys
import tkinter as tk
import psutil
import sys
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import QApplication, QLabel, QMainWindow, QWidget
from PyQt5.QtCore import QTimer, Qt

window_close_interval_in_ms = 5000
window_close_interval_in_sec = window_close_interval_in_ms / 1000

######################## Part0 Infrastructure Methods ##############################

def artec_studio_is_on_check_immediately():

    # Get the window title or program ID (modify this based on your app)
    window_title = "Artec Studio 16 Professional"
    # Find the window by title
    my_window = pygetwindow.getWindowsWithTitle(window_title)

    if len(my_window) != 0:
        return True
    else:
        return False

def window_close_after_some_seconds(text, auto_quit=True):
    app = QApplication(sys.argv)
    autoquit = False
    window = EBuddyWindow(text, auto_quit)
    window.show() # Launches the GUI
    #sys.exit(app.exec_()) # It works but it also kills EBuddy
    app.exec_() # Fills the GUI
    pass

######################## Part1 Commands ##############################

def ArtecStudioIsOnCheck():

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

def ArtecStudioIsOnRollback():
    message = f'Artec Studio App could not be opened \nThis transition will be rollbacked now'
    window_close_after_some_seconds(message, auto_quit=True)
    return True

def SaveFileIsOnDo():
    # Check if artec studio is up
    result = artec_studio_is_on_check_immediately()
    if not result:
        window_close_after_some_seconds("Artec Studio is not open, please open it", auto_quit=True)
        return False # Artec Studio is not even open, thus there is nothing to save
    else:
        return True # Artec Studio is open

#now
def ScannerIsOnCheck():
    message = f'Scanner is now connected'
    window_close_after_some_seconds(message, auto_quit=True)
    return True

def ScannerIsOnRollback():
    message = f'Scanner is not connected, please connect it'
    window_close_after_some_seconds(message, auto_quit=True)
    return True

# Check that no Artec Studio instances are opened, if there are any, warn to save data, then kill them
def StartArtecStudioDo():
    try:
        process_name = "astudio_pro.exe"
        for proc in psutil.process_iter():
            if proc.name() == process_name:
                message = "Please save now your work in Artec Studio, since it will be killed after you close this window"
                # The user is forced to quit, unfortunately, but this allows him to save its work if needed
                window_close_after_some_seconds(message, False)
                break
        # Kill all opened Artec Studio instances
        for proc in psutil.process_iter():
            if proc.name() == process_name:
                proc.kill()
        return True
    except Exception as error:
        return False

class EBuddyWindow(QMainWindow):
    def __init__(self, text_input, auto_quit=True):
        try:
            super().__init__()
            if auto_quit:
                basis_message = f"\n(This message will disappear in {window_close_interval_in_sec} seconds)"
            else:
                basis_message = "\n(Quit this message with Esc)"
            message = text_input + basis_message
            self.setWindowTitle("EBuddy Warning")
            self.setAutoQuit(auto_quit)
            # Create a label with red background
            label = QLabel(self)
            label.setText(message)
            label.setFont(QFont("Arial", 20))
            label.adjustSize()  # Adjust the label size to fit the text
            label.move(20, 20)
            label.setStyleSheet("background-color: red; color: white;")
            # Set the window size based on the label size
            self.setGeometry(100, 100, label.width() + 40, label.height() + 40)
        except Exception as error:
            self.print(f"Error {error}", Error)

    def setAutoQuit(self, auto_quit):
        if auto_quit:
            self.timer = QTimer(self)
            self.timer.timeout.connect(self.close)
            self.timer.start(window_close_interval_in_ms)

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Escape:
            self.close()

if __name__ == "__main__":
    StartArtecStudioDo()
    ScannerIsOnRollback()
    window_close_after_some_seconds("Test this message", auto_quit=False)
    #window_close_after_some_seconds("Test this message", auto_quit=True)
    StartArtecStudioDo()