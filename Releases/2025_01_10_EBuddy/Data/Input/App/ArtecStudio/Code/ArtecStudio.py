import winsound
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
frequency = 1000  # 1000 Hz
duration = 500    # 500 ms
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

def ArtecStudioIsOnCheck(message_queue):

    # Wait until the Artec Studio is up: 14 seconds seems a lot, but with 14 seconds at least once it did not work
    # Before installing ROS2 the pc was quicker (it may not depend on ROS), this parameter was 12, and did not work anymore
    time.sleep(22) # 1 seconds returns False, 10 seconds returns True

    # Get the window title or program ID (modify this based on your app)
    window_title = "Artec Studio 16 Professional"
    # Find the window by title
    my_window = pygetwindow.getWindowsWithTitle(window_title)

    if len(my_window) != 0:
        return True
    else:
        return False

def ArtecStudioIsOnRollback(message_queue):
    message = f'Artec Studio App could not be opened \nThis transition will be rollbacked now'
    message_queue.put(message)
    return True
    
def CheckStudioIsOffDo(message_queue):
    return ReadyDo(message_queue)

# Check that no Artec Studio instances are opened, if there are any, warn to save data, then kill them
def KillAllStudiosDo(message_queue):
    try:
        one_artec_studio_app_killed = False
        process_name = "astudio_pro.exe"
        for proc in psutil.process_iter():
            if proc.name() == process_name:
                proc.kill()
                one_artec_studio_app_killed = True
        if one_artec_studio_app_killed:
            message = "EBuddy closed Artec Studio"
        else:
            message = "No opened Artec Studio could be found"
        message_queue.put(message)
        return True
    except Exception as error:
        return False

# Check that no Artec Studio instances are opened, if there are any, warn to save data, then kill them
def ReadyDo(message_queue):
    try:
        process_name = "astudio_pro.exe"
        for proc in psutil.process_iter():
            if proc.name() == process_name:
                message = "Save&Close Artec Studio"
                message_queue.put(message)
                winsound.Beep(frequency, duration)
                return False
        #message = "No Artec Studio instances are opened, please Start Artec Studio"
        #message = "Ask me to Start Artec Studio"
        #message = ""
        #message_queue.put(message)
        return True
        # # Kill all opened Artec Studio instances
        # for proc in psutil.process_iter():
        #     if proc.name() == process_name:
        #         proc.kill()
    except Exception as error:
        return False

def StartStudioDo(message_queue):
    return ReadyDo(message_queue)

def SaveFileDo(message_queue):
    # Check if artec studio is up
    result = artec_studio_is_on_check_immediately()
    if not result:
        message = "Artec Studio is not open, nothing to save"
        message_queue.put(message)
        return False # Artec Studio is not even open, thus there is nothing to save
    else:
        message = ""
        message_queue.put(message)
        return True # Artec Studio is open, images will save it

def ScannerIsOnCheck(message_queue):
    message = f'Scanner is now connected'
    message_queue.put(message)
    return True

def ScannerIsOnRollback(message_queue):
    message = f'Scanner not connected, connect it'
    message_queue.put(message)
    winsound.Beep(frequency, duration)
    return True
  
# The Shutdown method is not implemented, since EBuddy itself responds to it
# def Shutdown(message_queue):

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
    StartArtecStudioDo()