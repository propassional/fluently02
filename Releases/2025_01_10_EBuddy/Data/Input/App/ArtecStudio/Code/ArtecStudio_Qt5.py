import threading
import time
import pygetwindow
import tkinter as tk
import psutil
import sys
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import QApplication, QLabel, QMainWindow, QWidget, QVBoxLayout
from PyQt5.QtCore import QTimer, Qt

import FLOS

message = "Hello World"
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
    global message
    message = text

def window_close_after_some_seconds_old(text, auto_quit=True):
    #FLOS.processes_write_to_file_pre()

    app = QApplication(sys.argv)
    autoquit = False
    window = EBuddyWindow(text, auto_quit)
    window.show() # Launches the GUI
    #sys.exit(app.exec_()) # It works but it also kills EBuddy
    app.exec_() # This ensures that the GUI remains responsive and displays the window.
    window.close()

    # time.sleep(2)
    # FLOS.processes_write_to_file_post()
    # pass # At this point, QApplication is launched but not closed!

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

def ScannerIsOnDo():
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

    # This overrides keyPressEvent, thus this method is called whenever a key is pressed while the window has focus
    # This behavior is always active
    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Escape:
            self.close()

class MessageThread(threading.Thread):
    def __init__(self, messages, label, window):
        super().__init__()
        self.messages = messages
        self.label = label
        self.window = window

    def print_into_window(self, message):
        self.label.setText(message)
        time.sleep(5)
        self.label.clear()
        self.window.showMinimized()

    def run(self):
        global message
        while True:
            if message != "":
                self.print_into_window(message)
                message = ""
            time.sleep(1)
            self.window.showNormal()

    def run_old(self):
        for message in self.messages:
            self.print_into_window(message)
            time.sleep(1)
            self.window.showNormal()

class CustomMainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

    def closeEvent(self, event):
        event.ignore()  # Ignore the close event

def main():
    app = QApplication(sys.argv)
    window = CustomMainWindow()
    central_widget = QWidget()
    layout = QVBoxLayout(central_widget)
    label = QLabel()
    layout.addWidget(label)
    window.setCentralWidget(central_widget)

    messages = ["hello world", "hello world again", "get hello world"]
    message_thread = MessageThread(messages, label, window)
    message_thread.start()

    window.show()
    #sys.exit(app.exec_())
    app.exec_()

if __name__ == "__main__":
    main()
    window_close_after_some_seconds("Eccomi")
    time.sleep(2)
    window_close_after_some_seconds("Mondo")
