import multiprocessing
import sys

import psutil
from PyQt5.QtWidgets import QApplication, QWidget

import FLOS


def get_process_name():
    return multiprocessing.current_process().name

def get_pid(process_name):
    for proc in psutil.process_iter():
        if proc.name() == process_name:
            return proc.pid

if __name__ == '__main__':
    # Create a QApplication instance
    app = QApplication([])

    # Create a basic window (QWidget)
    window = QWidget()
    window.setWindowTitle("Hello, World!")  # Set window title
    window.show()  # Show the window

    FLOS.processes_write_to_file_pre()

    # Start the event loop
    app.exec_() # In debug mode we stay here until the window is closed
    #sys.exit(app.exec_())

    FLOS.processes_write_to_file_post()

    print("Process name:", get_process_name())

    process_name = "fud"  # Replace with the desired process name
    print(f"Process ID for '{process_name}':", get_pid(process_name))

