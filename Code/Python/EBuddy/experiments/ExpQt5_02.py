import sys
import threading
import time
from PyQt5.QtWidgets import QApplication, QLabel, QMainWindow, QVBoxLayout, QWidget

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
        for message in self.messages:
            self.print_into_window(message)
            time.sleep(1)
            self.window.showNormal()

    def run_old(self):
        for message in self.messages:
            self.label.setText(message)
            time.sleep(5)
            self.label.clear()
            self.window.showMinimized()
            time.sleep(1)  # Wait for a short time before maximizing
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
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
