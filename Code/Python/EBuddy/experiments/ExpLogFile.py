# Generates following log lines:
# 2024-06-26 11:56:19,671 - INFO - Some stuff is ok from ExpLogFile.py:54 (do_some_stuff)
# 2024-06-26 11:56:19,671 - WARNING - Some stuff is strange from ExpLogFile.py:55 (do_some_stuff)
# 2024-06-26 11:56:19,671 - ERROR - Some stuff is wrong from ExpLogFile.py:56 (do_some_stuff)
# 2024-06-26 11:56:19,671 - INFO - Same stuff from ExpLogFile.py:59 (do_same_stuff)

# Usage concept
# Create a log file, create a FLBaseClass object passing the log file

import logging
import threading
import os
import inspect

class MyLogger:
    def __init__(self):
        self.log_file = "D:\my_log.txt"
        self.message_last = None
        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.INFO)
        self.handler = logging.FileHandler(self.log_file, mode='w')  # Overwrite the log file
        self.handler.setLevel(logging.INFO)
        self.lock = threading.Lock()  # Add a lock for thread safety
        self.formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
        self.handler.setFormatter(self.formatter)
        self.logger.addHandler(self.handler)
    def info(self, message):
        if message != self.message_last:
            with self.lock:  # Acquire the lock before writing to the log file
                self.logger.info(message)
            self.message_last = message
    def warning(self, message):
        if message != self.message_last:
            with self.lock:
                self.logger.warning(message)
    def error(self, message):
        if message != self.message_last:
            with self.lock:
                self.logger.error(message)

class BaseClass:
    def __init__(self, logger):
        self.logger = logger

    def print(self, message, level="info"):
        frame = inspect.currentframe().f_back
        filename = os.path.basename(frame.f_code.co_filename)
        lineno = frame.f_lineno
        funcname = frame.f_code.co_name
        self.last_message = message
        if level == "info":
            self.logger.info(f"{message} from {filename}:{lineno} ({funcname})")
        elif level == "warning":
            self.logger.warning(f"{message} from {filename}:{lineno} ({funcname})")
        elif level == "error":
            self.logger.error(f"{message} from {filename}:{lineno} ({funcname})")

class HelloWorld(BaseClass):
    def __init__(self, logger):
        super().__init__(logger)

    def do_some_stuff(self):
        self.print("Some stuff is ok", level="info")  # Info level
        self.print("Some stuff is strange", level="warning")  # Warning level
        self.print("Some stuff is wrong", level="error")  # Error level

    def do_same_stuff(self):
        self.print("Same stuff", level="info")  # Info level

if __name__ == '__main__':
    my_logger = MyLogger()
    my_world = HelloWorld(my_logger)
    my_world.do_some_stuff()

    my_world.do_same_stuff()
    # This log is the same as the above one and should be deleted
    my_world.do_same_stuff()
    # This log is the same as the above one and should be deleted
    my_world.do_same_stuff()
