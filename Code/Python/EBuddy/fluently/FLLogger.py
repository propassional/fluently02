import getpass
import logging
import threading
from datetime import datetime

from FLConstants import LOG_FILES, RELEASE_NAME

class FLLogger:
    def __init__(self, name):
        user_name = getpass.getuser()
        formatted_timestamp = datetime.now().strftime("%Y_%m_%d__%H_%M_%S")
        # Create a Python logger with a unique name logging.getLogger(name)
        # otherwise coding logging.getLogger(__name__) all logs are sent to all opened log files, which is wrong
        self.log_file_path = LOG_FILES + '\\' + f"{user_name}_{RELEASE_NAME}__{formatted_timestamp}__{name}.txt"

        self.message_last = None
        # self.logger = logging.getLogger(__name__) # This means that all instances of FLLogger will share the same logger object because __name__ is a module_declared_at_top_of_the_file-level variable that is constant for the entire runtime of the program. As a result, all messages are sent to all handlers attached to this logger, which includes the file handlers for both log files.
        self.logger = logging.getLogger(name)
        self.logger.setLevel(logging.INFO)
        self.handler = logging.FileHandler(self.log_file_path, mode='w')  # Overwrite the log file
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