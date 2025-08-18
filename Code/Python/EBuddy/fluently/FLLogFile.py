# This class has been substituted by FLBaseClass and FLLogger

import threading
import logging
import getpass
from datetime import datetime
import inspect
from FLConstants import LOG_FILES, RELEASE_NAME

class FLLogFile:
    def __init__(self, name):
        user_name = getpass.getuser()
        # Lock enables writing to log file both from GUI and GUIUpdater
        self.log_file_lock = threading.Lock()
        formatted_timestamp = datetime.now().strftime("%Y_%m_%d__%H_%M_%S")
        # Create a Python logger with a unique name logging.getLogger(name)
        # otherwise coding logging.getLogger(__name__) all logs are sent to all opened log files, which is wrong
        self.log_file_name = LOG_FILES + '\\' + f"{user_name}_{RELEASE_NAME}__{formatted_timestamp}__{name}.txt"
        self.logger = logging.getLogger(name)
        self.logger.setLevel(logging.DEBUG)  # Set the desired log level
        file_handler = logging.FileHandler(self.log_file_name) # Creates the log file
        formatter = logging.Formatter('[%(asctime)s] %(levelname)s [%(filename)s:%(lineno)d] %(message)s')
        self.logger.addHandler(file_handler)
        file_handler.setFormatter(formatter)

    def write(self, message):
        with self.log_file_lock:
            #caller_frame = inspect.stack()[1]
            #caller_info = inspect.getframeinfo(caller_frame[0])
            #log_message = file"{caller_info.function}() - {message}"
            #self.logger.info(log_message)
            self.logger.info(message) # This is THE line that will be written into the log file, not the calling one

    def close(self):
        for handler in self.logger.handlers:
            handler.close()
            self.logger.removeHandler(handler)

if __name__ == '__main__':
    log_file_instance = FLLogFile()
    log_file_instance.write("Log entry 1")
    log_file_instance.write("Log entry 2")
    log_file_instance.close()
