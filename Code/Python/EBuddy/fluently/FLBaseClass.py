import os
import inspect

from FLConstants import INFO, WARNING, ERROR
from FLLogger import FLLogger

class FLBaseClass:
    def __init__(self, logger):
        self.logger = logger

    def print(self, message, level=INFO):
        frame = inspect.currentframe().f_back
        file_name = os.path.basename(frame.f_code.co_filename)
        line_number = frame.f_lineno
        method_name = frame.f_code.co_name
        self.last_message = message
        log_message = f"from {file_name} line {line_number} method {method_name}: {message} "
        if level == INFO:
            #self.logger.info(file"{message} from {file_name}:{line_number} ({method_name})")
            self.logger.info(log_message)
        elif level == WARNING:
            self.logger.warning(log_message)
        elif level == ERROR:
            self.logger.error(log_message)
        self.logger.handler.flush()

class HelloWorld(FLBaseClass):
    def __init__(self, logger):
        super().__init__(logger)

    def do_some_stuff(self):
        self.print("Some stuff is ok", level="info")  # Info level
        self.print("Some stuff is strange", level="warning")  # Warning level
        self.print("Some stuff is wrong", level="error")  # Error level

    def do_same_stuff(self):
        self.print("Same stuff", level="info")  # Info level

if __name__ == '__main__':
    my_logger = FLLogger(r"ArtecStudio__2024_06_20__13_44_28")
    my_world = HelloWorld(my_logger)
    my_world.do_some_stuff()

    my_world.do_same_stuff()
    # This log is the same as the above one and should be deleted
    my_world.do_same_stuff()
    # This log is the same as the above one and should be deleted
    my_world.do_same_stuff()