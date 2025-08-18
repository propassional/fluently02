import logging

class CustomLogger:
    def __init__(self, name, level=logging.INFO, log_file_path="my_log_file.log"):
        self.logger = logging.getLogger(name)
        self.logger.setLevel(level)
        self.log_file_path = log_file_path
        self.handler = logging.FileHandler(log_file_path)  # Specify the log file name
        formatter = logging.Formatter('%(levelname)s: %(message)s')
        self.handler.setFormatter(formatter)
        self.logger.addHandler(self.handler)

class MainObject:
    def __init__(self, log_object):
        self.log_object = log_object

    def process_command(self, command):
        # Process the command
        # ...
        # Log a message
        self.print(f"process_command: {command}")

    def print(self, command):
        self.log_object.logger.info(f"{command}")

class CommandObject:
    def __init__(self, log_object):
        self.log_object = log_object

    def execute(self, command):
        # Execute the command
        # ...
        # Log a message
        self.print(f"execute: {command}")

    def print(self, command):
        self.log_object.logger.info(f"{command}")

# Example usage
if __name__ == "__main__":

    custom_logger = CustomLogger(__name__, log_file_path=r"D:\Banfi\my_app.log")
    custom_logger.logger.info(custom_logger.log_file_path)

    main_obj = MainObject(custom_logger)
    command_obj = CommandObject(custom_logger)

    # Process a command and log messages
    main_obj.process_command("Do the process command")
    command_obj.execute("Do the command object execute")

