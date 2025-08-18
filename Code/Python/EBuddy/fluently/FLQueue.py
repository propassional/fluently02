import queue
from FLBaseClass import FLBaseClass

class FLQueue(FLBaseClass):
    def __init__(self, logger):
        super().__init__(logger)
        self.queue = queue.Queue()

    def element_insert(self, SM_name, command, autopilot_is_on):
        # Create an element with two strings and a flag (default to False)
        element = CommandElement(SM_name, command, autopilot_is_on) # command is a string
        self.queue.put(element)
        self.print(f"element_insert: inserted SM_name = {element.SM_name}, command = {element.command}, autopilot_is_on = {element.autopilot_is_on}")

    def element_remove(self):
        try:
            # Get the next element from the queue
            element = self.queue.get_nowait()
            self.print(
                f"element_remove: removed SM_name = {element.SM_name}, command = {element.command}, autopilot_is_on = {element.autopilot_is_on}")
            return element
        except queue.Empty:
            return None

    def elements_display(self):
        while not self.queue.empty():
            string1, string2, flag = self.queue.get()
            print(f"String1: {string1}, String2: {string2}, Flag: {flag}")

    # def print(self, text):
    #     self.log_object.logger.info(file"{text}")

class CommandElement:
    def __init__(self, SM_name, command, autopilot_is_on):
        self.SM_name = SM_name
        self.command = command
        self.autopilot_is_on = autopilot_is_on

# Example usage
if __name__ == "__main__":
    my_command_element = CommandElement("SMName", "command", False)
    my_queue = FLQueue()

    # Insert elements
    my_queue.element_insert("ArtecStudio__2024_06_13__15_43_18", "CmdStartArtecStudio", True)
    my_queue.element_insert("Cobot__2024_06_13__15_43_18", "CmdIConfirmControllerIsStarted", False)
    my_queue.elements_display()
    # Remove elements
    while True:
        element = my_queue.element_remove()
        if element is None:
            break
        string1, string2, flag = element
        print(f"String1: {string1}, String2: {string2}, Flag: {flag}")