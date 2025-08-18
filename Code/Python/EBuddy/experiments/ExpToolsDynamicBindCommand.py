# This class enables to add dynamically a method to an instance, and to call it without hardcoding the method name
# That method contains an attribute called "result", that can be changed dynamically, without hardcoding it
# This class can be used for creating dynamically a state, create the transitions to it (the result attribute),
# and then change the state of this transition, and evaluate the transition
# Everything works without the need to hardcode methods nor attributes

class DynamicBindCommand:
    def __init__(self):
        # This is a dictionary of function pointers, used within execute_command
        self.command_functions = {}
        # This is a dictionary of results
        self.command_results = {}

    def register_command(self, command_name):
        # This does the same initialisation behavior of "static" in C
        if command_name not in self.command_results:
            # Set the initial result only if it hasn't been set before
            self.command_results[command_name] = None

        # Inner function
        def command_function(self):
            print(f"Executing command: {command_name}")
            # result = False is a big problem, since it overwrites result at each call
            # This is much better, since it enables the definition of result, but it does not define its value
            result = self.command_results[command_name]
            # setattr dynamically creates a method named after the command
            # lambda: result is a lambda function that, when called, returns the value of the "result" variable
            # The result variable is the outcome of the command logic
            # The lambda function created inside command_function is assigned to the attribute named "command_name"
            setattr(self, command_name, lambda: result)
            return result

        # This line stores "command_function" in the "command_functions" dictionary using the command_name as the key
        # This is crucial because it allows to retrieve the function later using the command name
        # Without this command, this part would fail: result3 = handler.command1()
        # command_function can be seen as a function pointer that is added to self
        self.command_functions[command_name] = command_function

    def set_command_result(self, command_name, new_result):
        if command_name in self.command_results:
            # Update the result variable for the specified command
            self.command_results[command_name] = new_result
        else:
            print(f"Error: Unknown command '{command_name}'")

    def execute_command(self, command_name):
        if command_name in self.command_functions:
            return self.command_functions[command_name](self)
        else:
            print(f"Error: Unknown command '{command_name}'")

if __name__ == '__main__':
    # Example usage
    handler = DynamicBindCommand()

    # 1) Registering commands dynamically
    handler.register_command("command1")

    # 2a) Executing commands without the need of hardcoding "command1"
    result1 = handler.execute_command("command1")
    print(result1)  # Output: None, since only instantiated but not initialised

    # 3) Initialise the "result" variable inside the "command1" method
    handler.set_command_result("command1", True)
    print(handler.command_results["command1"]) # True

    # 2b) Re-executing command1, but now after modification of result
    result1 = handler.execute_command("command1")
    print(result1)  # Output: True

    # Direct access of the previously dynamically created methods
    # For the python "static code analysis" it's obviously an "unresolved attribute", since it does not exist at compile time
    # a. handler.command1() is a call to a method named "command1" on the object handler.
    # b. Python looks for an attribute named "command1" in the handler object.
    # c. Since you've dynamically added a method with the name "command1" using setattr(self, command_name, lambda: result), Python finds this attribute.
    # d. The attribute is a lambda function created inside command_function, which, when called (lambda: result()), returns the value of result.
    # e. The lambda function is invoked, and result is evaluated. In your example, result is False.
    # file. The value of False is assigned to result3.
    result3 = handler.command1()
    print(result3)  # Output: True