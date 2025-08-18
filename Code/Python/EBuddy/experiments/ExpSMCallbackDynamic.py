import types
from transitions import Machine

class MyStateMachine:
    def __init__(self):
        self.machine = Machine(
            model=self,
            states=['state1', 'state2'],
            initial='state1',
            transitions=[['trigger', 'state1', 'state2']]
        )

    def add_dynamic_callback(self, callback_name):
        # Assuming callback_name is read from your text file
        #setattr(self.machine, file"on_enter_{callback_name}", lambda *args, **kwargs: print("have a nice day"))
        setattr(self.machine, callback_name, lambda *args, **kwargs: print("have a nice day"))

    def is_lambda_function(self, func):
        self.my_types = types.LambdaType
        return isinstance(func, types.LambdaType)

# Example usage
if __name__ == "__main__":
    my_instance = MyStateMachine()
    callback_name = "on_enter_state"  # Read from your text file
    my_instance.add_dynamic_callback(callback_name)
    result = my_instance.is_lambda_function(callback_name)
    my_instance.trigger('trigger', param='some_value')
    pass
