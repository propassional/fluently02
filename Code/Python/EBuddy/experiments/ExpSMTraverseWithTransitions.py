# Traverse the state machine by calling its transitions, that may fire or not

from transitions import Machine
import random

class MyModel:
    # Not used
    def is_larger_than_5(self):
        result = random.randint(1, 10) > 5
        print(f"is_larger_than_5: {result}")
        return result

if __name__ == '__main__':

    my_model = MyModel()
    states = ['Begin', 'MakeA', 'MakeB', 'End']
    transitions = [
        {'trigger': 'proceed', 'source': 'Begin', 'dest': 'MakeB'},
      #  {'trigger': 'proceed', 'source': 'Begin', 'dest': 'MakeA', transition_semaphores = 'is_larger_than_5' },
        {'trigger': 'proceed', 'source': ['MakeA', 'MakeB'], 'dest': 'End'}
    ]

    machine = Machine(model=my_model, states=states, transitions=transitions, initial='Begin')

    while my_model.state != 'End':
        print(f"Current state: {my_model.state}")
        my_model.proceed() # Transition defined above
