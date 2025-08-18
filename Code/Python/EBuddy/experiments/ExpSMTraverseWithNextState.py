# Good example: this class implements a simple state machine, that given states = ['A', 'B', 'C']
# it traverses A,B,C,A,B,C etc just by calling machine.next_state()
# If you use a model, .next_state() is bound to the model, not to the machine
# There is no need to define transitions, just use ordered_transitions=True, which enables the "next_state" trigger
# You can uncomment transitions everywhere, and see how everything remains unchanged

import time
from transitions import Machine

if __name__ == '__main__':

    # Define your states
    states = ['A', 'B', 'C']

    # This is unneccessary with this variant
    # transitions = [
    #     {'trigger': 'AToB', 'source': 'A', 'dest': 'B'},
    #     {'trigger': 'BToC', 'source': 'B', 'dest': 'C'},
    #     {'trigger': 'CToA', 'source': 'C', 'dest': 'A'}
    # ]

    # Initialize the machine with ordered transitions
    machine = Machine(
        states=states,
        #transitions=transitions, # This is unneccessary with this variant
        initial='A',
        ordered_transitions=True,  # Enables the "next_state" trigger
        ignore_invalid_triggers=True,  # Ignore invalid triggers
        auto_transitions=False  # Disable automatic transitions
    )

    # Example usage: traverse to the next state
    try:
        while True:
            print("Current state: " + machine.state)
            # Execute transition, but since there are no transition_semaphores, transition always fires to the next state
            # Alternative (to try): next(iter(self.sm_gui.machine.states))
            machine.next_state()
            time.sleep(1)  # Wait for 5 seconds
    except KeyboardInterrupt:
        machine.stop_server()  # Shutdown the machine on Ctrl + C
