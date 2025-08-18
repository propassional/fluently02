# This seems to be the best way to traverse a state machine: easy and effective
# Pattern:
# 1) define states
# 2) define transitions
# Or load a state machine
# 3) define transition_semaphores, which have the same name as transitions
# 4) use the generic method set_transition_condition for enabling/disabling transitions
# 5) avoid hardcoding: use getattr(self, transition_name)() for calling transitions
# 6) double transitions: transition priority is given by the transitions order withing self.transition_semaphores
# May dynamic binding help for other features?

from transitions import Machine

class MyStateMachine:
    states = ['A', 'B', 'C']

    transitions = [
        {'trigger': 'AtoB', 'source': 'A', 'dest': 'B'},
        {'trigger': 'BtoA', 'source': 'B', 'dest': 'A'},
        {'trigger': 'BtoC', 'source': 'B', 'dest': 'C'},
        {'trigger': 'CtoA', 'source': 'C', 'dest': 'A'}
    ]

    def __init__(self):
        self.machine = Machine(model=self, states=MyStateMachine.states, transitions=MyStateMachine.transitions,
                               initial='A', ordered_transitions=True, ignore_invalid_triggers=True,
                               auto_transitions=False)

        # Initial transition_semaphores for transitions
        #self.transition_semaphores = {'AtoB': True, 'BtoC': True, 'CtoA': True} # A>B>C>A>B>C etc if all transitions stay true as here
        #self.transition_semaphores = {'AtoB': True, 'BtoA': True, 'BtoC': True, 'CtoA': True} # A>B>A>B>A etc if all transitions stay true as here
        self.conditions = self.conditions_create()

    def conditions_create(self):
        conditions = {}
        for transition in MyStateMachine.transitions:
            trigger = transition['trigger']
            conditions[trigger] = True  # You can modify this line to set transition_semaphores dynamically
        return conditions

    # Set a specific condition to True or False (as a state machine semaphore)
    def transition_condition_set(self, transition_name, condition_boolean):
        # Set the condition for the specified transition
        if transition_name in self.conditions:
            self.conditions[transition_name] = condition_boolean

    def states_traverse(self):
        try:
            while True:
                # Iterate over all transitions and attempt to trigger them based on transition_semaphores
                for transition_name in self.conditions:
                    if self.conditions[transition_name]:
                        print(f"\nState before transition: {self.state}")
                        print(f"Launching transition: {transition_name}")
                        # Runs the specific transition: if transition is True, the transition fires, otherwise nothing happens
                        getattr(self, transition_name)()
                        print(f"State after transition: {self.state}")

        except KeyboardInterrupt:
            # You can add any cleanup logic here
            pass

    def traverse_states_hardcoded(self):
        try:
            while True:
                print(f"Current state: {self.state}") # A

                # Attempt to trigger AtoB
                if self.conditions['AtoB']:
                    self.AtoB() # This fires only if current state is 'A', if activated it does nothing
                    print(f"Current state: {self.state}") # B

                # Attempt to trigger BtoC
                if self.conditions['BtoC']:
                    self.BtoC() # This does not fire is current state is 'A', if activated it does nothing
                    print(f"Current state: {self.state}") # C

                # Attempt to trigger CtoA
                if self.conditions['CtoA']:
                    self.CtoA() # This does not fire is current state is 'A', if activated it does nothing
                    print(f"Current state: {self.state}") # A

        except KeyboardInterrupt:
            # You can add any cleanup logic here
            pass


# Example usage
if __name__ == "__main__":
    # Create an instance of the state machine
    my_state_machine = MyStateMachine()

    # Initial traversal with all transitions set to True
    print("Initial traversal with all transitions set to True:")

    # Update transition_semaphores to disable CtoA transition
    my_state_machine.transition_condition_set('CtoA', False)

    # my_state_machine.traverse_states_hardcoded() # Works fine
    my_state_machine.states_traverse() # Works fine