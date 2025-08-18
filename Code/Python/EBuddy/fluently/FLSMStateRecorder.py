class FLSMStateRecorder:
    def __init__(self):
        self.state_memory = {}

    def store_state(self, state_machine, state_name):
        # Store the state name for the given state machine
        self.state_memory[state_machine] = state_name

    def return_state(self, state_machine):
        # Return the last state name for the given state machine
        return self.state_memory.get(state_machine, None)
    def display_all_states(self):
        # Display all state machines and their stored states
        for state_machine, state_name in self.state_memory.items():
            print(f"State Machine: {state_machine}, State: {state_name}")

if __name__ == '__main__':
    sm = FLSMStateRecorder()
    sm.store_state("machine1", "stateB")
    print(sm.return_state("machine1"))  # Output: stateA

    sm.store_state("machine1", "stateA")
    print(sm.return_state("machine1"))  # Output: stateB

    print(sm.return_state("machine2"))  # Output: None (since "machine2" has no stored state)

    sm.store_state("machine2", "stateB")
    sm.store_state("machine3", "stateC")
    sm.store_state("machine4", "stateD")
    sm.store_state("machine5", "stateE")

    sm.display_all_states()

