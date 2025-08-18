from transitions.extensions.diagrams import GraphMachine
from FLLogFileOld import FLLogFile

class StateHistory:
    def __init__(self):
        self.state_history = []
        self.transitions = {}
        self.log_file = FLLogFile()

    def print_hi(name):
        # Use a breakpoint in the code line below to debug your script.
        print(f'Hi, {name}')  # Press Ctrl+F8 to toggle the breakpoint.

    def on_enter_state(self):
        self.log_file.write("Current state: " + self.state)
        self.log_file.write("Current transitions: " + str(self.transitions))
        self.log_file.write("Current state history: " + str(self.state_history))

        # Update state history
        # if len(self.state_history) >= 2:
        #     self.transitions[self.state] = self.state_history[-2]
        # self.state_history.append(self.state)
        if len(self.state_history) >= 1:
            self.transitions[self.state] = self.state_history[-1]
        self.state_history.append(self.state)
        self.log_file.write("Updated transitions: " + str(self.transitions))
        self.log_file.write("Updated state history: " + str(self.state_history))

if __name__ == '__main__':

    model = StateHistory()

    states = ['A', 'B', 'C', 'D']
    transitions = [
        {'trigger': 'move_on', 'source': 'A', 'dest': 'B'},
        {'trigger': 'move_on', 'source': 'B', 'dest': 'C'},
        {'trigger': 'move_on', 'source': 'C', 'dest': 'D'}
    ]

    # Works with both:
    #machine = Machine(model, states=states, transitions=transitions, initial='A', after_state_change='on_enter_state')
    machine = GraphMachine(model=model, states=states, transitions=transitions, initial='A', after_state_change='on_enter_state')

    model.log_file.write("Current state: " + model.state)
    model.state_history.append(machine.model.state)

    # Dynamic transition firing
    triggers = machine.get_triggers(machine.model.state)
    for trigger in triggers:
        # My transitions are not called "to_"
        if not trigger.__contains__('to_'):
            try:
                # Fire the transition trigger
                machine.model.trigger(trigger)
                print("Current state: " + machine.model.state)
            except transitions.core.MachineError as e:
                print(f"An error occurred: {e}")

    # Hardcoded transition firing
    model.move_on()  # Transition from A to B
    model.move_on()  # Transition from B to C
    model.move_on()  # Transition from C to D

    print(model.transitions)  # Outputs: {'B': 'A', 'C': 'B', 'D': 'C'}
    model.log_file.write("Final transitions: " + str(model.transitions))