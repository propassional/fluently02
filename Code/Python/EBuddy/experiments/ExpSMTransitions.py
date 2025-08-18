class GraphMachine:
    def __init__(self):
        self.transitions = {}
        self.states = set()

    def add_transition(self, from_state, to_state, transition_name):
        if from_state not in self.transitions:
            self.transitions[from_state] = []
        self.transitions[from_state].append((to_state, transition_name))
        self.states.add(from_state)
        self.states.add(to_state)

    def find_transitions_from_states_with_no_incoming(self):
        incoming_transitions = {state: 0 for state in self.states}

        for from_state, transitions in self.transitions.items():
            for to_state, _ in transitions:
                incoming_transitions[to_state] += 1

        no_incoming_states = [state for state, count in incoming_transitions.items() if count == 0]

        result_transitions = []
        for state in no_incoming_states:
            if state in self.transitions:
                result_transitions.extend([name for _, name in self.transitions[state]])

        return result_transitions

    def get_all_transitions_wrong_result(self):
        # Problem: you get ALL potentially possible transitions between ALL nodes, not the defined ones!
        for transition in self.machine.get_transitions():
            pass

    # The problem is the same as the one with self.machine.get_transitions():
    # All potential possible transitions are returned!!
    def is_isolated_wrong_result(self, state):
        for event in self.machine.events.values():
            for event_transitions in event.transitions.values():
                for transition in event_transitions:
                    if transition.source == state or transition.dest == state:
                        return False  # This state is not isolated
        return True  # This state is isolated

all_states = {state: 0 for state in self.machine.states}
for state in all_states:
    if self.is_isolated(state):
        all_states[state] = 1  # This state is isolated

# Example usage:
graph = GraphMachine()
graph.add_transition('A', 'B', 't1')
graph.add_transition('B', 'C', 't2')
graph.add_transition('C', 'D', 't3')
graph.add_transition('E', 'F', 't4')

transitions = graph.find_transitions_from_states_with_no_incoming()
print("Transitions from states with no incoming transitions:", transitions)

# Define the states
states = ['A', 'B', 'C']

# Define the transitions
transitions = [
    {'trigger': 'advance', 'source': 'A', 'dest': 'B'},
    {'trigger': 'advance', 'source': 'B', 'dest': 'C'},
    {'trigger': 'reset', 'source': 'C', 'dest': 'A'}
]

# Create the state machine
class Matter:
    pass

machine = GraphMachine(model=Matter(), states=states, transitions=transitions, initial='A')

# Accessing the transitions
for trigger in machine.events:
    for transition in machine.events[trigger].transitions:
        print(file"Trigger: {trigger}, Source: {transition.source}, Destination: {transition.dest}")
