from transitions import Machine

# Define states and transitions
states = ["initial", "test"]
transitions = [
    {"trigger": "test", "source": "initial", "dest": "test"},
]

# Create a state machine
machine = Machine(states=states, transitions=transitions, initial="initial")

# Define an after_state_change callback
def info(event):
    print("State changed:", event.transition.dest)

machine.add_transition("test", "initial", "test", after_state_change=info)

# Trigger the transition
machine.test()  # prints "State changed: test"
print("Current state:", machine.state)

# Force the state (no transition)
machine.set_state("initial")  # doesn't print anything
print("Current state:", machine.state)