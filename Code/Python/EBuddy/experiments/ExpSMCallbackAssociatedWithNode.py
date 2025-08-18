class SMCallbackAssociatedWithNode:
    def __init__(self):
        self.states = {}
        self.current_state = None

    def add_state(self, state_name, callback):
        self.states[state_name] = callback

    def set_state(self, state_name):
        if state_name in self.states:
            self.current_state = state_name
        else:
            print(f"State '{state_name}' does not exist.")

    def run(self):
        if self.current_state:
            callback = self.states.get(self.current_state)
            if callback:
                callback()
            else:
                print(f"Callback for state '{self.current_state}' is not defined.")
        else:
            print("No state set. Use set_state() to set a state.")

# Define the callback for the "HelloWorld" state
def hello_world_callback():
    print("Hello World")

# Create a state machine
sm = SMCallbackAssociatedWithNode()

# Add the "HelloWorld" state with the callback
sm.add_state("HelloWorld", hello_world_callback)

# Set the current state to "HelloWorld"
sm.set_state("HelloWorld")

# Run the state machine
sm.run()