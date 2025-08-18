# This example is focused on a simple GUI library integration: GraphMachine with tkinter

import tkinter as tk
from transitions.extensions import GraphMachine

class App:
    def __init__(self, root):
        self.root = root
        self.root.title("State Machine GUI")

        # Label to display the current state
        self.label = tk.Label(root, text="Current State: A")
        self.label.pack(pady=10)

        # Button to trigger state transitions
        self.button = tk.Button(root, text="Trigger", command=self.trigger)
        self.button.pack(pady=10)

        # Define states and transitions
        states = ['A', 'B', 'C']
        transitions = [
            {'trigger': 'advance', 'source': 'A', 'dest': 'B'},
            {'trigger': 'advance', 'source': 'B', 'dest': 'C'},
            {'trigger': 'advance', 'source': 'C', 'dest': 'A'}
        ]

        # Initialize the state machine with GraphMachine for visualization
        self.machine = GraphMachine(model=self, states=states, transitions=transitions, initial='A')

    def trigger(self):
        self.advance()
        self.label.config(text=f"Current State: {self.state}")


# Create the Tkinter root window
root = tk.Tk()
app = App(root)
root.mainloop()
