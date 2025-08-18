import tkinter as tk
from transitions.extensions import GraphMachine
import pygraphviz as pgv
from PIL import Image, ImageTk


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

        # Canvas to display the state machine graph
        self.canvas = tk.Canvas(root, width=400, height=300)
        self.canvas.pack(pady=10)

        # Define states and transitions
        states = ['A', 'B', 'C']
        transitions = [
            {'trigger': 'advance', 'source': 'A', 'dest': 'B'},
            {'trigger': 'advance', 'source': 'B', 'dest': 'C'},
            {'trigger': 'advance', 'source': 'C', 'dest': 'A'}
        ]

        # Initialize the state machine with GraphMachine for visualization
        self.machine = GraphMachine(model=self, states=states, transitions=transitions, initial='A')

        # Draw the initial state machine graph
        self.draw_graph()

    def trigger(self):
        self.advance()
        self.label.config(text=f"Current State: {self.state}")
        self.draw_graph()

    def draw_graph(self):
        # Generate the state machine graph
        graph = self.machine.get_graph()
        graph.draw('state_machine.png', prog='dot', format='png')

        # Load the image and display it on the canvas
        image = Image.open('state_machine.png')
        image = image.resize((400, 300), Image.ANTIALIAS)
        self.photo = ImageTk.PhotoImage(image)
        self.canvas.create_image(0, 0, anchor=tk.NW, image=self.photo)


# Create the Tkinter root window
root = tk.Tk()
app = App(root)
root.mainloop()
