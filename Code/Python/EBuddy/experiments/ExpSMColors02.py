from transitions.extensions import GraphMachine

# Define states and transitions
states = ["A", "B", "C"]
transitions = [
    {"trigger": "advance", "source": "A", "dest": "B"},
    {"trigger": "advance", "source": "B", "dest": "C"},
]

# Create the model class
class Model:
    pass

# Initialize the model and state machine
model = Model()
machine = GraphMachine(model=model, states=states, transitions=transitions, initial="A")

# Set visual attributes
machine.get_graph().attr(rankdir="LR", bgcolor="black", fontcolor="white")  # Black background, white text
machine.get_graph().node_attr.update(style="filled", fillcolor="gray", fontcolor="white")  # Default nodes
machine.get_graph().node("A", fillcolor="blue")  # Highlight active node in blue
machine.get_graph().edge_attr.update(color="white")  # White edges for contrast

# Generate and view the state machine graph
machine.get_graph().view()
