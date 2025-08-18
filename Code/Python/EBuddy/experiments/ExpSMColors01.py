# This is not working but it's a nice trial :-D by copilot: colors are not changed

from transitions.extensions import GraphMachine

states = ["A", "B", "C"]
transitions = [
    {"trigger": "advance", "source": "A", "dest": "B"},
    {"trigger": "advance", "source": "B", "dest": "C"},
]

class Model:
    pass

# Subclass GraphMachine to customize the graph appearance
class CustomGraphMachine(GraphMachine):
    def get_graph(self, *args, **kwargs):
        graph = super().get_graph(*args, **kwargs)
        # Set background color
        graph.graph_attr['bgcolor'] = 'black'
        # Update node styles
        for node in graph.get_nodes():
            node.attr['fontcolor'] = 'white'
            if node.get_name() == self.state:
                node.attr['color'] = 'blue'
                node.attr['fillcolor'] = 'blue'
            else:
                node.attr['fillcolor'] = 'black'
                node.attr['color'] = 'white'
        return graph

model = Model()
machine = CustomGraphMachine(model=model, states=states, transitions=transitions, initial="A")

# Generate and save the customized graph
graph = machine.get_graph()
graph.draw('state_machine_custom_colors.png', prog='dot')

# Print the current state to verify
print(model.state)

