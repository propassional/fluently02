

from transitions.extensions import GraphMachine
class Model:
    def __init__(self):
        conditions = {}
        pass

class MyGraphMachine:
    def __init__(self):
        # All transitions are dynamic bound to the model, not to the machine
        model = Model()
        self.machine = GraphMachine(model=model, states=['A', 'B', 'C'],
                                    transitions=[
                                        {'trigger': 'CmdAtoB', 'source': 'A', 'dest': 'B'},
                                        {'trigger': 'CmdBtoC', 'source': 'B', 'dest': 'C'}
                                    ],
                                    initial='A', show_conditions=True)

        self.machine.model.conditions = self.conditions_create()
    def get_transition_dict_old1(self):
        conditions = {}
        for transition in self.machine.get_transitions():
            trigger = transition.trigger
            transition_dict[trigger] = trigger
            # trigger = transition['trigger']
            # transition_semaphores[trigger] = True  # You can modify this line to set transition_semaphores dynamically
            # transition_dict[transition['trigger']] = {
            #     'source': transition['source'],
            #     'dest': transition['dest']
            # }
        return conditions

    def get_transition_dict_old2(self):
        transition_dict = {}
        for trigger in self.machine.get_triggers(self.machine.model.state):
            transition_dict[trigger] = trigger
        return transition_dict

    def get_transition_dict_old3(self):
        transition_dict = {}
        for state in self.machine.states:
            for trigger in self.machine.get_triggers(state):
                transition_dict[trigger] = trigger
        return transition_dict

    def get_transition_dict_old4(self):
        transition_dict = {}
        for state in self.machine.states:
            for trigger in self.machine.get_triggers(state):
                # Check if the trigger corresponds to a defined transition
                if trigger in [transition['trigger'] for transition in self.machine.transitions]:
                    transition_dict[trigger] = trigger
        return transition_dict

    def get_transition_dict_old5(self):
        transition_dict = {}
        defined_triggers = [transition['trigger'] for transition in self.machine.transitions]
        for state in self.machine.states:
            for trigger in self.machine.get_triggers(state):
                # Check if the trigger corresponds to a defined transition
                if trigger in defined_triggers:
                    transition_dict[trigger] = trigger
        return transition_dict

    def conditions_create(self):
        transition_dict = {}
        for state in self.machine.states:
            for trigger in self.machine.get_triggers(state):
                # I found no way to distinguish between all potential transitions, and the one that I defined, so "Cmd" distinguishes them
                if trigger.__contains__("Cmd"):
                    transition_dict[trigger] = True
        return transition_dict

if __name__ == '__main__':

    my_graph_machine = MyGraphMachine()
    print("Current state: " + my_graph_machine.machine.model.state)

    # Get the transition dictionary
    # transition_dict = my_graph_machine.get_transition_dict()
    print("Transition Dictionary:")
    print(my_graph_machine.machine.model.conditions)
