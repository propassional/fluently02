import json
import os
import sys

from transitions import Machine
from transitions.extensions import GraphMachine
import PySimpleGUI as sg

from FLConstants import JSON_FILES, MSG_CLOSE_APP
#from FLGraphMachine import FLGraphMachine
from FLModel import FLModel

class MyModel:
    pass

class GenericStateMachine:
    def __init__(self):
        my_model = MyModel
        self.states = ['Start']
        self.transitions = []
        self.machine = Machine(model=my_model, states=self.states, transitions=self.transitions, initial='Start')

class MyStateMachine:
    def __init__(self, states, transitions, firstState):
        my_model = MyModel
        self.machine = Machine(model=my_model, states=states, transitions=transitions, initial='Start')

    def on_enter_each_state(self):
        # Custom logic for each state
        pass

    # This is a special case, where states and transitions are defined explicitly

    # Generic version, extract states and transitions from SM, then save JSON
    def SM_load_from_json(self, filename):
        with open(filename, 'r') as json_file:
            machine_dict = json.load(json_file)

        # Reconstruct the machine
        self.machine = Machine(model=GenericStateMachine(), **machine_dict)

    def save_graph_to_file_basic(self, file_name):
        graph = self.machine.get_graph()
        # Good for small SM, but many overlaps on big SM
        graph.graph_attr['rankdir'] = 'TB' # 'LR' vs 'TB', Neato & fdp & sfdp does not support rankdir
        #self.sm_gui.machine.get_graph().draw(SM_GRAPH_DEFAULT_SIZE, format='png', prog='sfdp', args='-Goverlap=scale')

        # Save uncropped graph image, used for creating the cropped image if needed
        my_format = 'png'
        my_prog = 'dot'
        self.machine.get_graph().draw(file_name, format=my_format, prog=my_prog) # Works well for LR graph, rankdir does not change it
        message = f"SM graph saved as {file_name}"

# Generic version, extract states and transitions from SM, then save JSON
def SMGraph_load_from_json(filename, code_path, log_file=None):
    if os.path.exists(filename) and filename.lower().endswith(".json"):
        with open(filename, 'r') as json_file:
            machine_dict = json.load(json_file)
    else:
        sg.popup("SM can not be loaded from: " + filename)
        sg.popup(MSG_CLOSE_APP)
        sys.exit(MSG_CLOSE_APP)

    # Reconstruct the machine
    #self.machine = FLGraphMachineMachine(model=GenericStateMachine(), **machine_dict)
    # It would be nice to add a second callback, on_enter_state='on_enter_state_callback'
    # but it is not trivial how to do it dynamically (see SM* experiments with callbacks), thus everything will be done with a single callback
    new_machine = GraphMachine(model=FLModel(log_file, filename, code_path), **machine_dict, after_state_change='callback_after_state_change')
    # self.machine = GraphMachine(model=model, initial='Start',
    #                             after_state_change='on_enter_each_state')  # It used to work, before the multi-tab implementation
    return new_machine

def SM_save_to_json(machine, filename):

    # Problem: this error in json.dump happens when the state ordered dictionary is passed: TypeError: Object of type State is not JSON serializable
    # Solution: extract a list from the ordered dictionary self.machine.states, as done here
    extracted_states = []
    extracted_states = list(machine.states.keys())

    # Extract transitions
    extracted_transitions = []
    for transition in machine.events.values():
        if transition.name.__contains__("to_"):
            pass
        else:
            for model_transition in transition.transitions.values():
                for dest in model_transition:
                    extracted_transitions.append({
                        'trigger': transition.name,
                        'source': dest.source,  # dest.source.name,
                        'dest': dest.dest  # dest.dest.name
                    })

    # Convert the machine to a dictionary
    machine_dict = {
        'states': extracted_states, # Why this error in json.dump? TypeError: Object of type State is not JSON serializable
        'transitions': extracted_transitions, # There is no machine.transitions vs self.sm_gui.machine.events.values
        'initial': machine.initial
    }

    # Save the dictionary to a JSON file
    with open(filename, 'w') as json_file:
        json.dump(machine_dict, json_file, indent=4)

if __name__ == '__main__':

    SM_machine = MyStateMachine([], [], '')
    SM_machine.machine = SMGraph_load_from_json(
        r'D:\Banfi\Github\Fluently\Releases\2024_04_09_Teachix\Data\Input\JSON\ProcessingOrthogonal_2024_05_20__11_57_32.json')
    SM_machine.save_graph_to_file_basic(
        r"D:\Banfi\Github\Fluently\Releases\2024_04_09_Teachix\Data\Output\Screenshots\ProcessingOrthogonal_2024_05_20__11_57_32.png")

    ######################################################################################################

    SM_machine = MyStateMachine([], [], '')
    SM_machine.machine = SMGraph_load_from_json(
        r'D:\Banfi\Github\Fluently\Releases\2024_04_09_Teachix\Data\Input\JSON\Processing_2024_05_20__11_57_32.json')
    SM_machine.save_graph_to_file_basic(
        r"D:\Banfi\Github\Fluently\Releases\2024_04_09_Teachix\Data\Output\Screenshots\Processing_2024_05_20__11_57_32.png")

    ######################################################################################################

    SM_machine = MyStateMachine([], [], '')
    SM_machine.machine = SMGraph_load_from_json(r'D:\Banfi\Github\Fluently\Releases\2024_04_09_Teachix\Data\Input\JSON\DK_2024_05_06__08_27_18.json')
    SM_machine.save_graph_to_file_basic(r"D:\Banfi\Github\Fluently\Releases\2024_04_09_Teachix\Data\Output\Screenshots\DK_2024_05_06__08_27_18.png")

    ######################################################################################################

    SM_machine = MyStateMachine([], [], '')
    SM_machine.machine = SMGraph_load_from_json(r'D:\Banfi\Github\Fluently\Releases\2024_04_09_Teachix\Data\Input\JSON\Cobot_2024_05_02__14_20_15.json')
    SM_machine.save_graph_to_file_basic(r"D:\Banfi\Github\Fluently\Releases\2024_04_09_Teachix\Data\Output\Screenshots\Cobot_2024_05_02__14_20_15.png")

    ######################################################################################################

    states = ['Start', 'Middle', 'End']
    transitions = [
        {'trigger': 'move_forward', 'source': 'Start', 'dest': 'Middle'},
        {'trigger': 'move_backward', 'source': 'Middle', 'dest': 'Start'},
        {'trigger': 'finish', 'source': 'Middle', 'dest': 'End'}
    ]

    # Create an instance of your GraphMachine
    my_machine1 = MyStateMachine(states, transitions, 'Start')

    # Save SM to a JSON file
    #SM_save_to_json(my_machine1.machine, 'D:\\Banfi\\Github\\Fluently\\StateMachines\\Output\\JSON\\SM_2024_03_05__10_30_38.json')
    SM_save_to_json(my_machine1.machine, JSON_FILES + '\SM1.json') # Works

    # Create an empty state machine
    my_machine2 = MyStateMachine([], [], '')
    my_machine2.SM_load_from_json(JSON_FILES + '\SM1.json')
    SM_save_to_json(my_machine2.machine, JSON_FILES + '\SM1Saved.json')

    my_machine2.machine = SMGraph_load_from_json(JSON_FILES + '\SM1.json')
    SM_save_to_json(my_machine2.machine, JSON_FILES + '\SM1GraphSaved.json')

    # Save GraphMachine to JSON
    gm_default = FLGraphMachine()
    SM_save_to_json(gm_default.machine, JSON_FILES + '\SMGraphMachine.json')

    pass

