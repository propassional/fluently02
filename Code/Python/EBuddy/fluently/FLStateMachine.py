from transitions import Machine
import configparser
import dill as pickle  # only required for Python 3.3 and earlier
from transitions import Machine, State
from transitions.extensions import GraphMachine
from functools import partial
from FLConstants import *

class FLStateMachine(object):

    #states = ['ScannerIsOff', 'PreviewIsOn', 'Recording', 'Scanning', 'ScanStopped']
    #transitions = []

    def __init__(self):
        pass

    def SM_create_empty(self):
        #self.machine = Machine(model=self, states=FLStateMachine.states, initial='ScannerIsOff')
        #self.machine = Machine(model=self, states=FLStateMachine.states)
        #self.machine = Machine(model=self, states=['ScannerIsOff'])
        self.machine = Machine(model=self, states='')
        scanner_is_off = State('ScannerIsOff')
        self.machine.add_states(scanner_is_off)
        self.machine.set_state = next(iter(self.machine.states))

    def SM_create_from_declarations(self):
        #sm_abc = Machine(states=['A', 'B', 'C'], initial='A')
        self.machine = Machine(states=['A', 'B', 'C'], initial='')
        self.machine.set_state = next(iter(self.machine.states))
    def SM_create_from_graphmachine(self):
        model = Model()
        self.machine = GraphMachine(model=model, states=['A', 'B', 'C'],
                               transitions=[
                                   {'trigger': 'clear', 'source': 'B', 'dest': 'A', 'transition_semaphores': model.clear_state},
                                   {'trigger': 'clear', 'source': 'C', 'dest': 'A',
                                    'transition_semaphores': partial(model.clear_state, False, force=True)},
                               ],
                               initial='A', show_conditions=True)
    def SM_create_from_load_file(self, sm_file_name=''):
        if sm_file_name == '':
            sm_file_name = SM_FILE_PICKLE_DEFAULT
        with open(sm_file_name, 'rb') as sm:
            #my_sm = pickle.load(sm)
            self.machine = pickle.load(sm)
            # If self.machine.state is missing, there is a problem later in save graph
            self.machine.set_state = next(iter(self.machine.states))
            #return my_sm
    def config_file_write(self):
        config = configparser.ConfigParser()
        #config['section'] = {'key': 'value'}
        config['DEFAULT'] = {'states': 'ScannerIsOff, PreviewIsOn, Recording, Scanning, ScanStopped'}

        with open('config.ini', 'w') as configfile:
            config.write(configfile)

        FLStateMachine.states.append('AddedState')

    def config_file_read(self):
        config = configparser.ConfigParser()
        config.read('config.ini')
        state_type = 'DEFAULT'
        states = config[state_type]['states']
        states_list = states.split(', ')
        print("The " + state_type + f" states are: {states_list}")

        state_type = 'CURRENT'
        states = config[state_type]['states']
        states_list = states.split(', ')
        print("The " + state_type + f" states are: {states_list}")

        state_type = 'PA'
        states = config[state_type]['states']
        states_list = states.split(', ')
        print("The " + state_type + f" states are: {states_list}")

        this_is_the_end = 1

    def SM_store_to_default_file(self):
        # store the machine
        dump = pickle.dumps(self.machine)

        # load the Machine instance again
        #m2 = pickle.loads(dump)
        #print("Current state: " + m2.state)

        with open('SMDefault.pickle', 'wb') as f:
            f.write(dump)

    def SM_print_states(self):
        for state in self.machine.states:
            print("State: " + state)

class Model:
    def clear_state(self, deep=False, force=False):
        print("Clearing state ...")
        return True

if __name__ == '__main__':
    pass