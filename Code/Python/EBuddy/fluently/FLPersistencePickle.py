# pickle has problems with GraphMachine (see below), but works well with Machine

import pickle
from transitions import Machine

class SMBasic:
    states = ['start', 'middle', 'end']

    transitions = [
        {'trigger': 'go_middle', 'source': 'start', 'dest': 'middle'},
        {'trigger': 'go_end', 'source': 'middle', 'dest': 'end'}
    ]

    def __init__(self):
        self.machine = Machine(model=self, states=SMBasic.states, transitions=SMBasic.transitions, initial='')

    def save_as_pickle(self, filename):
        try:
            with open(filename, 'wb') as pickle_file:
                pickle.dump(self.machine, pickle_file)
                # pickle_file.write(self.machine) # I get this error: ("a bytes-like object is required, not 'Machine'",)
                #pickle_file.write(self)
        except Exception as e:
            print(f"An error occurred: {str(e)}")

    def load_from_pickle(self, filename):
        with open(filename, 'rb') as f:
            loaded_machine = pickle.load(f)
        #sm = cls()
        self.machine = loaded_machine
        #return sm

if __name__ == '__main__':
    # Basic example
    m = Machine(states=['A', 'B', 'C'], initial='A')
    m.to_B()
    print(m.state)
    try:
        # I don't know where it's stored
        dump = pickle.dumps(m)
    except Exception as e:
        print(f"An error occurred: {str(e)}")
    m2 = pickle.loads(dump)
    print(m2.state)

    sm = SMBasic()
    try:
        # I don't know where it's stored
        dump = pickle.dumps(sm)
    except Exception as e:
        print(f"An error occurred: {str(e)}")
    sm2 = pickle.loads(dump)

    sm_file_name = 'd:\\temp\\my_pickle.pickle'
    #sm_file_name = file'D:\Banfi\Github\Fluently\StateMachines\Output\Pickles\SM_2024_03_04__11_52_58.pickle'

    sm.save_as_pickle(sm_file_name)

    # Load the state machine from the pickle file
    sm.load_from_pickle(sm_file_name)

    # Print all states
    print("States:", sm.machine.states)

    # Print all transitions
    print("Transitions:", sm.machine.get_transitions())
