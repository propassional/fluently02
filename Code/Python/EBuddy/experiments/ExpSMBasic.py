from transitions import Machine

class SMBasic:
    states = ['start', 'middle', 'end']

    transitions = [
        {'trigger': 'go_middle', 'source': 'start', 'dest': 'middle'},
        {'trigger': 'go_end', 'source': 'middle', 'dest': 'end'}
    ]

    def __init__(self):
        self.machine = Machine(model=self, states=SMBasic.states, transitions=SMBasic.transitions, initial='')
        #self.machine.add_state('end')

if __name__ == '__main__':
    pass