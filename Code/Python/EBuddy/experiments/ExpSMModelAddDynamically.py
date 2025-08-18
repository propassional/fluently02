from transitions import Machine

class PoolController:
    def __init__(self, machine):
        machine.add_model(self)
        self.to_initial_state()

machine = Machine(model=None, states=['dummy_state', 'initial_state'], initial='dummy_state')
controller = PoolController(machine)
pass