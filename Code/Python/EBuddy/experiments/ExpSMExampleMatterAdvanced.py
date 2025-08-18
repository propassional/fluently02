# Example with specific state activities:
# prepare (before transition), transition_semaphores (for transition to fire), and post-state activities
import random
from transitions import Machine, State

class Matter(object):
    heat = False
    attempts = 0
    def count_attempts(self):
        self.attempts += 1
    def heat_up(self):
        self.heat = random.random() < 0.25
    def stats(self):
        print('It took you %i attempts to melt the lump!' %self.attempts)

    @property
    def is_really_hot(self):
        return self.heat

if __name__ == '__main__':

    states = ['solid', 'liquid', 'gas', 'plasma']
    transitions = [
        {'trigger': 'melt', 'source': 'solid', 'dest': 'liquid', 'prepare': ['heat_up', 'count_attempts'],
         'transition_semaphores': 'is_really_hot', 'after': 'stats'},
    ]
    lump = Matter()
    machine = Machine(lump, states, transitions=transitions, initial='solid')
    print("Current state: " + machine.model.state)
    # Should return the available triggers from state, but returns all triggers
    triggers = machine.get_triggers(machine.model.state)
    for trigger in triggers:
        if not trigger.__contains__('to_'):
            # My transitions are not called "to_"
            my_trigger = trigger

    # I could not use this method, since I could not find the transition name
    #my_transitions = machine.get_transitions(source=machine.model.state)
    #my_triggers = [t.trigger for t in machine.get_transitions(source=machine.model.state)]

    machine.model.trigger(my_trigger)
    print("Current state: " + machine.model.state)

    lump.melt()
    # Attention: it may need two attempts or more to do this transition
    print("Current state: " + machine.model.state)

    lump.melt()
    print("Current state: " + machine.model.state)