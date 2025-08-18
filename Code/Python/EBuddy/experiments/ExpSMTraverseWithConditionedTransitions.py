# This example shows how to bind functions to transitions: they act as boolean transition_semaphores, be True or False or random
# The second part roughly shows how to use imported functions

from transitions import Machine
#from mod import imported_func
import random

class Model(object):

    # def a_callback(self):
    #     imported_func()

    @property
    def coin_toss(self):
        """ Basically a coin toss. """
        # coin_toss = random.random()
        # return coin_toss < 0.5
        return random.random() < 0.5

    an_attribute = False

if __name__ == '__main__':
    model = Model()
    my_machine = Machine(model=model, states=['A', 'B'], initial='A')
    my_machine.add_transition('by_name', 'A', 'B', conditions='coin_toss') #, after='a_callback')
    #my_machine.add_transition('by_reference', 'A', 'A', unless=['a_property', 'an_attribute'], after=model.a_callback)
    my_machine.add_transition('imported', 'A', 'A', after='mod.imported_func')

    print(model.state) # It is resolved, but shown as unresolved

    my_machine.state = "B" # Does not work, but no error is returned
    print(model.state)

    model.by_name() # by_name is a transition having the coin_toss condition: it is random so it may fire or not
    print(model.state)
    model.by_name()

    # model.by_reference()
    # model.imported()