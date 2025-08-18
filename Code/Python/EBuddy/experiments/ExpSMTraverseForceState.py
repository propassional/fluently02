from transitions import Machine

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
    my_model = Model()
    my_machine = Machine(model=my_model, states=['A', 'B'], initial='A')
    my_machine.add_transition('by_name', 'A', 'B', conditions='coin_toss')  # , after='a_callback')

    print(f"Current state: {my_machine.model.state}")

    # Force the state change programmatically:
    # transitions are not used, since there are not even transitions between A and B
    my_machine.model.state = "B" # Works

    print(f"Current state: {my_machine.model.state}")

    my_machine.model.state = "A" # Works

    print(f"Current state: {my_machine.model.state}")


# Alternative B

# from transitions import Machine
#
# class MyStateMachine:
#     def __init__(self):
#         self.states = ["A", "B"]
#         self.machine = Machine(model=self, states=self.states, initial="A")
#
# my_machine = MyStateMachine()
#
# print(file"Current state: {my_machine.state}")
#
# # Force the state change programmatically: transitions are not used, since there are not even transitions between A and B
# my_machine.state = "B" # Works
#
# print(file"Current state: {my_machine.state}")