import threading
from transitions import Machine

class MyModel:
    def __init__(self):
        self.state_history = []
        self.transitions = {}

    def on_enter_state(self):
        if len(self.state_history) >= 2:
            self.transitions[self.state] = self.state_history[-2]
        self.state_history.append(self.state)

# Global variables
model = MyModel()

states = ['A', 'B', 'C']
transitions = [
    {'trigger': 'move_on', 'source': 'A', 'dest': 'B'},
    {'trigger': 'move_on', 'source': 'B', 'dest': 'C'}
]

def create_machine():
    machine = Machine(model, states=states, transitions=transitions, initial='A', after_state_change='on_enter_state')

def update_machine():
    while True:
        input_state = input("Enter the state to move on: ")
        if input_state == 'move_on':
            model.move_on()

if __name__ == '__main__':
    thread1 = threading.Thread(target=create_machine)
    thread2 = threading.Thread(target=update_machine)

    thread1.start()
    thread2.start()

    thread1.join()
    thread2.join()