from transitions.extensions import GraphMachine

class NarcolepticSuperhero(object):
    states = ['asleep', 'hanging out'] #, 'hungry', 'sweaty', 'saving the world']
    def __init__(self, name):
        self.machine = GraphMachine(model=self, states=NarcolepticSuperhero.states, initial='asleep')
        #self.machine.add_transition('wake_up', 'asleep', 'hanging out', before='always_true')
        self.machine.add_transition('wake_up', 'asleep', 'hanging out')

    def to_state_call(self, state_name):
        # The getattr() function looks for an attribute named "to_state_name" in the self.machine.model object
        # If such an attribute exists, it returns the corresponding method
        transition_method = getattr(self.machine.model, f"to_{state_name}")
        transition_method()

    # GPT says this is needed in order to force the state with "to_asleep", but it is not true
    # def always_true(self, *args, **kwargs):
    #     return True

# Rollback from hanging out to asleep in two different ways
force_state = False # True False

# Create an instance of our superhero
batman = NarcolepticSuperhero("Batman")

# Transition from asleep to hanging out
batman.wake_up()
print(f"State is now {batman.state}")
batman.get_graph().draw("D:\superhero_wake_up.png", prog='dot')

# Rollback from hanging out to asleep in two different ways
if force_state:
    # Force the state
    batman.machine.model.state = "asleep"
    print(f"State is now {batman.state}")
    # !! Error: now there is a mismatch between the current state (Batman is now asleep) and the graph (state is "hanging out") !!
    batman.get_graph().draw("D:\superhero_asleep_using_forced_bad_result", prog='dot')
else:
    # Transition, using the method always_true
    # batman.to_asleep() # Works, alternative1
    # batman.machine.model.to_asleep() # Works, alternative2
    # Generic version, without hard-coded method call
    batman.to_state_call("asleep")
    print(f"State is now {batman.state}")
    batman.get_graph().draw("D:\superhero_asleep_using_to_result_correct", prog='dot')

