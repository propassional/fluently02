# This alternative is focused on using the callback before_state_change,
# defined within the model and registered when machine is created, with and without parameters
# and an interesting way (but this part does not work now) of calling getattr for retrieving the condition function for evaluating the transition boolean value
# I prefered Ver3, that does not need callbacks for working
# https://github.com/pytransitions/transitions?tab=readme-ov-file#callback-execution-order

from transitions import Machine

class Model(object):
    def disappear(self):
        print("where'd all the liquid go?")
    def callback_before_state_change1(self):
        print("HISSSSSSSSSSSSSSSS")
    def callback_before_state_change2(machine, *args, source, dest, **kwargs):
        transition = kwargs.get('transition')
        if transition:
            condition_function = getattr(machine, f'condition_{transition.source}_{transition.dest}', None)
            if condition_function:
                if not condition_function():
                    # If the condition is not met, cancel the transition
                    kwargs['cancelled'] = True

if __name__ == '__main__':

    # Define your states
    states = ['A', 'B', 'C']

    # Define custom transition_semaphores for transitions
    def condition_AB():
        # Add your condition logic here
        return True  # Change to False if you don't want to transition from A to B

    def condition_BC():
        # Add your condition logic here
        return True  # Change to False if you don't want to transition from B to C

    #############################################################################################

    my_model=Model()

    machine = Machine(
        model=my_model,
        states=states,
        initial='A',
        ordered_transitions=True,
        ignore_invalid_triggers=True,
        auto_transitions=False,
        before_state_change='callback_before_state_change1'  # Define callback for custom transition_semaphores
        #before_state_change='callback_before_state_change2'  # Define callback for custom transition_semaphores
    )

    # Attach the callback function to the machine
    #machine.before_state_change = before_state_change

    try:
        while True:
            print("Current state: " + my_model.state)
            my_model.next_state()
            #time.sleep(1)
    except KeyboardInterrupt:
        # You can add any cleanup logic here
        pass
