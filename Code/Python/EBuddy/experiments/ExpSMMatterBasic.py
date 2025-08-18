# Given a state, you can extract its triggers with get_triggers, and fire next transition with getattr(matter, trigger)()

from transitions import Machine

class Matter:
    pass

if __name__ == '__main__':

    matter = Matter()

    # Define the states
    states = ['solid', 'liquid', 'gas', 'plasma']

    # Define the transitions
    transitions = [
        {'trigger': 'melt', 'source': 'solid', 'dest': 'liquid'},
        {'trigger': 'evaporate', 'source': 'liquid', 'dest': 'gas'},
        {'trigger': 'ionize', 'source': 'gas', 'dest': 'plasma'},
        {'trigger': 'deionize', 'source': 'plasma', 'dest': 'gas'},
        {'trigger': 'condense', 'source': 'gas', 'dest': 'liquid'},
        {'trigger': 'freeze', 'source': 'liquid', 'dest': 'solid'}
    ]

    # Initialize the state machine
    machine = Machine(model=matter, states=states, transitions=transitions, initial='solid')

    print("Current state: " + machine.model.state) # Expected "solid"

    # Get the triggers of the current state
    triggers = machine.get_triggers(matter.state)

    # Use the trigger method to initiate the transition
    for i in range(len(triggers)):
        trigger = triggers[i]
        # What does this?
        getattr(matter, trigger)()
        #machine.model.trigger('melt')
        # This will fire the trigger
        machine.model.trigger(trigger)
        print("Current state: " + machine.model.state)

    # Print the current state
    print(matter.state)