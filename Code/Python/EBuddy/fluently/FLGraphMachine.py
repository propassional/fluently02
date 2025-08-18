# Here you find the most relevant pytransitions docs
# Scroll this page down for documentation: https://github.com/pytransitions/transitions?tab=readme-ov-file
# All "SM*.py" files contain specific pytransitions problematics and docs
import copy
# GraphMachineBasic is the class that fits our specs at best, featuring:
#   create an empty state machine or graphmachine, add_states, add_transitions, update states,
#   load from file, save to file, create graphs
# I have created a SM similar the one of the GA
# Examples are easy, but to obtain some generalisation with complete separation between code and SM_JSON_data needs some work
# Removing a state is not included in the lib, since in some cases it may be not trivial
# Warning: you can add the same states and transitions multiple times, but I don't know the behavior
# How to implement a callback method on the event after_state_change: self.machine = GraphMachine(model=model, initial='Start', after_state_change='on_enter_each_state')

# The state "Start" and its transition "AutoStart" enable to model the use case when the first SM transition is fired,
# but the first state can not be maintained active since it is not "true", and the SM needs to rollback to a coerent generic "Start" state, which is always valid
# The AutoStart transition allows the class to perform a default or automatic transition, which allows to load all available commands from the current state,
# which need to be displayed as hints or available commands

# COMMAND_PREFIX: Why all my transitions have to start with Cmd? Since pytransitions inserts automatically a bunch of other potential transitions n to n
# such as to_ / may_ / is_ that I have to filter out
# For example to the state "MovementFinishedIsActive" this transition is added "is_MovementFinishedIsActive"
# Triggers: all potential transitions, triggers_available are triggers that can be used in the current state

# Callbacks:
# https://github.com/pytransitions/transitions?tab=readme-ov-file#callback-execution-order

# GPT test: write code for implementing a state machine based on pytransitions with three states (A, B, C),
# three transitions (AtoB, BtoC, CtoA), and three methods that enable me to activate or disactivate all three transitions.
# Traverse all states of the state machine according to the transition_semaphores of the transitions.
# As default mode, all transitions are True.
# When transition AtoB becomes False, the transition from A to B becomes impossible, and the state machine stays in state A

# Layouts supported:
# Brief description of each of the layouts supported by Graphviz: https://graphviz.org/docs/layouts/
# dot: This is the default layout engine for directed graphs. It produces hierarchical or layered drawings of directed graphs. The layout algorithm aims edges in the same direction (top to bottom, or left to right) and then attempts to avoid edge crossings and reduce edge length 12.
# neato: This layout engine uses a spring model to position the nodes of the graph. It is useful for undirected graphs and can produce more aesthetically pleasing layouts than dot for certain types of graphs 1.
# fdp: This layout engine is similar to neato, but it uses a more aggressive spring model to position the nodes of the graph. It is useful for larger graphs and can produce more compact layouts than neato 1.
# sfdp: This is a scalable version of fdp that can handle larger graphs. It uses a multilevel approach to layout the graph, starting with a coarse layout and refining it at each level 1.
# twopi: This layout engine produces radial layouts of graphs. It places the root node at the center of the drawing and arranges the other nodes in concentric circles around it 1.
# circo: This layout engine produces circular layouts of graphs. It places the root node at the center of the drawing and arranges the other nodes in concentric circles around it 1.
# osage: This layout engine is used to draw clustered graphs. It uses a combination of force-directed and hierarchical layout techniques to position the nodes of the graph 1.
# patchwork: This layout engine is used to draw maps of clustered graphs using a squarified treemap layout. It is useful for visualizing large hierarchies of SM_JSON_data 1.
# nop and nop1: These are both used to pretty-print DOT graph files. They are equivalent to each other 1.
# nop2: This is also used to pretty-print DOT graph files, but it assumes that the positions of the nodes are already known 1.

import os
import pickle
from transitions.extensions import GraphMachine
import inspect
import builtins

import FLOS
from FLBaseClass import FLBaseClass
from FLConstants import COMMAND_FOR_EBUDDY, ERROR
from FLGUISM import FLGUISM
from FLModel import FLModel

class FLGraphMachine(FLBaseClass):

    def __init__(self, name, code_path, logger):
        super().__init__(logger)
        self.name = name
        self.code_path = code_path
        self.log_message_last = ""
        # Avoid initial='', since this reports strange lib problems when you add the SECOND state (the first state is ok!)
        # rankdir='TB' does not seem to be supported, but it can be changed with graph = gm_default.machine.get_graph(), graph.graph_attr['rankdir'] = 'TB'
        # layout='dot' is not supported, try a different version of pygraphviz
        # When you want to use graphviz explicitly: machine = GraphMachine(model=m, use_pygraphviz=False, ...)
        model = FLModel(logger, name, code_path)
        # self.machine = GraphMachine(model=model, initial='Start') # Works, but it has no callbacks
        # self.machine = GraphMachine(model=model, initial='Start', on_enter='on_enter_state') # It does not work
        self.machine = GraphMachine(model=model, initial='Start') # No callback is defined now, they are defined at load time

    def get_terminal_states(self):



        incoming_transitions = {state: 0 for state in self.machine.states}
        all_transitions = self.machine.transitions
        for from_state, transitions in self.machine.transitions.items():
            for to_state, _ in transitions:
                incoming_transitions[to_state] += 1

        no_incoming_states = [state for state, count in incoming_transitions.items() if count == 0]


        all_states = self.machine.states
        # Otherwise error when doing terminal_states.pop(state), since it is a shallow copy
        terminal_states = copy.deepcopy(self.machine.states)
        for state in all_states:
            for transition in self.machine.get_transitions():
                if transition.dest == state:
                    if (state in terminal_states # state not yet removed
                            and transition.source != transition.dest): # it is not an autotransition
                        terminal_states.pop(state)
        return terminal_states

    def conditions_create(self):
        # terminal_states = self.get_terminal_states()
        # all_transitions = self.machine.get_transitions()

        with open(r"D:\transitions.txt", "w") as file:
            transition_dict = {}
            # A bounce state is a state (an "isolated" state) which has no transitions going into or coming out of the state
            state_is_not_a_bounce_state = False
            # states_bounce starts as identical subset of self.machine.states, and at the end it contains all states that have no transitions to any other states
            self.machine.model.states_bounce = copy.deepcopy(self.machine.states)
            for state in self.machine.states: # state is a string, but should be a state type
                # Here we get the list of all transitions starting from "state"
                for trigger in self.machine.get_triggers(state): # trigger is a string
                    file.write(f"state {state}, trigger {trigger} \n")
                    if not trigger.__contains__('to_'):
                        # We found a "real" transition leading to the state, thus the state from where trigger starts is surely not a jump state
                        # Terminal states can not be distinguished from bounce states
                        transition_dict[trigger] = False
                        state_is_not_a_bounce_state = True
                # A jump state does not contain no 'to_' transitions to it
                if state_is_not_a_bounce_state:
                    # state is not a jump state, thus remove it from the list
                    self.machine.model.states_bounce.pop(state)
                    state_is_not_a_bounce_state = False
            # This method returns also self.machine.model.states_bounce, which is a list of all states that do not have an "exit"
            return transition_dict

    def conditions_create_old2(self):
        transition_dict = {}
        state_is_not_a_jump_state = False
        # states_jump is a subset of self.machine.states, and contains all states that have no transitions to any other states
        self.machine.model.states_jump = copy.deepcopy(self.machine.states)
        for state in self.machine.states:
            for trigger in self.machine.get_triggers(state):
                if not trigger.__contains__('to_'):
                    transition_dict[trigger] = False
                    state_is_not_a_jump_state = True
            if state_is_not_a_jump_state:
                self.machine.model.states_jump.pop(state)
                state_is_not_a_jump_state = False
        return transition_dict

    def conditions_create_old1(self):
        conditions = {}
        for transition in self.machine.transitions:
            trigger = transition['trigger']
            conditions[trigger] = True  # You can modify this line to set transition_semaphores dynamically
        return conditions

    def macro_add(self, macro_state_name, state_previous):
        state_do = macro_state_name + "Do"
        state_is = macro_state_name + "IsOn"
        self.machine.add_states(state_do)
        self.machine.add_states(state_is)
        transition_done = macro_state_name + 'Done'
        self.machine.add_transition("Cmd" + macro_state_name, state_previous, state_do)
        self.machine.add_transition(transition_done, state_previous, state_is)
        self.machine.add_transition("!" + transition_done, state_do, state_previous)
        self.machine.add_transition(transition_done, state_do, state_is)

    def on_enter_state(self):
        if len(self.state_history) >= 2:
            self.transitions[self.sm_gui.machine.state] = self.state_history[-2]
        self.state_history.append(self.sm_gui.machine.state)

    def print(self, *args, **kwargs):
        caller_frame = inspect.stack()[1]
        caller_info = inspect.getframeinfo(caller_frame[0])
        log_message_no_caller = [f"{arg}" for arg in args]
        log_message = [f"{caller_info.function}() - {arg}" for arg in args]

        if (log_message_no_caller != self.log_message_last):
            # print to log file
            self.log_file.write(*log_message)
            # Standard print call
            builtins.print(*log_message, **kwargs)
            self.log_message_last = log_message_no_caller

    def states_and_transitions_add(self):
        # Checkout add_ordered_transitions(): you just define the states, then the transitions are added automatically in an ordered way

        # Add states: this code is apparently redundant with machine.add_transition,
        # but if you do not define it, you will get a crash in get state: State 'ArtecStudioIsOff' is not a registered state.
        self.machine.add_states(['Start']) # State is needed so that when SM is initialised, this state is hardcoded to be the first state
        self.machine.add_states(['ArtecStudioOffIsActive'])
        self.machine.add_states(['ArtecStudioIsActive'])
        self.machine.add_states(['ScanMenu1IsActive'])
        self.machine.add_states(['ScanMenu2IsActive'])
        self.machine.add_states(['PreviewIsActive'])
        self.machine.add_states(['RecordingIsActive'])
        self.machine.add_states(['PauseIsActive'])
        self.machine.add_states(['RecordingOffIsActive'])
        self.machine.add_states(['SaveFile1IsActive'])
        self.machine.add_states(['SaveFile2IsActive'])
        self.machine.add_states(['ShutdownIsActive'])
        self.machine.add_states(['AlldownIsActive'])

        #self.machine.add_states(['PCIsOff'])
        #self.machine.add_states(['ProjectSaved'])
        #self.machine.add_states(['OpenToolsIsActive'])

        # Add transitions
        # IMPORTANT: if a state is missing, it will be added
        #self.machine.add_transition('AutoStart', 'Start', 'ArtecStudioOffIsActive', transition_semaphores='AutoStart')
        self.machine.add_transition('CmdAutoStart', 'Start', 'ArtecStudioOffIsActive')
        self.machine.add_transition('CmdStartArtecStudio', 'ArtecStudioOffIsActive', 'ArtecStudioIsActive')
        self.machine.add_transition('CmdStartScanMenu', 'ArtecStudioIsActive', 'ScanMenu1IsActive')
        self.machine.add_transition('CmdStartPreview', 'ScanMenu1IsActive', 'PreviewIsActive')
        self.machine.add_transition('CmdStartRecording', 'PreviewIsActive', 'RecordingIsActive')

        #self.machine.add_transition('CmdStartPause', 'RecordingIsActive', 'PauseIsActive')
        self.machine.add_transition('CmdStartScanMenu', 'RecordingIsActive', 'ScanMenu2IsActive')
        self.machine.add_transition('CmdStartPause', 'ScanMenu2IsActive', 'PauseIsActive')

        self.machine.add_transition('CmdStopRecording', 'PauseIsActive', 'RecordingOffIsActive')
        self.machine.add_transition('CmdStopRecording', 'PreviewIsActive', 'RecordingOffIsActive')
        self.machine.add_transition('CmdStartPreview', 'RecordingOffIsActive', 'PreviewIsActive')
        self.machine.add_transition('CmdSaveFile1', 'RecordingOffIsActive', 'SaveFile1IsActive')
        self.machine.add_transition('CmdSaveFile2', 'SaveFile1IsActive', 'SaveFile2IsActive')
        self.machine.add_transition('CmdStopArtecStudio', 'SaveFile2IsActive', 'ArtecStudioOffIsActive')
        #self.machine.add_transition('CmdShutdown', 'ArtecStudioOffIsActive', 'ShutdownIsActive')
        self.machine.add_transition('CmdAlldown', 'ShutdownIsActive', 'AlldownIsActive')
        self.machine.add_transition('CmdAlldown', 'AlldownIsActive', 'ShutdownIsActive')
        # Unfortunately, this transition can not be defined, otherwise the state machine becomes unreadable
        # self.machine.add_transition('CmdShutdown', '*', 'Start')

        #self.machine.add_transition('CmdStopPC', 'ArtecStudioIsOff', 'PCIsOff', transition_semaphores= 'CmdStopPC')

        # self.machine.add_transition('CmdScannerStart', 'ArtecStudioIsOpen', 'PreviewIsActive')
        # self.machine.add_transition('!ArtecStudioIsOpen', 'ArtecStudioIsOpen', 'ArtecStudioIsOff')
        # self.machine.add_transition('CmdScanningFunctionality', 'PreviewIsActive', 'Recording')
        # self.machine.add_transition('CmdScanningProcedure', 'Recording', 'Scanning')
        # self.machine.add_transition('CmdScanPause', 'Scanning', 'ScanPaused')
        # self.machine.add_transition('CmdScanStop', 'ScanPaused', 'ScanStopped')
        # self.machine.add_transition('CmdScanSave', 'ScanStopped', 'ProjectSaved')
        # self.machine.add_transition('CmdScanClose', 'ProjectSaved', 'ArtecStudioIsOpen')
        # self.machine.add_transition('CmdOpenTools', 'ArtecStudioIsOpen', 'OpenTools')
        # self.machine.add_transition('ToolsIsOpen', 'OpenTools', 'ToolsIsOpen')
        #self.machine.add_transition('CmdCloseArtecStudio', '*', 'ArtecStudioIsOff')

        self.machine.model.conditions, self.machine.model.jump_states = self.conditions_create()

    def states_and_transitions_add_macro(self):
        # Traverse SM states
        # xxx self.machine.states
        # Is the state called IsActive?
        # Read its previous state: ToDeDone more than one?
        self.macro_add("ScannerStart", "Start")
        self.macro_add("PreviewStart", "ScannerStartIsOn")
        self.macro_add("RecordingStart", "PreviewStartIsOn")
        self.macro_add("ScanningStart", "RecordingStartIsOn")
        self.macro_add("ScanningPause", "ScanningStartIsOn")
        self.macro_add("ScanningStop", "ScanningPauseIsOn")
        self.macro_add("ScanningSave", "ScanningStopIsOn")
    #self.add_smacro("ScanningSave", "ScanningSaveIsOn")

    def SM_save(self, sm_file_name=''):
        if sm_file_name == '':
            sm_file_name = 'SMDefault.pickle'
        with (open(sm_file_name, 'wb') as sm):
            try:
                pickle.dump(self.machine, sm) # TypeError: cannot pickle '_thread.lock' object
            except Exception as e:
                self.print(f"SM_save: an error occurred: {str(e)}", ERROR)

    def transition_condition_set(self, transition_name, condition_boolean):
        # Set the condition for the specified transition
        #if transition_name in self.machine.model.conditions:
            #self.machine.model.conditions[transition_name] = condition_boolean
        if transition_name in self.machine.model.transition_semaphores:
            self.machine.model.transition_semaphores[transition_name] = condition_boolean

if __name__ == '__main__':
    gm_default = FLGraphMachine()

    # An error occurred: cannot pickle '_thread.lock' object
    gm_default.SM_save('d:\\temp\\my_pickle.pickle')

    pass