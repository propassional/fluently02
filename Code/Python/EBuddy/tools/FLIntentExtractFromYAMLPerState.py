# This tool extracts the EBuddy intents, to be sent to Rocco for training
# It extracts all triggers contained within all json files used by EBuddy, and stores them into a text file,
# whose path is shown at the end of the script
# It adds all intents that come from EBuddy GUI + Help system

import json
from datetime import datetime

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
print("\n".join(sys.path))
import fluently # Neglect this error, online it will work
print(fluently) # Expected: <module 'fluently' from '/path/to/your/project/fluently/__init__.py'>

import fluently.FLOS
from fluently.FLConstants import SM_FILES_OUTPUT, INTENTS_DIR

# We removed the settings intent: since we decided not to command by voice the settings button content, for consistency we also do not open it by voice\n")

# Release 1.2
# GUI_commands = [ "GoToTabLeft", "GoToTabRight", "AutopilotOn", "AutopilotOff" ]
tab_switch = [ "GoToTabLeft", "GoToTabRight" ]
GUI_buttons = [ "AutopilotOn", "AutopilotOff"]
GUI_commands = tab_switch + GUI_buttons

# Release 1.3a
info_navigation = [ "PreviousSlide", "NextSlide", "Okay" ] # LastSlide is better than Previous slide, but Emilia does not react to it now
info_frequentation = [ "SkipSlide", "DetailedSlide" ]
info_quality = [ "ThumbUp", "ThumbDown", "ILike", "IDislike" ]
GUI_commands = tab_switch + GUI_buttons + info_navigation + info_frequentation + info_quality

# Release 1.3b
mouse_move = [ "MoveMouseRight", "MoveMouseLeft", "MoveMouseUp", "MoveMouseDown" ]
mouse_press = [ "PressMouseLeft", "PressMouseMiddle", "PressMouseRight" ]
mouse_release = [ "ReleaseMouseLeft", "ReleaseMouseMiddle", "ReleaseMouseRight" ]
mouse_zoom = [ "ZoomIn", "ZoomOut" ]
GUI_commands = tab_switch + GUI_buttons + info_navigation + info_frequentation + info_quality + mouse_move + mouse_press + mouse_release + mouse_zoom

# Release 1.4
#browser_scroll = ["HelpScrollDown", "HelpScrollUp"]
#browser_zoom = ["HelpZoomIn", "HelpZoomOut"]
#video_navigation = ["HelpPlay", "HelpStop", "HelpPause"]


##########################################################################################################

# Good to know that a SM json file can be coded like this
json_test_not_used= {
    "states": [
        "Start",
        "Ready",
        "ArtecStudioIsOn",
    ],
    "transitions": [
        {
            "trigger": "Next",
            "source": "Start",
            "dest": "Ready"
        },
        {
            "trigger": "CheckArtecStudioIsOn",
            "source": "Ready",
            "dest": "ArtecStudioIsOn"
        }
    ],
    "initial": "Start"
}

def extract_states_isolated(data_json):
    # Extract states that are not connected to any transition
    states_isolated = set(data_json["states"])

    for transition in data_json["transitions"]:
        if transition["source"] in states_isolated:
            states_isolated.remove(transition["source"])
        if transition["dest"] in states_isolated:
            states_isolated.remove(transition["dest"])

    # Convert the set to a list
    states_isolated = list(states_isolated)

    print(f"States not connected to any transition: {states_isolated}")
    return states_isolated

# Function to format transitions
def format_transitions(data):
    transitions_dict = {state: [] for state in data["states"]}
    for transition in data["transitions"]:
        source = transition["source"]
        trigger = transition["trigger"]
        if trigger not in transitions_dict[source]:
            transitions_dict[source].append(trigger)
    return {state: triggers if len(triggers) > 1 else (triggers[0] if triggers else None) for state, triggers in transitions_dict.items()}

# Function to add words to strings or lists
def add_words_to_dict(dictionary, words):
    for key in dictionary:
        if isinstance(dictionary[key], str):
            # If the value is a string, add words to the string
            for word in words:
                dictionary[key] += " " + word
        elif isinstance(dictionary[key], list):
            # If the value is a list, add words to each element in the list
            dictionary[key] = [item + " " + " ".join(words) for item in dictionary[key]]
        elif dictionary[key] is None:
            # If the value is None, initialize it as an empty string and add words
            dictionary[key] = " ".join(words)
    return dictionary

# Function to transform the dictionary
def transform_dict_to_strings(dictionary):
    transformed_dict = {}
    for key, value in dictionary.items():
        if isinstance(value, list):
            # Join the list elements into a single string
            transformed_dict[key] = ", ".join(value)
        else:
            # If the value is not a list, keep it as is
            transformed_dict[key] = value
    return transformed_dict

if __name__ == '__main__':

    wrapped_data = {}
    triggers = []
    json_files_path_list = []
    tab_names = []

    # This file contains all SM data
    loaded_data = fluently.FLOS.yaml_load()

    # Now `loaded_data` contains the list of dictionaries with all SM paths and names
    no_name_index = 0
    for entry in loaded_data:
        print(f"SM path: {entry['SM_path']}, SM name: {entry['SM_name']}")
        json_files_path_list.append(entry['SM_path'])
        tab_names.append(entry['SM_name'])

    for json_file_path, tab_name in zip(json_files_path_list, tab_names):

        # Read the JSON SM_JSON_data from the file
        with open(json_file_path, 'r') as file_json:
            data_json = json.load(file_json)
            states_isolated = extract_states_isolated(data_json)

            # Initialize a dictionary to store the transitions
            transitions_dict = {state: [] for state in data_json["states"]}

            # Populate the dictionary with transitions
            for transition in data_json["transitions"]:
                source = transition["source"]
                trigger = transition["trigger"]
                if trigger not in transitions_dict[source]:
                    transitions_dict[source].append(trigger)

            # Format the dictionary as required
            formatted_transitions = {state: triggers if len(triggers) > 1 else (triggers[0] if triggers else None) for
                                     state, triggers in transitions_dict.items()}

            print("\nBefore adding isolated states: \n")
            print(formatted_transitions)

            formatted_transitions_no_lists = transform_dict_to_strings(formatted_transitions)

            # Add to the intents also the isolated states & GUI commands
            jump_commands = states_isolated + tab_names + GUI_commands
            # Since we do not jump to the tab where we already are, remove it from the list
            jump_commands.remove(tab_name)
            for key in formatted_transitions_no_lists:
                if formatted_transitions_no_lists[key] is None:
                    formatted_transitions_no_lists[key] = ""
                for word in jump_commands:
                    if formatted_transitions_no_lists[key] == "":
                        formatted_transitions_no_lists[key] = word
                    else:
                        formatted_transitions_no_lists[key] += ", " + word

            print("\nAfter adding isolated states: \n")
            print(formatted_transitions_no_lists)

            # Remove isolated states
            for key in states_isolated:
                if key in formatted_transitions_no_lists:
                    del formatted_transitions_no_lists[key]

            # Wrap the SM_JSON_data with "TabName"
            wrapped_data[tab_name] = formatted_transitions_no_lists

            # wrapped_data = {
            #     tab_name: formatted_transitions
            # }

            # Print the resulting JSON
            print(json.dumps(wrapped_data, indent=4))

    # wrapped_data = {
    #     tab_name: formatted_transitions
    # }
    # Get the current date

    current_date_time = datetime.now().strftime("%Y_%m_%d__%H_%M_%S")

    # Define the output file name
    file_name_intents_JSON = f"\Intents_{current_date_time}.json"
    file_output_JSON_path = SM_FILES_OUTPUT + INTENTS_DIR +  file_name_intents_JSON

    # Save the wrapped SM_JSON_data as JSON
    with open(file_output_JSON_path, 'w') as json_file:
        json.dump(wrapped_data, json_file, indent=4)

    print(f"Data has been successfully saved to {file_output_JSON_path}")

    file_name_intents_readme =  f"\Intents_{current_date_time}_readme.txt"
    file_output_readme_path = SM_FILES_OUTPUT + INTENTS_DIR + file_name_intents_readme

    # Save the wrapped SM_JSON_data as JSON
    with open(file_output_readme_path, 'w') as json_file:
        json_file.write("The file " + file_name_intents_JSON +
        " has been created automatically, using the FLIntentExtractFromYAMLPerState.py module \n")
        json_file.write("It contains all possible user interaction alternatives, for each SM state\n")
        json_file.write("\nThe user can select following alternatives:")
        json_file.write("\n1) Select within states or jump states of the current state machine")
        json_file.write("\n2) Select other tabs, containing other state machines: " + str(tab_names))
        json_file.write("\n3) Select EBuddy GUI buttons or tab change commands: " + str(GUI_commands) )
        json_file.write("\n\nNote: I removed the settings intent: since we decided not to command by voice the settings button contents, for consistency we do not open it by voice\n")

    print(f"Data has been successfully saved to {file_output_JSON_path}")