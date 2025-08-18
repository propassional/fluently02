# I have to find out the difference with the other file with similar name, but this one contains less functionality
# This tool extracts all EBuddy intents, to be sent to Rocco for training
# It extracts all triggers contained within all json files used by EBuddy, and stores them into a text file
import fluently
from fluently.FLOS import yaml_load
from fluently.FLConstants  import SM_FILES, YAML_MAIN_PATH, SM_FILES_OUTPUT
import json
from datetime import datetime

def extract_triggers_from_json_files(directory):
    triggers = []
    parsed_files = []
    tab_names = []

    loaded_data = fluently.FLOS.yaml_load()

    # Now `loaded_data` contains the list of dictionaries with all SM paths and names
    no_name_index = 0
    for entry in loaded_data:
        print(f"SM path: {entry['SM_path']}, SM name: {entry['SM_name']}")
        parsed_files.append(entry['SM_path'])
        tab_names.append(entry['SM_name'])
        with open(entry['SM_path'], 'r') as f:
            data = json.load(f)
            if 'transitions' in data and isinstance(data['transitions'], list):
                for transition in data['transitions']:
                    if 'trigger' in transition:
                        triggers.append(transition['trigger'])

    return parsed_files, triggers, tab_names

# Define the directory to search SM
directory = SM_FILES

# Extract triggers and parsed file paths
parsed_files, triggers, tab_names = extract_triggers_from_json_files(directory)

# Remove "Auto" and all triggers starting with "Cmd"
filtered_triggers = [trigger for trigger in triggers if trigger != "Auto" and not trigger.startswith("Cmd")]

# Keep only unique elements
unique_triggers = list(set(filtered_triggers))

# Organize triggers alphabetically
unique_triggers.sort()

# Get the current date
current_date = datetime.now().strftime("%Y_%m_%d")

# Define the output file name
output_file_name =  SM_FILES_OUTPUT + f"\Intents_{current_date}.txt"

# Store the note, parsed file paths, and triggers in the output file
with open(output_file_name, 'w') as file:
    file.write("Searching for EBuddy intents, by scanning all state machine JSON files defined within this YAML file: \n" + YAML_MAIN_PATH + '\n\n')
    file.write("Parsed JSON files:\n")
    for file_path in parsed_files:
        file.write(file_path + '\n')
    file.write("\nI removed the settings intent: since we decided not to command by voice the settings button content, we do not open it by voice\n")
    file.write("\nIntents from EBuddy GUI: \nStartAutopilot \nStopAutopilot \n")
    file.write("\nIntents for tabs switching:\n")
    for item in tab_names:
        file.write(item + "\n")
        print(item)
    file.write("\nIntents from states machine:\n")
    for trigger in unique_triggers:
        file.write(trigger + '\n')

print(f"Parsed JSON files and triggers have been extracted, filtered, sorted, and stored in {output_file_name}")

