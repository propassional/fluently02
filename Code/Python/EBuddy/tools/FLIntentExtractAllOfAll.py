# This tool extracts absolutely ALL EBuddy intents: not really useful
# Intents also may be grouped together using entities
# It extracts all triggers contained within all json files used by EBuddy, and stores them into a text file
# GPT: read all subdirectories in SM_FILES, extract all json files, return all strings associated with the json keyword “trigger” within “transitions” which contains numbers from 0 to n, organize triggers alphabetically, remove triggers “Auto” and all triggers starting with “Cmd”, keep only unique elements, store the complete path of the .json files parsed in the first part of the file, store the triggers in Intents_2024_11_14, and print a note into the text file, explaining in English what does this line: trigger for trigger in triggers if trigger != “Auto” and not trigger.startswith(“Cmd”)

from fluently.FLConstants import SM_FILES
import os
import json
from datetime import datetime

def extract_triggers_from_json_files(directory):
    triggers = []
    parsed_files = []

    # Walk through all subdirectories and files in the given directory
    for root, dirs, files in os.walk(directory):
        for file in files:
            if file.endswith('.json'):
                file_path = os.path.join(root, file)
                parsed_files.append(file_path)
                with open(file_path, 'r') as f:
                    data = json.load(f)
                    if 'transitions' in data and isinstance(data['transitions'], list):
                        for transition in data['transitions']:
                            if 'trigger' in transition:
                                triggers.append(transition['trigger'])

    return parsed_files, triggers

# Define the directory to search SM
directory = SM_FILES

# Extract triggers and parsed file paths
parsed_files, triggers = extract_triggers_from_json_files(directory)

# Remove "Auto" and all triggers starting with "Cmd"
filtered_triggers = [trigger for trigger in triggers if trigger != "Auto" and not trigger.startswith("Cmd")]

# Keep only unique elements
unique_triggers = list(set(filtered_triggers))

# Organize triggers alphabetically
unique_triggers.sort()

# Get the current date
current_date = datetime.now().strftime("%Y_%m_%d")

# Define the output file name
output_file_name = r"D:\Banfi\Github\Fluently\Releases\2025_01_10_EBuddy\Data\Output\Intents\_Released\2025_04_28" + f"\Intents_{current_date}.txt"

# Store the note, parsed file paths, and triggers in the output file
with open(output_file_name, 'w') as f:
    f.write("Searching for state machine intents, by scanning all JSON files within this directory: \n" + directory+ '\n\n')
    #file.write("Note:\n")
    #file.write('The line "trigger for trigger in triggers if trigger != "Auto" and not trigger.startswith("Cmd")" filters out the trigger "Auto" and any triggers that start with "Cmd".\n\n')
    f.write("Parsed JSON files:\n")
    for file_path in parsed_files:
        f.write(file_path + '\n')
    f.write("\nIntents from EBuddy GUI:\nGotoTabLeft \nGotoTabRight \nStartAutopilot \nStopAutopilot\n")
    f.write("\nIntents from states machine:\n")
    for trigger in unique_triggers:
        f.write(trigger + '\n')

print(f"Parsed JSON files and triggers have been extracted, filtered, sorted, and stored in {output_file_name}")
