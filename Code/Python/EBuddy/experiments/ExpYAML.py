import os
import yaml

from FLConstants import SM_FILES_INPUT, YAML_FILE_NAME

if __name__ == '__main__':
    #yaml_file_name = YAML_FILE_NAME
    yaml_file_name = "SMGeneric.yaml"
    yaml_path = os.path.join(SM_FILES_INPUT, yaml_file_name)
    with open(yaml_path, "r") as yaml_file:
        loaded_data = yaml.safe_load(yaml_file)

    for entry in loaded_data:
        pass