#!/bin/sh

# The program copies the last model from the training module and updates the config file accordingly.

directory='./ros2_ws/src/nlu/nlu'

# it copies all the new models in the directory
sudo cp -rp ../nlu-training/models "$directory"

# writes the name of the last model in the config.yaml file
echo "#nlu config\n---\n# the path to the model" > "$directory"/config.yaml

# Loop through each file in the current directory
for file in "$directory"/models/*; do
    # save the name of the file
    file_name=$(basename "$file")
done

# Add file name to YAML file
cd "$directory"
echo "model: src/nlu/nlu/models/$file_name" >> config.yaml
echo "Model updated."