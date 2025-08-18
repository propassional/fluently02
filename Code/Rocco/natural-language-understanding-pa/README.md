# Natural Language Understanding

The _nlu_ module provides the natural language understanding based on the [Rasa](https://github.com/RasaHQ/) framework.

## Prerequisites

The module has been tested with the following settings:

- Ubuntu 22.04.4 LTS
- ROS humble

## Run via Docker image

On the PA computer in the lab.
Whithin the /Desktop/natural-language-understanding directory:

```bash
  sudo ./build_docker.sh
```

The node will be able run automatically.

## Shut down 

Press 'Ctrl + C', then 'Ctrl + d', in the same the terminal the model is running.
Then use the command above to re-run. 

## Model selection 
The default model for _pa_ is alredy set in the in config.yml: 
```bash
nlu-20241204-174439-humane-tower.tar.gz
```

## Mantainer

- [Rocco Felici](rocco.felici@supsi.ch)


## Connection with state machine and GUI
- __input__:
On the topic *sm_state* publish a string with the name of the current state in the following format:
TabNode e.g. InstructionsStart, InstructionsReady or PreviewBasicArtecStudioIsOff.
For the complete list of names of states look at ros2_ws/src/nlu/state_machine/sm_dictionary.py.

- __output__:
On the topic *gui_output* you'll find the exact name of the transition or jumper that the operator want to execute.