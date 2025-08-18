import sys
import os
# This does not work: from FLOS import sys_path_to_file
import FLOS

from FLConstants import COMMAND_FOR_EBUDDY

sm_gui_name1 = "Registration"  # SM to which we want to send a message via file
path_app_code = "..\\..\\..\\App\\ArtecStudio\\Code"

# Artec Studio code is not in the sys path, add it
current_module_path = os.path.abspath(__file__)
FLOS.sys_path_append_app_path(current_module_path, path_app_code)

# Execute Artec Studio code
try:
    from ArtecStudio import *
    pass
    #ArtecStudioIsOnCheck()
except Exception as error:
    print(f"{__name__}: Error executing some Artec Studio methods")

def AutoMenuIsOnDo(message_queue=""):
    message = f"Tool Menu set on Auto"
    message_queue.put(message)
    return True

# Auto-switch off autopilot
def EndDo(message_queue):
    command = "" # Command we want to send
    autopilot_mode = False # How the command has to be executed
    path_sink = os.path.join(COMMAND_FOR_EBUDDY, sm_gui_name1 + "_" + command + ".txt")
    try:
        with open(path_sink, "w") as sink_file:
            sink_file.write(sm_gui_name1 + r"\n" + command + r"\n" + str(autopilot_mode))
    except FileNotFoundError:
        message = f"{__name__}: GUI_command_send: file '{command}' not found in source directory"
        message_queue.put(message)
        return False
    except Exception as e:
        message = f"{__name__}: GUI_command_send: an error occurred while copying '{command}': {str(e)}"
        message_queue.put(message)
        return False
    #message = f"EndDo sent command \"{command}\" to SM {sm_gui_name1}, autopilot {autopilot_mode}"
    message = f"End sent autopilot to {autopilot_mode}"
    message_queue.put(message)
    return True

def GlobalRegistrationEnteredDo(message_queue=""):
    message = f"Global Registration Entered"
    message_queue.put(message)
    return True

def GlobRegIsDoneDo(message_queue=""):
    message = f"Global Registration Calc Done"
    message_queue.put(message)
    return True
    
def ReadyDoOld(message_queue=""):
    message = f"Let's do Global Registration"
    message_queue.put(message)
    return True

def FusionIsDoneDo(message_queue=""):
    message = f"Fusion Calc Done"
    message_queue.put(message)
    return True

def ScanMenuIsOnDo(message_queue=""):
    message = f"Scan Menu started"
    message_queue.put(message)
    return True

def ToolsMenuIsOnDo(message_queue=""):
    message = f"Tools Menu started"
    message_queue.put(message)
    return True