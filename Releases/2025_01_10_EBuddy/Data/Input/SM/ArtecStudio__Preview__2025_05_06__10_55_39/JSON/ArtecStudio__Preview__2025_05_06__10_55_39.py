import sys
import os
# This does not work: from FLOS import sys_path_to_file
import FLOS

from FLConstants import COMMAND_FOR_EBUDDY

sm_gui_name1 = "Preview"  # SM to which we want to send a message via file
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

def ArtecStudioIsOnDo(message_queue=""):
    message = f"Artec Studio started"
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

def ReadyDoOld(message_queue=""):
    message = f"Let's start Artec Studio preview"
    message_queue.put(message)
    return True

def ScanMenuIsOnDo(message_queue=""):
    message = f"Scan Menu started"
    message_queue.put(message)
    return True

def VerifyCameraSetupBasicDo(message_queue=""):
    image_path = r'D:\Banfi\Github\Fluently\Releases\2024_07_09_EBuddy\Data\Input\App\ArtecStudio\Hints\CameraSetupBasic.png'
    message = f"Camera basic setup"
    message_queue.put(message)
    return image_path # So the image will be displayed in GUI

def VerifyCameraSetupAdvanced1Do(message_queue=""):
    image_path = r'D:\Banfi\Github\Fluently\Releases\2024_07_09_EBuddy\Data\Input\App\ArtecStudio\Hints\CameraSetupAdvanced1.png'
    message = f"Camera advanced setup"
    message_queue.put(message)
    return image_path # So the image will be displayed in GUI

def VerifyCameraSetupAdvanced2Do(message_queue=""):
    image_path = r'D:\Banfi\Github\Fluently\Releases\2024_07_09_EBuddy\Data\Input\App\ArtecStudio\Hints\CameraSetupAdvanced2.png'
    message = f"Camera advanced setup"
    message_queue.put(message)
    return image_path # So the image will be displayed in GUI