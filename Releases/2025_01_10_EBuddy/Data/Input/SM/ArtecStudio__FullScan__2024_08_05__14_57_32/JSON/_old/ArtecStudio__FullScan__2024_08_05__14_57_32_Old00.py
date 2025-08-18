import sys
import os
# This does not work: from FLOS import sys_path_to_file
import FLOS

from D:\Banfi\Github\Fluently\Code\Phyton\knowledge-transfer-fleuntly-main\move_robot.py import FanucRosInterface

from FLConstants import COMMAND_FOR_EBUDDY

sm_gui_name1 = "FullScan"  # SM to which we want to send a message via file
path_app_code = "..\\..\\..\\App\\ArtecStudio\\Code"

# Artec Studio code is not in the sys path, add it
current_module_path = os.path.abspath(__file__)
FLOS.sys_path_append_app_path(current_module_path, path_app_code)

cm = None

# Execute Artec Studio code
try:
    from ArtecStudio import *

    pass
    # ArtecStudioIsOnCheck()
except Exception as error:
    print(f"{__name__}: Error executing some Artec Studio methods")

#############################################################################################
def PauseIsOnDo(message_queue):
    message = "Pause is on"
    message_queue.put(message)
    return True

def PreviewIsOnDo(message_queue):
    # message = "Point the scanner to the object bottom, stay still, start recording"
    message = "Preview is on"
    message_queue.put(message)
    return True

def PreviewIsOnRollback(message_queue):
    message = "Error: Preview is off\nStart the Preview tab"
    message_queue.put(message)
    return True

def ReadyDo(message_queue=""):
    message = f"Let's do a full scan"
    message_queue.put(message)
    return True

def RecordingIsOffDo(message_queue):
    message = "Recording is off"
    message_queue.put(message)
    return True

def RecordingIsOnDo(message_queue):
    message = "Recording is on"
    message_queue.put(message)
    return True

def StartCobotDo(message_queue):
    global cm
    cm = FanucRosInterface()
    message = "Cobot goes to start pos"
    message_queue.put(message)
    return True
    
def StartRecordingDo(message_queue):
    global cm
    cm.goto_positions()
    message = "Cobot does scan pos"
    message_queue.put(message)
    return True