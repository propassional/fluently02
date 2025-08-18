import time
SLEEP_DO = True
SLEEP_TIME = 3

# Auto-switch off autopilot
def EndDo(message_queue):
    import os.path
    from FLConstants import COMMAND_FOR_EBUDDY
    sm_gui_name1 = "Instructions"

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
    # message = f"EndDo sent command \"{command}\" to SM {sm_gui_name1}, autopilot {autopilot_mode}"
    message = f"EndDo resets autopilot"
    message_queue.put(message)
    return True

def MarkersAppliedDo(message_queue=""):
    image_path = r'D:\Banfi\Github\Fluently\Releases\2024_07_09_EBuddy\Data\Input\App\ArtecStudio\Hints\ApplyMarkers500x1000.jpg'
    message = f"Apply markers onto the object, distance 5 m"
    message_queue.put(message)
    if SLEEP_DO:
        time.sleep(SLEEP_TIME)
    return image_path  # So the image will be displayed in GUI

def ReadyDoOld(message_queue):
    message = "Read generic instructions"
    message_queue.put(message)
    return True
    
def RingBelowScannedDo(message_queue=""):
    image_path = r'D:\Banfi\Github\Fluently\Releases\2024_07_09_EBuddy\Data\Input\App\ArtecStudio\Hints\ScanRingBelow500x1000.jpg'
    message = f"Keep scanner at fix position 12 cm, rotate the rototilt"
    message_queue.put(message)
    if SLEEP_DO:
        time.sleep(SLEEP_TIME)
    return image_path  # So the image will be displayed in GUI

def RingIntermediateScannedDo(message_queue=""):
    image_path = r'D:\Banfi\Github\Fluently\Releases\2024_07_09_EBuddy\Data\Input\App\ArtecStudio\Hints\ScanRingIntermediate500x1000.jpg'
    message = f"Keep scanner at fix position 12 cm, rotate the rototilt"
    message_queue.put(message)
    if SLEEP_DO:
        time.sleep(SLEEP_TIME)
    return image_path  # So the image will be displayed in GUI

def RingAboveScannedDo(message_queue=""):
    image_path = r'D:\Banfi\Github\Fluently\Releases\2024_07_09_EBuddy\Data\Input\App\ArtecStudio\Hints\ScanRingAbove500x1000.jpg'
    message = f"Keep scanner at fix position 12 cm, rotate the rototilt"
    message_queue.put(message)
    if SLEEP_DO:
        time.sleep(SLEEP_TIME)
    return image_path  # So the image will be displayed in GUI
    
def RototiltInstalledDo(message_queue=""):
    image_path = r'D:\Banfi\Github\Fluently\Releases\2024_07_09_EBuddy\Data\Input\App\ArtecStudio\Hints\GetARototilt500x1000.jpg'
    message = f"Install a rototilt like this one, which you can rotate by hand"
    message_queue.put(message)
    if SLEEP_DO:
        time.sleep(SLEEP_TIME)
    return image_path  # So the image will be displayed in GUI

