import time
SLEEP_DO = True
SLEEP_TIME = 2

def CobotDo(message_queue=""):
    image_path = r'D:\Banfi\Github\Fluently\Releases\2024_07_09_EBuddy\Data\Input\App\ArtecStudio\Hints\CobotSmallSmall.png'
    message = f"Cobot with scanner"
    message_queue.put(message)
    if SLEEP_DO:
        time.sleep(SLEEP_TIME)
    return image_path  # So the image will be displayed in GUI    

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

def HolderDo(message_queue=""):
    image_path = r'D:\Banfi\Github\Fluently\Releases\2024_07_09_EBuddy\Data\Input\App\ArtecStudio\Hints\HolderSmallSmall.png'
    message = f"Holder for the impeller"
    message_queue.put(message)
    if SLEEP_DO:
        time.sleep(SLEEP_TIME)
    return image_path  # So the image will be displayed in GUI

def ImpellerDo(message_queue=""):
    image_path = r'D:\Banfi\Github\Fluently\Releases\2024_07_09_EBuddy\Data\Input\App\ArtecStudio\Hints\ImpellerSmallSmall.png'
    message = f"Impeller to scan"
    message_queue.put(message)
    if SLEEP_DO:
        time.sleep(SLEEP_TIME)
    return image_path  # So the image will be displayed in GUI
    
def ImpellerInstalledDo(message_queue=""):
    image_path = r'D:\Banfi\Github\Fluently\Releases\2024_07_09_EBuddy\Data\Input\App\ArtecStudio\Hints\ImpellerInstalled.png'
    message = f"Impeller installed"
    message_queue.put(message)
    if SLEEP_DO:
        time.sleep(SLEEP_TIME)
    return image_path  # So the image will be displayed in GUI    

def ScannerDo(message_queue=""):
    image_path = r'D:\Banfi\Github\Fluently\Releases\2024_07_09_EBuddy\Data\Input\App\ArtecStudio\Hints\ScannerSmallSmall.png'
    message = f"Scanner sensors"
    message_queue.put(message)
    if SLEEP_DO:
        time.sleep(SLEEP_TIME)
    return image_path  # So the image will be displayed in GUI 

