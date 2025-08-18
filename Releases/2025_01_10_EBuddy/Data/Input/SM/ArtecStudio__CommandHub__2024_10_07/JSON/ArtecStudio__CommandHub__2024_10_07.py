# Artec Studio code is not in the sys path, add it
current_module_path = os.path.abspath(__file__)
FLOS.sys_path_append_app_path(current_module_path, path_app_code)

# Execute Artec Studio code
try:
    from ArtecStudio import *

    pass
    # ArtecStudioIsOnCheck()
except Exception as error:
    print(f"{__name__}: Error executing some Artec Studio methods")
    
# Auto-switch off autopilot
def CallPreviewDo(message_queue):
    sm_gui_name1 = "Preview"
    command = "" # Command we want to send
    autopilot_mode = True # How the command has to be executed
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
    message = f"AutopilotOn sent to {sm_gui_name1}"
    message_queue.put(message)
    return True