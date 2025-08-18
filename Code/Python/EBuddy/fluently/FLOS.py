# For avoiding circular imports, this module_declared_at_top_of_the_file does not have to use higher levels modules
import sys
from datetime import datetime

import psutil
import os

import pygetwindow
import win32com.client
import yaml
from screeninfo import get_monitors

from fluently.FLConstants import ARTEC_STUDIO_APP_NAME, LINK_TO_COMMAND_IMAGES, SM_COMMAND_IMAGES, COMMAND_FOR_EBUDDY, \
    SCREENSHOT_FILES, UI_LOGORRHOIC, YAML_MAIN_PATH, APP_DIR_NAME, CV2_FILES


# Useful methods:
# File System:
    # file_name = os.path.basename(file_input_path) # Extracts the file name from the path
    # os.path.join, if os.path.exists(path):, os.getcwd() current dir,
    # root_name, extension = os.path.splitext(file_name)
    # for root, dirs, files in os.walk(directory):
# time.sleep(1),
# return None, sys.exit()

def artec_studio_is_open():
    process_name = ARTEC_STUDIO_APP_NAME
    for proc in psutil.process_iter():
        if proc.name() == process_name:
            return True
    return False

def artec_studio_kill():
    process_kill_by_name(ARTEC_STUDIO_APP_NAME)

def datetime_now_get_string():
    now = datetime.now()
    now_string = now.strftime("%Y_%m_%d__%H_%M_%S")
    return now_string

def file_find(filename, search_path):
    """
    Searches for the given filename in the specified directory and its subdirectories.
    Returns the full path of the first occurrence found.
    """
    for root, dirs, files in os.walk(search_path):
        if filename in files:
            full_path = os.path.join(root, filename)
            app_name = os.path.basename(os.path.dirname(full_path))
            return full_path, app_name

def file_remove(file_path):
    os.remove(file_path)  # self.command_file_path is set in self.command_read()

def focus_set_on_app(window_title = "Artec Studio 16 Professional"):
    # Get the window title or program ID (modify this based on your app)
    # Find the window by title
    my_window = pygetwindow.getWindowsWithTitle(window_title)[0]
    # Activate the window (bring it to the foreground)
    if my_window:
        my_window.activate()
    else:
        if UI_LOGORRHOIC: print(f"window_title {window_title} not found")

def path_current_module():
    current_module_directory = os.path.abspath(__file__)
def path_find_item():
    # File or directory name to be searched
    item_searched = 'ArtecStudio'

    # Get the current directory where this script is located
    #current_directory = os.path.dirname(__file__)
    current_directory = r"D:\Banfi\Github\Fluently\Releases\2024_07_09_EBuddy\Data\Input"
    # Search for the target file or directory
    for root, dirs, files in os.walk(current_directory):
        if item_searched in dirs or item_searched in files:
            target_path = os.path.join(root, item_searched)
            if UI_LOGORRHOIC: print(f"Found at: {target_path}")
            break
    else:
        if UI_LOGORRHOIC: print(f"{item_searched} not found in the file system.")

def path_from_SM_code_to_app_code(path_SM_code, path_app_code):
    path_to_be_resolved = os.path.join(path_SM_code, path_app_code)
    path_to_artec_studio_code = os.path.abspath(path_to_be_resolved)
    if os.path.exists(path_to_artec_studio_code):
        if UI_LOGORRHOIC: print(f"ArtecStudio code path: {path_to_artec_studio_code}")
        return path_to_artec_studio_code
    else:
        if UI_LOGORRHOIC: print(f"Path does not exist: {path_to_artec_studio_code}")
        return None

def path_go_up_one_level(path):
    # Get the parent directory of the given path
    parent_dir = os.path.dirname(path)
    return parent_dir

def path_join(path1, path2):
    joined_path = os.path.join(path1, path2)
    return joined_path

# Input: D:\\Banfi\\Github\\Fluently\\Releases\\2024_07_09_EBuddy\\Data\\Input\\App\\Taskbar\\2560x1440_125%\\Widgets\\StartArtecStudio.png"
# Output: StartArtecStudio
def path_extract_filename(file_path):
    # Get the base filename from the path
    base_filename = os.path.basename(file_path)
    # Remove the file extension (if any)
    filename_without_extension = os.path.splitext(base_filename)[0]
    return filename_without_extension

# Input: D:\\Banfi\\Github\\Fluently\\Releases\\2024_07_09_EBuddy\\Data\\Input\\App\\Taskbar\\2560x1440_125%\\Widgets\\StartArtecStudio.png"
# Output: Taskbar
def path_extract_app_name(path):
    # Split the path into two parts
    path_split = path.split(APP_DIR_NAME)
    # Extract the string after "App" (if it exists)
    if len(path_split) > 1:
        #app_name = path_parts[1].split(os.sep)[2]
        path_right_part = path_split[1]
        path_right_part_split = path_right_part.split(os.sep)
        # This does not work always: app_name = path_right_part_split[2]
        for item in path_right_part_split:
            if item:
                # For focusing on app, the app name must be a subset of the app name resulting from OS
                # e.g. OS knows "Artec Studio Professional", so app name can be "Artec"
                app_name = item
                break
        if UI_LOGORRHOIC: print(f"Extracted string: {app_name}")
    else:
        if UI_LOGORRHOIC: print("No 'App' found in the given path.")
    return app_name

def path_remove_extension(file_path):
    # Remove the file extension and return the path with the base name
    return os.path.splitext(file_path)[0]

# Used for releases before 2024_04_09 included
def path_create_release_dir_2024_04_09(path):
    # Create the directories, only if they don't exist
    try:
        os.makedirs(os.path.join(path, "Data", "Input", "CommandImages"))
        os.makedirs(os.path.join(path, "Data", "Input", "Commands"))
        os.makedirs(os.path.join(path, "Data", "Input", "CommandTexts"))
        os.makedirs(os.path.join(path, "Data", "Input", "FeedbackTools"))
        os.makedirs(os.path.join(path, "Data", "Input", "JSON"))
        os.makedirs(os.path.join(path, "Data", "Output", "CV2Images"))
        os.makedirs(os.path.join(path, "Data", "Output", "JSON"))
        os.makedirs(os.path.join(path, "Data", "Output", "LogFiles"))
        os.makedirs(os.path.join(path, "Data", "Output", "Pickles"))
        os.makedirs(os.path.join(path, "Data", "Output", "Screenshots"))
        if UI_LOGORRHOIC: print(f"Directories in {path} created successfully")
    except FileExistsError:
        if UI_LOGORRHOIC: print(f"Some directories in {path} already exist")

# For simplicity all dir names are now singular (App vs Apps)
def path_create_release_dir_2024_07_09(path):
    # Create the directories, only if they don't exist
    try:
        os.makedirs(os.path.join(path, "Data", "Input", "App"))
        os.makedirs(os.path.join(path, "Data", "Input", "App", "ArtecStudio"))
        os.makedirs(os.path.join(path, "Data", "Input", "App", "DK"))
        os.makedirs(os.path.join(path, "Data", "Input", "App", "Taskbar"))
        os.makedirs(os.path.join(path, "Data", "Input", "App", "Taskbar", "2560x1440_125%")) # Supports multi screen resolution and zoom
        os.makedirs(os.path.join(path, "Data", "Input", "App", "Taskbar", "2560x1440_125%", "Widgets")) # Elements that can be clicked

        os.makedirs(os.path.join(path, "Data", "Input", "Command")) # Used online for the command files
        os.makedirs(os.path.join(path, "Data", "Input", "Command", "commandsForEBuddy"))
        os.makedirs(os.path.join(path, "Data", "Input", "Command", "commandsForEBuddyRepository")) # This will be filled manually
        os.makedirs(os.path.join(path, "Data", "Input", "SMs")) # This will be filled manually

        os.makedirs(os.path.join(path, "Data", "Output", "CV2Image"))
        os.makedirs(os.path.join(path, "Data", "Output", "JSON"))
        os.makedirs(os.path.join(path, "Data", "Output", "LogFile"))
        os.makedirs(os.path.join(path, "Data", "Output", "Pickle"))
        os.makedirs(os.path.join(path, "Data", "Output", "Screenshot"))
        if UI_LOGORRHOIC: print(f"Directories in {path} created successfully")
    except FileExistsError:
        if UI_LOGORRHOIC: print(f"Some directories in {path} already exist")

def processes_write_to_file_pre():
    with open("D:\processes_pre.txt", "w") as file:
        for proc in psutil.process_iter():
            file.write(f"{proc}\n")
def processes_write_to_file_post():
    with open("D:\processes_post.txt", "w") as file:
        for proc in psutil.process_iter():
            file.write(f"{proc}\n")

def process_kill_by_name(process_name):
    for proc in psutil.process_iter():
        if proc.name() == process_name:
            proc.kill()

# One screen connected: returns top left of first screen
# Two screens connected: returns top left of second screen
def screen_coordinates_for_GUI_main():
    # Get all available monitors
    monitors = get_monitors()

    # Check if there are at least two monitors
    if len(monitors) >= 2:
        # Get the second monitor's width and height
        first_monitor = monitors[0]
        first_monitor_width = first_monitor.width
        top = first_monitor_width
        left = 0
    else:
        top = 0
        left = monitors[0].height / 2 # One monitor only: top is Artec Studio, bottom is E-buddy
    return top, left

# One screen connected: returns top left of the center of the first screen
# Two screens connected: returns top left of the center of the second screen
def screen_coordinates_for_popup():
    # Get all available monitors
    monitors = get_monitors()

    # Check if there are at least two monitors
    if len(monitors) >= 2:
        # Get the second monitor's width and height
        first_monitor = monitors[0]
        second_monitor = monitors[1]
        second_monitor_width = second_monitor.width
        second_monitor_height = second_monitor.height
        top = first_monitor.width + (second_monitor_width / 2)
        left = second_monitor_height / 2
    else:
        # There is no second monitor available
        first_monitor = monitors[0]
        top = first_monitor.width / 2
        left = first_monitor.height / 2
    return top, left

def commands_delete(log_object):
    # Delete all old command files
    for filename in os.listdir(COMMAND_FOR_EBUDDY):
        if filename.endswith(".txt"):
            file_path = os.path.join(COMMAND_FOR_EBUDDY, filename)
            file_remove(file_path)
            log_object.print(f"Deleted {file_path}")

def CV2_images_delete(log_object):
    # Delete all old image files
    for filename in os.listdir(CV2_FILES):
        if filename.endswith(".png"):
            file_path = os.path.join(SCREENSHOT_FILES, filename)
            file_remove(file_path)
            log_object.print(f"Deleted {file_path}")

def screenshots_delete(log_object):
    # Delete all old command files
    for filename in os.listdir(SCREENSHOT_FILES):
        if filename.endswith(".png"):
            file_path = os.path.join(SCREENSHOT_FILES, filename)
            file_remove(file_path)
            log_object.print(f"Deleted {file_path}")

def shortcut_update(shortcut_path, target_path, description):
    # Create a new WScript.Shell object
    shell = win32com.client.Dispatch("WScript.Shell")
    # Create a shortcut object
    shortcut = shell.CreateShortcut(shortcut_path)
    # Set the target path (executable or file)
    shortcut.TargetPath = target_path
    # Set the description (comment field)
    shortcut.Description = description
    # Save the modified shortcut
    shortcut.Save()

def sys_path_append_app_path(current_module_path, path_app_code):
    if UI_LOGORRHOIC: print(f"current_module_path {current_module_path}")
    current_module_directory = os.path.dirname(current_module_path)

    # Debug: write sys path before changing it
    sys_path_to_file(current_module_directory)

    # Find the path to the app
    path_app_code_abs = path_from_SM_code_to_app_code(current_module_directory, path_app_code)

    if path_app_code_abs:
        if UI_LOGORRHOIC: print(f"Path to Artec Studio code is {path_app_code_abs}")
        sys.path.append(path_app_code_abs)
        # Debug: write sys path after changing it
        sys_path_to_file(current_module_directory)
    else:
        if UI_LOGORRHOIC: print(f"Error: path to Artec Studio code could not be found")

def sys_path_to_file(current_module_directory):
    # Create the file path in the current directory
    file_path = os.path.join(current_module_directory, "sys_path.txt")

    # Open the file in write mode
    with open(file_path, "w") as file:
        # Iterate over each path in sys.path
        for path in sys.path:
            # Write each path to the file
            file.write(path + '\n')
    if UI_LOGORRHOIC: print(f"sys.path {file_path} written to {file_path}")

def yaml_load():
    # Read SM SM_JSON_data from the YAML config file
    with open(YAML_MAIN_PATH, "r") as yaml_file:
        loaded_data = yaml.safe_load(yaml_file)
    return loaded_data

if __name__ == '__main__':
    path_find_item()

    file_path = "D:\\Banfi\\Github\\Fluently\\Releases\\2024_07_09_EBuddy\\Data\\Input\\App\\Taskbar\\2560x1440_125%\\Widgets\\StartArtecStudio.png"
    output = path_extract_filename(file_path) # Output: StartArtecStudio

    # Example usage
    file_path = "/path/to/myfile.txt"
    result = path_remove_extension(file_path)
    print(f"File path without extension: {result}")

    ################################################################################
    path = r"D:\\Banfi\\Github\\Fluently\\Releases\\2024_07_09_EBuddy\\Data\\Input\\App\\Taskbar\\2560x1440_125%\\Widgets\\StartArtecStudio.png"
    app = path_extract_app_name(path)

    # Create the new release directory
    path_create_release_dir_2024_04_09(r"D:\temp\Banfi\Releases\2024_04_09_Teachix")
    path_create_release_dir_2024_07_09(r"D:\temp\Banfi\Releases\2024_04_09_EBuddy")

    screen_coordinates_for_GUI_main()
    screen_coordinates_for_popup()

    # Search for "CmdStartScan.PNG" in SM_FILES_IMAGES and its subdirectories
    result, app_name = file_find("CmdStartScanMenu.PNG", SM_COMMAND_IMAGES)

    if result:
        print(f"Found 'CmdStartScanMenu.PNG' at: {result}")
        print(f"App name: {app_name}")
    else:
        print("File 'CmdStartScanMenu.PNG' not found.")

    ################################################################################

    # Shortcut usage example
    shortcut_path = LINK_TO_COMMAND_IMAGES # "C:\\path\\to\\your\\shortcut.lnk"
    target_path = SM_COMMAND_IMAGES # D:\Banfi\Github\Fluently\Releases\2024_04_09_Teachix\Data\Input\CommandImages # "C:\\path\\to\\new\\exe.exe"
    description = "Link to command images"

    shortcut_update(shortcut_path, target_path, description)
    print(f"Shortcut '{os.path.basename(shortcut_path)}' updated successfully!")

    # Get a list of all running processes
    all_processes = [proc.name() for proc in psutil.process_iter()]

    process_name_to_kill = ARTEC_STUDIO_APP_NAME  # Replace with your target process name
    process_kill_by_name(process_name_to_kill)