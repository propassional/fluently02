# This module moves the cobot using the program REMOTE_CTRL 
# 0) Run REMOTE_CTRL on the cobot tablet
# 1) Check in this file the path of file_path
# 2) Check in this file the path of os.path.abspath
# 3) Adjust PathToFile and run this interactive program in pycharm terminal: 
#    >python PathToFile\ArtecStudio__SliceScan__2024_09_15__11_43_22.py
#    >python D:\Temp\CobotComTest\ArtecStudio__SliceScan__2024_09_15__11_43_22.py

import time
import sys
import os
import queue

SLEEP_DO = True
SLEEP_TIME = 3

# Add the directory containing FanucRosInterface.py to the system path
path_to_add = os.path.abspath(r'D:\Temp\CobotComTest\knowledge-transfer-fleuntly-main')
if path_to_add not in sys.path:
    sys.path.append(path_to_add)
print("Printing sys.path \n")
for path in sys.path:
    print(path)
from move_robot import FanucRosInterface

# Cobot control has to remember where the cobot was, in order to move across secured positions and avoid collisions
def SaveGlobalVariable(value):
    """
    Save the integer value to a file.

    Args:
        value (int): The integer value to save.
    """
    file_path = r'D:\global_last_index.txt'
    try:
        with open(file_path, 'w') as file:
            file.write(str(value))
        print(f"Value {value} saved to {file_path}")
    except Exception as e:
        print(f"An error occurred while saving the value: {e}")


def LoadGlobalVariable():
    """
    Load the integer value from a file.

    Returns:
        int: The loaded integer value.
    """
    file_path = r'D:\global_last_index.txt'
    try:
        with open(file_path, 'r') as file:
            value = int(file.read())
        print(f"Value {value} loaded from {file_path}")
        return value
    except Exception as e:
        print(f"An error occurred while loading the value: {e}")
        return None

def ReadyDoOld(message_queue=""):
    image_path = r'D:\Banfi\Github\Fluently\Releases\2024_07_09_EBuddy\Data\Input\App\ArtecStudio\Hints\SliceScan.png'
    message = f"SliceScan holes"
    message_queue.put(message)
    if SLEEP_DO:
        time.sleep(SLEEP_TIME)
    return image_path  # So the image will be displayed in GUI

# Sends cobot to slice scan home pos
def HomeDo(message_queue):
    global global_last_index
    #StartCobotDo()
    # Set override to 100%
    csv_abs_path_root = r"D:\Banfi\Github\Fluently\Code\Python\knowledge-transfer-fleuntly-main\trajs"
    csv_name = "arc_traj.csv" # Slice scan
    csv_abs_path = os.path.join(csv_abs_path_root, csv_name)
    # Position nr in csv
    index_start = 0 # Goto position 6 starting from position 0, the highest one
    index_end = 1
    control_time = 1
    cobot = FanucRosInterface(csv_abs_path, index_start, index_end, control_time)
    global_last_index = cobot.goto_positions()
    SaveGlobalVariable(global_last_index)
    message = "Cobot goes to slice scan home"
    message_queue.put(message)
    return True

# Sends cobot down one pos
def MovedDownDo(message_queue):
    global global_last_index
    # Override 100%
    csv_abs_path_root = r"D:\Banfi\Github\Fluently\Code\Python\knowledge-transfer-fleuntly-main\trajs"
    csv_name = "arc_traj.csv" # Slice scan
    csv_abs_path = os.path.join(csv_abs_path_root, csv_name)
    # Position nr in csv
    if global_last_index > -1:
        index_start = global_last_index + 1
        index_end = global_last_index + 2
        control_time = 0
        cobot = FanucRosInterface(csv_abs_path, index_start, index_end, control_time)
        global_last_index = cobot.goto_positions()
        SaveGlobalVariable(global_last_index)
        message = "Cobot goes one pos down"
        message_queue.put(message)
        return True
    else:
        return False

# Sends cobot up one pos
def MovedUpDo(message_queue):
    global global_last_index
    # Override 100%
    csv_abs_path_root = r"D:\Banfi\Github\Fluently\Code\Python\knowledge-transfer-fleuntly-main\trajs"
    csv_name = "arc_traj.csv" # Slice scan
    csv_abs_path = os.path.join(csv_abs_path_root, csv_name)
    # Position nr in csv
    if global_last_index >= 1:
        index_start = global_last_index
        index_end = global_last_index - 1
        control_time = 0
        cobot = FanucRosInterface(csv_abs_path, index_start, index_end, control_time)
        global_last_index = cobot.goto_positions()
        SaveGlobalVariable(global_last_index)
        message = "Cobot goes one pos up"
        message_queue.put(message)
        return True
    else:
        message = "Cobot go home first!"
        message_queue.put(message)
        return False

# Sends Cobot to scan top pos
def StartCobotDo():
    # Override 100%
    csv_abs_path_root = r"D:\Banfi\Github\Fluently\Code\Python\knowledge-transfer-fleuntly-main\trajs"
    csv_name = "vincenzo_top_scan_all_positions.csv" # 60 positions, only the first one is used here
    csv_abs_path = os.path.join(csv_abs_path_root, csv_name)
    # Position nr in csv
    index_start = 0
    index_end = 1
    control_time = 0
    cobot = FanucRosInterface(csv_abs_path, index_start, index_end, control_time)
    cobot.goto_positions()
    # message = "Cobot goes to top scan pos"
    # message_queue.put(message)
    return True

global_last_index = LoadGlobalVariable()
print("global_last_index = " + str(global_last_index))

if __name__ == '__main__':
    message_queue = queue.Queue()

    print("\nStart cobot program REMOTE_CTRL, it must show line3 running\n")

    while True:
        print("Press t (top pos), h (slice scan home), u (move up), d (move down), 'q' to quit")
        user_input = input("What is your choice? ")
        user_input = user_input.lower()
        # Check the key and update my_int accordingly
        if user_input == 'h': # Slice scan home position
            print('goto home top pos')
            HomeDo(message_queue)
        elif user_input == 't': # Top scan position
            print('cobot sent to top scan position')
            StartCobotDo()
        elif user_input == 'u':
            print('moved up')
            MovedUpDo(message_queue)
        elif user_input == 'd':
            print('moved down')
            MovedDownDo(message_queue)
        elif user_input == 'q':
            print('quit')
            break