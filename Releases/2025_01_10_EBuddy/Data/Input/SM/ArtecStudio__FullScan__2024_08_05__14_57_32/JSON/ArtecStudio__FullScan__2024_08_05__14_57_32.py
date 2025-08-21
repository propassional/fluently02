import sys
import os
import queue
from fluently.FLOS import *

# Add the directory containing FanucRosInterface.py to the system path
sys.path.append(os.path.abspath(r'D:\Banfi\Github\Fluently\Code\Python\knowledge-transfer-fleuntly-main'))
print("Printing sys.path \n")
for path in sys.path:
    print(path)
from move_robot import FanucRosInterface

sm_gui_name1 = "FullScan"  # SM to which we want to send a message via file
path_app_code = "..\\..\\..\\App\\ArtecStudio\\Code"

# Artec Studio code is not in the sys path, add it
current_module_path = os.path.abspath(__file__)
sys_path_append_app_path(current_module_path, path_app_code)

# Execute Artec Studio code
try:
    from ArtecStudio import *

    pass
    # ArtecStudioIsOnCheck()
except Exception as error:
    print(f"{__name__}: Error executing some Artec Studio methods")
    
def count_csv_lines(file_path):
    with open(file_path, 'r', encoding='utf-8') as file:
        return sum(1 for line in file)


############################################################################################
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

def ReadyDoOld(message_queue=""):
    message = f"Let's do a full scan"
    message_queue.put(message)
    return True

def RecordingIsOffDo(message_queue):
    message = "Recording is off"
    message_queue.put(message)
    return True

def RecordingIsOnDoLoop(message_queue):
    csv_abs_path_root = r"D:\Banfi\Github\Fluently\Code\Python\knowledge-transfer-fleuntly-main\trajs"

    # Scan the top part of the propeller
    # Override 100%
    # csv_name = "vincenzo_top_scan_all_positions.csv" # 60 positions
    # index_start = 0
    # index_end = 25
    # control_time = 0.8

    # Scan the complete propeller: full scan
    # Override 100%q
    csv_name = "vincenzo_all_scan_positions.csv" # 1934 positions
    index_start = 0
    index_end = 1800 # 1800 circa end loop alto, default 1930
    control_time = 0.005

    csv_abs_path = os.path.join(csv_abs_path_root, csv_name)
    cobot = FanucRosInterface(csv_abs_path, index_start, index_end, control_time)

    last_index = cobot.goto_positions()

    #message = "Rec is on, pos " + last_index
    message = "Rec is on, pos " + str(last_index)
    message_queue.put(message)
    return True

# Final demo version, while old demos versions are commented
# It is called RecordingIsOnDo so that StartCobotDo can start the recording, and then RecordingIsOnDo can start the cobot afterwards
def RecordingIsOnDo(message_queue):

    # message = "Cobot goes to top scan pos"
    #StartCobotDo(message_queue)

    # Scan the top part of the propeller
    # Override 100%
    '''
    csv_abs_path_root = r"D:\Banfi\Github\Fluently\Code\Python\knowledge-transfer-fleuntly-main\trajs"
    csv_name = "vincenzo_top_scan_all_positions.csv" # 60 positions
    index_start = 0
    index_end = 25
    control_time = 0.8
    '''

    # Scan trajectory: canonical Fluently full scan demo using the points defined with Tommaso Bernasconi
    # Scan the complete propeller: full scan
    # Override 100%
    '''
    csv_abs_path_root = r"D:\Banfi\Github\Fluently\Code\Python\knowledge-transfer-fleuntly-main\trajs"
    csv_name = "vincenzo_all_scan_positions.csv" # 1934 positions
    index_start = 0
    index_end = 1930
    control_time = 0.05 # 0.05 Paolo canonical value # MB canonical value 0.005 # Bad too slow 0.5
    '''
    
    # Scan trajectory: trajectory of the full scan is automatically calculated
    # Override 100%
    '''
    csv_abs_path_root = r"D:\Banfi\Github\Fluently\Code\Python\knowledge-transfer-fleuntly-main\trajs\2025_05_07_Paolo"
    csv_name = "trj_smooth.csv" # 1246 positions
    index_start = 0
    index_end = 0 # If 0, use all data, 1293 # 740 # 2552 # 1246
    control_time = 0.05
    '''
    csv_abs_path_root = r"D:\Banfi\Github\Fluently02\Releases\2025_01_10_EBuddy\Data\Input\DataJoints\Diego"
    csv_name = "diego_arc_trj.csv" # 1495 positions
    index_start = 0
    index_end = 0 # 1495 
    control_time = 0.05
    
    csv_abs_path = os.path.join(csv_abs_path_root, csv_name)
    print("Cobot path used: " + csv_abs_path)

    if index_end == 0:
        # Use all available joint data
        index_end = count_csv_lines(csv_abs_path) - 1

    cobot = FanucRosInterface(csv_abs_path, index_start, index_end, control_time)

    last_index = cobot.goto_positions()

    message = "Rec is on, pos " + last_index
    message_queue.put(message)
    return True

def StartCobotDoOld(message_queue):
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
    message = "Cobot goes to top scan pos"
    message_queue.put(message)
    return True


if __name__ == '__main__':
    message_queue = queue.Queue()
    RecordingIsOnDo(message_queue)

    print("\nFirst start cobot program REMOTE_CTRL, it must show line3 running!\n")

    while True:
        print("Press t (top pos), r (record full scan), l (loop full scan), 'q' to quit")
        user_input = input("What is your choice? ")
        user_input = user_input.lower()
        # Check the key and update my_int accordingly
        if user_input == 't': # Top scan position
            print('cobot to top scan position')
            StartCobotDo(message_queue)
        elif user_input == 'r':
            print('Doing full scan once')
            RecordingIsOnDo(message_queue)
        elif user_input == 'l':
            while True:
                print('Doing full scan in loop')
                RecordingIsOnDoLoop(message_queue)
        elif user_input == 'q':
            print('quit')
            break