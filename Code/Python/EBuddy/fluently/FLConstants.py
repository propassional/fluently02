###############
# Conventions #
###############
# Github login as michele.banfi@supsi.ch
# Path naming convention: all paths defined her shall start with \ and end without any \, as RELEASE_NAME = r"\2024_03_13_Teachix"
import os

######################
# Release identifier #
######################
RELEASE_NAME = r"2025_01_10_EBuddy" # r"2024_04_09_Teachix"

######################
# Interpreters #
######################
# In pycharm terminal >conda env list => >conda activate ml_env => conda info --envs
# => C:\Users\operator\.conda\envs\ml_env\python.exe è il path da settare in pycharm => verifica il path >C:\Users\operator\.conda\envs\ml_env\python.exe --version => Python 3.10.13

######################
# Modes & Parameters #
######################
# This part influences which yaml file will be loaded
USER_ROLES = ['Beginner', 'Test', 'Advanced','Guru']
USER_ROLE = USER_ROLES[0]
COMMAND_HISTORY_SAVE = True # True: command files are moved to a history directory for debug or neuronal net training aims, False: command files are deleted
# Sockets: D:\Banfi\Github\Fluently\Releases\2025_01_10_EBuddy\_readme.txt
ROS2_INTERPRETER = r'C:\Python38\python.exe'
ROS2_SOCKET_LISTENER = r'D:\Banfi\Github\Fluently02\Code\Python\ROS2\FLROS2ListenerSocket.py'
SOCKET_SERVER_IS_USED = True # Default: True, ROS2 operation: EBuddy starts a socket server, which waits until a socket client connects, then it sends via socket the state name at each state change
SOCKET_SERVER_LAUNCHES_CLIENT = True # Default: True, Server launches client, False means that the admin has to launch it manually (see doc)

# SM graph images and GUI
UI_LOGORRHOIC = False

SCROLL_IS_ACTIVE = False # Default: False
# False: better for demo, image displayed in GUI is continuously cropped, so that manual scrolling is not needed (hands-free mode)
# True: better for debug, scrollbars are active and can be used by the user via mouse (debug mode) for centering

# Display Settings
# Screen / Monitor scales
# Screen scale expected values:
# Screen1 (bigger screen, used for Artec Studio) = 2560x1440, 125%
# Screen2 (smaller screen on the right showing E-Buddy) = 1024x768 (lower resolution = bigger images, 800x600 very big), 150% (it does not influence EBuddy, only windows taskbar),
# The center area of the GUI can not be as big as the whole screen, so scale it here:
# Following values are saved here for documentation, but they do not influence the EBuddy software
SCREEN_NR2_WIDTH_FACTOR = 0.95
SCREEN_NR2_HEIGHT_FACTOR = 1
# Remember: changing the scale applied to Screen1 - where Artec Studio runs - may have bad influence on finding the images within it!
# Remember: display > settings allows to change the amount of zoom % for the corresponding screen, and this influences the way the output is displayed
# Remember: the amount of zoom influences all GUI dimensions (from fonts to images): same font appears smaller with 100% than with 130%

# Timing
TIME_WAIT_GRAPH_IMAGE_IS_SAVED: float = 0.4 # [sec, decimal] 0.5? 0.4 is not enough in autopilot mode, GUI waits before loading image, # See error description in D:\Banfi\Github\Fluently\Errors\2024_08_13_ImageUpdate
TIME_MOUSE_MOVE_TO = 0.9 # 0.0 is the quickest good for advanced or guru users, 2 is good for very slow demo, 1 for beginner user?

# Image processing
CV2_MATCHING_SCORE_GOOD = 0.8
CV2_MATCHING_SCORE_SUFFICIENT = 0.5 # Default 0.5, very good = 1, bad = 0, 0.65 overlapping a not recording mode on a record mode image
CV2_TEST_IMAGES_SAVE = True
CV2_OCR_AREA = 1500 # "Auto" has 1000,"Ready" has 1176, preview has 129'000

# Command files
TIME_WAIT_READ_NEXT_COMMAND = 0.1 # [sec, decimal]
COMMAND_FILE_NR_OF_ARGS = 3 # See examples here: D:\Banfi\Github\Fluently\Releases\2024_07_09_EBuddy\Data\Output\Intents\_Released
# Dynamic loading of python modules
MODULES_RELOAD_ALWAYS = True

# Log files
BIG_LOG_SIZE_LIMIT = 600 * 1024  # 600 kilobytes in bytes

#########
# Names #
#########
# Commands / transitions
#COMMAND_PREFIX = 'Cmd' # See motivation in FLGUIMain.py
# This jump transition is not defined in the SM's, but it's defined within the code and thus it can be always executed
APP_NAME = "EBuddy for " + USER_ROLE + " User" # Title or name of the app shown to the user on top
# This is the text message in the top the main window
MESSAGE_GUI_STARTUP = "EBuddy is up and running"
COMMAND_SHUTDOWN = "Shutdown"
AUTO_START_TRANSITION_NAME = "Auto" # "AutoStart"
APP_DIR_NAME = "App"

ARTEC_STUDIO_APP_NAME = "astudio_pro.exe"
APPLICATION_TASKBAR = "taskbar"
APPLICATION_ARTECSTUDIO = "ArtecStudio"
APPLICATION_AODK = "AODK"
COMMAND_AUTOPILOT_ON = "AutopilotOn" # Decided with Rocco
COMMAND_AUTOPILOT_OFF = "AutopilotOff" # Decided with Rocco
COMMAND_TEXT_SPLIT_STRING = "|"
COMMAND_TEXT_NEW_FILE_NAME_WITH_DATE = "NewFileNameWithDate"
COMMAND_TEXT_DATA_OUTPUT_DIRECTORY = "DataDirectory"
COMMAND_TEXT_CONTROL_A_DELETE = "Ctrl&A&Del"
COMMAND_TEXT_DO_NOT_ENTER = "PressNotEnter"
SM_STATE_SEPARATOR = "¦" # Separates SM name from state name internally
SM_STATE_SEPARATOR_FOR_NLU = " " # Separates SM name from state name for Rocco

############
# Messages #
############
# GUI Messages
MSG_CLOSE_APP = RELEASE_NAME + " will be closed now"
# Log Messages
INFO = "info"
WARNING = "warning"
ERROR = "error"

#########
# Links #
#########
RELEASES_FOLDER = r"D:\Banfi\Github\Fluently02\Releases"
RELEASE_FOLDER = RELEASES_FOLDER + "\\" + RELEASE_NAME
SM_FILES = RELEASE_FOLDER + r"\Data"

###################################
# Files in INPUT, used by Teachix #
###################################
SM_FILES_INPUT = SM_FILES + "\Input"
YAML_FILES = SM_FILES_INPUT + "\YAML"
YAML_MAIN_PATH = os.path.join(YAML_FILES, USER_ROLE + r".yaml")
IMAGE_TRANSPARENT_PATH = SM_FILES_INPUT + '\ImageTransparent.PNG'

# The command text files report the text content of the voice commands
COMMAND_DIR = "\Command"
COMMAND_FOR_EBUDDY = SM_FILES_INPUT + COMMAND_DIR + "\CommandForEBuddy" # Dir for writing E-Buddy commands
COMMAND_FOR_EBUDDY_HISTORY = SM_FILES_INPUT + COMMAND_DIR + "\CommandForEBuddyHistory" # Dir for storing all commands
COMMAND_FOR_EBUDDY_REPOSITORY = SM_FILES_INPUT + COMMAND_DIR + "\CommandForEBuddyRepository" # Dir for storing some commands (not used)
SM_COMMAND_IMAGES = SM_FILES_INPUT + "\App" # "\CommandImage"
SM_FEEDBACK_TOOLS = SM_FILES_INPUT + "\App" # "\Tool" # Check tools and rollback tools
SM_FILES_IMAGES_ARTECSTUDIO = SM_COMMAND_IMAGES + "\ArtecStudio"
SM_FILES_IMAGES_TASKBAR = SM_COMMAND_IMAGES + "\Taskbar"
SM_FILES_TEXTS = SM_FILES_INPUT + "\App" # "\CommandText"

#####################################################################
# Files in OUTPUT, generated by EBuddy #############################
#####################################################################
SM_FILES_OUTPUT = SM_FILES + "\Output"
INTENTS_DIR = "\Intents\Generated"
LINK_TO_COMMAND_IMAGES = r"C:\users\operator\_SMBLinkToCommandImages.lnk"

# Artec Projects
DATA_OUTPUT_ARTEC = r"D:\Banfi\Lavoro\ArtecStudio\ArtecStudio16ProjectsTest" # D:\Banfi\Lavoro\ArtecStudio\ArtecStudio16ProjectsTest
DATA_OUTPUT = SM_FILES_OUTPUT + "\Data"

# Logs
LOG_FILES = SM_FILES_OUTPUT + "\LogFile"
LOG_FILES_BIG = SM_FILES_OUTPUT + "\LogFileBig"

# Images searched
CV2_IMAGES = SM_FILES_OUTPUT + "\CV2Image"

# SM screenshots
SCREENSHOT_FILES = SM_FILES_OUTPUT + "\Screenshot"
CV2_FILES = SM_FILES_OUTPUT + "\CV2Image"
SM_GRAPH_DEFAULT_SIZE = SCREENSHOT_FILES + "\SMGraphDefault_"
SM_GRAPH_CROPPED = SCREENSHOT_FILES + "\SMGraphCropped_"
SCREENSHOT_DEFAULT = "screenshot_default.png"
SCREENSHOT_CROPPED = "screenshot_cropped.png"
SM_GRAPH_SMALL = "SMGraphSmall.png"

# SM containers
JSON_FILES = SM_FILES_OUTPUT  + "\JSON"
PICKLE_FILES = SM_FILES_OUTPUT + "\Pickle"
SM_FILE_PICKLE_DEFAULT = "SMDefault.pickle"
SM_FILE_PICKLE_FLUENTLY_GA = "SMFluentlyGA.pickle"