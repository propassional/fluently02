# Usage instructions:
# 1) Run D:\Banfi\SikuliX\Executables\sikulixide-2.0.5-win.jar
# 2) Run this script from the Sikulix IDE

# Todo: search for the string "xx"
# Scanner save: capisce spesso skanner saive
# Scanner save: non riesco a fargli riconoscere l'immagine SaveAs, e non posso verificare il matching

# Developing difficulties with Sikulix
# - Sikulix IDE is not PyCharm: there is no breakpoint functionality, nor stack => insert popAsk or exit calls
# - Error "Not set" is absolutely meaningless, since no hints are given, sometimes only the top calling method name is returned, and no line
#       Example1: if you rename a method, and forget to rename its call, you will get "Not set"
#       Example2: you try to write into a string the sum of a string with a defined bool variable (this error is crazy!)
# - Lack of real robustness:
#    - If you change the monitor, or you change your monitor setup (more or less bright, all images to be found have to be rescanned!
#    - If click is not found, sometimes it can happen that no error is returned (so you are on the "error / no error" border), but click is not performed
# - Phyton: 1) Syntax is quite different from most languages: if cond:
#           2) with open(log_file_name_complete, "w") as log_file:
#           2) Spaces are accepted, but not always, so you have to insert tabs instead than spaces (INDENT error)
#           3) global variables, if you want to change them in a method use the global keyword
#           4) There are really no types? Not true! Here how to convert a boolean into a string: str(do_shutdown)
#           5) In Jython (and Python in general), when you reference a variable within a function, the interpreter first looks for that variable in the local scope of the function. If it doesn't find the variable there, it then looks in the enclosing (non-local) scope, and finally, it looks in the global scope.
# 6) Questo comando ha un bug, se lo usi cosi, la var globale non è leggibile: with open(log_file_name_complete, "w") as log_file:
# jython particlaritä es print(x.getClass()) é diverso dal phyton print(type(x))
# Absurd: os.remove seems to work only with r+ and adding a write 
# Negative Indexing: you can use negative indexes to access elements from a list. Negative indexes count from the end of the list, with -1 representing the last element, -2 representing the second-to-last element, and so on.
# global do_shutdown # By this way, its value can be changed within the method, otherwise the assignment does not happen, and you get no warnings nor errors!

# Usage instructions: 
# 1) In PCArtec > Sikulix, click the button "Run", the app will wait for commands written into commands_dir_in
# 2) In PCArtec > open the server: D:\Banfi\codeUtility\TCPServer\HelloWorld\HelloWorld\TCPServer.cpp
# 3) Run external components: BT connect Realware to PCBeast, run PBBeast > ROSBridge, run ThinkpadCentre > ROS1
# 4) In ThinkpadCentre > run the client 
# 5) Wearing Realware, you can tell the commands from the commands_list, in this order: 
#    preview > record > stop > tools > global registration > fast fusion > save > close > shut down
# See how Artec Studio is automatically opened, scanner initiated, frames are acquired and recorded, 
# and a global registration and fast fusion is performed

# File system: with open(file_name, 'w') as file:, 
import os # os.listdir, os.remove, os.path.join, os.path.isfile, os.path.dirname, os.system(command) execute a command
import datetime
import datetime
# Java libs:
import javax.swing as swing
import java.awt as awt
import java.awt.Font as Font
import java.awt.Toolkit as Toolkit
import java.awt.GridBagConstraints as GridBagConstraints
import java.awt.GridBagLayout as GridBagLayout

################################# Global constants #######################################################
# All commands written by client in this dir will be executed by this Sikulix app
# The directory "..Github\Fluently.." becomes in Jython "..Githubluently..", since \f becomes , 
#    thus I changed fluently to Fluently
commands_dir_in = "D:\Banfi\Github\Fluently\Commands\cmdForSikulix" 
# All commands written by Sikulix in this dir will be read by server, sent to client, and executed by Cobot
commands_dir_out = 'D:\Banfi\Github\Fluently\Commands\cmdFromSikulix'
# Sikulix log file directory
log_dir = 'D:\Banfi\Github\Fluently\Code\Sikulix\logs' # If this path ends \, an error will arise
# [SenderOfTheCommand] Subject Verb ObjectComplement
command_artec_to_cobot_start = "[artec] cobot goto top_scan_position" # Aknoledge expected: same string + "finished"
command_artec_to_cobot_scan = "[artec] cobot goto all_scan_positions" # Aknoledge expected: same string + "finished"
# Reduced scan, used just for quick testing of the scan process:
# rename this varibale to "command_artec_to_cobot_scan" and comment the same variable above
command_artec_to_cobot_scan_reduced = "[artec] cobot goto top_scan_all_positions" # Aknoledge expected: same string + "finished"

# This list corresponds to all defined vocal commands, it is so formatted to correspond to the method_list
# Start scanning, good: stop scanning, save scanning, stop scanning
commands_list = ["scanning procedure", "scanner start", "scanner pause", "scanner stop", "scanner tools", "scanner global registration", 
                 "scanner fast fusion", "scanner save", "scanner close", "shut",     "sleep", "scanner open result", "scanning functionality"]
# The formatting of this list corresponds to the above commands, that are able to call the methods below
methods_list =  ["record",             "preview",       "pause",         "stop",         "tools",         "registration_global",              
                "fusion_fast",          "save",         "close",         "shutdown",  "sleep", "results",            "cobot_start"]
# In Jython, you can't directly use the auto() feature from Python's enum module because Jython doesn't natively support 
# Python 3.5+ features like automatic enumeration values. 
# However, you can manually assign integer values to the enum members in Jython if you want to use numerical constants.
# States numbers are defined here, do not use negative values, since in Phyton, -1 is the last element!
# This list models the states of the Sikulix code
class State:
    sikulix_is_off = 0    
    artec_studio_is_off = 1
    preview_is_on = 2
    preview_is_off = 3
    cobot_is_scan_start_pos = 4
    recording_is_on = 5
    recording_is_done = 6
    recording_is_paused = 7 
    tools_is_on = 8
    registration_global_done = 9
    fusion_fast_done = 10
    project_saved = 11
    mesh_shown = 12
    project_closed = 13
# Dictionary:
state_to_state_name = {
    State.sikulix_is_off: "Sikulix is shutting down",    
    State.artec_studio_is_off: "Artec Studio is off",
    State.preview_is_on: "Preview is on", 
    State.preview_is_off: "Preview is off",     
    State.cobot_is_scan_start_pos: "Cobot is in scan\nstart position",    
    State.recording_is_on: "Recording is on",
    State.recording_is_done: "Recording is done",
    State.tools_is_on: "Tools Menu is active",
    State.registration_global_done: "Global Registration is done",
    State.project_saved: "Project is saved",
    State.mesh_shown: "Mesh is shown",
}
# List of available commands, depending on the current state, which can be shown to the user as a hint
state_to_state_hint = {
    State.sikulix_is_off: "No hints available",        
    State.artec_studio_is_off: "Scanner Start",
    State.preview_is_on: "Scanning Functionality", # "Robot Start", 
    State.preview_is_off: "Scanner Save",         
    State.cobot_is_scan_start_pos: "Scanning Procedure", # "Robot Scan",    
    State.recording_is_on: "[Scanning, please wait...]",
    State.recording_is_done: "Scanner Stop", # ["artec tools"],
    State.tools_is_on: "Scanner Global Registration",
    State.registration_global_done: "Scanner Save",
    State.project_saved: "Scanner Open results",
    State.mesh_shown: "Scanner Close",
}

# Debug levels:
''' # Start
do_debug_prio_a = True # False # Light level debugging, just some main messages and interactions
do_debug_prio_b = True # False # Medium level debugging, multiple main messages and interactions
do_debug_prio_c = True # False # Heavy level debugging, a lot of messages and interactions
''' # End
do_debug_prio_a = False # False # Light level debugging, just some main messages and interactions
do_debug_prio_b = False # False # Medium level debugging, multiple main messages and interactions
do_debug_prio_c = False # False # Heavy level debugging, a lot of messages and interactions

# 3 is ok for GA demo, 8 for debugging
# Beware: if it is 1 sec, any other PC interaction becomes difficult, since focus is continously shifted to Sikulix!
time_sleep_main_loop = 3 # 3 GA demo,[seconds] sleep time between consecutive reads from the command dir: 0.1 = 100 ms, if 1 sec all PC interactions are difficult
time_sleep_between_ui_events = 2 # 1 GA demo, [seconds] sleep time between consecutive automatic UI events
time_sleep_popup = 1 # [seconds]
################################# Global variables #######################################################
popup_frame = None
do_shutdown = False # Set by the corresponding method
state_active = State.artec_studio_is_off
log_file = None
##########################################################################################################
# Check if one or more other Artec Studio windows are already opened, and close them before starting the Sikulix app
def artec_studio_opened_close():
#    while True: # While is needed for the use case of having to close multiple Artec Studio instances
    try:  
        artec_studio_scanning_stop()
        artec_studio_project_do_not_save()
    except FindFailed as e:
        # Good, no opened Artec Studio window has been found
        # Good, no scan was active, so the window closed immediately without the need to stop scan    
        log_file_write("I checked if an Artec Studio instance was opened, in that case I closed it")
##########################################################################################################
def artec_studio_project_do_not_save(): 
    try:
        # If Artec Studio is not open, following command fails, then except is activated, and nothing is done
        click(Pattern("1693294926970.png").similar(0.88))                       
        wait(time_sleep_between_ui_events)
        ui_ask_to_continue("I detected an opened Artec Studio\nMay I close it now, without saving?")        
        click(Pattern("1694688347503.png").targetOffset(938,-11))
        wait(time_sleep_between_ui_events)        
        click(Pattern("1694703083392.png").targetOffset(47,37)) 
        wait(time_sleep_between_ui_events)
        
    except FindFailed as e:            
        log_file_write("I checked if save window was active, if it was, I closed Artec Studio anyway")        
##########################################################################################################
def artec_studio_scanning_stop(): 
    try:
        # If Artec Studio is not open, following command fails, then except is activated, and nothing is done       
#        click(Pattern("1693294926970.png").similar(0.88))                       
        wait(time_sleep_between_ui_events)
        log_file_write("I detected an opened Artec Studio\nMay I close it now?")        
        click(Pattern("1694688347503.png").targetOffset(938,-11))
        wait(time_sleep_between_ui_events)
        click(Pattern("SureToClose.PNG").targetOffset(51,38))
        wait(time_sleep_between_ui_events)    
    except FindFailed as e:            
        log_file_write("I checked if scanning was active, if it was, I closed Artec Studio anyway")        
##########################################################################################################    
def artec_studio_set_focus_on():
    try:
        # If Artec Studio is already maximized, click on top of it
        # WARNING: don't include the part "Artec Studio 16 Professional", since it disappears if a project is opened!
        click(Pattern("1695730068225.png").targetOffset(-43,-21))
    except FindFailed as e:
        # If Artec Studio is not already maximized, click on its icon on the app bar
        click(Pattern("1693294926970.png").similar(0.81))
##########################################################################################################
# Used for cleaning the Sikulix input and output directories
def command_dir_cleanup(directory):
    try:
        # List all files in the directory
        log_file_write("Removing files in this directory: " + directory)
        files = os.listdir(directory)
        for file_name in files:
            file_path = os.path.join(directory, file_name)
            if os.path.isfile(file_path):
                log_file_write("Removing file: " + file_path)
                command_remove(file_path)
            else:                
                log_file_write("I will NOT delete this path, since it is not a file: " + file_path)
    except Exception as e:
        log_file_write("Error removing file: " + file_path)
        log_file_write(e)    
##########################################################################################################
# Read from file system and execute command
def command_do():
    try:       
        # This loop is neverending, but it can be stopped by reading the command "shutdown", that runs exit(0)            
        while True: 
            # List all files found in the command directory
            files = os.listdir(commands_dir_in)
            log_file_write("Seeking for command files in dir: " + commands_dir_in)
            log_file_write(str("State: " + state_to_state_name[state_active]))            
            ui_popup("[Command hint:]\n" + str(state_to_state_hint[state_active]))                        
            for file_name in files:
                file_path = os.path.join(commands_dir_in, file_name)
                log_file_write("Command file found: " + file_path)
                command = command_extract(file_path)
                # Transform a string into a method call, and call it (lambda expression)                
                if(command != ""):
                    ScannerCommands().command_call(command)
                    log_file_write("Command completed execution: " + file_path)
                else:
                    log_file_write("I did not receive a real command, so I will do nothing :-D")
                # Delete cmd file
                if os.path.isfile(file_path):
                    log_file_write("I will delete last command received, since I have already executed it")
                    # Guarantee that the following remove file is correctly limited:
                    # https://docs.python.org/3/library/os.html#os.remove can not remove directories, so it is sufficient to check
                    # if file_path is a sub dir of directory and is a .txt file
                    file_path_dir = os.path.dirname(file_path) # Remove the file name from the path
                    if file_path_dir.startswith(commands_dir_in) and file_path.endswith('.txt'):
                            command_remove(file_path)
                    else:
                        ui_ask_to_continue("I should delete this file, but I can't since the path is not correct:\n" + file_path)
                else:
                    log_file_write("I do NOT delete this path, since it is not a file:" + file_path)
            if do_shutdown:
                ui_popup("[I stop Sikulix now]")
                exit(1)
            time.sleep(time_sleep_main_loop)
    except Exception as exceptionError:
        log_file_write("command_do: following error occurred:") 
        log_file_write(exceptionError)
##########################################################################################################
# Extract the command name from the command file contents
# Check if the command is available
def command_extract(cmdFilePath):
#    cmdFiltered = cmdFilePath.replace(os.path.dirname(cmdFilePath), '') # Remove the path from the file name
#    cmdFiltered, extension = os.path.splitext(cmdFiltered) # Remove the extension from the file name
    cmdFiltered = file_read_to_string(cmdFilePath)
    msg = "[Command received:]\n" + "[" + cmdFiltered + "]"
    ui_popup(msg)
    log_file_write(msg)
    # Check if cmd received is in the set of commands     
    for cmd in commands_list:
        # Searching for " + cmd + " within " + cmdFiltered
        if cmd in cmdFiltered:
            cmdFiltered = cmd
    log_file_write("cmd filtered: " + cmdFiltered)    
    # Check if command is known and return it
    if cmdFiltered in commands_list:
        element_index = commands_list.index(cmdFiltered)
        # Check if the index is within the range of set2
        if 0 <= element_index < len(methods_list):
            method = methods_list[element_index]
        else:
            method = "sleep" # Default method that does nothing       
        #log_file_write("method=" + method)        
        return method
    else:
        return "command_waiting_for_commands" # Method that does nothing
##########################################################################################################
def command_remove(file_path):
    try:
        if os.path.isfile(file_path):
            os.remove(file_path)
            msg = "File removed:\n" + file_path 
            log_file_write(msg)            
        else:
            msg = "Path is not a file, and will not be removed:\n" + file_path
            log_file_write(msg)             
    except Exception as e:
        msg = "Error on removing file:\n" + file_path        
        log_file_write(msg)
##########################################################################################################
def file_read_to_string(file_path):
    try:
        with open(file_path, "r") as file:
            file_content = file.read()
        log_file_write("Command file content:" + file_content)
        return file_content
    except Exception as e:
        return str(e)
##########################################################################################################
def log_file_create():
    global log_file
    log_file_name = "Sikulix_" + log_file_time() + ".txt"
    log_file_name_complete = os.path.join(log_dir, log_file_name)
    ui_ask_to_continue("log_file_name = " + log_file_name_complete)        
    log_file = open(log_file_name_complete, "w")
    log_file_write("Sikulix log file opened")
##########################################################################################################
def log_file_time():
    current_datetime = datetime.datetime.now()
    log_file_time = current_datetime.strftime("%Y_%m_%d__%H_%M_%S")
    return log_file_time
##########################################################################################################
# !!! DO NOT CALL UI METHODS HERE, SINCE UI METHODS WRITE TO THE LOG: CIRCULAR CALL == CRASH!!!
# Adds \n to msg, and writes msg to log file
def log_file_write(msg):
    global log_file # This is not a must, since it is only read
    msg = log_file_time() + ": " + msg + "\n"
    try:
        if log_file is not None:
#            ui_popup_prio_c(msg)
            log_file.write(msg)                   
            log_file.flush()
#        else:
#            ui_popup_prio_c("Error: log file is none")     
    except Exception as e:
        do_nothing = ""    
#        ui_popup_prio_c("Error during log_file_write")
#########################################################################################################
def project_name_create():
    current_datetime = datetime.datetime.now()
    formatted_datetime = current_datetime.strftime("%Y_%m_%d_%H_%M_%S")
    file_name = formatted_datetime
    log_file_write(file_name)
    return file_name
##########################################################################################################
# Function to change the active state
def state_change(state_new):
    global state_active # Needed, since I change its value in this method
    if 0 <= state_new < len(State.__dict__): # Less precise: len(states_list):
        state_old = state_active
        state_active = state_new
        #ui_popup(str(state_to_state_hint[state_active]))        
    else:
        msg = "Error: invalid state index"
        print(msg)
    if state_active in state_to_state_name and state_old in state_to_state_name:
        log_file_write("Active state changed, from " + str(state_to_state_name[state_old]) +  "\nto " + str(state_to_state_name[state_active]) )
    else:
        log_file_write("state missing in state_to_state_name")
        log_file_write("state_active = " + str(state_active))
        log_file_write("state_old = " + str(state_old))
##########################################################################################################
# Pop up with question yes/no
def ui_ask_to_continue(textIn):
    if do_debug_prio_b:
        text = "Hi Fluented!\n" + textIn + "\nReady to continue?"
        answer = popAsk(text)
        if not answer:
          exit(1)
##########################################################################################################
def ui_popup(message):
    global popup_frame
    # Create the popup frame if it doesn't exist
    if popup_frame is None:
        popup_frame = swing.JFrame("Popup Dialog")
        popup_frame.setDefaultCloseOperation(swing.JFrame.DISPOSE_ON_CLOSE)
        popup_frame.setSize(400, 200)
    
    # Dispose the previous message's content
    for component in popup_frame.getContentPane().getComponents():
        popup_frame.getContentPane().remove(component)
    
    # Replace spaces with non-breaking spaces to preserve leading spaces
    formatted_message = message.replace(" ", "&nbsp;")
    # Center alignment does not work
    label = swing.JLabel("<html><div style='text-align: center; vertical-align: middle; line-height: 1.5;'>" + formatted_message.replace("\n", "<br>") + "</div></html>")
    label.setHorizontalAlignment(swing.SwingConstants.CENTER);    
    custom_font = Font("Arial", Font.PLAIN, 24)  # Replace with your desired font
    label.setFont(custom_font)
    # Add the label to the frame
    popup_frame.getContentPane().add(label)
    
    # Center the frame on the screen
    screen_size = Toolkit.getDefaultToolkit().getScreenSize()
    screen_width = screen_size.width
    screen_height = screen_size.height
    x_position = screen_width - popup_frame.getWidth()
    y_position = screen_height - popup_frame.getHeight() - 50 # height_offset
    popup_frame.setLocation(x_position, y_position)
    popup_frame.setVisible(True)
##########################################################################################################
# Pop up that can not be disabled, it has to be executed
# https://sikulix-2014.readthedocs.io/en/latest/interaction.html#timedpopups
def ui_popup_old(textIn):
#        popup(textIn)
    result = Do.popup(textIn, time_sleep_popup)
    if not result:
      print "user did not click ok"
##########################################################################################################
# Pop up that can be disabled, according to the priority level
def ui_popup_prio_a(textIn):
    if do_debug_prio_a:
        Do.popup(textIn, time_sleep_popup) # Timed popup
    else:
        print(textIn)
##########################################################################################################
# Pop up that can be disabled, according to the priority level
def ui_popup_prio_b(textIn):
    if do_debug_prio_b:
        Do.popup(textIn, time_sleep_popup) # Timed popup
    else:
        print(textIn)
##########################################################################################################
# Pop up that can be disabled, according to the priority level
def ui_popup_prio_c(textIn):
    if do_debug_prio_c:
        log_file_write(textIn)
        Do.popup(textIn, time_sleep_popup) # Timed popup
    else:
        print(textIn)

########################## Class ScannerCommands #############################################################
# This class encloses the functionality needed for executing the user's commands
# Code for executing Sikulix commands are in global methods: empty directories, control UI,  
#     set up the UI environment(Artec Studio close, save project), etc
class ScannerCommands(object):
##########################################################################################################
    def command_call(self, command_name):
        name_of_method = "command_" + str(command_name)
        log_file_write("Method will be started now: " + name_of_method)
        #method = getattr(self, name_of_method, lambda :'Give an input as an integer from 1 to 12')
        method = getattr(self, name_of_method, lambda:'')        
        return method()
    ##########################################################################################################               
    def command_close(self):          
        try:
            click(Pattern("1694688347503.png").targetOffset(938,-11))
            wait(time_sleep_between_ui_events)        
            click(Pattern("1694703083392.png").targetOffset(47,37)) 
            wait(time_sleep_between_ui_events)
            state_change(State.project_closed)
        except FindFailed as e:            
            log_file_write("I checked if save window was active, if it was, I closed Artec Studio anyway")        
            state_change(State.project_closed)
    ##########################################################################################################                   
    def command_cobot(self,  command_artec_to_cobot):
        # Write cobot command to commands_dir_out, and wait for aknowledge
        log_file_write("command_cobot called with parameter \n" + command_artec_to_cobot)        
        file_name = command_artec_to_cobot + ".txt"
        file_name = os.path.join(commands_dir_out, file_name)
        log_file_write("Sikulix command file will be created now\n(content is in the file name only): \n" + file_name)        
        # Open the file in write mode and write the command to it
        with open(file_name, 'w') as file:
            # Write the command into the file
            file.write(command_artec_to_cobot)        
        msg = "[Waiting for a feedback\nfrom the cobot]"
        ui_popup(msg)
        log_file_write(msg)
        searching_for_cobot_feedback_file = True
        while searching_for_cobot_feedback_file:
            # Search for a cobot finished file
            time.sleep(time_sleep_main_loop)
            for filename in os.listdir(commands_dir_out):
                file_path = os.path.join(commands_dir_out, filename)
                if os.path.isfile(file_path):
                    log_file_write("Found a feedback file:\n" + file_path)
                    # Read the contents of the file
                    with open(file_path, 'r+') as file:
                        file_contents = file.read()
                        log_file_write("Contents read from feedback file: " + file_contents)
                        # Check if "finished" is within the file contents
                        if "finished" in file_contents:
                            # xx Absurd: os.remove seems to work only with r+ and adding a write                         
                            file.write("Cobot ")
                            file.close() 
                            log_file_write("Finished keyword found, removing feedback file: " + file_path)
                            command_remove(file_path)
                            # Exit the loop after the first iteration                        
                            searching_for_cobot_feedback_file = False
    ##########################################################################################################                   
    def command_cobot_scan(self):
        self.command_cobot(command_artec_to_cobot_scan)
        state_change(State.recording_is_done)
    ##########################################################################################################                   
    def command_cobot_start(self):
        self.command_cobot(command_artec_to_cobot_start)    
        state_change(State.cobot_is_scan_start_pos)  
    ##########################################################################################################               
    def command_fusion_fast(self):          
        artec_studio_set_focus_on()                               
        log_file_write("I will click FAST FUSION")
        try:            
            wait(time_sleep_between_ui_events)            
            click(Pattern("1692980600547.png").targetOffset(92,0))
            state_change(State.fusion_fast_done)
        except FindFailed as e:
            message = "[Please wait until your last command is completed, then relaunch it]" 
            ui_popup(message) 
#            self.feedback_write(message)
    ##########################################################################################################            
    def command_pause(self):  
        try:
            artec_studio_set_focus_on()                
            wait(time_sleep_between_ui_events)            
            click(Pattern("1692980112571.png").similar(0.42).targetOffset(1,17))
            state_change(State.recording_is_paused)
        except FindFailed as e:
            # xx It may happen (I don't know how to set it up) that after click on recording, the preview window disappears
            wait(time_sleep_between_ui_events)            
            click("1692893217975.png")
            wait(time_sleep_between_ui_events)
            click(Pattern("1692980112571.png").similar(0.42).targetOffset(1,17))
            state_change(State.recording_is_paused)        
    ##########################################################################################################     
    def command_preview(self):  
        #artec_studio_opened_close()
        
        # Open Artec Studio
        ui_ask_to_continue("I will open Artec Studio now")       

#        click(Pattern("1695651253533.png").targetOffset(2,-3))
        click(Pattern("1698246106127.png").targetOffset(-48,-2))
        wait("RunWindow-3.PNG")
        type("C:\\Program Files\\Artec\\Artec Studio 16 Professional\\astudio_pro.exe" + Key.ENTER)
        wait(10)
        
        # Set focus on Artec Studio: no need, since it is surely already on focus
        #artec_studio_set_focus_on()
        wait(time_sleep_between_ui_events)
#        click("1692893217975.png")
        click(Pattern("1698248682950.png").targetOffset(-2,-20))
#        click(Pattern("1698249031150.png").similar(0.48).targetOffset(-1,-33))
#        click(Pattern("1698250549935.png").similar(0.59).targetOffset(1,-143))
        wait(5) # Wait for the scanner to open
        # Check if the scanner is turned on or not
        try:
            # Set the real time fusion flag 
            # Disabled, since the flag is not used withing this demo
            # After General Assembly a new error arises when this flag is set, and I can not find how to avoid it
            # click(Pattern("1698249361460.png").targetOffset(-53,0))
            # To do: why this GA version code does not work in 20.12.23? The image to be searched was wrong, did Artec change GUI??
            #click(Pattern("1698249875488.png").similar(0.50).targetOffset(-122,0))
            click(Pattern("1703086635002.png").similar(0.56).targetOffset(-112,-4))
            state_change(State.preview_is_on)
        except FindFailed as e:
            msg = "[Error: scanner is not turned on,\nplease turn on the scanner,\nand restart the Sikulix script]"
            log_file_write(msg)
            ui_popup(msg)
            wait(7)
            self.command_shutdown()
    ##########################################################################################################            
    def command_record(self):  
        artec_studio_set_focus_on()        
        wait(time_sleep_between_ui_events)
        log_file_write("I will click on record")
        wait(time_sleep_between_ui_events)
#        click(Pattern("1693301929810.png").similar(0.44))
        click(Pattern("1698250288054.png").similar(0.44))
        log_file_write("I will acquire some frames, waiting for Artec Studio to show the pause button")
        log_file_write("I will start the cobot scan trajectory")
        state_change(State.recording_is_on)
        self.command_cobot_scan()
        wait(7)
        state_change(State.recording_is_done)
    ##########################################################################################################               
    def command_registration_global(self):          
        artec_studio_set_focus_on()                                
        log_file_write("I will click on Global Registration")
        wait(time_sleep_between_ui_events)
        click(Pattern("1692977964659.png").targetOffset(86,-5))
        state_change(State.registration_global_done)
    ##########################################################################################################                
    def command_results(self):
        try:
            step = 0
            artec_studio_set_focus_on() 
            wait(time_sleep_between_ui_events)
#            click("1695726525252.png")
            click(Pattern("1698251946484.png").similar(0.50).targetOffset(-115,12))
            wait(time_sleep_between_ui_events)
#            click(Pattern("OpenProject.PNG").targetOffset(-8,23))
            click(Pattern("OpenProject2.PNG").targetOffset(-1,29))
#            wait(Pattern("OpenProject.PNG").targetOffset(-8,23))            
            wait(time_sleep_between_ui_events)            
            type("D:\\Banfi\\ArtecStudio\\ArtecStudio16Projects\\TB_Complete\\TB_Complete.a3d" + Key.ENTER)
            step = 1
            click(Pattern("1695727794934.png").targetOffset(57,45))
            state_change(State.mesh_shown)
        except FindFailed as e:
            if(step == 0):
                log_file_write("Open project could not be opened")
            if(step == 1):
                log_file_write("Project close question was not found")
                self.command_shutdown()
    ##########################################################################################################            
    def command_save(self):          
        artec_studio_set_focus_on()                        
        log_file_write("I will save the project")
        wait(time_sleep_between_ui_events)
#        click("1692980730666.png")
        click(Pattern("1698251946484.png").similar(0.50).targetOffset(-115,12))
        wait(time_sleep_between_ui_events)

        # See in the file system all variants that I tried...
        # Problem: I can not verify the matching amount, since this artec studio subwindow can not be opened with Sikulix "behind"
#        click(Pattern("SaveAs-6.PNG").similar(0.61))
#        click(Pattern("SaveAs-7.PNG").similar(0.49).targetOffset(-61,-51))
        click("SaveAs-8.PNG") #30 is too low, it saves in win10 help window :-D, 35/40/45/50/70 tested but img not found
        wait(time_sleep_between_ui_events)
        file_name = project_name_create()
        type(file_name + Key.ENTER)
        wait(time_sleep_between_ui_events)
        log_file_write("I will save the project as " + file_name)
        state_change(State.project_saved)
    ##########################################################################################################               
    def command_sleep(self):          
        wait(time_sleep_between_ui_events)
    ##########################################################################################################               
    def command_shutdown(self):         
        global do_shutdown # By this way, its value can be changed here, otherwise it does not happen (no warnings nor errors!)
        do_shutdown = True
        state_change(State.sikulix_is_off)
        msg = "I will stop Sikulix"
        log_file_write(msg)
        ui_popup(msg)
        wait(7)
        # exit(1) is later performed in command_do, using do_shutdown
    ##########################################################################################################            
    def command_stop(self): 
        try:
            artec_studio_set_focus_on()                
            log_file_write("I will click on stop")
            wait(time_sleep_between_ui_events)
#            click(Pattern("1692980032860.png").similar(0.33).targetOffset(118,-1))
#            click(Pattern("1698249961681.png").similar(0.50).targetOffset(121,-2))
            click(Pattern("1698251793532.png").similar(0.48).targetOffset(112,-5))            
            state_change(State.preview_is_off)
        except FindFailed as e:
            # xx It may happen (I don't know how to set it up) that after click on recording, the preview window disappears
            wait(time_sleep_between_ui_events)            
#            click("1692893217975.png")
            click(Pattern("1698251729620.png").similar(0.47).targetOffset(2,-31))            
            wait(time_sleep_between_ui_events)
#            click(Pattern("1692980032860.png").similar(0.33).targetOffset(118,-1))
            click(Pattern("1698251793532.png").similar(0.48).targetOffset(112,-5))
            state_change(State.preview_is_off) 
    ##########################################################################################################            
    def command_tools(self):  
        try:
            artec_studio_set_focus_on()                        
            log_file_write("I will click on Tools")
            wait(time_sleep_between_ui_events)
            click("1692894689216.png")
            state_change(State.tools_is_on)
        except FindFailed as e:    
            log_file_write("Error: tools icon not found") 
    ##########################################################################################################               
    def command_waiting_for_commands(self):         
         log_file_write("I am waiting for a valid command")
    ##########################################################################################################               
    def main_old(self):         

        # Main start
        log_file_write("Main started") # This will print some debug infos on in the right Sikulix screen
        command_dir_cleanup()
        command_do()
        # Main end
        
        #command_extract("[global registration]")
        #command_extract("[fine registration]")
        #test_exception()
        ui_popup("popup")
        ui_popup_prio_a("ui_popup_prio_a")
        ui_popup_prio_b("ui_popup_prio_b")
        ui_popup_prio_c("ui_popup_prio_c")
        scan_cmd = ScannerCommands()
        scan_cmd.command_shutdown()
        command_extract("D:\Banfi\cmd\cmdForArtecStudio\[shutteladown].txt")
        
        scan_cmd = ScannerCommands()
        scan_cmd.command_fusion_fast()

        ''' # Global comment start
        scan_cmd = ScannerCommands()
        scan_cmd.command_call("preview")
        scan_cmd.command_call("record")
        scan_cmd.command_call("pause")
        scan_cmd.command_call("stop")
        scan_cmd.command_call("tools")
        scan_cmd.command_call("global registration")
        scan_cmd.command_call("fast_fusion")
        scan_cmd.command_call("save")
        scan_cmd.command_call("close")
        scan_cmd.command_call("close_opened_artec_studio")
        scan_cmd.command_call("shutdown")
        exit(1)
        ''' # Global comment end
        exit(1)

############################################ Main ##############################################################
# This will print some debug infos on in the right Sikulix screen
log_file_create() # First thing to do, since most methods are recorded
command_dir_cleanup(commands_dir_in)
command_dir_cleanup(commands_dir_out)
command_do()
# Following commands are needed for launching single commands:
#scan_cmd = ScannerCommands()
#result = scan_cmd.command_cobot_start()