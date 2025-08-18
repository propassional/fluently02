# 264 command__start
# 143 command_extract
# 211  def command_call(self, command_name):
# 287  def command_robot(self):  
# 330 command_waiting_for_commands

# Open programming items: search for the string "xx"

# Debugging difficulties
# - There is no breakpoint functionality, nor stack => insert popAsk or exit calls
# - Error "Not set" is so general that it is absolutely meaningless, since only the calling method name is returned
#       For example, if you rename a method, and forget to rename its call, you will get "Not set", and no line numbers

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

# Relevant system objects: artec studio, cobot, commands, methods, ui, 
################################# Global constants #######################################################
# All commands read from this dir will be executed by this Sikulix app
commands_dir_in = "D:\Banfi\cmd\cmdForSikulix" 
commands_dir_out = "D:\Banfi\cmd\cmdFromSikulix"
# [SenderOfTheCommand] Subject Verb ObjectComplement
command_artec_to_cobot_start = "[artec] cobot goto top_scan_position" # Aknoledge expected: same string + "finished"
#command_artec_to_cobot_scan = "[artec] cobot goto all_scan_positions" # Aknoledge expected: same string + "finished"
command_artec_to_cobot_scan = "[artec] cobot goto top_scan_all_positions" # Aknoledge expected: same string + "finished"

# This list corresponds to all defined vocal commands, it is so formatted to correspond to the method_list
# This list matches all defined voice commands and is formatted to match the method_list
commands_list = ["cobot scan", "artec start", "artec pause", "artec stop", "artec tools", "artec global registration", 
                "fast fusion", "artec save", "artec close", "shut",     "sleep", "artec open result", "cobot start"]
# This list corresponds to all methods called by the corresponding commands
methods_list =  ["record",     "preview",     "pause",       "stop",        "tools",       "registration_global",              
                "fusion_fast", "save",       "close",       "shutdown", "sleep", "results",            "cobot_start"]
states_list = ["artec_studio_is_off", "preview_is_on", "cobot_is_scan_start_pos", "recording_is_on", 
               "tools_is_on", "global_registration_done", "project saved", "mesh view"] # 8 states
# List of available commands, depending on the current state, which can be shown to the user as a hint
hints_list = ["artec start", "cobot start", "cobot scan", "artec stop",
                "artec tools", "artec global registration", "artec save", "artec open results"] 
'''
commands_for_states = {
    "all_devices_are_off": ["x"], 
    "artec_studio_is_off": ["artec start"],
    "preview_is_on": ["cobot start" ], 
    "cobot_is_scan_start_pos": ["artec record"],    
    "recording_is_on": ["artec stop"],
    "recording_is_done": ["artec tools"],
    "tools_is_on": ["artec global registration"],
    "global_registration_done": ["artec save"],
    "project saved": ["artec open results"],
    "mesh view": ["artec close"],
}
'''
# Debug levels
do_debug_prio_a = True # False # Light level debugging, just some main messages and interactions
do_debug_prio_b = True # False # Medium level debugging, multiple main messages and interactions
do_debug_prio_c = True # False # Heavy level debugging, a lot of messages and interactions
'''
do_debug_prio_a = False # False # Light level debugging, just some main messages and interactions
do_debug_prio_b = False # False # Medium level debugging, multiple main messages and interactions
do_debug_prio_c = False # False # Heavy level debugging, a lot of messages and interactions
'''
time_sleep_main_loop = 4 # [seconds] sleep time between consecutive reads from the command dir: 0.1 = 100 ms
time_sleep_between_ui_events = 4 # [seconds] sleep time between consecutive automatic UI events
time_sleep_popup = 4 # [seconds]
time_sleep_wait_for_cobot_feedback = 4 # [seconds]
################################# Global variables #######################################################
do_shutdown = False # Set by the corresponding method
state_active_nr = 0  # Start with the first state (State1)
##########################################################################################################
# Check if one or more other Artec Studio windows are already opened, and close them before starting the Sikulix app
def artec_studio_opened_close():
#    while True: # While is needed for the use case of having to close multiple Artec Studio instances
    try:  
        artec_studio_scanning_stop()
        artec_studio_project_save()
    except FindFailed as e:
        # Good, no opened Artec Studio window has been found
        # Good, no scan was active, so the window closed immediately without the need to stop scan    
        ui_popup_prio_c("I checked if an Artec Studio instance was opened, in that case I closed it")
##########################################################################################################
def artec_studio_project_save(): 
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
        ui_popup_prio_c("I checked if save window was active, if it was, I closed Artec Studio anyway")        
##########################################################################################################
def artec_studio_scanning_stop(): 
    try:
        # If Artec Studio is not open, following command fails, then except is activated, and nothing is done       
#        click(Pattern("1693294926970.png").similar(0.88))                       
        wait(time_sleep_between_ui_events)
        ui_popup_prio_c("I detected an opened Artec Studio\nMay I close it now?")        
        click(Pattern("1694688347503.png").targetOffset(938,-11))
        wait(time_sleep_between_ui_events)
        click(Pattern("SureToClose.PNG").targetOffset(51,38))
        wait(time_sleep_between_ui_events)    
    except FindFailed as e:            
        ui_popup_prio_c("I checked if scanning was active, if it was, I closed Artec Studio anyway")        
##########################################################################################################    
def artec_studio_set_focus_on():
    try:
        # If Artec Studio is already maximized, click on top of it
        # WARNING: don't include the part "Artec Studio 16 Professional", since it disappears if a project is opened!
        click("1695730068225.png")
    except FindFailed as e:
        # If Artec Studio is not already maximized, click on its icon on the app bar
        click("1693294926970.png")
##########################################################################################################
def command_dir_cleanup(commands_dir_in):
    try:
        # List all files in the directory
        files = os.listdir(commands_dir_in)
        for file_name in files:
            file_path = os.path.join(commands_dir_in, file_name)
            if os.path.isfile(file_path):
                ui_popup_prio_c("I will remove all files in the command dir before starting:\n deleting " + file_path)
                os.remove(file_path)
            else:                
                ui_popup_prio_b("I will NOT delete this path, since it is not a file:" + file_path)
    except Exception as e:
        ui_popup_prio_a("Error: ")
        ui_popup_prio_a(e)    
##########################################################################################################
# Read from file system and execute command
def command_do():
    try:       
        # This loop is neverending, but it can be stopped by reading the command "shutdown", that runs exit(0)            
        while True: 
            # List all files found in the command directory
            files = os.listdir(commands_dir_in)
            ui_popup_prio_c("Seeking for command files in dir: " + commands_dir_in)
            ui_popup(commands_available())
            for file_name in files:
                file_path = os.path.join(commands_dir_in, file_name)
                ui_popup_prio_c("Command file found: " + file_path)
                command = command_extract(file_path)
                # Transform a string into a method call, and call it (lambda expression)                
                if(command != ""):
                    ScannerCommands().command_call(command)
                else:
                    ui_popup_prio_b("I received an empty command, so I will do nothing :-D")
                # Delete cmd file
                if os.path.isfile(file_path):
                    ui_popup_prio_c("I will delete last command received, since I have already executed it:\n" + file_path)
                    # Guarantee that the following remove file is correctly limited:
                    # https://docs.python.org/3/library/os.html#os.remove can not remove directories, so it is sufficient to check
                    # if file_path is a sub dir of directory and is a .txt file
                    file_path_dir = os.path.dirname(file_path) # Remove the file name from the path
                    if file_path_dir.startswith(commands_dir_in) and file_path.endswith('.txt'):
                            os.remove(file_path)
                    else:
                        ui_ask_to_continue("I should delete this file, but I can't since the path is not correct:\n" + file_path)
                else:
                    ui_popup_prio_b("I will NOT delete this path, since it is not a file:" + file_path)
                if do_shutdown:
                    ui_popup_prio_a("I will stop Sikulix now")
                    exit(1)
            time.sleep(time_sleep_main_loop)
    except Exception as exceptionError:
        ui_popup_prio_a("command_do: following error occurred:") 
        ui_popup_prio_a(exceptionError)
##########################################################################################################
# Extract the command name from the command file contents
# Check if the command is available
def command_extract(cmdFilePath):
#    cmdFiltered = cmdFilePath.replace(os.path.dirname(cmdFilePath), '') # Remove the path from the file name
#    cmdFiltered, extension = os.path.splitext(cmdFiltered) # Remove the extension from the file name
    cmdFiltered = file_read_to_string(cmdFilePath)
    #ui_popup_prio_c("cmd full path received: " + cmdFilePath)    
    ui_popup_prio_c("cmd received: " + cmdFiltered)
    # Check if cmd received is in the set of commands     
    for cmd in commands_list:
        #ui_popup_prio_c("Searching for " + cmd + " within " + cmdFiltered)
        if cmd in cmdFiltered:
            cmdFiltered = cmd
    ui_popup_prio_b("cmd filtered: " + cmdFiltered)    
    # Check if command is known and return it
    if cmdFiltered in commands_list:
        element_index = commands_list.index(cmdFiltered)
        # Check if the index is within the range of set2
        if 0 <= element_index < len(methods_list):
            method = methods_list[element_index]
        else:
            method = "sleep" # Default method that does nothing       
        #ui_popup_prio_a("method=" + method)        
        return method
    else:
        return "command_waiting_for_commands" # Method that does nothing
##########################################################################################################
def commands_available():
    stateComment = hints_list[state_active_nr]    
    hint = "Commands available now: " + stateComment
    return hint
##########################################################################################################
def file_read_to_string(file_path):
    try:
        with open(file_path, "r") as file:
            file_content = file.read()
        ui_popup_prio_c("Command file content:" + file_content)
        return file_content
    except Exception as e:
        return str(e)
##########################################################################################################
def project_name_create():
    current_datetime = datetime.datetime.now()
    formatted_datetime = current_datetime.strftime("%Y_%m_%d_%H_%M_%S")
    file_name = formatted_datetime
    ui_popup_prio_c(file_name)
    return file_name
##########################################################################################################
def state_active():
    return states_list[state_active_nr] 
##########################################################################################################
# Function to change the active state
def state_change(new_state_index):
    global state_active_nr
    if 0 <= new_state_index < len(states_list):
        state_active_nr = new_state_index
        print("Active state is now:", states_list[state_active_nr])
    else:
        print("Invalid state index")
##########################################################################################################
# Pop up with question yes/no
def ui_ask_to_continue(textIn):
    if do_debug_prio_b:
        text = "Hi Fluented!\n" + textIn + "\nReady to continue?"
        answer = popAsk(text)
        if not answer:
          exit(1)
##########################################################################################################
# Pop up that can not be disabled, it has to be executed
# https://sikulix-2014.readthedocs.io/en/latest/interaction.html#timedpopups
def ui_popup(textIn):
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
        ui_popup_prio_c(name_of_method + " will be started now")
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
        except FindFailed as e:            
            ui_popup_prio_c("I checked if save window was active, if it was, I closed Artec Studio anyway")        
    ##########################################################################################################                   
    def command_cobot_scan(self):
        # Write cobot command to commands_dir_out, and wait for aknowledge
        file_name = command_artec_to_cobot_scan + ".txt"
        ui_popup_prio_c("Sikulix command file name: \n" + file_name)                
        file_name = os.path.join(commands_dir_out, file_name)
        ui_popup_prio_c("Sikulix command file will be created now: \n" + file_name)        
        command = command_artec_to_cobot_scan
        # Open the file in write mode and write the command to it
        with open(file_name, 'w') as file:
            file.write(command)        
        file_name_cobot_feedback = os.path.join(commands_dir_out, command_artec_to_cobot_scan)
        file_name_cobot_feedback = file_name_cobot_feedback + "_finished.txt"        
        ui_popup_prio_c("Sikulix feedback file name: \n" + file_name_cobot_feedback)                        
        ui_popup("Waiting for a feedback from the cobot")
        time.sleep(time_sleep_wait_for_cobot_feedback)
        for filename in os.listdir(commands_dir_out):
            file_path = os.path.join(commands_dir_out, filename)
            if os.path.isfile(file_path):
                ui_popup_prio_c("Found a feedback file:\n" + file_path)
                # Read the contents of the file
                with open(file_path, 'r+') as file:
                    file_contents = file.read()
                    ui_popup_prio_c("Contents read from feedback file: " + file_contents)
                    # Check if "finished" is within the file contents
                    if "finished" in file_contents:
                        file.write("ciao")
                        file.close() 
                        ui_popup_prio_c("Found 'finished' in feedback file, now I will remove it:\n" + file_path)                            
                        # xx Absurd: os.remove seems to work only with r+ and adding a write 
                        os.remove(file_path)
                        state_change(2)   
    ##########################################################################################################                   
    def command_cobot_start(self):
        # Write cobot command to commands_dir_out, and wait for aknowledge
        file_name = command_artec_to_cobot_start + ".txt"
        ui_popup_prio_c("Sikulix command file name: \n" + file_name)                
        file_name = os.path.join(commands_dir_out, file_name)
        ui_popup_prio_c("Sikulix command file will be created now: \n" + file_name)        
        command = command_artec_to_cobot_start
        # Open the file in write mode and write the command to it
        with open(file_name, 'w') as file:
            file.write(command)        
        file_name_cobot_feedback = os.path.join(commands_dir_out, command_artec_to_cobot_start)
        file_name_cobot_feedback = file_name_cobot_feedback + "_finished.txt"        
        ui_popup_prio_c("Sikulix feedback file name: \n" + file_name_cobot_feedback)                        
        ui_popup("Waiting for a feedback from the cobot")
        time.sleep(time_sleep_wait_for_cobot_feedback)
        for filename in os.listdir(commands_dir_out):
            file_path = os.path.join(commands_dir_out, filename)
            if os.path.isfile(file_path):
                ui_popup_prio_c("Found a feedback file:\n" + file_path)
                # Read the contents of the file
                with open(file_path, 'r+') as file:
                    file_contents = file.read()
                    ui_popup_prio_c("Contents read from feedback file: " + file_contents)
                    # Check if "finished" is within the file contents
                    if "finished" in file_contents:
                        file.write("ciao")
                        file.close() 
                        ui_popup_prio_c("Found 'finished' in feedback file, now I will remove it:\n" + file_path)                            
                        # xx Absurd: os.remove seems to work only with r+ and adding a write 
                        os.remove(file_path)
                        state_change(2)
##########################################################################################################               
    def command_help(self):        
        ui_popup_prio_c("command_help\n")
        # Find out which is the current state
        # Display all commands available in this state      
    ##########################################################################################################               
    def command_fusion_fast(self):          
        artec_studio_set_focus_on()                               
        ui_popup_prio_b("I will click FAST FUSION")
        try:            
            wait(time_sleep_between_ui_events)            
            click(Pattern("1692980600547.png").targetOffset(92,0))
        except FindFailed as e:
            message = "Please wait until your last command is completed, then relaunch it" 
            ui_popup(message) 
#            self.feedback_write(message)
    ##########################################################################################################            
    def command_pause(self):  
        try:
            artec_studio_set_focus_on()                
            wait(time_sleep_between_ui_events)            
            click(Pattern("1692980112571.png").similar(0.42).targetOffset(1,17))
        except FindFailed as e:
            # xx It may happen (I don't know how to set it up) that after click on recording, the preview window disappears
            wait(time_sleep_between_ui_events)            
            click("1692893217975.png")
            wait(time_sleep_between_ui_events)
            click(Pattern("1692980112571.png").similar(0.42).targetOffset(1,17))
        
    ##########################################################################################################     
    def command_preview(self):  

        #artec_studio_opened_close()
        
        # Open Artec Studio
        ui_ask_to_continue("I will open Artec Studio now")       
#        click("1692968770482.png")
        click(Pattern("1695651253533.png").targetOffset(2,-3))
        wait("RunWindow-3.PNG")
        type("C:\\Program Files\\Artec\\Artec Studio 16 Professional\\astudio_pro.exe" + Key.ENTER)
        wait(10)
        
        # Set focus on Artec Studio
        artec_studio_set_focus_on()
        wait(time_sleep_between_ui_events)
        click("1692893217975.png")
        wait(5) # Wait for the scanner to open
        state_change(1)
        # Check if the scanner is turned on or not
        try:
            click(Pattern("1695296451636.png").similar(0.72).targetOffset(-51,-2))
            click(Pattern("1692980032860.png").similar(0.41).targetOffset(-121,0))
            state_change(1)
        except FindFailed as e:
            ui_popup("Error: scanner is not turned on, please turn on the scanner, and restart the Sikulix script") 
            exit(1)
    ##########################################################################################################            
    def command_record(self):  
        artec_studio_set_focus_on()        
        wait(time_sleep_between_ui_events)
        ui_popup_prio_b("I will click on record")
        wait(time_sleep_between_ui_events)
        click(Pattern("1693301929810.png").similar(0.44))
        ui_popup_prio_b("I will acquire some frames, and wait for the button to appear")
        ui_popup_prio_b("I will start the cobot scan trajectory")            
        self.command_cobot_scan()
        wait(7)
        state_change(3)
    ##########################################################################################################               
    def command_registration_global(self):          
        artec_studio_set_focus_on()                                
        ui_popup_prio_b("I will click on Global Registration")
        wait(time_sleep_between_ui_events)
        click(Pattern("1692977964659.png").targetOffset(86,-5))
        state_change(6)
    ##########################################################################################################                
    def command_results(self):
        try:
            step = 0
            artec_studio_set_focus_on() 
            click("1695726525252.png")
#            wait(time_sleep_between_ui_events)
            click(Pattern("OpenProject.PNG").targetOffset(-8,23))
#            wait(Pattern("OpenProject.PNG").targetOffset(-8,23))            
#            wait(time_sleep_between_ui_events)            
            type("D:\\Banfi\\ArtecStudio\\ArtecStudio16Projects\\TB_CompleteAlmost\\TB_CompleteAlmost.a3d" + Key.ENTER)
            step = 1
            click(Pattern("1695727794934.png").targetOffset(57,45))
            self.command_shutdown()
        except FindFailed as e:
            if(step == 0):
                ui_popup_prio_c("Open project could not be opened")
            if(step == 1):
                ui_popup_prio_c("Project close question was not found")
                self.command_shutdown()
    ##########################################################################################################            
    def command_save(self):          
        artec_studio_set_focus_on()                        
        ui_popup_prio_b("I will save the project")
        wait(time_sleep_between_ui_events)
        click("1692980730666.png")
        wait(time_sleep_between_ui_events)
        click("SaveAs-2.PNG")
        wait(time_sleep_between_ui_events)
        file_name = project_name_create()
        type(file_name + Key.ENTER)
        wait(time_sleep_between_ui_events)
        ui_popup_prio_b("I will save the project as " + file_name)
        state_change(7)
    ##########################################################################################################               
    def command_sleep(self):          
        wait(time_sleep_between_ui_events)
    ##########################################################################################################               
    def command_shutdown(self):         
        do_shutdown = True
        ui_popup_prio_a("Sikulix will stop now")
        exit(1)
    ##########################################################################################################            
    def command_stop(self): 
        try:
            artec_studio_set_focus_on()                
            ui_popup_prio_b("I will click on stop")
            wait(time_sleep_between_ui_events)
            click(Pattern("1692980032860.png").similar(0.33).targetOffset(118,-1))
            state_change(4)
        except FindFailed as e:
            # xx It may happen (I don't know how to set it up) that after click on recording, the preview window disappears
            wait(time_sleep_between_ui_events)            
            click("1692893217975.png")
            wait(time_sleep_between_ui_events)
            click(Pattern("1692980032860.png").similar(0.33).targetOffset(118,-1))
            state_change(4) 
    ##########################################################################################################            
    def command_tools(self):  
        try:
            artec_studio_set_focus_on()                        
            ui_popup_prio_b("I will click on Tools")
            wait(time_sleep_between_ui_events)
            click("1692894689216.png")
            state_change(5)
        except FindFailed as e:    
            ui_popup_prio_b("Error: tools icon not found") 
    ##########################################################################################################               
    def command_waiting_for_commands(self):         
         ui_popup_prio_a("I am waiting for a valid command")
    ##########################################################################################################               
    def main_old(self):         

        # Main start
        ui_popup_prio_b("Main started") # This will print some debug infos on in the right Sikulix screen
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
welcome_string = "debugA = " + str(do_debug_prio_a) + "\ndebugB = " + str(do_debug_prio_b) + "\ndebugC = " + str(do_debug_prio_c)
ui_ask_to_continue(welcome_string)
command_dir_cleanup(commands_dir_in)
command_dir_cleanup(commands_dir_out)
# Following commands are needed for launching single commands:
#scan_cmd = ScannerCommands()
#result = scan_cmd.command_cobot_start()
command_do()