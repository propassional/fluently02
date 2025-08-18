import os.path
import queue
import time
import FLMouseKeyboardControl  # xxx Importing this item blocks sg.popup_get_folder: always?
import importlib.util
import win32gui
import sys
from FLBoolean import bool_eval_and_with_none

from FLBaseClass import FLBaseClass
from FLConstants import SM_COMMAND_IMAGES, SM_FILES_TEXTS, CV2_MATCHING_SCORE_SUFFICIENT, SM_FEEDBACK_TOOLS, ERROR, \
    COMMAND_TEXT_NEW_FILE_NAME_WITH_DATE, DATA_OUTPUT, COMMAND_TEXT_DATA_OUTPUT_DIRECTORY, UI_LOGORRHOIC, \
    MODULES_RELOAD_ALWAYS
import FLGUIMain
from FLLogger import FLLogger
from FLCV2 import FLCV2
from FLOS import path_extract_app_name, path_remove_extension, path_extract_filename, path_join, datetime_now_get_string
from FLString import extract_name_short


class FLModel(FLBaseClass):
    def __init__(self, logger, name, code_path_not_used_for_the_moment):
        super().__init__(logger)
        self.name = name
        self.name_short = extract_name_short(name)
        self.code_path = code_path_not_used_for_the_moment
        self.image_analyzer = FLCV2(logger)
        # Dictionary created from all user defined transitions, associated with True or False for firing them
        self.transition_semaphores = {}
        # Dictionary of all states that have no transitions to any other states
        self.states_bounce = {}
        # Transitions history
        self.bounced_from_state = ""
        self.jumped_from_state = ""
        self.trigger_current = None
        self.trigger_last = None
        self.trigger_last_last = None
        # History of all states traversed
        self.state_history = {}
        self.state_after_transitioning = ""
        self.state_before_transitioning = ""
        self.rollback_do = False
        self.GUI_update_requested = False
        self.image_GUI_path = "" # Image belonging to the SM tab

    def bounce_to_state(self, state_name):
        try:
            # The getattr() function looks for an attribute named "to_state_name" in the self.machine.model object
            # If such an attribute exists, it returns the corresponding method
            self.print("Bouncing to state", state_name)
            transition_method = getattr(self, f"to_{state_name}")
            if self.bounced_from_state != "":
                self.print(f"bounced_from_state = {self.bounced_from_state}")
                self.bounced_from_state = ""  # We are doing a back jump (a bounce), thus remove this jumped status
                self.print(f"bounced_from_state = {self.bounced_from_state}")
            else:
                self.print(f"bounced_from_state = {self.bounced_from_state}")
                self.bounced_from_state = self.state  # Jump, remember from which state we jumped
                self.print(f"bounced_from_state = {self.bounced_from_state}")
            transition_method()  # Fire the to_state transition
        except Exception as error:
            self.print("Error doing jump", ERROR)

    # This method gathers all tasks that can be done triggering a transition
    # 1) Do transition:
    #   a) Search for the command image related to the name of the last transition done,
    #   for example the image "CmdStartArtecStudio.png" located in D:\Banfi\Github\Fluently\Releases\2024_04_09_Teachix\Data\Input\CommandImages\Taskbar
    #   b) Identify which app to apply the command image
    #   Easy to do, since the image is in the folder \Taskbar, the taskbar is the searched app
    #   c) Set this app on focus clicking on it on the taskbar, then search where the command image is located, then click on this position
    # 2) Do feedback:
    #   a) Check feedback image, feedback code
    def callback_after_state_change(self):

        self.print(f"\n At callback beginning: current state {self.state}, rollback_do {self.rollback_do}")

        # If doing rollback just do nothing
        if not self.rollback_do:
            # Reset global variables
            self.global_variables_reset()

            # 1. Do "Transition Do", execute commands:
            self.do_transition_result = self.do_transition()

            # 2. Do "State Check"
            self.check_state_result = self.check_state()

            # Check if the current state can be considered "valid"
            # Correct the "Pytonata": "None and False" returns None and not False! See discussion in ExpBoolean
            self.state_is_valid = bool_eval_and_with_none(self.do_transition_result, self.check_state_result)

            # The second part is mainly for the Autostart transition, but not only
            # self.state_is_valid = None must be converted into True
            self.state_is_valid_final = self.state_is_valid or self.state_is_valid == None

            self.global_variables_print()

            # 4. Evaluate rollback
            # Check code or check images can confirm if this state is ok
            # If no code nor image are defined, the state is ok per default
            if not self.state_is_valid_final:
                # Check if the method is processing a transition or a jump
                if self.bounced_from_state == "":
                    # 4. Do "Transition rollback"
                    # If it is not a jump, do transition rollback
                    self.run_code_rollback()
                    self.print(
                        f"Do transition rollback, from current state {self.state} to {self.state_before_transitioning}")
                    self.rollback_state(self.state_before_transitioning)
            else:
                # 3. Do "State Do"
                self.run_code_state()

            # Store all fired transitions
            # if len(self.state_history) >= 1:
            #     self.transitions[self.state] = self.state_history[-1]
            # self.state_history.append(self.state) # Bug is here
            # self.log_file_path.write("updated transitions: " + str(self.transitions))
            # self.log_file_path.write("updated state history: " + str(self.state_history))
        else:
            self.print("No callback code is executed, since this callback was called by a state rollback")
            self.rollback_do = False
            self.print(f"Reset rollback_do, which now is {self.rollback_do}")

        # If jumped, now jump back
        if self.bounced_from_state != "":
            self.print("Since this state has been reached by bouncing, now do rollback")
            self.rollback_do = True
            self.print(f"Set rollback_do, which now is {self.rollback_do}")
            self.bounce_to_state(self.bounced_from_state)

        # At the end of the callback we have to update the graph of the GUI
        self.GUI_update_requested = True

        self.print("\n At callback end, current state: " + self.state)

    # xxx This method is not used,
    # since for the moment I am not able to define a graph machine having two callbacks, nor to define a callback dynamically
    # There is the trick to define a callback for all states, but it does not seem to work
    def callback_on_enter_state(self):
        pass

    def check_state(self):
        try:
            # Check code
            # Search for feedback code, run it if you find some
            self.print("STATE CHECK CODE")
            check_method_name = self.state + 'Check'
            check_code_path = path_remove_extension(self.name) + '.py'
            self.check_code_found, self.check_code_result = self.run_python_code(check_code_path,
                                                                                   check_method_name)  # Blocking
            # result may be a boolean or a path to an image to be loaded into main GUI
            if isinstance(self.check_code_result, bool) or self.check_code_result is None:
                self.check_code_tells_state_is_valid = self.check_code_result
            elif os.path.exists(self.check_code_result):
                self.image_GUI_path = self.check_code_result
                self.check_code_tells_state_is_valid = True  # Since the path to the image is valid
            else:
                self.check_code_tells_state_is_valid = False  # Since the path is not valid

            self.print(f"feedback_code_tells_state_is_valid is {self.check_code_tells_state_is_valid}")

            # feedback_code_name = self.state + 'Check.py'
            # feedback_code_path_list, app_name_list = self.items_find_in_dir(feedback_code_name, SM_FEEDBACK_TOOLS)
            # for feedback_code_path, app_name in zip(feedback_code_path_list, app_name_list):
            #     self.check_code_found = True
            #     self.print(file"check_code_found: {self.check_code_found}")
            #     self.check_code_tells_state_is_valid = self.run_python_code(feedback_code_path)
            #     self.print(file"check_code_tells_state_is_valid is {self.check_code_tells_state_is_valid}")

            # Check images
            # Search for and run feedback images if you find some
            app_name = ""  # The name of the app to be used, defined by the user
            feedback_images_path = ""  # The path to the command image to be used, defined by the user on file system
            self.print("CHECK IMAGES")
            # If no images will be found, it stays None: stay uncertain is correct
            self.check_image_tells_state_is_valid = None
            feedback_image_name = self.state + 'Check.png'
            feedback_images_path, app_names = self.items_find_in_dir(feedback_image_name, SM_FEEDBACK_TOOLS)
            # Use all found feedback images and feedback apps
            for feedback_image_path_item, app_name_item in zip(feedback_images_path, app_names):
                if (app_name_item != "" and feedback_image_path_item != ""):
                    self.check_image_found = True
                    self.print(
                        f"check_image_found: {self.check_image_found}, feedback_image_path_item {feedback_image_path_item} , app_name_item {app_name_item}")
                    self.focus_set_on_app(app_name_item)
                    mouse_x, mouse_y, matching_score = self.image_analyzer.screenshot_search(feedback_image_path_item,
                                                                                             app_name_item)
                    if matching_score < CV2_MATCHING_SCORE_SUFFICIENT:
                        # Record both values at the same time, since they are related
                        self.check_image_tells_state_is_valid = False
                        self.check_image_result = matching_score
                    else:
                        # This logic is for multiple image cases: False can never be overwritten by a True, so it is never lost
                        if self.check_image_tells_state_is_valid != False:
                            # Record both values at the same time, since they are related
                            self.check_image_tells_state_is_valid = True
                            self.check_image_result = matching_score
                    self.print(
                        f"feedback_image_tells_state_is_valid {self.check_image_tells_state_is_valid}, matching_score {matching_score}, matching threshold {CV2_MATCHING_SCORE_SUFFICIENT}")

            result = bool_eval_and_with_none(self.check_code_tells_state_is_valid, self.check_image_tells_state_is_valid)
            return result
        except Exception as error:
            self.print("STATE CHECK CODE error", ERROR)
            return False

    def do_code(self):
        try:
            # Try to run command code, returns False if code not found
            self.print("DO CODE")
            do_method_name = self.trigger_current + 'Do'
            do_code_path = path_remove_extension(self.name) + '.py'
            self.do_code_found, self.do_code_result = self.run_python_code(do_code_path, do_method_name)  # Blocking
            if self.do_code_found == False:
                self.do_code_tells_state_is_valid = None
            else:
                self.do_code_tells_state_is_valid = bool_eval_and_with_none(self.do_code_found, self.do_code_result)

            return self.do_code_tells_state_is_valid
            # self.print(file"do_code_path return is {do_code_result}")
            # do_code_path_list, app_name_list = self.items_find_in_dir(do_code_name, SM_FEEDBACK_TOOLS)
            # for do_code_path, app_name in zip(do_code_path_list, app_name_list):
            #     self.print(file"do_code_path: {do_code_path}")
            #     do_code_result = self.run_python_code(do_code_path) # Blocking
            #     self.print(file"do_code_path return is {do_code_result}")
        except Exception as error:
            self.print("DO CODE error", ERROR)

    def do_images(self):
        # We do not use True or False here, since the uncertainty given by "None" is the best fit at this level,
        # since we did not even try to find images or match images
        images_found = images_matched = error_happened = None
        try:
            # Do the required transition steps, defined by the corresponding SM command images and SM command texts
            # Search for the command images, whose names are based on the state name
            # Then search for the name of the app to be used, defined by the user
            self.print("DO IMAGES")
            app_name = ""
            command_image_path = ""  # Path to the command image to be used, defined by the user in the file system
            command_image_name = self.trigger_current + 'Do.png'
            command_image_path_list, app_name_list = self.items_find_in_dir(command_image_name, SM_COMMAND_IMAGES)
            #if (command_image_path_list != None):
            if not command_image_path_list:
                self.print("No do images found")
                images_found = False
                images_matched = None
                return images_found, images_matched, error_happened
            else:
                images_found = True
            for command_image_path, app_name in zip(command_image_path_list, app_name_list):
                # Set focus on app
                self.focus_set_on_app(app_name)
                # Do a screenshot, load the template image, search for the template image within the screenshot image
                mouse_x, mouse_y, matching_score = self.image_analyzer.screenshot_search(command_image_path, app_name)
                if matching_score <= CV2_MATCHING_SCORE_SUFFICIENT:
                    images_matched = False
                else:
                    # Once false, it has to stay false (for remembering it was false)
                    # but if it is still None, it can become True
                    if images_matched != False:
                        images_matched = True
                # Prepare to click on app
                # Find command image center, and if requested also enter some text
                self.print(
                    f"call screenshot_search with command_image_path {command_image_path}, app_name {app_name}")
                self.print(
                    f"screenshot_search returns mouse_x {mouse_x}, mouse_y {mouse_y}, matching_score {matching_score}")
                # command_text_name = os.path.join(SM_FILES_TEXTS, self.trigger_last + '.txt')
                # command_text_name = self.trigger_last + '.txt'
                command_text_name = path_extract_filename(command_image_path)
                command_text_name += '.txt'
                command_text_path_list, command_text_app_list = self.items_find_in_dir(command_text_name,
                                                                                       SM_FILES_TEXTS)
                # Read from file system if SM_FILES_TEXTS contains SM_FILES_TEXTS to be entered
                command_text_path = command_text_path_list[0] if command_text_path_list else ""
                if os.path.exists(command_text_path):
                    with open(command_text_path, 'r') as file:
                        # Text + Click
                        #command_text = file.read().strip() # Removes spaces before and after
                        command_text = file.read().rstrip()
                        FLMouseKeyboardControl.clickAndWriteTextAndEnter(mouse_x, mouse_y, matching_score, command_text)
                        self.print(
                            f"clickAndWriteTextAndEnter on mouse_x {mouse_x}, mouse_y {mouse_y}, matching_score {matching_score}, write command_text {command_text}")
                else:
                    try:
                        self.print(f"click on mouse_x {mouse_x}, mouse_y {mouse_y}")
                        FLMouseKeyboardControl.click(mouse_x, mouse_y)
                    except Exception as error:
                        self.print(f"EXCEPTION error {error}", ERROR)
                        self.print(
                            f"context of the error: mouse_x {mouse_x}, mouse_y {mouse_y}, matching_score {matching_score}: good result is {result}",
                            ERROR)
                        error_happened = True
                        return images_found, images_matched, error_happened
            return images_found, images_matched, error_happened
        except Exception as error:
            self.print("DO IMAGES error", ERROR)
            error_happened = True
            return images_found, images_matched, error_happened


    def do_transition(self):
        # Step1) Run code
        if self.bounced_from_state == "":
            # If a normal transition, the transition name leads
            self.trigger_current = self.trigger_last
        else:
            # If a jump happened, the state name leads
            self.trigger_current = self.state

        # Search for command code, run it if you find some
        do_code_result = self.do_code()
        # do_code_result may be a boolean or a path to an image to be loaded into main GUI
        if isinstance(do_code_result, bool) or do_code_result is None:
            self.do_code_tells_state_is_valid = do_code_result
        elif os.path.exists(do_code_result):
            self.image_GUI_path = do_code_result
            self.do_code_tells_state_is_valid = True # Since the path to the image is valid
        else:
            self.do_code_tells_state_is_valid = False # Since the path is not valid

        #if self.do_code_tells_state_is_valid:
        # Step2) Run images or run images + text
        # Search for command images, run them if you find some
        self.do_image_found, self.do_image_matched, self.do_image_error = self.do_images()
        # False and None = False, but here False and None should be = None, since no images have been found, so that the result will not be considered later
        if self.do_image_found == False:
            self.do_image_tells_state_is_valid = bool_eval_and_with_none(None, self.do_image_matched)
        else:
            self.do_image_tells_state_is_valid = bool_eval_and_with_none(self.do_image_found, self.do_image_matched)

        result = bool_eval_and_with_none(self.do_code_tells_state_is_valid, self.do_image_tells_state_is_valid)

        return result

    # Doing SaveFile, the SaveFile window is used by EBuddy for entering the save SM_JSON_data
    # Each time EBuddy enters that window, OS moves the focus onto it, so we have to reset it on Artec Studio,
    # since the images we are processing belong to Artec, not to the SaveFile window
    def focus_set_on_app(self, app_name):
        # If the required app is the Taskbar, no focusing is needed, since it is assumed that the Taskbar is always visible
        # Is the required app is not the Taskbar, check if focus is on the required app
        if (app_name != "Taskbar"):
            # Is the app already on focus?
            window_current = win32gui.GetForegroundWindow()
            window_current_name = win32gui.GetWindowText(window_current)
            window_current_name = window_current_name.replace(" ", "")
            self.print(
                f"focus is required on app {app_name}, focus is now on window {window_current_name}, window nr {window_current}")
            if app_name in window_current_name:
                self.print(f"app {app_name} is already on focus")
            else:
                self.print(f"Set focus on app {app_name}")
                # command_image_app_is_on_focus_path = os.path.join(SM_COMMAND_IMAGES, app_name, "_AppIsOnFocus.PNG") # Image when app is not in focus
                command_image_app_is_on_focus_name = "FocusIsOn" + app_name + ".png"
                command_image_path_list, app_name_list = self.items_find_in_dir(command_image_app_is_on_focus_name,
                                                                                SM_COMMAND_IMAGES)
                command_image_app_is_on_focus_path = command_image_path_list[0]
                mouse_x_focus, mouse_y_focus, matching_score_focus = self.image_analyzer.screenshot_search(
                    command_image_app_is_on_focus_path,
                    app_name)
                if (matching_score_focus > CV2_MATCHING_SCORE_SUFFICIENT):
                    FLMouseKeyboardControl.click(mouse_x_focus, mouse_y_focus)
                    self.print(
                        f"clicked on app {app_name} at mouse_x {mouse_x_focus}, mouse_y {mouse_y_focus}, matching_score_focus {matching_score_focus}")
                else:
                    # App is probably minimized
                    command_image_app_is_not_on_focus = "FocusNotOn" + app_name + ".png"
                    command_image_path_list, app_name_list = self.items_find_in_dir(command_image_app_is_not_on_focus,
                                                                                    SM_COMMAND_IMAGES)
                    command_image_app_is_not_on_focus_path = command_image_path_list[0]
                    mouse_x_focus, mouse_y_focus, matching_score_focus = self.image_analyzer.screenshot_search(
                        command_image_app_is_not_on_focus_path, "Taskbar")
                    FLMouseKeyboardControl.click(mouse_x_focus, mouse_y_focus)
                    self.print(f"clicked for setting focus to {app_name}")
                    self.print(
                        f"click position mouse_x {mouse_x_focus}, mouse_y {mouse_y_focus}, matching_score {matching_score_focus}")
                time.sleep(1)
        else:
            # xxx For simplification, I make the assumption that the taskbar is at the bottom, and always visible
            self.print("Currently this software is not able to focus on the taskbar, thus it has to be always visible")


    def global_variables_print(self):
        # self.print(file"self. {self.}")

        # Do phase
        self.print(f"self.do_code_found {self.do_code_found}")
        self.print(f"self.do_code_result {self.do_code_result}")
        self.print(f"self.do_code_tells_state_is_valid {self.do_code_tells_state_is_valid}")

        self.print(f"self.do_image_found {self.do_image_found}")
        self.print(f"self.do_image_result {self.do_image_result}")
        self.print(f"self.do_image_matched {self.do_image_matched}")
        self.print(f"self.do_image_error {self.do_image_error}")
        self.print(f"self.do_image_tells_state_is_valid {self.do_image_tells_state_is_valid}")

        self.print(f" self.do_transition_result {self.do_transition_result}")

        # Check phase
        self.print(f"self.check_code_found {self.check_code_found}")
        self.print(f"self.check_code_result {self.check_code_result}")
        self.print(f"self.check_code_tells_state_is_valid {self.check_code_tells_state_is_valid}")

        self.print(f"self.check_image_found {self.check_image_found}")
        self.print(f"self.check_image_result {self.check_image_result}")
        self.print(f"self.check_image_tells_state_is_valid {self.check_image_tells_state_is_valid}")

        self.print(f" self.check_state_result {self.check_state_result}")

        # Evaluation phase
        self.print(f"  self.state_is_valid {self.state_is_valid}")
        self.print(f"   self.state_is_valid_final {self.state_is_valid_final}")

    def global_variables_reset(self):
        # Do phase
        self.do_code_found = None
        self.do_code_result = None
        self.do_code_tells_state_is_valid = None

        self.do_image_found = None
        self.do_image_result = 0.0
        self.do_image_matched = None
        self.do_image_error = None
        self.do_image_tells_state_is_valid = None

        self.do_transition_result = None

        # Check phase
        self.check_code_found = None
        self.check_code_result = None
        self.check_code_tells_state_is_valid = None

        self.check_image_found = None
        self.check_image_result = 0.0
        self.check_image_tells_state_is_valid = None

        self.check_transition_result = None

        # Final phase
        self.transition_result = None
        self.state_is_valid = None
        self.state_is_valid_final = None

    # Finds in directory all items that have the extension of item_name, and somehow contain item_name
    # For example, if CmdStart.png is searched, possible returns are CmdStart.png, but also CmdStart01.png and CmdStart02.png
    def items_find_in_dir(self, item_name, directory):
        self.print(f"item_name searched: {item_name}, directory searched: {directory}")
        # Does this trigger have command images associated with it?
        # Search in dir and its subdirectories
        item_path_list = []
        app_name_list = []

        # Finds any item in directory, which contains command_image_name without extension
        item_name_with_no_extension, item_name_extension = os.path.splitext(item_name)  # Remove the file extension

        for root, dirs, files in os.walk(directory):
            # For each item, it checks if the string “Feedback” is not present in the item
            # files_filtered = [item for item in files if "Feedback" not in item]
            for file in files:
                file_path = os.path.join(root, file)
                # If a part of the searched image command is in the files that have been read...
                if item_name_with_no_extension in file:
                    # ...store it into command_images_path
                    item_path = file_path
                    app_name = path_extract_app_name(file_path)  # Extract the file name from the path
                    _, item_name_found_extension = os.path.splitext(os.path.basename(item_path))
                    if item_name_found_extension == item_name_extension:
                        item_path_list.append(item_path)
                        app_name_list.append(app_name)
        self.print(f"items found: item_path_list {item_path_list}, app_name_list: {app_name_list}")
        return item_path_list, app_name_list

    # def print_simple(self, string): # It worked, but it does not print the method name
    #     if self.log_file != None:
    #         self.log_file.write(string)
    # def print(self, *args, **kwargs):
    #     caller_frame = inspect.stack()[1]
    #     caller_info = inspect.getframeinfo(caller_frame[0])
    #     log_message_no_caller = [file"{arg}" for arg in args]
    #     log_message = [file"{caller_info.function}() - {arg}" for arg in args]
    #
    #     if (log_message_no_caller != self.log_message_last):
    #         # print to log file
    #         self.log_file.write(*log_message)
    #         # Standard print call
    #         builtins.print(*log_message, **kwargs)
    #         self.log_message_last = log_message_no_caller

    def jump_to_state(self, state_name):
        try:
            # The getattr() function looks for an attribute named "to_state_name" in the self.machine.model object
            # If such an attribute exists, it returns the corresponding method
            self.print("Jumping to state", state_name)
            transition_method = getattr(self, f"to_{state_name}")
            # Keep track of the jump only for debug aims
            self.jumped_from_state = self.state  # Jump, remember from which state we jumped
            transition_method()  # Fire the to_state transition
            debug_state = self.state
        except Exception as error:
            self.print("Error doing jump", ERROR)

    def run_code_rollback(self):
        try:
            # Search and execute rollback code
            self.print("ROLLBACK CODE")
            rollback_method_name = self.state + 'Rollback'
            rollback_code_path = path_remove_extension(self.name) + '.py'
            do_code_result = self.run_python_code(rollback_code_path, rollback_method_name)  # Blocking
            # rollback_code_path_list, app_name_list = self.items_find_in_dir(rollback_code_name, SM_FEEDBACK_TOOLS)
            # for rollback_code_path, app_name in zip(rollback_code_path_list, app_name_list):
            #     self.code_external_path = rollback_code_path
            # check_code_tells_state_is_valid = self.feedback_code_run(rollback_code_path)
        except Exception as error:
            self.print("Error ", ERROR)

    def run_code_state(self):
        try:
            # Search and execute state code
            self.print("STATE DO")
            state_method_name = self.state + 'Do'
            state_code_path = path_remove_extension(self.name) + '.py'
            # Search for command code, run it if you find some
            # do_code_result may be a boolean or a path to an image to be loaded into main GUI
            method_is_loadable, do_code_result = self.run_python_code(state_code_path, state_method_name)  # Blocking
            if isinstance(do_code_result, bool) or do_code_result is None:
                temp = do_code_result # Maybe useful in future
            elif os.path.exists(do_code_result):
                self.image_GUI_path = do_code_result

        except Exception as error:
            self.print("Error ", ERROR)

    def run_python_code(self, module_path, method_name="main"):
        module_is_loadable = None
        method_is_loadable = None
        module = ""
        result = None
        module_name = os.path.basename(module_path).split(".")[0]
        try:
            # Check if module has been already loaded or not
            if module_name not in sys.modules or MODULES_RELOAD_ALWAYS:
                # Load the module
                spec = importlib.util.spec_from_file_location(module_name, module_path)
                module = importlib.util.module_from_spec(spec)
                # Loads the module, executing code which is not included in methods (only this kind of code is executed),
                # reading top-level statements, class definitions and all method definitions (including __main__)
                spec.loader.exec_module(module)
                # The module is not added automatically to the system (as gpt says), so do it manually
                sys.modules[module_name] = module
                module_is_loadable = True
                self.print(f"Module loaded from file {module_path}")
            else:
                module_is_loadable = True
        except Exception as error:
            module_is_loadable = False
            self.print(f"Exception {error}, calling {module_path}, spec {spec}")

        # Execute the specified method (if it exists and the module_declared_at_top_of_the_file is loadable)
        if module_is_loadable and hasattr(sys.modules[module_name], method_name):
            try:
                # Execute the specified method
                method_is_loadable = True
                result = False
                self.print(f"Executing method {method_name} in module {sys.modules[module_name]}")
                # Run module_method_name, debug within method is possible
                #result = getattr(sys.modules[module_name], method_name)()
                queue_message = queue.Queue()
                result = getattr(sys.modules[module_name], method_name)(queue_message)
                if not queue_message.empty():
                    message = queue_message.get()
                else:
                    message = ""
                # Extracting the word between "__"
                if message != "":
                    FLGUIMain.message_GUI = self.name_short + ": " + message # FLModel has no pointer to FLGUIMain
                else:
                    FLGUIMain.message_GUI = "" # Delete previous message
                self.print(f"Executed module returned result {result}, message {message}")
                return method_is_loadable, result
            except Exception as error:
                self.print(f"Exception {error} while executing {method_name} in {module_path}")
                result = None
                return method_is_loadable, result
        else:
            # Module is not loadable
            self.print(f"Method {method_name} not found in {module_path}")
            method_is_loadable = False
            result = None
            return method_is_loadable, result

    def rollback_state(self, state_name):
        # The getattr() function looks for an attribute named "to_state_name" in the self.machine.model object
        # If such an attribute exists, it returns the corresponding method
        transition_method = getattr(self, f"to_{state_name}")
        # Prepare the correct last trigger, doing a rollback on that item as well
        self.trigger_last = self.trigger_last_last
        self.rollback_do = True  # Warn the callback that when called, nothing has to be done
        self.print(f"Set rollback_do, which now is {self.rollback_do}")
        # This will call the callback
        transition_method()

if __name__ == '__main__':
    log_file = FLLogger("ZTest")
    my_model = FLModel(log_file, "SM Nonsense Test", " ")
    # This works

    result = my_model.run_python_code(
        r"D:\Banfi\Github\Fluently\Releases\2024_07_09_EBuddy\Data\Input\SM\ArtecStudio__2024_09_07__10_22_29\JSON\ArtecStudio__2024_09_07__10_22_29.py", "ScannerIsOnRollback")

    sys.exit()

    result = my_model.run_python_code(
        r"D:\Banfi\Github\Fluently\Releases\2024_04_09_Teachix\Data\Input\FeedbackTools\ArtecStudio\ScanMyAss.py")

    result = my_model.run_python_code(
        r"D:\Banfi\Github\Fluently\Releases\2024_04_09_Teachix\Data\Input\FeedbackTools\ArtecStudio\ScannerIsActiveRollback.py")

    # result = my_model.feedback_code_run(r"D:\Banfi\Github\Fluently\Releases\2024_04_09_Teachix\Data\Input\FeedbackTools\ArtecStudio\ArtecStudioIsActiveRollback.py")
    # 'D:\\Banfi\\Github\\Fluently\\Releases\\2024_04_09_Teachix\\Data\\Input\\FeedbackTools\\ArtecStudio\\ArtecStudioIsActiveCheck.py'