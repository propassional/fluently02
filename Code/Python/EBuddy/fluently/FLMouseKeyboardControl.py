# Alternative: pwinauto
import sys

import pygetwindow
#from pywinauto.keyboard import send_keys
import pyautogui
import time

from FLString import split_string, string_is_in_reference_string
from FLConstants import CV2_MATCHING_SCORE_SUFFICIENT, COMMAND_TEXT_NEW_FILE_NAME_WITH_DATE, \
    COMMAND_TEXT_DATA_OUTPUT_DIRECTORY, DATA_OUTPUT, DATA_OUTPUT_ARTEC, COMMAND_TEXT_SPLIT_STRING, \
    COMMAND_TEXT_CONTROL_A_DELETE, COMMAND_TEXT_DO_NOT_ENTER, TIME_MOUSE_MOVE_TO
from FLOS import focus_set_on_app, datetime_now_get_string


def click(x, y):
    # Store the original mouse position
    original_position = pyautogui.position()

    # good default = 0.5 but not starting from original position
    # 0 will move istantly to the target
    pyautogui.moveTo(x, y, duration = TIME_MOUSE_MOVE_TO)
    pyautogui.click(x, y)

    # Move back to the original position
    pyautogui.moveTo(original_position.x, original_position.y, duration = TIME_MOUSE_MOVE_TO)

# Focus on app has to be done before
def clickAndWriteTextAndEnter(x, y, matching_score, text):
    do_enter = True
    # We don't want to click an unplanned spot
    if matching_score > CV2_MATCHING_SCORE_SUFFICIENT:
        # duration=0.0 works well for starting Artec studio
        # duration=0.5 works not well for starting artec studio, since if the user interacts with the mouse, it fails
        pyautogui.moveTo(x, y, duration = TIME_MOUSE_MOVE_TO)
        pyautogui.click(x, y)
        text_list = split_string(text, COMMAND_TEXT_SPLIT_STRING)
        text_to_type = ""
        for text_item in text_list:
            if string_is_in_reference_string(text_item, COMMAND_TEXT_NEW_FILE_NAME_WITH_DATE):
                file_name_with_time_stamp = datetime_now_get_string()
                text_to_type += file_name_with_time_stamp
            elif string_is_in_reference_string(text_item, COMMAND_TEXT_DATA_OUTPUT_DIRECTORY):
                text_to_type += DATA_OUTPUT_ARTEC
            elif string_is_in_reference_string(text_item, COMMAND_TEXT_CONTROL_A_DELETE):
                pyautogui.hotkey('ctrl', 'a')
                pyautogui.typewrite(['backspace'])
            elif string_is_in_reference_string(text_item, COMMAND_TEXT_DO_NOT_ENTER):
                do_enter = False
            else:
                text_to_type += text_item

        # It happened once that the text to type was auto-transformed into capital (!): restarting pycharm solved the problem
        pyautogui.typewrite(text_to_type)

        if do_enter:
            time.sleep(1)  # If removed, enter will not be pressed
            pyautogui.press('enter')
    else:
        print("clickAndWriteTextAndEnter: matching score is insufficient, no clicking is performed")

if __name__ == '__main__':
    # Notepad++ must be off, notepad must be maximized
    x0 = 9
    y0 = 65

    # Before running this code do following:
    # Unmaximize PyCharm and keep it to the right, unmaximize Notepad and keep it to the left
    focus_set_on_app("notepad")
    clickAndWriteTextAndEnter(x0, y0, 100, "Ctrl&A&Del | NewFileNameWithDate | PressNotEnter")
    clickAndWriteTextAndEnter(x0, y0, 100, "Ctrl&A&Del | DataDirectory")

    focus_set_on_app("notepad")
    clickAndWriteTextAndEnter(x0, y0, 100, "Test entering this text to the extreme left part, then wrap") # wrap = andare a capo
    time.sleep(2)

    focus_set_on_app("notepad")
    #clickAndWriteTextAndEnter(x0, y0, 100, "Test entering this text to the extreme left part, then DO NOT wrap | DoNotEnter") # Write Test, do not wrap
    clickAndWriteTextAndEnter(x0, y0, 100,"Ctrl&A&Del | NewFileNameWithDate | DoNotEnter")  # Write Test, do not wrap
    time.sleep(2)

    focus_set_on_app("notepad")
    clickAndWriteTextAndEnter(x0, y0, 100, "ctrl&a&del") # Do control-a, then delete
    clickAndWriteTextAndEnter(x0, y0, 100, r"C:\Program Files\Artec\Artec Studio 16 Professional\astudio_pro.exe")
    time.sleep(2)


    sys.exit() # Since it writes, do not let this code run without control!

    focus_set_on_app("notepad")
    clickAndWriteTextAndEnter(x0, y0, 100, "ctrl&a&del")
    clickAndWriteTextAndEnter(x0, y0, 100, r"C:\\ProgramFiles\\Artec\\Artec Studio 16 Professional\\astudio_pro.exe")
    time.sleep(2)

    focus_set_on_app("notepad")
    clickAndWriteTextAndEnter(x0, y0, 100, "ctrl&a&del")
    clickAndWriteTextAndEnter(x0, y0, 100, "ctrl&a&del | NewFileNameWithDate")
    time.sleep(2)

    focus_set_on_app("notepad")
    clickAndWriteTextAndEnter(x0, y0, 100, "ctrl&a&del")
    clickAndWriteTextAndEnter(x0, y0, 100, "ctrl&a&del | DataDirectory")
    time.sleep(2)

    focus_set_on_app("notepad")
    clickAndWriteTextAndEnter(x0, y0, 100, "ctrl&a&del")
    clickAndWriteTextAndEnter(x0, y0, 100, r"")
    time.sleep(2)

    #clickAndWriteTextAndEnter(28, 70, "") # Start Scan
    #clickAndWriteTextAndEnter(145, 357, "")  # Start Preview
    clickAndWriteTextAndEnter(300, 357, "")  # Start Record
    clickAndWriteTextAndEnter(449, 357, "")  # Start Stop

    # Center coordinates (replace with your desired position)
    center_x = 215
    center_y = 1415

    # Move the mouse cursor to the specified position
    pyautogui.moveTo(center_x, center_y, duration = TIME_MOUSE_MOVE_TO)

    # Simulate a mouse click
    #pyautogui.click()
    pyautogui.click(215, 1415);

    # Insert the given string, it does not work with links
    string_to_insert = r"C:\Program Files\Artec\Artec Studio 16 Professional\astudio_pro.exe"
    time.sleep(0.5) # This is needed, otherwise enter has no effect
    #string_to_insert = r"D:\Banfi\astudio_pro.exe"
    #string_to_insert = "Artec Studio 16 Professional" # Problem with artec studio license opening
    #string_to_insert = r"C:\Program Files\7-Zip\7zG.exe" # It works, with send_keys('{ENTER}')
    #string_to_insert = r"C:\Program Files\7-Zip\astudio_pro.exe"
    pyautogui.typewrite(string_to_insert)
    time.sleep(1)
    pyautogui.press('enter')
    #send_keys('{ENTER}') # With pywinauto
    breakpoint()

    #pyautogui.press('enter')
    pyautogui.keyDown('enter')
    time.sleep(1)
    pyautogui.keyUp('enter')

    #try:
        # Press Enter to confirm
        # pyautogui.write('\\n') # Adds \n to the string, but no enter is performed
        # pyautogui.press('enter') # It does nothing
        # pyautogui.hotkey("enter") # It does nothing
        # pyautogui.press("enter", presses=3) # It does nothing
        # pyautogui.keyDown('enter')
        # time.sleep(1)
        # pyautogui.keyUp('enter')

    # except :
    #     pass

    # Add a delay to allow time for the action to complete
    #time.sleep(1)

    print("Mouse moved, clicked, and string inserted successfully!")
