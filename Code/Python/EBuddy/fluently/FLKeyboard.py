# Problem
# My current keyboard setting (swiss german keyboard) often somehow switches to the US keyboard, that I don't even use, but I can not remove
# Select the US International keyboard layout and click Remove: there is no english (united states) language!

# Solution:
# Open regedit, do export (backup), save file to D:\Banfi\Github\Fluently\Releases
# Goto HKEY_LOCAL_MACHINE\SYSTEM\CurrentControlSet\Control\Keyboard Layouts goto 00020409, remove, restart computer

# This modules_path_loaded_global does not work: none of the methods is able to programmatically switch keyboard from Swiss German to US

# I use this modules_path_loaded_global for collecting all information related to handling keyboards

# How to add a Keyboard:
#   Under the “language” section, click on the options of the installed current language "English(Switzerland)",
#  “Add a keyboard” and choose “Swiss German QWERTZ”, switch to it with windows_button + space or alt-shift

# How to set a keyboard as the reference one
# Override for default input method: select "English (Switzerland) - Swiss German"

import keyboard
import win32api
import win32con
import ctypes
import sys

# Constants for keyboard layouts
US_KEYBOARD = '00000409'
US_INTERNATIONAL_KEYBOARD = '00020409'

# It partially works
def install_keyboard(layout):
    # Installs the keyboard into the OS, but it does not switch to it
    hkl = ctypes.windll.user32.LoadKeyboardLayoutW(layout, 1)
    # Switch to a different layout: it does not work
    ctypes.windll.user32.ActivateKeyboardLayout(hkl, 0)

def remove_keyboard(layout):
    try:
        # Removes the specified keyboard layout
        hkl = ctypes.windll.user32.LoadKeyboardLayoutW(layout, 1)
        # Removes the specified keyboard layout
        # result 1 is good, but does not work on US_INTERNATIONAL_KEYBOARD = '00020409'
        # HKEY_LOCAL_MACHINE\SYSTEM\CurrentControlSet\Control\Keyboard Layouts goto 00020409, remove, restart computer (Do reg backup before)
        result = ctypes.windll.user32.UnloadKeyboardLayout(hkl)
        if result == 0:
            raise ctypes.WinError()
    except Exception as error:
        print(f"Error removing keyboard layout {layout}: {error}")

def switch_keyboard(layout):
    # Load the keyboard layout
    win32api.LoadKeyboardLayout(layout, win32con.KLF_ACTIVATE)
    # Activate the keyboard layout
    win32api.ActivateKeyboardLayout(int(layout, 16), 0)

def install_keyboard(layout):
    # Installs the keyboard into the OS, but it does not switch to it
    win32api.LoadKeyboardLayout(layout, 1)

def remove_keyboard(layout):
    # Removes the specified keyboard layout
    hkl = win32api.LoadKeyboardLayout(layout, 1)
    win32api.UnloadKeyboardLayout(hkl)

# Function to check and set the keyboard layout
def set_keyboard_layout():
    # Get the current layout
    current_layout = keyboard.get_hotkey_name()
    print(f"Current layout: {current_layout}")

    # Assuming 'first layout' refers to a specific layout, e.g., 'us'
    desired_layout = 'us'

    if current_layout != desired_layout:
        # Set the keyboard layout to the desired one
        keyboard.write(f"setxkbmap {desired_layout}")
        print(f"Keyboard layout set to: {desired_layout}")
    else:
        print("Keyboard is already on the desired layout.")

if __name__ == '__main__':
    # Switch to US keyboard
    install_keyboard(US_KEYBOARD)

    # Switch to US-International keyboard
    install_keyboard(US_INTERNATIONAL_KEYBOARD)

    # Wait for the user to press a key combination
    print("Press a key combination...")
    keyboard.wait('esc')  # Wait until 'esc' is pressed to proceed

    # Get the name of the currently pressed keys
    hotkey_name = keyboard.get_hotkey_name()
    print(f"The hotkey pressed is: {hotkey_name}")

    set_keyboard_layout()


    # This code works
    swiss_german_layout = '00000807' # It is not the correct one!
    install_keyboard(swiss_german_layout) # This code works, installs a keyboard called "DEU German (Switzerland) Swiss German Keyboard"
    remove_keyboard(swiss_german_layout) # This code works

    # This code does not work
    US_INTERNATIONAL_KEYBOARD = '00020409'
    install_keyboard(US_INTERNATIONAL_KEYBOARD) # Probably works
    remove_keyboard(US_INTERNATIONAL_KEYBOARD) # It does not work

    sys.exit()

    # This code works
    swiss_german_layout = '00000807' # It is not the correct one!
    install_keyboard(swiss_german_layout) # This code works, installs a keyboard called "DEU German (Switzerland) Swiss German Keyboard"
    remove_keyboard(swiss_german_layout) # This code works

    # Example usage
    US_KEYBOARD = '00000409'
    switch_keyboard(US_KEYBOARD)
    remove_keyboard(US_KEYBOARD) # It works


