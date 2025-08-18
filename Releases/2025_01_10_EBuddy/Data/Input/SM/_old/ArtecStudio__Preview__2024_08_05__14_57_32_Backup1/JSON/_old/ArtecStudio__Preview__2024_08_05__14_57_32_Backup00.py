import sys
import os

# This does not work: from FLOS import sys_path_to_file
import FLOS

path_app_code = "..\\..\\..\\App\\ArtecStudio\\Code"

current_module_path = os.path.abspath(__file__)
FLOS.sys_path_append_app_path(current_module_path, path_app_code)

# Execute Artec Studio code
try:
    from ArtecStudio import *
    #ArtecStudioIsOnCheck()
except Exception as error:
    print(f"Error executing some Artec Studio methods")
