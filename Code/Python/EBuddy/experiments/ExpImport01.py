#from ArtecStudio import *

import sys
from FLOS import path_from_SM_code_to_app_code, sys_path_to_file

sys_path_to_file()

# Find the
app_code = path_from_SM_code_to_app_code()

if app_code:
    sys.path.append(app_code)
    sys_path_to_file()

