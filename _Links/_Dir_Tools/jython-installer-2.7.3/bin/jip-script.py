#!D:\Banfi\_tools\jython-installer-2.7.3\bin\jython.exe
# EASY-INSTALL-ENTRY-SCRIPT: 'jip==0.9.16','console_scripts','jip'
__requires__ = 'jip==0.9.16'
import re
import sys
from pkg_resources import load_entry_point

if __name__ == '__main__':
    sys.argv[0] = re.sub(r'(-script\.pyw?|\.exe)?$', '', sys.argv[0])
    sys.exit(
        load_entry_point('jip==0.9.16', 'console_scripts', 'jip')()
    )
