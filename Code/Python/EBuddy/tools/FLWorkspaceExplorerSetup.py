# This script opens several windows file explorer apps, showing important contents for EBuddy developers
# Before running this script, close all file explorers, open one and maximize it, otherwise this app opens not maximized windows

import psutil
import subprocess

paths = [
    r"D:\Banfi\Github\Fluently\Diagrams\ComponentDiagram",  # Component diagram
    r"D:\Banfi\Github\Fluently\Releases\2025_01_10_EBuddy\Data\Input\YAML",  # The EBuddy configuration file
    r"D:\Banfi\Github\Fluently\Releases\2025_01_10_EBuddy\Data\Input\SM",  # Data used by the SM
    r"D:\Banfi\Github\Fluently\Releases\2025_01_10_EBuddy\Data\Input\App\ArtecStudio",  # Images used for do/check/rollback/hints
    r"D:\Banfi\Github\Fluently\Releases\2025_01_10_EBuddy\Data\Input\Command\CommandForEBuddyHistory",  # Commands
    r"D:\Banfi\Github\Fluently\Releases\2025_01_10_EBuddy\Data\Output\LogFile",  # Dir containing all EBuddy logfiles, organised for SM name
    r"D:\Banfi\Github\Fluently\Releases\2025_01_10_EBuddy\Data\Output\CV2Image",  # Image processing results
    r"D:\Banfi\Github\Fluently\Releases\2025_01_10_EBuddy\Data\Output\Screenshot",  # State machine screenshots
    # r"D:\Banfi\Lavoro\ArtecStudio\ArtecStudio16Projects"  # Artec Studio projects containing mesh, .ply files
]

def is_directory_open(path):
    for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
        if proc.info['name'] == 'explorer.exe' and path in proc.info['cmdline']:
            return True
    return False

for path in paths:
    if not is_directory_open(path):
        subprocess.Popen(f'explorer /e,/root,{path}')