import sys

# 1. Add the path to your module to sys
sys.path.append(r'D:\Banfi\Github\Fluently\Releases\2024_07_09_EBuddy\Data\Input\SM\CreateMesh__2024_08_05_15_56_47\JSON')

# Editor warns an error, but there is none if you run it
# 2. import your module, the whole module is run, that is, all global variables and methods are read, and main is executed
import CreateMesh__2024_08_05_15_56_47

# 3. Call a method within your module
CreateMesh__2024_08_05_15_56_47.VerifyCameraSetupDo()