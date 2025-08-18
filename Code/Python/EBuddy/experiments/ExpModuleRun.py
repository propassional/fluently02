import importlib.util
import os
import sys

def load_module(module_path):
    # Check if the module is already loaded
    module_name = os.path.basename(module_path).split(".")[0]
    if module_name in sys.modules:
        print(f"Module '{module_name}' is already loaded.")
    else:
        # Load the module
        spec = importlib.util.spec_from_file_location(module_name, module_path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        sys.modules[module_name] = module  # Add the module to sys.modules
        print(f"Module '{module_name}' has been loaded now.")

def run_method_from_module(file_path, method_name):
    module_is_loadable = True
    try:
        # Create a module_declared_at_top_of_the_file spec from the file location
        spec = importlib.util.spec_from_file_location("custom_module", file_path)
        # Create a module_declared_at_top_of_the_file from the spec
        module = importlib.util.module_from_spec(spec)
        # Load the module_declared_at_top_of_the_file (not yet run)
        spec.loader.exec_module(module)
    except Exception as error:
        print(f"Exception {error}, calling {file_path}, spec {spec}")
        module_is_loadable = False

    # Execute the specified method (if it exists and the module_declared_at_top_of_the_file is loadable)
    if module_is_loadable and hasattr(module, method_name):
        try:
            # Execute the specified method
            result = getattr(module, method_name)()
            return result
        except Exception as error:
            print(f"Exception {error} while executing {method_name} in {file_path}")
            return None
    else:
        print(f"Method {method_name} not found in {file_path}")
        return None

if __name__ == '__main__':
    module_path = r"D:\\CreateMesh__2024_08_05_15_56_47.py"  # Replace with the actual path
    print("Modules currently imported:", list(sys.modules.keys()))
    load_module(module_path)
    print("Modules currently imported:", list(sys.modules.keys()))
    load_module(module_path)
    print("Modules currently imported:", list(sys.modules.keys()))

    # Example usage
    file_path = r"D:\Banfi\Github\Fluently\Releases\2024_07_09_EBuddy\Data\Input\SM\ArtecStudio__2024_09_07__10_22_29\JSON\ArtecStudio__2024_09_07__10_22_29.py"  # Replace with your actual file path
    method_name = "StartArtecStudioDo"  # Replace with your actual method name
    result = run_method_from_module(file_path, method_name)
    if result is not None:
        print(f"Method {method_name} executed successfully with result: {result}")
    else:
        print(f"Method {method_name} could not be executed.")
