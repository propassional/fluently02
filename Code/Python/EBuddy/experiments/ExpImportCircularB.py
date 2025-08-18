import importlib

# Dynamically import module_a here
module_a = importlib.import_module("ExpImportCircularA")

def some_other_function():
    print("Hello from ExpImportCircularB")

if __name__ == "__main__":
    some_other_function()
