# Not sure if this really works!!

import importlib

# Dynamically import module_b here
module_b = importlib.import_module("ExpImportCircularB")

def some_function():
    module_b.some_other_function()

if __name__ == "__main__":
    some_function()
