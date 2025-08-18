# How to define a global variable in 3 steps

# STEP ONE is facultative: declare the variable at top level
# This declaration is needed only if the main needs to know this variable, or an initialisation is needed
# If this is not needed, the methods do not need it
# In fact, if you comment it, only the main will give an error
# global module_declared_at_top_of_the_file = None # This is wrong
module_declared_at_top_of_the_file = "Default string"
message = "Hello from moduleA" # Used by moduleB
print(f"message: {message}")
print(f"message: {message}")

def method_can_change_global_variable(data):
    # STEP TWO is a must: global defines that this method can write into the global variable
    global module_declared_at_top_of_the_file
    module_declared_at_top_of_the_file = data

def method_can_change_global_variable_twin(data):
    global module_declared_at_top_of_the_file
    module_declared_at_top_of_the_file = data

def method_can_not_change_global_variable(data):
    # This variable has the same name as the global variable but it only exists locally
    module_declared_at_top_of_the_file = data

def method_can_read_global_value():
    # For read-only, there is no need to define the variable as global
    #global module_declared_at_top_of_the_file
    return module_declared_at_top_of_the_file

def method_writes_to_a_local_variable():
    # module_declared_at_top_of_the_file is known, since it is defined on top of the file
    module_declared_at_top_of_the_file = "This is some local SM_JSON_data" # It will be assigned, even if module_declared_at_top_of_the_file is not declared as global

if __name__ == '__main__':
    # This line can work only if the variable is declared at the top of the file
    print(f"module_declared_at_top_of_the_file {module_declared_at_top_of_the_file}") # prints: module_declared_at_top_of_the_file Default string

    method_can_change_global_variable("This is the SM_JSON_data coming from main")
    print(method_can_read_global_value()) # prints: This is the SM_JSON_data coming from main

    method_can_change_global_variable_twin("This is the SM_JSON_data coming from main This is the SM_JSON_data coming from main")
    print(method_can_read_global_value()) # prints: This is the SM_JSON_data coming from main This is the SM_JSON_data coming from main

    method_can_not_change_global_variable("This is the NEW NEW SM_JSON_data coming from main")
    print(method_can_read_global_value()) # prints: This is the SM_JSON_data coming from main This is the SM_JSON_data coming from main

    method_writes_to_a_local_variable()
    print(method_can_read_global_value()) # This is the SM_JSON_data coming from main This is the SM_JSON_data coming from main