import pyautogui
import re

# Input: D:\\complex_path\\ArtecStudio__Preview__2024_08_05__14_57_32.json
# Output: Preview
def extract_name_short(name):
    match = re.search(r'__(.*?)__', name) # This pattern looks for any substring wrapped between two pairs of double underscores (__):
    if match:
        #extracted_word = match.group(1)
        extracted_word = match.group(1).split('__')[0]
        return extracted_word

# input = [([[3, 6], [133, 6], [133, 25], [3, 25]], 'Recording paused', 0.8126072659611464)]
# output = 'Recording paused'
def extract_text(template):
    for item in template:
        for element in item:
            if isinstance(element, str):
                return element

def my_method(input_string):
    if "bye" in input_string:
        print("hello")

def remove_double_backslashes(input_string):
    #result = input_string.replace(r'\\', r'\\')
    #escaped_backslash = "This is a backslash: \\"
    print(input_string)
    #result = input_string.replace("\\\", "\\")
    #return result

# Checks if string_to_check is part of reference_string, case-insensitive.
def string_is_in_reference_string(string_to_check, reference_string):
    # Convert both strings to lower case for case-insensitive comparison
    main_string_lower = reference_string.lower()
    #string_to_check_lower = string_to_check.lower()
    string_to_check_lower = string_to_check.replace(" ", "").lower()

    # Check if string_to_check is in reference_string
    return string_to_check_lower in main_string_lower

# Input: "This is my ctrl&a text"
# Output: list ['This', 'is', 'my', 'ctrl&a', 'text']
def split_string(string, split_string):
    string_split_parts_list = string.split(split_string)
    return string_split_parts_list

# Differentiate input, if list do something, if a string do something else
def search_in_string(input_data):
    if isinstance(input_data, str):
        print(input_data)
    elif isinstance(input_data, list):
        for item in input_data:
            print(item)
    else:
        print("Unsupported type")

if __name__ == '__main__':
    original_string = 'D:\\Banfi\\Github\\Fluently\\Releases\\2024_07_09_EBuddy\\Data\\Input\\App\\ArtecStudio\\Code'
    cleaned_string = remove_double_backslashes(original_string)
    print(cleaned_string)

    reference_string = "Ctrl&A&Del"
    string_to_check = " ctrl&a&del "

    # reference_string = "Hello World"
    # string_to_check = "world"
    print(string_is_in_reference_string(string_to_check, reference_string))  # Output: True

    split_string("This is my ctrl&a text", " ")

    app_name = "ArtecStudio"
    window_current_name = "Artec Studio 16 Professional"

    # Replace string parts
    window_current_name = window_current_name.replace(" ", "")

    # Search within a string
    if app_name in window_current_name:
        print(f"app {app_name} is already on focus")

    my_string = "ctrl&a"

    # Search within a string
    result = my_string.__contains__("&") # I often have type problems with this method call

    # Split a string
    keys = my_string.split('&')

    pyautogui.hotkey(keys[0], keys[1])
    pass