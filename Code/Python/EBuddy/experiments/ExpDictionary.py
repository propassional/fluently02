import random
import calendar

example_dict = {} # Init an empty dictionary, while example_dict = [] initiates an empty list

def add_to_dict(key, value):
    if key not in example_dict:
        example_dict[key] = []  # Create a new list if the key doesn't exist
    example_dict[key].append(value)  # Append the value to the list

# Insert key-value pairs
def assign_vs_append():
    # Interesting situation:
    # This is wrong, since dict is empty, but it works if example_dict = {"SM1": [], "SM2": [] }
    # example_dict["SM1"].append("riciao")
    # Correct solution
    add_to_dict("SM1", "riciao")

    example_dict["SM1"] = ["ciao"] # Since the key is new, it creates a new pair of values
    example_dict["SM1"] = ["riciao"] # Since the key is not new, it overwrites the previous one!
    example_dict["SM1"].append("riciao") # Creates a new couple of values
    example_dict["SM2"] = ["SM2"] # Since the key is new, it creates a new pair of values

    print(example_dict)

def dictionary_from_dictionary():
    # Create a dictionary from month names
    month_names_dict = {i: calendar.month_name[i] for i in range(1, len(calendar.month_name))}
    print(month_names_dict)

    # Obtain a list of month names
    month_list = [month_names_dict[i] for i in range(1, len(month_names_dict))]
    print(month_list)

    # Obtain a list of month numbers and names
    month_list_tuple = [(i, month_names_dict[i]) for i in range(1, len(month_names_dict))]
    print(month_list_tuple)

def dictionary_to_list():

    month_dict = {
        1: "January", 2: "February", 3: "March", 4: "April",
        5: "May", 6: "June", 7: "July", 8: "August",
        9: "September", 10: "October", 11: "November", 12: "December"
    }
    month_names_list = list(month_dict.values())
    print(month_names_list)

def guess_the_band():
    # Create a dictionary with famous bands
    famous_bands = {
        'The Beatles': 'English rock band',
        'Led Zeppelin': 'English rock band',
        'Queen': 'British rock band',
        'Pink Floyd': 'English rock band',
        'The Rolling Stones': 'English rock band',
        # Add more bands if needed
    }

    # Ask the user for input
    user_band = input("Guess a famous band name: ")

    # Verify if the user's input is in the dictionary
    if user_band in famous_bands:
        print(f"{user_band} is a famous band!")
    else:
        print(f"{user_band} is not in our dictionary of famous bands.")

def guess_the_month():
    # Create a dictionary with values and keys
    # month names as "values" and their corresponding numbers as their "keys"
    month_dict = {
        1: "January", 2: "February", 3: "March", 4: "April",
        5: "May", 6: "June", 7: "July", 8: "August",
        9: "September", 10: "October", 11: "November"#, 12: "December"
    }

    # Add an element
    month_dict[12] = "December" # Works also: example_dict["third_key"] = "This is the third value"

    dict_length = len(month_dict)

    # Remember that next(iter(month_dict)) always returns 1, even when called multiple times, and not 2 as expected,
    # that's why you have to dreate an iterator over the dictionary keys
    key_iterator = iter(month_dict)

    first_element = month_dict[1] # January, index starts from 1, not 0!
    first_key = next(key_iterator)
    print(first_key)  # Output: 1

    second_element = month_dict[2]  # February
    second_key = next(key_iterator)
    print(second_key)  # Output: 2

    third_element = month_dict[3]  # March
    third_key = next(key_iterator)
    print(third_key)  # Output: 3

    # first_element = month_dict[1] does not work if the key is not 1, 2, 3 but a, b, c, but this works
    month_dict_values = list(month_dict.values())
    first_element = month_dict_values[0] # Read the first value without knowing its key

    # Select a random month
    selected_month_number = random.randint(1, 12)
    selected_month_name = month_dict[selected_month_number]

    # Ask the user to input a month
    user_input = input("Guess a month (e.g., January, February, etc.): ")

    # Verify if the user's input matches the selected month
    if user_input.capitalize() == selected_month_name:
        print("Congratulations! You guessed correctly.")
    else:
        print(f"Oops! The secret month was {selected_month_name}.")

if __name__ == '__main__':
    assign_vs_append()
    guess_the_month()
    dictionary_to_list()
    guess_the_band()
