# Show how to define a list, assign, print, remove elem, check, iterators

# Define lists
odd_months = ['Jan', 'Mar', 'May', 'Jul', 'Sep', 'Nov']

# Assignment
odd_months_link = odd_months # This is NOT a new list but a POINTER to it!!
odd_months_copy = odd_months.copy()
first_month = odd_months[0] # Jan
sunny_months = []

# Print lists
print(f"odd_months {odd_months}")

# Remove elements
# Remove a specific element
odd_months.remove('May')
# Remove the first element
odd_months.pop(0)
# Remove the last element
odd_months.pop()
# Remove all elements
odd_months.clear()
print(f"odd_months {odd_months}") # prints "odd_months []"

# Check whether list is empty
if odd_months: # An empty list will return False
    print("odd_months list is not empty")
if len(odd_months) > 0: # Less elegant
    print("odd_months list is not empty")

if not sunny_months:
    print("sunny_months list is empty, obviously :-D")
if len(sunny_months) == 0: # Less elegant
    print("sunny_months list is empty, obviously :-D")

# Combine lists
even_months = ['Feb', 'Apr', 'Jun', 'Aug', 'Oct', 'Dec']
all_months = odd_months + even_months

# Iterators
# Zip for dual (see examples in my code)
#for command_image_path, app_name in zip(command_image_path_list, app_name_list):

# For iterator
all_months_combined = []
for i in range(len(odd_months)):
    all_months_combined.append(odd_months[i])
    all_months_combined.append(even_months[i])

print(f"all_months_combined {all_months_combined}")

# Sort lists
all_months = odd_months + even_months
all_months = sorted(all_months)
print(f"all_months_sorted_alphabetically {all_months}")
