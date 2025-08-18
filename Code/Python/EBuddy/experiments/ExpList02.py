# Show the 6 different ways to add elements to a list

# Initial list
my_list = [1, 2, 3]

# 1. Using append()
my_list.append(4)
print("After append:", my_list)  # Output: [1, 2, 3, 4]

# 2. Using insert()
my_list.insert(1, 'a')
print("After insert:", my_list)  # Output: [1, 'a', 2, 3, 4]

# 3. Using extend()
my_list.extend([5, 6])
print("After extend:", my_list)  # Output: [1, 'a', 2, 3, 4, 5, 6]

# 4. Using the + operator
new_list = my_list + [7, 8]
print("After + operator:", new_list)  # Output: [1, 'a', 2, 3, 4, 5, 6, 7, 8]

# 5. Using list comprehension
my_list = [x for x in my_list] + [9, 10]
print("After list comprehension:", my_list)  # Output: [1, 'a', 2, 3, 4, 5, 6, 9, 10]

# 6. Using * operator (unpacking)
new_elements = [11, 12]
my_list = [*my_list, *new_elements]
print("After * operator:", my_list)  # Output: [1, 'a', 2, 3, 4, 5, 6, 9, 10, 11, 12]