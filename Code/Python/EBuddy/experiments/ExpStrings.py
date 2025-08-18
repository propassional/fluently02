# Define the strings
str1 = "hello"
str2 = "-world"

# Add the strings
result = str2[:1] + str1 + str2[1:]

# Append a string
str1 += str2
print(str1)

# Print the result
print(result) # -helloworld

tab_name = '-CIAO-'
tab_name = tab_name.replace("-", "")

# Search within a string
main_string = "helloworld"
# Check if the substring "world" is in the main string
if "world" in main_string:
    print("The string contains 'world'.")
else:
    print("The string does not contain 'world'.")

# Print a number into a string
# Calculate the length of the string
length_of_string = len(main_string)
# Concatenate the original string with its length
result_string = main_string + str(length_of_string)
# Print the result
print(result_string)

# Split a string
input_string = 'Instructions\\nNext'
parts = input_string.split('\\n')
sm_name = parts[0]
command = parts[1]