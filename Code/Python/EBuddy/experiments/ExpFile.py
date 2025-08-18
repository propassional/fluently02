# Open a file in write mode
with open("D:\example.txt", "w") as file:
    # Write the delete character using its Unicode escape sequence
    file.write("\u007F")