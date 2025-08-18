# Used for converting a text file task list into html format, probably useless now that I discovered html5 displaying feature in pycharm

def add_br_to_all_lines(input_file, output_file):
    with open(input_file, 'r', encoding='utf-8') as file:
        lines = file.readlines()

    with open(output_file, 'w', encoding='utf-8') as file:
        for line in lines:
            if line.strip():
                file.write(line.strip() + '<br>\n')
            else:
                file.write(line)  # Write the empty line as is

def add_br_to_empty_lines(input_file, output_file):
    with open(input_file, 'r', encoding='utf-8') as file:
        lines = file.readlines()

    with open(output_file, 'w', encoding='utf-8') as file:
        for line in lines:
            if line.strip():  # Check if the line is not empty
                file.write(line.strip() + '\n')
            else:
                file.write('<br>\n')  # Add <br> to empty lines

def remove_br(input_file, output_file):
    """
    Remove all <br> tags from the content of the input file line by line and write the modified content to the output file.

    Parameters:
    input_file (str): The path to the input file containing <br> tags.
    output_file (str): The path to the output file where the modified content will be saved.
    """
    with open(input_file, 'r', encoding='utf-8') as infile, open(output_file, 'w', encoding='utf-8') as outfile:
        for line in infile:
            # Remove all <br> tags from the current line
            line_without_br = line.replace('<br>', '')
            # Write the modified line to the output file
            outfile.write(line_without_br)

if __name__ == '__main__':
    input_file = 'D:\\AddBreakInput.txt'
    output_file = 'D:\\AddBreakOutput.txt'
    #add_br_to_all_lines(input_file, output_file)
    #add_br_to_empty_lines(input_file, output_file)
    remove_br(input_file, output_file)