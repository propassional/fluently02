import os

def count_lines(directory, lines=0):
    directories_to_skip = [ ".idea", "_pycache_", "_old", "venv"]
    for item in os.listdir(directory):
        if item in directories_to_skip:
            pass
        else:
            try:
                path = os.path.join(directory, item)
                if os.path.isdir(path):
                    lines = count_lines(path, lines)
                elif os.path.isfile(path) and path.endswith(".py"):
                    with open(path, "r") as file:
                        lines += len(file.readlines())
            except Exception as error:
                pass
    return lines

if __name__ == '__main__':
    # Usage:
    project_directory = r"D:\Banfi\Github\Fluently\Code\Python\EBuddy"
    total_lines = count_lines(project_directory)
    print(f"Total lines of code: {total_lines}") # 5500 lines @ 24.07.24