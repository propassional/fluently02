# Find out which line within the try block did rise the error
# try:
# except Exception as error:

import traceback
import io

try:
    # Your code with multiple lines
    line1 = 1 / 0  # This will raise a ZeroDivisionError
    line2 = "hello" + 5  # This will raise a TypeError
except Exception as e:
    # Create a StringIO object to capture the traceback
    with io.StringIO() as buf:
        traceback.print_exc(file=buf)
        error_message = buf.getvalue()

    # Save the error message to a file
    with open('error_log.txt', 'w') as f:
        f.write(error_message)

    print("Error details have been written to error_log.txt")
