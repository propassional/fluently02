import sys
import os

if __name__ == '__main__':
    print(sys.path)
    print("PYTHONPATH = " + os.environ.get('PYTHONPATH'))
