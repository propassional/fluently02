# Two objects share the same mutex, which is defined outside of them

import threading
import time

# Create a mutex
mutex = threading.Lock()

class ObjWriter:
    def write(self, filename):
        with mutex:
            print(f"Writing to {filename}")
            # Simulate writing to the file
            time.sleep(2)
            print(f"Finished writing to {filename}")

class ObjReader:
    def read(self, filename):
        with mutex:
            print(f"Reading from {filename}")
            # Simulate reading from the file
            time.sleep(2)
            print(f"Finished reading from {filename}")

# Create instances of ObjWriter and ObjReader
writer = ObjWriter()
reader = ObjReader()

# Create threads for writing and reading
writer_thread = threading.Thread(target=writer.write, args=("graph.png",))
reader_thread = threading.Thread(target=reader.read, args=("graph.png",))

# Start the threads
writer_thread.start()
reader_thread.start()

# Wait for both threads to complete
writer_thread.join()
reader_thread.join()

print("Both operations are complete.")
