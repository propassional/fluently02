# n objects share n mutex, which is defined outside of them

import threading
import time

class ObjWriter:
    def __init__(self, mutex):
        self.mutex = mutex

    def write(self, filename):
        with self.mutex:
            print(f"Writing to {filename}")
            # Simulate writing to the file
            time.sleep(2)
            print(f"Finished writing to {filename}")

class ObjReader:
    def __init__(self, mutex):
        self.mutex = mutex

    def read(self, filename):
        with self.mutex:
            print(f"Reading from {filename}")
            # Simulate reading from the file
            time.sleep(2)
            print(f"Finished reading from {filename}")

def create_threads(num_pairs):
    mutexes = [threading.Lock() for _ in range(num_pairs)]
    writers = [ObjWriter(mutexes[i]) for i in range(num_pairs)]
    readers = [ObjReader(mutexes[i]) for i in range(num_pairs)]

    writer_threads = [threading.Thread(target=writers[i].write, args=(f"file{i+1}.png",)) for i in range(num_pairs)]
    reader_threads = [threading.Thread(target=readers[i].read, args=(f"file{i+1}.png",)) for i in range(num_pairs)]

    return writer_threads, reader_threads

def main(num_pairs):
    writer_threads, reader_threads = create_threads(num_pairs)

    # Start all writer and reader threads
    for wt in writer_threads:
        wt.start()
    for rt in reader_threads:
        rt.start()

    # Wait for all threads to complete
    for wt in writer_threads:
        wt.join()
    for rt in reader_threads:
        rt.join()

    print("All operations are complete.")

# Example usage with any number of writer-reader pairs
num_pairs = 3  # You can change this to any number
main(num_pairs)
