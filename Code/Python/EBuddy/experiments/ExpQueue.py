import threading
import queue
import time

# Create a queue to communicate with the main thread
message_queue = queue.Queue()

def worker():
    # Simulate some work
    time.sleep(2)
    # Send a message to the main thread
    message_queue.put("Hello from the worker thread!")

# Create and start the worker thread
thread = threading.Thread(target=worker)
thread.start()

# Main thread: wait for the message
print("Main thread: Waiting for message...")
message = message_queue.get()
print(f"Main thread received: {message}")

# Wait for the worker thread to finish
thread.join()
