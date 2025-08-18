import threading
import queue
from ExpQueueWorker import worker

# Create a queue to communicate with the main thread
message_queue = queue.Queue()

# Create and start the worker thread
thread = threading.Thread(target=worker, args=(message_queue,))
thread.start()

# Main thread: wait for the message
print("Main thread: Waiting for message...")
message = message_queue.get()
print(f"Main thread received: {message}")

# Wait for the worker thread to finish
thread.join()