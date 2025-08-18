# worker_module.py
import time
import queue

def worker(message_queue):
    # Simulate some work
    time.sleep(2)
    # Send a message to the main thread
    message_queue.put("Hello from the worker thread!")