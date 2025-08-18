# worker_module.py
import time
import queue

def ArtecStudioIsOnDo(message_queue):
    # Simulate some work
    time.sleep(2)
    try:
        # Send a message to the main thread
        message_queue.put("Hello from ArtecStudioIsOnDo call")
        return True
    except Exception as error:
        return False

