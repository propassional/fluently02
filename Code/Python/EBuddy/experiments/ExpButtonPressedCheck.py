import threading
import time
import keyboard

def escape_button_thread(stop_flag, result):
    while not stop_flag.is_set():
        try:
            time.sleep(0.1)
            if keyboard.is_pressed('esc'):
                print("Escape key pressed. Exiting...")
                with open('D:/_EscPressed.txt', 'w') as file:
                    file.write('Escape key pressed!')
                result[0] = True
        except Exception as e:
            print(f"Error: {e}")
            result[0] = False
    print("Thread is stopping...")

if __name__ == '__main__':
    result = [None]
    thread_stop_flag = threading.Event()
    thread_escape_handle = threading.Thread(target=escape_button_thread, args=(thread_stop_flag, result))
    thread_escape_handle.start()

    time.sleep(5) # Do some processing...

    thread_stop_flag.set()  # Signal the thread to stop
    # Wait for the thread to finish
    thread_escape_handle.join()
    return_value = result[0]
    print(f"Button escape has been pressed: {return_value}")
