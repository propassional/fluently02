import time

SLEEP_DO = True
SLEEP_TIME = 5

def GetARototiltDo(message_queue=""):
    image_path = r'D:\Banfi\Github\Fluently\Releases\2024_07_09_EBuddy\Data\Input\App\ArtecStudio\Hints\GetARototilt500x1000.jpg'
    message = f"Install a rototilt like this one, which you can rotate by hand"
    message_queue.put(message)
    if SLEEP_DO:
        time.sleep(SLEEP_TIME)
    return image_path  # So the image will be displayed in GUI

def ApplyMarkersDo(message_queue=""):
    image_path = r'D:\Banfi\Github\Fluently\Releases\2024_07_09_EBuddy\Data\Input\App\ArtecStudio\Hints\ApplyMarkers500x1000.jpg'
    message = f"Apply markers onto the object, distance 5 m"
    message_queue.put(message)
    if SLEEP_DO:
        time.sleep(SLEEP_TIME)
    return image_path  # So the image will be displayed in GUI

def ScanRingBelowDo(message_queue=""):
    image_path = r'D:\Banfi\Github\Fluently\Releases\2024_07_09_EBuddy\Data\Input\App\ArtecStudio\Hints\ScanRingBelow500x1000.jpg'
    message = f"Keep scanner at fix position 12 cm, rotate the rototilt"
    message_queue.put(message)
    if SLEEP_DO:
        time.sleep(SLEEP_TIME)
    return image_path  # So the image will be displayed in GUI

def ScanRingIntermediateDo(message_queue=""):
    image_path = r'D:\Banfi\Github\Fluently\Releases\2024_07_09_EBuddy\Data\Input\App\ArtecStudio\Hints\ScanRingIntermediate500x1000.jpg'
    message = f"Keep scanner at fix position 12 cm, rotate the rototilt"
    message_queue.put(message)
    if SLEEP_DO:
        time.sleep(SLEEP_TIME)
    return image_path  # So the image will be displayed in GUI

def ScanRingAboveDo(message_queue=""):
    image_path = r'D:\Banfi\Github\Fluently\Releases\2024_07_09_EBuddy\Data\Input\App\ArtecStudio\Hints\ScanRingAbove500x1000.jpg'
    message = f"Keep scanner at fix position 12 cm, rotate the rototilt"
    message_queue.put(message)
    if SLEEP_DO:
        time.sleep(SLEEP_TIME)
    return image_path  # So the image will be displayed in GUI