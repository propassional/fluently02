# This file has the same contents as
# D:\Banfi\Github\Fluently\Releases\2024_07_09_EBuddy\Data\Input\SM\ArtecStudio__Preview__2024_08_05__14_57_32\JSON\ArtecStudio__Preview__2024_08_05__14_57_32.py
# but here the methods are of the Check type, not Do type

def CameraSetupBasicCheck(message_queue=""):
    image_path = r'D:\Banfi\Github\Fluently\Releases\2024_07_09_EBuddy\Data\Input\App\ArtecStudio\Hints\CameraSetupBasic.png'
    message = f"Opening image CameraSetupBasic.png"
    message_queue.put(message)
    return image_path

def CameraSetupAdvancedCheck(message_queue=""):
    image_path = r'D:\Banfi\Github\Fluently\Releases\2024_07_09_EBuddy\Data\Input\App\ArtecStudio\Hints\CameraSetupAdvanced1.png'
    message = f"Opening image CameraSetupAdvanced1.png"
    message_queue.put(message)
    return image_path

def UserPreset(message_queue=""):
    image_path = r'D:\Banfi\Github\Fluently\Releases\2024_07_09_EBuddy\Data\Input\App\ArtecStudio\Hints\UserPreset.png'
    message = f"Opening image UserPreset.png"
    message_queue.put(message)
    return image_path