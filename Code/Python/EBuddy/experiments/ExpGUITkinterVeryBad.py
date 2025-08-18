# This problem makes tkinter unusable with getattr calls
# Bad problem is in  photo = ImageTk.PhotoImage(image) and label.config(image=photo)

import tkinter as tk
from PIL import Image, ImageTk
import os

############################################################################################
def VerifyCameraSetupBasicDo(message_queue=""):
    image_path = r'D:\Banfi\Github\Fluently\Releases\2024_07_09_EBuddy\Data\Input\App\ArtecStudio\Hints\CameraSetupBasic.PNG'
    VerifyCameraSetup(image_path)

def VerifyCameraSetupAdvancedDo(message_queue=""):
    image_path = r'D:\Banfi\Github\Fluently\Releases\2024_07_09_EBuddy\Data\Input\App\ArtecStudio\Hints\CameraSetupAdvanced.PNG'
    VerifyCameraSetup(image_path)

def VerifyCameraSetup(image_path):
    root = tk.Tk()
    root.title(os.path.basename(image_path))

    label = tk.Label(root)
    label.pack()

    image = Image.open(image_path)
    # This code works only when called from this module or similar,
    # but when called via getattr, label.config crashes or raises "_tkinter.TclError: image "pyimage31" doesn't exist", or pyimage7
    # setting photo and image as global does not help
    photo = ImageTk.PhotoImage(image)
    label.config(image=photo)
    label.image = photo
    root.mainloop()

if __name__ == '__main__':
    VerifyCameraSetupBasicDo()
    pass
