# Again the same proble as tkinter when called with getattr
# UserWarning: Starting a Matplotlib GUI outside of the main thread will likely fail

import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import os

def show_image(image_path):
    img = mpimg.imread(image_path)
    imgplot = plt.imshow(img)
    plt.title(os.path.basename(image_path))
    plt.show()

def VerifyCameraSetupBasicDo(message_queue=""):
    image_path = r'D:\Banfi\Github\Fluently\Releases\2024_07_09_EBuddy\Data\Input\App\ArtecStudio\Hints\CameraSetupBasic.PNG'
    show_image(image_path)

if __name__ == '__main__':
    VerifyCameraSetupBasicDo()
