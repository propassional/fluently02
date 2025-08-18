# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.
from datetime import datetime
# Image processing library
import halcon as ha
# PyAutoGUI lets Python control the mouse and keyboard, and other GUI automation tasks
import pyautogui
import PySimpleGUI as sg

def print_hi(name):
    # Use a breakpoint in the code line below to debug your script.
    print(f'Hi, {name}')  # Press Ctrl+F8 to toggle the breakpoint.

import cv2
import numpy as np

def halcon_routine():
    img = ha.read_image('pcb')
    region = ha.threshold(img, 0, 122)
    num_regions = ha.count_obj(ha.connection(region))
    print(f'Number of Regions: {num_regions}')

def get_current_time():
    now = datetime.now()
    return now.strftime("%Y_%m_%d__%H_%M_%S")

path_screenshots = 'D:/Banfi/Github/Fluently/Code/Halcon/PyCharm01/screenshots/'
path_segmentations = 'D:/Banfi/Github/Fluently/Code/Halcon/PyCharm01/segmentations/'

# Press the green button in the gutter to run the script.
if __name__ == '__main__':

    screenshot = pyautogui.screenshot()
    screenshot_name = path_screenshots + get_current_time() + ".png"
    screenshot.save(screenshot_name)

    # Load the main image
    #img = cv2.imread('path/to/image.jpg')
    img = cv2.imread(screenshot_name)

    # Load the template image
    #template = cv2.imread('path/to/template.jpg')
    template = cv2.imread(r'D:\Banfi\Github\Fluently\Code\Halcon\PyCharm01\images\ArtecStudioLogo.png')

    # Get the width and height of the template image
    w, h = template.shape[:-1]

    # Perform template matching
    res = cv2.matchTemplate(img, template, cv2.TM_CCOEFF_NORMED)

    # Define a threshold
    threshold = 0.8

    # Get the location of the template in the image
    loc = np.where(res >= threshold)

    # Draw a rectangle around the template
    for pt in zip(*loc[::-1]):
        cv2.rectangle(img, pt, (pt[0] + w, pt[1] + h), (0, 0, 255), 2)

    # Display the result
    cv2.imshow('Result', img)
    cv2.imwrite(path_segmentations + "segmentation01.png", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
