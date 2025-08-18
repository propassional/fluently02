# Search template within an image (screenshot)
# Alternative1: Open CV
# Alternative2: Halcon
# Alternative3: pyautogui.locateOnScreen('submit.png')

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.
from datetime import datetime
#import halcon as ha
import cv2 # Image processing library
import pyautogui
import numpy as np
import pygetwindow
import time
from screeninfo import get_monitors

from FLOCREasy import read_OCR
from FLOS import datetime_now_get_string

from FLBaseClass import FLBaseClass
from FLConstants import CV2_TEST_IMAGES_SAVE, CV2_IMAGES, APPLICATION_TASKBAR, APPLICATION_ARTECSTUDIO, \
    ERROR, SCREEN_NR2_WIDTH_FACTOR, SCREEN_NR2_HEIGHT_FACTOR, CV2_MATCHING_SCORE_SUFFICIENT, CV2_MATCHING_SCORE_GOOD, \
    CV2_OCR_AREA, APPLICATION_AODK
from FLLogger import FLLogger
from FLString import extract_text
from FLPIL import extract_image

# def halcon_routine():
#     img = ha.read_image('pcb')
#     region = ha.threshold(img, 0, 122)
#     num_regions = ha.count_obj(ha.connection(region))
#     self.print(file'Number of Regions: {num_regions}')

class FLCV2(FLBaseClass):
    def __init__(self, logger):
        super().__init__(logger)

    # def datetime_now_get_string(self):
    #     now = datetime.now()
    #     return now.strftime("%Y_%m_%d__%H_%M_%S")

    def screenshot_search(self, template_image_path, application):
        if application == APPLICATION_ARTECSTUDIO:
            # Get the window title or program ID (modify this based on your app)
            window_title = "Artec Studio 16 Professional"
            # Find the window by title
            my_window = pygetwindow.getWindowsWithTitle(window_title)[0]
            # Activate the window (bring it to the foreground)
            my_window.activate()
            time.sleep(1)
        if application == APPLICATION_AODK:
            # Get the window title or program ID (modify this based on your app)
            window_title = "ARP - Automated repairing process"
            # Find the window by title
            my_window = pygetwindow.getWindowsWithTitle(window_title)[0]
            # Activate the window (bring it to the foreground)
            my_window.activate()
            time.sleep(1)
        elif application == APPLICATION_TASKBAR:
            # Do not change the focus, since it is supposed (for now) that the taskbar is always visible
            pass

        # template_numpy = cv2.cvtColor(np.array(template), cv2.COLOR_RGB2BGR)
        # Load the image to be searched
        # img = cv2.imread('path/to/image.jpg')

        screenshot = pyautogui.screenshot()
        # Convert the screenshot to a numpy array (BGR format)
        screenshot_numpy = cv2.cvtColor(np.array(screenshot), cv2.COLOR_RGB2BGR)
        # Convert image to gray
        screenshot_numpy_gray = cv2.cvtColor(screenshot_numpy, cv2.COLOR_BGR2GRAY)

        if (CV2_TEST_IMAGES_SAVE):
            try:
                image_screenshot_path = CV2_IMAGES + "\\" + datetime_now_get_string() + "_1_Screenshot.png"
                success = cv2.imwrite(image_screenshot_path, screenshot_numpy_gray)
                if success:
                    self.print(f"Screenshot image saved at {image_screenshot_path}")
                else:
                    self.print(f"Failed to save the screenshot image at {image_screenshot_path}")
            except Exception as e:
                self.print(f"An error occurred: {e}", ERROR)

        # Load the template image, which has to be searched
        # template = cv2.imread(r'D:\Banfi\Github\Fluently\StateMachines\_CommandImages\CmdStartArtecStudio.png')
        # template =  cv2.imread(template_image_path)
        self.print(f"Read template image {template_image_path}")
        template = cv2.imread(template_image_path)

        # Convert the screenshot to a numpy array (BGR format)
        template_numpy = cv2.cvtColor(np.array(template), cv2.COLOR_RGB2BGR)
        # Convert template image to gray
        template_gray = cv2.cvtColor(template_numpy, cv2.COLOR_BGR2GRAY)

        if (CV2_TEST_IMAGES_SAVE):
            try:
                image_template_path = CV2_IMAGES + "\\" + datetime_now_get_string() + "_2_Template.png"
                success = cv2.imwrite(image_template_path, template_gray)
                if success:
                    self.print(f"Template image saved at {image_template_path}")
                else:
                    self.print(f"Failed to save template image at {image_template_path}")
            except Exception as e:
                self.print(f"An error occurred: {e}")

        # Perform template matching
        # https://pyimagesearch.com/2021/03/22/opencv-template-matching-cv2-matchtemplate/
        # Template matching quickly fails if there are factors of variation in your input images, including changes to rotation, scale, viewing angle, etc.
        # If your input images contain these types of variations, you should not use template matching,
        # and utilize dedicated object detectors including HOG + Linear SVM, Faster R-CNN, SSDs, YOLO, etc.

        # Changing factors (keeping the app, app format, its menu, scale, resolution constant):
        # 1) Png is a lossless image format but it has compression (may change the results)
        # 2) Rendering engines can introduce subtle variations due to anti-aliasing, font rendering, or other graphical effects
        # 3) Screenshots may contain noise, compression artifacts, or unintended elements
        heat_map = cv2.matchTemplate(screenshot_numpy_gray, template_gray, cv2.TM_CCOEFF_NORMED)
        # _, _, _, max_loc = cv2.minMaxLoc(heat_map)
        _, max_val, _, max_loc = cv2.minMaxLoc(heat_map)

        # If the method is TM_SQDIFF or TM_SQDIFF_NORMED, take the minimum value
        # Otherwise, take the maximum value
        if cv2.TM_CCOEFF_NORMED in [cv2.TM_SQDIFF, cv2.TM_SQDIFF_NORMED]:
            matching_score = max_val
        else:
            matching_score = max_val

        # Get the top-left corner coordinates of the matched region
        x, y = max_loc

        self.print(f"Matching score: {matching_score:.4f}")
        self.print(f"Found template image at position (x, y): ({x}, {y})")

        # Create an inverted template
        inverted_template = cv2.bitwise_not(template)

        # Overlay the inverted template on the original image
        screenshot_numpy[y:y + template.shape[0], x:x + template.shape[1]] = inverted_template

        # Save the resulting image
        if (CV2_TEST_IMAGES_SAVE):
            image_path = CV2_IMAGES + "\\" + datetime_now_get_string() + "_3_Result.png"
            cv2.imwrite(image_path, screenshot_numpy)
            self.print(f"Image saved at {image_path}")

        # Calculate the center coordinates of the template
        center_x = x + template.shape[1] // 2  # Horizontal
        center_y = y + template.shape[0] // 2  # Vertical

        # If result is not convincing, apply OCR (not suitable for big images)
        if matching_score > CV2_MATCHING_SCORE_SUFFICIENT and matching_score < CV2_MATCHING_SCORE_GOOD:
            height, width, _ = template.shape
            area = height * width
            if area < CV2_OCR_AREA:
                # Apply an additional check, OCR based
                text_template_tuple = read_OCR(template_image_path)
                text_template_OCR = extract_text(text_template_tuple)
                self.print(f"Text extracted with OCR from template image: {text_template_OCR}")

                # Apply the template image over the area of the screenshot, extract it, read it
                image_screenshot_reduced = extract_image(image_screenshot_path, image_template_path, x, y)
                #image_screenshot_reduced.show()
                image_screenshot_reduced_bn = cv2.cvtColor(np.array(image_screenshot_reduced), cv2.COLOR_RGB2BGR)
                image_screenshot_reduced_bn_nm = cv2.cvtColor(image_screenshot_reduced_bn, cv2.COLOR_BGR2GRAY)
                if (CV2_TEST_IMAGES_SAVE):
                    try:
                        image_screenshot_reduced_path = CV2_IMAGES + "\\" + datetime_now_get_string() + "_1_ScreenshotReduced.png"
                        success = cv2.imwrite(image_screenshot_reduced_path, image_screenshot_reduced_bn_nm)
                        if success:
                            self.print(f"Screenshot image saved at {image_screenshot_reduced_path}")
                        else:
                            self.print(f"Failed to save the screenshot image at {image_screenshot_reduced_path}")
                    except Exception as e:
                        self.print(f"An error occurred: {e}", ERROR)
                text_screenshot_reduced_tuple = read_OCR(image_screenshot_reduced_path)
                text_screenshot_reduced_OCR = extract_text(text_screenshot_reduced_tuple)
                self.print(f"text_screenshot_reduced_OCR {text_screenshot_reduced_OCR}")
                if text_template_OCR != text_screenshot_reduced_OCR:
                    matching_score = 0
                    self.print(f"OCR are different, matching score after OCR correction: {matching_score:.4f}")
                else:
                    matching_score = 1
                    self.print(f"OCR are same, matching score after OCR correction: {matching_score:.4f}")

        return center_x, center_y, matching_score  # matching_score = 1 best, = 0 worst

    def screenshot_search_color(self, template_image_path, application):
        if application == APPLICATION_ARTECSTUDIO:
            # Get the window title or program ID (modify this based on your app)
            window_title = "Artec Studio 16 Professional"
            # Find the window by title
            my_window = pygetwindow.getWindowsWithTitle(window_title)[0]
            # Activate the window (bring it to the foreground)
            my_window.activate()
            time.sleep(1)
        elif application == APPLICATION_TASKBAR:
            # Do not change the focus, since it is supposed (for now) that the taskbar is always visible
            pass

        #template_numpy = cv2.cvtColor(np.array(template), cv2.COLOR_RGB2BGR)
        # Load the image to be searched
        #img = cv2.imread('path/to/image.jpg')

        screenshot = pyautogui.screenshot()
        # Convert the screenshot to a numpy array (BGR format)
        screenshot_numpy = cv2.cvtColor(np.array(screenshot), cv2.COLOR_RGB2BGR)

        if (CV2_TEST_IMAGES_SAVE):
            try:
                image_name = CV2_IMAGES + "\\" + datetime_now_get_string() + "_1_Screenshot.png"
                success = cv2.imwrite(image_name, screenshot_numpy)
                if success:
                    self.print(f"screenshot_search: image saved at {image_name}")
                else:
                    self.print(f"screenshot_search: failed to save the image at {image_name}")
            except Exception as e:
                self.print(f"screenshot_search: an error occurred: {e}", ERROR)

        # Load the template image, which has to be searched
        #template = cv2.imread(r'D:\Banfi\Github\Fluently\StateMachines\_CommandImages\CmdStartArtecStudio.png')
        #template =  cv2.imread(template_image_path)
        template = cv2.imread(template_image_path)

        if (CV2_TEST_IMAGES_SAVE):
            try:
                image_name = CV2_IMAGES + "\\" + datetime_now_get_string() + "_2_Template.png"
                success = cv2.imwrite(image_name, template)
                if success:
                    self.print(f"screenshot_search: image saved at {image_name}")
                else:
                    self.print(f"screenshot_search: failed to save the image at {image_name}")
            except Exception as e:
                self.print(f"screenshot_search: an error occurred: {e}")

        # Perform template matching
        # https://pyimagesearch.com/2021/03/22/opencv-template-matching-cv2-matchtemplate/
        # Template matching quickly fails if there are factors of variation in your input images, including changes to rotation, scale, viewing angle, etc.
        # If your input images contain these types of variations, you should not use template matching —
        # utilize dedicated object detectors including HOG + Linear SVM, Faster R-CNN, SSDs, YOLO, etc.

        # Changing factors (keeping the app, app format, its menu, scale, resolution constant):
        # 1) Png is a lossless image format but it has compression (may change the results)
        # 2) Rendering engines can introduce subtle variations due to anti-aliasing, font rendering, or other graphical effects
        # 3) Screenshots may contain noise, compression artifacts, or unintended elements
        heat_map = cv2.matchTemplate(screenshot_numpy, template, cv2.TM_CCOEFF_NORMED)
        #_, _, _, max_loc = cv2.minMaxLoc(heat_map)
        _, max_val, _, max_loc = cv2.minMaxLoc(heat_map)

        # If the method is TM_SQDIFF or TM_SQDIFF_NORMED, take the minimum value
        # Otherwise, take the maximum value
        if cv2.TM_CCOEFF_NORMED in [cv2.TM_SQDIFF, cv2.TM_SQDIFF_NORMED]:
            matching_score = max_val
        else:
            matching_score = max_val

        # Get the top-left corner coordinates of the matched region
        x, y = max_loc

        self.print(f"screenshot_search: matching score: {matching_score:.4f}")
        self.print(f"screenshot_search: found template image at position (x, y): ({x}, {y})")

        # Create an inverted template
        inverted_template = cv2.bitwise_not(template)

        # Overlay the inverted template on the original image
        screenshot_numpy[y:y + template.shape[0], x:x + template.shape[1]] = inverted_template

        # Save the resulting image
        if (CV2_TEST_IMAGES_SAVE):
            image_name = CV2_IMAGES + "\\" + self.datetime_now_get_string() + "_3_Result.png"
            cv2.imwrite(image_name, screenshot_numpy)
            self.print(f"screenshot_search: image saved at {image_name}")

        # Calculate the center coordinates of the template
        center_x = x + template.shape[1] // 2 # Horizontal
        center_y = y + template.shape[0] // 2 # Vertical
        return center_x, center_y, matching_score # matching_score = 1 best, = 0 worst

    # Since I did not succeed to programmatically control the window scrollbars, I create here an image focused around the active state,
    # so that in crop mode the user does not have to use the scrollbars to follow all state changes
    def image_around_active_state_save(self, input_image_default_size_path, output_image_cropped_size_path):
        # screen_width, screen_height = pyautogui.size() # Returns only SCREEN1 values
        for monitor in get_monitors():
            screen_width = monitor.width
            screen_height = monitor.height
            self.print(f"monitor.is_primary {monitor.is_primary}, screen resolution: {monitor.width} x {monitor.height}")
            # screen_width = SCREEN_NR2_WIDTH # + 0.3 * 1920
            # screen_height = SCREEN_NR2_HEIGHT # + 0.3 * 1080
            if (monitor.is_primary == False): 
                screen_width = int(monitor.width * SCREEN_NR2_WIDTH_FACTOR)
                screen_height = int(monitor.height * SCREEN_NR2_HEIGHT_FACTOR)

        self.print(f"Screen resolution adapted: {screen_width} x {screen_height}")

        # Load the source image
        image = cv2.imread(input_image_default_size_path)

        # Convert BGR to HSV
        image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define the lower and upper bounds for red color (adjust as needed)
        lower_red = np.array([0, 90, 50])
        upper_red = np.array([5, 255, 255])

        # Create a mask for red regions
        mask1 = cv2.inRange(image_hsv, lower_red, upper_red)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Initialize variables for cropping
        max_area = 0
        max_contour = None

        # Find the largest red region
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > max_area:
                max_area = area
                max_contour = contour

        # Get the bounding rectangle for the largest red region
        x, y, w, h = cv2.boundingRect(max_contour)

        # Calculate the center of the red region
        center_x = x + w // 2
        center_y = y + h // 2
        # Center can not be less than the half of the available screen size
        if center_x < screen_width / 2:
            center_x = int(screen_width / 2)
        if center_y < screen_height / 2:
            center_y = int(screen_height / 2)

        # Calculate the cropping boundaries
        crop_left = max(0, center_x - screen_width // 2)
        crop_right = min(image.shape[1], center_x + screen_width // 2)
        crop_top = max(0, center_y - screen_height // 2)
        crop_bottom = min(image.shape[0], center_y + screen_height // 2)

        # Crop the image
        cropped_image = image[crop_top:crop_bottom, crop_left:crop_right]
        cv2.imwrite(output_image_cropped_size_path, cropped_image)

        # Resize the cropped image to match screen resolution
        # screen_width_new = int(screen_width)
        # screen_height_new = int(screen_height)
        # cropped_image_resized = cv2.resize(cropped_image, (screen_width_new, screen_height_new))
        # cv2.imwrite(output_image_cropped_size_path, cropped_image_resized)

if __name__ == '__main__':
    log_file = FLLogger("TestFLCV2")
    my_FLCV2 = FLCV2(log_file)
    #my_FLCV2.image_around_active_state_save("", "")

    #template_file_path = r"D:\Banfi\Github\Fluently\Errors\CV2ImagesProblems\RecordingsPaused\03\2024_08_29__11_55_18_2_Template.png"
    template_file_path = r"D:\Banfi\Github\Fluently\Errors\CV2ImagesProblems\RecordingsPaused\03\RecordingPaused.png"
    my_FLCV2.screenshot_search(template_file_path, APPLICATION_ARTECSTUDIO)





    template_file_path = r"D:\Banfi\Github\Fluently\Releases\2024_03_13_Teachix\Data\Input\CommandImages\Taskbar\CmdStartArtecStudio.png"
    screenshot_search(template_file_path, APPLICATION_TASKBAR)

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
    cv2.imwrite("D:/Banfi/Github/Fluently/Code/Halcon/PyCharm01/segmentations/segmentation01.png", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
