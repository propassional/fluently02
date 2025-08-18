from PIL import Image

def extract_image(screenshot_image_path, template_image_path, x, y):
    # Open the screenshot image
    screenshot_image = Image.open(screenshot_image_path)

    # Open the template image to get its dimensions
    template_image = Image.open(template_image_path)
    template_width, template_height = template_image.size

    # Define the bounding box for the region to be extracted
    left = x
    top = y
    right = x + template_width
    bottom = y + template_height

    # Extract the region from the screenshot image
    image_result = screenshot_image.crop((left, top, right, bottom))

    return image_result

if __name__ == '__main__':
    screenshot_image_path = r"D:\Banfi\Github\Fluently\Errors\CV2ImagesProblems\RecordingsPaused\03\2024_08_29__11_55_18_1_Screenshot.png"
    template_image_path = r"D:\Banfi\Github\Fluently\Errors\CV2ImagesProblems\RecordingsPaused\03\RecordingPaused.png"

    x, y = 100, 150

    extracted_image = extract_image(screenshot_image_path, template_image_path, x, y)
    extracted_image.show()  # Display the extracted image