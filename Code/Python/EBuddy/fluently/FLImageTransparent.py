# Tool for creating transparent images

from PIL import Image

from FLConstants import IMAGE_TRANSPARENT_PATH


def create_transparent_png(width, height, file_name):
    # Create a new image with RGBA mode
    image = Image.new("RGBA", (width, height), (0, 0, 0, 0))

    # Save the image
    image.save(file_name, "PNG")

if __name__ == '__main__':
    image_path = r'D:\Banfi\Github\Fluently\Releases\2024_07_09_EBuddy\Data\Input\App\ArtecStudio\Hints\Transparent.PNG'
    #image_path = IMAGE_TRANSPARENT_PATH
    create_transparent_png(500, 1000, image_path)
