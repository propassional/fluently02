# This model warns that it runs much faster with GPU

import easyocr

def read_OCR(image_path):
    # Initialize the reader, takes some seconds
    reader = easyocr.Reader(['en'])
    text_read = reader.readtext(image_path)
    return text_read

if __name__ == '__main__':
    image_path1 = r'D:\Banfi\Github\Fluently\Errors\CV2ImagesProblems\RecordingsPaused\03\RecordingPaused.png'
    text1 = read_OCR(image_path1)
    print(f"Reading image {image_path1} returns {text1}")

    image_path2 = r'D:\Banfi\Github\Fluently\Errors\CV2ImagesProblems\RecordingsPaused\03\2024_08_29__11_55_18_2_Template.png'
    text2 = read_OCR(image_path2)
    print(f"Reading image {image_path2} returns {text2}")
