import cv2
import numpy as np

def normalize_and_colorize(image_path, output_path):
    # Load the 16-bit PNG image
    depth_image = cv2.imread(image_path, cv2.IMREAD_UNCHANGED)

    # Check if the image is loaded successfully
    if depth_image is None:
        print(f"Failed to load image: {image_path}")
        return
    
    # Normalize the image to 8-bit (0-255)
    normalized_image = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
    
    # Apply a colormap to the normalized image
    colorized_image = cv2.applyColorMap(normalized_image, cv2.COLORMAP_JET)
    
    # Save the colorized image
    cv2.imwrite(output_path, colorized_image)

    print(f"Colorized image saved at: {output_path}")

# Specify the path to the 16-bit PNG depth image
dpath = '/home/vigir3d/Datasets/cattle_scans/farm_scan1/Animal_482_2/anima_482_12/'
image_path = dpath + 'fg_depth.png'

# Specify the path where the colorized image will be saved
output_path = dpath + 'fg_depth_colored.png'

normalize_and_colorize(image_path, output_path)
