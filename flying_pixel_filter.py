def eliminate_flying_pixels(depth_image, ws, threshold):
   
    # Get image size
    height, width = depth_image.shape
    # Create an empty array for the result
    result = np.zeros_like(depth_image, dtype=float)

    # Iterate over the entire image
    for cy in range(height):
        for cx in range(width):
            # Set the range for the window
            x_start, x_end = max(0, cx - ws), min(width, cx + ws + 1)
            y_start, y_end = max(0, cy - ws), min(height, cy + ws + 1)
            
            # Get the window
            window = depth_image[y_start:y_end, x_start:x_end]

            # Calculate the sum of absolute differences
            result[cy, cx] = np.sum(np.abs(window - depth_image[cy, cx]))
    count = np.sum(result > threshold)

    depth_image[result > threshold] = 0
    return  depth_image