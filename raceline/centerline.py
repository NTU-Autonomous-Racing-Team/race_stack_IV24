import cv2
import numpy as np

# Load the occupancy grid image
image = cv2.imread('Austin/Austin_map.png', 0)  # Load as grayscale

# Apply edge detection
edges = cv2.Canny(image, 30, 200)

# Find contours
contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Sort contours by area in descending order
contours = sorted(contours, key=cv2.contourArea, reverse=True)

# Create a new image filled with zeros, the same size as the original image
filled_image = np.zeros_like(image)

# Always fill the largest contour with white color
cv2.fillPoly(filled_image, contours[0], 255)

# If there is a second contour, fill it with black color
if len(contours) > 1:
    cv2.fillPoly(filled_image, contours[1], 0)

print(edges.shape)

# Display the result
cv2.imshow('Final Image', filled_image)
cv2.waitKey(0)
cv2.destroyAllWindows()
