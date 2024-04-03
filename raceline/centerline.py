import numpy as np
import cv2

image = cv2.imread('Austin/Austin_map_grey.png', cv2.IMREAD_GRAYSCALE)

# Threshold the image to separate white areas (outside track) from the rest
_, thresh = cv2.threshold(image, 200, 255, cv2.THRESH_BINARY)

# Display the result
cv2.imshow('Result', thresh)
cv2.waitKey(0)
cv2.destroyAllWindows()