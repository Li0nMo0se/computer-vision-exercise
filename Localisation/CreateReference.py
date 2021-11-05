
import cv2 as cv
import numpy as np

from Homography import PerspectiveCorrection

file="carte"
image = cv.imread(f"data/{file}.jpg") # Read the image of your object

height_px = 635 # size in pixels along Y real: 5.4cm
width_px = 1000 # size in pixels along X real: 8.5cm
outSize = (width_px, height_px)

cor = PerspectiveCorrection()
outImage = cor.process(image, outSize)

# Save your image
cv.imwrite(f"data/{file}_out.jpg", outImage)