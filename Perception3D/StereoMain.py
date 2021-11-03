# -*- coding: utf-8 -*-
from Calibration import StereoCalibration
from Rectification import StereoRectification
import cv2 as cv
import numpy as np


# Calibration
calibrater = StereoCalibration(cols=6, rows=7, patternSize_m=0.108, patternType="chessboard")
calibrater.calibrate()

# Visualization
calibrater.plotRMS()

# Rectification
rectificater = StereoRectification(calibrater.cameraMatrixLeft, calibrater.distCoeffsLeft, calibrater.cameraMatrixRight,
                                   calibrater.distCoeffsRight, calibrater.imageSize, calibrater.R, calibrater.T)
left = cv.imread('data/stereo/MinnieRawLeft.png', cv.IMREAD_GRAYSCALE)
right = cv.imread('data/stereo/MinnieRawRight.png', cv.IMREAD_GRAYSCALE)
rectificater.display(left, right)
# 3D reconstruction


