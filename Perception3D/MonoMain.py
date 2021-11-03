# -*- coding: utf-8 -*-
from Calibration import MonoCalibration
from Rectification import MonoRectification


# Acquisition

# Calibration
calibrater = MonoCalibration(cols=5, rows=8, patternSize_m=0.0265, patternType="chessboard")
calibrater.calibrate()

# Visualization
# calibrater.visualizeBoards() # Set width and height for better visualization
# calibrater.plotRMS()

# Rectification
rectificater = MonoRectification(calibrater.cameraMatrix, calibrater.distCoeffs, calibrater.imageSize)
rectificater.display(cameraId=0)


