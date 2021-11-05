#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import print_function
import cv2 as cv
import numpy as np
from math import sqrt

from Tracking import Tracker
from PoseEstimation import PoseEstimator


# Implement the calibration results of TP1 Perception3D
# From MonoMain.py
cameraMatrix = np.array([[856.35521123, 0., 487.12291102],
                         [0., 804.97433103,  85.0992183 ],
                         [0., 0., 1.]], np.float64)
distCoeffs = np.array([-0.5162738,  0.71400404, 0.05614805, -0.00922509, -0.77096983], np.float64)


# img_object = # Read the image obtained in the first part
img_object = cv.imread("data/carte_out.jpg")
scale = 3
new_dim = (int(img_object.shape[1] / scale), int(img_object.shape[0] / scale))
img_object = cv.resize(img_object, new_dim)

tracker = Tracker(img_object)
tracker.display()

# Dimension of the object in the real world
obj_height = 0.054
obj_width = 0.085
obj_depth = 0.02
objectSize = (obj_height, obj_height, obj_depth)

# Estimate the pose and reproject on the image
posEst = PoseEstimator(img_object, objectSize, cameraMatrix, distCoeffs, tracker.detectorType)
posEst.display()
