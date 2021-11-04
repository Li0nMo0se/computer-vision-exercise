# -*- coding: utf-8 -*-
import numpy as np
import open3d as o3d

from Segmentation import Segmentation
from Registration import Registration
"""
box="box5"
### Display
segmenter = Segmentation(file=f"data/{box}_cut.ply")
# segmenter = Segmentation(file="data/OtherBoxes/box2.ply")
# segmenter = Segmentation(file="data/box.ply")

### Segmentation
# Select zone of interest

# Load in Segmentation object

# Remove Outliers
segmenter.removeOutliers(display=False)

# Compute normals
segmenter.computeNormals(normalize=True, alignVector=[0, 0, 1])
segmenter.display()
segmenter.normalsHistogram()

# Estimate the floor's normal
floorNormal = segmenter.estimateFloorNormal()

# ALign the floor with the horizontal plane
segmenter.alignFloor()

# Remove the floor
segmenter.removeFloor()
segmenter.display()
o3d.io.write_point_cloud(f"data/{box}_cut_without_floor.ply", segmenter.pointCloud)
"""


### Registration
# Load and visualize the objects

target_file = "box"
source_file = "box5"
target = o3d.io.read_point_cloud(f"data/{target_file}_cut_without_floor.ply")
source = o3d.io.read_point_cloud(f"data/{source_file}_cut_without_floor.ply")
register = Registration(source, target)
register.display()

# Global registration
register.processGlobal()
register.display()

# ICP Registration
register.processICP()
register.display()