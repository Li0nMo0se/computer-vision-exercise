# -*- coding: utf-8 -*-
import copy
import math
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

def rotationMatrixFromVectors(vec1, vec2):
    """ Find the rotation matrix that aligns vec1 to vec2

    :param vec1: A 3d "source" vector
    :param vec2: A 3d "destination" vector
    :return mat: A transform matrix (3x3) which when applied to vec1, aligns it with vec2.
    """
    # Find and implement a solution here
    a, b = (vec1 / np.linalg.norm(vec1)).reshape(3), (vec2 / np.linalg.norm(vec2)).reshape(3)
    v = np.cross(a, b)
    c = np.dot(a, b)
    s = np.linalg.norm(v)
    kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    rotation_matrix = np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s ** 2))
    return rotation_matrix

class Segmentation:
    def __init__(self, file = 'data/cropped_1.ply'):
        self.pointCloud = o3d.io.read_point_cloud(file)

    def removeOutliers(self, display = False):
        local_point_cloud, inlier_id =\
            self.pointCloud.remove_statistical_outlier(nb_neighbors=40, std_ratio=2.0)

        inlierCloud = self.pointCloud.select_by_index(inlier_id)
        outlierCloud = self.pointCloud.select_by_index(inlier_id, invert=True)

        if display:
            box = self.pointCloud.get_axis_aligned_bounding_box()
            frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05, origin=np.asarray(box.get_box_points())[0])
            outlierCloud.paint_uniform_color([1, 0, 0])
            o3d.visualization.draw_geometries([inlierCloud, outlierCloud, box, frame])   

        self.pointCloud = inlierCloud

    def computeNormals(self, normalize = True, alignVector = [] ):
        # Implement normals estimation here with an Hybrid KD-Tree Search Param
        self.pointCloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

        alignVector = np.asarray(alignVector)
        if alignVector.size == 3:
            # Implement normals alignement here
            self.pointCloud.orient_normals_to_align_with_direction(alignVector)

        if normalize:
            # Implement normals normalization here
            self.pointCloud.normalize_normals()

        self.normals = np.asarray(self.pointCloud.normals)
        return self.normals

    def estimateFloorNormal(self, bins = 10):
        histX, edgesX = np.histogram(self.normals[:,0],  bins = bins)
        histY, edgesY = np.histogram(self.normals[:,1], bins = bins)
        histZ, edgesZ = np.histogram(self.normals[:,2], bins = bins)

        minX, maxX = (edgesX[np.argmax(histX)], edgesX[np.argmax(histX)+1])
        minY, maxY = (edgesY[np.argmax(histY)], edgesY[np.argmax(histY)+1])
        minZ, maxZ = (edgesZ[np.argmax(histZ)], edgesZ[np.argmax(histZ)+1])

        floorNormals = np.empty((0, 3))
        for line in self.normals:
            if minX <= line[0] <= maxX:
                if minY <= line[1] <= maxY:
                    if minZ <= line[2] <= maxZ:
                        floorNormals = np.append(floorNormals, [line], axis=0)
        # self.floorNormals contains all the normals within the maximum X, Y, Z histogram
        self.floorNormal = floorNormals.mean(axis=0)
        return self.floorNormal

    def alignFloor(self):
        # Align the floor with the horizontal plane here
        rotMatrix = rotationMatrixFromVectors(self.floorNormal, np.array([0., 1., 0.]))
        self.pointCloud.rotate(rotMatrix, self.pointCloud.get_center())

    def removeFloor(self, bins = 10):
        xyz = np.asarray(self.pointCloud.points)
        newXYZ = np.empty((0, 3))

        # Find and remove the floor here
        # If y coordinate is over the floor y axis
        histY, edgesY = np.histogram(xyz[:,1], bins = bins)

        limit_floor = edgesY[np.argmax(histY)+1]

        eps = 0.01
        idx = []
        for curr_idx, point in enumerate(xyz):
            if point[1] >= limit_floor + eps:
                # newXYZ = np.append(newXYZ, [point], axis=0)
                idx.append(curr_idx)

        # self.pointCloud.points = o3d.utility.Vector3dVector(newXYZ)
        self.pointCloud = self.pointCloud.select_by_index(idx)

    def display(self):
        box = self.pointCloud.get_axis_aligned_bounding_box()
        frame = o3d.geometry.TriangleMesh.create_coordinate_frame( size=0.05, origin=np.asarray(box.get_box_points())[0])
        # o3d.visualization.draw_geometries([self.pointCloud, box, frame])
        o3d.visualization.draw_geometries_with_editing([self.pointCloud])

    def normalsHistogram(self, bins = 20):
        fig, ax = plt.subplots(1, 3, sharey=True, tight_layout=True)
        ax[0].set_title('X axis Hist')
        ax[1].set_title('Y axis Hist')
        ax[2].set_title('Z axis Hist')

        ax[0].hist(self.normals[:,0], bins= bins)
        ax[1].hist(self.normals[:,1], bins= bins)
        ax[2].hist(self.normals[:,2], bins= bins)
        plt.show()


