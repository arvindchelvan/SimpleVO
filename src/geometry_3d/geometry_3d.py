#!/usr/bin/env python3
"""
Module: geometry_3d
Description: Provides the 3DGeometry class for estimating the essential matrix,
recovering the camera pose, and triangulating 3D points.
"""

import cv2
import numpy as np

class Geometry3D:
    """
    Contains methods for essential matrix estimation, pose recovery, and 3D point triangulation.
    """
    def __init__(self, camera_intrinsics: np.ndarray):
        self.camera_intrinsics = camera_intrinsics

    def estimate_essential_matrix(self,
                                  kp1,
                                  kp2,
                                  matches,
                                  threshold = 5,
                                  prob = 0.99):
        """
        Computes the essential matrix from matched keypoints and selects inliers.
        """
        points1 = np.array([kp1[m.queryIdx].pt for m in matches])
        points2 = np.array([kp2[m.trainIdx].pt for m in matches])

        E, mask = cv2.findEssentialMat(points1, points2,
                                       self.camera_intrinsics,
                                       method=cv2.RANSAC,
                                       prob=prob,
                                       threshold=threshold)
        
        # Select inlier points
        inlier_points1 = points1[mask.ravel() == 1]
        inlier_points2 = points2[mask.ravel() == 1]
        return E, mask, inlier_points1, inlier_points2

    def recover_pose(self, E,
                     points1,
                     points2):
        """
        Recovers the relative camera pose (rotation and translation) from the essential matrix.
        Returns a 3x4 projection matrix.
        """
        _retval, R, t, _mask = cv2.recoverPose(E, points1, points2, self.camera_intrinsics)
        proj_mat =  np.hstack((R, t))
        return proj_mat
    
    def estimatePosePnP(self, points1, keypoints2):
        """
        Estimate the camera pose using PnP (Perspective-n-Point) algorithm.
        """
        pass

    def triangulate_points(self,
                           proj_mat1,
                           proj_mat2,
                           points1,
                           points2):
        """
        Triangulates 3D points given two projection matrices and corresponding image points.
        """
        P1 = np.dot(self.camera_intrinsics, proj_mat1)
        P2 = np.dot(self.camera_intrinsics, proj_mat2)
        points4d = cv2.triangulatePoints(P1, P2, points1.T, points2.T)
        points3d = cv2.convertPointsFromHomogeneous(points4d.T).reshape(-1, 3)
        return points3d
