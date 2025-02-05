#!/usr/bin/env python3
"""
Module: visual_odometry
Description: Implements the VisualOdometry class that orchestrates the visual odometry pipeline.
"""

import cv2
import numpy as np
from typing import List
from feature_utils.feature_utils import FeatureUtils
from geometry_3d.geometry_3d import Geometry3D

class VisualOdometry:
    """
    Processes a sequence of images to recover camera poses and reconstruct a sparse 3D point cloud.
    """
    def __init__(self, image_paths, camera_intrinsics):
        self.camera_intrinsics = camera_intrinsics
        self.image_paths = image_paths
        self.images = [cv2.imread(path, cv2.IMREAD_GRAYSCALE) for path in image_paths]
        self.num_images = len(self.images)
        # Initialize the first camera pose as identity (camera at origin)
        self.camera_poses = [np.hstack((np.eye(3), np.zeros((3, 1))))]
        self.feature_utils = FeatureUtils()
        self.pose_estimator = Geometry3D(camera_intrinsics)
        self.keypoints = []
        self.descriptors = []
        self.point_cloud = []

    def extract_features(self):
        """
        Detect keypoints and compute descriptors for each image.
        """
        for idx, img in enumerate(self.images):
            kp, des = self.feature_utils.detect_and_compute(img)
            self.keypoints.append(kp)
            self.descriptors.append(des)
            print(f"[INFO] Extracted {len(kp)} keypoints from image {idx}")

    def compute_camera_poses(self):
        """
        For each subsequent image, match features to previous images,
        estimate the relative pose, and compute the global pose.
        """
        pcd_size = 0
        for i in range(1, self.num_images):
            best_match_index = None
            max_matches = 0
            best_matches = None

            # Find the previous image with the most matches
            for j in range(i):
                matches = self.feature_utils.match(self.descriptors[j], self.descriptors[i])
                if len(matches) > max_matches:
                    best_match_index = j
                    max_matches = len(matches)
                    best_matches = matches

            if best_matches is None or max_matches < 8:
                print(f"[WARNING] Not enough matches for image {i}. Skipping...")
                self.camera_poses.append(self.camera_poses[-1])
                continue

            E, mask, inlier_pts1, inlier_pts2 = self.pose_estimator.estimate_essential_matrix(
                self.keypoints[best_match_index],
                self.keypoints[i],
                best_matches
            )
            rel_pose = self.pose_estimator.recover_pose(E, inlier_pts1, inlier_pts2)
            global_pose = self._transform_world_coord(self.camera_poses[best_match_index], rel_pose)
            self.camera_poses.append(global_pose)
            print(f"[INFO] Computed pose for image {i} using image {best_match_index} with {len(inlier_pts1)} inliers.")

            if inlier_pts1.shape[0] > 100:
                pts3d = self.pose_estimator.triangulate_points(self.camera_poses[best_match_index],
                                                        self.camera_poses[-1],
                                                        inlier_pts1,
                                                        inlier_pts2)
                print(f"[INFO] Triangulated {pts3d.shape[0]} 3D points.")

                if(pts3d.shape[0] > pcd_size):
                    pcd_size = pts3d.shape[0]
                    self.point_cloud = pts3d
            else:
                print(f"[INFO] Not enough inliers for triangulation. Skipping...")

    def _transform_world_coord(self, pose1, pose2):
        """
        Transforms a relative pose (pose2 relative to pose1) into the global coordinate system.
        """
        T1 = np.eye(4)
        T2 = np.eye(4)
        T1[:3, :] = pose1
        T2[:3, :] = pose2
        T_global = np.dot(T1, T2)
        return T_global[:3, :]

    def run(self):
        """
        Runs the complete visual odometry pipeline.
        """
        print("[INFO] Starting feature extraction...")
        self.extract_features()
        print("[INFO] Estimating camera poses...")
        self.compute_camera_poses()
        print("[INFO] Triangulating points...")
        # self.triangulate()
