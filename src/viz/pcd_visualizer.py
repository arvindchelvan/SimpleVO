#!/usr/bin/env python3
"""
Module: point_cloud_visualizer
Description: Contains the PointCloudVisualizer class for 3D point cloud visualization using Open3D.
"""

import open3d as o3d
import numpy as np

class PointCloudVisualizer:
    """
    Visualizes a 3D point cloud using Open3D.
    """
    def __init__(self, point_cloud):
        self.point_cloud = point_cloud

    def visualize(self):
        """
        Displays the point cloud along with a coordinate frame.
        """
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(self.point_cloud)
        mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.01, origin=[0, 0, 0])
        o3d.visualization.draw_geometries([pcd, mesh_frame])
