#!/usr/bin/env python3
"""
Module: main
Description: Entry point for running the visual odometry pipeline.
"""

import glob
import numpy as np
from vo.vo import VisualOdometry
from viz.pcd_visualizer import PointCloudVisualizer

def main():
    # Define the camera intrinsic matrix.
    camera_intrinsics = np.array([
        [1520.4,    0, 302.32],
        [   0, 1525.9, 246.87],
        [   0,    0,      1 ]
    ])

    # Get the list of image file paths.
    image_files = glob.glob('../templeSparseRing/*png')
    if not image_files:
        print("[ERROR] No images found in 'templeRing' directory.")
        return

    # Initialize and run the visual odometry pipeline.
    vo = VisualOdometry(image_files, camera_intrinsics)
    vo.run()

    # Visualize the reconstructed point cloud if available.
    if vo.point_cloud.size > 0:
        visualizer = PointCloudVisualizer(vo.point_cloud)
        visualizer.visualize()
    else:
        print("[ERROR] No point cloud was generated.")

if __name__ == '__main__':
    main()
