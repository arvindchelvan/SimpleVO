# SimpleVO

SimpleVO is a modular visual odometry frontend system that reconstructs a sparse 3D point cloud from a sequence of images. It leverages feature extraction, feature matching, pose estimation, and triangulation to compute camera trajectories and generate a 3D representation of the scene.

## Modules

- **Feature Extraction & Matching:**  
  Combines SIFT-based keypoint detection with a FLANN-based matcher (using a ratio test) for robust feature matching.

- **Pose Estimation:**  
  Estimates the essential matrix between image pairs and recovers the relative camera pose. Global camera poses are computed by transforming relative poses into a consistent coordinate system.

- **3D Reconstruction:**  
  Triangulates 3D points from matching keypoints to produce a sparse point cloud of the scene.

- **Visualization:**  
  Uses Open3D to visualize the resulting point cloud and coordinate frames of the camera poses.

## Installation

### Prerequisites

- Python 3.6+
- [NumPy](https://numpy.org/)
- [SciPy](https://www.scipy.org/)
- [Matplotlib](https://matplotlib.org/)
- [OpenCV (with opencv-contrib-python)](https://opencv.org/)
- [Open3D](http://www.open3d.org/)


## Build and Run Docker Container

   '''
   cd /{path to workspace}/SimpleVO/docker
   ./build_and_run.sh
   '''