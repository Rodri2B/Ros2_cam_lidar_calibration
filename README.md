# Ros2 Camera-Lidar Simulator and Calibrator

This repository contains the code developed for camera and lidar simulation and calibration. It also includes the ROS nodes used to interface both tools with the ROS 2 ecosystem.

The methodology used was based on two publications: ["Automatic extrinsic calibration between a camera and a 3D Lidar using 3D point and plane correspondences"](https://arxiv.org/abs/1904.12433) e ["Optimising the selection of samples for robust lidar camera calibration"](https://arxiv.org/abs/2103.12287). The first publication presents the methodology used for representing samples, establishing metrics, and performing the calibration itself. The second publication provided the metrics used to evaluate the quality of the selected samples, as well as the technique of dividing samples into small groups to assign them to the calibration algorithm in a way that maximizes the accuracy of the final calibration result.

### ⚠️ The Repository is under construction, not all resources have been uploaded and there is still work to be done!!!

### ✅ Tested and Functional
- **Camera and Lidar Simulation** 
- **ROS Interface**

### ❌ Non Functional or Partially Functional
- **Calibration GUI** - most of it built
- **ROS Calibration Tool Interface** - Done, but can only show camera data
- **Sample Selection** - Still to be done
- **Calibration Functionality** - Partially done

# 1. Getting started
## 1.1 Dependencies

| **Dependency Name** | **Version**      | **Functionality**                                             |
|----------------------|------------------|-------------------------------------------------------------|
| Boost                | 1.88             | Comprehensive C++ library for various applications          |
| Eigen                | 3.4              | C++ template library for linear algebra and matrices        |
| Flann                | 1.9.2            | Fast library for nearest neighbor searches                  |
| VTK                  | 9.4.2            | Visualization Toolkit for 3D computer graphics and visualization |
| Qhull                | 2020.2           | Computational geometry library for convex hulls, Delaunay triangulations, etc. |
| PCL                  | 1.15.0           | Point Cloud Library for 2D/3D image processing              |
| OpenCV               | 4.13+contrib     | Open Source Computer Vision Library for real-time image processing |
| Yaml-cpp             | 0.8.0            | YAML parser and emitter for C++                             |
| Assimp               | 5.3.0            | Import and export library for 3D model formats              |
| Glm                  | 1.0.1            | OpenGL Mathematics library for graphics programming         |
| ImGUI                | 1.91.8           | Immediate Mode GUI for creating graphical user interfaces    |
| Glfw                 | 3.5.0            | Library for creating windows and managing OpenGL contexts    |
| Stb-image            | 2.28             | Single-file image loading library for various formats       |
| Freetype             | 2.13.3           | Font rendering library that supports various font formats   |
| OpenGL               | 4.6 Core         | Graphics API for rendering 2D and 3D vector graphics       |








