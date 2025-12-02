# Ros2 Camera-Lidar Simulator and Calibrator

This repository contains the code developed for camera and lidar simulation and calibration. It also includes the ROS nodes used to interface both tools with the ROS 2 ecosystem.

The methodology used was based on two publications: ["Automatic extrinsic calibration between a camera and a 3D Lidar using 3D point and plane correspondences"](https://arxiv.org/abs/1904.12433) e ["Optimising the selection of samples for robust lidar camera calibration"](https://arxiv.org/abs/2103.12287). The first publication presents the methodology used for representing samples, establishing metrics, and performing the calibration itself. The second publication provided the metrics used to evaluate the quality of the selected samples, as well as the technique of dividing samples into small groups to assign them to the calibration algorithm in a way that maximizes the accuracy of the final calibration result.







