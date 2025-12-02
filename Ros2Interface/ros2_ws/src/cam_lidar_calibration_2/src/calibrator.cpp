#include <iostream> 
#include <vector>
//#include <stdexcept>

#include "ament_index_cpp/get_package_share_directory.hpp"


//!opencv
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>

int main(int argc, char** argv) {

    std::string node_name = "calibrator_node";

    rclcpp::init(argc, argv);
  
    //node_name = ros::this_node::getName();
  
    rclcpp::Node::SharedPtr nh = rclcpp::Node::make_shared(node_name);

    std::string share_dir = ament_index_cpp::get_package_share_directory("cam_lidar_calibration_2");
    std::string filepath = nh->declare_parameter<std::string>("image_path", "images/image2.png");
    filepath = share_dir+"/"+filepath;


    // --- Chessboard Parameters ---
    // Define the number of inner corners per chessboard row and column
    cv::Size patternSize(5, 7);
    // Define the real-world size of a square (e.g., 1.0 unit)
    float squareSize = 0.105f;

    // --- Prepare Object Points ---
    // Create a vector of 3D points for the chessboard corners in world coordinates.
    // These points are in the chessboard coordinate system (z = 0)
    std::vector<cv::Point3f> objp;
    for (int i = 0; i < patternSize.height; i++) {
        for (int j = 0; j < patternSize.width; j++) {
            objp.push_back(cv::Point3f(j * squareSize, i * squareSize, 0));
        }
    }

    // Load an image
    cv::Mat image = cv::imread(filepath,cv::IMREAD_UNCHANGED);
    if (image.empty()) {
        //throw std::runtime_error("Error: Unable to open the image file." << std::endl);
        RCLCPP_ERROR(nh->get_logger(), "Unable to open the image file.");
        return -1;
    }

    int rows = image.rows;
    int cols = image.cols;
    int channels = image.channels();
    int image_size[3] = {rows,cols,channels};

    // Convert to grayscale
    cv::Mat gray; 
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    /*
    cv::Mat hsv,msk;
    //cv::Mat lwr = cv::Mat(1,3,CV_8UC1,{0, 0, 252});//143
    //cv::Mat upr = cv::Mat(1,3,CV_8UC1,{179, 61, 252});
    cv::Scalar lwr(0, 0, 143);  // Lower bound (H, S, V)
    cv::Scalar upr(179, 61, 252);  // Upper bound (H, S, V)
    cv::cvtColor(image, hsv,cv::COLOR_BGR2HSV);
    cv::inRange(hsv, lwr, upr,msk);

    cv::Mat dlt,res;
    cv::Mat krn = cv::getStructuringElement(cv::MORPH_RECT, {50, 30});
    cv::dilate(msk,dlt, krn, cv::Point(-1, -1),5);
    cv::bitwise_and(dlt, msk,res);
    res  = 255 - res;
    
    //cv::Mat value;
    //extractChannel(hsv,value,0);
    //cv::cvtColor(hsv, image,cv::COLOR_HSV2RGB);
    cv::imshow("res", res);
    */
    cv::waitKey(0);
    cv::destroyAllWindows();

    //cv::Mat result(res,CV_8UC1);

    // --- Detect Chessboard Corners ---
    std::vector<cv::Point2f> corners;
    bool found = cv::findChessboardCorners(gray, patternSize, corners,
                                       cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE); //| cv::CALIB_CB_FAST_CHECK);
    if (!found) {
        RCLCPP_ERROR(nh->get_logger(), "Chessboard corners not found.");
        //return -1;
    }
    
    // Enhance corner detection accuracy
    cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001);
    cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1), criteria);
  
    // Draw detected corners for visualization
    //cv::drawChessboardCorners(image, patternSize, cv::Mat(corners), found);
  
    
    std::vector<double> cam_matrix_v = nh->declare_parameter<std::vector<double>>("cam_matrix", {959.554,0.0,940.789,0.0,960.194,670.737,0.0,0.0,1});
    std::vector<double> distortion_c_v = nh->declare_parameter<std::vector<double>>("distortion_coefficients", {-0.097824,0.141429,-0.148385,0.055918});

    cv::Mat cam_matrix(3,3,CV_64FC1,cam_matrix_v.data());
    cv::Mat distortion_c(1,4,CV_64FC1,distortion_c_v.data());

    //Undistort

    //cv::Mat undistorted;
    //cv::undistort(image, undistorted, cam_matrix, distortion_c);

    // Display the output
    //cv::imshow("Output", undistorted);
    //cv::waitKey(0);
    //cv::destroyAllWindows();

        // --- Pose Estimation ---
    // Compute the rotation and translation vectors using solvePnP

    std::vector<cv::Point2f> corners_rsh =  corners;//corners_reshape(corners,patternSize);
    //std::vector<cv::Point2f> corners_rsh2 = corners_reshape(corners,patternSize);

    cv::Mat rvec, tvec;
    bool solved = solvePnP(objp, corners_rsh, cam_matrix, distortion_c, rvec, tvec);
    if (!solved) {
        RCLCPP_ERROR(nh->get_logger(), "Could not solve PnP.");
        return -1;
    }

    // --- Define 3D Points for Projection ---
    // Here we define three points representing the endpoints of the coordinate axes.
    // Adjust axis length as necessary.
    float axisLength = 1 * squareSize;
    std::vector<cv::Point3f> axisPoints;
    axisPoints.push_back(cv::Point3f(axisLength, 0, 0));   // X axis (red)
    axisPoints.push_back(cv::Point3f(0, axisLength, 0));   // Y axis (green)
    axisPoints.push_back(cv::Point3f(0, 0, -axisLength));  // Z axis (blue; negative goes into the scene)

    // --- Project 3D Points to Image Plane ---
    std::vector<cv::Point2f> imgPoints;
    projectPoints(axisPoints, rvec, tvec, cam_matrix, distortion_c, imgPoints);

    // --- Draw the Axes on the Image ---
    // Use the first detected chessboard corner as the origin
    cv::Point2f origin = corners_rsh[0];
    line(image, origin, imgPoints[0], cv::Scalar(0, 0, 255), 5); // Red for X axis
    line(image, origin, imgPoints[1], cv::Scalar(0, 255, 0), 5); // Green for Y axis
    line(image, origin, imgPoints[2], cv::Scalar(255, 0, 0), 5); // Blue for Z axis

    // Display the output
    cv::imshow("Output", image);
    cv::waitKey(0);
    cv::destroyAllWindows();

    //Undistort

    cv::Mat undistorted;
    cv::undistort(image, undistorted, cam_matrix, distortion_c);

    std::vector<cv::Point2f> imagePoints;
    //cv::Mat R;
    //cv::Rodrigues(rvec, R); // Convert rvec to 3x3 rotation matrix

    axisPoints.push_back(objp.at(0));
    cv::projectPoints(axisPoints, rvec, tvec, cam_matrix, cv::Mat(), imagePoints);


    // Example: Draw lines representing the axes starting from the first chessboard corner
    origin = imagePoints[3];  // Make sure this is also from the undistorted image (or remapped accordingly)
    cv::line(undistorted, origin, imagePoints[0], cv::Scalar(0, 0, 255), 5); // Red: X axis
    cv::line(undistorted, origin, imagePoints[1], cv::Scalar(0, 255, 0), 5); // Green: Y axis
    cv::line(undistorted, origin, imagePoints[2], cv::Scalar(255, 0, 0), 5); // Blue: Z axis

    // Show the final undistorted image with overlays
    cv::imshow("Undistorted with Projections", undistorted);
    cv::waitKey(0);
    cv::destroyAllWindows();
    return 0;
}
