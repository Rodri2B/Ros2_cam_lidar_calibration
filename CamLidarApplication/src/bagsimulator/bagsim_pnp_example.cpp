#include <iostream> 
#include <vector>
#include <stdexcept>


//!opencv
#include <opencv2/opencv.hpp>
//#include <cv_bridge/cv_bridge.h>
//#include <image_transport/image_transport.hpp>


// --- Chessboard Parameters ---
// Define the number of inner corners per chessboard row and column
//cv::Size patternSize(5, 7);
cv::Size patternSize(6, 8);

// Define the real-world size of a square (e.g., 1.0 unit)
//float squareSize = 0.095f;
//float size_x = 0.610f;
//float size_y = 0.850f;
//float error_x = 0.002f;
//float error_y = 0.0f;

float squareSize = 0.065f;
float size_x = 0.650f;
float size_y = 0.910f;
float error_x = 0.010f;
float error_y = 0.030f;

std::string filepath = "../resources/images/bag.png";

int main(int argc, char** argv) {

    //std::cout << cv::getBuildInformation() << std::endl;

    //Camera settings
    //std::vector<double> cam_matrix_v = {959.554,0.0,940.789,0.0,960.194,670.737,0.0,0.0,1};
    std::vector<double> cam_matrix_v = {1207.11,0.0,700.0,0.0,1207.11,500.0,0.0,0.0,1.0};
    //std::vector<double> distortion_c_v = {-0.097824,0.141429,-0.148385,0.055918,0.0};
    std::vector<double> distortion_c_v = {0.0,0.0,0.0,0.0,0.0};

    cv::Mat cam_matrix(3,3,CV_64FC1,cam_matrix_v.data());
    cv::Mat distortion_c(1,4,CV_64FC1,distortion_c_v.data());

    // --- Prepare Object Points ---
    // Create a vector of 3D points for the chessboard corners in world coordinates.
    // These points are in the chessboard coordinate system (z = 0)
    std::vector<cv::Point3f> board_world_space_points(patternSize.height*patternSize.width);


    for (int i = 0; i < patternSize.height; i++) {
        for (int j = 0; j < patternSize.width; j++) {
            board_world_space_points.at(i*patternSize.width + j) = (cv::Point3f(static_cast<float>(j)*squareSize, static_cast<float>(i)*squareSize, 0.0f));
        }
    }

    //std::vector<cv::Point3f> board_world_space_points;

    //for (int i = 0; i < patternSize.height; i++) {
    //    for (int j = 0; j < patternSize.width; j++) {
    //        board_world_space_points.push_back(cv::Point3f(static_cast<float>(j)*squareSize, static_cast<float>(i)*squareSize, 0.0f));
    //    }
    //}

    cv::Point3f Pattern_center = cv::Point3f(
        static_cast<float>(patternSize.width-1)*
        squareSize*0.5f,
        static_cast<float>(patternSize.height-1)*
        squareSize*0.5f,
        0.0f);

    cv::Point3f Board_center = Pattern_center;
    Board_center.x -= error_x;
    Board_center.y += error_y; //y negativo(pra baixo)

    std::vector<cv::Point3f> Board_corners(5);
    //Board_corners.resize(5);
    Board_corners[0] = Board_center-cv::Point3f(size_x,size_y,0.0f)*0.5f;
    Board_corners[1] = Board_center+cv::Point3f(size_x,-size_y,0.0f)*0.5f;
    Board_corners[2] = Board_center+cv::Point3f(size_x,size_y,0.0f)*0.5f;
    Board_corners[3] = Board_center+cv::Point3f(-size_x,size_y,0.0f)*0.5f;
    Board_corners[4] = Board_center;

    //loop
    
    // Load an image
    cv::Mat image = cv::imread(filepath,cv::IMREAD_UNCHANGED);

    if (image.empty()) {
        throw std::runtime_error("Error: Unable to open the image file.\n");
        return -1;
    }
    
    //int rows = image.rows;
    //int cols = image.cols;
    //int channels = image.channels();
    //int image_size[3] = {rows,cols,channels};

    // Convert to grayscale
    cv::Mat gray; 
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    //Undistort
    cv::Mat undistorted;
    cv::undistort(image, undistorted, cam_matrix, distortion_c);

    // --- Detect Chessboard Corners ---
    std::vector<cv::Point2f> corners;
    bool found = cv::findChessboardCorners(gray, patternSize, corners,
                                       cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE); //| cv::CALIB_CB_FAST_CHECK);
    if (!found) {
        std::cout<< "Chessboard corners not found." <<std::endl;
        //throw std::runtime_error("Chessboard corners not found." << std::endl);
        //return -1;
    }
    
    // Enhance corner detection accuracy
    cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001);
    cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1), criteria);
  
    // Draw detected corners for visualization
    cv::drawChessboardCorners(image, patternSize, cv::Mat(corners), found);
  

        // --- Pose Estimation ---
    // Compute the rotation and translation vectors using solvePnP

    //std::vector<cv::Point2f> corners_rsh =  corners;//corners_reshape(corners,patternSize);

    cv::Mat rvec, tvec;
    //bool solved = solvePnPRansac(board_world_space_points, corners_rsh, cam_matrix, distortion_c, rvec, tvec);
    bool solved = cv::solvePnP(board_world_space_points, corners, cam_matrix, distortion_c, rvec, tvec);
    if (!solved) {
        throw std::runtime_error("Could not solve PnP.\n");
        return -1;
    }

    // --- Define 3D Points for Projection ---
    // Here we define three points representing the endpoints of the coordinate axes.
    // Adjust axis length as necessary.
    float axisLength = 1.0f * squareSize;
    std::vector<cv::Point3f> axisPoints;
    axisPoints.push_back(cv::Point3f(axisLength, 0, 0));   // X axis (red)
    axisPoints.push_back(cv::Point3f(0, axisLength, 0));   // Y axis (green)
    axisPoints.push_back(cv::Point3f(0, 0, -axisLength));  // Z axis (blue; negative goes into the scene)

    // --- Project 3D Points to Image Plane ---
    std::vector<cv::Point2f> imgPoints;
    cv::projectPoints(axisPoints, rvec, tvec, cam_matrix, distortion_c, imgPoints);
    
    std::vector<cv::Point2f> imgBorders;
    cv::projectPoints(Board_corners, rvec, tvec, cam_matrix, distortion_c, imgBorders);

    cv::line(image, imgBorders[0], imgBorders[1], cv::Scalar(255, 0, 255), 3);
    cv::line(image, imgBorders[1], imgBorders[2], cv::Scalar(255, 0, 255), 3);
    cv::line(image, imgBorders[2], imgBorders[3], cv::Scalar(255, 0, 255), 3);
    cv::line(image, imgBorders[3], imgBorders[0], cv::Scalar(255, 0, 255), 3);

    cv::circle(image, imgBorders[4],3, cv::Scalar(255, 0, 255), -1);

    // --- Draw the Axes on the Image ---
    // Use the first detected chessboard corner as the origin
    cv::Point2f origin = corners[0];
    cv::line(image, origin, imgPoints[0], cv::Scalar(0, 0, 255), 5); // Red for X axis
    cv::line(image, origin, imgPoints[1], cv::Scalar(0, 255, 0), 5); // Green for Y axis
    cv::line(image, origin, imgPoints[2], cv::Scalar(255, 0, 0), 5); // Blue for Z axis

    // Display the output
    cv::imshow("Distorted Output", image);
    
    cv::Mat W_to_cam_Rot;
    //cv::Rodrigues(rvec, W_to_cam_Rot); // Convert rvec to 3x3 rotation matrix
    cv::Mat rvec_float, tvec_float;
    rvec.convertTo(rvec_float, CV_32F);
    tvec.convertTo(tvec_float, CV_32F);
    cv::Rodrigues(rvec_float, W_to_cam_Rot);  // then it will be float


    std::vector<cv::Point3f> Board_corners_camera_coord(Board_corners.size());

    for (size_t i = 0; i < Board_corners.size(); i++) {
        const cv::Point3f &Pw = Board_corners[i]; // world
        cv::Mat Pw_mat = (cv::Mat_<float>(3,1) << Pw.x, Pw.y, Pw.z);
        cv::Mat Pc_mat = W_to_cam_Rot * Pw_mat + tvec_float;     // camera coordinates
        Board_corners_camera_coord[i] = cv::Point3f(
            Pc_mat.at<float>(0,0),
            -Pc_mat.at<float>(1,0), //to opencv coord
            -Pc_mat.at<float>(2,0) //to opencv coord
        );
        std::cout << "Corner " << i << ": " << Board_corners_camera_coord[i] << std::endl;
    }
    
    cv::Mat Pw_normal = (cv::Mat_<float>(3,1) << 0.0f, 0.0f, -1.0f);
    cv::Mat Pc_normal = W_to_cam_Rot * Pw_normal;

    cv::Point3f Board_normal_camera_coord = cv::Point3f(
        Pc_normal.at<float>(0,0),
        -Pc_normal.at<float>(1,0), //to opencv coord
        -Pc_normal.at<float>(2,0) //to opencv coord
    );

    std::cout << "Board Normal: " << Board_normal_camera_coord << std::endl;

//    for(uint32_t i = 0;i<5;i++){
//         
//        Board_corners_camera_coord.at(i).x = 
//        W_to_cam_Rot.at<float>(0,0)*Board_corners.at(i).x +
//        W_to_cam_Rot.at<float>(0,1)*Board_corners.at(i).y +
//        W_to_cam_Rot.at<float>(0,2)*Board_corners.at(i).z + tvec.at<float>(0,0);
//
//        Board_corners_camera_coord.at(i).y = 
//        W_to_cam_Rot.at<float>(1,0)*Board_corners.at(i).x +
//        W_to_cam_Rot.at<float>(1,1)*Board_corners.at(i).y +
//        W_to_cam_Rot.at<float>(1,2)*Board_corners.at(i).z + tvec.at<float>(1,0);
//
//        Board_corners_camera_coord.at(i).z = 
//        W_to_cam_Rot.at<float>(2,0)*Board_corners.at(i).x +
//        W_to_cam_Rot.at<float>(2,1)*Board_corners.at(i).y +
//        W_to_cam_Rot.at<float>(2,2)*Board_corners.at(i).z + tvec.at<float>(2,0);
//
//        std::cout<< "Corner " <<i+1<<": "<< Board_corners_camera_coord.at(i) << std::endl;
//    }


    std::vector<cv::Point2f> imageUndistoredPoints;
    axisPoints.push_back(board_world_space_points.at(0));
    cv::projectPoints(axisPoints, rvec, tvec, cam_matrix, cv::Mat(), imageUndistoredPoints);

    std::vector<cv::Point2f> imageUndistoredimgBorders;
    cv::projectPoints(Board_corners, rvec, tvec, cam_matrix, cv::Mat(), imageUndistoredimgBorders);

    cv::line(undistorted, imageUndistoredimgBorders[0], imageUndistoredimgBorders[1], cv::Scalar(255, 0, 255), 3);
    cv::line(undistorted, imageUndistoredimgBorders[1], imageUndistoredimgBorders[2], cv::Scalar(255, 0, 255), 3);
    cv::line(undistorted, imageUndistoredimgBorders[2], imageUndistoredimgBorders[3], cv::Scalar(255, 0, 255), 3);
    cv::line(undistorted, imageUndistoredimgBorders[3], imageUndistoredimgBorders[0], cv::Scalar(255, 0, 255), 3);

    cv::circle(undistorted, imageUndistoredimgBorders[4],3, cv::Scalar(255, 0, 255), -1);

    // Example: Draw lines representing the axes starting from the first chessboard corner
    origin = imageUndistoredPoints[3];  // Make sure this is also from the undistorted image (or remapped accordingly)
    cv::line(undistorted, origin, imageUndistoredPoints[0], cv::Scalar(0, 0, 255), 5); // Red: X axis
    cv::line(undistorted, origin, imageUndistoredPoints[1], cv::Scalar(0, 255, 0), 5); // Green: Y axis
    cv::line(undistorted, origin, imageUndistoredPoints[2], cv::Scalar(255, 0, 0), 5); // Blue: Z axis

    // Show the final undistorted image with overlays
    cv::imshow("Undistorted with Projections", undistorted);
    cv::waitKey(0);
    cv::destroyAllWindows();
    return 0;
}