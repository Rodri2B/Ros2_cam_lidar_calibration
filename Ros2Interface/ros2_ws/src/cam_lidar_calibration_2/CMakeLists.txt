cmake_minimum_required(VERSION 3.14.4)
project(cam_lidar_calibration_2)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -O3 -Wextra -Wpedantic)
endif()

list(APPEND CMAKE_PREFIX_PATH
     "$ENV{HOME}/RosLibs/boost1.88"
) 

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost 1.88 REQUIRED COMPONENTS thread) 
find_package(Eigen3 REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

#export LD_LIBRARY_PATH=$HOME/RosLibs/boost1.88/lib:$LD_LIBRARY_PATH

#add_executable(calibrator src/calibrator.cpp)

add_executable(calibrator_bridge
  src/calibrator_bridge.cpp
  include/cam_lidar_calibration_2/simulator_bridge.hpp
)
add_executable(simulator_bridge 
  src/simulator_bridge.cpp
  include/cam_lidar_calibration_2/simulator_bridge.hpp 
)

target_include_directories(simulator_bridge PUBLIC include/cam_lidar_calibration_2/)
target_include_directories(calibrator_bridge PUBLIC include/cam_lidar_calibration_2/)
target_include_directories(calibrator_bridge PRIVATE ${EIGEN3_INCLUDE_DIRS})

#target_link_libraries(calibrator PUBLIC ${OpenCV_LIBRARIES})

#ament_target_dependencies(calibrator rclcpp std_msgs sensor_msgs cv_bridge image_transport OpenCV)
#ament_target_dependencies(calibrator_bridge rclcpp std_msgs sensor_msgs) 

ament_target_dependencies(calibrator_bridge rclcpp std_msgs sensor_msgs geometry_msgs tf2 tf2_ros)

ament_target_dependencies(simulator_bridge rclcpp std_msgs sensor_msgs geometry_msgs tf2 tf2_ros)


# Boost libraries go here
target_link_libraries(simulator_bridge ${Boost_LIBRARIES} Boost::thread)

target_link_libraries(calibrator_bridge ${Boost_LIBRARIES} Boost::thread)
# Do not do Eigen3::Eigen it’s not necessary for header-only usage

install(TARGETS 
  calibrator_bridge 
  simulator_bridge
  DESTINATION lib/${PROJECT_NAME})

#install(
#  DIRECTORY images/
#  DESTINATION share/${PROJECT_NAME}/images)

install(
  FILES 
  "cfg/config.yaml"
  "launch/simulator_bridge.launch.py"
  "launch/calibrator_bridge.launch.py"
  DESTINATION share/${PROJECT_NAME})

ament_package()
