#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <sensor_msgs/msg/image.hpp>

#include <thread>
#include <atomic>

#include <boost/interprocess/managed_shared_memory.hpp>
//#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>
#include <boost/interprocess/ipc/message_queue.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread.hpp>


#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"



struct Point_XYZIR{
    float x;
    float y;
    float z;
    float intensity;
    uint32_t ring;
    uint32_t padding[3];
};

struct SharedControl {
    bool resized;   // flag for resize notification
};

struct LidarBoundsCfg{
    float x_min;
    float x_max;
    float y_min;
    float y_max;
    float z_min;
    float z_max;
};

