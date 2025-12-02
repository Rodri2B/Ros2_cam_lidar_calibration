#include <simulator_bridge.hpp>
//#include <Eigen/Core>
#include <Eigen/Dense>

//struct CoordSystemVec{
//    std::vector<double> right;
//    std::vector<double> up;
//    std::vector<double> front;
//};
//

class CalibPublisher : public rclcpp::Node
{
  public:
    CalibPublisher()
    : Node("calib_publisher")//, print_call_number(0), is_cam_thread_running(true), is_lidar_thread_running(true)
    {
      
        cam_topic_name = this->declare_parameter<std::string>("camera/camera_topic", "/cam/image_color");
        lidar_topic_name = this->declare_parameter<std::string>("lidar/lidar_topic", "/lidarsim/points");

        fifo_lidar_msg_max_size = sizeof(Point_XYZIR)*(static_cast<uint32_t>(this->declare_parameter<int64_t>("lidar/maximum_expected_points_number", 60000L))+1U);

        std::vector<double> lidar_right = this->declare_parameter<std::vector<double>>("lidar/lidar_coord_system/right", {0.0,-1.0,0.0});
        std::vector<double> lidar_up = this->declare_parameter<std::vector<double>>("lidar/lidar_coord_system/up", {0.0,0.0,1.0});
        std::vector<double> lidar_front = this->declare_parameter<std::vector<double>>("lidar/lidar_coord_system/front", {1.0,0.0,0.0});        

        Eigen::Matrix3f lidar_coord_matrix;
        lidar_coord_matrix.col(0) = Eigen::Vector3f(lidar_right[0],lidar_right[1],lidar_right[2]);
        lidar_coord_matrix.col(1) = Eigen::Vector3f(lidar_up[0],lidar_up[1],lidar_up[2]);
        lidar_coord_matrix.col(2) = Eigen::Vector3f(lidar_front[0],lidar_front[1],lidar_front[2]);

        Eigen::Matrix3f calib_coord_matrix;
        lidar_coord_matrix.col(0) = Eigen::Vector3f(0.0f,-1.0f,0.0f);
        lidar_coord_matrix.col(1) = Eigen::Vector3f(0.0f,0.0f,1.0f);
        lidar_coord_matrix.col(2) = Eigen::Vector3f(1.0f,0.0f,0.0f);

        Eigen::Matrix3f calib_coord_matrix_inverse = calib_coord_matrix.inverse();

        lidar_calib_matrix = calib_coord_matrix_inverse*lidar_coord_matrix;

        image_rows_number = static_cast<uint32_t>(this->declare_parameter<int64_t>("camera/image_size/height", 1000L));
        image_cols_number = static_cast<uint32_t>(this->declare_parameter<int64_t>("camera/image_size/width", 1400L));
        
        image_pixels_number = image_rows_number*image_cols_number;
        camera_max_message_size = image_rows_number*image_cols_number*3U + 8U;
        last_camera_message_size = camera_max_message_size;
        Camera_image_pixels.resize(camera_max_message_size);

        InitFIFOS();
        open_bbox_shared_mem();

        // Create callback groups
        camera_cb_group = this->create_callback_group(
          rclcpp::CallbackGroupType::MutuallyExclusive);
        lidar_cb_group = this->create_callback_group(
          rclcpp::CallbackGroupType::MutuallyExclusive);

        // Subscription options for camera
        rclcpp::SubscriptionOptions camera_opts;
        camera_opts.callback_group = camera_cb_group;

        // Subscription options for lidar
        rclcpp::SubscriptionOptions lidar_opts;
        lidar_opts.callback_group = lidar_cb_group;

        cam_subscriber = this->create_subscription<sensor_msgs::msg::Image>(cam_topic_name, 5,
          std::bind(&CalibPublisher::cam_callback, this, std::placeholders::_1),camera_opts);
        lidar_cloud_subscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>(lidar_topic_name, 5,
          std::bind(&CalibPublisher::lidar_callback, this, std::placeholders::_1),lidar_opts);
        //timer_ = this->create_wall_timer(
        //std::chrono::milliseconds(500), std::bind(&CalibPublisher::timer_callback, this));
        //cam_processing_thread = std::thread(&CalibPublisher::cam_processing, this);
        //lidar_cloud_processing_thread = std::thread(&CalibPublisher::lidar_processing, this);

    }

    //~CalibPublisher()
    //{
    //  is_cam_thread_running = false;
    //  is_lidar_thread_running = false;

    //  if(cam_processing_thread.joinable()) cam_processing_thread.join();
    //  if(lidar_cloud_processing_thread.joinable()) lidar_cloud_processing_thread.join();
    //}

  private:

    void cam_callback(const sensor_msgs::msg::Image::SharedPtr image_msg){

        boost::posix_time::ptime now;
        boost::posix_time::ptime abs_time;

        last_camera_sample_timestamp = image_msg->header.stamp;

        std::string encoding = image_msg->encoding;

        if (encoding == "rgb8"){

            camera_message_size = image_msg->height * image_msg->step + 8U;

            //Camera_image_pixels = image_msg->data;  // copy raw data

            if(camera_message_size != last_camera_message_size){

                last_camera_message_size = camera_message_size;
                Camera_image_pixels.resize(camera_message_size);
                image_pixels_number = image_msg->width*image_msg->height;
            }

            
            std::memcpy(Camera_image_pixels.data(),image_msg->data.data(),camera_message_size-8U);
            std::memcpy(Camera_image_pixels.data()+(camera_message_size-8U),&last_camera_sample_timestamp.sec,4U);
            std::memcpy(Camera_image_pixels.data()+(camera_message_size-4U),&last_camera_sample_timestamp.nanosec,4U);

            if(camera_message_size > camera_max_message_size){

                now = boost::posix_time::microsec_clock::universal_time();
                abs_time = now + boost::posix_time::milliseconds(500);

                if(!camera_ctrl_mtx->timed_lock(abs_time))
                {
                    camera_ctrl_mtx->unlock();
                    camera_ctrl_mtx->lock();
                }

                camera_resize_notification->resized = true;

                RecreateCameraFIFO();

                //RCLCPP_INFO(this->get_logger(), "Fifo Cam Recreated\n");

                camera_ctrl_mtx->unlock();

                camera_max_message_size = camera_message_size;
            }

            if(camera_send_queue->get_max_msg() > camera_send_queue->get_num_msg()){
                now = boost::posix_time::microsec_clock::universal_time();
                abs_time = now + boost::posix_time::milliseconds(20);
                camera_send_queue->timed_send(Camera_image_pixels.data(), camera_message_size, /*priority*/ 0, abs_time);
                //if(camera_send_queue->timed_send(Camera_image_pixels.data(), camera_message_size, /*priority*/ 0, abs_time))RCLCPP_INFO(this->get_logger(), "Message Sent\n");
                //mq_result->try_send(lidar_inliers.data(), inliers_size*sizeof(float)+sizeof(detected_board), /*priority*/ 0);
                
            }
        }
        else{
            RCLCPP_ERROR(this->get_logger(), "Error: used enconding type %s is different from rgb8\n",encoding.c_str());
        }
    }

    void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg){

        boost::posix_time::ptime now = boost::posix_time::microsec_clock::universal_time();
        boost::posix_time::ptime abs_time = now + boost::posix_time::milliseconds(500);

        if(!bbox_ctrl_mtx->timed_lock(abs_time))
        {
            bbox_ctrl_mtx->unlock();
            bbox_ctrl_mtx->lock();
        }

        lidar_bounds =  *lidar_bbox_values_pointer;
        
        bbox_ctrl_mtx->unlock();

        //LidarRecieveDataMutex.lock();

        //lidar_points.reserve(cloud_msg->width * cloud_msg->height);
        lidar_points.resize(cloud_msg->width * cloud_msg->height +1U);

        last_lidar_scan_sample_timestamp = cloud_msg->header.stamp;

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud_msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud_msg, "z");
        sensor_msgs::PointCloud2ConstIterator<float> iter_int(*cloud_msg, "intensity");
        sensor_msgs::PointCloud2ConstIterator<uint16_t> iter_ring(*cloud_msg, "ring");

        lidar_scan_size = 0;
        
        for (size_t i = 0; i < cloud_msg->width * cloud_msg->height; ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_int,++iter_ring) {
            if(*iter_x >= lidar_bounds.x_min && *iter_x <= lidar_bounds.x_max &&
               *iter_y >= lidar_bounds.y_min && *iter_y <= lidar_bounds.y_max &&
               *iter_z >= lidar_bounds.z_min && *iter_z <= lidar_bounds.z_max){

                //Point_XYZIR p;
                //p.x = *iter_x;
                //p.y = *iter_y;
                //p.z = *iter_z;
                //p.intensity = *iter_int;
                //p.ring = static_cast<uint32_t>(*iter_ring);
                //lidar_points.push_back(p);

                lidar_points.at(lidar_scan_size).x = lidar_calib_matrix(0,0)*(*iter_x)+lidar_calib_matrix(0,1)*(*iter_y)+lidar_calib_matrix(0,2)*(*iter_z);
                lidar_points.at(lidar_scan_size).y = lidar_calib_matrix(1,0)*(*iter_x)+lidar_calib_matrix(1,1)*(*iter_y)+lidar_calib_matrix(1,2)*(*iter_z);
                lidar_points.at(lidar_scan_size).z = lidar_calib_matrix(2,0)*(*iter_x)+lidar_calib_matrix(2,1)*(*iter_y)+lidar_calib_matrix(2,2)*(*iter_z);
                lidar_points.at(lidar_scan_size).intensity = *iter_int;
                lidar_points.at(lidar_scan_size).ring = static_cast<uint32_t>(*iter_ring);

                lidar_scan_size++;
            }
        }

        //lidar_scan_size = lidar_points.size();

        Point_XYZIR lidar_time_stamp_info;
        std::memcpy(lidar_time_stamp_info.padding+1,&last_lidar_scan_sample_timestamp.sec,sizeof(uint32_t));
        //lidar_time_stamp_info.padding[1] = reinterpret_cast<uint32_t>(last_lidar_scan_sample_timestamp.sec);
        lidar_time_stamp_info.padding[2] = last_lidar_scan_sample_timestamp.nanosec;

        //lidar_points.push_back(lidar_time_stamp_info);
        lidar_points.at(lidar_scan_size) = lidar_time_stamp_info;

        //lidar_msg_read_first_time = true;
        fifo_lidar_msg_send_size = lidar_scan_size*sizeof(Point_XYZIR) + sizeof(Point_XYZIR);
        //LidarRecieveDataMutex.unlock();

        if(fifo_lidar_msg_send_size > fifo_lidar_msg_max_size){

            now = boost::posix_time::microsec_clock::universal_time();
            abs_time = now + boost::posix_time::milliseconds(500);

            if(!lidar_ctrl_mtx->timed_lock(abs_time))
            {
                lidar_ctrl_mtx->unlock();
                lidar_ctrl_mtx->lock();
            }

            lidar_resize_notification->resized = true;

            RecreateLidarBroadcastFIFO();

            lidar_ctrl_mtx->unlock();

            fifo_lidar_msg_max_size = fifo_lidar_msg_send_size;
        }

        if(lidar_send_queue->get_max_msg() > lidar_send_queue->get_num_msg()){
            now = boost::posix_time::microsec_clock::universal_time();
            abs_time = now + boost::posix_time::milliseconds(20);
            lidar_send_queue->timed_send(lidar_points.data(), fifo_lidar_msg_send_size, /*priority*/ 0, abs_time);
            //mq_result->try_send(lidar_inliers.data(), inliers_size*sizeof(float)+sizeof(detected_board), /*priority*/ 0);

        }

    }


    void open_bbox_shared_mem(){

        bbox_ctrl_mtx = std::make_unique<boost::interprocess::named_mutex>(boost::interprocess::open_or_create, "bbox_ctrl_mtx");
        boost::posix_time::ptime now = boost::posix_time::microsec_clock::universal_time();
        boost::posix_time::ptime abs_time = now + boost::posix_time::milliseconds(500);
        if(!(bbox_ctrl_mtx->timed_lock(abs_time)))bbox_ctrl_mtx->unlock();

        //RCLCPP_INFO(this->get_logger(), "lidar initted1\n");

        while(rclcpp::ok()){

          boost::this_thread::sleep(boost::posix_time::milliseconds(500));
          //RCLCPP_INFO(this->get_logger(), "lidar initted2\n");
          try{
              bbox_segment_ptr = std::make_unique<boost::interprocess::managed_shared_memory>(boost::interprocess::open_only, "bbox_shared_mem");
              //lidar_resize_notification = segment.construct<SharedControl>("shared_notification")();

              // Look for "Ctrl"
              auto response = bbox_segment_ptr->find<LidarBoundsCfg>("bbox_values");
              if (response.first != nullptr) {
                  RCLCPP_INFO(this->get_logger(), "Found bounding box existing SharedControl\n");
                  lidar_bbox_values_pointer = response.first; // attach to existing
                  break;
              } else {
                  continue;
              }

          }
          catch(boost::interprocess::interprocess_exception& ex)
          { 
              RCLCPP_ERROR(this->get_logger(), "Error: %s\n", ex.what());
              continue;
          }

        }

    }

void InitFIFOS(){
    //fifo_lidar_msg_send_size = LidarObject->points.size()*sizeof(Point_XYZIR);
    //fifo_lidar_msg_send_size = lidar_scan_size*sizeof(Point_XYZIR) + sizeof(Point_XYZIR);


    segment_ptr = std::make_unique<boost::interprocess::managed_shared_memory>(boost::interprocess::open_or_create, "calib_shared_mem", 512);


    // Look for "Ctrl"
    auto response_lidar = segment_ptr->find<SharedControl>("shared_notification");

    if (response_lidar.first) {
        RCLCPP_INFO(this->get_logger(), "Found existing SharedControl\n");
        lidar_resize_notification = response_lidar.first; // attach to existing
    } else {
        RCLCPP_INFO(this->get_logger(), "Not found, creating\n");
        lidar_resize_notification = segment_ptr->construct<SharedControl>("shared_notification")();
        lidar_resize_notification->resized = true;
    }
    
    lidar_ctrl_mtx = std::make_unique<boost::interprocess::named_mutex>(boost::interprocess::open_or_create, "calib_lidar_ctrl_mtx");

    boost::posix_time::ptime now = boost::posix_time::microsec_clock::universal_time();
    boost::posix_time::ptime abs_time = now + boost::posix_time::milliseconds(500);

    if(!lidar_ctrl_mtx->timed_lock(abs_time))
    {
        lidar_ctrl_mtx->unlock();
        lidar_ctrl_mtx->lock();
    }

    lidar_resize_notification->resized = true;

    boost::interprocess::message_queue::remove("calib_lidar_pts_queue");
    lidar_send_queue = std::make_unique<boost::interprocess::message_queue>(
                                    boost::interprocess::create_only,
                                    "calib_lidar_pts_queue",
                                    /*max msgs*/  5,
                                    /*max size*/  fifo_lidar_msg_max_size);

    lidar_ctrl_mtx->unlock();


    //camera_message_size = image_pixels_number*3U;//*sizeof(float)
                                       

    auto response_camera = segment_ptr->find<SharedControl>("shared_camera_notification");

    if (response_camera.first) {
        RCLCPP_INFO(this->get_logger(), "Found existing SharedCameraControl\n");
        camera_resize_notification = response_camera.first; // attach to existing
    } else {
        RCLCPP_INFO(this->get_logger(), "Not found, creating SharedCameraControl\n");
        camera_resize_notification = segment_ptr->construct<SharedControl>("shared_camera_notification")();
        camera_resize_notification->resized = true;
    }
    
    camera_ctrl_mtx = std::make_unique<boost::interprocess::named_mutex>(boost::interprocess::open_or_create, "calib_camera_ctrl_mtx");

    now = boost::posix_time::microsec_clock::universal_time();
    abs_time = now + boost::posix_time::milliseconds(1000);

    if(!camera_ctrl_mtx->timed_lock(abs_time))
    {
        camera_ctrl_mtx->unlock();
        camera_ctrl_mtx->lock();
    }

    camera_resize_notification->resized = true;

    boost::interprocess::message_queue::remove("calib_camera_queue");
    camera_send_queue = std::make_unique<boost::interprocess::message_queue>(
                                    boost::interprocess::create_only,
                                    "calib_camera_queue",
                                    /*max msgs*/  5,
                                    /*max size*/  camera_max_message_size); 

    camera_ctrl_mtx->unlock();
}

    void RecreateLidarBroadcastFIFO(){


        fifo_lidar_msg_send_size = lidar_scan_size*sizeof(Point_XYZIR) + sizeof(Point_XYZIR);

        boost::interprocess::message_queue::remove("calib_lidar_pts_queue");
        lidar_send_queue.reset();
        lidar_send_queue = std::make_unique<boost::interprocess::message_queue>(
                                        boost::interprocess::create_only,
                                        "calib_lidar_pts_queue",
                                        /*max msgs*/  5,
                                        /*max size*/  fifo_lidar_msg_send_size);

    }


    void RecreateCameraFIFO(){

        boost::interprocess::message_queue::remove("calib_camera_queue");
        camera_send_queue.reset();
        camera_send_queue = std::make_unique<boost::interprocess::message_queue>(
                                        boost::interprocess::create_only,
                                        "calib_camera_queue",
                                        /*max msgs*/  5,
                                        /*max size*/  camera_message_size); 

    }

    //std::thread cam_processing_thread;
    //std::thread lidar_cloud_processing_thread;
    
    //std::atomic<size_t> print_call_number;

    //std::atomic<bool> is_cam_thread_running;
    //std::atomic<bool> is_lidar_thread_running;

    //std::atomic<bool> camera_msg_read_first_time = false;
    //std::atomic<bool> lidar_msg_read_first_time = false;

    std::string cam_topic_name;
    std::string lidar_topic_name;

    rclcpp::CallbackGroup::SharedPtr camera_cb_group;
    rclcpp::CallbackGroup::SharedPtr lidar_cb_group;
    
    //rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cam_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_cloud_subscriber;

    // Open the existing message queue lidar
    std::unique_ptr<boost::interprocess::message_queue> lidar_send_queue;
    uint32_t lidar_scan_size;
    uint32_t fifo_lidar_msg_send_size;
    builtin_interfaces::msg::Time last_lidar_scan_sample_timestamp;

    // Open the existing message queue camera
    std::unique_ptr<boost::interprocess::message_queue> camera_send_queue;
    uint32_t image_pixels_number;
    uint32_t camera_message_size;
    uint32_t camera_max_message_size;
    uint32_t last_camera_message_size;
    builtin_interfaces::msg::Time last_camera_sample_timestamp;

    uint32_t image_rows_number;
    uint32_t image_cols_number;
    
    std::unique_ptr<boost::interprocess::managed_shared_memory> segment_ptr;
    std::unique_ptr<boost::interprocess::managed_shared_memory> bbox_segment_ptr;
    std::unique_ptr<boost::interprocess::named_mutex> bbox_ctrl_mtx;
    std::unique_ptr<boost::interprocess::named_mutex> lidar_ctrl_mtx;
    std::unique_ptr<boost::interprocess::named_mutex> camera_ctrl_mtx;
    SharedControl* lidar_resize_notification = nullptr;
    LidarBoundsCfg* lidar_bbox_values_pointer = nullptr;
    SharedControl* camera_resize_notification = nullptr;


    //boost::mutex LidarRecieveDataMutex;
    uint32_t fifo_lidar_msg_max_size;

    LidarBoundsCfg lidar_bounds;

    std::vector<Point_XYZIR> lidar_points;
    std::vector<uint8_t> Camera_image_pixels;
    //std::vector<float> lidar_inliers;

    Eigen::Matrix3f lidar_calib_matrix;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  //std::shared_ptr<CalibPublisher> pub_node = std::make_shared<CalibPublisher>();

  auto node = std::make_shared<CalibPublisher>();

  // Use MultiThreadedExecutor so camera + lidar callbacks can run in parallel
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  executor.spin();
  rclcpp::shutdown();
  return 0;
}
