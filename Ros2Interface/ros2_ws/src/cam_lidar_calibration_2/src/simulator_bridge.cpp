#include <simulator_bridge.hpp>

//using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class SimulatorBridgeSubscriber : public rclcpp::Node
{
  public:
    SimulatorBridgeSubscriber()
    : Node("simulator_bridge_subscriber"), print_call_number(0), is_cam_thread_running(true), is_lidar_thread_running(true)
    {
      
      cam_topic_name = this->declare_parameter<std::string>("camera/simulation_camera_topic", "/cam/image_color");
      lidar_topic_name = this->declare_parameter<std::string>("lidar/simulation_lidar_topic", "/lidarsim/points");
      
      image_rows_number = this->declare_parameter<int>("camera/simulated_image_size/height", 1000);
      image_cols_number = this->declare_parameter<int>("camera/simulated_image_size/width", 1400);

      cam_publisher = this->create_publisher<sensor_msgs::msg::Image>(cam_topic_name, 5);
      lidar_cloud_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(lidar_topic_name, 5);
      //timer_ = this->create_wall_timer(
      //std::chrono::milliseconds(500), std::bind(&SimulatorBridgeSubscriber::timer_callback, this));
      cam_processing_thread = std::thread(&SimulatorBridgeSubscriber::cam_processing, this);
      lidar_cloud_processing_thread = std::thread(&SimulatorBridgeSubscriber::lidar_processing, this);


    // Initialize the transform broadcaster
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    tf_broadcaster_timer = this->create_wall_timer(
    std::chrono::milliseconds(50), std::bind(&SimulatorBridgeSubscriber::tf_broadcaster_callback, this));

    }

    ~SimulatorBridgeSubscriber()
    {
      is_cam_thread_running = false;
      is_lidar_thread_running = false;

      if(cam_processing_thread.joinable()) cam_processing_thread.join();
      if(lidar_cloud_processing_thread.joinable()) lidar_cloud_processing_thread.join();
    }

  private:

      void tf_broadcaster_callback()
      {
        geometry_msgs::msg::TransformStamped transform;
      
        // Read message content and assign it to
        // corresponding tf variables
        transform.header.stamp = this->get_clock()->now();
        transform.header.frame_id = "base_link";          // parent
        transform.child_frame_id = "simulated_lidar_frame";          // child
      
        // Turtle only exists in 2D, thus we get x and y translation
        // coordinates from the message and set the z coordinate to 0
        transform.transform.translation.x = 0.0;
        transform.transform.translation.y = 0.0;
        transform.transform.translation.z = 0.0;
      
        // For the same reason, turtle can only rotate around one axis
        // and this why we set rotation in x and y to 0 and obtain
        // rotation in z axis from the message
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, 0.0);
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();
      
        // Send the transformation
        tf_broadcaster->sendTransform(transform);

        // Send camera transform

        geometry_msgs::msg::TransformStamped camera_transform;

        // Read message content and assign it to
        // corresponding tf variables
        camera_transform.header.stamp = this->get_clock()->now();
        camera_transform.header.frame_id = "base_link";          // parent
        camera_transform.child_frame_id = "simulated_camera_frame";          // child
      
        // Turtle only exists in 2D, thus we get x and y translation
        // coordinates from the message and set the z coordinate to 0
        camera_transform.transform.translation.x = -1.0;
        camera_transform.transform.translation.y = 0.0;
        camera_transform.transform.translation.z = 0.0;
      
        // For the same reason, turtle can only rotate around one axis
        // and this why we set rotation in x and y to 0 and obtain
        // rotation in z axis from the message
        camera_transform.transform.rotation.x = q.x();
        camera_transform.transform.rotation.y = q.y();
        camera_transform.transform.rotation.z = q.z();
        camera_transform.transform.rotation.w = q.w();

        tf_broadcaster->sendTransform(camera_transform);
      }


    void cam_processing()
    {

      //int rows = cfg_yaml.camera.image_size.height;
      //int cols = cfg_yaml.camera.image_size.width;

      camera_ctrl_mtx = std::make_unique<boost::interprocess::named_mutex>(boost::interprocess::open_or_create, "camera_ctrl_mtx");
      boost::posix_time::ptime now = boost::posix_time::microsec_clock::universal_time();
      boost::posix_time::ptime abs_time = now + boost::posix_time::milliseconds(1000);
      if(!(camera_ctrl_mtx->timed_lock(abs_time)))camera_ctrl_mtx->unlock();

      while(rclcpp::ok() && is_cam_thread_running){
      
          boost::this_thread::sleep(boost::posix_time::milliseconds(500));
      
          try{
              camera_segment_ptr = std::make_unique<boost::interprocess::managed_shared_memory>(boost::interprocess::open_only, "shared_mem");
              //camera_resize_notification = segment.construct<SharedControl>("shared_notification")();
          
              // Look for "Ctrl"
              auto response = camera_segment_ptr->find<SharedControl>("shared_camera_notification");
              if (response.first != nullptr) {
                  RCLCPP_INFO(this->get_logger(), "Found existing SharedControl for camera\n");
                  camera_resize_notification = response.first; // attach to existing
              } else {
                  continue;
              }
            
          }
          catch(boost::interprocess::interprocess_exception& ex)
          {
              RCLCPP_ERROR(this->get_logger(), "Error: %s\n", ex.what());
              continue;
          }
        
          try
          {   
              camera_recieve_queue = std::make_unique<boost::interprocess::message_queue>(boost::interprocess::open_only, "camera_queue");
              image_pixels_number = camera_recieve_queue->get_max_msg_size()/(3U); //(3U*sizeof(float))
          
              break;
          

          }
          catch (boost::interprocess::interprocess_exception& ex) 
          {
              RCLCPP_ERROR(this->get_logger(), "Error: %s\n", ex.what());
              continue;
          }
      }

      RCLCPP_INFO(this->get_logger(), "Number of pixels in the image:  %u pixels\n", image_pixels_number);

      Camera_image_pixels.resize(image_pixels_number*3U); //std::vector<float>

      RCLCPP_INFO(this->get_logger(), "Camera Resized state: %d\n", camera_resize_notification->resized);

      while(rclcpp::ok() && is_cam_thread_running){

        uint64_t recvd_size;
        uint32_t priority;

        try
        {   

            bool recieve_queue_state = false;

            while(!recieve_queue_state){

                now = boost::posix_time::microsec_clock::universal_time();
                abs_time = now + boost::posix_time::milliseconds(1000);

                //boost::interprocess::scoped_lock<boost::interprocess::named_mutex> recieve_lock(*camera_ctrl_mtx.get(), boost::interprocess::defer_lock);

                if(!camera_ctrl_mtx->timed_lock(abs_time))
                {
                    camera_ctrl_mtx->unlock();
                    camera_ctrl_mtx->lock();
                }

                if(!camera_resize_notification->resized){
                    now = boost::posix_time::microsec_clock::universal_time();
                    abs_time = now + boost::posix_time::milliseconds(500);
                    recieve_queue_state = camera_recieve_queue->timed_receive(Camera_image_pixels.data(), image_pixels_number*3U, recvd_size, priority, abs_time); //image_pixels_number*3U*sizeof(float)
                    camera_ctrl_mtx->unlock();
                }else {

                    camera_recieve_queue.reset();

                    camera_recieve_queue = std::make_unique<boost::interprocess::message_queue>(boost::interprocess::open_only, "camera_queue");
                    image_pixels_number = camera_recieve_queue->get_max_msg_size()/(3U); //(3U*sizeof(float))

                    Camera_image_pixels.resize(image_pixels_number*3U);

                    camera_resize_notification->resized = false;

                    camera_ctrl_mtx->unlock();
                    continue;
                }

                //std::cerr << "Pass: " << std::endl;
            }
        }
        catch (boost::interprocess::interprocess_exception& ex) 
        {
            RCLCPP_ERROR(this->get_logger(), "Error: %s\n", ex.what());

            camera_recieve_queue.reset();
            now = boost::posix_time::microsec_clock::universal_time();
            abs_time = now + boost::posix_time::milliseconds(1000);

            //boost::interprocess::scoped_lock<boost::interprocess::named_mutex> recieve_lock(*camera_ctrl_mtx.get(), boost::interprocess::defer_lock);

            if(!camera_ctrl_mtx->timed_lock(abs_time))
            {
                camera_ctrl_mtx->unlock();
                camera_ctrl_mtx->lock();
            }

            camera_recieve_queue = std::make_unique<boost::interprocess::message_queue>(boost::interprocess::open_only, "camera_queue");
            image_pixels_number = camera_recieve_queue->get_max_msg_size()/(3U); //(3U*sizeof(float))
            Camera_image_pixels.resize(image_pixels_number*3U);
            camera_ctrl_mtx->unlock();

            continue;
        }


        if(image_pixels_number*3U != (uint32_t)(image_rows_number*image_cols_number*3)){
            RCLCPP_ERROR(this->get_logger(), "Recieved image and expected image have different sizes\n");
            continue;
        }

        auto cam_msg = sensor_msgs::msg::Image();
        cam_msg.header.stamp = this->get_clock()->now();
        cam_msg.header.frame_id = "simulated_camera_frame";

        cam_msg.height = image_rows_number;
        cam_msg.width  = image_cols_number;
        cam_msg.encoding = "rgb8";
        cam_msg.is_bigendian = false;  // adjust if your data is big endian
        cam_msg.step = image_cols_number * 3U;  // number of bytes per row

        // Copy the data
        cam_msg.data = Camera_image_pixels;

        cam_publisher->publish(cam_msg);

      }

    }

    void lidar_processing()
    {

      lidar_ctrl_mtx = std::make_unique<boost::interprocess::named_mutex>(boost::interprocess::open_or_create, "lidar_ctrl_mtx");
      boost::posix_time::ptime now = boost::posix_time::microsec_clock::universal_time();
      boost::posix_time::ptime abs_time = now + boost::posix_time::milliseconds(500);
      if(!(lidar_ctrl_mtx->timed_lock(abs_time)))lidar_ctrl_mtx->unlock();

      //RCLCPP_INFO(this->get_logger(), "lidar initted1\n");

      while(rclcpp::ok() && is_lidar_thread_running){

        boost::this_thread::sleep(boost::posix_time::milliseconds(500));
        //RCLCPP_INFO(this->get_logger(), "lidar initted2\n");
        try{
            lidar_segment_ptr = std::make_unique<boost::interprocess::managed_shared_memory>(boost::interprocess::open_only, "shared_mem");
            //lidar_resize_notification = segment.construct<SharedControl>("shared_notification")();

            // Look for "Ctrl"
            auto response = lidar_segment_ptr->find<SharedControl>("shared_notification");
            if (response.first != nullptr) {
                RCLCPP_INFO(this->get_logger(), "Found existing SharedControl\n");
                lidar_resize_notification = response.first; // attach to existing
            } else {
                continue;
            }

        }
        catch(boost::interprocess::interprocess_exception& ex)
        { 
            RCLCPP_ERROR(this->get_logger(), "Error: %s\n", ex.what());
            continue;
        }

        try
        {   
            mq = std::make_unique<boost::interprocess::message_queue>(boost::interprocess::open_only, "pts_queue");
            lidar_scan_size = mq->get_max_msg_size()/sizeof(Point_XYZIR);

            break;
        }
        catch (boost::interprocess::interprocess_exception& ex) 
        {
            RCLCPP_ERROR(this->get_logger(), "Error: %s\n", ex.what());
            continue;
        }
      }

      RCLCPP_INFO(this->get_logger(), "Point cloud size: %u points\n", lidar_scan_size);

      lidar_points.resize(lidar_scan_size);

      RCLCPP_INFO(this->get_logger(), "Resized state: %d\n", lidar_resize_notification->resized);

      while (rclcpp::ok() && is_lidar_thread_running){

        uint64_t recvd_size;
        uint32_t priority;

        try
        {   

            bool recieve_queue_state = false;

            while(!recieve_queue_state){

                //RCLCPP_INFO(this->get_logger(), "Starting to open lidar queue\n");

                now = boost::posix_time::microsec_clock::universal_time();
                abs_time = now + boost::posix_time::milliseconds(500);

                //boost::interprocess::scoped_lock<boost::interprocess::named_mutex> recieve_lock(*lidar_ctrl_mtx.get(), boost::interprocess::defer_lock);

                if(!lidar_ctrl_mtx->timed_lock(abs_time))
                {
                    lidar_ctrl_mtx->unlock();
                    lidar_ctrl_mtx->lock();
                }


                //RCLCPP_INFO(this->get_logger(), "Lidar message resize mutex locked\n");

                if(!lidar_resize_notification->resized){
                    //RCLCPP_INFO(this->get_logger(), "Lidar mensage size not changed\n");
                    boost::posix_time::ptime now = boost::posix_time::microsec_clock::universal_time();
                    boost::posix_time::ptime abs_time = now + boost::posix_time::milliseconds(500);
                    recieve_queue_state = mq->timed_receive(lidar_points.data(), lidar_scan_size*sizeof(Point_XYZIR), recvd_size, priority, abs_time);
                    lidar_ctrl_mtx->unlock();
                    //RCLCPP_INFO(this->get_logger(), "Lidar message resize mutex unlocked and message read\n");
                }else {

                    mq.reset();

                    mq = std::make_unique<boost::interprocess::message_queue>(boost::interprocess::open_only, "pts_queue");

                    lidar_scan_size = mq->get_max_msg_size()/sizeof(Point_XYZIR);
                    lidar_points.resize(lidar_scan_size);

                    lidar_resize_notification->resized = false;

                    lidar_ctrl_mtx->unlock();
                    continue;
                }

            }
        }
        catch (boost::interprocess::interprocess_exception& ex) 
        {
            RCLCPP_ERROR(this->get_logger(), "Error: %s\n", ex.what());

            mq.reset();
            now = boost::posix_time::microsec_clock::universal_time();
            abs_time = now + boost::posix_time::milliseconds(500);

            if(!lidar_ctrl_mtx->timed_lock(abs_time))
            {
                lidar_ctrl_mtx->unlock();
                lidar_ctrl_mtx->lock();
            }

            mq = std::make_unique<boost::interprocess::message_queue>(boost::interprocess::open_only, "pts_queue");
            lidar_scan_size = mq->get_max_msg_size()/sizeof(Point_XYZIR);
            lidar_points.resize(lidar_scan_size);
            lidar_ctrl_mtx->unlock();

            continue;
        }

        //////// converting simulated lidar cloud to ros2 format ////////

        sensor_msgs::msg::PointCloud2 lidar_point_cloud_msg;

        lidar_point_cloud_msg.header.stamp = this->get_clock()->now();
        lidar_point_cloud_msg.header.frame_id = "simulated_lidar_frame"; // adjust frame id as needed

        lidar_point_cloud_msg.height = 1;
        lidar_point_cloud_msg.width = lidar_scan_size;
        lidar_point_cloud_msg.is_bigendian = false;
        lidar_point_cloud_msg.is_dense = true;

        lidar_point_cloud_msg.point_step = 32;//3*sizeof(float) + sizeof(float)+ sizeof(uint16_t);
        lidar_point_cloud_msg.row_step = lidar_point_cloud_msg.point_step*lidar_scan_size;

        // Define fields manually
        lidar_point_cloud_msg.fields.resize(5);
        lidar_point_cloud_msg.fields[0].name = "x";
        lidar_point_cloud_msg.fields[0].offset = 0;
        lidar_point_cloud_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
        lidar_point_cloud_msg.fields[0].count = 1;

        lidar_point_cloud_msg.fields[1].name = "y";
        lidar_point_cloud_msg.fields[1].offset = 4;
        lidar_point_cloud_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
        lidar_point_cloud_msg.fields[1].count = 1;

        lidar_point_cloud_msg.fields[2].name = "z";
        lidar_point_cloud_msg.fields[2].offset = 8;
        lidar_point_cloud_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
        lidar_point_cloud_msg.fields[2].count = 1;

        lidar_point_cloud_msg.fields[3].name = "intensity";
        lidar_point_cloud_msg.fields[3].offset = 16;
        lidar_point_cloud_msg.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
        lidar_point_cloud_msg.fields[3].count = 1;

        lidar_point_cloud_msg.fields[4].name = "ring";
        lidar_point_cloud_msg.fields[4].offset = 20;
        lidar_point_cloud_msg.fields[4].datatype = sensor_msgs::msg::PointField::UINT16;
        lidar_point_cloud_msg.fields[4].count = 1;

        lidar_point_cloud_msg.data.resize(lidar_point_cloud_msg.row_step * lidar_point_cloud_msg.height);

        // Fill data buffer
        uint8_t *ptr = lidar_point_cloud_msg.data.data();
        //for (const auto &p : lidar_points)
        for (auto &p : lidar_points)
        { //converting from open GL to ROS
          p.z = -p.z;
          p.x = -p.x;
          uint16_t temp_ring = static_cast<uint16_t>(p.ring);

          memcpy(ptr + 0,  &(p.z), sizeof(float)); //front
          memcpy(ptr + 4,  &(p.x), sizeof(float)); //-right
          memcpy(ptr + 8,  &p.y, sizeof(float)); //up
          memcpy(ptr + 16, &p.intensity, sizeof(float));
          memcpy(ptr + 20, &temp_ring, sizeof(uint16_t));
          ptr += lidar_point_cloud_msg.point_step;
        }

        /////////////////////////////////////////////////////////////////

        // publishing lidar menssage
        lidar_cloud_publisher->publish(lidar_point_cloud_msg);

      } 



    }

    void print_task_test(const std::string &task_name)
    {
      auto message = std_msgs::msg::String();
      message.data = "Hello from " + task_name + " task! : " + std::to_string(++print_call_number);
      //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      //if(task_name == "camera") cam_publisher->publish(message);
      //else if(task_name == "lidar") lidar_cloud_publisher->publish(message);
    }

    std::thread cam_processing_thread;
    std::thread lidar_cloud_processing_thread;
    
    std::atomic<size_t> print_call_number;

    std::atomic<bool> is_cam_thread_running;
    std::atomic<bool> is_lidar_thread_running;

    std::string cam_topic_name;
    std::string lidar_topic_name;

    //rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr cam_publisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_cloud_publisher;

    // Open the existing message queue lidar
    std::unique_ptr<boost::interprocess::message_queue> mq;
    uint32_t lidar_scan_size;

    // Open the existing message queue camera
    std::unique_ptr<boost::interprocess::message_queue> camera_recieve_queue;
    uint32_t image_pixels_number;

    int image_rows_number;
    int image_cols_number;
    
    std::unique_ptr<boost::interprocess::managed_shared_memory> lidar_segment_ptr;
    std::unique_ptr<boost::interprocess::managed_shared_memory> camera_segment_ptr;
    std::unique_ptr<boost::interprocess::named_mutex> lidar_ctrl_mtx;
    std::unique_ptr<boost::interprocess::named_mutex> camera_ctrl_mtx;
    SharedControl* lidar_resize_notification = nullptr;
    SharedControl* camera_resize_notification = nullptr;

    std::vector<Point_XYZIR> lidar_points;
    std::vector<uint8_t> Camera_image_pixels;
    //std::vector<float> lidar_inliers;

    rclcpp::TimerBase::SharedPtr tf_broadcaster_timer;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  //std::shared_ptr<SimulatorBridgeSubscriber> pub_node = std::make_shared<SimulatorBridgeSubscriber>();
  rclcpp::spin(std::make_shared<SimulatorBridgeSubscriber>());
  rclcpp::shutdown();
  return 0;
}
