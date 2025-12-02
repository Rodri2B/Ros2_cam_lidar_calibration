#include <calib_yaml_config.hpp>
#include <iostream>

#include <vector>
#include <memory>
#include <algorithm>

#include <boost/interprocess/ipc/message_queue.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>

#include <thread>

#include <calib_yaml_config.hpp>


#include <chrono>

#include <csignal>

struct Timestamp {
    int32_t sec;     // Seconds
    uint32_t nanosec; // Nanoseconds
};


struct Point_XYZIR{
    float x;
    float y;
    float z;
    float intensity;
    uint32_t ring;
    // Padding to match std430 alignment
    uint32_t padding[3];
};

struct SharedControl {
    bool resized;  
};

class Bridge {

public:

    Bridge():run_flag(true){
        LoadYamlParams();
        InitFIFOS();
        open_bbox_shared_mem();
        InitLidarCameraThreads();
    }

    ~Bridge(){
        JoinLidarCameraThreads();
    }

    
    void RecieveLidarDataAsync_func();
    void RecieveCameraDataAsync_func();
    void LoadYamlParams();
    void InitFIFOS();
    void RecreateCameraFIFO();
    void RecreateLidarBroadcastFIFO();

    void open_bbox_shared_mem();

    void InitLidarCameraThreads();
    void JoinLidarCameraThreads();

    LidarBoundsCfg lidar_bounds;

    uint32_t fifo_lidar_msg_send_size;
    uint32_t fifo_lidar_msg_send_max_size = 60001U*sizeof(Point_XYZIR);

    uint32_t camera_message_size;
    uint32_t camera_max_message_size;

    std::thread RecieveCameraDataAsync;
    std::thread RecieveLidarDataAsync;

    //std::unique_ptr<boost::interprocess::message_queue> send_queue;
    std::unique_ptr<boost::interprocess::message_queue> lidar_recieve_queue;

    //std::unique_ptr<boost::interprocess::message_queue> camera_send_queue;
    //std::unique_ptr<boost::interprocess::message_queue> camera_detected_board_queue;
    std::unique_ptr<boost::interprocess::message_queue> camera_recieve_queue;

    SharedControl* lidar_resize_notification = nullptr;
    std::unique_ptr<boost::interprocess::named_mutex> lidar_ctrl_mtx;
    std::unique_ptr<boost::interprocess::managed_shared_memory> lidar_segment_ptr;

    SharedControl* camera_resize_notification = nullptr;
    std::unique_ptr<boost::interprocess::named_mutex> camera_ctrl_mtx;
    std::unique_ptr<boost::interprocess::managed_shared_memory> camera_segment_ptr;

    LidarYamlConfig lidar_bo_settings;
    LidarBoundsCfg* lidar_bbox_values_pointer = nullptr;
    std::unique_ptr<boost::interprocess::managed_shared_memory> bbox_segment_ptr;
    std::unique_ptr<boost::interprocess::named_mutex> bbox_ctrl_mtx;

    TopicsInfo ipc_topics_names;
    SimTopicsInfo sim_ipc_topics_names;

    std::unique_ptr<boost::interprocess::managed_shared_memory> calib_segment_ptr;

    SharedControl* calib_lidar_resize_notification = nullptr;
    std::unique_ptr<boost::interprocess::named_mutex> calib_lidar_ctrl_mtx;

    SharedControl* calib_camera_resize_notification = nullptr;
    std::unique_ptr<boost::interprocess::named_mutex> calib_camera_ctrl_mtx;

    std::unique_ptr<boost::interprocess::message_queue> camera_send_queue;
    std::unique_ptr<boost::interprocess::message_queue> lidar_send_queue;


private:

    std::atomic<bool> run_flag;

};

bool stop_requested = false;

void signalHandler(int signum) {
    stop_requested = true;
}

int main(){

    Bridge bridge_class;

    // Set up the signal handler
    signal(SIGINT, signalHandler);  // Catch Ctrl+C (SIGINT)

    while (!stop_requested) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    //bridge_class().JoinLidarCameraThreads();

    return 0;
}

void Bridge::RecieveLidarDataAsync_func(){

    std::vector<Point_XYZIR> lidar_points;
    uint32_t lidar_point_cloud_size;

    lidar_ctrl_mtx = std::make_unique<boost::interprocess::named_mutex>(boost::interprocess::open_or_create, sim_ipc_topics_names.lidar_ctrl_mtx_name.c_str());
    boost::posix_time::ptime now = boost::posix_time::microsec_clock::universal_time();
    boost::posix_time::ptime abs_time = now + boost::posix_time::milliseconds(500);
    if(!(lidar_ctrl_mtx->timed_lock(abs_time)))lidar_ctrl_mtx->unlock();

    while(run_flag){

        boost::this_thread::sleep(boost::posix_time::milliseconds(500));

        try{
            lidar_segment_ptr = std::make_unique<boost::interprocess::managed_shared_memory>(boost::interprocess::open_only, sim_ipc_topics_names.shared_mem_name.c_str());

            // Look for "Ctrl"
            auto response = lidar_segment_ptr->find<SharedControl>("shared_notification");
            if (response.first != nullptr) {
                std::cout << "Found existing Lidar SharedControl"<<std::endl;
                lidar_resize_notification = response.first; // attach to existing
            } else {
                continue;
            }

        }
        catch(boost::interprocess::interprocess_exception& ex)
        {
            std::cerr << "Error: " << ex.what() << std::endl;
            continue;
        }

        try
        {   
            lidar_recieve_queue = std::make_unique<boost::interprocess::message_queue>(boost::interprocess::open_only, sim_ipc_topics_names.lidar_recieve_queue_name.c_str());
            lidar_point_cloud_size = lidar_recieve_queue->get_max_msg_size()/sizeof(Point_XYZIR);

            //mq_result = std::make_unique<boost::interprocess::message_queue>(boost::interprocess::open_only, "pts_queue_result");

            break;


        }
        catch (boost::interprocess::interprocess_exception& ex) 
        {
            std::cerr << "Error: " << ex.what() << std::endl;
            continue;
        }
    }


    lidar_points.resize(lidar_point_cloud_size);


    while(run_flag){

        uint64_t recvd_size;
        uint32_t priority;

        try
        {   

            bool recieve_queue_state = false;

            while(!recieve_queue_state){

                //std::cout << "Pass5"<<std::endl;

                now = boost::posix_time::microsec_clock::universal_time();
                abs_time = now + boost::posix_time::milliseconds(500);


                if(!lidar_ctrl_mtx->timed_lock(abs_time))
                {
                    lidar_ctrl_mtx->unlock();
                    lidar_ctrl_mtx->lock();
                }


                //std::cout << "Pass5.1"<<std::endl;
                if(!lidar_resize_notification->resized){
                    //std::cout << "Pass6"<<std::endl;
                    boost::posix_time::ptime now = boost::posix_time::microsec_clock::universal_time();
                    boost::posix_time::ptime abs_time = now + boost::posix_time::milliseconds(500);
                    recieve_queue_state = lidar_recieve_queue->timed_receive(lidar_points.data(), lidar_point_cloud_size*sizeof(Point_XYZIR), recvd_size, priority, abs_time);
                    lidar_ctrl_mtx->unlock();
                    //std::cout << "Pass7"<<std::endl;
                }else {

                    lidar_recieve_queue.reset();
                    //mq_result.reset();

                    lidar_recieve_queue = std::make_unique<boost::interprocess::message_queue>(boost::interprocess::open_only, sim_ipc_topics_names.lidar_recieve_queue_name.c_str());

                    //mq_result = std::make_unique<boost::interprocess::message_queue>(boost::interprocess::open_only, "pts_queue_result");

                    lidar_point_cloud_size = lidar_recieve_queue->get_max_msg_size()/sizeof(Point_XYZIR);
                    lidar_points.resize(lidar_point_cloud_size);

                    lidar_resize_notification->resized = false;

                    lidar_ctrl_mtx->unlock();
                    continue;
                }

            }
        }
        catch (boost::interprocess::interprocess_exception& ex) 
        {
            std::cerr << "Error: " << ex.what() << std::endl;

            lidar_recieve_queue.reset();
            //mq_result.reset();
            now = boost::posix_time::microsec_clock::universal_time();
            abs_time = now + boost::posix_time::milliseconds(500);

            if(!lidar_ctrl_mtx->timed_lock(abs_time))
            {
                lidar_ctrl_mtx->unlock();
                lidar_ctrl_mtx->lock();
            }

            lidar_recieve_queue = std::make_unique<boost::interprocess::message_queue>(boost::interprocess::open_only, sim_ipc_topics_names.lidar_recieve_queue_name.c_str());
            lidar_point_cloud_size = lidar_recieve_queue->get_max_msg_size()/sizeof(Point_XYZIR);
            lidar_points.resize(lidar_point_cloud_size);
            //mq_result = std::make_unique<boost::interprocess::message_queue>(boost::interprocess::open_only, "pts_queue_result");
            lidar_ctrl_mtx->unlock();

            continue;
        }





        //Point_XYZIR timestamp_sample = lidar_points.back();   // get last element
        //lidar_points.push_back(timestamp_sample);        


        //uint32_t lidar_points_size_vec = lidar_points.size()-1U;

        //LidarPointsVisualizeMutex.lock();
        ////the draw function must play with the copy

        //lidar_points_visualize.resize(lidar_points_size_vec);

        now = boost::posix_time::microsec_clock::universal_time();
        abs_time = now + boost::posix_time::milliseconds(500);

        if(!bbox_ctrl_mtx->timed_lock(abs_time))
        {
            bbox_ctrl_mtx->unlock();
            bbox_ctrl_mtx->lock();
        }

        lidar_bounds =  *lidar_bbox_values_pointer;
        
        bbox_ctrl_mtx->unlock();

        //LidarRecieveDataMutex.lock();

        uint32_t lidar_scan_size = 0;

        
        
    //for (size_t i = 0; i < cloud_msg->width * cloud_msg->height; ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_int,++iter_ring) {
    //    if(*iter_x >= lidar_bounds.x_min && *iter_x <= lidar_bounds.x_max &&
    //       *iter_y >= lidar_bounds.y_min && *iter_y <= lidar_bounds.y_max &&
    //       *iter_z >= lidar_bounds.z_min && *iter_z <= lidar_bounds.z_max){

    //        //Point_XYZIR p;
    //        //p.x = *iter_x;
    //        //p.y = *iter_y;
    //        //p.z = *iter_z;
    //        //p.intensity = *iter_int;
    //        //p.ring = static_cast<uint32_t>(*iter_ring);
    //        //lidar_points.push_back(p);

    //        lidar_points.at(lidar_scan_size).x = lidar_calib_matrix(0,0)*(*iter_x)+lidar_calib_matrix(0,1)*(*iter_y)+lidar_calib_matrix(0,2)*(*iter_z);
    //        lidar_points.at(lidar_scan_size).y = lidar_calib_matrix(1,0)*(*iter_x)+lidar_calib_matrix(1,1)*(*iter_y)+lidar_calib_matrix(1,2)*(*iter_z);
    //        lidar_points.at(lidar_scan_size).z = lidar_calib_matrix(2,0)*(*iter_x)+lidar_calib_matrix(2,1)*(*iter_y)+lidar_calib_matrix(2,2)*(*iter_z);
    //        lidar_points.at(lidar_scan_size).intensity = *iter_int;
    //        lidar_points.at(lidar_scan_size).ring = static_cast<uint32_t>(*iter_ring);

    //        lidar_scan_size++;
    //    }

    //    



    //}

        for (uint32_t i = 0; i < lidar_points.size(); i++){

            if(lidar_points.at(i).x >= lidar_bounds.x_min && lidar_points.at(i).x <= lidar_bounds.x_max &&
               lidar_points.at(i).y >= lidar_bounds.y_min && lidar_points.at(i).y <= lidar_bounds.y_max &&
               lidar_points.at(i).z >= lidar_bounds.z_min && lidar_points.at(i).z <= lidar_bounds.z_max)
            {   

                lidar_points.at(lidar_scan_size) = lidar_points.at(i);
                lidar_scan_size++;
            }


        }

        Point_XYZIR lidar_time_stamp_info;

        auto chrono_now = std::chrono::system_clock::now();
        // Convert to time_point<system_clock>
        auto chrono_duration = chrono_now.time_since_epoch();
        // Get total seconds
        auto chrono_seconds = std::chrono::duration_cast<std::chrono::seconds>(chrono_duration).count();
        // Get remaining nanoseconds
        auto chrono_nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(chrono_duration).count() - (chrono_seconds * 1'000'000'000);

        Timestamp timestamp;
        timestamp.sec = static_cast<int32_t>(chrono_seconds);
        timestamp.nanosec = static_cast<uint32_t>(chrono_nanoseconds);

        std::memcpy(lidar_time_stamp_info.padding+1,&timestamp.sec,sizeof(uint32_t));
        //lidar_time_stamp_info.padding[1] = reinterpret_cast<uint32_t>(last_lidar_scan_sample_timestamp.sec);
        lidar_time_stamp_info.padding[2] = timestamp.nanosec;

        //lidar_points.push_back(lidar_time_stamp_info);
        lidar_points.push_back(lidar_time_stamp_info);
        lidar_points.at(lidar_scan_size) = lidar_time_stamp_info;

        fifo_lidar_msg_send_size = lidar_scan_size*sizeof(Point_XYZIR) + sizeof(Point_XYZIR);

        if(fifo_lidar_msg_send_size > fifo_lidar_msg_send_max_size){

            now = boost::posix_time::microsec_clock::universal_time();
            abs_time = now + boost::posix_time::milliseconds(500);

            if(!calib_lidar_ctrl_mtx->timed_lock(abs_time))
            {
                calib_lidar_ctrl_mtx->unlock();
                calib_lidar_ctrl_mtx->lock();
            }

            RecreateLidarBroadcastFIFO();

            calib_lidar_resize_notification->resized = true;

            calib_lidar_ctrl_mtx->unlock();

            fifo_lidar_msg_send_max_size = fifo_lidar_msg_send_size;
        }

        if(lidar_send_queue->get_max_msg() > lidar_send_queue->get_num_msg()){
            now = boost::posix_time::microsec_clock::universal_time();
            abs_time = now + boost::posix_time::milliseconds(20);
            lidar_send_queue->timed_send(lidar_points.data(), fifo_lidar_msg_send_size, /*priority*/ 0, abs_time);
            //mq_result->try_send(lidar_inliers.data(), inliers_size*sizeof(float)+sizeof(detected_board), /*priority*/ 0);

        }

        lidar_points.pop_back();
    }
}



void Bridge::RecieveCameraDataAsync_func(){

    std::vector<uint8_t> Camera_cpu_buffer;
    uint32_t image_pixels_number;

    // Open the existing message queue

    camera_ctrl_mtx = std::make_unique<boost::interprocess::named_mutex>(boost::interprocess::open_or_create, sim_ipc_topics_names.camera_ctrl_mtx_name.c_str());
    boost::posix_time::ptime now = boost::posix_time::microsec_clock::universal_time();
    boost::posix_time::ptime abs_time = now + boost::posix_time::milliseconds(1000);
    if(!(camera_ctrl_mtx->timed_lock(abs_time)))camera_ctrl_mtx->unlock();



    while(run_flag){

        boost::this_thread::sleep(boost::posix_time::milliseconds(500));

        try{
            camera_segment_ptr = std::make_unique<boost::interprocess::managed_shared_memory>(boost::interprocess::open_only, sim_ipc_topics_names.shared_mem_name.c_str());
            //camera_resize_notification = segment.construct<SharedControl>("shared_notification")();

            // Look for "Ctrl"
            auto response = camera_segment_ptr->find<SharedControl>("shared_camera_notification");
            if (response.first != nullptr) {
                std::cout << "Found existing camera SharedControl"<<std::endl;
                camera_resize_notification = response.first; // attach to existing
            } else {
                continue;
            }

        }
        catch(boost::interprocess::interprocess_exception& ex)
        {
            std::cerr << "Error: " << ex.what() << std::endl;
            continue;
        }

        try
        {   
            camera_recieve_queue = std::make_unique<boost::interprocess::message_queue>(boost::interprocess::open_only, sim_ipc_topics_names.camera_recieve_queue_name.c_str());
            image_pixels_number = (camera_recieve_queue->get_max_msg_size())/(3U); //(3U*sizeof(float))

            //camera_detected_board_queue = std::make_unique<boost::interprocess::message_queue>(boost::interprocess::open_only, "camera_queue_board");

            break;


        }
        catch (boost::interprocess::interprocess_exception& ex) 
        {
            std::cerr << "Error: " << ex.what() << std::endl;
            continue;
        }
    }

    Camera_cpu_buffer.resize(image_pixels_number*3U+8U); 


    while(run_flag){

        uint64_t recvd_size;
        uint32_t priority;

        try
        {   

            bool recieve_queue_state = false;

            while(!recieve_queue_state){

                //std::cout << "Pass5"<<std::endl;

                now = boost::posix_time::microsec_clock::universal_time();
                abs_time = now + boost::posix_time::milliseconds(1000);

                //boost::interprocess::scoped_lock<boost::interprocess::named_mutex> recieve_lock(*camera_ctrl_mtx.get(), boost::interprocess::defer_lock);

                if(!camera_ctrl_mtx->timed_lock(abs_time))
                {
                    camera_ctrl_mtx->unlock();
                    camera_ctrl_mtx->lock();
                }


                std::cout << "Pass5.1"<<std::endl;
                if(!camera_resize_notification->resized){
                    std::cout << "Pass6"<<std::endl;
                    now = boost::posix_time::microsec_clock::universal_time();
                    abs_time = now + boost::posix_time::milliseconds(500);
                    recieve_queue_state = camera_recieve_queue->timed_receive(Camera_cpu_buffer.data(), image_pixels_number*3U, recvd_size, priority, abs_time); //image_pixels_number*3U*sizeof(float)
                    std::cout << "Pass7.0"<<std::endl;
                    camera_ctrl_mtx->unlock();
                    std::cout << "Pass7"<<std::endl;
                }else {

                    camera_recieve_queue.reset();
                    //camera_detected_board_queue.reset();

                    camera_recieve_queue = std::make_unique<boost::interprocess::message_queue>(boost::interprocess::open_only, sim_ipc_topics_names.camera_recieve_queue_name.c_str());
                    image_pixels_number = (camera_recieve_queue->get_max_msg_size())/(3U); //(3U*sizeof(float))

                    //camera_detected_board_queue = std::make_unique<boost::interprocess::message_queue>(boost::interprocess::open_only, "camera_queue_board");

                    Camera_cpu_buffer.resize(image_pixels_number*3U+8U);

                    camera_resize_notification->resized = false;

                    camera_ctrl_mtx->unlock();
                    continue;
                }

                //std::cerr << "Pass: " << std::endl;
            }
        }
        catch (boost::interprocess::interprocess_exception& ex) 
        {
            std::cerr << "Error: " << ex.what() << std::endl;

            camera_recieve_queue.reset();
            //camera_detected_board_queue.reset();
            now = boost::posix_time::microsec_clock::universal_time();
            abs_time = now + boost::posix_time::milliseconds(1000);

            if(!camera_ctrl_mtx->timed_lock(abs_time))
            {
                camera_ctrl_mtx->unlock();
                camera_ctrl_mtx->lock();
            }

            camera_recieve_queue = std::make_unique<boost::interprocess::message_queue>(boost::interprocess::open_only, sim_ipc_topics_names.camera_recieve_queue_name.c_str());
            image_pixels_number = (camera_recieve_queue->get_max_msg_size())/(3U); //(3U*sizeof(float))
            Camera_cpu_buffer.resize(image_pixels_number*3U+8U);
            //camera_detected_board_queue = std::make_unique<boost::interprocess::message_queue>(boost::interprocess::open_only, "camera_queue_board");
            camera_ctrl_mtx->unlock();

            continue;
        }

        //////////////////////////////////

        auto chrono_now = std::chrono::system_clock::now();
        // Convert to time_point<system_clock>
        auto chrono_duration = chrono_now.time_since_epoch();
        // Get total seconds
        auto chrono_seconds = std::chrono::duration_cast<std::chrono::seconds>(chrono_duration).count();
        // Get remaining nanoseconds
        auto chrono_nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(chrono_duration).count() - (chrono_seconds * 1'000'000'000);

        Timestamp timestamp;
        timestamp.sec = static_cast<int32_t>(chrono_seconds);
        timestamp.nanosec = static_cast<uint32_t>(chrono_nanoseconds);

        camera_message_size = image_pixels_number*3U+8U;

        std::memcpy(Camera_cpu_buffer.data()+(camera_message_size-8U),&timestamp.sec,4U);
        std::memcpy(Camera_cpu_buffer.data()+(camera_message_size-4U),&timestamp.nanosec,4U);

        if(camera_message_size > camera_max_message_size){

            now = boost::posix_time::microsec_clock::universal_time();
            abs_time = now + boost::posix_time::milliseconds(500);

            if(!calib_camera_ctrl_mtx->timed_lock(abs_time))
            {
                calib_camera_ctrl_mtx->unlock();
                calib_camera_ctrl_mtx->lock();
            }

            RecreateCameraFIFO();

            calib_camera_resize_notification->resized = true;

            //RCLCPP_INFO(this->get_logger(), "Fifo Cam Recreated\n");

            calib_camera_ctrl_mtx->unlock();

            camera_max_message_size = camera_message_size;
        }

        if(camera_send_queue->get_max_msg() > camera_send_queue->get_num_msg()){
            now = boost::posix_time::microsec_clock::universal_time();
            abs_time = now + boost::posix_time::milliseconds(20);
            camera_send_queue->timed_send(Camera_cpu_buffer.data(), camera_message_size, /*priority*/ 0, abs_time);
            
        }



    }
}


void Bridge::LoadYamlParams(){


    TopicsInfo topics_cfg_yaml;
    //Camera settings
    if (!loadTopicsConfig("../config/cfg.yaml", topics_cfg_yaml)) {
        std::cerr << "Failed to load topics config, loading default parameters\n";

        topics_cfg_yaml.shared_mem_name = "calib_shared_mem";
        topics_cfg_yaml.bbox_shared_mem_name = "bbox_shared_mem";

        topics_cfg_yaml.camera_ctrl_mtx_name = "calib_camera_ctrl_mtx";
        topics_cfg_yaml.lidar_ctrl_mtx_name = "calib_lidar_ctrl_mtx";
        topics_cfg_yaml.bbox_ctrl_mtx_name = "bbox_ctrl_mtx";

        topics_cfg_yaml.camera_recieve_queue_name = "calib_camera_queue";
        topics_cfg_yaml.lidar_recieve_queue_name = "calib_lidar_pts_queue";

    }

    ipc_topics_names = topics_cfg_yaml;

    SimTopicsInfo sim_topics_cfg_yaml;

    if (!loadSimTopicsConfig("../config/cfg.yaml", sim_topics_cfg_yaml)) {
        std::cerr << "Failed to load sim topics config, loading default parameters\n";

        sim_topics_cfg_yaml.shared_mem_name = "shared_mem";

        sim_topics_cfg_yaml.camera_ctrl_mtx_name = "camera_ctrl_mtx";
        sim_topics_cfg_yaml.lidar_ctrl_mtx_name = "lidar_ctrl_mtx";

        sim_topics_cfg_yaml.camera_recieve_queue_name = "camera_queue";
        sim_topics_cfg_yaml.lidar_recieve_queue_name = "pts_queue";

    }

    sim_ipc_topics_names = sim_topics_cfg_yaml;

    YAML::Node root;
    bool cam_size_loaded = true;
    try {
        root = YAML::LoadFile("../config/cfg.yaml");
    } catch (const YAML::BadFile &e) {
        std::cerr << "Could not open config file: " << e.what() << "\n";
        cam_size_loaded = false;
    }

    // camera
    auto cam = root["camera"];
    if (!cam) { std::cerr << "Missing camera section\n"; cam_size_loaded = false; }

    if(cam_size_loaded){
        auto imsz = cam["image_size"];
        int width = imsz["width"].as<int>();
        int height = imsz["height"].as<int>();
        camera_max_message_size = static_cast<uint32_t>(width)*static_cast<uint32_t>(height)*3U+8U;
    }
    else{
        std::cerr << "Failed to load image size config, loading default parameters\n";
        int width = 1400;
        int height = 1000;
        camera_max_message_size = static_cast<uint32_t>(width)*static_cast<uint32_t>(height)*3U+8U;
    }
 
}



void Bridge::InitFIFOS(){
    //fifo_lidar_msg_send_size = LidarObject->points.size()*sizeof(Point_XYZIR);
    //fifo_lidar_msg_send_size = lidar_scan_size*sizeof(Point_XYZIR) + sizeof(Point_XYZIR);


    calib_segment_ptr = std::make_unique<boost::interprocess::managed_shared_memory>(boost::interprocess::open_or_create, ipc_topics_names.shared_mem_name.c_str(), 512);


    // Look for "Ctrl"
    auto response_lidar = calib_segment_ptr->find<SharedControl>("shared_notification");

    if (response_lidar.first) {
        std::cout << "Found existing SharedControl\n";
        calib_lidar_resize_notification = response_lidar.first; // attach to existing
    } else {
        std::cout << "Not found, creating\n";
        calib_lidar_resize_notification = calib_segment_ptr->construct<SharedControl>("shared_notification")();
        calib_lidar_resize_notification->resized = true;
    }
    
    calib_lidar_ctrl_mtx = std::make_unique<boost::interprocess::named_mutex>(boost::interprocess::open_or_create, ipc_topics_names.lidar_ctrl_mtx_name.c_str());

    boost::posix_time::ptime now = boost::posix_time::microsec_clock::universal_time();
    boost::posix_time::ptime abs_time = now + boost::posix_time::milliseconds(500);

    if(!calib_lidar_ctrl_mtx->timed_lock(abs_time))
    {
        calib_lidar_ctrl_mtx->unlock();
        calib_lidar_ctrl_mtx->lock();
    }

    calib_lidar_resize_notification->resized = true;

    boost::interprocess::message_queue::remove(ipc_topics_names.lidar_recieve_queue_name.c_str());
    lidar_send_queue = std::make_unique<boost::interprocess::message_queue>(
                                    boost::interprocess::create_only,
                                    ipc_topics_names.lidar_recieve_queue_name.c_str(),
                                    /*max msgs*/  5,
                                    /*max size*/  fifo_lidar_msg_send_max_size);

    calib_lidar_ctrl_mtx->unlock();


    //camera_message_size = image_pixels_number*3U;//*sizeof(float)
                                       

    auto response_camera = calib_segment_ptr->find<SharedControl>("shared_camera_notification");

    if (response_camera.first) {
        std::cout << "Found existing SharedCameraControl\n";
        calib_camera_resize_notification = response_camera.first; // attach to existing
    } else {
        std::cout << "Not found, creating SharedCameraControl\n";
        calib_camera_resize_notification = calib_segment_ptr->construct<SharedControl>("shared_camera_notification")();
        calib_camera_resize_notification->resized = true;
    }
    
    calib_camera_ctrl_mtx = std::make_unique<boost::interprocess::named_mutex>(boost::interprocess::open_or_create, ipc_topics_names.camera_ctrl_mtx_name.c_str());

    now = boost::posix_time::microsec_clock::universal_time();
    abs_time = now + boost::posix_time::milliseconds(1000);

    if(!calib_camera_ctrl_mtx->timed_lock(abs_time))
    {
        calib_camera_ctrl_mtx->unlock();
        calib_camera_ctrl_mtx->lock();
    }

    calib_camera_resize_notification->resized = true;

    boost::interprocess::message_queue::remove(ipc_topics_names.camera_recieve_queue_name.c_str());
    camera_send_queue = std::make_unique<boost::interprocess::message_queue>(
                                    boost::interprocess::create_only,
                                    ipc_topics_names.camera_recieve_queue_name.c_str(),
                                    /*max msgs*/  5,
                                    /*max size*/  camera_max_message_size); 

    calib_camera_ctrl_mtx->unlock();
}


void Bridge::RecreateLidarBroadcastFIFO(){

    boost::interprocess::message_queue::remove(ipc_topics_names.lidar_recieve_queue_name.c_str());
    lidar_send_queue.reset();
    lidar_send_queue = std::make_unique<boost::interprocess::message_queue>(
                                    boost::interprocess::create_only,
                                    ipc_topics_names.lidar_recieve_queue_name.c_str(),
                                    /*max msgs*/  5,
                                    /*max size*/  fifo_lidar_msg_send_size);

}


void Bridge::RecreateCameraFIFO(){

    boost::interprocess::message_queue::remove(ipc_topics_names.camera_recieve_queue_name.c_str());
    camera_send_queue.reset();
    camera_send_queue = std::make_unique<boost::interprocess::message_queue>(
                                    boost::interprocess::create_only,
                                    ipc_topics_names.camera_recieve_queue_name.c_str(),
                                    /*max msgs*/  5,
                                    /*max size*/  camera_message_size); 

}

void Bridge::open_bbox_shared_mem(){

    bbox_ctrl_mtx = std::make_unique<boost::interprocess::named_mutex>(boost::interprocess::open_or_create, ipc_topics_names.bbox_ctrl_mtx_name.c_str());
    boost::posix_time::ptime now = boost::posix_time::microsec_clock::universal_time();
    boost::posix_time::ptime abs_time = now + boost::posix_time::milliseconds(500);
    if(!(bbox_ctrl_mtx->timed_lock(abs_time)))bbox_ctrl_mtx->unlock();

    //RCLCPP_INFO(this->get_logger(), "lidar initted1\n");

    while(run_flag){

      boost::this_thread::sleep(boost::posix_time::milliseconds(500));
      //RCLCPP_INFO(this->get_logger(), "lidar initted2\n");
      try{
          bbox_segment_ptr = std::make_unique<boost::interprocess::managed_shared_memory>(boost::interprocess::open_only, ipc_topics_names.bbox_shared_mem_name.c_str());
          //lidar_resize_notification = segment.construct<SharedControl>("shared_notification")();

          // Look for "Ctrl"
          auto response = bbox_segment_ptr->find<LidarBoundsCfg>("bbox_values");
          if (response.first != nullptr) {
              std::cout << "Found bounding box existing SharedControl\n";
              lidar_bbox_values_pointer = response.first; // attach to existing
              break;
          } else {
              continue;
          }

      }
      catch(boost::interprocess::interprocess_exception& ex)
      { 
          std::cerr << "Error: " << ex.what() << std::endl;
          continue;
      }

    }

}

void Bridge::InitLidarCameraThreads(){

    if (!RecieveLidarDataAsync.joinable()) {
        RecieveLidarDataAsync = std::thread(&Bridge::RecieveLidarDataAsync_func, this);
    }

    if (!RecieveCameraDataAsync.joinable()) {
        RecieveCameraDataAsync = std::thread(&Bridge::RecieveCameraDataAsync_func, this);
    }


}

void Bridge::JoinLidarCameraThreads(){

    run_flag = false;

    if (RecieveLidarDataAsync.joinable()) {
        RecieveLidarDataAsync.join();
    }

    if (RecieveCameraDataAsync.joinable()) {
        RecieveCameraDataAsync.join();
    }

    
}
