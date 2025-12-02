#include <iostream>

#include <lidar_yaml_config.hpp>

#include <pcl/ModelCoefficients.h>

#include <pcl/io/pcd_io.h>

#include <pcl/point_types.h>

#include <pcl/sample_consensus/method_types.h>

#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/filters/extract_indices.h>

#include <pcl/filters/convolution_3d.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <boost/interprocess/managed_shared_memory.hpp>
//#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>
#include <boost/interprocess/ipc/message_queue.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread.hpp>

#include <omp.h>

struct Point_XYZIR{
    float x;
    float y;
    float z;
    float intensity;
    uint32_t ring;
    uint32_t padding[3];
};

//struct BoardDetection{
//    float border_points[4*3];
//    float center[3];
//    float normal_vector[3];
//    float plane_d;
//};

struct SharedControl {
    //boost::interprocess::interprocess_mutex mtx;
    //boost::interprocess::interprocess_condition cond_finished;

    bool resized;   // flag for resize notification
};

struct line_model_3D{
    Eigen::Vector3f line_point;
    Eigen::Vector3f line_vector;
};

struct line_model_2D{
    Eigen::Vector2f line_point;
    Eigen::Vector2f line_vector;
};

Eigen::Vector3f up_vector;
float lidar_dist_offset;


void find_board_edges(pcl::PointCloud<pcl::PointXYZ>::Ptr side_edge_cloud_input,std::vector<pcl::ModelCoefficients> &edge_lines_side_output);

Eigen::Vector3f line_intersect(line_model_3D &line1, line_model_3D &line2);

std::unique_ptr<boost::interprocess::managed_shared_memory> segment_ptr;
std::unique_ptr<boost::interprocess::named_mutex> lidar_ctrl_mtx;
SharedControl* lidar_resize_notification = nullptr;

int main ()
{

LidarYamlConfig cfg_yaml;
if (!loadLidarConfig("../config/cfg.yaml", cfg_yaml)) {
    std::cerr << "Failed to load config, loading default parameters\n";

    cfg_yaml.lidar_bounds.x_min = -10.0f;
    cfg_yaml.lidar_bounds.x_max = 10.0f;
    cfg_yaml.lidar_bounds.y_min = -0.95f;
    cfg_yaml.lidar_bounds.y_max = 10.0f;
    cfg_yaml.lidar_bounds.z_min = -10.0f;
    cfg_yaml.lidar_bounds.z_max = 10.0f;

    cfg_yaml.coord_sys.right = Eigen::Vector3f(1.0f,0.0f,0.0f);
    cfg_yaml.coord_sys.up = Eigen::Vector3f(0.0f,1.0f,0.0f);
    cfg_yaml.coord_sys.front = Eigen::Vector3f(0.0f,0.0f,-1.0f);

    cfg_yaml.lidar_offset = 0.0f;

    //return 1;
}

up_vector = cfg_yaml.coord_sys.up;
lidar_dist_offset = cfg_yaml.lidar_offset;

float detected_board[21];
uint32_t detected_board_element_size = sizeof(detected_board)/sizeof(float);

// Open the existing message queue
std::unique_ptr<boost::interprocess::message_queue> mq;
std::unique_ptr<boost::interprocess::message_queue> mq_result;
uint32_t lidar_scan_size;

lidar_ctrl_mtx = std::make_unique<boost::interprocess::named_mutex>(boost::interprocess::open_or_create, "lidar_ctrl_mtx");
boost::posix_time::ptime now = boost::posix_time::microsec_clock::universal_time();
boost::posix_time::ptime abs_time = now + boost::posix_time::milliseconds(500);
if(!(lidar_ctrl_mtx->timed_lock(abs_time)))lidar_ctrl_mtx->unlock();

while(true){

    boost::this_thread::sleep(boost::posix_time::milliseconds(500));

    try{
        segment_ptr = std::make_unique<boost::interprocess::managed_shared_memory>(boost::interprocess::open_only, "shared_mem");
        //lidar_resize_notification = segment.construct<SharedControl>("shared_notification")();

        // Look for "Ctrl"
        auto response = segment_ptr->find<SharedControl>("shared_notification");
        if (response.first != nullptr) {
            std::cout << "Found existing SharedControl"<<std::endl;
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
        mq = std::make_unique<boost::interprocess::message_queue>(boost::interprocess::open_only, "pts_queue");
        lidar_scan_size = mq->get_max_msg_size()/sizeof(Point_XYZIR);



        mq_result = std::make_unique<boost::interprocess::message_queue>(boost::interprocess::open_only, "pts_queue_result");

        break;
        // Fill in the cloud data

        //cloud->points.resize (lidar_scan_size);

        
    }
    catch (boost::interprocess::interprocess_exception& ex) 
    {
        std::cerr << "Error: " << ex.what() << std::endl;
        continue;
    }
}

std::cerr << "Point cloud size: " << lidar_scan_size << " points" << std::endl;

std::vector<Point_XYZIR> lidar_points(lidar_scan_size);
std::vector<float> lidar_inliers;


std::cerr << "resized state " << lidar_resize_notification->resized << " xxx" << std::endl;

while(true){

    uint64_t recvd_size;
    uint32_t priority;

    try
    {   

        bool recieve_queue_state = false;

        while(!recieve_queue_state){

            std::cout << "Pass5"<<std::endl;

            now = boost::posix_time::microsec_clock::universal_time();
            abs_time = now + boost::posix_time::milliseconds(500);

            //boost::interprocess::scoped_lock<boost::interprocess::named_mutex> recieve_lock(*lidar_ctrl_mtx.get(), boost::interprocess::defer_lock);

            if(!lidar_ctrl_mtx->timed_lock(abs_time))
            {
                lidar_ctrl_mtx->unlock();
                lidar_ctrl_mtx->lock();
            }


            std::cout << "Pass5.1"<<std::endl;
            if(!lidar_resize_notification->resized){
                std::cout << "Pass6"<<std::endl;
                boost::posix_time::ptime now = boost::posix_time::microsec_clock::universal_time();
                boost::posix_time::ptime abs_time = now + boost::posix_time::milliseconds(500);
                recieve_queue_state = mq->timed_receive(lidar_points.data(), lidar_scan_size*sizeof(Point_XYZIR), recvd_size, priority, abs_time);
                lidar_ctrl_mtx->unlock();
                std::cout << "Pass7"<<std::endl;
            }else {

                mq.reset();
                mq_result.reset();

                mq = std::make_unique<boost::interprocess::message_queue>(boost::interprocess::open_only, "pts_queue");
                //lidar_scan_size = mq->get_max_msg_size()/sizeof(Point_XYZIR);
                
                mq_result = std::make_unique<boost::interprocess::message_queue>(boost::interprocess::open_only, "pts_queue_result");

                lidar_scan_size = mq->get_max_msg_size()/sizeof(Point_XYZIR);
                lidar_points.resize(lidar_scan_size);

                lidar_resize_notification->resized = false;

                lidar_ctrl_mtx->unlock();
                continue;
            }
            
            //std::cerr << "Pass: " << std::endl;
        }
    }
    catch (boost::interprocess::interprocess_exception& ex) 
    {
        std::cerr << "Error: " << ex.what() << std::endl;

        mq.reset();
        mq_result.reset();
        now = boost::posix_time::microsec_clock::universal_time();
        abs_time = now + boost::posix_time::milliseconds(500);

        //boost::interprocess::scoped_lock<boost::interprocess::named_mutex> recieve_lock(*lidar_ctrl_mtx.get(), boost::interprocess::defer_lock);

        if(!lidar_ctrl_mtx->timed_lock(abs_time))
        {
            lidar_ctrl_mtx->unlock();
            lidar_ctrl_mtx->lock();
        }

        mq = std::make_unique<boost::interprocess::message_queue>(boost::interprocess::open_only, "pts_queue");
        lidar_scan_size = mq->get_max_msg_size()/sizeof(Point_XYZIR);
        lidar_points.resize(lidar_scan_size);
        mq_result = std::make_unique<boost::interprocess::message_queue>(boost::interprocess::open_only, "pts_queue_result");
        lidar_ctrl_mtx->unlock();

        continue;
    }
    //std::cout << lidar_points.at(55500).x << ", " 
    //<< lidar_points.at(55500).y << ", "
    //<< lidar_points.at(55500).z << ", "
    //<< lidar_points.at(55500).ring << std::endl;
    
    //lidar_points.resize(recvd_size / sizeof(Point_XYZIR));
    //cloud->points.resize (recvd_size / sizeof(Point_XYZIR));

    //pcl::PointCloud<pcl::PointXYZL>::Ptr cloud_not_filtered(new pcl::PointCloud<pcl::PointXYZL>);
    pcl::PointCloud<pcl::PointXYZL>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZL>);

    //float offset = 0.06f;
    for (uint32_t i = 0; i < lidar_points.size(); i++){

        //(*cloud)[i].x  = lidar_points[i].x;

        //(*cloud)[i].y  = lidar_points[i].y;

        //(*cloud)[i].z  = lidar_points[i].z; 

        pcl::PointXYZL point; 
        point.x = lidar_points[i].x;
        point.y = lidar_points[i].y;
        point.z = lidar_points[i].z;
        point.label = lidar_points[i].ring;

        if(point.x >= cfg_yaml.lidar_bounds.x_min && point.x <= cfg_yaml.lidar_bounds.x_max &&
           point.y >= cfg_yaml.lidar_bounds.y_min && point.y <= cfg_yaml.lidar_bounds.y_max &&
           point.z >= cfg_yaml.lidar_bounds.z_min && point.z <= cfg_yaml.lidar_bounds.z_max)
        {   

            //conpute offset
            if(lidar_dist_offset != 0.0f){
                float length = std::sqrt(point.x*point.x+point.y*point.y+point.z*point.z);
                point.x += lidar_dist_offset*(point.x/length);
                point.y += lidar_dist_offset*(point.y/length);
                point.z += lidar_dist_offset*(point.z/length);
            }
            //cloud_not_filtered->push_back(point);
            cloud->push_back(point);
        }
        

    }

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    // create gaussian filter object

    //pcl::PointCloud<pcl::PointXYZL>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZL>);

    //pcl::filters::GaussianKernel<pcl::PointXYZL, pcl::PointXYZL> gaussian_kernel;
    //gaussian_kernel.setSigma(0.15);
    //gaussian_kernel.setThresholdRelativeToSigma(6);

    //pcl::filters::Convolution3D<pcl::PointXYZL, pcl::PointXYZL, pcl::filters::GaussianKernel<pcl::PointXYZL, pcl::PointXYZL>> convolution;
    //convolution.setInputCloud(cloud_not_filtered);
    //convolution.setKernel(gaussian_kernel);

    //pcl::search::KdTree<pcl::PointXYZL>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZL>);
    //convolution.setSearchMethod(kdtree);

    //convolution.setRadiusSearch(0.1);
    //convolution.convolve(*cloud);

    // Create the segmentation object

    pcl::SACSegmentation<pcl::PointXYZL> seg;

    // Optional

    seg.setOptimizeCoefficients (true);

    // Mandatory

    seg.setModelType (pcl::SACMODEL_PLANE);

    seg.setMethodType (pcl::SAC_LMEDS);

    seg.setDistanceThreshold (0.03);

    seg.setMaxIterations (300);

    seg.setInputCloud (cloud);

    seg.segment (*inliers, *coefficients);


    if (inliers->indices.size () != 0)
    {

        std::cerr << "Model coefficients: " << coefficients->values[0] << " " 

                                            << coefficients->values[1] << " "

                                            << coefficients->values[2] << " " 

                                            << coefficients->values[3] << std::endl;


        try
        {
        
        boost::interprocess::scoped_lock<boost::interprocess::named_mutex> send_lock(*lidar_ctrl_mtx.get(), boost::interprocess::try_to_lock);

        bool is_send_queue_not_full = false;
        
        if(send_lock){
            if(!lidar_resize_notification->resized){
                is_send_queue_not_full = (mq_result->get_max_msg() > mq_result->get_num_msg());
                send_lock.unlock();//unnecessary
            }else {
                send_lock.unlock();
                continue;
            }
        }
        else{
            continue;
        }

        if(is_send_queue_not_full){

            lidar_inliers.resize(inliers->indices.size()*3+detected_board_element_size);
            std::map<uint32_t, std::vector<Eigen::Vector3f>> points_in_rings;
            std::map<uint32_t, std::vector<Eigen::Vector3f>> borders_in_rings;

            uint32_t j = 0;
            for (const auto& idx : inliers->indices) {

                Eigen::Vector3f point = Eigen::Vector3f((*cloud)[idx].x, (*cloud)[idx].y, (*cloud)[idx].z);

                lidar_inliers[j]   = point.x();
                lidar_inliers[j+1] = point.y();
                lidar_inliers[j+2] = point.z();

                //pcl::PointXYZHSV point_dir;
                //point_dir.x = point.x();
                //point_dir.y = point.y();
                //point_dir.z = point.z();

                //point.normalize();

                //point_dir.h = point.x();
                //point_dir.s = point.y();
                //point_dir.v = point.z();

                points_in_rings[(*cloud)[idx].label].push_back(point);

                j+=3;
            }

            uint32_t inliers_size = j;

            std::cout << (*cloud)[100].x << ", " 
                      << (*cloud)[100].y << ", "
                      << (*cloud)[100].z << ", "
                      << (*cloud)[100].label << std::endl;

            Eigen::Vector3f plane_normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);

            for (auto& [ring_id, points] : points_in_rings) {
                std::sort(points.begin(), points.end(), [](const Eigen::Vector3f& a, const Eigen::Vector3f& b) {
                //return std::atan2(a.y, a.x) < std::atan2(b.y, b.x);}
                //Eigen::Vector3f a_norm = a.normalized();//
                //Eigen::Vector3f b_norm = b.normalized();//
                return ((a.cross(b)).dot(up_vector) >= 0.0f);});

                if(points.size() > 1){ 
                    borders_in_rings[ring_id].push_back(points.front());
                    borders_in_rings[ring_id].push_back(points.back());

                    //borders_in_rings[ring_id][0].normalize();//
                    //borders_in_rings[ring_id][1].normalize();//

                    float t = (-coefficients->values[3])/(plane_normal.dot(borders_in_rings[ring_id][0]));
                    borders_in_rings[ring_id][0] = t*borders_in_rings[ring_id][0];

                    t = (-coefficients->values[3])/(plane_normal.dot(borders_in_rings[ring_id][1]));
                    borders_in_rings[ring_id][1] = t*borders_in_rings[ring_id][1];
                } else{
                    borders_in_rings[ring_id].push_back(points.front());

                    //borders_in_rings[ring_id][0].normalize();//
                    
                    float t = (-coefficients->values[3])/(plane_normal.dot(borders_in_rings[ring_id][0]));
                    borders_in_rings[ring_id][0] = t*borders_in_rings[ring_id][0];
                }

            }
            /*
            j = 0;
            for (const auto& [ring_id, points] : borders_in_rings){
                for (const auto& point_proj : points){
                    lidar_inliers[j]   = point_proj.x();
                    lidar_inliers[j+1] = point_proj.y();
                    lidar_inliers[j+2] = point_proj.z();
                    
                    j+=3;
                }
            }
            */

            ////try
            ////{
            ////    mq_result->try_send(lidar_inliers.data(), j*sizeof(float), /*priority*/ 0);
            ////}
            ////catch (boost::interprocess::interprocess_exception& ex) 
            ////{
            ////    std::cerr << "Error: " << ex.what() << std::endl;
            ////    continue;
            ////}

            //auto it = borders_in_rings.begin();
            //uint32_t first_key = it->first;
            //const std::vector<Eigen::Vector3f>& first_value = it->second;

            //std::cout << borders_in_rings.end()->first << std::endl;

            //std::cout << "check1" << std::endl;

            Eigen::Vector3f plane_coord_origin = (borders_in_rings.begin()->second)[0];
            Eigen::Vector3f planar_vec_x = ((std::prev(borders_in_rings.end())->second)[0] - (borders_in_rings.begin()->second)[0]).normalized();
            Eigen::Vector3f planar_vec_z = (Eigen::Vector3f(coefficients->values[0], coefficients->values[1], coefficients->values[2])).normalized();
            Eigen::Vector3f planar_vec_y = (planar_vec_z.cross(planar_vec_x)).normalized();

//
//            //std::cout << "check2" << std::endl;
//
            Eigen::Matrix4f plane_coord = Eigen::Matrix4f::Identity();
            plane_coord.block<3,1>(0,0) = planar_vec_x;
            plane_coord.block<3,1>(0,1) = planar_vec_y;
            plane_coord.block<3,1>(0,2) = planar_vec_z;
            plane_coord.block<3,1>(0,3) = plane_coord_origin;

            Eigen::Matrix4f plane_coord_inverse = plane_coord.inverse();

            pcl::PointCloud<pcl::PointXYZ>::Ptr left_edge_cloud(new  pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr right_edge_cloud(new  pcl::PointCloud<pcl::PointXYZ>);

//            uint32_t j_plane = 0;
//            Eigen::Vector3f center_plane(0.0f,0.0f,0.0f);

            for (auto& [ring_id, points] : borders_in_rings){
                for (auto& point_proj : points){
                    Eigen::Vector4f point4 = point_proj.homogeneous();//Eigen::Vector4f point4(point_proj, 1.0f);
                    Eigen::Vector4f transformed_point4 = plane_coord_inverse * point4;
                    point_proj = transformed_point4.head<3>();

//                    center_plane+=point_proj;
//                    j_plane++;
                }

                pcl::PointXYZ point;
                point.z = 0.0f;

                if(points.size() > 1){

                    point.x = points[0].x();
                    point.y = points[0].y();

                    right_edge_cloud->push_back(point);

                    point.x = points[1].x();
                    point.y = points[1].y();

                    left_edge_cloud->push_back(point);
                } else{

                    point.x = points[0].x();
                    point.y = points[0].y();

                    right_edge_cloud->push_back(point);
                }
            }

//            center_plane/=static_cast<float>(j_plane);

            std::vector<pcl::ModelCoefficients> edge_lines_right;
            std::vector<pcl::ModelCoefficients> edge_lines_left;

            //std::cout << "check3" << std::endl;
            //pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

            //pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

            #pragma omp parallel num_threads(2)
            {
                #pragma omp single
                {   

                    #pragma omp task
                        find_board_edges(right_edge_cloud,edge_lines_right);
                    #pragma omp task
                        find_board_edges(left_edge_cloud,edge_lines_left);

                    #pragma omp taskwait
                }
            }

            float board_points[3*5];

            if(!edge_lines_right.empty() && !edge_lines_left.empty()){

                line_model_3D board_edges[4]; //{right1,leftx},{right2,lefty}
                Eigen::Vector3f board_corners[4];

                uint32_t right_edge_vec_size = edge_lines_right.size();
                uint32_t left_edge_vec_size = edge_lines_left.size();

                Eigen::Vector3f right_normal1(edge_lines_right[0].values[3],edge_lines_right[0].values[4],0.0f);
                Eigen::Vector3f left_normal1(edge_lines_left[0].values[3],edge_lines_left[0].values[4],0.0f);

                right_normal1.normalize();
                left_normal1.normalize();

                float tol_cos_ort = 0.34; //20°
                float tol_cos_parall = 0.94; //20°

                if(right_edge_vec_size + left_edge_vec_size == 4)
                {
                    Eigen::Vector3f right_normal2(edge_lines_right[1].values[3],edge_lines_right[1].values[4],0.0f);
                    Eigen::Vector3f left_normal2(edge_lines_left[1].values[3],edge_lines_left[1].values[4],0.0f);
                    
                    right_normal2.normalize();
                    left_normal2.normalize();
                    
                    if(std::abs(right_normal1.dot(right_normal2)) > tol_cos_ort || left_normal1.dot(left_normal2) > tol_cos_ort){
                        PCL_ERROR ("Could not detect board edges for the given dataset, keep the board at 45 degrees.\n");
                        continue;
                    }

                    if(std::abs(right_normal1.dot(left_normal1)) > std::abs(right_normal1.dot(left_normal2)))
                    {
                        board_edges[0].line_vector = right_normal1;
                        board_edges[1].line_vector = left_normal1;
                        board_edges[2].line_vector = right_normal2;
                        board_edges[3].line_vector = left_normal2;

                        board_edges[0].line_point = Eigen::Vector3f(edge_lines_right[0].values[0],edge_lines_right[0].values[1],0.0f);
                        board_edges[1].line_point = Eigen::Vector3f(edge_lines_left[0].values[0],edge_lines_left[0].values[1],0.0f);
                        board_edges[2].line_point = Eigen::Vector3f(edge_lines_right[1].values[0],edge_lines_right[1].values[1],0.0f);
                        board_edges[3].line_point = Eigen::Vector3f(edge_lines_left[1].values[0],edge_lines_left[1].values[1],0.0f);
                        
                        board_corners[0] = line_intersect(board_edges[0],board_edges[2]);
                        board_corners[1] = line_intersect(board_edges[0],board_edges[3]);
                        board_corners[2] = line_intersect(board_edges[3],board_edges[1]);
                        board_corners[3] = line_intersect(board_edges[2],board_edges[1]);
                        

                    }
                    else
                    {
                        board_edges[0].line_vector = right_normal1;
                        board_edges[1].line_vector = left_normal2;
                        board_edges[2].line_vector = right_normal2;
                        board_edges[3].line_vector = left_normal1;

                        board_edges[0].line_point = Eigen::Vector3f(edge_lines_right[0].values[0],edge_lines_right[0].values[1],0.0f);
                        board_edges[1].line_point = Eigen::Vector3f(edge_lines_left[1].values[0],edge_lines_left[1].values[1],0.0f);
                        board_edges[2].line_point = Eigen::Vector3f(edge_lines_right[1].values[0],edge_lines_right[1].values[1],0.0f);
                        board_edges[3].line_point = Eigen::Vector3f(edge_lines_left[0].values[0],edge_lines_left[0].values[1],0.0f);

                        board_corners[0] = line_intersect(board_edges[0],board_edges[2]);
                        board_corners[1] = line_intersect(board_edges[0],board_edges[3]);
                        board_corners[2] = line_intersect(board_edges[3],board_edges[1]);
                        board_corners[3] = line_intersect(board_edges[2],board_edges[1]);
                    }

                    

                }



                else if(right_edge_vec_size + left_edge_vec_size == 3)
                {   

                    PCL_WARN ("Could not detect one of the board edges for the given dataset, the mssing edge has to be estimated.\n");
                    PCL_WARN ("Sample discarted.\n");
                    continue;
//
//                    if(right_edge_vec_size == 2)
//                    {   
//                        Eigen::Vector3f right_normal2(edge_lines_right[1].values[3],edge_lines_right[1].values[4],0.0f);
//                        right_normal2.normalize();
//
//                        if(std::abs(right_normal1.dot(right_normal2)) > tol_cos_ort){
//                            PCL_ERROR ("Could not detect board edges for the given dataset, keep the board at 45 degrees.\n");
//                            continue;
//                        } 
//
//
//                        if(std::abs(right_normal1.dot(left_normal1)) > std::abs(right_normal2.dot(left_normal1)))
//                        {
//                            board_edges[0].line_vector = right_normal1;
//                            board_edges[1].line_vector = left_normal1;
//                            board_edges[2].line_vector = right_normal2;
//                            board_edges[3].line_vector = right_normal2;//
//
//                            board_edges[0].line_point = Eigen::Vector3f(edge_lines_right[0].values[0],edge_lines_right[0].values[1],0.0f);
//                            board_edges[1].line_point = Eigen::Vector3f(edge_lines_left[0].values[0],edge_lines_left[0].values[1],0.0f);
//                            board_edges[2].line_point = Eigen::Vector3f(edge_lines_right[1].values[0],edge_lines_right[1].values[1],0.0f);
//                            //board_edges[3].line_point = Eigen::Vector3f(edge_lines_right[1].values[0],edge_lines_right[1].values[1],0.0f);
//                            
//
//                        }
//                        else
//                        {
//                            board_edges[0].line_vector = right_normal1;
//                            board_edges[1].line_vector = left_normal1;//
//                            board_edges[2].line_vector = right_normal2;
//                            board_edges[3].line_vector = left_normal1;
//
//                            board_edges[0].line_point = Eigen::Vector3f(edge_lines_right[0].values[0],edge_lines_right[0].values[1],0.0f);
//                            //board_edges[1].line_point = Eigen::Vector3f(edge_lines_left[0].values[0],edge_lines_left[0].values[1],0.0f);
//                            board_edges[2].line_point = Eigen::Vector3f(edge_lines_right[1].values[0],edge_lines_right[1].values[1],0.0f);
//                            board_edges[3].line_point = Eigen::Vector3f(edge_lines_left[0].values[0],edge_lines_left[0].values[1],0.0f);
//                        }
//
//                    }
//                    else
//                    {
//                        Eigen::Vector3f left_normal2(edge_lines_left[1].values[3],edge_lines_left[1].values[4],0.0f);
//                        left_normal2.normalize();
//
//                        if(std::abs(left_normal1.dot(left_normal2)) > tol_cos_ort){
//                            PCL_ERROR ("Could not detect board edges for the given dataset, keep the board at 45 degrees.\n");
//                            continue;
//                        } 
//
//                        if(std::abs(right_normal1.dot(left_normal1)) > std::abs(right_normal1.dot(left_normal2)))
//                        {
//                            board_edges[0].line_vector = right_normal1;
//                            board_edges[1].line_vector = left_normal1;
//                            board_edges[2].line_vector = right_normal1;//
//                            board_edges[3].line_vector = left_normal2;
//
//                            board_edges[0].line_point = Eigen::Vector3f(edge_lines_right[0].values[0],edge_lines_right[0].values[1],0.0f);
//                            board_edges[1].line_point = Eigen::Vector3f(edge_lines_left[0].values[0],edge_lines_left[0].values[1],0.0f);
//                            //board_edges[2].line_point = Eigen::Vector3f(edge_lines_right[0].values[0],edge_lines_right[0].values[1],0.0f);
//                            board_edges[3].line_point = Eigen::Vector3f(edge_lines_left[1].values[0],edge_lines_left[1].values[1],0.0f);
//                            
//
//                        }
//                        else
//                        {
//                            board_edges[0].line_vector = right_normal1;
//                            board_edges[1].line_vector = left_normal2;
//                            board_edges[2].line_vector = right_normal1;//
//                            board_edges[3].line_vector = left_normal1;
//
//                            board_edges[0].line_point = Eigen::Vector3f(edge_lines_right[0].values[0],edge_lines_right[0].values[1],0.0f);
//                            board_edges[1].line_point = Eigen::Vector3f(edge_lines_left[1].values[0],edge_lines_left[1].values[1],0.0f);
//                            //board_edges[2].line_point = Eigen::Vector3f(edge_lines_right[1].values[0],edge_lines_right[1].values[1],0.0f);
//                            board_edges[3].line_point = Eigen::Vector3f(edge_lines_left[0].values[0],edge_lines_left[0].values[1],0.0f);
//                        }
//                    }
//
//                    
                }
                else
                {
                    if(std::abs(right_normal1.dot(left_normal1)) < tol_cos_parall){
                        PCL_ERROR ("Could not detect board edges for the given dataset, keep the board at 45 degrees.\n");
                        continue;
                    }
                        PCL_WARN ("Could not detect two of the board edges for the given dataset, the two mssing edges have to be estimated.\n");
                        PCL_WARN ("Sample discarted.\n");
                        continue;
                        
//                        board_edges[0].line_vector = right_normal1;
//                        //board_edges[1].line_vector = left_normal2;
//                        //board_edges[2].line_vector = right_normal1;//
//                        board_edges[3].line_vector = left_normal1;
//
//                        board_edges[0].line_point = Eigen::Vector3f(edge_lines_right[0].values[0],edge_lines_right[0].values[1],0.0f);
//                        //board_edges[1].line_point = Eigen::Vector3f(edge_lines_left[1].values[0],edge_lines_left[1].values[1],0.0f);
//                        //board_edges[2].line_point = Eigen::Vector3f(edge_lines_right[1].values[0],edge_lines_right[1].values[1],0.0f);
//                        board_edges[3].line_point = Eigen::Vector3f(edge_lines_left[0].values[0],edge_lines_left[0].values[1],0.0f);
                        

                }

                //extracting the center
                
                board_corners[0] = (plane_coord*(board_corners[0].homogeneous())).head<3>();
                board_corners[1] = (plane_coord*(board_corners[1].homogeneous())).head<3>();
                board_corners[2] = (plane_coord*(board_corners[2].homogeneous())).head<3>();
                board_corners[3] = (plane_coord*(board_corners[3].homogeneous())).head<3>();
                
                for(uint8_t i = 0; i< 4;i++)
                {
                    board_points[3*i] = board_corners[i].x();
                    board_points[3*i +1] = board_corners[i].y();
                    board_points[3*i +2] = board_corners[i].z();
                }

                Eigen::Vector3f center = (board_corners[0] + board_corners[2])*0.25f;
                center += (board_corners[1] + board_corners[3])*0.25f;

                board_points[3*4] = center.x();
                board_points[3*4 +1] = center.y();
                board_points[3*4 +2] = center.z();

                std::memcpy(detected_board,board_points,15*sizeof(float));
                if(plane_normal.dot((-center).normalized()) < 0.0f) plane_normal = -plane_normal;
                detected_board[15] = plane_normal.x();
                detected_board[16] = plane_normal.y();
                detected_board[17] = plane_normal.z();
                detected_board[18] = coefficients->values[3];
                //detected_board[18] = -(detected_board[15]*detected_board[12]+
                //                       detected_board[16]*detected_board[13]+
                //                       detected_board[17]*detected_board[14]);

                uint32_t lidar_inliers_board_start = inliers_size;//lidar_inliers.size()-sizeof(detected_board)/sizeof(float)

                std::memcpy(lidar_inliers.data()+lidar_inliers_board_start,detected_board,sizeof(detected_board));
                std::cout<<"normal x: "<<*(lidar_inliers.data()+lidar_inliers_board_start+15)<< std::endl;
                std::cout<<"inliers size: "<<(lidar_inliers.size()*sizeof(float))<< std::endl;
                 for(int i = 0;i<21;i++){
                    std::cout<<"["<<lidar_inliers[i+lidar_inliers_board_start] <<"]";
                }
                std::cout<<std::endl;
                //uint32_t lidar_inliers_last_element = lidar_inliers.size()-1U;
                //for (uint32_t i = 0; i < detected_board_element_size; i++) {
                //    lidar_inliers[lidar_inliers_last_element-i] = detected_board[i];
                //}

                is_send_queue_not_full = false;
        

                try
                {
                    boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*lidar_ctrl_mtx.get(), boost::interprocess::try_to_lock);

                    if(lock){
                        if(!lidar_resize_notification->resized){
                            mq_result->try_send(lidar_inliers.data(), inliers_size*sizeof(float)+sizeof(detected_board), /*priority*/ 0);
                            //mq_result->try_send(board_points, 5*3*sizeof(float), /*priority*/ 0);
                            lock.unlock();

                        }else{
                            lock.unlock();
                            continue;
                        }


                    }
                    else{
                        continue;
                    }
                }
                catch (boost::interprocess::interprocess_exception& ex) 
                {
                    std::cerr << "Error: " << ex.what() << std::endl;
                    continue;
                }

//                //mq_result.try_send(lidar_inliers.data(), j*sizeof(float), /*priority*/ 0);
//                mq_result.try_send(board_points, 5*3*sizeof(float), /*priority*/ 0);
            }
            else{
                PCL_ERROR ("Could not detect board edges for the given dataset.\n");
            }
            
            //mq_result.try_send(lidar_inliers.data(), inliers->indices.size()*3*sizeof(float), /*priority*/ 0);

        }
        }
        catch (boost::interprocess::interprocess_exception& ex) 
        {
            std::cerr << "Error: " << ex.what() << std::endl;
            continue;
        }

    }
    else
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.\n");
    }

}

return (0);

}

void find_board_edges(pcl::PointCloud<pcl::PointXYZ>::Ptr side_edge_cloud_input,std::vector<pcl::ModelCoefficients> &edge_lines_side_output){

    pcl::ModelCoefficients full_coeff, half_coeff;
    pcl::PointIndices::Ptr full_inliers(new pcl::PointIndices), half_inliers(new pcl::PointIndices);
    pcl::PointCloud<pcl::PointXYZ>::Ptr half_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setModelType(pcl::SACMODEL_LINE);
    seg.setMethodType(pcl::SAC_LMEDS);
    seg.setDistanceThreshold(0.02);
    seg.setMaxIterations(300);
    seg.setInputCloud(side_edge_cloud_input);
    seg.segment(*full_inliers, full_coeff);  // Fitting line1 through all points

     // Failed RANSAC returns empty coeffs
    if (full_coeff.values.empty()) {
        return;
    }

    edge_lines_side_output.push_back(full_coeff);

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(side_edge_cloud_input);
    extract.setIndices(full_inliers);
    extract.setNegative(true);
    extract.filter(*half_cloud);
    seg.setInputCloud(half_cloud);
    seg.segment(*half_inliers, half_coeff);

     // Failed RANSAC returns empty coeffs        
    if (half_coeff.values.empty()) {
        return;
    }

    edge_lines_side_output.push_back(half_coeff);

}

Eigen::Vector3f line_intersect(line_model_3D &line1, line_model_3D &line2){

    Eigen::Matrix2f mat1;
    Eigen::Matrix2f mat2;

    mat1(0,0) = line2.line_point.x() - line1.line_point.x();
    mat1(1,0) = line2.line_point.y() - line1.line_point.y();
    mat1(0,1) = -line2.line_vector.x();
    mat1(1,1) = -line2.line_vector.y();

    mat2(0,0) = line1.line_vector.x();
    mat2(1,0) = line1.line_vector.y();
    mat2(0,1) = -line2.line_vector.x();
    mat2(1,1) = -line2.line_vector.y();

    float t_l1 = mat1.determinant()/mat2.determinant();

    return (line1.line_point + t_l1*line1.line_vector);

}