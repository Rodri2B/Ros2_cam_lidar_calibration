#include <iostream> 
#include <vector>

#include <boost/interprocess/managed_shared_memory.hpp>
//#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>
#include <boost/interprocess/ipc/message_queue.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread.hpp>

#include <opencv2/opencv.hpp>

#include <yaml_config.hpp>


//struct BoardDetection{
//    float border_points[4*3];
//    float center[3];
//    float normal_vector[3];
//    float plane_d;
//};

struct SharedControl {
    bool resized;   // flag for resize notification
};


std::unique_ptr<boost::interprocess::managed_shared_memory> segment_ptr;
std::unique_ptr<boost::interprocess::named_mutex> camera_ctrl_mtx;
SharedControl* camera_resize_notification = nullptr;




int main ()
{

//Camera settings

YamlConfig cfg_yaml;
if (!loadConfig("../config/cfg.yaml", cfg_yaml)) {
    std::cerr << "Failed to load config, loading default parameters\n";

    cfg_yaml.chessboard.pattern_size.height = 8;
    cfg_yaml.chessboard.pattern_size.width = 6;
    cfg_yaml.chessboard.square_length = 0.065f;
    cfg_yaml.chessboard.board_dimension.width = 0.650f;
    cfg_yaml.chessboard.board_dimension.height = 0.910f;
    cfg_yaml.chessboard.translation_error.x = 0.010f;
    cfg_yaml.chessboard.translation_error.y = 0.030f;

    cfg_yaml.camera.image_size.width = 1000;
    cfg_yaml.camera.image_size.height = 600;

    cfg_yaml.camera.K = {724.26,0.0,500.0,0.0,724.26,300.0,0.0,0.0,1.0};

    cfg_yaml.camera.distortion_model = "plumb_bob";

    cfg_yaml.camera.D = {-0.170,0.041,-0.005,0.001,0.000};

    //return 1;
}

// --- Chessboard Parameters ---
// Define the number of inner corners per chessboard row and column
cv::Size patternSize(cfg_yaml.chessboard.pattern_size.width, cfg_yaml.chessboard.pattern_size.height);

// Define the real-world size of a square (e.g., 1.0 unit)

float squareSize = cfg_yaml.chessboard.square_length;
float size_x = cfg_yaml.chessboard.board_dimension.width;
float size_y = cfg_yaml.chessboard.board_dimension.height;
float error_x = cfg_yaml.chessboard.translation_error.x;
float error_y = cfg_yaml.chessboard.translation_error.y;

int rows = cfg_yaml.camera.image_size.height;
int cols = cfg_yaml.camera.image_size.width;

cv::Mat cam_matrix(3,3,CV_64FC1,cfg_yaml.camera.K.data());
cv::Mat distortion_c(1,cfg_yaml.camera.D.size(),CV_64FC1,cfg_yaml.camera.D.data());


// Open the existing message queue
std::unique_ptr<boost::interprocess::message_queue> camera_recieve_queue;
std::unique_ptr<boost::interprocess::message_queue> camera_detected_board_queue;
uint32_t image_pixels_number;

camera_ctrl_mtx = std::make_unique<boost::interprocess::named_mutex>(boost::interprocess::open_or_create, "camera_ctrl_mtx");
boost::posix_time::ptime now = boost::posix_time::microsec_clock::universal_time();
boost::posix_time::ptime abs_time = now + boost::posix_time::milliseconds(1000);
if(!(camera_ctrl_mtx->timed_lock(abs_time)))camera_ctrl_mtx->unlock();



while(true){

    boost::this_thread::sleep(boost::posix_time::milliseconds(500));

    try{
        segment_ptr = std::make_unique<boost::interprocess::managed_shared_memory>(boost::interprocess::open_only, "shared_mem");
        //camera_resize_notification = segment.construct<SharedControl>("shared_notification")();

        // Look for "Ctrl"
        auto response = segment_ptr->find<SharedControl>("shared_camera_notification");
        if (response.first != nullptr) {
            std::cout << "Found existing SharedControl"<<std::endl;
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
        camera_recieve_queue = std::make_unique<boost::interprocess::message_queue>(boost::interprocess::open_only, "camera_queue");
        image_pixels_number = camera_recieve_queue->get_max_msg_size()/(3U); //(3U*sizeof(float))



        camera_detected_board_queue = std::make_unique<boost::interprocess::message_queue>(boost::interprocess::open_only, "camera_queue_board");

        break;

        
    }
    catch (boost::interprocess::interprocess_exception& ex) 
    {
        std::cerr << "Error: " << ex.what() << std::endl;
        continue;
    }
}

std::cerr << "Image number of pixels: " << image_pixels_number << " pixels" << std::endl;

std::vector<uint8_t> Camera_image_pixels(image_pixels_number*3U); //std::vector<float>
float Detected_board[19];
//float bounds[4] = {-10.0f,10.0f,-0.95f,10.0f};


std::cerr << "resized state " << camera_resize_notification->resized << " xxx" << std::endl;


// --- Prepare Object Points ---
// Create a vector of 3D points for the chessboard corners in world coordinates.
// These points are in the chessboard coordinate system (z = 0)
std::vector<cv::Point3f> board_world_space_points(patternSize.height*patternSize.width);


for (int i = 0; i < patternSize.height; i++) {
    for (int j = 0; j < patternSize.width; j++) {
        board_world_space_points.at(i*patternSize.width + j) = (cv::Point3f(static_cast<float>(j)*squareSize, static_cast<float>(i)*squareSize, 0.0f));
    }
}


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

while(true){

    uint64_t recvd_size;
    uint32_t priority;

    try
    {   

        bool recieve_queue_state = false;

        while(!recieve_queue_state){

            std::cout << "Pass5"<<std::endl;

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
                recieve_queue_state = camera_recieve_queue->timed_receive(Camera_image_pixels.data(), image_pixels_number*3U, recvd_size, priority, abs_time); //image_pixels_number*3U*sizeof(float)
                camera_ctrl_mtx->unlock();
                std::cout << "Pass7"<<std::endl;
            }else {

                camera_recieve_queue.reset();
                camera_detected_board_queue.reset();

                camera_recieve_queue = std::make_unique<boost::interprocess::message_queue>(boost::interprocess::open_only, "camera_queue");
                image_pixels_number = camera_recieve_queue->get_max_msg_size()/(3U); //(3U*sizeof(float))
                
                camera_detected_board_queue = std::make_unique<boost::interprocess::message_queue>(boost::interprocess::open_only, "camera_queue_board");

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
        std::cerr << "Error: " << ex.what() << std::endl;

        camera_recieve_queue.reset();
        camera_detected_board_queue.reset();
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
        camera_detected_board_queue = std::make_unique<boost::interprocess::message_queue>(boost::interprocess::open_only, "camera_queue_board");
        camera_ctrl_mtx->unlock();

        continue;
    }


    if(image_pixels_number*3U != (uint32_t)(rows*cols*3)){
        std::cerr << "Recieved image and expected image have different sizes\n";
        continue;
    }

    cv::Mat Camera_recieved_image(rows, cols, CV_8UC3, (void*)Camera_image_pixels.data());

    if (Camera_recieved_image.empty()) {
        std::cerr << "Error while casting the recieved image \n";
        continue;
    }

    
    // Convert to grayscale
    cv::Mat Camera_recieved_image_gray; 
    cv::cvtColor(Camera_recieved_image, Camera_recieved_image_gray, cv::COLOR_RGB2GRAY);

    //cv::Mat Camera_recieved_image_gray;
    //Camera_recieved_image_gray_float.convertTo(Camera_recieved_image_gray, CV_8U, 255.0);

    //cv::Mat Camera_recieved_image_BGR;
    //cv::cvtColor(Camera_recieved_image, Camera_recieved_image_BGR, cv::COLOR_RGB2BGR);
    //cv::imshow("image",Camera_recieved_image_BGR);

    //cv::imshow("image",Camera_recieved_image_gray);
    //cv::waitKey(0);
    //cv::destroyAllWindows();

    // --- Detect Chessboard Corners ---
    std::vector<cv::Point2f> Chessboardcorners;
    bool found = cv::findChessboardCorners(Camera_recieved_image_gray, patternSize, Chessboardcorners,
                                       cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE); //| cv::CALIB_CB_FAST_CHECK);


    if (found)
    {

        try
        {
        
        boost::interprocess::scoped_lock<boost::interprocess::named_mutex> send_lock(*camera_ctrl_mtx.get(), boost::interprocess::try_to_lock);

        bool is_send_queue_not_full = false;
        
        if(send_lock){
            if(!camera_resize_notification->resized){
                is_send_queue_not_full = (camera_detected_board_queue->get_max_msg() > camera_detected_board_queue->get_num_msg());
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

                is_send_queue_not_full = false;

                // Enhance corner detection accuracy
                cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001);
                cv::cornerSubPix(Camera_recieved_image_gray, Chessboardcorners, cv::Size(11, 11), cv::Size(-1, -1), criteria);

                cv::Mat rvec, tvec;
                //bool solved = solvePnPRansac(board_world_space_points, corners_rsh, cam_matrix, distortion_c, rvec, tvec);
                bool solved = cv::solvePnP(board_world_space_points, Chessboardcorners, cam_matrix, distortion_c, rvec, tvec);
        

                if(!solved){
                    std::cerr << "Could not solve PNP\n";
                    continue;
                }

                cv::Mat W_to_cam_Rot;
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
                        -Pc_mat.at<float>(0,0),
                        -Pc_mat.at<float>(1,0), //to model opencv coord (-x,-y,z) to opencv camera coord (x,-y,-z)
                        Pc_mat.at<float>(2,0) //to model opencv coord (-x,-y,z) to opencv camera coord (x,-y,-z)
                    );
                    std::cout << "Corner " << i << ": " << Board_corners_camera_coord[i] << std::endl;
                }

                cv::Mat Pw_normal = (cv::Mat_<float>(3,1) << 0.0f, 0.0f, -1.0f);
                cv::Mat Pc_normal = W_to_cam_Rot * Pw_normal;

                cv::Point3f Board_normal_camera_coord = cv::Point3f(
                    -Pc_normal.at<float>(0,0),
                    -Pc_normal.at<float>(1,0), //to model opencv coord (-x,-y,z) to opencv camera coord (x,-y,-z)
                    Pc_normal.at<float>(2,0) //to model opencv coord (-x,-y,z) to opencv camera coord (x,-y,-z)
                );

                std::cout << "Board Normal: " << Board_normal_camera_coord << std::endl;


                //coping to detected board
                for(size_t i = 0; i < (Board_corners_camera_coord.size()); i++){
                    Detected_board[i*3] = Board_corners_camera_coord[i].x;
                    Detected_board[i*3+1] = Board_corners_camera_coord[i].y;
                    Detected_board[i*3+2] = Board_corners_camera_coord[i].z;
                }

                Detected_board[15] = Board_normal_camera_coord.x;
                Detected_board[16] = Board_normal_camera_coord.y;
                Detected_board[17] = Board_normal_camera_coord.z;
                
                //calculating d and writing

                Detected_board[18] = -(Detected_board[15]*Detected_board[12]+
                                       Detected_board[16]*Detected_board[13]+
                                       Detected_board[17]*Detected_board[14]);

                try
                {
                    boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*camera_ctrl_mtx.get(), boost::interprocess::try_to_lock);

                    if(lock){
                        if(!camera_resize_notification->resized){
                            camera_detected_board_queue->try_send(&Detected_board, sizeof(Detected_board), /*priority*/ 0);
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


        }
                    
        //else{
        //        std::cerr << "Send Queue Already full\n";
        //}
        }
        catch (boost::interprocess::interprocess_exception& ex) 
        {
            std::cerr << "Error: " << ex.what() << std::endl;
            continue;
        }

    }
    else
    {
        std::cerr << "Could not detect chessboard pattern.\n";
    }

}

return (0);

}

