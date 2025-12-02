#ifndef SCENE_CLASS
#define SCENE_CLASS

#define IMGUI_ENABLE_FREETYPE
//#define IMGUI_ENABLE_FREETYPE_LUNASVG

#include <imgui.h>
#include <imgui_internal.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include "misc/freetype/imgui_freetype.h"

#define GL_SILENCE_DEPRECATION
#define IMGUI_DEFINE_MATH_OPERATORS


#include <ImGuizmo.h>
#include <imoguizmo.hpp>

#include <lib_opengl/shader.h>

#include <texture_loader.hpp>
#include <object_loader.hpp>

#include <lib_opengl/camera_quat.h>

#include <lidar_scan.hpp>

#include <iostream>

#include <vector>
#include <memory>
#include <algorithm>

#include <boost/interprocess/ipc/message_queue.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
//#include <boost/interprocess/sync/interprocess_mutex.hpp>
//#include <boost/interprocess/sync/interprocess_condition.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>

#include <thread>

#include <calib_yaml_config.hpp>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <Eigen/Dense>

#include <opencv2/opencv.hpp>

#include <omp.h>

#include <atomic>


#define LIDAR_BOARD_HISTORY_SIZE 5U
#define CAMERA_BOARD_HISTORY_SIZE 5U

//const unsigned int LIDAR_BOARD_HISTORY_SIZE = 5;
//const unsigned int CAMERA_BOARD_HISTORY_SIZE = 5;

const unsigned int SCR_WIDTH_V = 800;
const unsigned int SCR_HEIGHT_V = 600;

void mouse_callback(GLFWwindow* window, double xposIn, double yposIn);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);

namespace LidarCamSim {

struct line_model_3D{
    Eigen::Vector3f line_point;
    Eigen::Vector3f line_vector;
};

struct line_model_2D{
    Eigen::Vector2f line_point;
    Eigen::Vector2f line_vector;
};

enum shader_type{
    DEFAULT,   
    WTH_GEOMETRY,
    COMPUTE
};

struct shaderConfig{
    std::string shader_name;
    std::string vertex_path;
    std::string fragment_path;
    std::string geometry_path;
    std::string compute_path;
    shader_type type;
};

struct VirtualCamConfig{
    unsigned int CamWidth;
    unsigned int CamHeight;
    float Render_Ratio;
    float CamVfov;
    glm::vec3 position;
    glm::vec3 yaw_pitch_roll;
    glm::vec3 radialDistortion;
    glm::vec2 tangentialDistortion;
};

struct ViewportCamConfig{

    glm::vec3 pivot_position;
    float pivot_distance;
    glm::vec2 yaw_pitch;
};


struct DetectedBoardDataSimple {
    glm::vec4 detected_board_buffer[6];
    float detected_board_plane_d;
};

//struct VBODualBuffer{
//
//    unsigned int VBO[2];
//    uint32_t lidar_points_number[2];
//    uint8_t buffer_index;
//    uint8_t rbuffer_index;
//    boost::mutex VBO_mutex[2];
//
//    bool points_recieved_first_time[2];
//
//    VBODualBuffer(){
//        buffer_index = 0;
//        rbuffer_index = 1;
//        lidar_points_number[0] = 0;
//        lidar_points_number[1] = 0;
//        points_recieved_first_time[0] = false;
//        points_recieved_first_time[1] = false;
//    }
//
//    void LockWrite(){
//        VBO_mutex[buffer_index].lock();
//    }
//
//    void UnlockWrite(){
//        buffer_index = 1-buffer_index;
//        rbuffer_index = 1-rbuffer_index;
//        VBO_mutex[1-buffer_index].unlock();
//    }
//
//    void LockRead(){
//        VBO_mutex[rbuffer_index].lock();
//        //while(true){
//        //    if(VBO_mutex[0].try_lock()){
//        //        rbuffer_index = 0;
//        //        break;
//        //    } 
//        //    else if(VBO_mutex[1].try_lock()){
//        //        rbuffer_index = 1;
//        //        break;
//        //    } 
//        //}
//    }
//
//    void UnlockRead(){
//        VBO_mutex[rbuffer_index].unlock();
//    }
//};

struct VBOBoardDualBuffer{

    unsigned int VBO[2];
    uint32_t lidar_points_number[2];
    //std::atomic<uint8_t> buffer_index;
    //std::atomic<uint8_t> rbuffer_index;
    uint8_t buffer_index;
    uint8_t rbuffer_index;
    boost::mutex VBO_mutex[2];

    bool points_recieved_first_time[2];

    DetectedBoardDataSimple board_data[2];

    VBOBoardDualBuffer(){
        buffer_index = 0;
        rbuffer_index = 1;
        lidar_points_number[0] = 0;
        lidar_points_number[1] = 0;
        points_recieved_first_time[0] = false;
        points_recieved_first_time[1] = false;
    }

    void LockWrite(){
        VBO_mutex[buffer_index].lock();
    }

    void UnlockWrite(){
        buffer_index = 1-buffer_index;
        rbuffer_index = 1-rbuffer_index;
        VBO_mutex[1-buffer_index].unlock();
    }

    void LockRead(){
        VBO_mutex[rbuffer_index].lock();
        //while(true){
        //    if(VBO_mutex[0].try_lock()){
        //        rbuffer_index = 0;
        //        break;
        //    } 
        //    else if(VBO_mutex[1].try_lock()){
        //        rbuffer_index = 1;
        //        break;
        //    } 
        //}
    }

    void UnlockRead(){
        VBO_mutex[rbuffer_index].unlock();
    }
};


//struct TexBoardDualBuffer{
//
//    unsigned int textureID[2];
//    uint32_t image_width[2];
//    uint32_t image_height[2];
//    uint32_t pixels_number[2];
//    //std::atomic<uint8_t> buffer_index;
//    //std::atomic<uint8_t> rbuffer_index;
//    uint8_t texture_index;
//    uint8_t rtexture_index;
//    boost::mutex textureID_mutex[2];
//
//    bool image_recieved_first_time[2];
//
//    DetectedBoardDataSimple board_data[2];
//
//    bool board_existence[2];
//
//    TexBoardDualBuffer(){
//        texture_index = 0;
//        rtexture_index = 1;
//        pixels_number[0] = 0;
//        pixels_number[1] = 0;
//        image_height[0] = 0;
//        image_height[1] = 0;
//        image_width[0] = 0;
//        image_width[1] = 0;
//        image_recieved_first_time[0] = false;
//        image_recieved_first_time[1] = false;
//    }
//
//    void LockWrite(){
//        textureID_mutex[texture_index].lock();
//    }
//
//    void UnlockWrite(){
//        texture_index = 1-texture_index;
//        rtexture_index = 1-rtexture_index;
//        textureID_mutex[1-texture_index].unlock();
//    }
//
//    void LockRead(){
//        textureID_mutex[rtexture_index].lock();
//    }
//
//    void UnlockRead(){
//        textureID_mutex[rtexture_index].unlock();
//    }
//};

struct TexBoardBuffer{

    uint32_t image_width;
    uint32_t image_height;
    uint32_t pixels_number;

    boost::mutex textureID_mutex;
    DetectedBoardDataSimple board_data;

    std::vector<uint8_t> Camera_draw_cpu_buffer;

    bool board_existence;
    bool image_recieved_first_time;

};

struct VBOBoardBuffer{

    uint32_t lidar_points_number;

    boost::mutex VBO_mutex;
    DetectedBoardDataSimple board_data;

    unsigned int VBO;

    bool board_existence;
    bool points_recieved_first_time;

};

struct VBOBuffer{

    uint32_t lidar_points_number;

    boost::mutex VBO_mutex;

    unsigned int VBO;

    bool points_recieved_first_time;

};

struct sceneObject{

    public:
    std::shared_ptr<Object> object;

    void AddTexture(const unsigned int &texture_index, const std::string &texture_name){
        Object_textures[texture_name] = texture_index;
    }

    void RemoveTexture(const std::string &texture_name){
        Object_textures.erase(texture_name);
    }

    std::map<std::string,bool> cameras_for_render;

    bool texture_transform_support;
    bool auto_render;

    std::map<std::string,unsigned int> Object_textures;
};

struct BoardConfig {
    glm::uvec2 pattern_size;
    float square_length;
    glm::vec2 board_dimension;
    glm::vec2 translation_error;
};

struct LidarDetectedBoardData {
    glm::vec4 gpu_lidar_detected_board_buffer[6] = {
        glm::vec4(0.0f), glm::vec4(0.0f), glm::vec4(0.0f),
        glm::vec4(0.0f), glm::vec4(0.0f), glm::vec4(0.0f)
    };
    float lidar_detected_board_plane_d = 0.0f;
    std::shared_ptr<std::vector<float>> lidar_inliers;
    int32_t lidar_timestamp_sec = 0;
    uint32_t lidar_timestamp_nanosec = 0U;
    
};

struct CameraDetectedBoardData {
    glm::vec4 gpu_camera_detected_board_buffer[6] = {
        glm::vec4(0.0f), glm::vec4(0.0f), glm::vec4(0.0f),
        glm::vec4(0.0f), glm::vec4(0.0f), glm::vec4(0.0f)
    };
    float camera_detected_board_plane_d = 0.0f;
    int32_t camera_timestamp_sec = 0;
    uint32_t camera_timestamp_nanosec = 0U;
};

struct SharedControl {
    //boost::interprocess::interprocess_mutex mtx;
    //boost::interprocess::interprocess_condition cond_finished;

    bool resized;   // flag for resize notification
};

class Scene {

public:


TopicsInfo ipc_topics_names;


//Camera context
GLFWwindow* Camera_context;
unsigned int camera_context_pbo;
//bool virtual_camera_rescaled = false;
bool camera_detected_board_recieved_first_time =false;
//bool camera_recieve_debug_board = false;
bool camera_recieve_data = false;
//boost::mutex CameraSendDataMutex;
//boost::mutex CameraRecieveBoardMutex;
boost::mutex CameraRecieveDataMutex;
std::vector<uint8_t> Camera_cpu_buffer; //reused //std::vector<float>
unsigned int camera_message_size;
unsigned int camera_detected_board_message_size;

//TexBoardDualBuffer CameraDualColorB;
TexBoardBuffer CameraColorB;
bool camera_recieve_debug_board = false;

//Lidar context

GLFWwindow* Lidar_visualize_context;
GLFWwindow* Lidar_inliers_context;
unsigned int lidar_context_ssbo;
uint32_t lidar_context_ssbo_zize = 0;
//bool generating_lidar_buffers = false;
//bool generating_lidar_buffers_finished = false;
bool inliers_recieved_first_time = false;
bool lidar_recieve_debug_board = false;
bool render_ground_truth_normal = false;
bool lidar_detected_board_recieved_first_time = false;
bool view_board = true;
boost::mutex RecalculateLidarMutex;
//boost::mutex LidarSendDataMutex;
boost::mutex LidarRecieveDataMutex;
boost::mutex LidarDrawInliersMutex;
//std::vector<glm::vec3> lidar_inliers_recieved;

unsigned int LidarInliersVAO_main;
unsigned int LidarInliersVBO_main;
VBOBoardBuffer LidarInliersVBO;
DetectedBoardDataSimple RenderFrameLidarBoard;

unsigned int RenderFrameCameraTexture;
DetectedBoardDataSimple RenderFrameCameraBoard;

unsigned int LidarPointsVisualizeVAO_main;
unsigned int LidarPointsVisualizeVBO_main;
VBOBuffer LidarPointsVisualizeVBO;

//main contex
GLFWwindow* main_window;

bool enable_vsync = 1;
//float swap_interval = 1.0f;

ImGuiWindowFlags &windowFlags_global_reference;
ImGuiIO &io_reference;
ImGuiStyle &global_style_reference;
ImGuiStyle global_style_copy;

ImWchar *icons_ranges;

ImFont *font;
ImFont *font_mono;
ImFont *font_mono_offset;
ImFont *icons;

GLFWcursor* arrowCursor;
GLFWcursor* dragCursor;
GLFWcursor* invisible_cursor;

double global_cursor_xpos = 0.0f;
double global_cursor_ypos = 0.0f;

unsigned int window_viewport_current_width = SCR_WIDTH_V;
unsigned int window_viewport_current_height = SCR_HEIGHT_V;
ImVec2 previousImGuiSceneWindowSize = ImVec2(0, 0);
ImVec2 previousPickingImGuiSceneWindowSize = ImVec2(0, 0);


int monitorScreenHeight;
int monitorScreenWidth;

ImVec2 ViewportSize;
ImVec2 ViewportPos;
ImVec2 ViewportMin;
ImVec2 ViewportMax;



bool scene_hovered = false;
bool isDragging = false;

bool capture_mouse = false;
bool keyPressed = false;
bool scroll_callback_installed = false;
bool shift_pressed = false;

std::string object_selected = ""; 

bool isUsingGuizmo = false;

ImGuizmo::OPERATION currentTransformOp = ImGuizmo::TRANSLATE;
ImGuizmo::MODE currentTransformMode = ImGuizmo::WORLD;

bool button_state_vector[6] = {true,false,false,true,false,true};
bool want_to_toggle_proj = false;

std::unique_ptr<ArcBallCamera> viewport_camera;
glm::mat4 viewport_camera_view_matrix;
glm::mat4 viewport_camera_projection_matrix;
float ViewportCamVfov = 45.0f;
float ViewportCamVfovTemp;
bool last_frame_view_fov_input_txt_active = false;

glm::vec3 radialDistortion_coefs;
glm::vec2 tangentialDistortion_coefs;
float VirtualCamVfov = 45.0f;
bool camera_broadcast = false;

float imoguizmo_proj[16];

GLuint dummyVAO;

Shader gridShader;
Shader InfoShader;
Shader OutlineShader;
Shader DrawOutlineShader;
Shader VirtaulCamScreenShader;
Shader CameraDebugBoardShader;

unsigned int viewport_framebuffer;
unsigned int viewport_textureColorbuffer;
unsigned int viewport_rbo;

unsigned int m_info_framebuffer;
unsigned int m_picking_texture;
unsigned int m_picking_depth_rbo;

unsigned int m_mask_framebuffer;
unsigned int m_mask_texture;
//unsigned int m_mask_depth_texture;

glm::mat4 guizmoview_proj;

std::map<std::string,std::shared_ptr<sceneObject>> ObjectsVector;
std::map<std::string,std::shared_ptr<Model>> ModelsVector;
std::map<std::string,std::shared_ptr<Shader>> ShadersVector;
std::map<std::string,std::shared_ptr<Shader_compute>> ComputeShadersVector;

std::shared_ptr<std::vector<unsigned int>> global_uniform_binding_indexes;

unsigned int uboViewportCamera;
unsigned int uboVirtualCamera;
unsigned int uboDebugBoard;

unsigned int ProjViewBindingViewportCamera;

unsigned int ProjViewBindingVirtualCamera;

unsigned int DebugBoardBinding;

glm::vec4 background_scene_color = glm::vec4(0.69f, 0.69f, 0.93f, 1.0f);

Lidar_config Lidar_params;
//uint32_t Lidar_dir_buffer_size;
glm::mat4 normalized_lidar_model = glm::mat4(1.0f);
glm::mat4 inverse_normalized_lidar_model = glm::mat4(1.0f);
bool show_raw_lidar_cloud = true;
bool lidar_broadcast = false;
bool lidar_recieve_debug_cloud = false;
bool lidar_view_debug_cloud = false;
bool lidar_view_limited_cloud = false;

BoardConfig BoardParams;

float lidar_board_area_error = 0.0f;
float lidar_board_angle_error = 0.0f;
float lidar_board_position_error = 0.0f;

float camera_board_area_error = 0.0f;
float camera_board_angle_error = 0.0f;
float camera_board_position_error = 0.0f;

float bet_camlidar_board_angle_error = 0.0f;
float bet_camlidar_board_position_error = 0.0f;

//float camera_detected_board[19];
//float lidar_detected_board[21];

boost::circular_buffer<CameraDetectedBoardData> CameraDetectedBoardHistory;
boost::circular_buffer<LidarDetectedBoardData> LidarDetectedBoardHistory;

//glm::vec4 gpu_camera_detected_board_buffer[6];
//float camera_detected_board_plane_d;
//int32_t camera_timestamp_sec;
//uint32_t camera_timestamp_nanosec;
cv::Size BoardPatternSize;
CameraYamlConfig camera_board_settings;
cv::Mat cam_matrix;
cv::Mat distortion_c;
uint32_t image_pixels_number;
std::vector<cv::Point3f> board_world_space_points;


//glm::vec4 gpu_lidar_detected_board_buffer[6];
//float lidar_detected_board_plane_d;
//int32_t lidar_timestamp_sec;
//uint32_t lidar_timestamp_nanosec;


bool lidar_board_render_mode[4] = {false,false,false,false};
bool camera_board_render_mode[4] = {false,false,false,false};
bool lidar_sample_render_mode[5] = {false,false,false,false,false};
bool camera_sample_render_mode[4] = {false,false,false,false};

//uint32_t fifo_msg_send_size;
uint32_t fifo_lidar_msg_recieve_size;
//uint32_t inliers_msg_recieve_size;
uint32_t lidar_point_cloud_size;
std::vector<Point_XYZIR> lidar_points;

std::vector<glm::vec3> lidar_points_visualize;
//uint32_t lidar_points_visualize_npoints;
uint32_t lidar_points_visualize_max_npoints;
uint32_t lidar_inliers_size_max_npoints;
//boost::mutex  LidarPointsVisualizeMutex;


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

Scene(GLFWwindow *window,const int &screenWidth,const int &screenHeight,
        ImFont *m_font,ImFont *m_font_mono,ImFont *m_font_mono_off,ImFont *m_icons,
        GLFWcursor* m_arrowCursor,
        GLFWcursor* m_dragCursor,
        GLFWcursor* m_invisible_cursor,
        ImWchar *m_icons_ranges,
        ImGuiIO& io,ImGuiStyle& style, ImGuiWindowFlags &m_windowFlags_global_reference,
        std::shared_ptr<std::vector<unsigned int>> m_global_uniform_binding_indexes);

~Scene(){
    JoinLidarCameraThreads();
}

void AddShader(const shaderConfig &shader_cfg);
void RemoveShader(const std::string &shader_name);

void AddModel(const std::string &model_path, const std::string &model_name);
void RemoveModel(const std::string &model_name);

void AddObject(std::shared_ptr<Object> object_ptr,const std::string &object_name);
void RemoveObject(const std::string &object_name);

void InitCameras(const ViewportCamConfig &view_cfg,const VirtualCamConfig &real_cam_cfg);


void Verify_clicked_object();

void RenderStartMainViewport();
void RenderEndMainViewport();
void RenderMainViewportAuto();

void draw_outline();

void draw_grid();

void RenderInfoBuffer();

void Create_viewport_framebuffer();
void Rescale_viewport_framebuffer(int width, int height);
void Create_info_framebuffer();
void Rescale_info_framebuffer(int width, int height);
void Create_mask_framebuffer();
void Rescale_mask_framebuffer(int width, int height);

void draw_real_camera_settings();
void draw_real_camera_viewport();

void draw_statistics_viewport();

void draw_guizmo_toolbar();
void draw_imoguizmo();
void update_viewport_cam_view_matrix_ubo();
void update_viewport_cam_projection_matrix_ubo();

void draw_viewport();
void draw_camera_lidar_matrix();

void draw_lidar_cloud_settings();

void draw_transform_settings();

void draw_calibration_settings();
void draw_calibration_log();

void draw_loaded_camera_params();
void draw_recieved_cam_image_viewport();

void processInput(const float &deltaTime);

//void RecieveCameraBoardAsync();
std::thread RecieveCameraDataAsync;
void RecieveCameraDataAsync_func();
void Get_Camera_Board_Image();
//void RecreateLidarBroadcastFIFO();
//void RecreateLidarInliersFIFO();
//void RecreateCameraFIFOS();
//void InitFIFOS();

void InitLidarInfoSharedMem();
void LoadYamlParams();
void SetBBoxValues();

void LidarDrawPoints();
void LidarCalculateNormalizedModel();
//void RecieveLidarInliersAsync();
std::thread RecieveLidarDataAsync;
void RecieveLidarDataAsync_func();
void Draw_Lidar_Board_Inliers();
void Draw_Lidar_Points_Limited();
void DrawLidarCameraBoard();

void InitLidarCameraThreads();
void JoinLidarCameraThreads();

void find_board_edges(pcl::PointCloud<pcl::PointXYZ>::Ptr side_edge_cloud_input,std::vector<pcl::ModelCoefficients> &edge_lines_side_output);
Eigen::Vector3f line_intersect(line_model_3D &line1, line_model_3D &line2);

private:



};

}

extern std::shared_ptr<LidarCamSim::Scene> main_scene_class;

#endif