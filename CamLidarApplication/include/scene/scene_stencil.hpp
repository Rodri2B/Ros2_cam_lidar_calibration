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

#include <learnopengl/shader.h>

#include <texture_loader.hpp>
#include <object_loader.hpp>

#include <learnopengl/camera_quat.h>

#include <lidar_scan.hpp>

#include <iostream>

#include <vector>
#include <memory>
#include <algorithm>

#include <boost/interprocess/ipc/message_queue.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/mutex.hpp>

#include <thread>

const unsigned int SCR_WIDTH_V = 800;
const unsigned int SCR_HEIGHT_V = 600;

void mouse_callback(GLFWwindow* window, double xposIn, double yposIn);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);

namespace LidarCamSim {

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

class Scene {

public:

//Camera context
GLFWwindow* Camera_context;
unsigned int camera_context_pbo;
bool virtual_camera_rescaled = false;
boost::mutex CameraSendDataMutex;
std::vector<float> Camera_cpu_buffer;
unsigned int camera_message_size;
//Lidar context

GLFWwindow* Lidar_context;
unsigned int lidar_context_ssbo;
uint32_t lidar_context_ssbo_zize = 0;
bool generating_lidar_buffers = false;
bool generating_lidar_buffers_finished = false;
boost::mutex RecalculateLidarMutex;
boost::mutex LidarSendDataMutex;
std::vector<Point_XYZIR> points_broadcast_buffer;
//main contex
GLFWwindow* main_window;

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

std::unique_ptr<VirtualCamera> plumb_bob_cam;

glm::vec3 radialDistortion_coefs;
glm::vec2 tangentialDistortion_coefs;
float VirtualCamVfov = 45.0f;
bool camera_broadcast = false;

float imoguizmo_proj[16];

GLuint dummyVAO;

Shader gridShader;
Shader InfoShader;
Shader OutlineShader;
Shader VirtaulCamScreenShader;

unsigned int viewport_framebuffer;
unsigned int viewport_textureColorbuffer;
unsigned int viewport_rbo;

unsigned int m_info_framebuffer;
unsigned int m_picking_texture;
unsigned int m_picking_depth_tex;


glm::mat4 guizmoview_proj;

std::map<std::string,std::shared_ptr<sceneObject>> ObjectsVector;
std::map<std::string,std::shared_ptr<Model>> ModelsVector;
std::map<std::string,std::shared_ptr<Shader>> ShadersVector;
std::map<std::string,std::shared_ptr<Shader_compute>> ComputeShadersVector;

std::shared_ptr<std::vector<unsigned int>> global_uniform_binding_indexes;

unsigned int uboViewportCamera;
unsigned int uboVirtualCamera;

unsigned int ProjViewBindingViewportCamera;

unsigned int ProjViewBindingVirtualCamera;

glm::vec4 background_scene_color = glm::vec4(0.69f, 0.69f, 0.93f, 1.0f);

std::unique_ptr<SensorScene> LidarObject;
Lidar_config Lidar_params;
uint32_t Lidar_dir_buffer_size;
glm::mat4 normalized_lidar_model = glm::mat4(1.0f);
glm::mat4 inverse_normalized_lidar_model = glm::mat4(1.0f);
bool show_raw_lidar_cloud = true;
bool lidar_broadcast = false;
bool lidar_recieve_debug_cloud = false;
bool lidar_view_debug_cloud = false;

BoardConfig BoardParams;

uint32_t fifo_msg_send_size;
uint32_t fifo_msg_recieve_size;

std::unique_ptr<boost::interprocess::message_queue> send_queue;
std::unique_ptr<boost::interprocess::message_queue> recieve_queue;

std::unique_ptr<boost::interprocess::message_queue> camera_send_queue;

Scene(GLFWwindow *window,const int &screenWidth,const int &screenHeight,
        ImFont *m_font,ImFont *m_font_mono,ImFont *m_font_mono_off,ImFont *m_icons,
        GLFWcursor* m_arrowCursor,
        GLFWcursor* m_dragCursor,
        GLFWcursor* m_invisible_cursor,
        ImWchar *m_icons_ranges,
        ImGuiIO& io,ImGuiStyle& style, ImGuiWindowFlags &m_windowFlags_global_reference,
        std::shared_ptr<std::vector<unsigned int>> m_global_uniform_binding_indexes);

void AddShader(const shaderConfig &shader_cfg);
void RemoveShader(const std::string &shader_name);

void AddModel(const std::string &model_path, const std::string &model_name);
void RemoveModel(const std::string &model_name);

void AddObject(std::shared_ptr<Object> object_ptr,const std::string &object_name);
void RemoveObject(const std::string &object_name);

void InitCameras(const ViewportCamConfig &view_cfg,const VirtualCamConfig &vir_cfg);

void InitBoard(const BoardConfig &board_cfg);

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
void StartRenderVirtual();
void EndRenderVirtual();
void AutoRenderVirtual();

void draw_virtual_camera_settings();
void draw_virtual_camera_viewport();

void draw_guizmo_toolbar();
void draw_imoguizmo();
void update_viewport_cam_view_matrix_ubo();
void update_viewport_cam_projection_matrix_ubo();
void update_virtual_cam_view_matrix_ubo();
void update_virtual_cam_projection_matrix_ubo();

void draw_viewport();
void draw_camera_lidar_matrix();
void draw_lidar_settings();
void draw_board_settings();
void draw_transform_settings();

void processInput(const float &deltaTime);

void InitializeDefaultBoard();

void SendLidarCloud();
void SendLidarCloudAsync();
void SendCameraImageAsync();
void RecreateLidarFIFOS();
void RecreateCameraFIFOS();
void InitFIFOS();
void InitLidar(const Lidar_config &lidar_cfg,const glm::vec3 &Pos,const float &roll,const float &pitch,const float &yaw);
void LidarAddObject(const string &object_name,const bool &visible);
void LidarDrawPoints();
void LidarDoScan(const float &time);
void LidarCalculateNormalizedModel();
//void RecieveLidarProcessedResult();

private:



};

}

extern std::shared_ptr<LidarCamSim::Scene> main_scene_class;

#endif