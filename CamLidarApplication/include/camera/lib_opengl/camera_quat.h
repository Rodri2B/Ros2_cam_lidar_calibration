#ifndef CAMERA_QUAT_H
#define CAMERA_QUAT_H

#include "quaternion_custom.h"
//#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#ifndef GLM_ENABLE_EXPERIMENTAL
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/matrix_transform_2d.hpp>
#include <glm/gtx/euler_angles.hpp>
#endif

#include <lib_opengl/shader.h>

// Defines several possible options for camera movement. Used as abstraction to stay away from window-system specific input methods
enum Camera_Movement {
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT,
    CAMERA_YAW_POS,
    CAMERA_YAW_NEG,
    CAMERA_PITCH_POS,
    CAMERA_PITCH_NEG,
    UP,
    DOWN,
    ACCEL,
    DEACCEL
};

// Default camera values
const float YAW         = -90.0f;
const float PITCH       =  0.0f;
const float SPEED       =  2.5f;
const float SENSITIVITY =  0.3f;
const float SCROLL_SENSITIVITY = 0.5f;
const float LATERAL_SENSITIVITY = 0.006f;
const float ZOOM        =  45.0f;

class Camera
{
public:

    Camera(unsigned int WindowWidth, unsigned int WindowHeight);

    Camera(unsigned int WindowWidth, unsigned int WindowHeight, const glm::vec3& Pos, const glm::vec3& Target, const glm::vec3& Up);

    Camera(unsigned int WindowWidth, unsigned int WindowHeight,glm::vec3 Pos = glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f), float yaw = YAW, float pitch = PITCH);

    Camera(unsigned int WindowWidth, unsigned int WindowHeight, float posX, float posY, float posZ, float upX, float upY, float upZ, float yaw = YAW, float pitch = PITCH);

    void SetPosition(float x, float y, float z);

    void ProcessKeyboard(Camera_Movement direction, float deltaTime);

    void ProcessMouseMovement(float x, float y);

    void OnRender(float deltaTime);

    glm::mat4 GetViewMatrix();

    glm::vec3 GetCameraPosition();

    void setConstrainAngle(bool set);

    void ProcessMouseScroll(float yoffset);

    void ScreenResize(const unsigned int &windowWidth,const unsigned int &windowHeight);

    bool firstMouse = true;
    float MouseSensitivity = 0.1f;
    float MovementSpeed = 2.5f;
    float EDGE_STEP = 1.0f;

    float m_windowWidth;
    float m_windowHeight;

    float Zoom = 45.0f;

private:

    void Init();
    void Update();

    glm::vec3 m_pos;
    glm::vec3 m_target;
    glm::vec3 m_up;

    float m_AngleH;
    float m_AngleV;

    float delta_m_AngleH = 0;
    float delta_m_AngleV = 0;

    bool m_OnUpperEdge = false;
    bool m_OnLowerEdge = false;
    bool m_OnLeftEdge = false;
    bool m_OnRightEdge = false;

    GLboolean constrainAngleV = true;


    glm::vec2 m_mousePos;

};

class ArcBallCamera
{
public:

    ArcBallCamera(unsigned int WindowWidth, unsigned int WindowHeight);

    ArcBallCamera(unsigned int WindowWidth, unsigned int WindowHeight, const glm::vec3& PivotPos,const float& PivotDistance, const glm::vec3& Target, const glm::vec3& Up);

    ArcBallCamera(unsigned int WindowWidth, unsigned int WindowHeight,glm::vec3 PivotPos,const float& PivotDistance, float yaw_y_up = YAW, float pitch = PITCH);
    
    void SetPivotPosition(float x, float y, float z);

    void ProcessKeyboard(Camera_Movement direction, float deltaTime);

    void ProcessMouseMovement(float x, float y,bool lateral_move);

    glm::mat4 GetViewMatrix();

    glm::vec3 GetCameraPosition();
    glm::vec3 GetCameraPivotPosition();

    void setConstrainAngle(bool set);

    void ProcessMouseScroll(float yoffset);

    void ProcessKeyboardPivotDistance(Camera_Movement direction, float deltaTime);

    void SetVectors(const glm::vec3 &right,const glm::vec3 &up,const glm::vec3 &target);

    void ScreenResize(const unsigned int &windowWidth,const unsigned int &windowHeight);

    bool firstMouse = true;
    float MouseSensitivity = 0.3f;
    float ScrollSensitivity = 0.5f;
    float LateralSensitivity = 0.006f;
    float MovementSpeed = 2.5f;
    float EDGE_STEP = 1.0f;

    float m_windowWidth;
    float m_windowHeight;

    float Zoom = 45.0f;

    float m_pivot_distance;

    glm::vec3 m_pos;
    glm::vec3 m_pivot_pos;

    bool fly_mode = false;

private:

    void Init();
    void Update();
    void Update_local();

    glm::vec3 m_target;
    glm::vec3 m_up;
    glm::vec3 m_right;

    float m_AngleH;
    float m_AngleV;

    float delta_m_AngleH = 0;
    float delta_m_AngleV = 0;

    GLboolean constrainAngleV = true;


    glm::vec2 m_mousePos;

};


class VirtualCamera
{
public:

    VirtualCamera(unsigned int WindowWidth, unsigned int WindowHeight, float Render_Ratio, Shader &screenShader);

    VirtualCamera(unsigned int WindowWidth, unsigned int WindowHeight, float Render_Ratio, Shader &screenShader, const glm::vec3& Pos, const glm::vec3& Target, const glm::vec3& Up);

    VirtualCamera(unsigned int WindowWidth, unsigned int WindowHeight, float Render_Ratio, Shader &screenShader, glm::vec3 Pos, float yaw_y_up = YAW, float pitch = PITCH,float roll = 0.0f);
    
    void SetPosition(const float &x,const float &y,const float &z);

    glm::mat4 GetViewMatrix();

    glm::vec3 GetCameraPosition();
    glm::vec3 GetCameraRotation();

    void setConstrainAngle(bool set);

    void SetVectors(const glm::vec3 &right,const glm::vec3 &up,const glm::vec3 &target);

    void ScreenResize(const unsigned int &windowWidth,const unsigned int &windowHeight);

    glm::mat4 GetCameraMatrix();
    void CalculateIntrisicMatrices(const float &V_fov);
    void RecalculateIntrisicMatrices();
    void RecalculateCamera(const glm::mat4 &transform);
    void RecalculateCameraNormalize(const glm::mat4 &transform);
    void AddCameraTransform(const glm::mat4 &transform);
    void AddCameraTranslate(const glm::mat4 &transform);
    void AddCameraRotate(const glm::mat4 &transform);
    void StartRender();
    void FinishRender();
    void GenarateImage();

    void setDistortioncoeffs(const glm::vec3 &s_radialDistortion,const glm::vec2 &s_tangentialDistortion);
    void resubmitDistortioncoeffs();

    void rescale_camera_resolution(const float &new_width,const float &new_height,const float &new_Ratio);
    void rescale_camera_resolution(const float &new_width,const float &new_height);
    void rescale_camera_resolution(const float &new_Ratio);
    void rescale_camera_framebuffer();
    void create_camera_framebuffer();

    float m_windowWidth_previous;
    float m_windowHeight_previous;
    float RenderRatio_previous;

    float Vfov = 45.0f;

    glm::vec2 pixel_size;
    
    float tile_size = 16.0f;

    float borderColor[4] = { 0.0f, 0.0f, 0.0f, 1.0f };
    float backgroundColor[4] = { 0.2f, 0.3f, 0.3f, 1.0f };

    unsigned int Cameraframebuffer;
    unsigned int CameratextureColorbuffer;
    unsigned int CameratextureFinalColorbuffer;
    unsigned int Camera_rbo;

    unsigned int Camera_intermed_framebuffer;

    unsigned int imGenbuffer;
    unsigned int undistortedColorbuffer;

    unsigned int quadDummyVAO;

    float m_windowWidth;
    float m_windowHeight;
    float RenderRatio;

    glm::mat4 camera_matrix = glm::mat4(1.0f);
    glm::mat4 projection_matrix = glm::mat4(1.0f);

    glm::mat4 camera_intrinsic = glm::mat4(1.0f);

    glm::vec3 radialDistortionParams = glm::vec3(0.0f);
    glm::vec2 tangentialDistortionParams = glm::vec2(0.0f);

    Shader &quadShader;

    glm::vec3 m_pos;

private:

    void Init();
    void Update();

    glm::vec3 m_target;
    glm::vec3 m_up;
    glm::vec3 m_right;

    float m_AngleH;
    float m_AngleV;
    float m_AngleRoll;

    GLboolean constrainAngleV = true;


};

#endif  