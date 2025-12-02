#ifndef CAMERA_QUAT_H
#define CAMERA_QUAT_H

#include "quaternion_custom.h"
//#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>



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
const float SENSITIVITY =  0.1f;
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

    void setConstrainAngle(bool set);

    void ProcessMouseScroll(float yoffset);

    bool firstMouse = true;
    float MouseSensitivity = 0.1f;
    float MovementSpeed = 2.5f;
    float EDGE_STEP = 1.0f;

    float m_windowWidth;
    float m_windowHeight;

    float Zoom = 45.0f;

    unsigned int iterations = 5;

private:

    void Init();
    void Update();

    glm::vec3 m_pos;
    glm::vec3 m_target;
    glm::vec3 m_up;

    float m_AngleH;
    float m_AngleV;

    float m_AngleH_old;
    float m_AngleV_old;

    float displace[3] = {0,0,0};

    bool m_OnUpperEdge = false;
    bool m_OnLowerEdge = false;
    bool m_OnLeftEdge = false;
    bool m_OnRightEdge = false;

    GLboolean constrainAngleV = true;


    glm::vec2 m_mousePos;

    bool use = true;
};

#endif  