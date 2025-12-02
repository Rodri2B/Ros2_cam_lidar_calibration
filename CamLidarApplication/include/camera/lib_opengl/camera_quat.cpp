#include "camera_quat.h"


static int MARGIN = 10;

//define camera up vector on the constructor is useless given that it is redefined in the contructor correctly, the initial 
//value is never used then it can be removed

Camera::Camera(unsigned int WindowWidth, unsigned int WindowHeight):  MovementSpeed(SPEED), MouseSensitivity(SENSITIVITY), Zoom(ZOOM)
{
    m_windowWidth  = static_cast<float>(WindowWidth);
    m_windowHeight = static_cast<float>(WindowHeight);
    m_pos          = glm::vec3(0.0f, 0.0f, 0.0f);
    m_target       = glm::vec3(0.0f, 0.0f, 1.0f);
    m_up           = glm::vec3(0.0f, 1.0f, 0.0f);

    Init();
    Update();
}


Camera::Camera(unsigned int WindowWidth, unsigned int WindowHeight, const glm::vec3& Pos, const glm::vec3& Target, const glm::vec3& Up):  MovementSpeed(SPEED), MouseSensitivity(SENSITIVITY), Zoom(ZOOM)
{
    m_windowWidth  = static_cast<float>(WindowWidth);
    m_windowHeight = static_cast<float>(WindowHeight);
    m_pos = Pos;

    //normalize
    m_target = glm::normalize(Target);

    m_up = glm::normalize(Up);

    Init();
    Update();
}

// constructor with vectors
Camera::Camera(unsigned int WindowWidth, unsigned int WindowHeight,glm::vec3 Pos, glm::vec3 up, float yaw, float pitch) : m_target(glm::vec3(0.0f, 0.0f, -1.0f)), MovementSpeed(SPEED), MouseSensitivity(SENSITIVITY), Zoom(ZOOM)
{
    m_windowWidth  = static_cast<float>(WindowWidth);
    m_windowHeight = static_cast<float>(WindowHeight);
    m_pos = Pos;
    m_up = up;
    m_AngleH = -yaw;
    m_AngleV = -pitch;
    
    m_OnUpperEdge = false;
    m_OnLowerEdge = false;
    m_OnLeftEdge  = false;
    m_OnRightEdge = false;

    m_mousePos.x  = m_windowWidth / 2.0f;
    m_mousePos.y  = m_windowHeight / 2.0f;

    Update();
}

Camera::Camera(unsigned int WindowWidth, unsigned int WindowHeight, float posX, float posY, float posZ, float upX, float upY, float upZ, float yaw, float pitch): m_target(glm::vec3(0.0f, 0.0f, -1.0f)), MovementSpeed(SPEED), MouseSensitivity(SENSITIVITY), Zoom(ZOOM)
{
    m_windowWidth  = static_cast<float>(WindowWidth);
    m_windowHeight = static_cast<float>(WindowHeight);
    m_pos = glm::vec3(posX,posY,posZ);
    m_up = glm::vec3(upX,upY,upZ);
    m_AngleH = -yaw;
    m_AngleV = -pitch;
    
    m_OnUpperEdge = false;
    m_OnLowerEdge = false;
    m_OnLeftEdge  = false;
    m_OnRightEdge = false;

    m_mousePos.x  = m_windowWidth / 2.0f;
    m_mousePos.y  = m_windowHeight / 2.0f;

    Update();
}


void Camera::Init()
{
    glm::vec3 HTarget(m_target.x, 0.0, m_target.z);
    HTarget = glm::normalize(HTarget);

    float Angle = glm::degrees(asin(abs(HTarget.z)));
    
    if (HTarget.z >= 0.0f)
    {
        if (HTarget.x >= 0.0f)
        {
            m_AngleH = 360.0f - Angle;
        }
        else
        {
            m_AngleH = 180.0f + Angle;
        }
    }
    else
    {
        if (HTarget.x >= 0.0f)
        {
            m_AngleH = Angle;
        }
        else
        {
            m_AngleH = 180.0f - Angle;
        }
    }

    m_AngleV = -glm::degrees(asin(m_target.y));

    m_OnUpperEdge = false;
    m_OnLowerEdge = false;
    m_OnLeftEdge  = false;
    m_OnRightEdge = false;

    m_mousePos.x  = m_windowWidth / 2.0f;
    m_mousePos.y  = m_windowHeight / 2.0f;
}



void Camera::SetPosition(float x, float y, float z)
{
    m_pos.x = x;
    m_pos.y = y;
    m_pos.z = z;
}


void Camera::ProcessKeyboard(Camera_Movement direction, float deltaTime)
{
    float velocity = MovementSpeed * deltaTime;
    float turn_velocity = 50*MovementSpeed * deltaTime;
    switch (direction) {
    
    case CAMERA_YAW_POS:
        m_AngleH -= turn_velocity;
        Update();
        break;

    case CAMERA_YAW_NEG:
        m_AngleH += turn_velocity;
        Update();
        break;

    case CAMERA_PITCH_POS:
        m_AngleV -= turn_velocity;
        Update();
        break;

    case CAMERA_PITCH_NEG:
        m_AngleV += turn_velocity;
        Update();
        break;

    case FORWARD:
        m_pos += (m_target * velocity);
        break;

    case BACKWARD:
        m_pos -= (m_target * velocity);
        break;

    case LEFT:
        {
            glm::vec3 Left = glm::normalize(glm::cross(m_up,m_target));
            Left *= velocity;
            m_pos += Left;
        }
        break;

    case RIGHT:
        {
            glm::vec3 Right = glm::normalize(glm::cross(m_target,m_up));
            Right *= velocity;
            m_pos += Right;
        }
        break;

    case UP:
        m_pos.y += MovementSpeed;
        break;

    case DOWN:
        m_pos.y -= MovementSpeed;
        break;

    case ACCEL:
        MovementSpeed += 0.1f;
        printf("Speed changed to %f\n", MovementSpeed);
        break;

    case DEACCEL:
        MovementSpeed -= 0.1f;
        if (MovementSpeed < 0.1f) {
            MovementSpeed = 0.1f;
        }
        printf("Speed changed to %f\n", MovementSpeed);
        break;
    }
}

void Camera::ProcessMouseMovement(float x, float y)
{

    if (firstMouse)
    {
        m_mousePos.x = x;
        m_mousePos.y = y;
        firstMouse = false;
    }

    float DeltaX = m_mousePos.x - x;
    float DeltaY = y - m_mousePos.y;

    m_mousePos.x = x;
    m_mousePos.y = y;

    m_AngleH += DeltaX * MouseSensitivity;
    m_AngleV += DeltaY * MouseSensitivity;

    // make sure that when pitch is out of bounds, screen doesn't get flipped
    if (constrainAngleV)
    {
        if (m_AngleV > 89.0f)
            m_AngleV = 89.0f;
        if (m_AngleV < -89.0f)
            m_AngleV = -89.0f;
    }
    else{

        if (DeltaX == 0) {
            if (x <= MARGIN) {
                m_OnLeftEdge = true;
            }
            else if (x >= (m_windowWidth - MARGIN)) {
                m_OnRightEdge = true;
            }
        }
        else {
            m_OnLeftEdge = false;
            m_OnRightEdge = false;
        }

        if (DeltaY == 0) {
            if (y <= MARGIN) {
                m_OnUpperEdge = true;
            }
            else if (y >= (m_windowHeight - MARGIN)) {
                m_OnLowerEdge = true;
            }
        }
        else {
            m_OnUpperEdge = false;
            m_OnLowerEdge = false;
        }

    }

    Update();
}

void Camera::setConstrainAngle(bool set){

    if(set) constrainAngleV = true;
    else{
        constrainAngleV = false;
        m_OnUpperEdge = false;
        m_OnLowerEdge = false;
        m_OnLeftEdge = false;
        m_OnRightEdge = false;
    }
}


// processes input received from a mouse scroll-wheel event. Only requires input on the vertical wheel-axis
void Camera::ProcessMouseScroll(float yoffset)
{
    Zoom -= (float)yoffset;
    if (Zoom < 1.0f)
        Zoom = 1.0f;
    if (Zoom > 45.0f)
        Zoom = 45.0f;
}

void Camera::OnRender(float deltaTime)
{

    bool ShouldUpdate = false;


    if (m_OnLeftEdge) {
        m_AngleH -= 100*EDGE_STEP * deltaTime;
        ShouldUpdate = true;
    }
    else if (m_OnRightEdge) {
        m_AngleH += 100*EDGE_STEP* deltaTime;
        ShouldUpdate = true;
    }

    if (m_OnUpperEdge) {
        if (m_AngleV > -90.0f) {
            m_AngleV -= 100*EDGE_STEP* deltaTime;
            ShouldUpdate = true;
        }
    }
    else if (m_OnLowerEdge) {
        if (m_AngleV < 90.0f) {
           m_AngleV += 100*EDGE_STEP* deltaTime;
           ShouldUpdate = true;
        }
    }

    if (ShouldUpdate) {
        Update();
    }
}

void Camera::Update()
{
    glm::vec3 Yaxis(0.0f, 1.0f, 0.0f);

    // Rotate the view vector by the horizontal angle around the vertical axis
    glm::vec3 View(1.0f, 0.0f, 0.0f);
    View = customquat::rotation_quat(m_AngleH, Yaxis,View);
    View = glm::normalize(View);

    // Rotate the view vector by the vertical angle around the horizontal axis
    glm::vec3 U = glm::normalize(glm::cross(Yaxis,View));

    View = customquat::rotation_quat(m_AngleV, U,View);

    m_target = glm::normalize(View);

    m_up = glm::normalize(glm::cross(m_target,U));
}



glm::mat4 Camera::GetViewMatrix()
{
    return glm::lookAt(m_pos, m_pos + m_target, m_up);

    
}

glm::vec3 Camera::GetCameraPosition(){
    return m_pos;
}

void Camera::ScreenResize(const unsigned int &windowWidth,const unsigned int &windowHeight){


    m_windowWidth = static_cast<float>(windowWidth);
    m_windowHeight = static_cast<float>(windowHeight);

    if (firstMouse)
    {
        m_mousePos.x = m_windowWidth / 2.0f;
        m_mousePos.y = m_windowHeight / 2.0f;
    }

}