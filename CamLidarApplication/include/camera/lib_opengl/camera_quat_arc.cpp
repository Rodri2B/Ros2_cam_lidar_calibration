#include "camera_quat.h"

#define MAX_PIVOT_DISTANCE 20.0F

static int MARGIN = 10;
 

//define camera up vector on the constructor is useless given that it is redefined in the contructor correctly, the initial 
//value is never used then it can be removed

ArcBallCamera::ArcBallCamera(unsigned int WindowWidth, unsigned int WindowHeight):  
MovementSpeed(SPEED), MouseSensitivity(SENSITIVITY), Zoom(ZOOM), ScrollSensitivity(SCROLL_SENSITIVITY), LateralSensitivity(LATERAL_SENSITIVITY)
{
    m_windowWidth  = static_cast<float>(WindowWidth);
    m_windowHeight = static_cast<float>(WindowHeight); 
    m_pivot_pos    = glm::vec3(0.0f, 0.0f, 0.0f);
    m_pivot_distance = 1.0f;
    m_target       = glm::vec3(0.0f, 0.0f, 1.0f);
    m_up           = glm::vec3(0.0f, 1.0f, 0.0f);
    m_right = glm::cross(m_target,m_up);

    m_mousePos.x  = m_windowWidth / 2.0f;
    m_mousePos.y  = m_windowHeight / 2.0f;

    Init();
    Update();
}


ArcBallCamera::ArcBallCamera(unsigned int WindowWidth, unsigned int WindowHeight, const glm::vec3& PivotPos,const float& PivotDistance, const glm::vec3& Target, const glm::vec3& Up):  
MovementSpeed(SPEED), MouseSensitivity(SENSITIVITY), Zoom(ZOOM), ScrollSensitivity(SCROLL_SENSITIVITY), LateralSensitivity(LATERAL_SENSITIVITY)
{
    m_windowWidth  = static_cast<float>(WindowWidth);
    m_windowHeight = static_cast<float>(WindowHeight);
    m_pivot_pos = PivotPos;
    m_pivot_distance = PivotDistance;

    //normalize
    m_target = glm::normalize(Target);

    m_up = glm::normalize(Up);

    m_right = glm::cross(m_target,m_up);

    m_mousePos.x  = m_windowWidth / 2.0f;
    m_mousePos.y  = m_windowHeight / 2.0f;

    Init();
    Update();
}

// constructor with vectors
ArcBallCamera::ArcBallCamera(unsigned int WindowWidth, unsigned int WindowHeight,glm::vec3 PivotPos,const float& PivotDistance, float yaw_y_up, float pitch) : 
MovementSpeed(SPEED), MouseSensitivity(SENSITIVITY), Zoom(ZOOM), ScrollSensitivity(SCROLL_SENSITIVITY), LateralSensitivity(LATERAL_SENSITIVITY)
{
    m_windowWidth  = static_cast<float>(WindowWidth);
    m_windowHeight = static_cast<float>(WindowHeight);
    m_pivot_pos = PivotPos;
    m_pivot_distance = PivotDistance;
    m_AngleH = yaw_y_up;
    m_AngleV = -pitch;

    m_mousePos.x  = m_windowWidth / 2.0f;
    m_mousePos.y  = m_windowHeight / 2.0f;

    Update();
}


void ArcBallCamera::Init()
{
    glm::vec3 HTarget(m_right.x, 0.0, m_right.z);
    HTarget = glm::normalize(HTarget);

    float Angle = glm::degrees(acos(abs(HTarget.x)));
    
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

    m_AngleV = glm::degrees(asin(m_target.y));


}



void ArcBallCamera::SetPivotPosition(float x, float y, float z)
{
    m_pivot_pos.x = x;
    m_pivot_pos.y = y;
    m_pivot_pos.z = z;

    m_pos = glm::vec3(x,y,z) - m_pivot_distance*m_target;

}


void ArcBallCamera::ProcessKeyboard(Camera_Movement direction, float deltaTime)
{
    float velocity = MovementSpeed * deltaTime;
    float turn_velocity = 50*MovementSpeed * deltaTime;
    switch (direction) {
    
    case CAMERA_YAW_POS:
        m_AngleH -= turn_velocity;
        //Update();
        Update_local();
        break;

    case CAMERA_YAW_NEG:
        m_AngleH += turn_velocity;
        //Update();
        Update_local();
        break;

    case CAMERA_PITCH_POS:
        m_AngleV += turn_velocity;
        //Update();
        Update_local();
        break;

    case CAMERA_PITCH_NEG:
        m_AngleV -= turn_velocity;
        //Update();
        Update_local();
        break;

    case FORWARD:
        m_pivot_pos += (m_target * velocity);
        m_pos += (m_target * velocity);
        break;

    case BACKWARD:
        m_pivot_pos -= (m_target * velocity);
        m_pos -= (m_target * velocity);
        break;

    case LEFT:
        {
            glm::vec3 Left = glm::normalize(glm::cross(m_up,m_target));
            Left *= velocity;
            m_pivot_pos += Left;
            m_pos += Left;
        }
        break;

    case RIGHT:
        {
            glm::vec3 Right = glm::normalize(glm::cross(m_target,m_up));
            Right *= velocity;
            m_pivot_pos += Right;
            m_pos += Right;
        }
        break;

    case UP:
        m_pivot_pos.y += MovementSpeed;
        m_pos.y += MovementSpeed;
        break;

    case DOWN:
        m_pivot_pos.y -= MovementSpeed;
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

void ArcBallCamera::ProcessMouseMovement(float x, float y, bool lateral_move)
{

    if (firstMouse)
    {
        m_mousePos.x = x;
        m_mousePos.y = y;
        firstMouse = false;
    }

    float DeltaX = m_mousePos.x - x;
    float DeltaY = m_mousePos.y - y;

    m_mousePos.x = x;
    m_mousePos.y = y;


    if(!lateral_move){
        m_AngleH += DeltaX * MouseSensitivity;
        m_AngleV += DeltaY * MouseSensitivity;
        (fly_mode)?Update_local():Update();
    }
    else
    {   

        glm::vec3 DeltaRight = DeltaX * LateralSensitivity*m_right;
        glm::vec3 DeltaUp = DeltaY * LateralSensitivity*m_up;

        m_pos +=  DeltaRight;
        m_pos -=  DeltaUp;

        m_pivot_pos +=  DeltaRight;
        m_pivot_pos -=  DeltaUp;

    }

    // make sure that when pitch is out of bounds, screen doesn't get flipped
    if (constrainAngleV)
    {
        if (m_AngleV > 89.0f)
            m_AngleV = 89.0f;
        if (m_AngleV < -89.0f)
            m_AngleV = -89.0f;
    }

}

void ArcBallCamera::setConstrainAngle(bool set){

    constrainAngleV = set;
}


// processes input received from a mouse scroll-wheel event. Only requires input on the vertical wheel-axis
void ArcBallCamera::ProcessMouseScroll(float yoffset)
{
    if(!fly_mode){
        m_pivot_distance -= (float)yoffset*ScrollSensitivity;
        if (m_pivot_distance < 0.0f)
            m_pivot_distance = 0.0f;
        if (m_pivot_distance > MAX_PIVOT_DISTANCE)
            m_pivot_distance = MAX_PIVOT_DISTANCE;
        Update();
    }
}

void ArcBallCamera::ProcessKeyboardPivotDistance(Camera_Movement direction, float deltaTime)
{
    
    float velocity = MovementSpeed * deltaTime;

    switch (direction) {

    case FORWARD:
        m_pivot_distance -= (velocity);

        if (m_pivot_distance < 0.0f)
            m_pivot_distance = 0.0f;
        if (m_pivot_distance > MAX_PIVOT_DISTANCE)
            m_pivot_distance = MAX_PIVOT_DISTANCE;

        m_pos = m_pivot_pos - m_pivot_distance*m_target;
        break;

    case BACKWARD:
        m_pivot_distance += (velocity);

        if (m_pivot_distance < 0.0f)
            m_pivot_distance = 0.0f;
        if (m_pivot_distance > MAX_PIVOT_DISTANCE)
            m_pivot_distance = MAX_PIVOT_DISTANCE;

        m_pos = m_pivot_pos - m_pivot_distance*m_target;
        break;

    }
    
}

void ArcBallCamera::SetVectors(const glm::vec3 &right,const glm::vec3 &up,const glm::vec3 &target){

    m_right = right;
    m_up = up;
    m_target = target;
    m_pos = m_pivot_pos - m_pivot_distance*m_target;

    Init();
    
}

void ArcBallCamera::Update()
{
    glm::vec3 Yaxis(0.0f, 1.0f, 0.0f);

    // Rotate the view vector by the horizontal angle around the vertical axis
    glm::vec3 View(0.0f, 0.0f, -1.0f);
    View = customquat::rotation_quat(m_AngleH, Yaxis,View);
    View = glm::normalize(View);

    // Rotate the view vector by the vertical angle around the horizontal axis
    glm::vec3 U = glm::normalize(glm::cross(View,Yaxis)); //right vector

    View = customquat::rotation_quat(m_AngleV, U,View);

    m_right = glm::normalize(U);

    m_target = glm::normalize(View);

    m_up = glm::normalize(glm::cross(U, m_target));

    m_pos = m_pivot_pos - m_pivot_distance*m_target;
}

void ArcBallCamera::Update_local()
{
    glm::vec3 Yaxis(0.0f, 1.0f, 0.0f);

    // Rotate the view vector by the horizontal angle around the vertical axis
    glm::vec3 View(0.0f, 0.0f, -1.0f);
    View = customquat::rotation_quat(m_AngleH, Yaxis,View);
    View = glm::normalize(View);

    // Rotate the view vector by the vertical angle around the horizontal axis
    glm::vec3 U = glm::normalize(glm::cross(View,Yaxis)); //right vector

    View = customquat::rotation_quat(m_AngleV, U,View);

    m_right = glm::normalize(U);

    m_target = glm::normalize(View);

    m_up = glm::normalize(glm::cross(U, m_target));

    m_pivot_pos = m_pos + m_pivot_distance*m_target;
}

glm::mat4 ArcBallCamera::GetViewMatrix()
{
    //return glm::lookAt(m_pos, m_pivot_pos, m_up);
    return glm::lookAt(m_pos, m_target + m_pos, m_up);

    
}

glm::vec3 ArcBallCamera::GetCameraPosition(){
    return m_pos;
}

glm::vec3 ArcBallCamera::GetCameraPivotPosition(){
    return m_pivot_pos;
}

void ArcBallCamera::ScreenResize(const unsigned int &windowWidth,const unsigned int &windowHeight){


    m_windowWidth = static_cast<float>(windowWidth);
    m_windowHeight = static_cast<float>(windowHeight);

    if (firstMouse)
    {
        m_mousePos.x = m_windowWidth / 2.0f;
        m_mousePos.y = m_windowHeight / 2.0f;
    }

}