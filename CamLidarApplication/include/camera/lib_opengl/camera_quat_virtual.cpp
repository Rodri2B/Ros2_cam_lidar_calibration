#include "camera_quat.h"

//define camera up vector on the constructor is useless given that it is redefined in the contructor correctly, the initial 
//value is never used then it can be removed

VirtualCamera::VirtualCamera(unsigned int WindowWidth, unsigned int WindowHeight, float Render_Ratio, Shader &screenShader)
: quadShader(screenShader)
{
    m_windowWidth  = static_cast<float>(WindowWidth);
    m_windowHeight = static_cast<float>(WindowHeight); 
    RenderRatio = Render_Ratio;

    m_pos = glm::vec3(0.0f,0.0f,-1.0f);

    m_target       = glm::vec3(0.0f, 0.0f, 1.0f);
    m_up           = glm::vec3(0.0f, 1.0f, 0.0f);
    m_right = glm::cross(m_target,m_up);

    Init();
    //Update();
}


VirtualCamera::VirtualCamera(unsigned int WindowWidth, unsigned int WindowHeight, float Render_Ratio, Shader &screenShader ,const glm::vec3& Pos, const glm::vec3& Target, const glm::vec3& Up)
: quadShader(screenShader)
{
    m_windowWidth  = static_cast<float>(WindowWidth);
    m_windowHeight = static_cast<float>(WindowHeight);
    RenderRatio = Render_Ratio;

    m_pos = Pos;

    //normalize
    m_target = glm::normalize(Target);

    m_up = glm::normalize(Up);

    m_right = glm::cross(m_target,m_up);

    Init();
   //Update();
}

// constructor with vectors
VirtualCamera::VirtualCamera(unsigned int WindowWidth, unsigned int WindowHeight, float Render_Ratio, Shader &screenShader, glm::vec3 Pos, float yaw_y_up, float pitch,float roll)
: quadShader(screenShader)
{
    m_windowWidth  = static_cast<float>(WindowWidth);
    m_windowHeight = static_cast<float>(WindowHeight);

    RenderRatio = Render_Ratio;

    m_pos = Pos;

    m_AngleH = yaw_y_up;
    m_AngleV = pitch;
    m_AngleRoll = roll;

    camera_matrix[3] = glm::vec4(m_pos,1.0f);

    Update();
}


void VirtualCamera::Init()
{
    
    //if(m_up.y >= 0.0f)
    //    m_AngleV = glm::degrees(asin(m_target.y));
    //else{
    //    if(m_target.y >= 0.0f){
    //        m_AngleV = 180.0f - glm::degrees(asin(m_target.y));
    //    }else {
    //        m_AngleV = -180.0f - glm::degrees(asin(m_target.y));
    //    }
    //}

    camera_matrix[0] = glm::vec4(-m_right,0.0f);
    camera_matrix[1] = glm::vec4(m_up,0.0f);
    camera_matrix[2] = glm::vec4(m_target,0.0f);
    camera_matrix[3] = glm::vec4(m_pos,1.0f);

    glm::extractEulerAngleZXY(camera_matrix, m_AngleRoll, m_AngleV, m_AngleH);
    m_AngleRoll = glm::degrees(m_AngleRoll);
    m_AngleV = glm::degrees(m_AngleV);
    m_AngleH = glm::degrees(m_AngleH);

}



void VirtualCamera::SetPosition(const float &x,const float &y,const float &z)
{
    m_pos.x = x;
    m_pos.y = y;
    m_pos.z = z;

    camera_matrix[3] = glm::vec4(m_pos,1.0f);

}



void VirtualCamera::setConstrainAngle(bool set){

    constrainAngleV = set;
}


void VirtualCamera::SetVectors(const glm::vec3 &right,const glm::vec3 &up,const glm::vec3 &target){

    m_right = right;
    m_up = up;
    m_target = target;

    camera_matrix[0] = glm::vec4(-m_right,0.0f);
    camera_matrix[1] = glm::vec4(m_up,0.0f);
    camera_matrix[2] = glm::vec4(m_target,0.0f);
    
}

void VirtualCamera::Update()
{
    glm::vec3 Yaxis(0.0f, 1.0f, 0.0f);

    // Rotate the view vector by the horizontal angle around the vertical axis
    glm::vec3 View(0.0f, 0.0f, 1.0f);
    View = customquat::rotation_quat(m_AngleH, Yaxis,View);
    View = glm::normalize(View);

    // Rotate the view vector by the vertical angle around the horizontal axis
    glm::vec3 U = glm::normalize(glm::cross(View,Yaxis)); //right vector

    View = customquat::rotation_quat(m_AngleV, U,View);

    //m_right = glm::normalize(U);

    m_target = glm::normalize(View);

    m_up = glm::normalize(glm::cross(U, m_target));

    //m_right = customquat::rotation_quat(m_AngleRoll, m_target,m_right);

    m_right = glm::normalize(customquat::rotation_quat(m_AngleRoll, m_target,U));

    //m_up = customquat::rotation_quat(m_AngleRoll, m_target,m_up);

    m_up = glm::normalize(customquat::rotation_quat(m_AngleRoll, m_target,m_up));

    camera_matrix[0] = glm::vec4(-m_right,0.0f);
    camera_matrix[1] = glm::vec4(m_up,0.0f);
    camera_matrix[2] = glm::vec4(m_target,0.0f);
    
}

glm::mat4 VirtualCamera::GetViewMatrix()
{
    //return glm::lookAt(m_pos, m_pivot_pos, m_up);
    return glm::lookAt(m_pos, m_target + m_pos, m_up);

    
}

glm::mat4 VirtualCamera::GetCameraMatrix()
{
    //return glm::lookAt(m_pos, m_pivot_pos, m_up);
    return camera_matrix;

    
}

void VirtualCamera::CalculateIntrisicMatrices(const float &V_fov)
{
    //return glm::lookAt(m_pos, m_pivot_pos, m_up);
    Vfov = V_fov;
    float aspect_ratio = m_windowWidth/m_windowHeight;
    float alpha = m_windowWidth/(2.0f*tan(glm::radians(Vfov/2.0f))*aspect_ratio);
    float betha = m_windowHeight/(2.0f*tan(glm::radians(Vfov/2.0f)));
    float cx = m_windowWidth/2.0f;
    float cy = m_windowHeight/2.0f;

    pixel_size = glm::vec2(1.0f/alpha,1.0f/betha);

    float conditioned_Vfov = 2.0f*atan(tan(glm::radians(Vfov/2.0f))*RenderRatio);
    projection_matrix = glm::perspective(conditioned_Vfov, aspect_ratio, 0.1f, 100.0f);


    camera_intrinsic[0] = glm::vec4(alpha,0.0f,0.0f,0.0f);
    camera_intrinsic[1] = glm::vec4(0.0f,betha,0.0f,0.0f);
    camera_intrinsic[2] = glm::vec4(cx,cy,1.0f,0.0f);
    camera_intrinsic[3] = glm::vec4(0.0f,0.0f,0.0f,1.0f);

    
}

void VirtualCamera::RecalculateIntrisicMatrices()
{
    float aspect_ratio = m_windowWidth/m_windowHeight;
    float alpha = m_windowWidth/(2.0f*tan(glm::radians(Vfov/2.0f))*aspect_ratio);
    float betha = m_windowHeight/(2.0f*tan(glm::radians(Vfov/2.0f)));
    float cx = m_windowWidth/2.0f;
    float cy = m_windowHeight/2.0f;

    pixel_size = glm::vec2(1.0f/alpha,1.0f/betha);

    float conditioned_Vfov = 2.0f*atan(tan(glm::radians(Vfov/2.0f))*RenderRatio);
    projection_matrix = glm::perspective(conditioned_Vfov, aspect_ratio, 0.1f, 100.0f);


    camera_intrinsic[0] = glm::vec4(alpha,0.0f,0.0f,0.0f);
    camera_intrinsic[1] = glm::vec4(0.0f,betha,0.0f,0.0f);
    camera_intrinsic[2] = glm::vec4(cx,cy,1.0f,0.0f);
    camera_intrinsic[3] = glm::vec4(0.0f,0.0f,0.0f,1.0f);


    
}

void VirtualCamera::RecalculateCamera(const glm::mat4 &transform)
{

    m_right = -glm::vec3(transform[0]);
    m_up = glm::vec3(transform[1]);
    m_target = glm::vec3(transform[2]);
    m_pos = glm::vec3(transform[3]);

    camera_matrix = transform;

    glm::extractEulerAngleZXY(camera_matrix, m_AngleRoll, m_AngleV, m_AngleH);
    m_AngleRoll = glm::degrees(m_AngleRoll);
    m_AngleV = glm::degrees(m_AngleV);
    m_AngleH = glm::degrees(m_AngleH);
    
}

void VirtualCamera::RecalculateCameraNormalize(const glm::mat4 &transform)
{
    camera_matrix[0] = glm::normalize(transform[0]);
    camera_matrix[1] = glm::normalize(transform[1]);
    camera_matrix[2] = glm::normalize(transform[2]);
    camera_matrix[3] = transform[3];

    m_right = -glm::vec3(camera_matrix[0]);
    m_up = glm::vec3(camera_matrix[1]);
    m_target = glm::vec3(camera_matrix[2]);
    m_pos = glm::vec3(camera_matrix[3]);


    glm::extractEulerAngleZXY(camera_matrix, m_AngleRoll, m_AngleV, m_AngleH);
    m_AngleRoll = glm::degrees(m_AngleRoll);
    m_AngleV = glm::degrees(m_AngleV);
    m_AngleH = glm::degrees(m_AngleH);
    
}

void VirtualCamera::AddCameraTransform(const glm::mat4 &transform)
{

    camera_matrix = transform*camera_matrix;

    m_right = -glm::vec3(camera_matrix[0]);
    m_up = glm::vec3(camera_matrix[1]);
    m_target = glm::vec3(camera_matrix[2]);
    m_pos = glm::vec3(camera_matrix[3]);

    glm::extractEulerAngleZXY(camera_matrix, m_AngleRoll, m_AngleV, m_AngleH);
    m_AngleRoll = glm::degrees(m_AngleRoll);
    m_AngleV = glm::degrees(m_AngleV);
    m_AngleH = glm::degrees(m_AngleH);
    
}

void VirtualCamera::AddCameraTranslate(const glm::mat4 &transform)
{

    camera_matrix[3][0] += transform[3][0];
    camera_matrix[3][1] += transform[3][1];
    camera_matrix[3][2] += transform[3][2];

    m_right = -glm::vec3(camera_matrix[0]);
    m_up = glm::vec3(camera_matrix[1]);
    m_target = glm::vec3(camera_matrix[2]);
    m_pos = glm::vec3(camera_matrix[3]);

    glm::extractEulerAngleZXY(camera_matrix, m_AngleRoll, m_AngleV, m_AngleH);
    m_AngleRoll = glm::degrees(m_AngleRoll);
    m_AngleV = glm::degrees(m_AngleV);
    m_AngleH = glm::degrees(m_AngleH);
    
}

void VirtualCamera::AddCameraRotate(const glm::mat4 &transform)
{
    glm::vec4 camera_pos = camera_matrix[3];
    camera_matrix[3] = glm::vec4(0.0f,0.0f,0.0f,1.0f);

    camera_matrix = transform*camera_matrix;
    camera_matrix[3] = camera_pos;

    m_right = -glm::vec3(camera_matrix[0]);
    m_up = glm::vec3(camera_matrix[1]);
    m_target = glm::vec3(camera_matrix[2]);
    m_pos = glm::vec3(camera_matrix[3]);

    glm::extractEulerAngleZXY(camera_matrix, m_AngleRoll, m_AngleV, m_AngleH);
    m_AngleRoll = glm::degrees(m_AngleRoll);
    m_AngleV = glm::degrees(m_AngleV);
    m_AngleH = glm::degrees(m_AngleH);
    
}

glm::vec3 VirtualCamera::GetCameraPosition(){
    return m_pos;
}

glm::vec3 VirtualCamera::GetCameraRotation(){
    return glm::vec3(m_AngleRoll,m_AngleV,m_AngleH);
}

void VirtualCamera::setDistortioncoeffs(const glm::vec3 &s_radialDistortion,const glm::vec2 &s_tangentialDistortion){

    radialDistortionParams = s_radialDistortion;
    tangentialDistortionParams = s_tangentialDistortion;    
    quadShader.use();

    quadShader.setVec3("radialDistortionParams", radialDistortionParams);
    quadShader.setVec2("tangentialDistortionParams", tangentialDistortionParams);

}

void VirtualCamera::resubmitDistortioncoeffs(){
    
    quadShader.use();

    quadShader.setVec3("radialDistortionParams", radialDistortionParams);
    quadShader.setVec2("tangentialDistortionParams", tangentialDistortionParams);

}

void VirtualCamera::StartRender(){

        glBindFramebuffer(GL_FRAMEBUFFER, Cameraframebuffer);
        glViewport(0, 0, static_cast<unsigned int>(m_windowWidth*RenderRatio), static_cast<unsigned int>(m_windowHeight*RenderRatio));
        //GLenum drawBuffers[2] = { GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1 };
        //glDrawBuffers(2, drawBuffers);
        glClearColor(backgroundColor[0], backgroundColor[1], backgroundColor[2], backgroundColor[3]);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
}

void VirtualCamera::FinishRender(){
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void VirtualCamera::GenarateImage(){
    glBindFramebuffer(GL_READ_FRAMEBUFFER, Cameraframebuffer);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, Camera_intermed_framebuffer);

    //glFramebufferTexture2D(
    //GL_DRAW_FRAMEBUFFER,
    //GL_COLOR_ATTACHMENT1,
    //GL_TEXTURE_2D,
    //0, 0
    //);

    glReadBuffer(GL_COLOR_ATTACHMENT0); // Read from color_attachment0 of framebuffer
    glDrawBuffer(GL_COLOR_ATTACHMENT0);  // Write to color_attachment0 of intermediateFBO

    //GLenum blitBufs[1] = { GL_COLOR_ATTACHMENT0 };
    //glDrawBuffers(1, blitBufs);

    unsigned int width = static_cast<unsigned int>(m_windowWidth*RenderRatio);
    unsigned int height = static_cast<unsigned int>(m_windowHeight*RenderRatio);

    glBlitFramebuffer(0, 0, width, height, 0, 0, width, height, GL_COLOR_BUFFER_BIT, GL_NEAREST);

    //glFramebufferTexture2D(
    //GL_FRAMEBUFFER,
    //GL_COLOR_ATTACHMENT1,
    //GL_TEXTURE_2D,
    //CameratextureFinalColorbuffer,
    //0
    //);

    glBindFramebuffer(GL_FRAMEBUFFER, imGenbuffer);

    //GLenum drawBuffers[2] = { GL_COLOR_ATTACHMENT0,GL_COLOR_ATTACHMENT1};
    //glDrawBuffers(2, drawBuffers);

    //GLenum drawBuffers[] = { GL_NONE, GL_COLOR_ATTACHMENT1 };
    //glDrawBuffers(2, drawBuffers);

    //glReadBuffer(GL_COLOR_ATTACHMENT0);
    //glDrawBuffer(GL_COLOR_ATTACHMENT1);
    

    glViewport(0, 0, static_cast<unsigned int>(m_windowWidth), static_cast<unsigned int>(m_windowHeight));

    glClearBufferfv(GL_COLOR, 0, borderColor);

    glDisable(GL_DEPTH_TEST);
    //glDisable(GL_CULL_FACE);
    //glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, undistortedColorbuffer);

    //draw

    quadShader.use();
    /////////////////////////
    //unsigned int triangle_num = 2U*(static_cast<unsigned int>(RenderRatio*m_windowWidth)/static_cast<unsigned int>(tile_size))*
    //                               (static_cast<unsigned int>(RenderRatio*m_windowHeight)/static_cast<unsigned int>(tile_size));

    glm::uvec2 tile_quantity(static_cast<unsigned int>(std::ceil(std::floor(RenderRatio*m_windowWidth)/tile_size)),
                             static_cast<unsigned int>(std::ceil(std::floor(RenderRatio*m_windowHeight)/tile_size)));

    glm::vec2 tile_step(tile_size/(0.5f*m_windowWidth),tile_size/(0.5f*m_windowHeight));

    glm::vec2 start_pos((m_windowWidth-tile_size*static_cast<float>(tile_quantity.x))/m_windowWidth  -1.0f,
                        (m_windowHeight-tile_size*static_cast<float>(tile_quantity.y))/m_windowHeight  -1.0f);

    glm::vec2 tile_step_tex(tile_size/(std::floor(RenderRatio*m_windowWidth)),tile_size/(std::floor(RenderRatio*m_windowHeight)));

    glm::vec2 start_pos_tex((0.5f*(std::floor(RenderRatio*m_windowWidth)-tile_size*static_cast<float>(tile_quantity.x)))/std::floor(RenderRatio*m_windowWidth),
                            (0.5f*(std::floor(RenderRatio*m_windowHeight)-tile_size*static_cast<float>(tile_quantity.y)))/std::floor(RenderRatio*m_windowHeight));

    glm::vec2 pixel_scale(m_windowWidth*0.5f*pixel_size.x,m_windowHeight*0.5f*pixel_size.y);

    unsigned int triangle_num = 2U*tile_quantity.x*tile_quantity.y;
    
    quadShader.setVec2("start_pos",start_pos);
    quadShader.setVec2("start_pos_tex",start_pos_tex);
    quadShader.setVec2("tile_step",tile_step);
    quadShader.setVec2("tile_step_tex",tile_step_tex);
    quadShader.setVec2("scale_to_meters",pixel_scale);

    quadShader.setUint("tile_quantity_x",tile_quantity.x);
    quadShader.setUint("tile_quantity_y",tile_quantity.y);

    ///////////////////////////
    //float tex_coord_offset = (1.0f-1.0f/RenderRatio)/2.0f;

    //quadShader.setFloat("offset",tex_coord_offset);
    quadShader.setInt("screenTexture", 0);
    
    glBindVertexArray(quadDummyVAO);
    //glDrawArrays(GL_TRIANGLES, 0, 6);
    glDrawArrays(GL_TRIANGLES, 0, 3U*triangle_num);

    glBindVertexArray(0);

    //glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    //glClearBufferfv(GL_COLOR, 1, backgroundColor);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}


void VirtualCamera::create_camera_framebuffer(){

    // framebuffer configuration
    // -------------------------
    glGenFramebuffers(1, &Cameraframebuffer);
    glBindFramebuffer(GL_FRAMEBUFFER, Cameraframebuffer);
    // create a color attachment texture
    glGenTextures(1, &CameratextureColorbuffer);
    glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, CameratextureColorbuffer);
    float width = m_windowWidth*RenderRatio;
    float height = m_windowHeight*RenderRatio;
    glTexImage2DMultisample(GL_TEXTURE_2D_MULTISAMPLE, 4, GL_RGB, static_cast<unsigned int>(width), static_cast<unsigned int>(height), GL_TRUE);
    //glTexParameterfv(GL_TEXTURE_2D_MULTISAMPLE, GL_TEXTURE_BORDER_COLOR, borderColor);
    //glTexParameteri(GL_TEXTURE_2D_MULTISAMPLE, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    //glTexParameteri(GL_TEXTURE_2D_MULTISAMPLE, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    //glTexParameteri(GL_TEXTURE_2D_MULTISAMPLE, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    //glTexParameteri(GL_TEXTURE_2D_MULTISAMPLE, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D_MULTISAMPLE, CameratextureColorbuffer, 0);
    
    // create a renderbuffer object for depth and stencil attachment (we won't be sampling these)
    glGenRenderbuffers(1, &Camera_rbo);
    glBindRenderbuffer(GL_RENDERBUFFER, Camera_rbo);
    glRenderbufferStorageMultisample(GL_RENDERBUFFER, 4, GL_DEPTH24_STENCIL8, static_cast<unsigned int>(width), static_cast<unsigned int>(height));
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, Camera_rbo); // now actually attach it
    // now that we actually created the framebuffer and added all attachments we want to check if it is actually complete now
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        std::cout << "ERROR::FRAMEBUFFER:: Framebuffer is not complete!" << std::endl;
    glBindFramebuffer(GL_FRAMEBUFFER, 0);


    glGenFramebuffers(1, &Camera_intermed_framebuffer);
    glBindFramebuffer(GL_FRAMEBUFFER, Camera_intermed_framebuffer);

    glGenTextures(1, &undistortedColorbuffer);
    glBindTexture(GL_TEXTURE_2D, undistortedColorbuffer);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, static_cast<unsigned int>(width), static_cast<unsigned int>(height), 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
    glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, borderColor);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, undistortedColorbuffer, 0);

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        std::cout << "ERROR::FRAMEBUFFER:: Framebuffer is not complete!" << std::endl;
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    // framebuffer configuration
    // -------------------------
    glGenFramebuffers(1, &imGenbuffer);
    glBindFramebuffer(GL_FRAMEBUFFER, imGenbuffer);
    // create a color attachment texture
    // create a renderbuffer object for depth and stencil attachment (we won't be sampling these)

    glGenTextures(1, &CameratextureFinalColorbuffer);
    glBindTexture(GL_TEXTURE_2D, CameratextureFinalColorbuffer);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, static_cast<unsigned int>(m_windowWidth), static_cast<unsigned int>(m_windowHeight), 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, CameratextureFinalColorbuffer, 0);

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        std::cout << "ERROR::FRAMEBUFFER:: Framebuffer is not complete!" << std::endl;
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    glGenVertexArrays(1, &quadDummyVAO);

    m_windowWidth_previous = m_windowWidth;
    m_windowHeight_previous = m_windowHeight;
    RenderRatio_previous = RenderRatio;
    
}

void VirtualCamera::rescale_camera_framebuffer()
{   

    glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, CameratextureColorbuffer);
    float width = m_windowWidth*RenderRatio;
    float height = m_windowHeight*RenderRatio;
    glTexImage2DMultisample(GL_TEXTURE_2D_MULTISAMPLE, 4, GL_RGB, static_cast<unsigned int>(width), static_cast<unsigned int>(height), GL_TRUE);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D_MULTISAMPLE, CameratextureColorbuffer, 0);

    glBindRenderbuffer(GL_RENDERBUFFER, Camera_rbo);
    glRenderbufferStorageMultisample(GL_RENDERBUFFER, 4, GL_DEPTH24_STENCIL8, static_cast<unsigned int>(width), static_cast<unsigned int>(height));
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, Camera_rbo);

    glBindTexture(GL_TEXTURE_2D, undistortedColorbuffer);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, static_cast<unsigned int>(width), static_cast<unsigned int>(height), 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
    glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, borderColor);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, undistortedColorbuffer, 0);

    glBindTexture(GL_TEXTURE_2D, CameratextureFinalColorbuffer);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, static_cast<unsigned int>(m_windowWidth), static_cast<unsigned int>(m_windowHeight), 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, CameratextureFinalColorbuffer, 0);
}

void VirtualCamera::rescale_camera_resolution(const float &new_width,const float &new_height){

    m_windowWidth_previous = m_windowWidth;
    m_windowHeight_previous = m_windowHeight;

    m_windowWidth = new_width;
    m_windowHeight = new_height;
    rescale_camera_framebuffer();
}

void VirtualCamera::rescale_camera_resolution(const float &new_width,const float &new_height,const float &new_Ratio){
    m_windowWidth_previous = m_windowWidth;
    m_windowHeight_previous = m_windowHeight;
    RenderRatio_previous = RenderRatio;

    m_windowWidth = new_width;
    m_windowHeight = new_height;
    RenderRatio = new_Ratio;

    rescale_camera_framebuffer();
}

void VirtualCamera::rescale_camera_resolution(const float &new_Ratio){

    RenderRatio_previous = RenderRatio;

    RenderRatio = new_Ratio;

    rescale_camera_framebuffer();
}
