#include "scene.hpp"



LidarCamSim::Scene::Scene(GLFWwindow *window,const int &screenWidth,const int &screenHeight,
                            ImFont *m_font,ImFont *m_font_mono,ImFont *m_font_mono_off,ImFont *m_icons,
                            GLFWcursor* m_arrowCursor,
                            GLFWcursor* m_dragCursor,
                            GLFWcursor* m_invisible_cursor,
                            ImWchar *m_icons_ranges,
                            ImGuiIO& io,ImGuiStyle& style, ImGuiWindowFlags &m_windowFlags_global_reference,
                            std::shared_ptr<std::vector<unsigned int>> m_global_uniform_binding_indexes): 
                            io_reference(io), global_style_reference(style), windowFlags_global_reference(m_windowFlags_global_reference),
                            InfoShader("../shaders/gui_test/uniforms/pick.vs", "../shaders/gui_test/uniforms/pick.fs"),
                            gridShader("../shaders/gui_test/grid.vs", "../shaders/gui_test/grid2.fs"),
                            OutlineShader("../shaders/gui_test/uniforms/outline.vs", "../shaders/gui_test/uniforms/outline.fs"),
                            VirtaulCamScreenShader("../shaders/gui_test/screen.vs", "../shaders/gui_test/screen.fs")

{

    main_window = window;
    font = m_font;
    font_mono = m_font_mono;
    font_mono_offset = m_font_mono_off;
    icons = m_icons;

    icons_ranges = m_icons_ranges;

    global_style_copy = style;

    monitorScreenHeight = screenHeight;
    monitorScreenWidth = screenWidth;

    arrowCursor = m_arrowCursor;
    dragCursor = m_dragCursor;
    invisible_cursor = m_invisible_cursor;

    global_uniform_binding_indexes = m_global_uniform_binding_indexes;

    glm::mat4 guizmoview_proj = glm::perspective(glm::radians(140.0f), 1.0f, 0.1f, 1000.0f);
    std::memcpy(imoguizmo_proj, glm::value_ptr(guizmoview_proj), sizeof(float) * 16);

    glGenVertexArrays(1, &dummyVAO);

    int width;
    int height;

    glfwGetWindowSize(window, &width, &height);

    window_viewport_current_width = static_cast<unsigned int>(width/2);
    window_viewport_current_height = static_cast<unsigned int>(height/2);

    /////create

    glGenBuffers(1, &uboViewportCamera);
    glBindBuffer(GL_UNIFORM_BUFFER, uboViewportCamera);
    unsigned long ubuffer_size =  2 * sizeof(glm::mat4);
    glBufferData(GL_UNIFORM_BUFFER, ubuffer_size, NULL, GL_STATIC_DRAW);
    glBindBuffer(GL_UNIFORM_BUFFER, 0);

    glGenBuffers(1, &uboVirtualCamera);
    glBindBuffer(GL_UNIFORM_BUFFER, uboVirtualCamera);
    glBufferData(GL_UNIFORM_BUFFER, ubuffer_size, NULL, GL_STATIC_DRAW);
    glBindBuffer(GL_UNIFORM_BUFFER, 0);

    if (global_uniform_binding_indexes->size() >= 2)
    {   
        ProjViewBindingViewportCamera = global_uniform_binding_indexes->back();

        glBindBufferRange(GL_UNIFORM_BUFFER, ProjViewBindingViewportCamera, uboViewportCamera, 0, 2 * sizeof(glm::mat4));

        global_uniform_binding_indexes->pop_back();

        ProjViewBindingVirtualCamera = global_uniform_binding_indexes->back();

        glBindBufferRange(GL_UNIFORM_BUFFER, ProjViewBindingVirtualCamera, uboVirtualCamera, 0, 2 * sizeof(glm::mat4));

        global_uniform_binding_indexes->pop_back();
    }
    else 
        throw std::runtime_error("There is no uniform buffer biding availiable!");

    unsigned int shID = OutlineShader.ID;
    glBindBuffer(GL_UNIFORM_BUFFER, uboViewportCamera);
    OutlineShader.use();
    glUniformBlockBinding(shID, glGetUniformBlockIndex(shID, "Matrices_cam"), ProjViewBindingViewportCamera);
    shID = InfoShader.ID;
    InfoShader.use();
    glUniformBlockBinding(shID, glGetUniformBlockIndex(shID, "Matrices_cam"), ProjViewBindingViewportCamera);
    shID = gridShader.ID;
    gridShader.use();
    glUniformBlockBinding(shID, glGetUniformBlockIndex(shID, "Matrices_cam"), ProjViewBindingViewportCamera);

    glBindBuffer(GL_UNIFORM_BUFFER, 0);

    Create_viewport_framebuffer();
    Create_info_framebuffer();

    //create lidar and camera context
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    Lidar_context = glfwCreateWindow(1, 1, "", nullptr, main_window);
    Camera_context = glfwCreateWindow(1, 1, "", nullptr, main_window);
    if (Lidar_context == nullptr || Camera_context == nullptr)
        std::cout << "Failed to create Lidar-Camera contexts" << std::endl;

    glGenBuffers(1, &lidar_context_ssbo);
    glGenBuffers(1, &camera_context_pbo);
    //glfwWindowHint(GLFW_VISIBLE, GLFW_TRUE);

}

void LidarCamSim::Scene::InitCameras(const ViewportCamConfig &view_cfg,const VirtualCamConfig &vir_cfg){

    viewport_camera  = std::make_unique<ArcBallCamera>(window_viewport_current_width, window_viewport_current_height,
                        view_cfg.pivot_position, view_cfg.pivot_distance,
                        view_cfg.yaw_pitch.x,view_cfg.yaw_pitch.y);

    viewport_camera->setConstrainAngle(false);
    

    plumb_bob_cam = std::make_unique<VirtualCamera>(vir_cfg.CamWidth,vir_cfg.CamHeight,vir_cfg.Render_Ratio,
                    VirtaulCamScreenShader,vir_cfg.position,
                    vir_cfg.yaw_pitch_roll.x,vir_cfg.yaw_pitch_roll.y,vir_cfg.yaw_pitch_roll.z);

    plumb_bob_cam->setDistortioncoeffs(vir_cfg.radialDistortion,vir_cfg.tangentialDistortion);

    radialDistortion_coefs = vir_cfg.radialDistortion;
    tangentialDistortion_coefs = vir_cfg.tangentialDistortion;

    plumb_bob_cam->CalculateIntrisicMatrices(vir_cfg.CamVfov);

    VirtualCamVfov = vir_cfg.CamVfov;

    plumb_bob_cam->create_camera_framebuffer();

    std::string default_cam_shader_path = "../shaders/gui_test/uniforms/";
    ShadersVector["vcam_model_shader"] = std::make_shared<Shader>((default_cam_shader_path+"vcam.vs").c_str(), (default_cam_shader_path+"vcam.fs").c_str());

    ModelsVector["cam_model"] = std::make_shared<Model>("../resources/models/camera/camera.obj");

    std::shared_ptr<sceneObject> cam_mesh = std::make_shared<sceneObject>();

    cam_mesh->object = std::make_shared<object_model>(plumb_bob_cam->GetCameraPosition(),
                                                    glm::radians(plumb_bob_cam->GetCameraRotation()),
                                                    *(ShadersVector["vcam_model_shader"].get()),
                                                    *(ModelsVector["cam_model"].get()),DRAW_MODEL);

    cam_mesh->cameras_for_render["viewport_cam"] = true; 
    cam_mesh->texture_transform_support = false;
    cam_mesh->auto_render = true;
    ObjectsVector["virtual_cam"] = cam_mesh;


    cam_mesh->object->SetInfoShader(&InfoShader);
    cam_mesh->object->SetOutlineShader(&OutlineShader);
    cam_mesh->object->SetDrawInfoMode(DRAW_MODEL_INFO);
    cam_mesh->object->SetDrawOutlineMode(DRAW_MODEL_OUTLINE);

    viewport_camera_projection_matrix = glm::perspective(glm::radians(ViewportCamVfov), (float)window_viewport_current_width / (float)window_viewport_current_height, 0.1f, 100.0f);
    viewport_camera_view_matrix = viewport_camera->GetViewMatrix();

    update_viewport_cam_view_matrix_ubo();
    update_virtual_cam_view_matrix_ubo();
    
    //initing pixel buffer object, make transfers to the cpu
    unsigned int camera_buffer_size = vir_cfg.CamWidth*vir_cfg.CamHeight *3U;
    glBindBuffer(GL_PIXEL_PACK_BUFFER, camera_context_pbo);
    glBufferData(GL_PIXEL_PACK_BUFFER, camera_buffer_size*sizeof(float), nullptr, GL_STREAM_READ);
    glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);

    Camera_cpu_buffer.resize(camera_buffer_size);
}

void LidarCamSim::Scene::InitBoard(const BoardConfig &board_cfg){
    BoardParams = board_cfg;

    LidarCamSim::shaderConfig board_shader_cfg;
    board_shader_cfg.type = LidarCamSim::shader_type::DEFAULT;
    board_shader_cfg.vertex_path = "../shaders/gui_test/uniforms/chessboard.vs";
    board_shader_cfg.fragment_path =  "../shaders/gui_test/uniforms/chessboard.fs";
    board_shader_cfg.shader_name = "chessboardShader";
    AddShader(board_shader_cfg);


    std::shared_ptr<plane> s_chessboard = std::make_shared<plane>(glm::vec3(0.0f,1.1f,0.0f),
                                                            *(ShadersVector["chessboardShader"].get()),
                                                            DRAW_UNTEXTURED,1);
    s_chessboard->SetInfoShader(&InfoShader);
    s_chessboard->SetOutlineShader(&OutlineShader);
    s_chessboard->SetDrawInfoMode(DRAW_INFO);
    s_chessboard->SetDrawOutlineMode(DRAW_OUTLINE);
    s_chessboard->Change_scale(glm::vec3(1.0f,BoardParams.board_dimension.y/1000.0f,BoardParams.board_dimension.x/1000.0f));
    s_chessboard->back_face_culling = false;
    //s_chessboard->Rotate(glm::radians(90.0f),glm::vec3(0.0,0.0,1.0));

    AddObject(s_chessboard,"chessboard");
    ObjectsVector["chessboard"]->auto_render = true;
    ObjectsVector["chessboard"]->texture_transform_support = false;
    ObjectsVector["chessboard"]->cameras_for_render["viewport_cam"] = true;
    ObjectsVector["chessboard"]->cameras_for_render["virtual_cam"] = true;

    LidarAddObject("chessboard",1);

    ShadersVector["chessboardShader"]->use();
    ShadersVector["chessboardShader"]->setVec2("pattern_size",glm::vec2(BoardParams.pattern_size));
    ShadersVector["chessboardShader"]->setFloat("square_length",BoardParams.square_length/1000.0f);
    ShadersVector["chessboardShader"]->setVec2("board_dimension",BoardParams.board_dimension/1000.0f);
    ShadersVector["chessboardShader"]->setVec2("translation_error",BoardParams.translation_error/1000.0f);

}

void LidarCamSim::Scene::AddShader(const shaderConfig &shader_cfg){

    switch(shader_cfg.type){
        case LidarCamSim::shader_type::DEFAULT:
            ShadersVector[shader_cfg.shader_name] = std::make_shared<Shader>(shader_cfg.vertex_path.c_str(),shader_cfg.fragment_path.c_str());
            break;
        case LidarCamSim::shader_type::WTH_GEOMETRY:
            ShadersVector[shader_cfg.shader_name] = std::make_shared<Shader>(shader_cfg.vertex_path.c_str(),shader_cfg.geometry_path.c_str(),shader_cfg.fragment_path.c_str());
            break;
        case LidarCamSim::shader_type::COMPUTE:
            ComputeShadersVector[shader_cfg.shader_name] = std::make_shared<Shader_compute>(shader_cfg.compute_path.c_str());
            break;
    }
}

void LidarCamSim::Scene::RemoveShader(const std::string &shader_name){

    ShadersVector.erase(shader_name);

}

void LidarCamSim::Scene::AddModel(const std::string &model_path, const std::string &model_name){

    ModelsVector[model_name] = std::make_shared<Model>(model_path);

}

void LidarCamSim::Scene::RemoveModel(const std::string &model_name){

    ModelsVector.erase(model_name);

}

void LidarCamSim::Scene::AddObject(std::shared_ptr<Object> object_ptr,const std::string &object_name){
    ObjectsVector[object_name] = std::make_shared<sceneObject>();
    ObjectsVector[object_name]->object = object_ptr;
}

void LidarCamSim::Scene::RemoveObject(const std::string &object_name){
    ObjectsVector.erase(object_name);
}

void LidarCamSim::Scene::Verify_clicked_object(){
    
    
    if(ImGui::IsMouseClicked(ImGuiMouseButton_Left,true) && scene_hovered && !isUsingGuizmo){
        glBindFramebuffer(GL_READ_FRAMEBUFFER, m_info_framebuffer);
        glReadBuffer(GL_COLOR_ATTACHMENT0);

        unsigned int Pixel[3];
        ImVec2 mouse_position  = ImGui::GetMousePos();
        mouse_position.x -= ViewportPos.x;
        mouse_position.y -= ViewportPos.y;
        mouse_position.y = (ViewportSize.y - 1) - mouse_position.y;
        glReadPixels(static_cast<unsigned int>(mouse_position.x),static_cast<unsigned int>(mouse_position.y),1,1,GL_RGB_INTEGER,GL_UNSIGNED_INT,&Pixel);

        std::cout << "("<<Pixel[0]<<","<<Pixel[1]<<","<<Pixel[2]<<")" << std::endl;

        if(Pixel[0] == 0) {
            object_selected = "";
        }
        else {
            auto it = ObjectsVector.begin();
            std::advance(it, Pixel[0] - 1);
            object_selected  = it->first;
        }

        glReadBuffer(GL_NONE);
        glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
    }
}

void LidarCamSim::Scene::draw_outline(){

    if(object_selected != ""){
        glBindBuffer(GL_UNIFORM_BUFFER, uboViewportCamera);
        glEnable(GL_STENCIL_TEST);
        glStencilFunc(GL_NOTEQUAL, 0x01, 0xFF);
        glStencilMask(0x00); 
        //glDisable(GL_DEPTH_TEST);
        float distance = glm::length(ObjectsVector[object_selected]->object->position - viewport_camera->m_pos);
        ObjectsVector[object_selected]->object->SetOutlineTransform(); 
        OutlineShader.setFloat("distant",distance);
        if(ObjectsVector[object_selected]->object->object_type == ObjTypes::Object_types::Plane){

            std::shared_ptr<plane> PlaneObj = std::static_pointer_cast<plane>(ObjectsVector[object_selected]->object);
            if(PlaneObj->back_face_culling)
                PlaneObj->DrawOutline();
            else{
                glDisable(GL_CULL_FACE);
                PlaneObj->DrawOutline();
                glEnable(GL_CULL_FACE);
            }

        }
        else ObjectsVector[object_selected]->object->DrawOutline();
        glStencilMask(0xFF);
        glStencilFunc(GL_ALWAYS, 0, 0xFF);
        glDisable(GL_STENCIL_TEST);
        glBindBuffer(GL_UNIFORM_BUFFER, 0);
    }

    
}

void LidarCamSim::Scene::draw_grid(){

    glEnable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);   
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);

    glBindBuffer(GL_UNIFORM_BUFFER, uboViewportCamera);

    gridShader.use();
    //glUniformBlockBinding(gridShader.ID, glGetUniformBlockIndex(gridShader.ID, "Matrices_cam"), ProjViewBindingViewportCamera);
    gridShader.setVec3("cameraPos",viewport_camera->m_pos);
    glBindVertexArray(dummyVAO);
    glDrawArrays(GL_TRIANGLES,0,6);
    
    glEnable(GL_CULL_FACE);

    glBindVertexArray(0);

    glBindBuffer(GL_UNIFORM_BUFFER, 0);
}

void LidarCamSim::Scene::RenderInfoBuffer(){

    //rendering picking buffer

    glViewport(0, 0, window_viewport_current_width, window_viewport_current_height);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, m_info_framebuffer);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glFrontFace(GL_CCW);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glBindBuffer(GL_UNIFORM_BUFFER, uboViewportCamera);

    unsigned int index = 1;
    for (auto it = ObjectsVector.begin(); it != ObjectsVector.end(); it++) {

        if(it->second->cameras_for_render.find("viewport_cam") != it->second->cameras_for_render.end()){

            //glm::mat4 view_matrix = camera.GetViewMatrix();
            //glm::mat4 projection;

            it->second->object->SetInfoTransform();

            switch (it->second->object->object_type)
            {
            case ObjTypes::Object_types::Plane:{
                std::shared_ptr<plane> PlaneObj = std::static_pointer_cast<plane>(it->second->object);
                if(PlaneObj->back_face_culling)
                    PlaneObj->DrawInfo(index);
                else{
                    glDisable(GL_CULL_FACE);
                    PlaneObj->DrawInfo(index);
                    glEnable(GL_CULL_FACE);
                }
                
                break;
            }
            case ObjTypes::Object_types::Cube:
                std::static_pointer_cast<cube>(it->second->object)->DrawInfo(index);
                break;
            case ObjTypes::Object_types::Sphere:
                std::static_pointer_cast<sphere>(it->second->object)->DrawInfo(index);
                break;
            case ObjTypes::Object_types::Mesh:
                std::static_pointer_cast<object_model>(it->second->object)->DrawInfo(index);
                break;
            }
        }
        ++index; // Increment the index
    }
    glBindBuffer(GL_UNIFORM_BUFFER, 0);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
}

void LidarCamSim::Scene::RenderStartMainViewport(){
        glBindFramebuffer(GL_FRAMEBUFFER, viewport_framebuffer);

        glViewport(0, 0, window_viewport_current_width , window_viewport_current_height);
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LESS);
        glEnable(GL_CULL_FACE);
        glCullFace(GL_BACK);
        glFrontFace(GL_CCW);
        //glEnable(GL_DEPTH_TEST); // enable depth testing (is disabled for rendering screen-space quad)
        // render
        // ------
        glClearColor(background_scene_color.x, background_scene_color.y, background_scene_color.z, background_scene_color.w);
        //glClearColor(0.2f, 0.3f, 0.3f, 1.0f);

        //glEnable(GL_STENCIL_TEST);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
        //stencil
        glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
        glStencilFunc(GL_ALWAYS, 0x01, 0xFF); 
        glStencilMask(0xFF); 

        glBindBuffer(GL_UNIFORM_BUFFER, uboViewportCamera);

}

void LidarCamSim::Scene::RenderEndMainViewport(){
        glBindBuffer(GL_UNIFORM_BUFFER, 0);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void LidarCamSim::Scene::RenderMainViewportAuto(){



        for (auto it = ObjectsVector.begin(); it != ObjectsVector.end(); it++) {

            if(it->second->cameras_for_render.find("viewport_cam") != it->second->cameras_for_render.end() && it->second->auto_render){
                glDisable(GL_STENCIL_TEST);

                if(it->first == object_selected) glEnable(GL_STENCIL_TEST);

                
                it->second->object->SetViewProjUBuffer(ProjViewBindingViewportCamera);
                it->second->object->SetTransform();
                //glm::mat4 *model_matrix;
                //glm::mat3 *model_matrix_tex;

                if(it->second->texture_transform_support){
                    it->second->object->SetTextureTransform();
                }

                switch (it->second->object->object_type)
                {
                case ObjTypes::Object_types::Plane:{
                    std::shared_ptr<plane> PlaneObj = std::static_pointer_cast<plane>(it->second->object);
                    if(PlaneObj->back_face_culling)
                        PlaneObj->Draw();
                    else{
                        glDisable(GL_CULL_FACE);
                        PlaneObj->Draw();
                        glEnable(GL_CULL_FACE);
                    }

                    break;
                }
                case ObjTypes::Object_types::Cube:
                    std::static_pointer_cast<cube>(it->second->object)->Draw();
                    break;
                case ObjTypes::Object_types::Sphere:
                    std::static_pointer_cast<sphere>(it->second->object)->Draw();
                    break;
                case ObjTypes::Object_types::Mesh:
                    std::static_pointer_cast<object_model>(it->second->object)->Draw();
                    //model_matrix = &(std::static_pointer_cast<object_model>(it->second->object))->model_m;
                    //model_matrix_tex = &(std::static_pointer_cast<object_model>(it->second->object))->model_m_tex;
                    //glBufferSubData(GL_UNIFORM_BUFFER, 0,sizeof(glm::mat4), glm::value_ptr(*model_matrix));
                    break;
                }


                //glBufferSubData(GL_UNIFORM_BUFFER, 2*sizeof(glm::mat4),sizeof(float), &near);
                //glBufferSubData(GL_UNIFORM_BUFFER, 2*sizeof(glm::mat4) + sizeof(float),sizeof(float), &far);
                


            }
        }
        glDisable(GL_STENCIL_TEST); 

}

void LidarCamSim::Scene::StartRenderVirtual(){

    plumb_bob_cam->StartRender();

    glEnable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);

    glBindBuffer(GL_UNIFORM_BUFFER, uboVirtualCamera);

    //cube.SetProjection(plumb_bob_cam.projection_matrix); 
    //cube.SetView(plumb_bob_cam.GetViewMatrix());

}

void LidarCamSim::Scene::AutoRenderVirtual(){

    for (auto it = ObjectsVector.begin(); it != ObjectsVector.end(); it++) {

        if(it->second->cameras_for_render.find("virtual_cam") != it->second->cameras_for_render.end() && it->second->auto_render){

            it->second->object->SetViewProjUBuffer(ProjViewBindingVirtualCamera);
            it->second->object->SetTransform();
            //glm::mat4 *model_matrix;
            //glm::mat3 *model_matrix_tex;

            if(it->second->texture_transform_support){
                it->second->object->SetTextureTransform();
            }

            switch (it->second->object->object_type)
            {
            case ObjTypes::Object_types::Plane:{
                std::shared_ptr<plane> PlaneObj = std::static_pointer_cast<plane>(it->second->object);
                if(PlaneObj->back_face_culling)
                    PlaneObj->Draw();
                else{
                    glDisable(GL_CULL_FACE);
                    PlaneObj->Draw();
                    glEnable(GL_CULL_FACE);
                }

                break;
            }
            case ObjTypes::Object_types::Cube:
                std::static_pointer_cast<cube>(it->second->object)->Draw();
                break;
            case ObjTypes::Object_types::Sphere:
                std::static_pointer_cast<sphere>(it->second->object)->Draw();
                break;
            case ObjTypes::Object_types::Mesh:
                std::static_pointer_cast<object_model>(it->second->object)->Draw();
                //model_matrix = &(std::static_pointer_cast<object_model>(it->second->object))->model_m;
                //model_matrix_tex = &(std::static_pointer_cast<object_model>(it->second->object))->model_m_tex;
                //glBufferSubData(GL_UNIFORM_BUFFER, 0,sizeof(glm::mat4), glm::value_ptr(*model_matrix));
                break;
            }


            //glBufferSubData(GL_UNIFORM_BUFFER, 2*sizeof(glm::mat4),sizeof(float), &near);
            //glBufferSubData(GL_UNIFORM_BUFFER, 2*sizeof(glm::mat4) + sizeof(float),sizeof(float), &far);
            


        }
    } 
}
void LidarCamSim::Scene::EndRenderVirtual(){

    glBindBuffer(GL_UNIFORM_BUFFER, 0);

    plumb_bob_cam->FinishRender();
    //plumb_bob_cam.setDistortioncoeffs(glm::vec3(0.0f),glm::vec2(0.0f));
    plumb_bob_cam->GenarateImage();
}

void LidarCamSim::Scene::Create_viewport_framebuffer()
{   

    // framebuffer configuration
    // -------------------------
    glGenFramebuffers(1, &viewport_framebuffer);
    glBindFramebuffer(GL_FRAMEBUFFER, viewport_framebuffer);
    // create a color attachment texture
    glGenTextures(1, &viewport_textureColorbuffer);
    glBindTexture(GL_TEXTURE_2D, viewport_textureColorbuffer);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, window_viewport_current_width, window_viewport_current_height, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, viewport_textureColorbuffer, 0);
    // create a renderbuffer object for depth and stencil attachment (we won't be sampling these)
    glGenRenderbuffers(1, &viewport_rbo);
    glBindRenderbuffer(GL_RENDERBUFFER, viewport_rbo);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, window_viewport_current_width, window_viewport_current_height); // use a single renderbuffer object for both a depth AND stencil buffer.
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, viewport_rbo); // now actually attach it
    // now that we actually created the framebuffer and added all attachments we want to check if it is actually complete now
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        std::cout << "ERROR::FRAMEBUFFER:: Framebuffer is not complete!" << std::endl;
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    previousImGuiSceneWindowSize = ImVec2(window_viewport_current_width, window_viewport_current_height);
}

// and we rescale the buffer, so we're able to resize the window
void LidarCamSim::Scene::Rescale_viewport_framebuffer(int width, int height)
{
	glBindTexture(GL_TEXTURE_2D, viewport_textureColorbuffer);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, viewport_textureColorbuffer, 0);

	glBindRenderbuffer(GL_RENDERBUFFER, viewport_rbo);
	glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, width, height);
	glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, viewport_rbo);
}

void LidarCamSim::Scene::Create_info_framebuffer()
{   

    // framebuffer configuration
    // -------------------------
    glGenFramebuffers(1, &m_info_framebuffer);
    glBindFramebuffer(GL_FRAMEBUFFER, m_info_framebuffer);
    // create a color attachment texture
    glGenTextures(1, &m_picking_texture);
    glBindTexture(GL_TEXTURE_2D, m_picking_texture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB32UI, window_viewport_current_width, window_viewport_current_height, 0, GL_RGB_INTEGER, GL_UNSIGNED_INT, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_picking_texture, 0);
    // create a renderbuffer object for depth and stencil attachment (we won't be sampling these)
    glGenTextures(1, &m_picking_depth_tex);
    glBindTexture(GL_TEXTURE_2D, m_picking_depth_tex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, window_viewport_current_width, window_viewport_current_height, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
    glFramebufferTexture2D(GL_FRAMEBUFFER,GL_DEPTH_ATTACHMENT,GL_TEXTURE_2D,m_picking_depth_tex,0);
    // now that we actually created the framebuffer and added all attachments we want to check if it is actually complete now
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        std::cout << "ERROR::FRAMEBUFFER:: Framebuffer is not complete!" << std::endl;
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    previousPickingImGuiSceneWindowSize = ImVec2(window_viewport_current_width, window_viewport_current_height);
}


void LidarCamSim::Scene::Rescale_info_framebuffer(int width, int height)
{
    glBindTexture(GL_TEXTURE_2D, m_picking_texture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB32UI, width, height, 0, GL_RGB_INTEGER, GL_UNSIGNED_INT, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_picking_texture, 0);

    glBindTexture(GL_TEXTURE_2D, m_picking_depth_tex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, width, height, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
    glFramebufferTexture2D(GL_FRAMEBUFFER,GL_DEPTH_ATTACHMENT,GL_TEXTURE_2D,m_picking_depth_tex,0);
}

void LidarCamSim::Scene::draw_guizmo_toolbar(){
    ImGui::PushFont(icons);
    ImVec2 toolbarPos = ImVec2(ViewportMin.x + 10, ViewportMin.y + 10); // 10 px padding
    ImGui::SetCursorScreenPos(toolbarPos);

    // Begin a transparent child window for the button bar
    ImGuiWindowFlags toolbarFlagsWindow =
        ImGuiWindowFlags_NoDecoration |
        ImGuiWindowFlags_NoSavedSettings |
        ImGuiWindowFlags_NoFocusOnAppearing |
        ImGuiWindowFlags_NoNav |
        windowFlags_global_reference;

    ImGuiChildFlags toolbarFlagsChild = 
        ImGuiChildFlags_AlwaysAutoResize | 
        ImGuiChildFlags_AutoResizeX | 
        ImGuiChildFlags_AutoResizeY;


    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(4, 4));
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(4, 4));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 5.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_Alpha, 0.6f);
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(2.0f, 0.0f)); 
    ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 5.0f);    

    float button_size = static_cast<float>(monitorScreenHeight)*0.03;

    ImGui::BeginChild("GizmoToolbar", ImVec2(3*button_size +4*2, button_size), toolbarFlagsChild, toolbarFlagsWindow);


    //global_style_reference.Colors[ImGuiCol_Button] = ImVec4(0.85f, 0.85f, 0.85f, 1.0f); // Default button color
    //global_style_reference.Colors[ImGuiCol_ButtonHovered] = ImVec4(0.75f, 0.75f, 0.75f, 1.0f); // Hover color
    //global_style_reference.Colors[ImGuiCol_ButtonActive] = ImVec4(1.0f, 0.0f, 0.0f, 1.0f); // Active color

    ImVec4 default_color_def = ImVec4(0.85f, 0.85f, 0.85f, 1.0f); // Default button color
    ImVec4 default_color_hover = ImVec4(0.75f, 0.75f, 0.75f, 1.0f); // Hover color

    ImVec4 selected_color_def = ImVec4(1.0f, 0.0f, 0.0f, 1.0f); // selected Default button color
    ImVec4 selected_color_hover = ImVec4(1.0f, 0.0f, 0.0f, 1.0f); //selected Hover color

    if(ImGui::InvisibleButton("##hiddenb0", ImVec2(button_size, button_size))){
        currentTransformOp = ImGuizmo::TRANSLATE;
        button_state_vector[0] = true;
        button_state_vector[1] = false;
        button_state_vector[2] = false;
    }
    ImGui::SetCursorScreenPos(toolbarPos);

    bool active = ImGui::IsItemActive();
    bool hovered = ImGui::IsItemHovered();

    if(button_state_vector[0]){
        global_style_reference.Colors[ImGuiCol_Button] = selected_color_def; // Default button color
        //global_style_reference.Colors[ImGuiCol_ButtonHovered] = selected_color_hover; // Hover color
    }else if(hovered && !active){
        global_style_reference.Colors[ImGuiCol_Button] = default_color_hover; // Default button color
    }else if(hovered && active){
        global_style_reference.Colors[ImGuiCol_Button] = selected_color_def;
    }else{
        global_style_reference.Colors[ImGuiCol_Button] = default_color_def;
    }

    ImGui::Button("\ue900", ImVec2(button_size, button_size));
        
    ImGui::SameLine();

    ImVec2 button_pos  = ImGui::GetCursorScreenPos();
    
    if(ImGui::InvisibleButton("##hiddenb1", ImVec2(button_size, button_size))){
        currentTransformOp = ImGuizmo::ROTATE;
        button_state_vector[0] = false;
        button_state_vector[1] = true;
        button_state_vector[2] = false;
    }
    ImGui::SetCursorScreenPos(button_pos);

    active = ImGui::IsItemActive();
    hovered = ImGui::IsItemHovered();

    if(button_state_vector[1]){
        global_style_reference.Colors[ImGuiCol_Button] = selected_color_def; // Default button color
        //global_style_reference.Colors[ImGuiCol_ButtonHovered] = selected_color_hover; // Hover color
    }else if(hovered && !active){
        global_style_reference.Colors[ImGuiCol_Button] = default_color_hover; // Default button color
    }else if(hovered && active){
        global_style_reference.Colors[ImGuiCol_Button] = selected_color_def;
    }else{
        global_style_reference.Colors[ImGuiCol_Button] = default_color_def;
    }

    ImGui::Button("\ue901", ImVec2(button_size, button_size));

    ImGui::SameLine();

    button_pos  = ImGui::GetCursorScreenPos();
    
    if(ImGui::InvisibleButton("##hiddenb2", ImVec2(button_size, button_size))){
        currentTransformOp = ImGuizmo::SCALE;
        button_state_vector[0] = false;
        button_state_vector[1] = false;
        button_state_vector[2] = true;
    }
    ImGui::SetCursorScreenPos(button_pos);

    active = ImGui::IsItemActive();
    hovered = ImGui::IsItemHovered();

    if(button_state_vector[2]){
        global_style_reference.Colors[ImGuiCol_Button] = selected_color_def; // Default button color
        //global_style_reference.Colors[ImGuiCol_ButtonHovered] = selected_color_hover; // Hover color
    }else if(hovered && !active){
        global_style_reference.Colors[ImGuiCol_Button] = default_color_hover; // Default button color
    }else if(hovered && active){
        global_style_reference.Colors[ImGuiCol_Button] = selected_color_def;
    }else{
        global_style_reference.Colors[ImGuiCol_Button] = default_color_def;
    }

    ImGui::Button("\ue902", ImVec2(button_size, button_size));

    global_style_reference.Colors[ImGuiCol_Button] = default_color_def; // Default button color
    global_style_reference.Colors[ImGuiCol_ButtonHovered] = default_color_hover; // Hover color
    global_style_reference.Colors[ImGuiCol_ButtonActive] = selected_color_def; // Active color

    ImGui::EndChild();


    toolbarPos = ImVec2(ViewportMin.x + 20 + 4*button_size, ViewportMin.y + 10); // 10 px padding
    ImGui::SetCursorScreenPos(toolbarPos);

    ImGui::BeginChild("world_transform_fly_mode", ImVec2(2*button_size+3*2, button_size), toolbarFlagsChild, toolbarFlagsWindow);

    if(ImGui::InvisibleButton("##hiddenb6", ImVec2(button_size, button_size))){
        button_state_vector[3] = !button_state_vector[3];
        currentTransformMode = (button_state_vector[3])?ImGuizmo::WORLD:ImGuizmo::LOCAL;
    }
    ImGui::SetCursorScreenPos(toolbarPos);

    active = ImGui::IsItemActive();
    hovered = ImGui::IsItemHovered();

    if(button_state_vector[3]){
        global_style_reference.Colors[ImGuiCol_Button] = selected_color_def; // Default button color
        //global_style_reference.Colors[ImGuiCol_ButtonHovered] = selected_color_hover; // Hover color
    }else if(hovered && !active){
        global_style_reference.Colors[ImGuiCol_Button] = default_color_hover; // Default button color
    }else if(hovered && active){
        global_style_reference.Colors[ImGuiCol_Button] = selected_color_def;
    }else{
        global_style_reference.Colors[ImGuiCol_Button] = default_color_def;
    }
    
    ImGui::Button("\ue906", ImVec2(button_size, button_size));

    ImGui::SameLine();

    button_pos  = ImGui::GetCursorScreenPos();

    if(ImGui::InvisibleButton("##hiddenbfly", ImVec2(button_size, button_size))){
        button_state_vector[4] = !button_state_vector[4];
        viewport_camera->fly_mode = button_state_vector[4];
    }
    ImGui::SetCursorScreenPos(button_pos);

    active = ImGui::IsItemActive();
    hovered = ImGui::IsItemHovered();

    if(button_state_vector[4]){
        global_style_reference.Colors[ImGuiCol_Button] = selected_color_def; // Default button color
        //global_style_reference.Colors[ImGuiCol_ButtonHovered] = selected_color_hover; // Hover color
    }else if(hovered && !active){
        global_style_reference.Colors[ImGuiCol_Button] = default_color_hover; // Default button color
    }else if(hovered && active){
        global_style_reference.Colors[ImGuiCol_Button] = selected_color_def;
    }else{
        global_style_reference.Colors[ImGuiCol_Button] = default_color_def;
    }
    
    ImGui::Button("\ue909", ImVec2(button_size, button_size));

    global_style_reference.Colors[ImGuiCol_Button] = default_color_def; // Default button color
    global_style_reference.Colors[ImGuiCol_ButtonHovered] = default_color_hover; // Hover color
    global_style_reference.Colors[ImGuiCol_ButtonActive] = selected_color_def; // Active color

    ImGui::EndChild();

    toolbarPos = ImVec2(ViewportMin.x + 22 + 8*button_size, ViewportMin.y + 10); // 10 px padding
    ImGui::SetCursorScreenPos(toolbarPos);

    ImGui::BeginChild("camera_mode", ImVec2(button_size, button_size), toolbarFlagsChild, toolbarFlagsWindow);

    if(button_state_vector[5]){
        if (ImGui::Button("\ue904", ImVec2(button_size, button_size)) || want_to_toggle_proj){
            button_state_vector[5] = !button_state_vector[5];
            want_to_toggle_proj = false;

        }
    }else{
        if (ImGui::Button("\ue905", ImVec2(button_size, button_size)) || want_to_toggle_proj){
            button_state_vector[5] = !button_state_vector[5];
            want_to_toggle_proj = false;
            viewport_camera_projection_matrix = glm::perspective(glm::radians(ViewportCamVfov), 
                (float)window_viewport_current_width / (float)window_viewport_current_height, 
                0.1f, 100.0f);
        }
    }
    
    ImGui::EndChild();

    toolbarPos = ImVec2(ViewportMin.x + 20 + 9.3*button_size, ViewportMin.y + 10); // 10 px padding
    ImGui::SetCursorScreenPos(toolbarPos);
    ImGui::PopFont();

    ImGui::BeginChild("perspective_angle", ImVec2(2.6f*button_size,button_size), toolbarFlagsChild, toolbarFlagsWindow);
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(5, 7));
    ImGui::PushStyleColor(ImGuiCol_FrameBg,        ImVec4(default_color_def.x,default_color_def.y,default_color_def.z,0.6f)); // normal bg
    ImGui::PushStyleColor(ImGuiCol_FrameBgHovered, ImVec4(default_color_hover.x,default_color_hover.y,default_color_hover.z, 0.6f)); // hover bg
    ImGui::PushStyleColor(ImGuiCol_FrameBgActive,  ImVec4(selected_color_def.x,selected_color_def.y,selected_color_def.z, 0.6f)); // active bg;
    ImGui::PushStyleColor(ImGuiCol_Text,           ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
    //global_style_reference.Colors[ImGuiCol_Button] = ImVec4(0.85f, 0.85f, 0.85f, 1.0f); // Default button color
    //global_style_reference.Colors[ImGuiCol_ButtonHovered] = ImVec4(0.75f, 0.75f, 0.75f, 1.0f); // Hover color
    ImGui::PushStyleVar(ImGuiStyleVar_Alpha, 1.0f);
    ImGui::PushItemWidth(2.6f*button_size);

    float vfov_tmp = ViewportCamVfov;

    ImGui::PushFont(font_mono_offset);
    ImGui::DragFloat("##DragViewFov", &vfov_tmp,0.1f,0.1f, 89.0f, "\ue907:%.1f°",ImGuiSliderFlags_AlwaysClamp);
    ImGui::PopFont();

    if(ImGui::IsItemActive()){
        if (ImGui::TempInputIsActive(ImGui::GetItemID())) {
            ViewportCamVfovTemp = vfov_tmp;
            last_frame_view_fov_input_txt_active = true;

        }else if(ImGui::IsMouseDown(ImGuiMouseButton_Left))
        {
            ViewportCamVfov = std::clamp(vfov_tmp, 0.1f, 89.0f);
            last_frame_view_fov_input_txt_active = false;
            if(button_state_vector[5])
                viewport_camera_projection_matrix = glm::perspective(glm::radians(ViewportCamVfov), 
                    (float)window_viewport_current_width / (float)window_viewport_current_height, 
                    0.1f, 100.0f);
            
        }else last_frame_view_fov_input_txt_active = false;
        //std::cout<<"changed"<<std::endl;

    }else if(last_frame_view_fov_input_txt_active){

        if (ImGui::IsKeyPressed(ImGuiKey_Enter) || ImGui::IsKeyPressed(ImGuiKey_KeypadEnter)){
            ViewportCamVfov = std::clamp(ViewportCamVfovTemp, 0.1f, 89.0f);
            if(button_state_vector[5])
                viewport_camera_projection_matrix = glm::perspective(glm::radians(ViewportCamVfov), 
                    (float)window_viewport_current_width / (float)window_viewport_current_height, 
                    0.1f, 100.0f);
        }
        last_frame_view_fov_input_txt_active = false;
    }
    
    ImGui::PopStyleColor(4);
    ImGui::PopStyleVar(2);
    ImGui::PopItemWidth();

    ImGui::EndChild();


    global_style_reference.Colors[ImGuiCol_Button] = global_style_copy.Colors[ImGuiCol_Button];
    global_style_reference.Colors[ImGuiCol_ButtonHovered] = global_style_copy.Colors[ImGuiCol_ButtonHovered];
    global_style_reference.Colors[ImGuiCol_ButtonActive] = global_style_copy.Colors[ImGuiCol_ButtonActive];

    ImGui::PopStyleVar(6);

    //ImGui::PopFont();
}

void LidarCamSim::Scene::draw_imoguizmo(){

    ImOGuizmo::config.axisLengthScale = 1.0f;
    const float square_size = 120.0f;
    // specify position and size of gizmo (and its window when using ImOGuizmo::BeginFrame())
    ImOGuizmo::SetRect(ViewportPos.x +ViewportSize.x-square_size -5 /* x */, ViewportPos.y + 5 /* y */, square_size /* square size */);
    //ImOGuizmo::BeginFrame(); // to use you own window remove this call 
    // and wrap everything in between ImGui::Begin() and ImGui::End() instead

    float viewMatrix[16];
    viewport_camera_view_matrix = viewport_camera->GetViewMatrix();
    memcpy(viewMatrix, glm::value_ptr(viewport_camera_view_matrix), sizeof(float) * 16);
    // optional: set distance to pivot (-> activates interaction)
    if(ImOGuizmo::DrawGizmo(viewMatrix, imoguizmo_proj, viewport_camera->m_pivot_distance /* optional: default = 0.0f */))
    {   
    	// in case of user interaction viewMatrix gets updated
        glm::vec3 right_v(viewMatrix[0],viewMatrix[4],viewMatrix[8]);
        glm::vec3 up_v(viewMatrix[1],viewMatrix[5],viewMatrix[9]);
        glm::vec3 front_v(-viewMatrix[2],-viewMatrix[6],-viewMatrix[10]);
        viewport_camera->SetVectors(right_v,up_v,front_v);
        viewport_camera_view_matrix = viewport_camera->GetViewMatrix();
    }

}

void LidarCamSim::Scene::update_viewport_cam_view_matrix_ubo(){
        glBindBuffer(GL_UNIFORM_BUFFER, uboViewportCamera);
        glBufferSubData(GL_UNIFORM_BUFFER, 0, sizeof(glm::mat4), glm::value_ptr(viewport_camera_view_matrix));
        glBufferSubData(GL_UNIFORM_BUFFER, sizeof(glm::mat4), sizeof(glm::mat4), glm::value_ptr(viewport_camera_projection_matrix));
        glBindBuffer(GL_UNIFORM_BUFFER, 0);

}

void LidarCamSim::Scene::update_viewport_cam_projection_matrix_ubo(){
        glBindBuffer(GL_UNIFORM_BUFFER, uboViewportCamera);
        glBufferSubData(GL_UNIFORM_BUFFER, sizeof(glm::mat4), sizeof(glm::mat4), glm::value_ptr(viewport_camera_projection_matrix));
        glBindBuffer(GL_UNIFORM_BUFFER, 0);

}

void LidarCamSim::Scene::update_virtual_cam_view_matrix_ubo(){
        glBindBuffer(GL_UNIFORM_BUFFER, uboVirtualCamera);
        glBufferSubData(GL_UNIFORM_BUFFER, 0, sizeof(glm::mat4), glm::value_ptr(plumb_bob_cam->GetViewMatrix()));
        glBufferSubData(GL_UNIFORM_BUFFER, sizeof(glm::mat4), sizeof(glm::mat4), glm::value_ptr(plumb_bob_cam->projection_matrix));
        glBindBuffer(GL_UNIFORM_BUFFER, 0);

}

void LidarCamSim::Scene::update_virtual_cam_projection_matrix_ubo(){
        glBindBuffer(GL_UNIFORM_BUFFER, uboVirtualCamera);
        glBufferSubData(GL_UNIFORM_BUFFER, sizeof(glm::mat4), sizeof(glm::mat4), glm::value_ptr(plumb_bob_cam->projection_matrix));
        glBindBuffer(GL_UNIFORM_BUFFER, 0);
}

void LidarCamSim::Scene::draw_viewport(){

    global_style_reference.WindowPadding = ImVec2(0.0f, 0.0f);
    global_style_reference.WindowBorderSize = 0.0f;  

    ImGui::Begin("Viewport",nullptr,ImGuiWindowFlags_None | windowFlags_global_reference);

    //bool scene_focused = ImGui::IsWindowFocused();
    scene_hovered = ImGui::IsWindowHovered() && glfwGetWindowAttrib(main_window, GLFW_HOVERED);  
    if (scene_hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Middle))
    {
        ImGui::SetNextWindowFocus();
    }   
    // we access the ImGui window size
    ViewportSize = ImGui::GetContentRegionAvail();
    ViewportMin = ImGui::GetCursorScreenPos();
    ViewportMax = ImVec2(ViewportMin.x + ViewportSize.x,ViewportMin.y + ViewportSize.y);

    ViewportPos = ViewportMin;
    // Here we can render into the ImGui window
    // ImGui Buttons, Drop Downs, etc. and later our framebuffer    
    // Check if the ImGui window size has changed
    if (ViewportSize.x != previousImGuiSceneWindowSize.x || ViewportSize.y != previousImGuiSceneWindowSize.y) {
        // Update framebuffer size
        window_viewport_current_width = static_cast<unsigned int>(ViewportSize.x);
        window_viewport_current_height = static_cast<unsigned int>(ViewportSize.y);
        // we rescale the framebuffer to the actual window size here and reset the glViewport 
        Rescale_viewport_framebuffer(static_cast<int>(window_viewport_current_width), 
                                    static_cast<int>(window_viewport_current_height));
        Rescale_info_framebuffer(static_cast<int>(window_viewport_current_width), 
                                static_cast<int>(window_viewport_current_height));
        viewport_camera->ScreenResize(window_viewport_current_width,window_viewport_current_height);
        if(button_state_vector[5])
            viewport_camera_projection_matrix = glm::perspective(glm::radians(ViewportCamVfov), (float)window_viewport_current_width / (float)window_viewport_current_height, 0.1f, 100.0f);
        previousImGuiSceneWindowSize = ViewportSize;
    }   
    // and here we can add our created texture as image to ImGui
    // unfortunately we need to use the cast to void* or I didn't find another way tbh
    //ImGui::Image((ImTextureID)textureColorbuffer, ImVec2(framebufferWidth, framebufferHeight));   
    //ImGui::SetCursorPos(ViewportMin);
    // Draw the texture (OpenGL)
    ImGui::GetWindowDrawList()->AddImage(
        (ImTextureID)(intptr_t)viewport_textureColorbuffer,
        ViewportMin,
        ImVec2(ViewportMin.x + ViewportSize.x, ViewportMin.y + ViewportSize.y),
        ImVec2(0, 1), // UVs
        ImVec2(1, 0)  // Flip vertically for OpenGL
    );  
    // Use your camera/view/proj matrices
    draw_imoguizmo();  
    // Tell ImGuizmo where the viewport is  
    draw_guizmo_toolbar(); 

    ImGuizmo::BeginFrame();
    ImGuizmo::SetDrawlist();
    ImGuizmo::SetOrthographic(!button_state_vector[5]);
    ImGuizmo::SetRect(ViewportPos.x, ViewportPos.y, ViewportSize.x, ViewportSize.y);

    float viewMatrix[16];
    float projMatrix[16];
    float modelMatrix[16];
    float deltaMatrix[16];  
    float translation[3],rotation[3],scale[3];  
    memcpy(viewMatrix, glm::value_ptr(viewport_camera_view_matrix), sizeof(float) * 16);
    memcpy(projMatrix, glm::value_ptr(viewport_camera_projection_matrix), sizeof(float) * 16); 

    //ImGuizmo::Manipulate(viewMatrix, projMatrix,ImGuizmo::TRANSLATE, ImGuizmo::LOCAL, modelMatrix,deltaMatrix);
    if(object_selected != ""){
        
        memcpy(modelMatrix, glm::value_ptr(ObjectsVector[object_selected]->object->model_m), sizeof(float) * 16);
        ImGuizmo::ManipulateForward(viewMatrix, projMatrix,currentTransformOp, currentTransformMode, modelMatrix,deltaMatrix);
    } 
    //cube.Rotate(glm::radians(1.0f),glm::vec3(1.0f,0.0f,0.0f));
    //cube.Change_rotation(glm::radians(glm::vec3(0.0f,45.0f,45.0f)));
    //cube.SetTransform();  
    //glm::vec3 rot = cube.Get_rotation();
    //std::cout<<rot.y<<","<<rot.z<<","<<rot.x<<","<<std::endl;
    if (ImGuizmo::IsUsing()){
        isUsingGuizmo = true;
        //cube.Tanslate(glm::vec3(deltaMatrix[12],deltaMatrix[13],deltaMatrix[14]));    
        //ImGuizmo::DecomposeMatrixToComponents(deltaMatrix,translation,rotation,scale);
        //std::cout<<rotation[0]<<","<<rotation[1]<<","<<rotation[2]<<","<<std::endl;
        //cube.Rotate(glm::radians(glm::vec3(rotation[2],rotation[0],rotation[1])));

        glm::mat4 delta_transform(1.0f);

        switch (currentTransformOp)
        {
        case ImGuizmo::TRANSLATE:
            delta_transform[3] = glm::vec4(deltaMatrix[12],deltaMatrix[13],deltaMatrix[14],1.0f); 
            ObjectsVector[object_selected]->object->Tanslate(glm::vec3(deltaMatrix[12],deltaMatrix[13],deltaMatrix[14]));
            break;
        case ImGuizmo::ROTATE:
            memcpy(glm::value_ptr(delta_transform),deltaMatrix, sizeof(float) * 16);
            delta_transform[3] = glm::vec4(0.0f,0.0f,0.0f,1.0f);
            ObjectsVector[object_selected]->object->Rotate(delta_transform);
            break;
        case ImGuizmo::SCALE:
            memcpy(glm::value_ptr(delta_transform),deltaMatrix, sizeof(float) * 16);
            ObjectsVector[object_selected]->object->Scale(delta_transform);
            break;        
        }

        //extract rpy
        //memcpy(deltaMatrix, glm::value_ptr(cube), sizeof(float) * 16);
        //ImGuizmo::DecomposeMatrixToComponents(deltaMatrix,translation,rotation,scale);    
        //cube.Change_rotation(cube.Get_rotation());
        //cube.Scale(glm::vec3(scale[0],scale[1],scale[2]));
        //cube.Change_position(glm::vec3(modelMatrix[12],modelMatrix[13],modelMatrix[14]));

        if(object_selected == "virtual_cam" && currentTransformOp != ImGuizmo::SCALE){
        
            if(currentTransformOp == ImGuizmo::TRANSLATE) plumb_bob_cam->AddCameraTranslate(delta_transform);
            else plumb_bob_cam->AddCameraRotate(delta_transform);

            update_virtual_cam_view_matrix_ubo();
        }
    }else isUsingGuizmo = false;

    //}

    ImGui::End();   
    //global_style_reference.WindowPadding = ImVec2(8.0f, 8.0f);
    //global_style_reference.WindowBorderSize = 2.0f;
    global_style_reference.WindowPadding = global_style_copy.WindowPadding;
    global_style_reference.WindowBorderSize = global_style_copy.WindowBorderSize;

    if(!button_state_vector[5]){

        float ortho_gain_half = std::clamp(viewport_camera->m_pivot_distance*0.6f*0.5f,0.01f,MAXFLOAT);
        float hor_ratio_half = ortho_gain_half*static_cast<float>(window_viewport_current_width)/
                               static_cast<float>(window_viewport_current_height);

        viewport_camera_projection_matrix = glm::ortho(-hor_ratio_half,
            hor_ratio_half,
            -ortho_gain_half,
            ortho_gain_half, 
            -100.0f, 100.0f);
    } 

    update_viewport_cam_view_matrix_ubo();

}

void LidarCamSim::Scene::draw_virtual_camera_settings(){

    ImGui::Begin("Camera Settings",nullptr,ImGuiWindowFlags_None | windowFlags_global_reference);
    bool modfied = false;
    float new_width = plumb_bob_cam->m_windowWidth;
    float new_height = plumb_bob_cam->m_windowHeight;
    float new_Ratio = plumb_bob_cam->RenderRatio;
    // Draw your viewport content (OpenGL/Vulkan texture, etc.)
    ImVec2 size = ImGui::GetContentRegionAvail();
    float spacing_x = global_style_reference.ItemSpacing.x;
    ImGui::Text("Screen Parameters");
    ImGui::PushItemWidth((size.x-spacing_x)/2.0f);
    if(ImGui::DragFloat("##camWidth", &new_width,1.0f, 10.0f,FLT_MAX,"Width: %.0f",ImGuiSliderFlags_AlwaysClamp))modfied = true;
    ImGui::SameLine();
    if(ImGui::DragFloat("##camHeight", &new_height,1.0f, 10.0f,FLT_MAX,"Height: %.0f",ImGuiSliderFlags_AlwaysClamp))modfied = true;
    ImGui::PushItemWidth(size.x);
    if(ImGui::DragFloat("##camratio", &new_Ratio,0.01f, 0.01f,FLT_MAX,"Render Ratio: %.2f",ImGuiSliderFlags_AlwaysClamp))modfied = true;
    if(ImGui::DragFloat("##vcamvfov", &plumb_bob_cam->Vfov,0.1f, 2.0f,178.0f,"Vertical FOV: %.0f°",ImGuiSliderFlags_AlwaysClamp))modfied = true;
    ImGui::PushItemWidth((size.x-2.0f*spacing_x)/3.0f);
    ImGui::Text("Radial Distortion");
    if(ImGui::DragFloat("##rdistor1", &plumb_bob_cam->radialDistortionParams.x, 0.01f,-FLT_MAX,FLT_MAX,"K1: %.3f",ImGuiSliderFlags_AlwaysClamp))modfied = true;
    ImGui::SameLine();
    if(ImGui::DragFloat("##rdistor2", &plumb_bob_cam->radialDistortionParams.y, 0.01f,-FLT_MAX,FLT_MAX,"K2: %.3f",ImGuiSliderFlags_AlwaysClamp))modfied = true;
    ImGui::SameLine();
    if(ImGui::DragFloat("##rdistor3", &plumb_bob_cam->radialDistortionParams.z, 0.01f,-FLT_MAX,FLT_MAX,"K3: %.3f",ImGuiSliderFlags_AlwaysClamp))modfied = true;
    ImGui::PushItemWidth((size.x-spacing_x)/2.0f);
    ImGui::Text("Tangential Distortion");
    if(ImGui::DragFloat("##tdistor1", &plumb_bob_cam->tangentialDistortionParams.x, 0.01f,-FLT_MAX,FLT_MAX,"P1: %.3f",ImGuiSliderFlags_AlwaysClamp))modfied = true;
    ImGui::SameLine();
    if(ImGui::DragFloat("##tdistor2", &plumb_bob_cam->tangentialDistortionParams.y, 0.01f,-FLT_MAX,FLT_MAX,"P2: %.3f",ImGuiSliderFlags_AlwaysClamp))modfied = true;
    ImGui::Text("Broadcast Options");
    ImGui::Checkbox("Broadcast",&camera_broadcast);
    ImGui::PopItemWidth();
    if(modfied){
        plumb_bob_cam->rescale_camera_resolution(new_width,new_height,new_Ratio);
        plumb_bob_cam->RecalculateIntrisicMatrices();
        plumb_bob_cam->resubmitDistortioncoeffs();
        
        update_virtual_cam_projection_matrix_ubo();

        virtual_camera_rescaled = true;
    }

    ImGui::End();

    ImGui::Begin("Camera Projection Matrix",nullptr,ImGuiWindowFlags_None | windowFlags_global_reference);
    ImGui::NewLine();
    float windowWidth = ImGui::GetWindowSize().x;
    float textWidth   = ImGui::CalcTextSize("PROJECTION MATRIX").x;
    ImGui::SetCursorPosX((windowWidth - textWidth) * 0.5f);
    ImGui::Text("PROJECTION MATRIX");
        float line1[4] = {
            plumb_bob_cam->camera_intrinsic[0][0],
            plumb_bob_cam->camera_intrinsic[1][0],
            plumb_bob_cam->camera_intrinsic[2][0],
            plumb_bob_cam->camera_intrinsic[3][0]
        };
        float line2[4] = {
            plumb_bob_cam->camera_intrinsic[0][1],
            plumb_bob_cam->camera_intrinsic[1][1],
            plumb_bob_cam->camera_intrinsic[2][1],
            plumb_bob_cam->camera_intrinsic[3][1]
        };
        float line3[4] = {
            plumb_bob_cam->camera_intrinsic[0][2],
            plumb_bob_cam->camera_intrinsic[1][2],
            plumb_bob_cam->camera_intrinsic[2][2],
            plumb_bob_cam->camera_intrinsic[3][2]
        };
        float line4[4] = {
            plumb_bob_cam->camera_intrinsic[0][3],
            plumb_bob_cam->camera_intrinsic[1][3],
            plumb_bob_cam->camera_intrinsic[2][3],
            plumb_bob_cam->camera_intrinsic[3][3]
        };
        ImGui::SetCursorPosX((windowWidth) * 0.175f);
        ImGui::InputFloat4("##l1p", line1, "%.2f",ImGuiInputTextFlags_ReadOnly);
        ImGui::SetCursorPosX((windowWidth) * 0.175f);
        ImGui::InputFloat4("##l2p", line2, "%.2f",ImGuiInputTextFlags_ReadOnly);
        ImGui::SetCursorPosX((windowWidth) * 0.175f);
        ImGui::InputFloat4("##l3p", line3, "%.2f",ImGuiInputTextFlags_ReadOnly);
        ImGui::SetCursorPosX((windowWidth) * 0.175f);
        ImGui::InputFloat4("##l4p", line4, "%.2f",ImGuiInputTextFlags_ReadOnly);

    ImGui::End();
}

void LidarCamSim::Scene::draw_camera_lidar_matrix(){

    ImGui::Begin("Camera Lidar Matrix",nullptr,ImGuiWindowFlags_None | windowFlags_global_reference);

    ImGui::NewLine();
    float windowWidth = ImGui::GetWindowSize().x;
    float textWidth   = ImGui::CalcTextSize("CAMERA TO LIDAR MATRIX").x;
    ImGui::SetCursorPosX((windowWidth - textWidth) * 0.5f);

    glm::mat4 camera_lidar_m = inverse_normalized_lidar_model*plumb_bob_cam->camera_matrix;

    ImGui::Text("CAMERA TO LIDAR MATRIX");
        float line1[4] = {
            camera_lidar_m[0][0],
            camera_lidar_m[1][0],
            camera_lidar_m[2][0],
            camera_lidar_m[3][0]
        };
        float line2[4] = {
            camera_lidar_m[0][1],
            camera_lidar_m[1][1],
            camera_lidar_m[2][1],
            camera_lidar_m[3][1]
        };
        float line3[4] = {
            camera_lidar_m[0][2],
            camera_lidar_m[1][2],
            camera_lidar_m[2][2],
            camera_lidar_m[3][2]
        };
        float line4[4] = {
            camera_lidar_m[0][3],
            camera_lidar_m[1][3],
            camera_lidar_m[2][3],
            camera_lidar_m[3][3]
        };
        ImGui::SetCursorPosX((windowWidth) * 0.175f);
        ImGui::InputFloat4("##l1clm", line1, "%.3f",ImGuiInputTextFlags_ReadOnly);
        ImGui::SetCursorPosX((windowWidth) * 0.175f);
        ImGui::InputFloat4("##l2clm", line2, "%.3f",ImGuiInputTextFlags_ReadOnly);
        ImGui::SetCursorPosX((windowWidth) * 0.175f);
        ImGui::InputFloat4("##l3clm", line3, "%.3f",ImGuiInputTextFlags_ReadOnly);
        ImGui::SetCursorPosX((windowWidth) * 0.175f);
        ImGui::InputFloat4("##l4clm", line4, "%.3f",ImGuiInputTextFlags_ReadOnly);

    ImGui::End();
}

void LidarCamSim::Scene::draw_lidar_settings(){

    ImGui::Begin("Lidar Settings",nullptr,ImGuiWindowFlags_None | windowFlags_global_reference);

    // Draw your viewport content (OpenGL/Vulkan texture, etc.)
    //ImVec2 size = ImGui::GetContentRegionAvail();
    //ImGui::Text("My win2 %.0fx%.0f", size.x, size.y);

    float button_size = static_cast<float>(monitorScreenHeight)*0.03;

    float ring_count_f = static_cast<float>(Lidar_params.ring_count);

    bool modfied = false;
    //std::size_t lidar_name_size = Lidar_params.model.size()+21;
    //char *Lidar_name = new char[lidar_name_size];
    //std::strcpy(Lidar_name, Lidar_params.model.c_str());

    std::size_t lidar_name_size = 101;
    char Lidar_name[lidar_name_size];
    std::memcpy(Lidar_name,Lidar_params.model.c_str(),Lidar_params.model.size());
    Lidar_name[Lidar_params.model.size()] = '\0';
    
    // Draw your viewport content (OpenGL/Vulkan texture, etc.)
    ImVec2 size = ImGui::GetContentRegionAvail();
    float spacing_x = global_style_reference.ItemSpacing.x;

    ImGui::Text("Lidar Online Parameters");
    ImGui::PushItemWidth((size.x-spacing_x)/2.0f);
    if(ImGui::InputTextWithHint("##LidarName","Lidar Model Name" ,Lidar_name,lidar_name_size,ImGuiInputTextFlags_EnterReturnsTrue))modfied = true;
    ImGui::SameLine();
    ImGui::Checkbox("View Points",&show_raw_lidar_cloud);
    ImGui::PushItemWidth((size.x-spacing_x)/2.0f);
    if(ImGui::DragFloat("##LidarRange", &Lidar_params.range,1.0f, 0.001f,FLT_MAX,"Range: %.1f m",ImGuiSliderFlags_AlwaysClamp))modfied = true;
    ImGui::SameLine();
    if(ImGui::DragFloat("##LidarPrecision", &Lidar_params.lidar_precision,0.01f, 0.0f,FLT_MAX,"Uncertainty: %.3f m",ImGuiSliderFlags_AlwaysClamp))modfied = true;
    ImGui::Text("Lidar Offline Parameters");
    ImGui::PushItemWidth((size.x-spacing_x)/2.0f);
    if(ImGui::DragFloat("##Lidarrings", &ring_count_f, 1.0f,1.0f,FLT_MAX,"Rings: %.0f",ImGuiSliderFlags_AlwaysClamp))modfied = true;
    ImGui::SameLine();
    ImGui::DragFloat("##Lidarrpm", &Lidar_params.rpm, 1.0f,1.0f,FLT_MAX,"RPM: %.0f",ImGuiSliderFlags_AlwaysClamp);
    ImGui::PushItemWidth(size.x);
    ImGui::DragFloat("##Lidarpointspersec", &Lidar_params.points_sec,10.0f, 1.0f,FLT_MAX,"Points/s: %.0f",ImGuiSliderFlags_AlwaysClamp);
    ImGui::PushItemWidth((size.x-spacing_x)/2.0f);
    ImGui::Text("Vertical Angle Ranges");
    ImGui::DragFloat("##Lidarposvrange", &Lidar_params.pos_vertical_fov, 0.1f,-180.0f,180.0f,"VPos Range: %.2f°",ImGuiSliderFlags_AlwaysClamp);
    ImGui::SameLine();
    ImGui::DragFloat("##Lidarnegvrange", &Lidar_params.neg_vertical_fov, 0.1f,-180.0f,180.0f,"VNeg Range: %.2f°",ImGuiSliderFlags_AlwaysClamp);
    ImGui::PushItemWidth((size.x-spacing_x)/2.0f);
    ImGui::Text("Horizontal Angle Ranges");
    ImGui::Checkbox("360° Range",&Lidar_params.is_360);
    if(Lidar_params.is_360)ImGui::BeginDisabled();
    ImGui::DragFloat("##Lidarposhrange", &Lidar_params.pos_horizontal_fov, 0.1f,-180.0f,180.0f,"HPos Range: %.2f°",ImGuiSliderFlags_AlwaysClamp);
    ImGui::SameLine();
    ImGui::DragFloat("##Lidarneghrange", &Lidar_params.neg_horizontal_fov, 0.1f,-180.0f,180.0f,"HNeg Range: %.2f°",ImGuiSliderFlags_AlwaysClamp);
    if(Lidar_params.is_360)ImGui::EndDisabled();
    ImGui::Text("Lidar Debug Options");
    ImGui::PushItemWidth((size.x-spacing_x)/2.0f);
    ImGui::Checkbox("Broadcast",&lidar_broadcast);
    ImGui::SameLine();
    if(ImGui::Checkbox("Receive Cloud",&lidar_recieve_debug_cloud))lidar_view_debug_cloud = (lidar_recieve_debug_cloud)?lidar_view_debug_cloud:false;
    if(!lidar_recieve_debug_cloud)ImGui::BeginDisabled();
    ImGui::Checkbox("View Recieved Cloud",&lidar_view_debug_cloud);
    if(!lidar_recieve_debug_cloud)ImGui::EndDisabled();
    ImGui::PushItemWidth((size.x));

    if(ImGui::Button("Apply",ImVec2(2.0f*button_size,button_size))){

        GLFWwindow* window_x = glfwGetCurrentContext();
        std::cout << "main: "<<(unsigned long)window_x <<std::endl;

        //mutex
        if(RecalculateLidarMutex.try_lock()){
            //disable scan and lidar render and broadcasting
            generating_lidar_buffers = true;
            std::thread Lidar_recalculate_func([=]
            {

            glfwMakeContextCurrent(Lidar_context);
            LidarObject->config.ring_count = Lidar_params.ring_count;
            LidarObject->config.rpm = Lidar_params.rpm;
            LidarObject->config.points_sec = Lidar_params.points_sec;
            LidarObject->config.pos_vertical_fov = Lidar_params.pos_vertical_fov;
            LidarObject->config.neg_vertical_fov = Lidar_params.neg_vertical_fov;
            LidarObject->config.is_360 = Lidar_params.is_360;
            LidarObject->config.pos_horizontal_fov = Lidar_params.pos_horizontal_fov;
            LidarObject->config.neg_horizontal_fov = Lidar_params.neg_horizontal_fov;
            LidarObject->GenerateDirBuffer(0);
            Lidar_dir_buffer_size = static_cast<uint32_t>(LidarObject->direction_buf.size());

            /////////////////
            GLFWwindow* window = glfwGetCurrentContext();
            std::cout << "lidar: "<<(unsigned long)window <<std::endl;
            ///////////////////
            glfwMakeContextCurrent(NULL);
            LidarSendDataMutex.lock();
            points_broadcast_buffer.resize(Lidar_dir_buffer_size);
            RecreateLidarFIFOS();
            LidarSendDataMutex.unlock();
            //enable scan and lidar render and broadcasting
            //generating_lidar_buffers = false;
            generating_lidar_buffers_finished = true;
            RecalculateLidarMutex.unlock();
            
            });

            Lidar_recalculate_func.detach();

        }
        //endopenmp

        //glfwMakeContextCurrent(main_window);

        GLFWwindow* window_y = glfwGetCurrentContext();
        std::cout << "main: "<< (unsigned long)window_y <<std::endl;
        
    }

    if(generating_lidar_buffers_finished){
        LidarObject->BindStorageBuffers();
        generating_lidar_buffers_finished = false;
        generating_lidar_buffers = false; 
    }

    ImGui::PopItemWidth();

    if(modfied){
        Lidar_params.ring_count = static_cast<uint32_t>(ring_count_f);

        //string
        Lidar_params.model.assign(Lidar_name);
        //Lidar_params.model = std::string(Lidar_name);

        LidarObject->config.model = Lidar_params.model;

        LidarObject->config.range = Lidar_params.range;

        LidarObject->config.lidar_precision = Lidar_params.lidar_precision;

    }

    //delete[] Lidar_name;
    ImGui::End();
}

void LidarCamSim::Scene::draw_board_settings(){

    ImGui::Begin("Board Settings",nullptr,ImGuiWindowFlags_None | windowFlags_global_reference);

    // Draw your viewport content (OpenGL/Vulkan texture, etc.)
    //ImVec2 size = ImGui::GetContentRegionAvail();
    //ImGui::Text("My win2 %.0fx%.0f", size.x, size.y);

    //float button_size = static_cast<float>(monitorScreenHeight)*0.03;

    bool modfied = false;

    // Draw your viewport content (OpenGL/Vulkan texture, etc.)
    ImVec2 size = ImGui::GetContentRegionAvail();
    float spacing_x = global_style_reference.ItemSpacing.x;

    glm::vec2 pattern_s = glm::vec2(BoardParams.pattern_size);

    glm::vec3 obj_scale = ObjectsVector["chessboard"]->object->scale*1000.0f;

    if(BoardParams.board_dimension.x != obj_scale.z || 
        BoardParams.board_dimension.y != obj_scale.y)
    {
        BoardParams.board_dimension.x = obj_scale.z;
        BoardParams.board_dimension.y = obj_scale.y;
        modfied = true;
    }

    ImGui::Text("Pattern Size");
    ImGui::PushItemWidth((size.x-spacing_x)/2.0f);
    if(ImGui::DragFloat("##BoardPatternSizeh", &pattern_s.x,1.0f, 1.0f,FLT_MAX,"Horizontal: %.0f",ImGuiSliderFlags_AlwaysClamp))modfied = true;
    ImGui::SameLine();
    if(ImGui::DragFloat("##BoardPatternSizev", &pattern_s.y,1.0f, 1.0f,FLT_MAX,"Vertical: %.0f",ImGuiSliderFlags_AlwaysClamp))modfied = true;
    ImGui::PushItemWidth(size.x);
    ImGui::Text("Square Size");
    if(ImGui::DragFloat("##BoardSquareL", &BoardParams.square_length,1.0f, 0.1f,FLT_MAX,"Size: %.1f mm",ImGuiSliderFlags_AlwaysClamp))modfied = true;
    ImGui::PushItemWidth((size.x-spacing_x)/2.0f);
    ImGui::Text("Board Size");
    if(ImGui::DragFloat("##BoardSizeh", &BoardParams.board_dimension.x,1.0f, 1.0f,FLT_MAX,"Horizontal: %.0f mm",ImGuiSliderFlags_AlwaysClamp))modfied = true;
    ImGui::SameLine();
    if(ImGui::DragFloat("##BoardSizev", &BoardParams.board_dimension.y,1.0f, 1.0f,FLT_MAX,"Vertical: %.0f mm",ImGuiSliderFlags_AlwaysClamp))modfied = true;
    
    ImGui::Text("Board Translation Error");
    if(ImGui::DragFloat("##BoardErrorh", &BoardParams.translation_error.x,1.0f, -FLT_MAX,FLT_MAX,"Horizontal: %.1f mm",ImGuiSliderFlags_AlwaysClamp))modfied = true;
    ImGui::SameLine();
    if(ImGui::DragFloat("##BoardErrorv", &BoardParams.translation_error.y,1.0f, -FLT_MAX,FLT_MAX,"Vertical: %.1f mm",ImGuiSliderFlags_AlwaysClamp))modfied = true;

    ImGui::PopItemWidth();

    if(modfied){
        BoardParams.pattern_size = glm::uvec2(pattern_s);
        ObjectsVector["chessboard"]->object->Change_scale(glm::vec3(1.0f,BoardParams.board_dimension.y/1000.0f,
                                                                    BoardParams.board_dimension.x/1000.0f));
        ShadersVector["chessboardShader"]->use();
        ShadersVector["chessboardShader"]->setVec2("pattern_size",pattern_s);
        ShadersVector["chessboardShader"]->setFloat("square_length",BoardParams.square_length/1000.0f);
        ShadersVector["chessboardShader"]->setVec2("board_dimension",BoardParams.board_dimension/1000.0f);
        ShadersVector["chessboardShader"]->setVec2("translation_error",BoardParams.translation_error/1000.0f);

    }

    ImGui::End();
}

void LidarCamSim::Scene::draw_transform_settings(){

    ImGui::Begin("Object Transform",nullptr,ImGuiWindowFlags_None | windowFlags_global_reference);
    float modfied = false;
    if(object_selected != ""){

    glm::vec3 position = ObjectsVector[object_selected]->object->position; 
    glm::vec3 scale = ObjectsVector[object_selected]->object->scale;
    glm::vec3 orientation;
    orientation.z = glm::degrees(ObjectsVector[object_selected]->object->rotation.x);
    orientation.x = glm::degrees(ObjectsVector[object_selected]->object->rotation.y);
    orientation.y = glm::degrees(ObjectsVector[object_selected]->object->rotation.z);

    ImGui::Text(("Object: " + object_selected).c_str());
    ImGui::NewLine();
    // Draw your viewport content (OpenGL/Vulkan texture, etc.)
    ImGui::Text("Translation");
    // Create sliders for X and Y coordinates
    //ImGui::SliderFloat("Z Position", &board_position.z, -pos_amplitude, pos_amplitude, "Z: %.1f");
    if(ImGui::DragFloat("X Position", &position.x, 0.01f,-FLT_MAX,FLT_MAX,"X: %.2f m",ImGuiSliderFlags_AlwaysClamp))modfied = true;
    if(ImGui::DragFloat("Y Position", &position.y, 0.01f,-FLT_MAX,FLT_MAX,"Y: %.2f m",ImGuiSliderFlags_AlwaysClamp))modfied = true;
    if(ImGui::DragFloat("Z Position", &position.z, 0.01f,-FLT_MAX,FLT_MAX,"Z: %.2f m",ImGuiSliderFlags_AlwaysClamp))modfied = true;

    ImGui::Text("Rotation");

    if(ImGui::DragFloat("X Rotation", &orientation.x,0.1f,-FLT_MAX, FLT_MAX, "X: %.1f°",ImGuiSliderFlags_AlwaysClamp))modfied = true;
    if(ImGui::DragFloat("Y Rotation", &orientation.y,0.1f,-FLT_MAX, FLT_MAX, "Y: %.1f°",ImGuiSliderFlags_AlwaysClamp))modfied = true;
    if(ImGui::DragFloat("Z Rotation", &orientation.z,0.1f,-FLT_MAX, FLT_MAX, "Z: %.1f°",ImGuiSliderFlags_AlwaysClamp))modfied = true;

    ImGui::Text("Scale");

    if(ImGui::DragFloat("X Scale", &scale.x,0.01f,-FLT_MAX, FLT_MAX, "X: %.3f",ImGuiSliderFlags_AlwaysClamp))modfied = true;
    if(ImGui::DragFloat("Y Scale", &scale.y,0.01f,-FLT_MAX, FLT_MAX, "Y: %.3f",ImGuiSliderFlags_AlwaysClamp))modfied = true;
    if(ImGui::DragFloat("Z Scale", &scale.z,0.01f,-FLT_MAX, FLT_MAX, "Z: %.3f",ImGuiSliderFlags_AlwaysClamp))modfied = true;

    if(modfied){
    ObjectsVector[object_selected]->object->Change_position(position);
    ObjectsVector[object_selected]->object->Change_scale(scale);
    ObjectsVector[object_selected]->object->Change_rotation(glm::radians(glm::vec3(orientation.z,orientation.x,orientation.y)));

    if(object_selected == "virtual_cam"){
        plumb_bob_cam->RecalculateCameraNormalize(ObjectsVector[object_selected]->object->model_m);
        update_virtual_cam_view_matrix_ubo();
    }
    }
    }else{
        ImGui::Text("Object: No Object Selected!");
    }

    ImGui::End();
     
    ImGui::Begin("Object Model Matrix",nullptr,ImGuiWindowFlags_None | windowFlags_global_reference);
    if(object_selected != ""){

    ImGui::Text(("Object: " + object_selected).c_str());
    ImGui::NewLine();
    ImGui::NewLine();

    float windowWidth = ImGui::GetWindowSize().x;
    float textWidth   = ImGui::CalcTextSize("MODEL MATRIX").x;
    ImGui::SetCursorPosX((windowWidth - textWidth) * 0.5f);
    ImGui::Text("MODEL MATRIX");
    // Draw your viewport content (OpenGL/Vulkan texture, etc.)
        float line1[4] = {
            ObjectsVector[object_selected]->object->model_m[0][0],
            ObjectsVector[object_selected]->object->model_m[1][0],
            ObjectsVector[object_selected]->object->model_m[2][0],
            ObjectsVector[object_selected]->object->model_m[3][0]
        };
        float line2[4] = {
            ObjectsVector[object_selected]->object->model_m[0][1],
            ObjectsVector[object_selected]->object->model_m[1][1],
            ObjectsVector[object_selected]->object->model_m[2][1],
            ObjectsVector[object_selected]->object->model_m[3][1]
        };
        float line3[4] = {
            ObjectsVector[object_selected]->object->model_m[0][2],
            ObjectsVector[object_selected]->object->model_m[1][2],
            ObjectsVector[object_selected]->object->model_m[2][2],
            ObjectsVector[object_selected]->object->model_m[3][2]
        };
        float line4[4] = {
            ObjectsVector[object_selected]->object->model_m[0][3],
            ObjectsVector[object_selected]->object->model_m[1][3],
            ObjectsVector[object_selected]->object->model_m[2][3],
            ObjectsVector[object_selected]->object->model_m[3][3]
        };
        ImGui::SetCursorPosX((windowWidth) * 0.175f);
        ImGui::InputFloat4("##l1m", line1, "%.3f",ImGuiInputTextFlags_ReadOnly);
        ImGui::SetCursorPosX((windowWidth) * 0.175f);
        ImGui::InputFloat4("##l2m", line2, "%.3f",ImGuiInputTextFlags_ReadOnly);
        ImGui::SetCursorPosX((windowWidth) * 0.175f);
        ImGui::InputFloat4("##l3m", line3, "%.3f",ImGuiInputTextFlags_ReadOnly);
        ImGui::SetCursorPosX((windowWidth) * 0.175f);
        ImGui::InputFloat4("##l4m", line4, "%.3f",ImGuiInputTextFlags_ReadOnly);
    }
    else{
        ImGui::Text("Object: No Object Selected!");
    }
    ImGui::End();
}

void LidarCamSim::Scene::draw_virtual_camera_viewport(){

    global_style_reference.WindowPadding = ImVec2(0.0f, 0.0f);
    global_style_reference.WindowBorderSize = 0.0f;

    ImGui::Begin("Camera Viewport");

            // we access the ImGui window size
        const ImVec2 currentImGuiWindowSize_cam = ImGui::GetContentRegionAvail();
        const ImVec2 viewportMin_cam = ImGui::GetCursorScreenPos();
        const ImVec2 viewportMax_cam = ImVec2(viewportMin_cam.x + currentImGuiWindowSize_cam.x,viewportMin_cam.y + currentImGuiWindowSize_cam.y);

        ImVec2 viewportMin_cam_view;
        ImVec2 viewportMax_cam_view;

        //ImGui::SetCursorPos(viewportMin_cam);

        float image_size_y = (plumb_bob_cam->m_windowHeight/plumb_bob_cam->m_windowWidth)*currentImGuiWindowSize_cam.x;
        float image_size_x;
        if(image_size_y > currentImGuiWindowSize_cam.y){
            image_size_y = currentImGuiWindowSize_cam.y;
            image_size_x = (plumb_bob_cam->m_windowWidth/plumb_bob_cam->m_windowHeight)*currentImGuiWindowSize_cam.y;

            viewportMin_cam_view.x = viewportMin_cam.x+(currentImGuiWindowSize_cam.x - image_size_x)/2.0f;
            viewportMin_cam_view.y = viewportMin_cam.y;
            viewportMax_cam_view = ImVec2(viewportMin_cam_view.x + image_size_x,viewportMin_cam_view.y + image_size_y);
        }
        else{
            image_size_x = currentImGuiWindowSize_cam.x;
            viewportMin_cam_view.x = viewportMin_cam.x;
            viewportMin_cam_view.y = viewportMin_cam.y+(currentImGuiWindowSize_cam.y - image_size_y)/2.0f;
            viewportMax_cam_view = ImVec2(viewportMin_cam_view.x + image_size_x,viewportMin_cam_view.y + image_size_y);
        }

        ImGui::GetWindowDrawList()->AddImage(
            (ImTextureID)(intptr_t)plumb_bob_cam->CameratextureFinalColorbuffer,
            viewportMin_cam_view,//viewportMin_cam,
            viewportMax_cam_view,//viewportMax_cam,
            ImVec2(0, 1), // UVs
            ImVec2(1, 0)  // Flip vertically for OpenGL
        );


    ImGui::End();

    //global_style_reference.WindowPadding = ImVec2(8.0f, 8.0f);
    //global_style_reference.WindowBorderSize = 2.0f;

    global_style_reference.WindowPadding = global_style_copy.WindowPadding;
    global_style_reference.WindowBorderSize = global_style_copy.WindowBorderSize;

    ImGui::Begin("Statistics");

    ImGui::PushFont(font_mono);
    ImGui::SetWindowFontScale(0.9f);
    ImGui::Text("Framerate: %.1f FPS (%.3f ms/frame) ", io_reference.Framerate, 1000.0f / io_reference.Framerate);
    ImGui::SetWindowFontScale(1.0f);
    ImGui::PopFont();

    ImGui::End();

}

void LidarCamSim::Scene::processInput(const float &deltaTime)
{
    if (glfwGetKey(main_window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(main_window, true);

    if(scene_hovered && button_state_vector[4]){
        if (glfwGetKey(main_window, GLFW_KEY_W) == GLFW_PRESS)
            (button_state_vector[5])?
            viewport_camera->ProcessKeyboard(FORWARD, deltaTime):
            viewport_camera->ProcessKeyboardPivotDistance(FORWARD, deltaTime);
        if (glfwGetKey(main_window, GLFW_KEY_S) == GLFW_PRESS)
            (button_state_vector[5])?
            viewport_camera->ProcessKeyboard(BACKWARD, deltaTime):
            viewport_camera->ProcessKeyboardPivotDistance(BACKWARD, deltaTime);
        if (glfwGetKey(main_window, GLFW_KEY_A) == GLFW_PRESS)
            viewport_camera->ProcessKeyboard(LEFT, deltaTime);
        if (glfwGetKey(main_window, GLFW_KEY_D) == GLFW_PRESS)
            viewport_camera->ProcessKeyboard(RIGHT, deltaTime);
        if (glfwGetKey(main_window, GLFW_KEY_Q) == GLFW_PRESS)
            viewport_camera->ProcessKeyboard(CAMERA_YAW_NEG, deltaTime);
        if (glfwGetKey(main_window, GLFW_KEY_E) == GLFW_PRESS)
            viewport_camera->ProcessKeyboard(CAMERA_YAW_POS, deltaTime);
        if (glfwGetKey(main_window, GLFW_KEY_R) == GLFW_PRESS)
            viewport_camera->ProcessKeyboard(CAMERA_PITCH_POS, deltaTime);
        if (glfwGetKey(main_window, GLFW_KEY_F) == GLFW_PRESS)
            viewport_camera->ProcessKeyboard(CAMERA_PITCH_NEG, deltaTime);
        if(ImGui::IsKeyPressed(ImGuiKey_P,false) && !isUsingGuizmo)
            want_to_toggle_proj = true;
                
        if (ImGui::IsKeyPressed(ImGuiKey_M,false) && !isUsingGuizmo && !isDragging){
            button_state_vector[4] = !button_state_vector[4];
            viewport_camera->fly_mode = button_state_vector[4];
        }
    }
    else if(scene_hovered && !isUsingGuizmo){

        
        //if (glfwGetKey(main_window, GLFW_KEY_G) == GLFW_PRESS)
        if(ImGui::IsKeyPressed(ImGuiKey_G,false))
        {
            currentTransformOp = ImGuizmo::TRANSLATE;
            button_state_vector[0] = true;
            button_state_vector[1] = false;
            button_state_vector[2] = false; 
        } 
        //if (glfwGetKey(main_window, GLFW_KEY_R) == GLFW_PRESS)
        if(ImGui::IsKeyPressed(ImGuiKey_R,false))
        {
            currentTransformOp = ImGuizmo::ROTATE;
            button_state_vector[0] = false;
            button_state_vector[1] = true;
            button_state_vector[2] = false;
        }
        //if (glfwGetKey(main_window, GLFW_KEY_S) == GLFW_PRESS)
        if(ImGui::IsKeyPressed(ImGuiKey_S,false))
        {
            currentTransformOp = ImGuizmo::SCALE;
            button_state_vector[0] = false;
            button_state_vector[1] = false;
            button_state_vector[2] = true;   
        }        
        //if (glfwGetKey(main_window, GLFW_KEY_W) == GLFW_PRESS)
        if(ImGui::IsKeyPressed(ImGuiKey_W,false))
        {
            button_state_vector[3] = !button_state_vector[3];
            currentTransformMode = (button_state_vector[3])?ImGuizmo::WORLD:ImGuizmo::LOCAL;
        }

        //if (glfwGetKey(main_window, GLFW_KEY_P) == GLFW_PRESS)
        if(ImGui::IsKeyPressed(ImGuiKey_P,false))
            want_to_toggle_proj = true;
        
        if (ImGui::IsKeyPressed(ImGuiKey_M,false) && !isDragging){
            button_state_vector[4] = !button_state_vector[4];
            viewport_camera->fly_mode = button_state_vector[4];
        }
            

    }

    if (glfwGetKey(main_window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS && !isDragging)
        shift_pressed = true;
    else if(glfwGetKey(main_window, GLFW_KEY_LEFT_SHIFT) == GLFW_RELEASE && !isDragging)
        shift_pressed = false; 
        
    if (glfwGetMouseButton(main_window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS && !keyPressed && scene_hovered){

        keyPressed = true;
        viewport_camera->firstMouse = true;

        glfwGetCursorPos(main_window, &global_cursor_xpos, &global_cursor_ypos);

        glfwSetCursorPosCallback(main_window, mouse_callback);
        glfwSetMouseButtonCallback(main_window, nullptr);

        ImGui::GetIO().MouseDown[0] = 0;
        ImGui::GetIO().MouseDown[1] = 0;

        //change cursor icon
        isDragging = true;
        glfwSetInputMode(main_window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
        (button_state_vector[4] && !shift_pressed)?
        glfwSetCursor(main_window, invisible_cursor):
        glfwSetCursor(main_window, dragCursor);
    } 
    //or just consider processing only at each frame 
    //else if (glfwGetKey(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS && keyPressed && scene_hovered){
    //    //camera.ProcessMouseMovement();
    //}
    else if (glfwGetMouseButton(main_window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_RELEASE || !scene_hovered){

        keyPressed=false;
        glfwSetCursorPosCallback(main_window, ImGui_ImplGlfw_CursorPosCallback);
        glfwSetMouseButtonCallback(main_window, ImGui_ImplGlfw_MouseButtonCallback);

        //restore cursor icon
        isDragging = false;
        if(glfwGetInputMode(main_window, GLFW_CURSOR) == GLFW_CURSOR_DISABLED) glfwSetCursor(main_window, arrowCursor);
        glfwSetInputMode(main_window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
    }
    

    if(scene_hovered && !scroll_callback_installed){
        glfwSetScrollCallback(main_window, scroll_callback);
        scroll_callback_installed = true;
    }
    else if(!scene_hovered && scroll_callback_installed){
        glfwSetScrollCallback(main_window, ImGui_ImplGlfw_ScrollCallback);
        scroll_callback_installed = false;

    }


    //////if (glfwGetKey(window, GLFW_KEY_G) == GLFW_PRESS && !keyPressed){
    //////    
    //////    capture_mouse = !capture_mouse;
    //////    keyPressed = true;

    //////    if(capture_mouse){
    //////        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    //////        glfwSetCursorPosCallback(window, mouse_callback);
    //////        //glfwSetCursorPosCallback(window, nullptr);
    //////        glfwSetMouseButtonCallback(window, nullptr);
    //////        ImGui::GetIO().MouseDown[0] = 0;
    //////        ImGui::GetIO().MouseDown[1] = 0;
    //////        camera.firstMouse = true;
    //////        fly_mode = true;
    //////    }
    //////    else{
    //////        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
    //////        glfwSetCursorPosCallback(window, ImGui_ImplGlfw_CursorPosCallback);
    //////        glfwSetMouseButtonCallback(window, ImGui_ImplGlfw_MouseButtonCallback);
    //////        fly_mode = false;
    //////    }
    //////} 
    //////else if (glfwGetKey(window, GLFW_KEY_G) == GLFW_RELEASE){
    //////    keyPressed = false;  // Reset when key is released
    //////}

}

void LidarCamSim::Scene::InitLidar(const Lidar_config &lidar_cfg,const glm::vec3 &Pos,const float &roll,const float &pitch,const float &yaw){

    Lidar_params = lidar_cfg;
    ComputeShadersVector["lscanCompute"] = std::make_shared<Shader_compute>("../shaders/compute/lscan.comp");

    std::string default_lidar_shader_path = "../shaders/gui_test/uniforms/";
    ShadersVector["lidar_model_shader"] = std::make_shared<Shader>((default_lidar_shader_path+"lidar.vs").c_str(), (default_lidar_shader_path+"lidar.fs").c_str());
    ShadersVector["lidar_point_shader"] = std::make_shared<Shader>((default_lidar_shader_path+"point_sh.vs").c_str(), (default_lidar_shader_path+"point_sh.fs").c_str());

    ModelsVector["lidar_model"] = std::make_shared<Model>("../resources/models/lidar/lidar.obj");

    std::shared_ptr<sceneObject> lidar_mesh = std::make_shared<sceneObject>();

    lidar_mesh->object = std::make_shared<object_model>(Pos,
                                                    glm::radians(glm::vec3(roll,pitch,yaw)),
                                                    *(ShadersVector["lidar_model_shader"].get()),
                                                    *(ModelsVector["lidar_model"].get()),DRAW_MODEL);

    lidar_mesh->cameras_for_render["viewport_cam"] = true; 
    lidar_mesh->texture_transform_support = false;
    lidar_mesh->auto_render = true;
    ObjectsVector["lidar_object"] = lidar_mesh;


    lidar_mesh->object->SetInfoShader(&InfoShader);
    lidar_mesh->object->SetOutlineShader(&OutlineShader);
    lidar_mesh->object->SetDrawInfoMode(DRAW_MODEL_INFO);
    lidar_mesh->object->SetDrawOutlineMode(DRAW_MODEL_OUTLINE);

    //Pos here is useless, origin is not used
    LidarObject = std::make_unique<SensorScene>(*(ComputeShadersVector["lscanCompute"].get()),Lidar_params,Pos);

    LidarObject->GenerateDirBuffer(1);

    Lidar_dir_buffer_size = static_cast<uint32_t>(LidarObject->direction_buf.size());
    points_broadcast_buffer.resize(Lidar_dir_buffer_size);
    
    ShadersVector["lidar_point_shader"]->use();
    unsigned int point_shID = ShadersVector["lidar_point_shader"]->ID; 
    // then we link each shader's uniform block to this uniform binding point
    glUniformBlockBinding(point_shID, glGetUniformBlockIndex(point_shID, "Matrices_cam"), ProjViewBindingViewportCamera);

}

void LidarCamSim::Scene::LidarAddObject(const string &object_name,const bool &visible){

    LidarObject->addObject((ObjectsVector[object_name]->object).get(),visible,object_name);
}

void LidarCamSim::Scene::LidarDrawPoints(){

    if(show_raw_lidar_cloud && !generating_lidar_buffers){
    glBindVertexArray(dummyVAO);   
    ShadersVector["lidar_point_shader"]->use();
    ShadersVector["lidar_point_shader"]->setUint("ring_count",LidarObject->config.ring_count -1U);
    ShadersVector["lidar_point_shader"]->setFloat("range",LidarObject->config.range);
    ShadersVector["lidar_point_shader"]->setMat4("model_lidar", normalized_lidar_model);

    RenderCloud(*(ShadersVector["lidar_point_shader"].get()),Lidar_dir_buffer_size);

    glBindVertexArray(0);
    }

}

void LidarCamSim::Scene::LidarCalculateNormalizedModel(){
    normalized_lidar_model = ObjectsVector["lidar_object"]->object->model_m;
    normalized_lidar_model[0] = glm::normalize(normalized_lidar_model[0]);
    normalized_lidar_model[1] = glm::normalize(normalized_lidar_model[1]);
    normalized_lidar_model[2] = glm::normalize(normalized_lidar_model[2]);
    inverse_normalized_lidar_model = glm::inverse(normalized_lidar_model);
}

void LidarCamSim::Scene::LidarDoScan(const float &time){
    if(!generating_lidar_buffers){
    LidarObject->LidarScan(inverse_normalized_lidar_model, time);
    if(lidar_broadcast)SendLidarCloudAsync();
    }
}

void LidarCamSim::Scene::SendLidarCloudAsync(){

        
    if(LidarSendDataMutex.try_lock()){

        if(send_queue->get_max_msg() > send_queue->get_num_msg()){
            
            uint32_t buffer_size = Lidar_dir_buffer_size;

            glfwMakeContextCurrent(Lidar_context);
            glBindBuffer(GL_SHADER_STORAGE_BUFFER, lidar_context_ssbo);
            glBufferData(GL_SHADER_STORAGE_BUFFER, buffer_size*sizeof(Point_XYZIR), nullptr, GL_DYNAMIC_DRAW);
            glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
            glBindBuffer(GL_COPY_READ_BUFFER, LidarObject->GetPointBufferOBJ());
            glBindBuffer(GL_COPY_WRITE_BUFFER, lidar_context_ssbo);
            glCopyBufferSubData(GL_COPY_READ_BUFFER, GL_COPY_WRITE_BUFFER, 0, 0, buffer_size*sizeof(Point_XYZIR));

            glfwMakeContextCurrent(main_window);

            std::thread Lidar_send_data_func([=]
            {   
                glfwMakeContextCurrent(Lidar_context);

                //points_broadcast_buffer.resize(buffer_size);
                glBindBuffer(GL_SHADER_STORAGE_BUFFER, lidar_context_ssbo);
                glGetBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, buffer_size*sizeof(Point_XYZIR), points_broadcast_buffer.data());
                glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);

                std::cout << points_broadcast_buffer.at(55500).x << ", " 
                << points_broadcast_buffer.at(55500).y << ", "
                << points_broadcast_buffer.at(55500).z << ", "
                << points_broadcast_buffer.at(55500).ring << std::endl;

                send_queue->try_send(points_broadcast_buffer.data(), buffer_size*sizeof(Point_XYZIR), /*priority*/ 0);
                glfwMakeContextCurrent(NULL);
                LidarSendDataMutex.unlock();
            });

            Lidar_send_data_func.detach();
        }
        else{
            LidarSendDataMutex.unlock();
        }
    }
        

}

void LidarCamSim::Scene::SendLidarCloud(){

    if(send_queue->get_max_msg() > send_queue->get_num_msg()){

        LidarObject->TransferLidarScan();

            std::cout << LidarObject->points.at(55500).x << ", " 
            << LidarObject->points.at(55500).y << ", "
            << LidarObject->points.at(55500).z << ", "
            << LidarObject->points.at(55500).ring << std::endl;

        send_queue->try_send(LidarObject->points.data(), fifo_msg_send_size, /*priority*/ 0);
    }

}

void LidarCamSim::Scene::SendCameraImageAsync(){

if(camera_broadcast)
{
    //virtual_camera_rescaled
        
    if(CameraSendDataMutex.try_lock()){

        if(camera_send_queue->get_max_msg() > camera_send_queue->get_num_msg() || virtual_camera_rescaled){

            glfwMakeContextCurrent(Camera_context);

            bool rescaled = virtual_camera_rescaled;
            virtual_camera_rescaled = false;

            unsigned int width = static_cast<unsigned int>(plumb_bob_cam->m_windowWidth);
            unsigned int height = static_cast<unsigned int>(plumb_bob_cam->m_windowHeight);

            if(rescaled){
            
                camera_message_size = width*
                                      height*
                                      3U*sizeof(float);

                glBindBuffer(GL_PIXEL_PACK_BUFFER, camera_context_pbo);
                glBufferData(GL_PIXEL_PACK_BUFFER, camera_message_size, nullptr, GL_STREAM_READ);
                glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);

                
            }

            glBindFramebuffer(GL_READ_FRAMEBUFFER, plumb_bob_cam->imGenbuffer);
            glBindBuffer(GL_PIXEL_PACK_BUFFER, camera_context_pbo);
            glReadPixels(0, 0, width, height, GL_RGB, GL_FLOAT, 0);
            glfwMakeContextCurrent(main_window);

            

            std::thread Camera_send_data_func([=]
            {   
                glfwMakeContextCurrent(Camera_context);
                std::cout<<"cam_msg_size:"<<camera_message_size<<std::endl;
                if(rescaled){
                    RecreateCameraFIFOS();
                    Camera_cpu_buffer.resize(camera_message_size/sizeof(float));
                }
                //points_broadcast_buffer.resize(buffer_size);
                glBindBuffer(GL_PIXEL_PACK_BUFFER, camera_context_pbo);
                

                float* ptr = (float*)glMapBuffer(GL_PIXEL_PACK_BUFFER, GL_READ_ONLY);

                if (ptr) {

                    memcpy(Camera_cpu_buffer.data(), ptr, width * height * 3 * sizeof(float));
                    glUnmapBuffer(GL_PIXEL_PACK_BUFFER);

                    std::cout << "Image transfered to the cpu" << std::endl;

                    camera_send_queue->try_send(Camera_cpu_buffer.data(), camera_message_size, /*priority*/ 0);
                }

                glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);

                glfwMakeContextCurrent(NULL);
                CameraSendDataMutex.unlock();
            });

            Camera_send_data_func.detach();
        }
        else{
            CameraSendDataMutex.unlock();
        }
    }
        
}
}

void LidarCamSim::Scene::InitFIFOS(){
    fifo_msg_send_size = LidarObject->points.size()*sizeof(Point_XYZIR);
    fifo_msg_recieve_size = LidarObject->points.size()*3*sizeof(float);

    boost::interprocess::message_queue::remove("pts_queue");
    send_queue = std::make_unique<boost::interprocess::message_queue>(
                                    boost::interprocess::create_only,
                                    "pts_queue",
                                    /*max msgs*/  5,
                                    /*max size*/  fifo_msg_send_size);

    boost::interprocess::message_queue::remove("pts_queue_result");
    recieve_queue = std::make_unique<boost::interprocess::message_queue>(
                                    boost::interprocess::create_only,
                                    "pts_queue_result",
                                    /*max msgs*/  5,
                                    /*max size*/  fifo_msg_recieve_size);

    camera_message_size = static_cast<unsigned int>(plumb_bob_cam->m_windowWidth)*
                                       static_cast<unsigned int>(plumb_bob_cam->m_windowHeight)*
                                       3U*sizeof(float);

    boost::interprocess::message_queue::remove("camera_queue");
    camera_send_queue = std::make_unique<boost::interprocess::message_queue>(
                                    boost::interprocess::create_only,
                                    "camera_queue",
                                    /*max msgs*/  5,
                                    /*max size*/  camera_message_size); 
}

void LidarCamSim::Scene::RecreateLidarFIFOS(){


    fifo_msg_send_size = LidarObject->points.size()*sizeof(Point_XYZIR);
    fifo_msg_recieve_size = LidarObject->points.size()*3*sizeof(float);

    boost::interprocess::message_queue::remove("pts_queue");
    send_queue.reset();
    send_queue = std::make_unique<boost::interprocess::message_queue>(
                                    boost::interprocess::create_only,
                                    "pts_queue",
                                    /*max msgs*/  5,
                                    /*max size*/  fifo_msg_send_size);

    boost::interprocess::message_queue::remove("pts_queue_result");
    recieve_queue.reset();
    recieve_queue = std::make_unique<boost::interprocess::message_queue>(
                                    boost::interprocess::create_only,
                                    "pts_queue_result",
                                    /*max msgs*/  5,
                                    /*max size*/  fifo_msg_recieve_size);

    //boost::interprocess::message_queue::remove("pts_queue_result");
    //boost::interprocess::message_queue mq_inliers(boost::interprocess::create_only,
    //                                                           "pts_queue_result",
    //                                                           /*max msgs*/  10,
    //                                                           /*max size*/  fifo_recieve_size);
}


void LidarCamSim::Scene::RecreateCameraFIFOS(){

    boost::interprocess::message_queue::remove("camera_queue");
    camera_send_queue.reset();
    camera_send_queue = std::make_unique<boost::interprocess::message_queue>(
                                    boost::interprocess::create_only,
                                    "camera_queue",
                                    /*max msgs*/  5,
                                    /*max size*/  camera_message_size); 

}
//void LidarCamSim::Scene::RecieveLidarProcessedResult(){
//
//    uint64_t recvd_size;
//    uint32_t priority;
//
//    if(recieve_queue->try_receive(lidar_inliers.data(), lidar_inliers.size()*sizeof(glm::vec3), recvd_size, priority)){
//        //lidar_points.resize(recvd_size / sizeof(Point_XYZIR));
//
//        glBindVertexArray(inliersVAO);
//        glBindBuffer(GL_ARRAY_BUFFER, inliersVBO);
//        glBufferSubData(GL_ARRAY_BUFFER, 0, recvd_size, lidar_inliers.data());
//
//        inliers_sh.use();
//
//        inliers_sh.setMat4("model_lidar", lidar_model);
//
//        if(show_lidar_points_borders) glDrawArrays(GL_POINTS, 0, recvd_size/sizeof(glm::vec3));
//
//        glBindVertexArray(0);
//
//    }
//}

void LidarCamSim::Scene::InitializeDefaultBoard(){

    float board_width = 0.705;
    float board_height = 1.0f;
    float translation_error_x = 0.002f;
    float translation_error_y = 0.002f;

    glm::vec2 texture_c(0.5,0.5);

    float texture_width = 0.9f;
    float texture_height = 1.2f;

    float norm_width_b = board_width/texture_width;
    float norm_height_b = board_height/texture_height;

    glm::vec2 board_c(norm_width_b*0.5f,norm_height_b*0.5f);

    glm::vec2 center_dist = texture_c - board_c;

    //glm::vec2 board_tex_scale(norm_width_b,norm_height_b);
    glm::vec2 board_tex_scale(norm_width_b,norm_height_b);

    glm::vec2 norm_tranlation_error(translation_error_x/texture_width,translation_error_y/texture_height);

    std::string path_plane = "../shaders/project2/";
    shaderConfig shader_cfg;
    shader_cfg.vertex_path = (path_plane+"plane.vs").c_str(),
    shader_cfg.fragment_path = (path_plane+"plane.fs").c_str(),
    shader_cfg.type = shader_type::DEFAULT,
    shader_cfg.shader_name = "plane_shader",


    AddShader(shader_cfg);

    std::shared_ptr<plane> object_ptr = std::make_shared<plane>(glm::vec3(0.0,1.0,0.0), 
                                                                *(ShadersVector["plane_shader"]),
                                                                DRAW_TEXTURED);

    AddObject(object_ptr,"Board");

   

     ObjectsVector["Board"]->object->Change_scale(glm::vec3(board_width,1.0,board_height));
    // ObjectsVector["Board"]->object->Rotate(glm::radians(-45.0),glm::vec3(0.0,1.0,0.0));
     ObjectsVector["Board"]->object->Rotate(glm::radians(-90.0),glm::vec3(1.0,0.0,0.0));
     ObjectsVector["Board"]->object->Rotate(glm::radians(90.0),glm::vec3(0.0,1.0,0.0));
     ObjectsVector["Board"]->object->ApplyRotation();
     ObjectsVector["Board"]->object->Rotate(glm::radians(45.0),glm::vec3(1.0,0.0,0.0));
    // ObjectsVector["Board"]->object->RotateTexture(glm::radians(-90.0));
    // ObjectsVector["Board"]->object->ApplyRotationTexture();
     ObjectsVector["Board"]->object->ScaleTexture(board_tex_scale);
     ObjectsVector["Board"]->object->PositionateTexture(center_dist - norm_tranlation_error);

     //ObjectsVector["Board"]->object->SetTransform();
     //ObjectsVector["Board"]->object->SetTextureTransform();

    //board_position = Board.Get_position();
    //glm::vec3 board_angles_rpy = glm::degrees(Board.Get_rotation());
    //board_orientation = glm::vec3(board_angles_rpy.y, board_angles_rpy.z, board_angles_rpy.x);
    //board_scale = Board.Get_scale();
    // load textures
    // -------------
    //glFlush();
    unsigned int chessTexture  = loadTexture("../resources/textures/chessboard.jpg");
    ObjectsVector["Board"]->object->SetTexture(chessTexture);
    ObjectsVector["Board"]->AddTexture(chessTexture,"texture_diffuse1");
    ObjectsVector["Board"]->texture_transform_support = true;
    ObjectsVector["Board"]->auto_render = true;
}

// glfw: whenever the mouse moves, this callback is called
// -------------------------------------------------------
void mouse_callback(GLFWwindow* window, double xposIn, double yposIn)
{
    float xpos = static_cast<float>(xposIn);
    float ypos = static_cast<float>(yposIn);


    //cout << "("<<xpos<<","<<ypos<<")"<<"("<<ViewportPos.x<<","<<ViewportPos.y<<")" << std::endl;
    
    //camera.ProcessMouseMovement(xoffset, yoffset);
    main_scene_class->viewport_camera->ProcessMouseMovement(xpos, ypos, main_scene_class->shift_pressed);
    //if out of bounds teleport && camera set initial position

    //if(xpos > ){
    //xpos = fmod(xpos - ViewportPos.x, WindowSize.x) + ViewportPos.x;
    //ypos = fmod(ypos - ViewportPos.y, WindowSize.y) + ViewportPos.y;
    //glfwSetCursorPos(window, xpos, ypos);
    //camera.firstMouse;
    //}

    if(xpos < main_scene_class->ViewportPos.x){
        glfwSetCursorPos(window, xpos + main_scene_class->ViewportSize.x, ypos);
        xpos += main_scene_class->ViewportSize.x;
        main_scene_class->viewport_camera->firstMouse = true;
    } else if(xpos > (main_scene_class->ViewportPos.x+main_scene_class->ViewportSize.x)){
        glfwSetCursorPos(window, xpos - main_scene_class->ViewportSize.x, ypos);
        xpos -= main_scene_class->ViewportSize.x;
        main_scene_class->viewport_camera->firstMouse = true;
    }
    if(ypos < main_scene_class->ViewportPos.y){
        glfwSetCursorPos(window, xpos, ypos + main_scene_class->ViewportSize.y);
        main_scene_class->viewport_camera->firstMouse = true;
    } else if(ypos > (main_scene_class->ViewportPos.y+main_scene_class->ViewportSize.y)){
        glfwSetCursorPos(window, xpos, ypos - main_scene_class->ViewportSize.y);
        main_scene_class->viewport_camera->firstMouse = true;
    }
    
    main_scene_class->global_cursor_xpos = static_cast<double>(xpos);
    main_scene_class->global_cursor_ypos = static_cast<double>(ypos);

}

// glfw: whenever the mouse scroll wheel scrolls, this callback is called
// ----------------------------------------------------------------------
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    main_scene_class->viewport_camera->ProcessMouseScroll(static_cast<float>(yoffset));
}
