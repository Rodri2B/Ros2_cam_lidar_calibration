#include "calib_scene.hpp"


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
                            OutlineShader("../shaders/gui_test/uniforms/outline_mask.vs", "../shaders/gui_test/uniforms/outline_mask.fs"),
                            DrawOutlineShader("../shaders/gui_test/uniforms/draw_outline.vs", "../shaders/gui_test/uniforms/draw_outline.fs"),
                            VirtaulCamScreenShader("../shaders/gui_test/screen.vs", "../shaders/gui_test/screen.fs"),
                            CameraDebugBoardShader("../shaders/gui_test/uniforms/camera_lidar_board.vs", "../shaders/gui_test/uniforms/camera_lidar_board.fs")

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

    /////create ubo

    glGenBuffers(1, &uboViewportCamera);
    glBindBuffer(GL_UNIFORM_BUFFER, uboViewportCamera);
    unsigned long ubuffer_size =  2 * sizeof(glm::mat4);
    glBufferData(GL_UNIFORM_BUFFER, ubuffer_size, NULL, GL_STATIC_DRAW);
    glBindBuffer(GL_UNIFORM_BUFFER, 0);

    glGenBuffers(1, &uboVirtualCamera);
    glBindBuffer(GL_UNIFORM_BUFFER, uboVirtualCamera);
    glBufferData(GL_UNIFORM_BUFFER, ubuffer_size, NULL, GL_STATIC_DRAW);
    glBindBuffer(GL_UNIFORM_BUFFER, 0);

    //ubuffer_size = sizeof(gpu_lidar_detected_board_buffer);
    ubuffer_size = 6*sizeof(glm::vec4);

    glGenBuffers(1, &uboDebugBoard);
    glBindBuffer(GL_UNIFORM_BUFFER, uboDebugBoard);
    glBufferData(GL_UNIFORM_BUFFER, ubuffer_size, NULL, GL_STATIC_DRAW);
    glBindBuffer(GL_UNIFORM_BUFFER, 0);

    if (global_uniform_binding_indexes->size() >= 3)
    {   
        ProjViewBindingViewportCamera = global_uniform_binding_indexes->back();

        glBindBufferRange(GL_UNIFORM_BUFFER, ProjViewBindingViewportCamera, uboViewportCamera, 0, 2 * sizeof(glm::mat4));

        global_uniform_binding_indexes->pop_back();

        ProjViewBindingVirtualCamera = global_uniform_binding_indexes->back();

        glBindBufferRange(GL_UNIFORM_BUFFER, ProjViewBindingVirtualCamera, uboVirtualCamera, 0, 2 * sizeof(glm::mat4));

        global_uniform_binding_indexes->pop_back();

        DebugBoardBinding = global_uniform_binding_indexes->back();

        glBindBufferRange(GL_UNIFORM_BUFFER, DebugBoardBinding, uboDebugBoard, 0, ubuffer_size);

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

    DrawOutlineShader.use();
    glUniform1i(glGetUniformLocation(DrawOutlineShader.ID, "bufferMask"), 0);
    DrawOutlineShader.setVec2("viewportSize",glm::vec2(
        static_cast<float>(window_viewport_current_width),
        static_cast<float>(window_viewport_current_height)));
    
    shID = CameraDebugBoardShader.ID;
    CameraDebugBoardShader.use();
    glUniformBlockBinding(shID, glGetUniformBlockIndex(shID, "BoardBlock"), DebugBoardBinding);

    glBindBuffer(GL_UNIFORM_BUFFER, 0);

    Create_viewport_framebuffer();
    Create_info_framebuffer();
    Create_mask_framebuffer();

    //create lidar and camera context
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    Lidar_visualize_context = glfwCreateWindow(1, 1, "", nullptr, main_window);
    Lidar_inliers_context = glfwCreateWindow(1, 1, "", nullptr, main_window);
    Camera_context = glfwCreateWindow(1, 1, "", nullptr, main_window);
    if (Lidar_visualize_context == nullptr || Camera_context == nullptr || Lidar_inliers_context == nullptr)
        std::cout << "Failed to create Lidar-Camera contexts" << std::endl;

    glGenBuffers(1, &lidar_context_ssbo);
    glGenBuffers(1, &camera_context_pbo);
    //glfwWindowHint(GLFW_VISIBLE, GLFW_TRUE);

    LidarDetectedBoardHistory.set_capacity(LIDAR_BOARD_HISTORY_SIZE); 
    CameraDetectedBoardHistory.set_capacity(CAMERA_BOARD_HISTORY_SIZE); 


    //init Lidar
    std::string default_lidar_shader_path = "../shaders/gui_test/uniforms/";
    //ShadersVector["lidar_point_shader"] = std::make_shared<Shader>((default_lidar_shader_path+"point_sh.vs").c_str(), (default_lidar_shader_path+"point_sh.fs").c_str());
    ShadersVector["lidar_inliers_shader"] = std::make_shared<Shader>((default_lidar_shader_path+"inliers.vs").c_str(), (default_lidar_shader_path+"inliers.fs").c_str());
    
    //ShadersVector["lidar_point_shader"]->use();
    //unsigned int point_shID = ShadersVector["lidar_point_shader"]->ID; 
    // then we link each shader's uniform block to this uniform binding point
    //glUniformBlockBinding(point_shID, glGetUniformBlockIndex(point_shID, "Matrices_cam"), ProjViewBindingViewportCamera);

    ShadersVector["lidar_inliers_shader"]->use();
    unsigned int point_shID = ShadersVector["lidar_inliers_shader"]->ID; 
    // then we link each shader's uniform block to this uniform binding point
    glUniformBlockBinding(point_shID, glGetUniformBlockIndex(point_shID, "Matrices_cam"), ProjViewBindingViewportCamera);

    lidar_points_visualize_max_npoints = 1000U;
    lidar_inliers_size_max_npoints = 1000U;

    glGenVertexArrays(1, &LidarInliersVAO_main);
    glGenBuffers(1, &LidarInliersVBO.VBO);
    glGenBuffers(1, &LidarInliersVBO_main);
    glBindVertexArray(LidarInliersVAO_main);

    glBindBuffer(GL_ARRAY_BUFFER, LidarInliersVBO_main);
    glBufferData(GL_ARRAY_BUFFER, lidar_inliers_size_max_npoints*sizeof(glm::vec3), NULL, GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);

    glBindVertexArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, LidarInliersVBO.VBO);
    glBufferData(GL_ARRAY_BUFFER, lidar_inliers_size_max_npoints*sizeof(glm::vec3), NULL, GL_STATIC_DRAW);

    LidarInliersVBO.lidar_points_number = 0;
    LidarInliersVBO.points_recieved_first_time = false;
    LidarInliersVBO.board_existence = false;


    glGenVertexArrays(1, &LidarPointsVisualizeVAO_main);
    glGenBuffers(1, &LidarPointsVisualizeVBO.VBO);
    glGenBuffers(1, &LidarPointsVisualizeVBO_main);
    glBindVertexArray(LidarPointsVisualizeVAO_main);

    glBindBuffer(GL_ARRAY_BUFFER, LidarPointsVisualizeVBO_main);
    glBufferData(GL_ARRAY_BUFFER, lidar_points_visualize_max_npoints*sizeof(glm::vec3), NULL, GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);

    glBindVertexArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, LidarPointsVisualizeVBO.VBO);
    glBufferData(GL_ARRAY_BUFFER, lidar_points_visualize_max_npoints*sizeof(glm::vec3), NULL, GL_STATIC_DRAW);

    LidarPointsVisualizeVBO.lidar_points_number = 0;
    LidarPointsVisualizeVBO.points_recieved_first_time = false;


    LoadYamlParams();

    //glGenTextures(2, CameraDualColorB.textureID);

    //glBindTexture(GL_TEXTURE_2D, CameraDualColorB.textureID[0]);

    //glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, camera_board_settings.camera.image_size.width, 
    //    camera_board_settings.camera.image_size.height, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);

    //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    //glBindTexture(GL_TEXTURE_2D, CameraDualColorB.textureID[1]);

    //glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, camera_board_settings.camera.image_size.width, 
    //    camera_board_settings.camera.image_size.height, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);

    //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    glGenTextures(1, &RenderFrameCameraTexture);

    glBindTexture(GL_TEXTURE_2D, RenderFrameCameraTexture);

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, camera_board_settings.camera.image_size.width, 
        camera_board_settings.camera.image_size.height, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    
    
    glBindTexture(GL_TEXTURE_2D, 0);

    //CameraColorB.texture_index = 0;
    //CameraColorB.rtexture_index = 1;
    CameraColorB.pixels_number = 0;
    //CameraColorB.pixels_number[1] = 0;
    CameraColorB.image_height = 0;
    //CameraColorB.image_height[1] = 0;
    CameraColorB.image_width = 0;
    //CameraColorB.image_width[1] = 0;
    CameraColorB.image_recieved_first_time = false;
    //CameraColorB.image_recieved_first_time[1] = false;





}
/*
void LidarCamSim::Scene::InitBBoxes(const ViewportCamConfig &view_cfg,const VirtualCamConfig &real_cam_cfg){
    LidarCamSim::shaderConfig secondary_bbox_shader_cfg;
    secondary_bbox_shader_cfg.type = LidarCamSim::shader_type::DEFAULT;
    secondary_bbox_shader_cfg.vertex_path = "../shaders/gui_test/uniforms/4.6.shader5.vs";
    secondary_bbox_shader_cfg.fragment_path =  "../shaders/gui_test/uniforms/4.6.shader5.fs";
    secondary_bbox_shader_cfg.shader_name = "cubeShader";
    AddShader(secondary_bbox_shader_cfg);

    std::shared_ptr<cube> secondary_bbox = std::make_shared<cube>(glm::vec3(2.0f,0.6f,-2.5f),
                                                            *(ShadersVector["cubeShader"].get()),
                                                            DRAW_UNTEXTURED);

    secondary_bbox->Rotate(glm::radians(-11.0f),glm::vec3(0.0f,1.0f,0.0f));                                                  
    secondary_bbox->SetInfoShader(&main_scene_class->InfoShader);
    //secondary_bbox->SetOutlineShader(&main_scene_class->OutlineShader);
    secondary_bbox->SetDrawInfoMode(DRAW_INFO);
    //secondary_bbox->SetDrawOutlineMode(DRAW_OUTLINE);

    main_scene_class->AddObject(secondary_bbox,"secondary_bbox");
    main_scene_class->ObjectsVector["secondary_bbox"]->auto_render = false;
    main_scene_class->ObjectsVector["secondary_bbox"]->texture_transform_support = false;
    main_scene_class->ObjectsVector["secondary_bbox"]->cameras_for_render["viewport_cam"] = true;


}
*/
void LidarCamSim::Scene::InitCameras(const ViewportCamConfig &view_cfg,const VirtualCamConfig &real_cam_cfg){

    viewport_camera  = std::make_unique<ArcBallCamera>(window_viewport_current_width, window_viewport_current_height,
                        view_cfg.pivot_position, view_cfg.pivot_distance,
                        view_cfg.yaw_pitch.x,view_cfg.yaw_pitch.y);

    viewport_camera->setConstrainAngle(false);

    radialDistortion_coefs = real_cam_cfg.radialDistortion;
    tangentialDistortion_coefs = real_cam_cfg.tangentialDistortion;

    //VirtualCamVfov = ;

    viewport_camera_projection_matrix = glm::perspective(glm::radians(ViewportCamVfov), (float)window_viewport_current_width / (float)window_viewport_current_height, 0.1f, 100.0f);
    viewport_camera_view_matrix = viewport_camera->GetViewMatrix();

    update_viewport_cam_view_matrix_ubo();
    
    //initing pixel buffer object, make transfers to the cpu
    unsigned int camera_buffer_size = real_cam_cfg.CamWidth*real_cam_cfg.CamHeight *3U;

    Camera_cpu_buffer.resize(camera_buffer_size);
}

void LidarCamSim::Scene::InitLidarCameraThreads(){

    //if (!RecieveLidarDataAsync.joinable()) {
    //    RecieveLidarDataAsync = std::thread(&LidarCamSim::Scene::RecieveLidarDataAsync_func, this);
    //}

    if (!RecieveCameraDataAsync.joinable()) {
        RecieveCameraDataAsync = std::thread(&LidarCamSim::Scene::RecieveCameraDataAsync_func, this);
    }


}

void LidarCamSim::Scene::JoinLidarCameraThreads(){

    //if (RecieveLidarDataAsync.joinable()) {
    //    RecieveLidarDataAsync.join();
    //}

    if (RecieveCameraDataAsync.joinable()) {
        RecieveCameraDataAsync.join();
    }

    
}

void LidarCamSim::Scene::LoadYamlParams(){
    //lidar_bo_settings = 

    LidarYamlConfig lidar_cfg_yaml;
    if (!loadLidarConfig("../config/cfg.yaml", lidar_cfg_yaml)) {
        std::cerr << "Failed to load lidar config, loading default parameters\n";

        lidar_cfg_yaml.lidar_bounds.x_min = -10.0f;
        lidar_cfg_yaml.lidar_bounds.x_max = 10.0f;
        lidar_cfg_yaml.lidar_bounds.y_min = -0.95f;
        lidar_cfg_yaml.lidar_bounds.y_max = 10.0f;
        lidar_cfg_yaml.lidar_bounds.z_min = -10.0f;
        lidar_cfg_yaml.lidar_bounds.z_max = 10.0f;

        lidar_cfg_yaml.coord_sys.right = Eigen::Vector3f(1.0f,0.0f,0.0f);
        lidar_cfg_yaml.coord_sys.up = Eigen::Vector3f(0.0f,1.0f,0.0f);
        lidar_cfg_yaml.coord_sys.front = Eigen::Vector3f(0.0f,0.0f,-1.0f);

        lidar_cfg_yaml.lidar_offset = 0.0f;
        //return 1;
    }

    lidar_bo_settings.lidar_bounds = lidar_cfg_yaml.lidar_bounds;
    lidar_bo_settings.lidar_offset = lidar_cfg_yaml.lidar_offset;
    lidar_bo_settings.coord_sys = lidar_cfg_yaml.coord_sys;

    CameraYamlConfig camera_cfg_yaml;
    //Camera settings
    if (!loadCameraConfig("../config/cfg.yaml", camera_cfg_yaml)) {
        std::cerr << "Failed to load camera config, loading default parameters\n";

        camera_cfg_yaml.chessboard.pattern_size.height = 8;
        camera_cfg_yaml.chessboard.pattern_size.width = 6;
        camera_cfg_yaml.chessboard.square_length = 0.065f;
        camera_cfg_yaml.chessboard.board_dimension.width = 0.650f;
        camera_cfg_yaml.chessboard.board_dimension.height = 0.910f;
        camera_cfg_yaml.chessboard.translation_error.x = 0.010f;
        camera_cfg_yaml.chessboard.translation_error.y = 0.030f;

        camera_cfg_yaml.camera.image_size.width = 1000;
        camera_cfg_yaml.camera.image_size.height = 600;

        camera_cfg_yaml.camera.K = {724.26,0.0,500.0,0.0,724.26,300.0,0.0,0.0,1.0};

        camera_cfg_yaml.camera.distortion_model = "plumb_bob";

        camera_cfg_yaml.camera.D = {-0.170,0.041,-0.005,0.001,0.000};

    }
    
    camera_board_settings = camera_cfg_yaml;

    // --- Chessboard Parameters ---
    // Define the number of inner corners per chessboard row and column
    BoardPatternSize = cv::Size(camera_board_settings.chessboard.pattern_size.width, camera_board_settings.chessboard.pattern_size.height);

    // Define the real-world size of a square (e.g., 1.0 unit)

    //float squareSize = cfg_yaml.chessboard.square_length;
    //float size_x = cfg_yaml.chessboard.board_dimension.width;
    //float size_y = cfg_yaml.chessboard.board_dimension.height;
    //float error_x = cfg_yaml.chessboard.translation_error.x;
    //float error_y = cfg_yaml.chessboard.translation_error.y;

    //int rows = cfg_yaml.camera.image_size.height;
    //int cols = cfg_yaml.camera.image_size.width;

    cam_matrix   = cv::Mat(3,3,CV_64FC1,camera_board_settings.camera.K.data());
    distortion_c = cv::Mat(1,camera_board_settings.camera.D.size(),CV_64FC1,camera_board_settings.camera.D.data());

    TopicsInfo topics_cfg_yaml;
    //Camera settings
    if (!loadTopicsConfig("../config/cfg.yaml", topics_cfg_yaml)) {
    //if (true) {
        std::cerr << "Failed to load topics config, loading default parameters\n";

        topics_cfg_yaml.shared_mem_name = "calib_shared_mem";
        topics_cfg_yaml.bbox_shared_mem_name = "bbox_shared_mem";

        topics_cfg_yaml.camera_ctrl_mtx_name = "calib_camera_ctrl_mtx";
        topics_cfg_yaml.lidar_ctrl_mtx_name = " calib_lidar_ctrl_mtx";
        topics_cfg_yaml.bbox_ctrl_mtx_name = "bbox_ctrl_mtx";

        topics_cfg_yaml.camera_recieve_queue_name = "calib_camera_queue";
        topics_cfg_yaml.lidar_recieve_queue_name = "calib_lidar_pts_queue";

    }

    ipc_topics_names = topics_cfg_yaml;
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
        if(ObjectsVector[object_selected]->cameras_for_render["viewport_cam"]){
            glBindFramebuffer(GL_FRAMEBUFFER, m_mask_framebuffer);
            glViewport(0, 0, static_cast<unsigned int>(window_viewport_current_width), static_cast<unsigned int>(window_viewport_current_height));
            glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
            //glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            glClear(GL_COLOR_BUFFER_BIT);
            //glEnable(GL_DEPTH_TEST);
            glBindBuffer(GL_UNIFORM_BUFFER, uboViewportCamera);
            //glEnable(GL_STENCIL_TEST);
            //glStencilFunc(GL_NOTEQUAL, 0x01, 0xFF);
            //glStencilMask(0x00); 
            glDisable(GL_DEPTH_TEST);
            //float distance = glm::length(ObjectsVector[object_selected]->object->position - viewport_camera->m_pos);
            ObjectsVector[object_selected]->object->SetOutlineTransform(); 
            //OutlineShader.setFloat("distant",distance);
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
            //glStencilMask(0xFF);
            //glStencilFunc(GL_ALWAYS, 0, 0xFF);
            //glDisable(GL_STENCIL_TEST);
            glBindBuffer(GL_UNIFORM_BUFFER, 0);

            glBindFramebuffer(GL_FRAMEBUFFER, viewport_framebuffer);     

            glViewport(0, 0, static_cast<unsigned int>(window_viewport_current_width), static_cast<unsigned int>(window_viewport_current_height));

            //glDisable(GL_DEPTH_TEST);
            //glDepthMask(GL_TRUE);
            glDisable(GL_CULL_FACE);

            //draw

            DrawOutlineShader.use();
            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, m_mask_texture);

            //glActiveTexture(GL_TEXTURE1);
            //glUniform1i(glGetUniformLocation(DrawOutlineShader.ID, "bufferDepthMask"), 1);
            //glBindTexture(GL_TEXTURE_2D, m_mask_depth_texture);
            ////glTexParameteri(GL_TEXTURE_2D,
            ////        GL_TEXTURE_COMPARE_MODE,
            ////        GL_NONE);

            //glActiveTexture(GL_TEXTURE2);
            //glUniform1i(glGetUniformLocation(DrawOutlineShader.ID, "bufferDepth"), 2);
            //glBindTexture(GL_TEXTURE_2D, viewport_rbo);
            ////glTexParameteri(GL_TEXTURE_2D, GL_DEPTH_STENCIL_TEXTURE_MODE, GL_DEPTH_COMPONENT);

            glBindVertexArray(dummyVAO);
            glDrawArrays(GL_TRIANGLES,0,6);

            glBindVertexArray(0);

            glActiveTexture(GL_TEXTURE0);

            glEnable(GL_DEPTH_TEST);
            glEnable(GL_CULL_FACE);
        }
        else{
            object_selected = "";
        }

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
            if(it->second->cameras_for_render["viewport_cam"]){
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
        glDisable(GL_STENCIL_TEST);
        //glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        //stencil
        //glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
        //glStencilFunc(GL_ALWAYS, 0x01, 0xFF); 
        //glStencilMask(0xFF); 

        glBindBuffer(GL_UNIFORM_BUFFER, uboViewportCamera);

}

void LidarCamSim::Scene::RenderEndMainViewport(){
        glBindBuffer(GL_UNIFORM_BUFFER, 0);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void LidarCamSim::Scene::RenderMainViewportAuto(){



        for (auto it = ObjectsVector.begin(); it != ObjectsVector.end(); it++) {

            if(it->second->cameras_for_render.find("viewport_cam") != it->second->cameras_for_render.end() && it->second->auto_render){
                if(it->second->cameras_for_render["viewport_cam"]){

                    //glDisable(GL_STENCIL_TEST);

                    //if(it->first == object_selected) glEnable(GL_STENCIL_TEST);

                    
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
        }
        //glDisable(GL_STENCIL_TEST); 

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
    //glGenTextures(1, &m_picking_depth_tex);
    //glBindTexture(GL_TEXTURE_2D, m_picking_depth_tex);
    //glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, window_viewport_current_width, window_viewport_current_height, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
    //glFramebufferTexture2D(GL_FRAMEBUFFER,GL_DEPTH_ATTACHMENT,GL_TEXTURE_2D,m_picking_depth_tex,0);

    glGenRenderbuffers(1, &m_picking_depth_rbo);
    glBindRenderbuffer(GL_RENDERBUFFER, m_picking_depth_rbo);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, window_viewport_current_width, window_viewport_current_height); // use a single renderbuffer object for both a depth AND stencil buffer.
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, m_picking_depth_rbo); // now actually attach it

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

    //glBindTexture(GL_TEXTURE_2D, m_picking_depth_tex);
    //glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, width, height, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
    //glFramebufferTexture2D(GL_FRAMEBUFFER,GL_DEPTH_ATTACHMENT,GL_TEXTURE_2D,m_picking_depth_tex,0);


	glBindRenderbuffer(GL_RENDERBUFFER, m_picking_depth_rbo);
	glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, width, height);
	glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, m_picking_depth_rbo);
}

void LidarCamSim::Scene::Create_mask_framebuffer()
{   

    // framebuffer configuration
    // -------------------------
    glGenFramebuffers(1, &m_mask_framebuffer);
    glBindFramebuffer(GL_FRAMEBUFFER, m_mask_framebuffer);
    // create a color attachment texture
    glGenTextures(1, &m_mask_texture);
    glBindTexture(GL_TEXTURE_2D, m_mask_texture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, window_viewport_current_width, window_viewport_current_height, 0, GL_RED, GL_UNSIGNED_BYTE, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_mask_texture, 0);

    //glGenTextures(1, &m_mask_depth_texture);
    //glBindTexture(GL_TEXTURE_2D, m_mask_depth_texture);
    //glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, window_viewport_current_width, window_viewport_current_height, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
    //glFramebufferTexture2D(GL_FRAMEBUFFER,GL_DEPTH_ATTACHMENT,GL_TEXTURE_2D,m_mask_depth_texture,0);

    // now that we actually created the framebuffer and added all attachments we want to check if it is actually complete now
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        std::cout << "ERROR::FRAMEBUFFER:: Framebuffer is not complete!" << std::endl;
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    //previousPickingImGuiSceneWindowSize = ImVec2(window_viewport_current_width, window_viewport_current_height);
}


void LidarCamSim::Scene::Rescale_mask_framebuffer(int width, int height)
{
    glBindTexture(GL_TEXTURE_2D, m_mask_texture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, width, height, 0, GL_RED, GL_UNSIGNED_BYTE, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_mask_texture, 0);

    //glBindTexture(GL_TEXTURE_2D, m_mask_depth_texture);
    //glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, width, height, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
    //glFramebufferTexture2D(GL_FRAMEBUFFER,GL_DEPTH_ATTACHMENT,GL_TEXTURE_2D,m_mask_depth_texture,0);

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
        Rescale_mask_framebuffer(static_cast<int>(window_viewport_current_width), 
                                static_cast<int>(window_viewport_current_height));
                                
        viewport_camera->ScreenResize(window_viewport_current_width,window_viewport_current_height);

        DrawOutlineShader.use();
        DrawOutlineShader.setVec2("viewportSize",glm::vec2(ViewportSize.x,ViewportSize.y));

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

void LidarCamSim::Scene::draw_loaded_camera_params(){

    ImGui::Begin("Camera Parameters",nullptr,ImGuiWindowFlags_None | windowFlags_global_reference);
    bool modfied = false;
    bool modfied_dist = false;
    //float new_width = plumb_bob_cam->m_windowWidth;
    //float new_height = plumb_bob_cam->m_windowHeight;
    //float new_Ratio = plumb_bob_cam->RenderRatio;

    float new_width = 1000;
    float new_height = 1000;

    // Draw your viewport content (OpenGL/Vulkan texture, etc.)
    ImVec2 size = ImGui::GetContentRegionAvail();
    float spacing_x = global_style_reference.ItemSpacing.x;
    ImGui::Text("Image Dimensions");
    ImGui::PushItemWidth((size.x-spacing_x)/2.0f);
    if(ImGui::DragFloat("##camWidth", &new_width,1.0f, 10.0f,FLT_MAX,"Width: %.0f",ImGuiSliderFlags_AlwaysClamp))modfied = true;
    ImGui::SameLine();
    if(ImGui::DragFloat("##camHeight", &new_height,1.0f, 10.0f,FLT_MAX,"Height: %.0f",ImGuiSliderFlags_AlwaysClamp))modfied = true;
    //if(ImGui::DragFloat("##vcamvfov", &plumb_bob_cam->Vfov,0.1f, 2.0f,178.0f,"Vertical FOV: %.0f°",ImGuiSliderFlags_AlwaysClamp))modfied = true;
    ImGui::PushItemWidth((size.x-2.0f*spacing_x)/3.0f);
    ImGui::Text("Radial Distortion");
    if(ImGui::DragFloat("##rdistor1", &radialDistortion_coefs.x, 0.01f,-FLT_MAX,FLT_MAX,"K1: %.3f",ImGuiSliderFlags_AlwaysClamp))modfied_dist = true;
    ImGui::SameLine();
    if(ImGui::DragFloat("##rdistor2", &radialDistortion_coefs.y, 0.01f,-FLT_MAX,FLT_MAX,"K2: %.3f",ImGuiSliderFlags_AlwaysClamp))modfied_dist = true;
    ImGui::SameLine();
    if(ImGui::DragFloat("##rdistor3", &radialDistortion_coefs.z, 0.01f,-FLT_MAX,FLT_MAX,"K3: %.3f",ImGuiSliderFlags_AlwaysClamp))modfied_dist = true;
    ImGui::PushItemWidth((size.x-spacing_x)/2.0f);
    ImGui::Text("Tangential Distortion");
    if(ImGui::DragFloat("##tdistor1", &tangentialDistortion_coefs.x, 0.01f,-FLT_MAX,FLT_MAX,"P1: %.3f",ImGuiSliderFlags_AlwaysClamp))modfied_dist = true;
    ImGui::SameLine();
    if(ImGui::DragFloat("##tdistor2", &tangentialDistortion_coefs.y, 0.01f,-FLT_MAX,FLT_MAX,"P2: %.3f",ImGuiSliderFlags_AlwaysClamp))modfied_dist = true;
    ImGui::PushItemWidth(size.x);
    ImGui::Text("Recieve Camera Image Options");
    ImGui::Checkbox("Recieve Image",&camera_broadcast);
    ImGui::PopItemWidth();

    if(modfied){

        //virtual_camera_rescaled = true;
    }

    ImGui::End();

    ImGui::Begin("Camera Projection Matrix",nullptr,ImGuiWindowFlags_None | windowFlags_global_reference);
    ImGui::NewLine();
    float windowWidth = ImGui::GetWindowSize().x;
    float textWidth   = ImGui::CalcTextSize("PROJECTION MATRIX").x;
    ImGui::SetCursorPosX((windowWidth - textWidth) * 0.5f);
    ImGui::Text("PROJECTION MATRIX");
        //float line1[4] = {
        //    plumb_bob_cam->camera_intrinsic[0][0],
        //    plumb_bob_cam->camera_intrinsic[1][0],
        //    plumb_bob_cam->camera_intrinsic[2][0],
        //    plumb_bob_cam->camera_intrinsic[3][0]
        //};
        //float line2[4] = {
        //    plumb_bob_cam->camera_intrinsic[0][1],
        //    plumb_bob_cam->camera_intrinsic[1][1],
        //    plumb_bob_cam->camera_intrinsic[2][1],
        //    plumb_bob_cam->camera_intrinsic[3][1]
        //};
        //float line3[4] = {
        //    plumb_bob_cam->camera_intrinsic[0][2],
        //    plumb_bob_cam->camera_intrinsic[1][2],
        //    plumb_bob_cam->camera_intrinsic[2][2],
        //    plumb_bob_cam->camera_intrinsic[3][2]
        //};
        //float line4[4] = {
        //    plumb_bob_cam->camera_intrinsic[0][3],
        //    plumb_bob_cam->camera_intrinsic[1][3],
        //    plumb_bob_cam->camera_intrinsic[2][3],
        //    plumb_bob_cam->camera_intrinsic[3][3]
        //};
        //ImGui::SetCursorPosX((windowWidth) * 0.175f);
        //ImGui::InputFloat4("##l1p", line1, "%.2f",ImGuiInputTextFlags_ReadOnly);
        //ImGui::SetCursorPosX((windowWidth) * 0.175f);
        //ImGui::InputFloat4("##l2p", line2, "%.2f",ImGuiInputTextFlags_ReadOnly);
        //ImGui::SetCursorPosX((windowWidth) * 0.175f);
        //ImGui::InputFloat4("##l3p", line3, "%.2f",ImGuiInputTextFlags_ReadOnly);
        //ImGui::SetCursorPosX((windowWidth) * 0.175f);
        //ImGui::InputFloat4("##l4p", line4, "%.2f",ImGuiInputTextFlags_ReadOnly);

    ImGui::End();
}

void LidarCamSim::Scene::draw_camera_lidar_matrix(){

    ImGui::Begin("Camera Lidar Matrix",nullptr,ImGuiWindowFlags_None | windowFlags_global_reference);

    ImGui::NewLine();
    float windowWidth = ImGui::GetWindowSize().x;
    float textWidth   = ImGui::CalcTextSize("CAMERA TO LIDAR MATRIX").x;
    ImGui::SetCursorPosX((windowWidth - textWidth) * 0.5f);

    glm::mat4 camera_lidar_m = glm::mat4(1.0f);//inverse_normalized_lidar_model*plumb_bob_cam->camera_matrix;

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

        // Draw your viewport content (OpenGL/Vulkan texture, etc.)

        ImGui::NewLine();

        bool modfied = false;
        ImVec2 size = ImGui::GetContentRegionAvail();
        float spacing_x = global_style_reference.ItemSpacing.x;
        float button_size = static_cast<float>(monitorScreenHeight)*0.03;
        ImGui::Text("Manual Calibration");
        ImGui::Text("Translation");
        glm::vec3 position(0.0f);
        glm::vec3 orientation(0.0f);
        // Create sliders for X and Y coordinates
        //ImGui::SliderFloat("Z Position", &board_position.z, -pos_amplitude, pos_amplitude, "Z: %.1f");
        if(ImGui::DragFloat("X Position##anualcalib", &position.x, 0.01f,-FLT_MAX,FLT_MAX,"X: %.2f m",ImGuiSliderFlags_AlwaysClamp))modfied = true;
        if(ImGui::DragFloat("Y Position##anualcalib", &position.y, 0.01f,-FLT_MAX,FLT_MAX,"Y: %.2f m",ImGuiSliderFlags_AlwaysClamp))modfied = true;
        if(ImGui::DragFloat("Z Position##anualcalib", &position.z, 0.01f,-FLT_MAX,FLT_MAX,"Z: %.2f m",ImGuiSliderFlags_AlwaysClamp))modfied = true;

        ImGui::Text("Rotation");

        if(ImGui::DragFloat("X Rotation##manualcalib", &orientation.x,0.1f,-FLT_MAX, FLT_MAX, "X: %.1f°",ImGuiSliderFlags_AlwaysClamp))modfied = true;
        if(ImGui::DragFloat("Y Rotation##manualcalib", &orientation.y,0.1f,-FLT_MAX, FLT_MAX, "Y: %.1f°",ImGuiSliderFlags_AlwaysClamp))modfied = true;
        if(ImGui::DragFloat("Z Rotation##manualcalib", &orientation.z,0.1f,-FLT_MAX, FLT_MAX, "Z: %.1f°",ImGuiSliderFlags_AlwaysClamp))modfied = true;

        ImGui::PushItemWidth((size.x-spacing_x)/2.0f);
        ImGui::NewLine();
        if(ImGui::Button("Save Matrix",ImVec2((size.x-spacing_x)/2.0f,button_size)))modfied=true;
        ImGui::SameLine();
        if(ImGui::Button("Load Matrix",ImVec2((size.x-spacing_x)/2.0f,button_size)))modfied=true;

        ImGui::PopItemWidth();

    ImGui::End();
}


void LidarCamSim::Scene::draw_lidar_cloud_settings(){

    ImGui::Begin("Lidar Cloud Settings",nullptr,ImGuiWindowFlags_None | windowFlags_global_reference);

    // Draw your viewport content (OpenGL/Vulkan texture, etc.)
    //ImVec2 size = ImGui::GetContentRegionAvail();
    //ImGui::Text("My win2 %.0fx%.0f", size.x, size.y);

    float button_size = static_cast<float>(monitorScreenHeight)*0.03;

    //lidar_bo_settings

    //float ring_count_f = static_cast<float>(Lidar_params.ring_count);

    bool view_main_bbox = true;
    bool view_secondary_bbox = true;

    bool modfied = false;

    //std::size_t lidar_name_size = 101;
    //char Lidar_name[lidar_name_size];
    //std::memcpy(Lidar_name,Lidar_params.model.c_str(),Lidar_params.model.size());
    //Lidar_name[Lidar_params.model.size()] = '\0';
    
    // Draw your viewport content (OpenGL/Vulkan texture, etc.)
    ImVec2 size = ImGui::GetContentRegionAvail();
    float spacing_x = global_style_reference.ItemSpacing.x;

    ImGui::Text("Lidar Cloud Parameters");

    ImGui::NewLine();
    
    ImGui::Text("Bounding Box Visualization Options");
    ImGui::Checkbox("View Main Bounding Box",&view_main_bbox);
    ImGui::Checkbox("View Secondary Bounding Box",&view_secondary_bbox);
    ImGui::Text("Main Bounding Box Options");
    ImGui::PushItemWidth((size.x-spacing_x)/2.0f);
    if(ImGui::DragFloat("##lidarcxmin", &lidar_bo_settings.lidar_bounds.x_min,0.1f, -FLT_MAX,FLT_MAX,"Xmin: %.2f",ImGuiSliderFlags_AlwaysClamp))modfied = true;
    ImGui::SameLine();
    if(ImGui::DragFloat("##lidarcxmax", &lidar_bo_settings.lidar_bounds.x_max,0.1f, -FLT_MAX,FLT_MAX,"Xmax: %.2f",ImGuiSliderFlags_AlwaysClamp))modfied = true;

    if(ImGui::DragFloat("##lidarcymin", &lidar_bo_settings.lidar_bounds.y_min,0.1f, -FLT_MAX,FLT_MAX,"Ymin: %.2f",ImGuiSliderFlags_AlwaysClamp))modfied = true;
    ImGui::SameLine();
    if(ImGui::DragFloat("##lidarcymax", &lidar_bo_settings.lidar_bounds.y_max,0.1f, -FLT_MAX,FLT_MAX,"Ymax: %.2f",ImGuiSliderFlags_AlwaysClamp))modfied = true;

    if(ImGui::DragFloat("##lidarczmin", &lidar_bo_settings.lidar_bounds.z_min,0.1f, -FLT_MAX,FLT_MAX,"Zmin: %.2f",ImGuiSliderFlags_AlwaysClamp))modfied = true;
    ImGui::SameLine();
    if(ImGui::DragFloat("##lidarczmax", &lidar_bo_settings.lidar_bounds.z_max,0.1f, -FLT_MAX,FLT_MAX,"Zmax: %.2f",ImGuiSliderFlags_AlwaysClamp))modfied = true;
    ImGui::PushItemWidth((size.x));

    ImGui::Text("Cloud Offset");
    if(ImGui::DragFloat("##lidarcoffset", &lidar_bo_settings.lidar_offset,1.0f, -FLT_MAX,FLT_MAX,"%.0f mm",ImGuiSliderFlags_AlwaysClamp))modfied = true;

    ImGui::Text("Cloud Recieve Options");

    if(ImGui::Checkbox("Receive Cloud",&lidar_recieve_debug_cloud))lidar_view_debug_cloud = (lidar_recieve_debug_cloud)?lidar_view_debug_cloud:false;
    ImGui::SameLine();
    if(!lidar_recieve_debug_cloud)ImGui::BeginDisabled();
    ImGui::Checkbox("View Recieved Cloud",&lidar_view_debug_cloud);
    if(!lidar_recieve_debug_cloud)ImGui::EndDisabled();

    ImGui::NewLine();

    if(ImGui::Button("Apply",ImVec2(2.0f*button_size,button_size))){
        
    }

    ImGui::PopItemWidth();

    if(modfied){
        SetBBoxValues(); 
    }

    //delete[] Lidar_name;
    ImGui::End();
}



void LidarCamSim::Scene::draw_calibration_settings(){
   
    float button_size = static_cast<float>(monitorScreenHeight)*0.03;

    bool modfied = false;

    float cam_hist_size = 5.0f;
    float max_time_disp = 500.0f;
    
    ImGui::Begin("Calibration Settings",nullptr,ImGuiWindowFlags_None | windowFlags_global_reference);

    // Draw your viewport content (OpenGL/Vulkan texture, etc.)
    ImVec2 size = ImGui::GetContentRegionAvail();
    float spacing_x = global_style_reference.ItemSpacing.x;

    ImGui::PushItemWidth(size.x);

    ImGui::Text("Camera History Buffer Size");
    if(ImGui::DragFloat("##camerahistbuffersize", &cam_hist_size,1.0f, 1.0f,FLT_MAX,"Size: %.0f",ImGuiSliderFlags_AlwaysClamp))modfied = true;
    ImGui::Text("Maximum Time Disparity");
    if(ImGui::DragFloat("##maxtimedispar", &max_time_disp,10.0f, 0.0f,FLT_MAX,"Maximum Disparity: %.0f ms",ImGuiSliderFlags_AlwaysClamp))modfied = true;
    ImGui::PushItemWidth((size.x-spacing_x)/2.0f);
    ImGui::NewLine();
    if(ImGui::Button("Capture Sample",ImVec2((size.x-spacing_x)/2.0f,button_size)))modfied=true;
    ImGui::SameLine();
    if(ImGui::Button("Discard Last Sample",ImVec2((size.x-spacing_x)/2.0f,button_size)))modfied=true;
    if(ImGui::Button("Save Samples",ImVec2((size.x-spacing_x)/2.0f,button_size)))modfied=true;
    ImGui::SameLine();
    if(ImGui::Button("Load Samples",ImVec2((size.x-spacing_x)/2.0f,button_size)))modfied=true;
    ImGui::PushItemWidth(size.x);
    if(ImGui::Button("Optimize",ImVec2(size.x,button_size)))modfied=true;

    ImGui::PopItemWidth();

    ImGui::End();
}

void LidarCamSim::Scene::draw_calibration_log(){
   
    float button_size = static_cast<float>(monitorScreenHeight)*0.03;

    bool modfied = false;

    
    ImGui::Begin("Calibration Logs",nullptr,ImGuiWindowFlags_None | windowFlags_global_reference);

    // Draw your viewport content (OpenGL/Vulkan texture, etc.)
    ImVec2 size = ImGui::GetContentRegionAvail();
    float spacing_x = global_style_reference.ItemSpacing.x;

    //ImGui::NewLine();
    //ImGui::PushItemWidth((size.x-spacing_x)/2.0f);
    //ImGui::Checkbox("View Main Bounding Box",&view_main_bbox);
    //ImGui::Checkbox("View Secondary Bounding Box",&view_secondary_bbox);
    //ImGui::PushItemWidth(size.x);
    //if(ImGui::DragFloat("##lidarcxmin", &size_test_xmin,0.1f, -FLT_MAX,FLT_MAX,"Xmin: %.2f",ImGuiSliderFlags_AlwaysClamp))modfied = true;
    //ImGui::SameLine();
    //if(ImGui::DragFloat("##lidarcxmax", &size_test_xmax,0.1f, -FLT_MAX,FLT_MAX,"Xmax: %.2f",ImGuiSliderFlags_AlwaysClamp))modfied = true;


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

    }
    }else{
        ImGui::Text("Object: No Object Selected!");
    }

    ImGui::End();
     
    /*
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
    */
}

void LidarCamSim::Scene::draw_recieved_cam_image_viewport(){

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

        float image_size_y = (static_cast<float>(camera_board_settings.camera.image_size.height)/static_cast<float>(camera_board_settings.camera.image_size.width))*currentImGuiWindowSize_cam.x;
        float image_size_x;
        if(image_size_y > currentImGuiWindowSize_cam.y){
            image_size_y = currentImGuiWindowSize_cam.y;
            image_size_x = (static_cast<float>(camera_board_settings.camera.image_size.width)/static_cast<float>(camera_board_settings.camera.image_size.height))*currentImGuiWindowSize_cam.y;

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
            (ImTextureID)(intptr_t)RenderFrameCameraTexture,
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

}

void LidarCamSim::Scene::draw_statistics_viewport(){

    ImGui::Begin("Statistics");

    bool modfied = false;

    // Draw your viewport content (OpenGL/Vulkan texture, etc.)
    ImVec2 size = ImGui::GetContentRegionAvail();
    float spacing_x = global_style_reference.ItemSpacing.x;

    ImGui::PushFont(font_mono);
    ImGui::SetWindowFontScale(0.9f);
    ImGui::Text("Framerate: %.1f FPS (%.3f ms/frame) ", io_reference.Framerate, 1000.0f / io_reference.Framerate);
    ImGui::SetWindowFontScale(1.0f);
    ImGui::PopFont();
    if(ImGui::Checkbox("Enable Vsync##statistics",&enable_vsync))(enable_vsync)?glfwSwapInterval(1):glfwSwapInterval(0);

    ImGui::NewLine();

    ImGui::Text("Online Detection Settings");
    
    ImGui::NewLine();

    ImGui::Text("Lidar Debug Board");

    ImGui::PushItemWidth(size.x);

    ImGui::Checkbox("View Corners##board_corners_lidar",&lidar_board_render_mode[2]);
    ImGui::SameLine();
    ImGui::Checkbox("View Borders##board_borders_lidar",&lidar_board_render_mode[1]);
    ImGui::Checkbox("View Normal##board_normal_lidar",&lidar_board_render_mode[3]);
    ImGui::SameLine();
    ImGui::Checkbox("View Plane##board_plane_lidar",&lidar_board_render_mode[0]);
    ImGui::InputFloat("##lboard_area_error", &lidar_board_area_error, 0.0f,0.0f,"Area Error: %.2f cm²",ImGuiInputTextFlags_ReadOnly);


    ImGui::NewLine();

    ImGui::Text("Camera Debug Board");
    ImGui::Checkbox("View Corners##board_corners_camera",&camera_board_render_mode[2]);
    ImGui::SameLine();
    ImGui::Checkbox("View Borders##board_borders_camera",&camera_board_render_mode[1]);
    ImGui::Checkbox("View Normal##board_normal_camera",&camera_board_render_mode[3]);
    ImGui::SameLine();
    ImGui::Checkbox("View Plane##board_plane_camera",&camera_board_render_mode[0]);
    ImGui::InputFloat("##cboard_area_error", &camera_board_area_error, 0.0f,0.0f,"Area Error: %.2f cm²",ImGuiInputTextFlags_ReadOnly);

    ImGui::NewLine();

    ImGui::Text("Errors Between Boards");

    //ImGui::PushItemWidth((size.x-spacing_x)/2.0f);
    ImGui::InputFloat("##bclboard_angle_error", &bet_camlidar_board_angle_error, 0.0f,0.0f, "Angle: %.1f°",ImGuiInputTextFlags_ReadOnly);
    //ImGui::SameLine();
    ImGui::InputFloat("##bclposition_error", &bet_camlidar_board_position_error, 0.0f,0.0f, "Position: %.2f mm",ImGuiInputTextFlags_ReadOnly);

    //ImGui::PushItemWidth(size.x);

    ImGui::NewLine();

    ImGui::Text("Samples Settings");
    
    ImGui::NewLine();

    ImGui::Text("Lidar Board");
    ImGui::Checkbox("View Corners##sample_corners_lidar",&lidar_sample_render_mode[2]);
    ImGui::SameLine();
    ImGui::Checkbox("View Borders##sample_borders_lidar",&lidar_sample_render_mode[1]);
    ImGui::Checkbox("View Normal##sample_normal_lidar",&lidar_sample_render_mode[3]);
    ImGui::SameLine();
    ImGui::Checkbox("View Plane##sample_plane_lidar",&lidar_sample_render_mode[0]);
    ImGui::Checkbox("View Inliers##sample_plane_lidar_inliers",&lidar_sample_render_mode[4]);

    

    ImGui::Text("Camera Board");
    ImGui::Checkbox("View Corners##sample_corners_camera",&camera_sample_render_mode[2]);
    ImGui::SameLine();
    ImGui::Checkbox("View Borders##sample_borders_camera",&camera_sample_render_mode[1]);
    ImGui::Checkbox("View Normal##sample_normal_camera",&camera_sample_render_mode[3]);
    ImGui::SameLine();
    ImGui::Checkbox("View Plane##sample_plane_camera",&camera_sample_render_mode[0]);
    
    ImGui::NewLine();
    ImGui::Text("Last Sample Errors");
    ImGui::NewLine();

    ImGui::Text("Errors From Lidar");
    float lidar_reprojection_error =0.0f;
    ImGui::InputFloat("##lidarreprojection_error", &lidar_reprojection_error, 0.0f,0.0f, "Reprojection Error: %.2f mm",ImGuiInputTextFlags_ReadOnly);

    ImGui::PopItemWidth();

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


}

/*
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
*/

void LidarCamSim::Scene::LidarCalculateNormalizedModel(){
    normalized_lidar_model = ObjectsVector["lidar_object"]->object->model_m;
    normalized_lidar_model[0] = glm::normalize(normalized_lidar_model[0]);
    normalized_lidar_model[1] = glm::normalize(normalized_lidar_model[1]);
    normalized_lidar_model[2] = glm::normalize(normalized_lidar_model[2]);
    inverse_normalized_lidar_model = glm::inverse(normalized_lidar_model);
}


//void LidarCamSim::Scene::RecieveCameraBoardAsync(){
//
//if(camera_recieve_debug_board){
//
//    if(CameraRecieveBoardMutex.try_lock()){
//
//        if(camera_detected_board_queue->get_num_msg() > 0){
//
//            //prepare
//
//            //func
//            std::thread Camera_recieve_board_data_func([=]
//            {
//            
//            uint64_t recvd_size;
//            uint32_t priority;
//
//            if(camera_detected_board_queue->try_receive(&camera_detected_board, sizeof(camera_detected_board), recvd_size, priority)){
//
//                
//                //lidar_points.resize(recvd_size / sizeof(Point_XYZIR));
//                //glfwMakeContextCurrent(Camera_detected_board_context);
//                //inliers_msg_recieve_size = static_cast<uint32_t>(recvd_size);
//                
//                for (int i = 0; i < 5; i++) {
//                    gpu_camera_detected_board_buffer[i] = glm::vec4(
//                        camera_detected_board[i * 3 + 0],
//                        camera_detected_board[i * 3 + 1],
//                        camera_detected_board[i * 3 + 2],
//                        1.0f    // padding; you could also use 0.0f or something else
//                    );
//                }
//
//                gpu_camera_detected_board_buffer[5] = glm::vec4(
//                    camera_detected_board[15],
//                    camera_detected_board[16],
//                    camera_detected_board[17],
//                    0.0f    // padding; you could also use 1.0f or something else
//                );
//
//                //shader use
//
//                camera_detected_board_recieved_first_time = true;
//                //glfwMakeContextCurrent(NULL);
//            }
//
//            CameraRecieveBoardMutex.unlock();
//            
//           });
//
//           Camera_recieve_board_data_func.detach();
//        }
//        else{
//            CameraRecieveBoardMutex.unlock();
//        }
//    }
//}
//
//
//}

void LidarCamSim::Scene::RecieveLidarDataAsync_func(){

    lidar_ctrl_mtx = std::make_unique<boost::interprocess::named_mutex>(boost::interprocess::open_or_create, ipc_topics_names.lidar_ctrl_mtx_name.c_str());
    boost::posix_time::ptime now = boost::posix_time::microsec_clock::universal_time();
    boost::posix_time::ptime abs_time = now + boost::posix_time::milliseconds(500);
    if(!(lidar_ctrl_mtx->timed_lock(abs_time)))lidar_ctrl_mtx->unlock();

    while(true){

        boost::this_thread::sleep(boost::posix_time::milliseconds(500));

        try{
            lidar_segment_ptr = std::make_unique<boost::interprocess::managed_shared_memory>(boost::interprocess::open_only, ipc_topics_names.shared_mem_name.c_str());

            // Look for "Ctrl"
            auto response = lidar_segment_ptr->find<SharedControl>("shared_notification");
            if (response.first != nullptr) {
                std::cout << "Found existing Lidar SharedControl"<<std::endl;
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
            lidar_recieve_queue = std::make_unique<boost::interprocess::message_queue>(boost::interprocess::open_only, ipc_topics_names.lidar_recieve_queue_name.c_str());
            lidar_point_cloud_size = lidar_recieve_queue->get_max_msg_size()/sizeof(Point_XYZIR);

            //mq_result = std::make_unique<boost::interprocess::message_queue>(boost::interprocess::open_only, "pts_queue_result");

            break;


        }
        catch (boost::interprocess::interprocess_exception& ex) 
        {
            std::cerr << "Error: " << ex.what() << std::endl;
            continue;
        }
    }

    std::cerr << "Point cloud size: " << lidar_point_cloud_size << " points" << std::endl;

    lidar_points.resize(lidar_point_cloud_size);

    std::cerr << "Resized state: " << lidar_resize_notification->resized << std::endl;

    glfwMakeContextCurrent(Lidar_inliers_context);

    while(true){

        uint64_t recvd_size;
        uint32_t priority;

        try
        {   

            bool recieve_queue_state = false;

            while(!recieve_queue_state){

                //std::cout << "Pass5"<<std::endl;

                now = boost::posix_time::microsec_clock::universal_time();
                abs_time = now + boost::posix_time::milliseconds(500);


                if(!lidar_ctrl_mtx->timed_lock(abs_time))
                {
                    lidar_ctrl_mtx->unlock();
                    lidar_ctrl_mtx->lock();
                }


                //std::cout << "Pass5.1"<<std::endl;
                if(!lidar_resize_notification->resized){
                    //std::cout << "Pass6"<<std::endl;
                    boost::posix_time::ptime now = boost::posix_time::microsec_clock::universal_time();
                    boost::posix_time::ptime abs_time = now + boost::posix_time::milliseconds(500);
                    recieve_queue_state = lidar_recieve_queue->timed_receive(lidar_points.data(), lidar_point_cloud_size*sizeof(Point_XYZIR), recvd_size, priority, abs_time);
                    lidar_ctrl_mtx->unlock();
                    //std::cout << "Pass7"<<std::endl;
                }else {

                    lidar_recieve_queue.reset();
                    //mq_result.reset();

                    lidar_recieve_queue = std::make_unique<boost::interprocess::message_queue>(boost::interprocess::open_only, ipc_topics_names.lidar_recieve_queue_name.c_str());

                    //mq_result = std::make_unique<boost::interprocess::message_queue>(boost::interprocess::open_only, "pts_queue_result");

                    lidar_point_cloud_size = lidar_recieve_queue->get_max_msg_size()/sizeof(Point_XYZIR);
                    lidar_points.resize(lidar_point_cloud_size);

                    lidar_resize_notification->resized = false;

                    lidar_ctrl_mtx->unlock();
                    continue;
                }

            }
        }
        catch (boost::interprocess::interprocess_exception& ex) 
        {
            std::cerr << "Error: " << ex.what() << std::endl;

            lidar_recieve_queue.reset();
            //mq_result.reset();
            now = boost::posix_time::microsec_clock::universal_time();
            abs_time = now + boost::posix_time::milliseconds(500);

            if(!lidar_ctrl_mtx->timed_lock(abs_time))
            {
                lidar_ctrl_mtx->unlock();
                lidar_ctrl_mtx->lock();
            }

            lidar_recieve_queue = std::make_unique<boost::interprocess::message_queue>(boost::interprocess::open_only, ipc_topics_names.lidar_recieve_queue_name.c_str());
            lidar_point_cloud_size = lidar_recieve_queue->get_max_msg_size()/sizeof(Point_XYZIR);
            lidar_points.resize(lidar_point_cloud_size);
            //mq_result = std::make_unique<boost::interprocess::message_queue>(boost::interprocess::open_only, "pts_queue_result");
            lidar_ctrl_mtx->unlock();

            continue;
        }

        pcl::PointCloud<pcl::PointXYZL>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZL>);
        uint32_t lidar_points_size_vec = recvd_size/sizeof(Point_XYZIR)-1U;
        Point_XYZIR timestamp_sample = lidar_points.at(lidar_points_size_vec);   // get last element
        //lidar_points.pop_back();           // remove it

        //float offset = 0.06f;

        //func inliers

        
        std::thread Lidar_limit_inliers_func([=]
        {
        for (uint32_t i = 0; i < lidar_points_size_vec; i++){

            pcl::PointXYZL point; 
            point.x = lidar_points[i].x;
            point.y = lidar_points[i].y;
            point.z = lidar_points[i].z;
            point.label = lidar_points[i].ring;

            if(point.x >= lidar_bo_settings.lidar_bounds.x_min && point.x <= lidar_bo_settings.lidar_bounds.x_max &&
               point.y >= lidar_bo_settings.lidar_bounds.y_min && point.y <= lidar_bo_settings.lidar_bounds.y_max &&
               point.z >= lidar_bo_settings.lidar_bounds.z_min && point.z <= lidar_bo_settings.lidar_bounds.z_max)
            {   

                //conpute offset
                if(lidar_bo_settings.lidar_offset != 0.0f){
                    float length = std::sqrt(point.x*point.x+point.y*point.y+point.z*point.z);
                    point.x += lidar_bo_settings.lidar_offset*(point.x/length);
                    point.y += lidar_bo_settings.lidar_offset*(point.y/length);
                    point.z += lidar_bo_settings.lidar_offset*(point.z/length);
                }
                // second comparison if enabled

                //cloud_not_filtered->push_back(point);
                cloud->push_back(point);
            }


        }
        });
        //Lidar_limit_inliers_func.detach();
        //// end func inliers

        //LidarPointsVisualizeMutex.lock();
        //the draw function must play with the copy

        lidar_points_visualize.resize(lidar_points_size_vec);

        ////func

        std::thread Lidar_limit_visualize_func([=]
        {
        uint32_t iterator = 0;
        
        for (uint32_t i = 0; i < lidar_points_size_vec; i++){

            // first comparison
            if(lidar_points[i].x >= lidar_bo_settings.lidar_bounds.x_min && lidar_points[i].x <= lidar_bo_settings.lidar_bounds.x_max &&
               lidar_points[i].y >= lidar_bo_settings.lidar_bounds.y_min && lidar_points[i].y <= lidar_bo_settings.lidar_bounds.y_max &&
               lidar_points[i].z >= lidar_bo_settings.lidar_bounds.z_min && lidar_points[i].z <= lidar_bo_settings.lidar_bounds.z_max)
            {   
                
                glm::vec3 point;

                point.x = lidar_points[i].x;
                point.y = lidar_points[i].y;
                point.z = lidar_points[i].z;

                //conpute offset
                if(lidar_bo_settings.lidar_offset != 0.0f){

                    float length = std::sqrt(point.x*point.x+point.y*point.y+point.z*point.z);
                    point.x += lidar_bo_settings.lidar_offset*(point.x/length);
                    point.y += lidar_bo_settings.lidar_offset*(point.y/length);
                    point.z += lidar_bo_settings.lidar_offset*(point.z/length);
                }

                // second comparison if enabled

                lidar_points_visualize.at(iterator) = point;

                iterator++;
            }


        }

        LidarPointsVisualizeVBO.VBO_mutex.lock();


        LidarPointsVisualizeVBO.lidar_points_number = iterator;

        glfwMakeContextCurrent(Lidar_visualize_context);
        
        if(LidarPointsVisualizeVBO.lidar_points_number>lidar_points_visualize_max_npoints){

            lidar_points_visualize_max_npoints = 2U*LidarPointsVisualizeVBO.lidar_points_number;

            glBindBuffer(GL_ARRAY_BUFFER, LidarPointsVisualizeVBO.VBO);
            glBufferData(GL_ARRAY_BUFFER, lidar_points_visualize_max_npoints*sizeof(glm::vec3), NULL, GL_STATIC_DRAW);

            glBindBuffer(GL_ARRAY_BUFFER, LidarPointsVisualizeVAO_main);
            glBufferData(GL_ARRAY_BUFFER, lidar_points_visualize_max_npoints*sizeof(glm::vec3), NULL, GL_STATIC_DRAW);

            glBindBuffer(GL_ARRAY_BUFFER, 0);

        }

        glBindBuffer(GL_ARRAY_BUFFER, LidarPointsVisualizeVBO.VBO);
        glBufferSubData(GL_ARRAY_BUFFER, 0, LidarPointsVisualizeVBO.lidar_points_number*sizeof(glm::vec3), lidar_points_visualize.data());
        glBindBuffer(GL_ARRAY_BUFFER, 0);

        LidarPointsVisualizeVBO.points_recieved_first_time = true;

        glfwMakeContextCurrent(NULL);

        LidarPointsVisualizeVBO.VBO_mutex.unlock();
        
        });

        //Lidar_limit_visualize_func.detach();
        ////end func

        // join funcs

        if (Lidar_limit_inliers_func.joinable())
            Lidar_limit_inliers_func.join();

        if (Lidar_limit_visualize_func.joinable())
            Lidar_limit_visualize_func.join();

        //LidarPointsVisualizeMutex.unlock();

        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

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

            //std::cerr << "Model coefficients: " << coefficients->values[0] << " " 

            //                                    << coefficients->values[1] << " "

            //                                    << coefficients->values[2] << " " 

            //                                    << coefficients->values[3] << std::endl;


            try
            {
                std::shared_ptr<std::vector<float>> lidar_inliers = std::make_shared<std::vector<float>>();
                lidar_inliers->resize(inliers->indices.size()*3);
                std::map<uint32_t, std::vector<Eigen::Vector3f>> points_in_rings;
                std::map<uint32_t, std::vector<Eigen::Vector3f>> borders_in_rings;

                uint32_t j = 0;
                for (const auto& idx : inliers->indices) {

                    Eigen::Vector3f point = Eigen::Vector3f((*cloud)[idx].x, (*cloud)[idx].y, (*cloud)[idx].z);

                    lidar_inliers->at(j)   = point.x();
                    lidar_inliers->at(j+1) = point.y();
                    lidar_inliers->at(j+2) = point.z();

                    points_in_rings[(*cloud)[idx].label].push_back(point);

                    j+=3;
                }

                uint32_t inliers_size = j;

                //std::cout << (*cloud)[100].x << ", " 
                //          << (*cloud)[100].y << ", "
                //          << (*cloud)[100].z << ", "
                //          << (*cloud)[100].label << std::endl;

                Eigen::Vector3f plane_normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
                
                for (auto& [ring_id, points] : points_in_rings) {
                    std::sort(points.begin(), points.end(), [this](const Eigen::Vector3f& a, const Eigen::Vector3f& b) {
                    //return std::atan2(a.y, a.x) < std::atan2(b.y, b.x);}
                    //Eigen::Vector3f a_norm = a.normalized();//
                    //Eigen::Vector3f b_norm = b.normalized();//
                    return ((a.cross(b)).dot(this->lidar_bo_settings.coord_sys.up) >= 0.0f);});

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


                Eigen::Vector3f plane_coord_origin = (borders_in_rings.begin()->second)[0];
                Eigen::Vector3f planar_vec_x = ((std::prev(borders_in_rings.end())->second)[0] - (borders_in_rings.begin()->second)[0]).normalized();
                Eigen::Vector3f planar_vec_z = (Eigen::Vector3f(coefficients->values[0], coefficients->values[1], coefficients->values[2])).normalized();
                Eigen::Vector3f planar_vec_y = (planar_vec_z.cross(planar_vec_x)).normalized();

                Eigen::Matrix4f plane_coord = Eigen::Matrix4f::Identity();
                plane_coord.block<3,1>(0,0) = planar_vec_x;
                plane_coord.block<3,1>(0,1) = planar_vec_y;
                plane_coord.block<3,1>(0,2) = planar_vec_z;
                plane_coord.block<3,1>(0,3) = plane_coord_origin;

                Eigen::Matrix4f plane_coord_inverse = plane_coord.inverse();

                pcl::PointCloud<pcl::PointXYZ>::Ptr left_edge_cloud(new  pcl::PointCloud<pcl::PointXYZ>);
                pcl::PointCloud<pcl::PointXYZ>::Ptr right_edge_cloud(new  pcl::PointCloud<pcl::PointXYZ>);

                for (auto& [ring_id, points] : borders_in_rings){
                    for (auto& point_proj : points){
                        Eigen::Vector4f point4 = point_proj.homogeneous();//Eigen::Vector4f point4(point_proj, 1.0f);
                        Eigen::Vector4f transformed_point4 = plane_coord_inverse * point4;
                        point_proj = transformed_point4.head<3>();

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


                std::vector<pcl::ModelCoefficients> edge_lines_right;
                std::vector<pcl::ModelCoefficients> edge_lines_left;


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



                    }

                    LidarDrawInliersMutex.lock();

                    //extracting the center
                    LidarDetectedBoardHistory.push_back();
                    LidarCamSim::LidarDetectedBoardData &lidar_board_data =  LidarDetectedBoardHistory.back();
                    for(uint8_t i = 0; i< 4;i++)
                    {   
                        Eigen::Vector3f corner = (plane_coord*(board_corners[i].homogeneous())).head<3>();
                        lidar_board_data.gpu_lidar_detected_board_buffer[i].x = corner.x();
                        lidar_board_data.gpu_lidar_detected_board_buffer[i].y = corner.y();
                        lidar_board_data.gpu_lidar_detected_board_buffer[i].z = corner.z();
                        lidar_board_data.gpu_lidar_detected_board_buffer[i].w = 1.0f;
                    }

                    Eigen::Vector3f center = (board_corners[0] + board_corners[2])*0.25f;
                    center += (board_corners[1] + board_corners[3])*0.25f;

                    lidar_board_data.gpu_lidar_detected_board_buffer[4].x = center.x();
                    lidar_board_data.gpu_lidar_detected_board_buffer[4].y = center.y();
                    lidar_board_data.gpu_lidar_detected_board_buffer[4].z = center.z();

                    lidar_board_data.gpu_lidar_detected_board_buffer[5].x = plane_normal.x();
                    lidar_board_data.gpu_lidar_detected_board_buffer[5].y = plane_normal.y();
                    lidar_board_data.gpu_lidar_detected_board_buffer[5].z = plane_normal.z();
                    lidar_board_data.lidar_detected_board_plane_d = coefficients->values[3];

                    std::memcpy(&lidar_board_data.lidar_timestamp_sec,&timestamp_sample.padding[1],4U);
                    std::memcpy(&lidar_board_data.lidar_timestamp_nanosec,&timestamp_sample.padding[2],4U);

                    //Now move the detected board to the detected board vector
                    lidar_board_data.lidar_inliers = lidar_inliers;
                    
                    //glfwMakeContextCurrent(Lidar_inliers_context);

                    LidarInliersVBO.VBO_mutex.lock();

                    LidarInliersVBO.board_data.detected_board_plane_d = 
                                        lidar_board_data.lidar_detected_board_plane_d;

                    std::memcpy(LidarInliersVBO.board_data.detected_board_buffer,
                                lidar_board_data.gpu_lidar_detected_board_buffer,6*sizeof(glm::vec4));

                    LidarDrawInliersMutex.unlock();

                    uint32_t inliers_size_npoints = (inliers_size/3U);
                    LidarInliersVBO.lidar_points_number = inliers_size_npoints;

                    if(inliers_size_npoints>lidar_inliers_size_max_npoints){

                        lidar_inliers_size_max_npoints = 2U*inliers_size_npoints;

                        glBindBuffer(GL_ARRAY_BUFFER, LidarInliersVBO.VBO);
                        glBufferData(GL_ARRAY_BUFFER, lidar_inliers_size_max_npoints*sizeof(glm::vec3), NULL, GL_STATIC_DRAW);

                        glBindBuffer(GL_ARRAY_BUFFER, LidarInliersVAO_main);
                        glBufferData(GL_ARRAY_BUFFER, lidar_inliers_size_max_npoints*sizeof(glm::vec3), NULL, GL_STATIC_DRAW);
                        
                        glBindBuffer(GL_ARRAY_BUFFER, 0);

                    }

                    glBindBuffer(GL_ARRAY_BUFFER, LidarInliersVBO.VBO);
                    glBufferSubData(GL_ARRAY_BUFFER, 0, inliers_size*sizeof(float), lidar_inliers->data());
                    glBindBuffer(GL_ARRAY_BUFFER, 0);

                    LidarInliersVBO.points_recieved_first_time = true;
                    //glfwMakeContextCurrent(main_window);

                    //tem qe salvar parametros no dual buffer e na history
                    LidarInliersVBO.VBO_mutex.unlock();
                    
                }
                else{
                    PCL_ERROR ("Could not detect board edges for the given dataset.\n");
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

    glfwMakeContextCurrent(NULL);
}

void LidarCamSim::Scene::find_board_edges(pcl::PointCloud<pcl::PointXYZ>::Ptr side_edge_cloud_input,std::vector<pcl::ModelCoefficients> &edge_lines_side_output){

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

Eigen::Vector3f LidarCamSim::Scene::line_intersect(line_model_3D &line1, line_model_3D &line2){

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


void LidarCamSim::Scene::RecieveCameraDataAsync_func(){

    // Open the existing message queue

    camera_ctrl_mtx = std::make_unique<boost::interprocess::named_mutex>(boost::interprocess::open_or_create, ipc_topics_names.camera_ctrl_mtx_name.c_str());
    boost::posix_time::ptime now = boost::posix_time::microsec_clock::universal_time();
    boost::posix_time::ptime abs_time = now + boost::posix_time::milliseconds(1000);
    if(!(camera_ctrl_mtx->timed_lock(abs_time)))camera_ctrl_mtx->unlock();



    while(true){

        boost::this_thread::sleep(boost::posix_time::milliseconds(500));

        try{
            camera_segment_ptr = std::make_unique<boost::interprocess::managed_shared_memory>(boost::interprocess::open_only, ipc_topics_names.shared_mem_name.c_str());
            //camera_resize_notification = segment.construct<SharedControl>("shared_notification")();

            // Look for "Ctrl"
            auto response = camera_segment_ptr->find<SharedControl>("shared_camera_notification");
            if (response.first != nullptr) {
                std::cout << "Found existing camera SharedControl"<<std::endl;
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
            camera_recieve_queue = std::make_unique<boost::interprocess::message_queue>(boost::interprocess::open_only, ipc_topics_names.camera_recieve_queue_name.c_str());
            image_pixels_number = (camera_recieve_queue->get_max_msg_size()-8U)/(3U); //(3U*sizeof(float))

            //camera_detected_board_queue = std::make_unique<boost::interprocess::message_queue>(boost::interprocess::open_only, "camera_queue_board");

            break;


        }
        catch (boost::interprocess::interprocess_exception& ex) 
        {
            std::cerr << "Error: " << ex.what() << std::endl;
            continue;
        }
    }

    std::cerr << "Image queue max number of pixels: " << image_pixels_number << " pixels" << std::endl;

    Camera_cpu_buffer.resize(image_pixels_number*3U+8U); //std::vector<float>
    //float Detected_board[19];

    std::cerr << "Camera resized state: " << camera_resize_notification->resized << std::endl;


    // --- Prepare Object Points ---
    // Create a vector of 3D points for the chessboard corners in world coordinates.
    // These points are in the chessboard coordinate system (z = 0)
    board_world_space_points.resize(BoardPatternSize.height*BoardPatternSize.width);


    for (int i = 0; i < BoardPatternSize.height; i++) {
        for (int j = 0; j < BoardPatternSize.width; j++) {
            board_world_space_points.at(i*BoardPatternSize.width + j) = 
            (cv::Point3f(static_cast<float>(j)*camera_board_settings.chessboard.square_length, 
            static_cast<float>(i)*camera_board_settings.chessboard.square_length, 0.0f));
        }
    }


    cv::Point3f Pattern_center = cv::Point3f(
        static_cast<float>(camera_board_settings.chessboard.pattern_size.width-1)*
        camera_board_settings.chessboard.square_length*0.5f,
        static_cast<float>(camera_board_settings.chessboard.pattern_size.height-1)*
        camera_board_settings.chessboard.square_length*0.5f,
        0.0f);

    cv::Point3f Board_center = Pattern_center;
    Board_center.x -= camera_board_settings.chessboard.translation_error.x;
    Board_center.y += camera_board_settings.chessboard.translation_error.y; //y negativo(pra baixo)

    std::vector<cv::Point3f> Board_corners(5);
    //Board_corners.resize(5);

    float b_size_x = camera_board_settings.chessboard.board_dimension.width;
    float b_size_y = camera_board_settings.chessboard.board_dimension.height;

    Board_corners[0] = Board_center-cv::Point3f(b_size_x,b_size_y,0.0f)*0.5f;
    Board_corners[1] = Board_center+cv::Point3f(b_size_x,-b_size_y,0.0f)*0.5f;
    Board_corners[2] = Board_center+cv::Point3f(b_size_x,b_size_y,0.0f)*0.5f;
    Board_corners[3] = Board_center+cv::Point3f(-b_size_x,b_size_y,0.0f)*0.5f;
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
                    recieve_queue_state = camera_recieve_queue->timed_receive(Camera_cpu_buffer.data(), image_pixels_number*3U+8U, recvd_size, priority, abs_time); //image_pixels_number*3U*sizeof(float)
                    std::cout << "Pass7.0"<<std::endl;
                    camera_ctrl_mtx->unlock();
                    std::cout << "Pass7"<<std::endl;
                }else {

                    camera_recieve_queue.reset();
                    //camera_detected_board_queue.reset();

                    camera_recieve_queue = std::make_unique<boost::interprocess::message_queue>(boost::interprocess::open_only, ipc_topics_names.camera_recieve_queue_name.c_str());
                    image_pixels_number = (camera_recieve_queue->get_max_msg_size()-8U)/(3U); //(3U*sizeof(float))

                    //camera_detected_board_queue = std::make_unique<boost::interprocess::message_queue>(boost::interprocess::open_only, "camera_queue_board");

                    Camera_cpu_buffer.resize(image_pixels_number*3U+8U);

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
            //camera_detected_board_queue.reset();
            now = boost::posix_time::microsec_clock::universal_time();
            abs_time = now + boost::posix_time::milliseconds(1000);

            if(!camera_ctrl_mtx->timed_lock(abs_time))
            {
                camera_ctrl_mtx->unlock();
                camera_ctrl_mtx->lock();
            }

            camera_recieve_queue = std::make_unique<boost::interprocess::message_queue>(boost::interprocess::open_only, ipc_topics_names.camera_recieve_queue_name.c_str());
            image_pixels_number = (camera_recieve_queue->get_max_msg_size()-8U)/(3U); //(3U*sizeof(float))
            Camera_cpu_buffer.resize(image_pixels_number*3U+8U);
            //camera_detected_board_queue = std::make_unique<boost::interprocess::message_queue>(boost::interprocess::open_only, "camera_queue_board");
            camera_ctrl_mtx->unlock();

            continue;
        }

        uint32_t real_image_pixels_number = (recvd_size - 8U)/3U;
        if(real_image_pixels_number*3U != (uint32_t)(camera_board_settings.camera.image_size.height*camera_board_settings.camera.image_size.width*3)){
            std::cerr << "Recieved image and expected image have different sizes\n";
            continue;
        }

        uint32_t timestamp_sample_nanosec;
        int32_t timestamp_sample_sec;
        //nanosec
        std::memcpy(&timestamp_sample_nanosec,Camera_cpu_buffer.data()+real_image_pixels_number*3U+4U,4U);
        //timestamp_sample_nanosec = Camera_cpu_buffer.back();
        //Camera_cpu_buffer.pop_back();
        //sec
        std::memcpy(&timestamp_sample_sec,Camera_cpu_buffer.data()+real_image_pixels_number*3U,4U);
        //timestamp_sample_sec = Camera_cpu_buffer.back();
        //Camera_cpu_buffer.pop_back();

        cv::Mat Camera_recieved_image(camera_board_settings.camera.image_size.height, camera_board_settings.camera.image_size.width, CV_8UC3, (void*)Camera_cpu_buffer.data());

        if (Camera_recieved_image.empty()) {
            std::cerr << "Error while casting the recieved image \n";
            continue;
        }


        // Convert to grayscale
        cv::Mat Camera_recieved_image_gray; 
        cv::cvtColor(Camera_recieved_image, Camera_recieved_image_gray, cv::COLOR_RGB2GRAY);

        // --- Detect Chessboard Corners ---
        std::vector<cv::Point2f> Chessboardcorners;
        bool found = cv::findChessboardCorners(Camera_recieved_image_gray, BoardPatternSize, Chessboardcorners,
                                           cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE); //| cv::CALIB_CB_FAST_CHECK);

        
        if(!found){
            CameraColorB.textureID_mutex.lock();
            CameraColorB.board_existence = false;
            CameraColorB.pixels_number = real_image_pixels_number;
            CameraColorB.image_width = camera_board_settings.camera.image_size.width;
            CameraColorB.image_height = camera_board_settings.camera.image_size.height;
            CameraColorB.image_recieved_first_time = true;

            if(CameraColorB.Camera_draw_cpu_buffer.size() != real_image_pixels_number*3U)CameraColorB.Camera_draw_cpu_buffer.resize(real_image_pixels_number*3U);

            uint32_t row_bytes = camera_board_settings.camera.image_size.width * 3U;

            for (uint32_t y = 0U; y < camera_board_settings.camera.image_size.height; y++) {
                const uint8_t* src_row = Camera_cpu_buffer.data() + y * row_bytes;
                uint8_t* dst_row = CameraColorB.Camera_draw_cpu_buffer.data() + (camera_board_settings.camera.image_size.height - 1 - y) * row_bytes;
            
                memcpy(dst_row, src_row, row_bytes);
            }

            CameraColorB.textureID_mutex.unlock();
        }

        //resize if needed

        //glfwMakeContextCurrent(Camera_context);

        //if(inliers_size_npoints>lidar_inliers_size_max_npoints){

        //    lidar_inliers_size_max_npoints = 2U*inliers_size_npoints;

        //    glBindBuffer(GL_ARRAY_BUFFER, LidarInliersDualVBO.VBO[bufindex]);
        //    glBufferData(GL_ARRAY_BUFFER, lidar_inliers_size_max_npoints*sizeof(glm::vec3), NULL, GL_STATIC_DRAW);
        //    glBindBuffer(GL_ARRAY_BUFFER, 0);

        //}

        //upload image

        //glBindTexture(GL_TEXTURE_2D, CameraDualColorB.textureID[texindex]);
        //glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, camera_board_settings.camera.image_size.width, 
        //    camera_board_settings.camera.image_size.height, GL_RGB, GL_UNSIGNED_BYTE, Camera_cpu_buffer.data());
        //glBindTexture(GL_TEXTURE_2D, 0);

        //glfwMakeContextCurrent(main_window);

        if (found)
        {

            try
            {

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
                    //std::cout << "Corner " << i << ": " << Board_corners_camera_coord[i] << std::endl;
                }

                cv::Mat Pw_normal = (cv::Mat_<float>(3,1) << 0.0f, 0.0f, -1.0f);
                cv::Mat Pc_normal = W_to_cam_Rot * Pw_normal;

                cv::Point3f Board_normal_camera_coord = cv::Point3f(
                    -Pc_normal.at<float>(0,0),
                    -Pc_normal.at<float>(1,0), //to model opencv coord (-x,-y,z) to opencv camera coord (x,-y,-z)
                    Pc_normal.at<float>(2,0) //to model opencv coord (-x,-y,z) to opencv camera coord (x,-y,-z)
                );

                //std::cout << "Board Normal: " << Board_normal_camera_coord << std::endl;
                CameraRecieveDataMutex.lock();
                CameraDetectedBoardHistory.push_back();
                LidarCamSim::CameraDetectedBoardData &camera_board_data = CameraDetectedBoardHistory.back();
                //coping to detected board
                for(size_t i = 0; i < (Board_corners_camera_coord.size()); i++){
                    camera_board_data.gpu_camera_detected_board_buffer[i].x = Board_corners_camera_coord[i].x;
                    camera_board_data.gpu_camera_detected_board_buffer[i].y = Board_corners_camera_coord[i].y;
                    camera_board_data.gpu_camera_detected_board_buffer[i].z = Board_corners_camera_coord[i].z;
                }

                camera_board_data.gpu_camera_detected_board_buffer[5].x = Board_normal_camera_coord.x;
                camera_board_data.gpu_camera_detected_board_buffer[5].y = Board_normal_camera_coord.y;
                camera_board_data.gpu_camera_detected_board_buffer[5].z = Board_normal_camera_coord.z;

                //calculating d and writing

                camera_board_data.camera_detected_board_plane_d = -(
                    camera_board_data.gpu_camera_detected_board_buffer[5].x*camera_board_data.gpu_camera_detected_board_buffer[4].x+
                    camera_board_data.gpu_camera_detected_board_buffer[5].y*camera_board_data.gpu_camera_detected_board_buffer[4].y+
                    camera_board_data.gpu_camera_detected_board_buffer[5].z*camera_board_data.gpu_camera_detected_board_buffer[4].z
                    );

                camera_board_data.camera_timestamp_sec = timestamp_sample_sec;
                camera_board_data.camera_timestamp_nanosec = timestamp_sample_nanosec;

                float detected_board_plane_d_temp = camera_board_data.camera_detected_board_plane_d;

                glm::vec4 detected_board_buffer_temp[6];
                std::memcpy(detected_board_buffer_temp,
                    camera_board_data.gpu_camera_detected_board_buffer,6*sizeof(glm::vec4));


                CameraRecieveDataMutex.unlock();

                CameraColorB.textureID_mutex.lock();
                CameraColorB.board_existence = true;
                CameraColorB.pixels_number = real_image_pixels_number;
                CameraColorB.image_width = camera_board_settings.camera.image_size.width;
                CameraColorB.image_height = camera_board_settings.camera.image_size.height;
                CameraColorB.image_recieved_first_time = true;

                uint32_t row_bytes = camera_board_settings.camera.image_size.width * 3U;


                if(CameraColorB.Camera_draw_cpu_buffer.size() != real_image_pixels_number*3U)CameraColorB.Camera_draw_cpu_buffer.resize(real_image_pixels_number*3U);

                for (uint32_t y = 0U; y < camera_board_settings.camera.image_size.height; y++) {
                    const uint8_t* src_row = Camera_cpu_buffer.data() + y * row_bytes;
                    uint8_t* dst_row = CameraColorB.Camera_draw_cpu_buffer.data() + (camera_board_settings.camera.image_size.height - 1 - y) * row_bytes;
                
                    memcpy(dst_row, src_row, row_bytes);
                }

                CameraColorB.board_data.detected_board_plane_d = detected_board_plane_d_temp;

                std::memcpy(CameraColorB.board_data.detected_board_buffer,
                            detected_board_buffer_temp,6*sizeof(glm::vec4));

                CameraColorB.textureID_mutex.unlock();
                //CameraRecieveDataMutex.unlock();
                
                //CameraDualColorB.UnlockWrite();
            }
            catch (boost::interprocess::interprocess_exception& ex) 
            {
                std::cerr << "Error: " << ex.what() << std::endl;
                CameraRecieveDataMutex.unlock();
                //CameraColorB.textureID_mutex.unlock();
                //CameraDualColorB.UnlockWrite();
                continue;
            }

        }
        else
        {   
            //CameraDualColorB.UnlockWrite();
            std::cerr << "Could not detect chessboard pattern.\n";
        }

    }
}

/*
void LidarCamSim::Scene::DrawLidarCameraBoard(){
    //shader use
    if(camera_recieve_debug_board || lidar_recieve_debug_board || render_ground_truth_normal){
        
        glm::mat4 proj_view = viewport_camera_projection_matrix*viewport_camera_view_matrix;

        glm::mat4 normalized_model = ObjectsVector["chessboard"]->object->model_m;
        glm::vec4 vectors[2];
        vectors[0] = normalized_model*glm::vec4(0.0f,0.0f,0.0f,1.0f);
        normalized_model[0] = glm::normalize(normalized_model[0]);
        normalized_model[1] = glm::normalize(normalized_model[1]);
        normalized_model[2] = glm::normalize(normalized_model[2]);

        vectors[1] = normalized_model*glm::vec4(-1.0f,0.0f,0.0f,0.0f);
        


        if(CameraRecieveBoardMutex.try_lock()){
            if(camera_detected_board_recieved_first_time && camera_recieve_debug_board){

                glm::mat4 pv_inverse_virtual = glm::mat4(1.0f);//proj_view*plumb_bob_cam->camera_matrix;

                glBindBuffer(GL_UNIFORM_BUFFER, uboDebugBoard);
                glBufferSubData(GL_UNIFORM_BUFFER, 0, sizeof(gpu_camera_detected_board_buffer), gpu_camera_detected_board_buffer);
                

                CameraDebugBoardShader.use();
                CameraDebugBoardShader.setMat4("proj_view_inverse_virtual_view", pv_inverse_virtual);

                glDisable(GL_CULL_FACE);

                glBindVertexArray(dummyVAO);

                if(camera_board_render_mode[0]){
                    CameraDebugBoardShader.setUint("render_mode", 0);
                    CameraDebugBoardShader.setUint("color_mode", 1);
                    glDrawArrays(GL_TRIANGLES, 0, 6);
                    
                }
                if(camera_board_render_mode[1]){
                    //glLineWidth(4.0f);
                    CameraDebugBoardShader.setUint("render_mode", 1);
                    CameraDebugBoardShader.setUint("color_mode", 2);
                    glDrawArrays(GL_LINE_LOOP, 0, 4);
                    
                }
                if(camera_board_render_mode[2]){
                    CameraDebugBoardShader.setUint("render_mode", 2);
                    CameraDebugBoardShader.setUint("color_mode", 4);
                    glDrawArrays(GL_POINTS, 0, 5);
                    
                }
                if(camera_board_render_mode[3]){
                    CameraDebugBoardShader.setUint("render_mode", 3);
                    CameraDebugBoardShader.setFloat("normal_size", 1.5f);
                    CameraDebugBoardShader.setUint("color_mode", 0);
                    glDrawArrays(GL_LINES, 0, 2);
                }    
                    
                glBindVertexArray(0);
                glEnable(GL_CULL_FACE);
                glBindBuffer(GL_UNIFORM_BUFFER, uboViewportCamera);

                //Compute errors

                //Area error

                glm::vec3 side1 = glm::vec3(gpu_camera_detected_board_buffer[1]-gpu_camera_detected_board_buffer[0]);
                glm::vec3 side2 = glm::vec3(gpu_camera_detected_board_buffer[2]-gpu_camera_detected_board_buffer[0]);

                camera_board_area_error = 0.5f*glm::length(glm::cross(side1,side2));

                side1 = glm::vec3(gpu_camera_detected_board_buffer[0]-gpu_camera_detected_board_buffer[2]);
                side2 = glm::vec3(gpu_camera_detected_board_buffer[3]-gpu_camera_detected_board_buffer[2]);

                camera_board_area_error += 0.5f*glm::length(glm::cross(side1,side2));
                camera_board_area_error = camera_board_area_error*10000.0f;
                camera_board_area_error -= (BoardParams.board_dimension.x*BoardParams.board_dimension.y)/100.0f;
                //Angle error
                glm::vec4 camera_board_normal = plumb_bob_cam->camera_matrix*gpu_camera_detected_board_buffer[5];
                camera_board_angle_error = glm::degrees(std::acos(std::clamp(glm::dot(camera_board_normal,vectors[1]),-1.0f,1.0f)));

                //Position Error
                glm::vec4 camera_board_position = plumb_bob_cam->camera_matrix*gpu_camera_detected_board_buffer[4];
                camera_board_position_error = glm::length((camera_board_position - vectors[0]))*1000.0f;
            }
            CameraRecieveBoardMutex.unlock();
        }


        if(LidarRecieveDataMutex.try_lock()){
            if(lidar_detected_board_recieved_first_time && lidar_recieve_debug_board){

                glm::mat4 pv_inverse_virtual = proj_view*normalized_lidar_model;

                glBindBuffer(GL_UNIFORM_BUFFER, uboDebugBoard);
                glBufferSubData(GL_UNIFORM_BUFFER, 0, sizeof(gpu_lidar_detected_board_buffer), gpu_lidar_detected_board_buffer);

                CameraDebugBoardShader.use();
                CameraDebugBoardShader.setMat4("proj_view_inverse_virtual_view", pv_inverse_virtual);

                glDisable(GL_CULL_FACE);

                glBindVertexArray(dummyVAO);


                if(lidar_board_render_mode[0]){
                    CameraDebugBoardShader.setUint("render_mode", 0);
                    CameraDebugBoardShader.setUint("color_mode", 3);
                    glDrawArrays(GL_TRIANGLES, 0, 6);
                }
                
                if(lidar_board_render_mode[1]){
                    //glLineWidth(4.0f);
                    CameraDebugBoardShader.setUint("render_mode", 1);
                    CameraDebugBoardShader.setUint("color_mode", 5);
                    glDrawArrays(GL_LINE_LOOP, 0, 4);
                }

                if(lidar_board_render_mode[2]){
                    CameraDebugBoardShader.setUint("render_mode", 2);
                    CameraDebugBoardShader.setUint("color_mode", 0);
                    glDrawArrays(GL_POINTS, 0, 5);
                }

                if(lidar_board_render_mode[3]){
                    std::cout<<"gpu_lidar normal x:"<<gpu_lidar_detected_board_buffer[5].x<<std::endl;
                    CameraDebugBoardShader.setUint("render_mode", 3);
                    CameraDebugBoardShader.setFloat("normal_size", 1.0f);
                    CameraDebugBoardShader.setUint("color_mode", 2);
                    glDrawArrays(GL_LINES, 0, 2);
                }
                

                glBindVertexArray(0);
                glEnable(GL_CULL_FACE);
                glBindBuffer(GL_UNIFORM_BUFFER, uboViewportCamera);

                //Compute errors

                //Area error

                glm::vec3 side1 = glm::vec3(gpu_lidar_detected_board_buffer[1]-gpu_lidar_detected_board_buffer[0]);
                glm::vec3 side2 = glm::vec3(gpu_lidar_detected_board_buffer[2]-gpu_lidar_detected_board_buffer[0]);

                lidar_board_area_error = 0.5f*glm::length(glm::cross(side1,side2));

                side1 = glm::vec3(gpu_lidar_detected_board_buffer[0]-gpu_lidar_detected_board_buffer[2]);
                side2 = glm::vec3(gpu_lidar_detected_board_buffer[3]-gpu_lidar_detected_board_buffer[2]);

                lidar_board_area_error += 0.5f*glm::length(glm::cross(side1,side2));
                lidar_board_area_error = lidar_board_area_error*10000.0f;
                lidar_board_area_error -= (BoardParams.board_dimension.x*BoardParams.board_dimension.y)/100.0f;
                //Angle error
                glm::vec4 lidar_board_normal = normalized_lidar_model*gpu_lidar_detected_board_buffer[5];
                lidar_board_angle_error = glm::degrees(std::acos(std::clamp(glm::dot(lidar_board_normal,vectors[1]),-1.0f,1.0f)));

                //Position Error
                glm::vec4 lidar_board_position = normalized_lidar_model*gpu_lidar_detected_board_buffer[4];
                lidar_board_position_error = glm::length((lidar_board_position - vectors[0]))*1000.0f;

            }
            LidarRecieveDataMutex.unlock();
        }

        if(render_ground_truth_normal){
            //glm::mat4 normalized_model = ObjectsVector["chessboard"]->object->model_m;
            //glm::vec4 vectors[2];
            //vectors[0] = normalized_model*glm::vec4(0.0f,0.0f,0.0f,1.0f);
            //normalized_model[0] = glm::normalize(normalized_model[0]);
            //normalized_model[1] = glm::normalize(normalized_model[1]);
            //normalized_model[2] = glm::normalize(normalized_model[2]);

            //vectors[1] = normalized_model*glm::vec4(-1.0f,0.0f,0.0f,0.0f);

            glBindBuffer(GL_UNIFORM_BUFFER, uboDebugBoard);
            glBufferSubData(GL_UNIFORM_BUFFER, 4*sizeof(glm::vec4), 2*sizeof(glm::vec4), vectors);

            CameraDebugBoardShader.use();
            CameraDebugBoardShader.setMat4("proj_view_inverse_virtual_view", proj_view);

            glBindVertexArray(dummyVAO);
            //glLineWidth(2.0f);
            CameraDebugBoardShader.setUint("render_mode", 3);
            CameraDebugBoardShader.setFloat("normal_size", 0.5f);
            CameraDebugBoardShader.setUint("color_mode", 4);
            glDrawArrays(GL_LINES, 0, 2);


            glBindVertexArray(0);
            glBindBuffer(GL_UNIFORM_BUFFER, uboViewportCamera);

        }
    
    }

    draw_statistics_viewport();
}


*/

void LidarCamSim::Scene::InitLidarInfoSharedMem(){
    bbox_segment_ptr = std::make_unique<boost::interprocess::managed_shared_memory>(boost::interprocess::open_or_create, ipc_topics_names.bbox_shared_mem_name.c_str(), 512);
    auto response_lidar_bbox = bbox_segment_ptr->find<LidarBoundsCfg>("bbox_values");
    if (response_lidar_bbox.first) {
        std::cout << "Found bounding box existing SharedControl"<<std::endl;
        lidar_bbox_values_pointer = response_lidar_bbox.first; // attach to existing
    } else {
        std::cout << "Bounding box SharedControl not found, creating"<<std::endl;
        lidar_bbox_values_pointer = bbox_segment_ptr->construct<LidarBoundsCfg>("bbox_values")();
    }

    bbox_ctrl_mtx = std::make_unique<boost::interprocess::named_mutex>(boost::interprocess::open_or_create, ipc_topics_names.bbox_ctrl_mtx_name.c_str());
    boost::posix_time::ptime now = boost::posix_time::microsec_clock::universal_time();
    boost::posix_time::ptime abs_time = now + boost::posix_time::milliseconds(500);

    if(!bbox_ctrl_mtx->timed_lock(abs_time))
    {
        bbox_ctrl_mtx->unlock();
        bbox_ctrl_mtx->lock();
    }

    *lidar_bbox_values_pointer = lidar_bo_settings.lidar_bounds;

    bbox_ctrl_mtx->unlock();
}

void LidarCamSim::Scene::SetBBoxValues(){

    boost::posix_time::ptime now = boost::posix_time::microsec_clock::universal_time();
    boost::posix_time::ptime abs_time = now + boost::posix_time::milliseconds(500);

    if(!bbox_ctrl_mtx->timed_lock(abs_time))
    {
        bbox_ctrl_mtx->unlock();
        bbox_ctrl_mtx->lock();
    }

    *lidar_bbox_values_pointer = lidar_bo_settings.lidar_bounds;

    bbox_ctrl_mtx->unlock();
}

//void LidarCamSim::Scene::InitFIFOS(){
//    fifo_msg_send_size = LidarObject->points.size()*sizeof(Point_XYZIR);
//    fifo_msg_recieve_size = LidarObject->points.size()*3*sizeof(float) + sizeof(lidar_detected_board);
//
//    //boost::interprocess::message_queue::remove("shared_mem");
//
//    segment_ptr = std::make_unique<boost::interprocess::managed_shared_memory>(boost::interprocess::open_or_create, "shared_mem", 512);
//
//    //lidar_resize_notification = segment.construct<SharedControl>("shared_notification")();
//
//    // Look for "Ctrl"
//    auto response_lidar = segment_ptr->find<SharedControl>("shared_notification");
//
//    if (response_lidar.first) {
//        std::cout << "Found existing SharedControl"<<std::endl;
//        lidar_resize_notification = response_lidar.first; // attach to existing
//    } else {
//        std::cout << "Not found, creating"<<std::endl;
//        lidar_resize_notification = segment_ptr->construct<SharedControl>("shared_notification")();
//        lidar_resize_notification->resized = true;
//    }
//    
//    lidar_ctrl_mtx = std::make_unique<boost::interprocess::named_mutex>(boost::interprocess::open_or_create, "lidar_ctrl_mtx");
//
//    boost::posix_time::ptime now = boost::posix_time::microsec_clock::universal_time();
//    boost::posix_time::ptime abs_time = now + boost::posix_time::milliseconds(500);
//
//    //boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*lidar_ctrl_mtx.get(), boost::interprocess::defer_lock);
//
//    if(!lidar_ctrl_mtx->timed_lock(abs_time))
//    {
//        lidar_ctrl_mtx->unlock();
//        lidar_ctrl_mtx->lock();
//    }
//
//    lidar_resize_notification->resized = true;
//
//    boost::interprocess::message_queue::remove("pts_queue");
//    send_queue = std::make_unique<boost::interprocess::message_queue>(
//                                    boost::interprocess::create_only,
//                                    "pts_queue",
//                                    /*max msgs*/  5,
//                                    /*max size*/  fifo_msg_send_size);
//
//    boost::interprocess::message_queue::remove("pts_queue_result");
//    recieve_queue = std::make_unique<boost::interprocess::message_queue>(
//                                    boost::interprocess::create_only,
//                                    "pts_queue_result",
//                                    /*max msgs*/  5,
//                                    /*max size*/  fifo_msg_recieve_size);
//    lidar_ctrl_mtx->unlock();
//
//
//    camera_message_size = static_cast<unsigned int>(plumb_bob_cam->m_windowWidth)*
//                                       static_cast<unsigned int>(plumb_bob_cam->m_windowHeight)*
//                                       3U;//*sizeof(float)
//
//    camera_detected_board_message_size = sizeof(camera_detected_board);
//
//    auto response_camera = segment_ptr->find<SharedControl>("shared_camera_notification");
//
//    if (response_camera.first) {
//        std::cout << "Found existing SharedCameraControl"<<std::endl;
//        camera_resize_notification = response_camera.first; // attach to existing
//    } else {
//        std::cout << "Not found, creating SharedCameraControl"<<std::endl;
//        camera_resize_notification = segment_ptr->construct<SharedControl>("shared_camera_notification")();
//        camera_resize_notification->resized = true;
//    }
//    
//    camera_ctrl_mtx = std::make_unique<boost::interprocess::named_mutex>(boost::interprocess::open_or_create, "camera_ctrl_mtx");
//
//    now = boost::posix_time::microsec_clock::universal_time();
//    abs_time = now + boost::posix_time::milliseconds(1000);
//
//    //boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*lidar_ctrl_mtx.get(), boost::interprocess::defer_lock);
//
//    if(!camera_ctrl_mtx->timed_lock(abs_time))
//    {
//        camera_ctrl_mtx->unlock();
//        camera_ctrl_mtx->lock();
//    }
//
//    camera_resize_notification->resized = true;
//
//    boost::interprocess::message_queue::remove("camera_queue");
//    camera_send_queue = std::make_unique<boost::interprocess::message_queue>(
//                                    boost::interprocess::create_only,
//                                    "camera_queue",
//                                    /*max msgs*/  5,
//                                    /*max size*/  camera_message_size); 
//
//    boost::interprocess::message_queue::remove("camera_queue_board");
//    camera_detected_board_queue = std::make_unique<boost::interprocess::message_queue>(
//                                    boost::interprocess::create_only,
//                                    "camera_queue_board",
//                                    /*max msgs*/  5,
//                                    /*max size*/  camera_detected_board_message_size);
//
//    camera_ctrl_mtx->unlock();
//}

//void LidarCamSim::Scene::RecreateLidarBroadcastFIFO(){
//
//
//    fifo_msg_send_size = LidarObject->points.size()*sizeof(Point_XYZIR);
//
//    boost::interprocess::message_queue::remove("pts_queue");
//    send_queue.reset();
//    send_queue = std::make_unique<boost::interprocess::message_queue>(
//                                    boost::interprocess::create_only,
//                                    "pts_queue",
//                                    /*max msgs*/  5,
//                                    /*max size*/  fifo_msg_send_size);
//
//}


//void LidarCamSim::Scene::RecreateLidarInliersFIFO(){
//
//    fifo_msg_recieve_size = LidarObject->points.size()*3*sizeof(float) + sizeof(lidar_detected_board);
//
//    boost::interprocess::message_queue::remove("pts_queue_result");
//    recieve_queue.reset();
//    recieve_queue = std::make_unique<boost::interprocess::message_queue>(
//                                    boost::interprocess::create_only,
//                                    "pts_queue_result",
//                                    /*max msgs*/  5,
//                                    /*max size*/  fifo_msg_recieve_size);
//
//}
//

//void LidarCamSim::Scene::RecreateCameraFIFOS(){
//
//    boost::interprocess::message_queue::remove("camera_queue");
//    camera_send_queue.reset();
//    camera_send_queue = std::make_unique<boost::interprocess::message_queue>(
//                                    boost::interprocess::create_only,
//                                    "camera_queue",
//                                    /*max msgs*/  5,
//                                    /*max size*/  camera_message_size); 
//
//    boost::interprocess::message_queue::remove("camera_queue_board");
//    camera_detected_board_queue.reset();
//    camera_detected_board_queue = std::make_unique<boost::interprocess::message_queue>(
//                                    boost::interprocess::create_only,
//                                    "camera_queue_board",
//                                    /*max msgs*/  5,
//                                    /*max size*/  camera_detected_board_message_size);
//
//}

/*
void LidarCamSim::Scene::RecieveLidarInliersAsync(){

if(lidar_recieve_debug_cloud){

    if(LidarRecieveDataMutex.try_lock()){

        if(recieve_queue->get_num_msg() > 0){

            //prepare

            //func
            std::thread Lidar_recieve_inliers_data_func([=]
            {
            
            uint64_t recvd_size;
            uint32_t priority;

            if(recieve_queue->try_receive(lidar_inliers_recieved.data(), lidar_inliers_recieved.size()*sizeof(glm::vec3), recvd_size, priority)){
                //lidar_points.resize(recvd_size / sizeof(Point_XYZIR));
                glfwMakeContextCurrent(Lidar_inliers_context);
                std::cout<<"recieved size: "<<recvd_size<<std::endl;
                recvd_size -= sizeof(lidar_detected_board);
                inliers_msg_recieve_size = static_cast<uint32_t>(recvd_size);

                glBindBuffer(GL_ARRAY_BUFFER, LidarInliersVBO);
                glBufferSubData(GL_ARRAY_BUFFER, 0, recvd_size, lidar_inliers_recieved.data());


                glBindBuffer(GL_ARRAY_BUFFER, 0);

                uint32_t lidar_inliers_board_start = recvd_size/sizeof(glm::vec3);

                std::memcpy(lidar_detected_board,lidar_inliers_recieved.data()+lidar_inliers_board_start,sizeof(lidar_detected_board));

                for (int i = 0; i < 5; i++) {
                    gpu_lidar_detected_board_buffer[i] = glm::vec4(
                        lidar_detected_board[i * 3 + 0],
                        lidar_detected_board[i * 3 + 1],
                        lidar_detected_board[i * 3 + 2],
                        1.0f    // padding; you could also use 0.0f or something else
                    );
                }

                gpu_lidar_detected_board_buffer[5] = glm::vec4(
                    lidar_detected_board[15],
                    lidar_detected_board[16],
                    lidar_detected_board[17],
                    0.0f    // padding; you could also use 1.0f or something else
                );
                for(int i = 0;i<21;i++){
                    std::cout<<"["<<lidar_detected_board[i] <<"]";
                }
                std::cout<<std::endl;
                inliers_recieved_first_time = true;
                lidar_detected_board_recieved_first_time = true;
                glfwMakeContextCurrent(NULL);
            }

            LidarRecieveDataMutex.unlock();
            
           });

           Lidar_recieve_inliers_data_func.detach();
        }
        else{
            LidarRecieveDataMutex.unlock();
        }
    }
}

}
*/


void LidarCamSim::Scene::Draw_Lidar_Board_Inliers(){

    if(lidar_view_debug_cloud){
        
        ShadersVector["lidar_inliers_shader"]->use();

        ShadersVector["lidar_inliers_shader"]->setMat4("lidar_model", normalized_lidar_model);
    
        //pega parametros do dual buffer a salva para toda a iteração
        //if(LidarDrawInliersMutex.try_lock()){
        LidarInliersVBO.VBO_mutex.lock();

        if(LidarInliersVBO.points_recieved_first_time){

            //unsigned int inliers_data_size = (LidarDetectedBoardHistory.back().lidar_inliers->size())/sizeof(glm::vec3);

            RenderFrameLidarBoard = LidarInliersVBO.board_data;
            lidar_detected_board_recieved_first_time = true;

            glBindBuffer(GL_COPY_READ_BUFFER, LidarInliersVBO.VBO);
            glBindBuffer(GL_COPY_WRITE_BUFFER, LidarInliersVBO_main);

            glCopyBufferSubData(
                GL_COPY_READ_BUFFER,
                GL_COPY_WRITE_BUFFER,
                0,   // usually 0
                0,   // usually 0
                LidarInliersVBO.lidar_points_number*sizeof(glm::vec3) // number of bytes to copy
            );
            LidarInliersVBO.VBO_mutex.unlock();

            glBindVertexArray(LidarInliersVAO_main);

            glBindBuffer(GL_ARRAY_BUFFER, LidarInliersVBO_main);
            //glEnableVertexAttribArray(0);
            //glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);

            glDrawArrays(GL_POINTS, 0, LidarInliersVBO.lidar_points_number);
            glBindVertexArray(0);
        }else{
            LidarInliersVBO.VBO_mutex.unlock();
        }
        //LidarDrawInliersMutex.unlock();
        //}
    }
}

void LidarCamSim::Scene::Draw_Lidar_Points_Limited(){


    if(lidar_view_limited_cloud){
        
        ShadersVector["lidar_inliers_shader"]->use();

        ShadersVector["lidar_inliers_shader"]->setMat4("lidar_model", normalized_lidar_model);
    
        //if(LidarPointsVisualizeMutex.try_lock()){
        LidarPointsVisualizeVBO.VBO_mutex.lock();

        if(LidarPointsVisualizeVBO.points_recieved_first_time){

            glBindBuffer(GL_COPY_READ_BUFFER, LidarPointsVisualizeVBO.VBO);
            glBindBuffer(GL_COPY_WRITE_BUFFER, LidarPointsVisualizeVBO_main);

            glCopyBufferSubData(
                GL_COPY_READ_BUFFER,
                GL_COPY_WRITE_BUFFER,
                0,   // usually 0
                0,   // usually 0
                LidarPointsVisualizeVBO.lidar_points_number*sizeof(glm::vec3) // number of bytes to copy
            );
            LidarPointsVisualizeVBO.VBO_mutex.unlock();

            glBindVertexArray(LidarPointsVisualizeVAO_main);

            glBindBuffer(GL_ARRAY_BUFFER, LidarPointsVisualizeVBO_main);
            //glEnableVertexAttribArray(0);
            //glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);

            glDrawArrays(GL_POINTS, 0, LidarPointsVisualizeVBO.lidar_points_number);
            glBindVertexArray(0);
        }else{
            LidarPointsVisualizeVBO.VBO_mutex.unlock();
        }
        //LidarPointsVisualizeMutex.unlock();
        //}
        
    
    }

}

void LidarCamSim::Scene::Get_Camera_Board_Image(){

    //if(recieve_camera_data){
        
        //ShadersVector["lidar_inliers_shader"]->use();

        //ShadersVector["lidar_inliers_shader"]->setMat4("lidar_model", normalized_lidar_model);
    
        //pega parametros do dual buffer a salva para toda a iteração
        //if(LidarDrawInliersMutex.try_lock()){

        CameraColorB.textureID_mutex.lock();
        //uint8_t idx = CameraDualColorB.rtexture_index;

        if(CameraColorB.image_recieved_first_time){


            //glBindVertexArray(LidarInliersVAO_main);

            //glCopyImageSubData(
            //CameraDualColorB.textureID[idx], GL_TEXTURE_2D, 0, 0, 0, 0,   // source texture
            //RenderFrameCameraTexture, GL_TEXTURE_2D, 0, 0, 0, 0,   // destination texture
            //CameraDualColorB.image_width[idx], CameraDualColorB.image_height[idx], 1                     // copy region
            //);

            glBindTexture(GL_TEXTURE_2D, RenderFrameCameraTexture);
            glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, camera_board_settings.camera.image_size.width, 
                camera_board_settings.camera.image_size.height, GL_RGB, GL_UNSIGNED_BYTE, CameraColorB.Camera_draw_cpu_buffer.data());
            glBindTexture(GL_TEXTURE_2D, 0);

            if(CameraColorB.board_existence){
                RenderFrameCameraBoard = CameraColorB.board_data;
                camera_detected_board_recieved_first_time = true;
            }
        }
        //LidarDrawInliersMutex.unlock();
        //}
        CameraColorB.textureID_mutex.unlock();
    //}
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
