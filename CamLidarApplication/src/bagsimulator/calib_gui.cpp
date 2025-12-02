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

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <lib_opengl/shader.h>

#include <texture_loader.hpp>
#include <object_loader.hpp>

#include <lib_opengl/camera_quat.h>

#include <iostream>

#include <vector>
#include <memory>

#include <calib_scene.hpp>

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void RenderMainDockspace();
void SetupImGuiStyle();
// settings

const unsigned int SCR_WIDTH = 1300;
const unsigned int SCR_HEIGHT = 600;

ImGuiWindowFlags windowFlags_global = ImGuiWindowFlags_None;
ImWchar icons_ranges[] = { 0xe900, 0xe90c, 0 };

GLFWcursor* arrowCursor;
GLFWcursor* dragCursor;
GLFWcursor* invisible_cursor;

ImFont *font;
ImFont *font_mono;
ImFont *font_mono_offset;
ImFont *icons;


// timing
double deltaTime = 0.0;
double lastFrame = 0.0;

GLFWmonitor* monitor;
const GLFWvidmode* mode;

GLFWwindow* window;

int main_window_current_width;
int main_window_current_height;

int screenWidth;
int screenHeight;

std::shared_ptr<LidarCamSim::Scene> main_scene_class;

////////////ArcBallCamera camera(SCR_WIDTH, SCR_HEIGHT,glm::vec3(0.0f, 0.0f, 0.0f), 2.0f,0.0f,0.0f);

bool fly_mode = 0;

int main()
{
    // glfw: initialize and configure
    // ------------------------------
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    //glfwInitHint(GLFW_PLATFORM, GLFW_PLATFORM_X11);

#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    // glfw window creation
    // --------------------
    window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "CamLidar Simulation Tool", NULL, NULL);
    if (window == NULL)
    {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);

    // glad: load all OpenGL function pointers
    // ---------------------------------------
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return -1;
    }

    monitor = glfwGetPrimaryMonitor();
    mode = glfwGetVideoMode(monitor);

    screenWidth  = mode->width;
    screenHeight = mode->height;

    arrowCursor = glfwCreateStandardCursor(GLFW_ARROW_CURSOR);
    dragCursor  = glfwCreateStandardCursor(GLFW_CROSSHAIR_CURSOR);
    // Create a 1x1 pixel transparent image
    unsigned char pixels[4] = { 0, 0, 0, 0 }; // RGBA
    GLFWimage image = { 1, 1, pixels }; // 1x1 pixel image
    // Create a cursor from the image
    invisible_cursor = glfwCreateCursor(&image, 0, 0);
     

    // configure global opengl state
    // -----------------------------
    glfwSwapInterval(1); // Enable vsync
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_PROGRAM_POINT_SIZE);


    // Setup ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    //io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;

    ImFontConfig cfg;
    cfg.FontBuilderFlags = ImGuiFreeTypeBuilderFlags_ForceAutoHint; // or LightHinting
    cfg.OversampleH = 2;
    cfg.OversampleV = 1;
    //cfg.MergeMode = true;
    font = io.Fonts->AddFontFromFileTTF("../resources/fonts/AdwaitaSans-Regular.ttf", 18.0f, &cfg, io.Fonts->GetGlyphRangesDefault());
    icons = io.Fonts->AddFontFromFileTTF("../resources/fonts/icons.ttf", 18.0f, &cfg, icons_ranges);
    cfg.OversampleV = 2;
    cfg.OversampleH = 2;
    font_mono = io.Fonts->AddFontFromFileTTF("../resources/fonts/AdwaitaMono-Regular.ttf", 18.0f, &cfg, io.Fonts->GetGlyphRangesDefault());
    cfg.OversampleV = 1;
    cfg.OversampleH = 2;
    cfg.GlyphOffset = ImVec2(1.0f, 0.0f);
    font_mono_offset = io.Fonts->AddFontFromFileTTF("../resources/fonts/AdwaitaMono-Regular.ttf", 18.0f, &cfg, io.Fonts->GetGlyphRangesDefault());
    cfg.MergeMode = true;
    //cfg.GlyphOffset = ImVec2(0.0f, 0.0f);
    font_mono_offset = io.Fonts->AddFontFromFileTTF("../resources/fonts/icons.ttf", 14.0f, &cfg, icons_ranges);
    //ImGuiFreeType::BuildFontAtlas(io.Fonts, cfg.FontBuilderFlags);
    io.Fonts->Build(); // or let backend do it


    SetupImGuiStyle();


    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 460");

///////////////////////////////////////////////////////////////

    std::shared_ptr<std::vector<unsigned int>> global_uniform_binding_indexes = std::make_shared<std::vector<unsigned int>>();

    global_uniform_binding_indexes->reserve(30);

    for(unsigned int i = 29; i > 0 ; i--){
        global_uniform_binding_indexes->push_back(i);
    }
    global_uniform_binding_indexes->push_back(0);

    main_scene_class = std::make_shared<LidarCamSim::Scene>(
        window,screenWidth,screenHeight,
        font,font_mono,font_mono_offset,
        icons,arrowCursor,dragCursor,
        invisible_cursor, icons_ranges,io,
        ImGui::GetStyle(),windowFlags_global,
        global_uniform_binding_indexes
    );

    LidarCamSim::ViewportCamConfig view_cfg;

    view_cfg.pivot_position = glm::vec3(0.0f,0.0f,0.0f);
    view_cfg.pivot_distance = 2.0f;
    view_cfg.yaw_pitch = glm::vec2(0.0f,0.0f);

    LidarCamSim::VirtualCamConfig real_cam_cfg;

    real_cam_cfg.CamWidth = 1000;
    real_cam_cfg.CamHeight = 600;
    real_cam_cfg.CamVfov = 45.0f;
    real_cam_cfg.Render_Ratio = 1.0;
    real_cam_cfg.position = glm::vec3(-2.65f,1.1f,1.3f);
    real_cam_cfg.yaw_pitch_roll = glm::vec3(115.0f,0.0f,0.0f);
    real_cam_cfg.radialDistortion = glm::vec3(-0.169694115533936f, 0.04075467706851231f,0.0f);
    real_cam_cfg.tangentialDistortion = glm::vec2(-0.004826682535351785f, 0.0008093718878340755f);

    main_scene_class->InitCameras(view_cfg,real_cam_cfg);
    main_scene_class->InitLidarInfoSharedMem();
    main_scene_class->InitLidarCameraThreads();

    // build and compile our shader program
    // ------------------------------------

    //Shader lineShader("../shaders/gui_test/line.vs", "../shaders/gui_test/line.fs");

    //enable broadcasting
    //main_scene_class->InitFIFOS();

    // load textures
    // -------------
    
////////////////////////////////////////////////////////////////////////////////
    lastFrame = glfwGetTime();

    //glEnable(GL_MULTISAMPLE);  

    // render loop
    // -----------
    while (!glfwWindowShouldClose(window))
    {
        if (glfwGetWindowAttrib(window, GLFW_ICONIFIED) != 0)
        {
            //ImGui_ImplGlfw_Sleep(10);
            glfwWaitEvents();
            continue;
        }

        // per-frame time logic
        // --------------------
        double currentFrame = glfwGetTime();
        float LidarScanTime = static_cast<float>(std::fmod(currentFrame,5000.0));
        deltaTime = static_cast<float>(currentFrame - lastFrame);
        lastFrame = currentFrame;
        // input
        // -----
        glfwPollEvents();
/////////////////////////////////////////////////////////////////////////////////////
        main_scene_class->processInput(deltaTime);

////////////////////////////////////////////////////////////////////////////
        //camera.OnRender(deltaTime);


        glBindFramebuffer(GL_FRAMEBUFFER, 0);

        // and tell our program that we'll create a ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame(); 
        ImGui::NewFrame();
        RenderMainDockspace(); 

        if(fly_mode) windowFlags_global |= ImGuiWindowFlags_NoInputs;



/////////////////////////////////////////////////////////////////////////////////////////////////
        main_scene_class->draw_viewport();
//////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////


        main_scene_class->draw_loaded_camera_params();
        main_scene_class->draw_recieved_cam_image_viewport();

        main_scene_class->draw_lidar_cloud_settings();
        main_scene_class->draw_transform_settings();
        //main_scene_class->LidarCalculateNormalizedModel();
        main_scene_class->draw_camera_lidar_matrix();
        //main_scene_class->draw_board_settings();
        main_scene_class->draw_statistics_viewport();

        main_scene_class->draw_calibration_settings();
        main_scene_class->draw_calibration_log();


        
/////////////////////////////////////////////////////////////

        //ImGui::Render();
///////////////////////////////////////////////////////////////

        main_scene_class->RenderInfoBuffer();
        main_scene_class->Verify_clicked_object();

        main_scene_class->RenderStartMainViewport();
        main_scene_class->RenderMainViewportAuto();
        main_scene_class->draw_outline();
        //main_scene_class->LidarDrawPoints();
        //main_scene_class->Draw_Lidar_inliers();
        //draw lidar board
        //draw camera board
        //main_scene_class->DrawLidarCameraBoard();
        main_scene_class->Get_Camera_Board_Image();
        /////////////////////////////////////////////////////////////
        ImGui::Render();
        ///////////////////////////////////////////////////////////////
        //main_scene_class->RecieveLidarInliersAsync(); //aquire lidar board
        //aquire camera board
        //main_scene_class->RecieveCameraBoardAsync();
        main_scene_class->draw_grid();
        main_scene_class->RenderEndMainViewport();

        

        //main_scene_class->StartRenderVirtual();
        //main_scene_class->AutoRenderVirtual();
        //main_scene_class->EndRenderVirtual();

        //main_scene_class->SendCameraImageAsync();

        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        glViewport(0, 0, main_window_current_width, main_window_current_height);
        glDisable(GL_DEPTH_TEST); // disable depth test so screen-space quad isn't discarded due to depth test.
        glDisable(GL_CULL_FACE);
        glEnable(GL_BLEND);
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);


//////////////////////////////////////////////////////////////////////////////////////////////
        // and we have to pass the render data further
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());	
        if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable)
        {
    	    GLFWwindow* backup_current_context = glfwGetCurrentContext();
    	    ImGui::UpdatePlatformWindows();
    	    ImGui::RenderPlatformWindowsDefault();
    	    glfwMakeContextCurrent(backup_current_context);
        }

//////////////////////////////////////////////////////////////////////////////////////////


        // glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
        // -------------------------------------------------------------------------------
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // some ImGui cleanups here
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    // optional: de-allocate all resources once they've outlived their purpose:
    // ------------------------------------------------------------------------
    //glDeleteVertexArrays(1, &VAO);
    //glDeleteBuffers(1, &VBO);

    // glfw: terminate, clearing all previously allocated GLFW resources.
    // ------------------------------------------------------------------
    glfwTerminate();
    return 0;
}

// process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
// ---------------------------------------------------------------------------------------------------------
// process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
// ---------------------------------------------------------------------------------------------------------





// glfw: whenever the window size changed (by OS or user resize) this callback function executes
// ---------------------------------------------------------------------------------------------
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    // make sure the viewport matches the new window dimensions; note that width and 
    // height will be significantly larger than specified on retina displays.
    main_window_current_width = width;
    main_window_current_height = height;
    //glViewport(0, 0, width, height);
}

static bool first_time = true;

void RenderMainDockspace()
{
    static ImGuiDockNodeFlags dockspace_flags = ImGuiDockNodeFlags_None | 
                                                ImGuiDockNodeFlags_PassthruCentralNode;

    ImGuiWindowFlags window_flags = ImGuiWindowFlags_MenuBar |
                                    ImGuiWindowFlags_NoDocking |
                                    ImGuiWindowFlags_NoTitleBar |
                                    ImGuiWindowFlags_NoCollapse |
                                    ImGuiWindowFlags_NoResize |
                                    ImGuiWindowFlags_NoMove |
                                    ImGuiWindowFlags_NoBringToFrontOnFocus |
                                    ImGuiWindowFlags_NoNavFocus |
                                    windowFlags_global;



    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(viewport->Pos);
    ImGui::SetNextWindowSize(viewport->Size);
    ImGui::SetNextWindowViewport(viewport->ID);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
    
    ImGuiStyle& style = ImGui::GetStyle();
    style.Colors[ImGuiCol_WindowBg] = ImVec4(0.1f, 0.1f, 0.1f, 1.0f);

    // Important: Enable menu bar so you can have embedded menus
    ImGui::Begin("MainDockSpace", nullptr, window_flags);
    ImGui::PopStyleVar(3);

    // Create the docking space
    ImGuiID dockspace_id = ImGui::GetID("MyDockSpace");
    ImGui::DockSpace(dockspace_id, ImVec2(0.0f, 0.0f), dockspace_flags);


    if (ImGui::BeginMenuBar())
    {   
        //bool show_lidar_settings;
        if (ImGui::BeginMenu("Main"))
        {
            //if (ImGui::Checkbox("Show Lidar Settings", &show_lidar_settings)) { /* Do stuff */ }
            //if (ImGui::MenuItem("Save", "Ctrl+S"))   { /* Do stuff */ }
            if (ImGui::MenuItem("Save Params"))  {glfwSetWindowShouldClose(window, true);}
            if (ImGui::MenuItem("Load Params"))  {glfwSetWindowShouldClose(window, true);}
            if (ImGui::MenuItem("Close", "Esc"))  {glfwSetWindowShouldClose(window, true);}
            ImGui::EndMenu();
        }
        ImGui::EndMenuBar();
    }



    if (first_time)
    {
        first_time = false;
        ImGui::DockBuilderRemoveNode(dockspace_id); // clear previous layout
        ImGui::DockBuilderAddNode(dockspace_id, ImGuiDockNodeFlags_DockSpace);
        ImGui::DockBuilderSetNodeSize(dockspace_id, viewport->Size);

        static ImGuiID dock_id_left1;
        static ImGuiID dock_id_right1;
        static ImGuiID dock_id_center;
        static ImGuiID dock_id_center_up;
        static ImGuiID dock_id_center_down;
        static ImGuiID dock_id_down1_cam;
        static ImGuiID dock_id_rightdow1;

        ImGui::DockBuilderSplitNode(dockspace_id, ImGuiDir_Left, 0.25f, &dock_id_left1, &dock_id_center);
        ImGui::DockBuilderSplitNode(dock_id_center, ImGuiDir_Right, 0.38f, &dock_id_right1, &dock_id_center);
        ImGui::DockBuilderSplitNode(dock_id_right1, ImGuiDir_Down, 0.5f, &dock_id_rightdow1, &dock_id_right1);
        ImGui::DockBuilderSplitNode(dock_id_left1, ImGuiDir_Down, 0.5f, &dock_id_down1_cam, &dock_id_left1);

        ImGui::DockBuilderSplitNode(dock_id_center, ImGuiDir_Up, 0.75f, &dock_id_center_up, &dock_id_center_down);

        ImGui::DockBuilderDockWindow("Camera Viewport", dock_id_left1);
        ImGui::DockBuilderDockWindow("Statistics", dock_id_left1);
        ImGui::DockBuilderDockWindow("Camera Parameters", dock_id_down1_cam);
        ImGui::DockBuilderDockWindow("Camera Projection Matrix", dock_id_down1_cam);
        ImGui::DockBuilderDockWindow("Lidar Cloud Settings", dock_id_right1);
        ImGui::DockBuilderDockWindow("Board Settings", dock_id_right1);
        ImGui::DockBuilderDockWindow("Camera Lidar Matrix", dock_id_right1);
        ImGui::DockBuilderDockWindow("Object Transform", dock_id_rightdow1);
        //ImGui::DockBuilderDockWindow("Object Model Matrix", dock_id_rightdow1);
        ImGui::DockBuilderDockWindow("Calibration Settings", dock_id_rightdow1);
        ImGui::DockBuilderDockWindow("Viewport", dock_id_center_up); //Locked in center
        ImGui::DockBuilderDockWindow("Calibration Logs", dock_id_center_down); //Locked in center

        ImGui::DockBuilderFinish(dockspace_id);
    }

    ImGui::End();

    style.Colors[ImGuiCol_WindowBg] = ImVec4(0.1882352977991104f, 0.1882352977991104f, 0.1882352977991104f, 1.0f);
}


//void render_cube(const unsigned int &framebuffer,const unsigned int &width,const unsigned int &height){
//    
//}

void SetupImGuiStyle()
{
	// Lidar_engine style from ImThemes
    ImGuiStyle& style = ImGui::GetStyle();

	style.Alpha = 1.0f;
	style.DisabledAlpha = 0.2000000029802322f;
	style.WindowPadding = ImVec2(8.0f, 8.0f);
	style.WindowRounding = 10.0f;
	style.WindowBorderSize = 2.0f;
	style.WindowMinSize = ImVec2(30.0f, 30.0f);
	style.WindowTitleAlign = ImVec2(0.5f, 0.5f);
	style.WindowMenuButtonPosition = ImGuiDir_Right;
	style.ChildRounding = 5.0f;
	style.ChildBorderSize = 1.0f;
	style.PopupRounding = 10.0f;
	style.PopupBorderSize = 0.0f;
	style.FramePadding = ImVec2(5.0f, 3.5f);
	style.FrameRounding = 5.0f;
	style.FrameBorderSize = 0.0f;
	style.ItemSpacing = ImVec2(5.0f, 4.0f);
	style.ItemInnerSpacing = ImVec2(5.0f, 5.0f);
	style.CellPadding = ImVec2(4.0f, 2.0f);
	style.IndentSpacing = 5.0f;
	style.ColumnsMinSpacing = 5.0f;
	style.ScrollbarSize = 15.0f;
	style.ScrollbarRounding = 9.0f;
	style.GrabMinSize = 15.0f;
	style.GrabRounding = 5.0f;
	style.TabRounding = 5.0f;
	style.TabBorderSize = 0.0f;
	style.TabMinWidthForCloseButton = 0.0f;
	style.ColorButtonPosition = ImGuiDir_Right;
	style.ButtonTextAlign = ImVec2(0.5f, 0.5f);
	style.SelectableTextAlign = ImVec2(0.0f, 0.0f);

    style.TabBarOverlineSize = 0.0f;

    style.DockingSeparatorSize = 2.0f;
    style.AntiAliasedFill = true;

    style.Colors[ImGuiCol_DockingPreview] = ImVec4(0.8f, 0.8f, 0.8f, 0.4f);
    //style.Colors[ImGuiCol_DockingEmptyBg] = ImVec4(0.0f, 0.0f, 0.0f, 1.0f);
	
	style.Colors[ImGuiCol_Text] = ImVec4(1.0f, 1.0f, 1.0f, 1.0f);
	style.Colors[ImGuiCol_TextDisabled] = ImVec4(1.0f, 1.0f, 1.0f, 0.553648054599762f);
	style.Colors[ImGuiCol_WindowBg] = ImVec4(0.1882352977991104f, 0.1882352977991104f, 0.1882352977991104f, 1.0f);
	style.Colors[ImGuiCol_ChildBg] = ImVec4(1.0f, 0.0f, 0.0f, 0.0f);
	style.Colors[ImGuiCol_PopupBg] = ImVec4(0.1137254908680916f, 0.1137254908680916f, 0.1137254908680916f, 1.0f);
	//style.Colors[ImGuiCol_Border] = ImVec4(0.4235294163227081f, 0.3803921639919281f, 0.572549045085907f, 0.54935622215271f);
    style.Colors[ImGuiCol_Border] = ImVec4(0.4737301170825958f, 0.4568144679069519f, 0.5321888327598572f, 0.54935622215271f);
	style.Colors[ImGuiCol_BorderShadow] = ImVec4(0.0f, 0.0f, 0.0f, 0.0f);
	style.Colors[ImGuiCol_FrameBg] = ImVec4(0.1137254908680916f, 0.1137254908680916f, 0.1137254908680916f, 1.0f);
	style.Colors[ImGuiCol_FrameBgHovered] = ImVec4(0.3803921639919281f, 0.4235294163227081f, 0.572549045085907f, 0.5490196347236633f);
	style.Colors[ImGuiCol_FrameBgActive] = ImVec4(0.6196078658103943f, 0.5764706134796143f, 0.7686274647712708f, 0.5490196347236633f);
	style.Colors[ImGuiCol_TitleBg] = ImVec4(0.283261775970459f, 0.2820460498332977f, 0.2820460498332977f, 1.0f);
	style.Colors[ImGuiCol_TitleBgActive] = ImVec4(0.3047210574150085f, 0.304718017578125f, 0.304718017578125f, 1.0f);
	style.Colors[ImGuiCol_TitleBgCollapsed] = ImVec4(0.2823529541492462f, 0.2823529541492462f, 0.2823529541492462f, 1.0f);
	style.Colors[ImGuiCol_MenuBarBg] = ImVec4(0.2575107216835022f, 0.2575081288814545f, 0.2575081288814545f, 0.6137338876724243f);
	style.Colors[ImGuiCol_ScrollbarBg] = ImVec4(0.1568627506494522f, 0.1568627506494522f, 0.1568627506494522f, 0.0f);
	style.Colors[ImGuiCol_ScrollbarGrab] = ImVec4(0.2918455004692078f, 0.2918425798416138f, 0.2918425798416138f, 1.0f);
	style.Colors[ImGuiCol_ScrollbarGrabHovered] = ImVec4(0.3948497772216797f, 0.3948458135128021f, 0.3948458135128021f, 1.0f);
	style.Colors[ImGuiCol_ScrollbarGrabActive] = ImVec4(0.6652360558509827f, 0.6652293801307678f, 0.6652293801307678f, 1.0f);
	style.Colors[ImGuiCol_CheckMark] = ImVec4(0.9227467775344849f, 0.9227375388145447f, 0.9227375388145447f, 1.0f);
	style.Colors[ImGuiCol_SliderGrab] = ImVec4(0.6196078658103943f, 0.5764706134796143f, 0.7686274647712708f, 0.5490196347236633f);
	style.Colors[ImGuiCol_SliderGrabActive] = ImVec4(0.8156862854957581f, 0.772549033164978f, 0.9647058844566345f, 0.5490196347236633f);
	style.Colors[ImGuiCol_Button] = ImVec4(0.6196078658103943f, 0.5764706134796143f, 0.7686274647712708f, 0.5490196347236633f);
	style.Colors[ImGuiCol_ButtonHovered] = ImVec4(0.7372549176216125f, 0.6941176652908325f, 0.886274516582489f, 0.5490196347236633f);
	style.Colors[ImGuiCol_ButtonActive] = ImVec4(0.8156862854957581f, 0.772549033164978f, 0.9647058844566345f, 0.5490196347236633f);
	style.Colors[ImGuiCol_Header] = ImVec4(0.6196078658103943f, 0.5764706134796143f, 0.7686274647712708f, 0.5490196347236633f);
	style.Colors[ImGuiCol_HeaderHovered] = ImVec4(0.7372549176216125f, 0.6941176652908325f, 0.886274516582489f, 0.5490196347236633f);
	style.Colors[ImGuiCol_HeaderActive] = ImVec4(0.8156862854957581f, 0.772549033164978f, 0.9647058844566345f, 0.5490196347236633f);
    //style.Colors[ImGuiCol_Separator] = ImVec4(0.695024311542511f, 0.677558958530426f, 0.7553647756576538f, 0.5490196347236633f);
	//style.Colors[ImGuiCol_SeparatorHovered] = ImVec4(0.7121792435646057f, 0.6934738159179688f, 0.7768239974975586f, 0.5490196347236633f);
	//style.Colors[ImGuiCol_SeparatorActive] = ImVec4(0.9194769859313965f, 0.8974193930625916f, 0.995708167552948f, 0.5490196347236633f);
    //style.Colors[ImGuiCol_Separator] = ImVec4(0.6196078658103943f, 0.5764706134796143f, 0.7686274647712708f, 0.5490196347236633f);
    style.Colors[ImGuiCol_Separator] = ImVec4(0.0f, 0.0f, 0.0f, 1.0f);	
    style.Colors[ImGuiCol_SeparatorHovered] = ImVec4(0.6196078658103943f, 0.5764706134796143f, 0.7686274647712708f, 0.5490196347236633f);
	style.Colors[ImGuiCol_SeparatorActive] = ImVec4(0.8156862854957581f, 0.772549033164978f, 0.9647058844566345f, 0.5490196347236633f);
	style.Colors[ImGuiCol_ResizeGrip] = ImVec4(0.6196078658103943f, 0.5764706134796143f, 0.7686274647712708f, 0.5490196347236633f);
	style.Colors[ImGuiCol_ResizeGripHovered] = ImVec4(0.7372549176216125f, 0.6941176652908325f, 0.886274516582489f, 0.5490196347236633f);
	style.Colors[ImGuiCol_ResizeGripActive] = ImVec4(0.8156862854957581f, 0.772549033164978f, 0.9647058844566345f, 0.5490196347236633f);
	style.Colors[ImGuiCol_Tab] = ImVec4(0.6196078658103943f, 0.5764706134796143f, 0.7686274647712708f, 0.5490196347236633f);
	style.Colors[ImGuiCol_TabHovered] = ImVec4(0.7372549176216125f, 0.6941176652908325f, 0.886274516582489f, 0.6223175525665283f);
	style.Colors[ImGuiCol_TabActive] = ImVec4(0.8156862854957581f, 0.772549033164978f, 0.9647058844566345f, 0.6480686664581299f);

	style.Colors[ImGuiCol_TabUnfocused] = ImVec4(0.6196078658103943f, 0.5764706134796143f, 0.7686274647712708f, 0.5490196347236633f);
	style.Colors[ImGuiCol_TabUnfocusedActive] = ImVec4(0.8156862854957581f, 0.772549033164978f, 0.9647058844566345f, 0.6480686664581299f);

	style.Colors[ImGuiCol_PlotLines] = ImVec4(0.8240343332290649f, 0.8240261077880859f, 0.8240261077880859f, 1.0f);
	style.Colors[ImGuiCol_PlotLinesHovered] = ImVec4(0.9124851226806641f, 0.0720219686627388f, 0.9871244430541992f, 1.0f);
	style.Colors[ImGuiCol_PlotHistogram] = ImVec4(0.6196078658103943f, 0.5764706134796143f, 0.7686274647712708f, 0.8154506683349609f);
	style.Colors[ImGuiCol_PlotHistogramHovered] = ImVec4(0.8402199745178223f, 0.7939914464950562f, 1.0f, 0.7811158895492554f);
	style.Colors[ImGuiCol_TableHeaderBg] = ImVec4(0.1882352977991104f, 0.1882352977991104f, 0.2000000029802322f, 1.0f);
	style.Colors[ImGuiCol_TableBorderStrong] = ImVec4(0.4235294163227081f, 0.3803921639919281f, 0.572549045085907f, 0.5490196347236633f);
	style.Colors[ImGuiCol_TableBorderLight] = ImVec4(0.4235294163227081f, 0.3803921639919281f, 0.572549045085907f, 0.2918455004692078f);
	style.Colors[ImGuiCol_TableRowBg] = ImVec4(0.0f, 0.0f, 0.0f, 0.0f);
	style.Colors[ImGuiCol_TableRowBgAlt] = ImVec4(1.0f, 1.0f, 1.0f, 0.03433477878570557f);
	style.Colors[ImGuiCol_TextSelectedBg] = ImVec4(0.7372549176216125f, 0.6941176652908325f, 0.886274516582489f, 0.5490196347236633f);
	style.Colors[ImGuiCol_DragDropTarget] = ImVec4(1.0f, 1.0f, 0.0f, 0.8999999761581421f);
	style.Colors[ImGuiCol_NavHighlight] = ImVec4(0.0f, 0.0f, 0.0f, 1.0f);
	style.Colors[ImGuiCol_NavWindowingHighlight] = ImVec4(1.0f, 1.0f, 1.0f, 0.699999988079071f);
	style.Colors[ImGuiCol_NavWindowingDimBg] = ImVec4(0.800000011920929f, 0.800000011920929f, 0.800000011920929f, 0.2000000029802322f);
	style.Colors[ImGuiCol_ModalWindowDimBg] = ImVec4(0.800000011920929f, 0.800000011920929f, 0.800000011920929f, 0.3499999940395355f);

}


