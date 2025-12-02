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

#include <learnopengl/shader.h>

#include <texture_loader.hpp>
#include <object_loader.hpp>

#include <learnopengl/camera_quat.h>

#include <iostream>

#include <vector>


void create_framebuffer();
void create_info_framebuffer();
void rescale_framebuffer(int width, int height);
void rescale_info_framebuffer(int width, int height);
void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void processInput(GLFWwindow *window);
void mouse_callback(GLFWwindow* window, double xposIn, double yposIn);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void RenderMainDockspace();
void draw_guizmo_toolbar(const ImVec2 &contentMin, const ImVec2 &contentMax);
void draw_imoguizmo(const float *proj_m,ArcBallCamera &camera,const ImVec2 &WindowSize);
void SetupImGuiStyle();
// settings
const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;
unsigned int window_current_width = SCR_WIDTH;
unsigned int window_current_height = SCR_HEIGHT;
ImVec2 previousImGuiWindowSize = ImVec2(0, 0);
ImGuiWindowFlags windowFlags_global = ImGuiWindowFlags_None;

ImVec2 WindowSize;
ImVec2 ViewportPos;

GLFWcursor* arrowCursor;
GLFWcursor* dragCursor;
GLFWcursor* invisible_cursor;

double global_cursor_xpos = 0.0f;
double global_cursor_ypos = 0.0f;

bool scene_hovered = false;
bool isDragging = false;

bool selected = false;
bool selected_cam = false;

bool isUsingGuizmo = false;

ImGuiStyle global_style;

ImFont *font;
static const ImWchar icons_ranges[] = { 0x0030, 0x0038, 0 };
ImFont *icons;

unsigned int framebuffer;
unsigned int textureColorbuffer;
unsigned int rbo;

unsigned int m_info_framebuffer;
unsigned int m_picking_texture;
unsigned int m_picking_depth_tex;
ImVec2 previousPickingImGuiWindowSize = ImVec2(0, 0);

// timing
float deltaTime = 0.0f;
float lastFrame = 0.0f;

GLFWmonitor* monitor;
const GLFWvidmode* mode;

int screenWidth;
int screenHeight;

// camera
//Camera camera(SCR_WIDTH, SCR_HEIGHT, glm::vec3(0.0f, 0.0f, -2.0f),glm::vec3(0.0f, 0.0f, 0.0f) - glm::vec3(0.0f, 0.0f, -2.0f),glm::vec3(0.0f, 1.0f, 0.0f)); //up vector here is useless
ArcBallCamera camera(SCR_WIDTH, SCR_HEIGHT,glm::vec3(0.0f, 0.0f, 0.0f), 2.0f,0.0f,0.0f);
//float lastX = (float)SCR_WIDTH  / 2.0;
//float lastY = (float)SCR_HEIGHT / 2.0;
//bool firstMouse = true;
bool fly_mode = 0;

int main()
{
    // glfw: initialize and configure
    // ------------------------------
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    //glfwInitHint(GLFW_PLATFORM, GLFW_PLATFORM_X11);

#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    // glfw window creation
    // --------------------
    GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "LearnOpenGL", NULL, NULL);
    if (window == NULL)
    {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);

    //glfwSetScrollCallback(window, scroll_callback);

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
     
    glm::mat4 guizmoview_proj = glm::perspective(glm::radians(140.0f), 1.0f, 0.1f, 1000.0f);

    float guizmo_proj[16];
    memcpy(guizmo_proj, glm::value_ptr(guizmoview_proj), sizeof(float) * 16);

    // configure global opengl state
    // -----------------------------
    glfwSwapInterval(1); // Enable vsync
    glEnable(GL_DEPTH_TEST);


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
    //ImGuiFreeType::BuildFontAtlas(io.Fonts, cfg.FontBuilderFlags);
    io.Fonts->Build(); // or let backend do it

    ////// Setup Dear ImGui style
    //////ImGui::StyleColorsDark();

    //////ImGuiStyle& style = ImGui::GetStyle();
    //////if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable){
    //////    style.WindowRounding = 0.0f;
    //////    style.Colors[ImGuiCol_WindowBg].w = 1.0f;
    //////}

    SetupImGuiStyle();


    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 460");

    // build and compile our shader program
    // ------------------------------------
    Shader ourShader("../shaders/gui_test/4.6.shader.vs", "../shaders/gui_test/4.6.shader.fs"); // you can name your shader files however you like
    Shader screenShader("../shaders/gui_test/5.1.framebuffers_screen.vs", "../shaders/gui_test/5.1.framebuffers_screen.fs");

    // set up vertex data (and buffer(s)) and configure vertex attributes
    // ------------------------------------------------------------------
    float vertices[] = {
        // positions         // colors
         0.5f, -0.5f, 0.0f,  1.0f, 0.0f, 0.0f,  // bottom right
        -0.5f, -0.5f, 0.0f,  0.0f, 1.0f, 0.0f,  // bottom left
         0.0f,  0.5f, 0.0f,  0.0f, 0.0f, 1.0f   // top 
    };

    unsigned int VBO, VAO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    // bind the Vertex Array Object first, then bind and set vertex buffer(s), and then configure vertex attributes(s).
    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    // position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    // color attribute
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    // You can unbind the VAO afterwards so other VAO calls won't accidentally modify this VAO, but this rarely happens. Modifying other
    // VAOs requires a call to glBindVertexArray anyways so we generally don't unbind VAOs (nor VBOs) when it's not directly necessary.
     glBindVertexArray(0);

    float quadVertices[] = { // vertex attributes for a quad that fills the entire screen in Normalized Device Coordinates.
    // positions   // texCoords
    -1.0f,  1.0f,  0.0f, 1.0f,
    -1.0f, -1.0f,  0.0f, 0.0f,
     1.0f, -1.0f,  1.0f, 0.0f,

    -1.0f,  1.0f,  0.0f, 1.0f,
     1.0f, -1.0f,  1.0f, 0.0f,
     1.0f,  1.0f,  1.0f, 1.0f
    };
    // screen quad VAO
    unsigned int quadVAO, quadVBO;
    glGenVertexArrays(1, &quadVAO);
    glGenBuffers(1, &quadVBO);
    glBindVertexArray(quadVAO);
    glBindBuffer(GL_ARRAY_BUFFER, quadVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(quadVertices), &quadVertices, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)(2 * sizeof(float)));

    screenShader.use();
    screenShader.setInt("screenTexture", 0);

    GLuint dummyVAO;
    glGenVertexArrays(1, &dummyVAO);

    Shader pickingShader("../shaders/gui_test/pick.vs", "../shaders/gui_test/pick.fs");

    Shader gridShader("../shaders/gui_test/grid.vs", "../shaders/gui_test/grid2.fs");
    //Shader lineShader("../shaders/gui_test/line.vs", "../shaders/gui_test/line.fs");

    Shader screen_Shader("../shaders/gui_test/screen.vs", "../shaders/gui_test/screen.fs");

    Shader cubeShader("../shaders/gui_test/4.6.shader5.vs", "../shaders/gui_test/4.6.shader5.fs");
    Shader cubeInfoShader("../shaders/gui_test/pick.vs", "../shaders/gui_test/pick.fs");
    Shader cubeOutlineShader("../shaders/gui_test/outline.vs", "../shaders/gui_test/outline.fs");


    cube cube(glm::vec3(0.0f,0.0f,-1.0f),cubeShader,DRAW_TEXTURED);
    cube.SetInfoShader(&cubeInfoShader);
    cube.SetOutlineShader(&cubeOutlineShader);
    cube.SetDrawInfoMode(DRAW_INFO);
    cube.SetDrawOutlineMode(DRAW_OUTLINE);
    cube.SetTransform();
    cube.SetInfoTransform();
    cube.SetOutlineTransform();
    // load textures
    // -------------
    unsigned int cubeTexture  = loadTexture("../resources/textures/container.jpg");
    cube.SetTexture(cubeTexture);

    glm::mat4 projection = glm::perspective(glm::radians(45.0f), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);

    cube.SetProjection(projection);

    create_framebuffer();
    create_info_framebuffer();

    int framebufferWidth = window_current_width;
    int framebufferHeight = window_current_height;
    
    camera.setConstrainAngle(false);

    VirtualCamera plumb_bob_cam(1000,600,1.0f,screen_Shader,glm::vec3(0.0f,0.5f,1.0f),180.0f,0.0f,0.0f);
    plumb_bob_cam.setDistortioncoeffs(glm::vec3(-0.169694115533936f, 0.04075467706851231f,0.0f),glm::vec2(-0.004826682535351785f, 0.0008093718878340755f));
    plumb_bob_cam.CalculateIntrisicMatrices(45.0f);
    plumb_bob_cam.create_camera_framebuffer();
    
    std::string default_shader_path = "../shaders/gui_test/";
    Shader cam_shader((default_shader_path+"default.vs").c_str(), (default_shader_path+"default.fs").c_str());
    
    Model cam_mesh("../resources/models/camera/camera.obj");

    object_model cam_model(plumb_bob_cam.GetCameraPosition(),glm::radians(plumb_bob_cam.GetCameraRotation()),cam_shader,cam_mesh,DRAW_MODEL);

    cam_model.SetInfoShader(&cubeInfoShader);
    cam_model.SetOutlineShader(&cubeOutlineShader);
    cam_model.SetDrawInfoMode(DRAW_MODEL_INFO);
    cam_model.SetDrawOutlineMode(DRAW_MODEL_OUTLINE);
    cam_model.SetTransform();
    cam_model.SetInfoTransform();
    cam_model.SetOutlineTransform();

    cam_model.SetProjection(projection);

    lastFrame = static_cast<float>(glfwGetTime());

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
        float currentFrame = static_cast<float>(glfwGetTime());
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;
        
        // input
        // -----
        glfwPollEvents();
        processInput(window);
        //camera.OnRender(deltaTime);


        glBindFramebuffer(GL_FRAMEBUFFER, 0);

        // and tell our program that we'll create a ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame(); 
        ImGui::NewFrame();
        RenderMainDockspace(); 

        windowFlags_global = ImGuiWindowFlags_None;

        if(fly_mode) windowFlags_global |= ImGuiWindowFlags_NoInputs;

        //ImGui::SetWindowBorderSize(0.0f);
        //ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
        //ImGuiStyle& style = ImGui::GetStyle();
        //style.WindowBorderSize = 0.0f;  // Set window border size to zero

        ImGuiStyle& style = ImGui::GetStyle();
        style.WindowPadding = ImVec2(0.0f, 0.0f);
        style.WindowBorderSize = 0.0f;

        ImGui::Begin("Viewport",nullptr,ImGuiWindowFlags_None | windowFlags_global);
        
        //bool scene_focused = ImGui::IsWindowFocused();
        scene_hovered = ImGui::IsWindowHovered() && glfwGetWindowAttrib(window, GLFW_HOVERED);

        if (scene_hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Middle))
        {
            ImGui::SetNextWindowFocus();
        }

        // we access the ImGui window size
        const ImVec2 currentImGuiWindowSize = ImGui::GetContentRegionAvail();
        const ImVec2 viewportMin = ImGui::GetCursorScreenPos();
        const ImVec2 viewportMax = ImVec2(viewportMin.x + currentImGuiWindowSize.x,viewportMin.y + currentImGuiWindowSize.y);

        WindowSize = currentImGuiWindowSize;
        ViewportPos = viewportMin;
        // Here we can render into the ImGui window
        // ImGui Buttons, Drop Downs, etc. and later our framebuffer

        // Check if the ImGui window size has changed
        if (currentImGuiWindowSize.x != previousImGuiWindowSize.x || currentImGuiWindowSize.y != previousImGuiWindowSize.y) {
            // Update framebuffer size
            framebufferWidth = static_cast<int>(currentImGuiWindowSize.x);
            framebufferHeight = static_cast<int>(currentImGuiWindowSize.y);
            // we rescale the framebuffer to the actual window size here and reset the glViewport 
            rescale_framebuffer(framebufferWidth, framebufferHeight);
            rescale_info_framebuffer(framebufferWidth, framebufferHeight);
            camera.ScreenResize(framebufferWidth,framebufferHeight);
            projection = glm::perspective(glm::radians(45.0f), (float)framebufferWidth / (float)framebufferHeight, 0.1f, 100.0f);
            previousImGuiWindowSize = currentImGuiWindowSize;
        }

        // and here we can add our created texture as image to ImGui
        // unfortunately we need to use the cast to void* or I didn't find another way tbh
        //ImGui::Image((ImTextureID)textureColorbuffer, ImVec2(framebufferWidth, framebufferHeight));

        ImGui::SetCursorPos(viewportMin);
        // Draw the texture (OpenGL)
        ImGui::GetWindowDrawList()->AddImage(
            (ImTextureID)(intptr_t)textureColorbuffer,
            viewportMin,
            ImVec2(viewportMin.x + currentImGuiWindowSize.x, viewportMin.y + currentImGuiWindowSize.y),
            ImVec2(0, 1), // UVs
            ImVec2(1, 0)  // Flip vertically for OpenGL
        );

        // Use your camera/view/proj matrices
        draw_imoguizmo(guizmo_proj,camera,WindowSize);

        // Tell ImGuizmo where the viewport is

        draw_guizmo_toolbar(viewportMin, viewportMax);

        ImGuizmo::BeginFrame();
        ImGuizmo::SetDrawlist();
        ImGuizmo::SetOrthographic(false);
        ImGuizmo::SetRect(ViewportPos.x, ViewportPos.y, WindowSize.x, WindowSize.y);

        float viewMatrix[16];
        float projMatrix[16];
        float modelMatrix[16];
        float deltaMatrix[16];

        float translation[3],rotation[3],scale[3];

        memcpy(viewMatrix, glm::value_ptr(camera.GetViewMatrix()), sizeof(float) * 16);
        memcpy(projMatrix, glm::value_ptr(projection), sizeof(float) * 16);

        //ImGuizmo::Manipulate(viewMatrix, projMatrix,ImGuizmo::TRANSLATE, ImGuizmo::LOCAL, modelMatrix,deltaMatrix);
        if(selected){
            memcpy(modelMatrix, glm::value_ptr(cube.Get_model_matrix()), sizeof(float) * 16);
            ImGuizmo::Manipulate(viewMatrix, projMatrix,ImGuizmo::ROTATE, ImGuizmo::LOCAL, modelMatrix,deltaMatrix);
        }
        else if(selected_cam){
            memcpy(modelMatrix, glm::value_ptr(plumb_bob_cam.camera_matrix), sizeof(float) * 16);
            ImGuizmo::Manipulate(viewMatrix, projMatrix,ImGuizmo::ROTATE, ImGuizmo::LOCAL, modelMatrix,deltaMatrix);
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
            glm::mat4 delta_rot;
            memcpy(glm::value_ptr(delta_rot),deltaMatrix, sizeof(float) * 16);
            if(selected){
            cube.Rotate(delta_rot);
        
            //extract rpy
            //memcpy(deltaMatrix, glm::value_ptr(cube), sizeof(float) * 16);
            //ImGuizmo::DecomposeMatrixToComponents(deltaMatrix,translation,rotation,scale);

            //cube.Change_rotation(cube.Get_rotation());
            //cube.Scale(glm::vec3(scale[0],scale[1],scale[2]));
            //cube.Change_position(glm::vec3(modelMatrix[12],modelMatrix[13],modelMatrix[14]));
            cube.SetTransform();
            cube.SetInfoTransform();
            cube.SetOutlineTransform();
            } else if(selected_cam){
                plumb_bob_cam.AddCameraTransform(delta_rot);
                cam_model.Rotate(delta_rot);
                cam_model.SetTransform();
                cam_model.SetInfoTransform();
                cam_model.SetOutlineTransform();
            }
        }else isUsingGuizmo = false;
        
        //}
        
        ImGui::End();

        style.WindowPadding = ImVec2(8.0f, 8.0f);
        style.WindowBorderSize = 2.0f;
        ImGui::Begin("View",nullptr,ImGuiWindowFlags_None | ImGuiWindowFlags_MenuBar | windowFlags_global);
            ImGui::BeginMenuBar();
               
            bool show_lidar_settings;
            if (ImGui::BeginMenu("View"))
            {
                if (ImGui::Checkbox("Show Lidar Settings", &show_lidar_settings)) { /* Do stuff */ }
                if (ImGui::MenuItem("Save", "Ctrl+S"))   { /* Do stuff */ }
                if (ImGui::MenuItem("Close", "Ctrl+W"))  { /*my_tool_active = false; */}
                ImGui::EndMenu();
            }
            ImGui::EndMenuBar();
            
        ImGui::Text("Viewport %.0fx%.0f", currentImGuiWindowSize.x, currentImGuiWindowSize.y);
        ImGui::End();
        //ImGuiWindowFlags viewport_flags =
        //ImGuiWindowFlags_NoCollapse |  // Optional: can't collapse
        //ImGuiWindowFlags_NoResize;     // Optional: docked resizing only

        //static ImGuiWindowClass viewportWindowClass;
        //viewportWindowClass.DockNodeFlagsOverrideSet = ImGuiDockNodeFlags_NoUndocking;

        //ImGui::SetNextWindowClass(&viewportWindowClass);

        ImGui::Begin("Camera Settings",nullptr,ImGuiWindowFlags_None | windowFlags_global);

        // Draw your viewport content (OpenGL/Vulkan texture, etc.)
        ImVec2 size = ImGui::GetContentRegionAvail();
        ImGui::Text("My win2 %.0fx%.0f", size.x, size.y);

        ImGui::End();
        
        style.WindowPadding = ImVec2(0.0f, 0.0f);
        style.WindowBorderSize = 0.0f;
        ImGui::Begin("Camera Viewport");

                // we access the ImGui window size
            const ImVec2 currentImGuiWindowSize_cam = ImGui::GetContentRegionAvail();
            const ImVec2 viewportMin_cam = ImGui::GetCursorScreenPos();
            const ImVec2 viewportMax_cam = ImVec2(viewportMin_cam.x + currentImGuiWindowSize_cam.x,viewportMin_cam.y + currentImGuiWindowSize_cam.y);

            ImVec2 viewportMin_cam_view;
            ImVec2 viewportMax_cam_view;

            ImGui::SetCursorPos(viewportMin_cam);

            float image_size_y = (plumb_bob_cam.m_windowHeight/plumb_bob_cam.m_windowWidth)*currentImGuiWindowSize_cam.x;
            float image_size_x;
            if(image_size_y > currentImGuiWindowSize_cam.y){
                image_size_y = currentImGuiWindowSize_cam.y;
                image_size_x = (plumb_bob_cam.m_windowWidth/plumb_bob_cam.m_windowHeight)*currentImGuiWindowSize_cam.y;

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
                (ImTextureID)(intptr_t)plumb_bob_cam.CameratextureFinalColorbuffer,
                viewportMin_cam_view,//viewportMin_cam,
                viewportMax_cam_view,//viewportMax_cam,
                ImVec2(0, 1), // UVs
                ImVec2(1, 0)  // Flip vertically for OpenGL
            );


        ImGui::End();
        style.WindowPadding = ImVec2(8.0f, 8.0f);
        style.WindowBorderSize = 2.0f;
        ImGui::Render();
        
        //rendering picking buffer

        glViewport(0, 0, framebufferWidth, framebufferHeight);
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, m_info_framebuffer);
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LESS);
        glEnable(GL_CULL_FACE);
        glCullFace(GL_BACK);
        glFrontFace(GL_CCW);

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glm::mat4 view_matrix = camera.GetViewMatrix();

        cube.SetInfoProjection(projection); 
        cube.SetInfoView(view_matrix);
        cube.SetInfoTransform();
        cube.DrawInfo(1);

        //cam_model.SetInfoProjection(projection);
        //cam_model.SetInfoView(view_matrix);
        cam_model.SetInfoTransform();
        cam_model.DrawInfo(2);

        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);

        glBindFramebuffer(GL_READ_FRAMEBUFFER, m_info_framebuffer);
        glReadBuffer(GL_COLOR_ATTACHMENT0);
        
        
        if(ImGui::IsMouseClicked(ImGuiMouseButton_Left,true) && scene_hovered && !isUsingGuizmo){
            unsigned int Pixel[3];
            ImVec2 mouse_position  = ImGui::GetMousePos();
            mouse_position.x -= ViewportPos.x;
            mouse_position.y -= ViewportPos.y;
            mouse_position.y = (WindowSize.y - 1) - mouse_position.y;
            glReadPixels(static_cast<unsigned int>(mouse_position.x),static_cast<unsigned int>(mouse_position.y),1,1,GL_RGB_INTEGER,GL_UNSIGNED_INT,&Pixel);
            std::cout << "("<<Pixel[0]<<","<<Pixel[1]<<","<<Pixel[2]<<")" << std::endl;

            if(Pixel[0] == 1) {
                selected = true;
                selected_cam = false;
            }
            else if(Pixel[0] == 2){
                selected = false;
                selected_cam = true;
            }
            else selected = selected_cam = false;
        }
        glReadBuffer(GL_NONE);
        glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);

        // ... rendering our triangle as before
        glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);
        glViewport(0, 0, framebufferWidth, framebufferHeight);
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LESS);
        glEnable(GL_CULL_FACE);
        glCullFace(GL_BACK);
        glFrontFace(GL_CCW);
        //glEnable(GL_DEPTH_TEST); // enable depth testing (is disabled for rendering screen-space quad)
        // render
        // ------
        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
        //glEnable(GL_STENCIL_TEST);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

        //stencil
        glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
        glStencilFunc(GL_ALWAYS, 0x01, 0xFF); 
        glStencilMask(0xFF); 

        cube.SetProjection(projection); 
        cube.SetView(view_matrix);
        cube.SetTransform();
        glDisable(GL_STENCIL_TEST);
        if(selected) glEnable(GL_STENCIL_TEST);
        cube.Draw();

        cam_model.SetProjection(projection); 
        cam_model.SetView(view_matrix);
        cam_model.SetTransform();
        glDisable(GL_STENCIL_TEST);
        if(selected_cam) glEnable(GL_STENCIL_TEST);
        cam_model.Draw();

        glDisable(GL_STENCIL_TEST); 

        if(selected){
        glEnable(GL_STENCIL_TEST);
        glStencilFunc(GL_NOTEQUAL, 0x01, 0xFF);
        glStencilMask(0x00); 
        //glDisable(GL_DEPTH_TEST);
        float distance = glm::length(cube.Get_position() - camera.GetCameraPosition());
        cube.SetOutlineProjection(projection); 
        cube.SetOutlineView(view_matrix);
        cube.SetOutlineTransform();
        cube.localShader_outline->setFloat("distant",distance);
        cube.DrawOutline();
        glStencilMask(0xFF);
        glStencilFunc(GL_ALWAYS, 0, 0xFF);
        glDisable(GL_STENCIL_TEST);
        }else if(selected_cam){
            glEnable(GL_STENCIL_TEST);
            glStencilFunc(GL_NOTEQUAL, 0x01, 0xFF);
            glStencilMask(0x00); 
            //glDisable(GL_DEPTH_TEST);
            float distance = glm::length(cam_model.Get_position() - camera.GetCameraPosition());
            cam_model.SetOutlineProjection(projection); 
            cam_model.SetOutlineView(view_matrix);
            cam_model.localShader_outline->setFloat("distant",distance*2.0f);
            cam_model.SetOutlineTransform();
            cam_model.DrawOutline();
            glStencilMask(0xFF);
            glStencilFunc(GL_ALWAYS, 0, 0xFF);
            glDisable(GL_STENCIL_TEST);
        }

        glEnable(GL_DEPTH_TEST);
        // render the triangle
        //ourShader.use();
        //ourShader.setMat4("projection",projection);
        //ourShader.setMat4("view",camera.GetViewMatrix());
        //ourShader.setMat4("model",glm::mat4(1.0));
        //glBindVertexArray(VAO);
        //glDrawArrays(GL_TRIANGLES, 0, 3);
        //glBindVertexArray(0);

        //glStencilMask(0xFF);  

        glDisable(GL_CULL_FACE);   
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);

        //glDisable(GL_DEPTH_TEST);
        //lineShader.use();
        //lineShader.setMat4("projection",projection);
        //lineShader.setMat4("view",camera.GetViewMatrix());
        //lineShader.setVec3("cameraPos",camera.GetCameraPosition());
        // Set line thickness
        //glLineWidth(2.0f); // Set thickness to 5.0
        //glBindVertexArray(dummyVAO);
        //glDrawArrays(GL_LINES, 0, 4);
        //glBindVertexArray(0);

        gridShader.use();
        gridShader.setMat4("projection",projection);
        gridShader.setMat4("view",camera.GetViewMatrix());
        gridShader.setVec3("cameraPos",camera.GetCameraPosition());
        glBindVertexArray(dummyVAO);
        glDrawArrays(GL_TRIANGLES,0,6);
        
        glEnable(GL_CULL_FACE);

        glBindVertexArray(0);


        plumb_bob_cam.StartRender();

        glEnable(GL_CULL_FACE);
        glEnable(GL_DEPTH_TEST);
        glDisable(GL_BLEND);


        cube.SetProjection(plumb_bob_cam.projection_matrix); 
        cube.SetView(plumb_bob_cam.GetViewMatrix());
        cube.SetTransform();
        cube.Draw();


        plumb_bob_cam.FinishRender();
        //plumb_bob_cam.setDistortioncoeffs(glm::vec3(0.0f),glm::vec2(0.0f));
        plumb_bob_cam.GenarateImage();

        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        glViewport(0, 0, window_current_width, window_current_height);
        glDisable(GL_DEPTH_TEST); // disable depth test so screen-space quad isn't discarded due to depth test.
        glDisable(GL_CULL_FACE);
        glEnable(GL_BLEND);
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        //screenShader.use();
        //glBindVertexArray(quadVAO);
        //glBindTexture(GL_TEXTURE_2D, textureColorbuffer);	// use the color attachment texture as the texture of the quad plane
        //glDrawArrays(GL_TRIANGLES, 0, 6);

        
        //ImGuiWindow* focused_window = ImGui::GetFocusWindow();
        //static bool scene_hovered;
        
        //////if (scene_hovered)
        //////{
        //////    std::cout << "viewport hovered" << std::endl;
        //////}
        //////else std::cout << "viewport not hovered" << std::endl;;

        // and we have to pass the render data further
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());	
        if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable)
        {
    	    GLFWwindow* backup_current_context = glfwGetCurrentContext();
    	    ImGui::UpdatePlatformWindows();
    	    ImGui::RenderPlatformWindowsDefault();
    	    glfwMakeContextCurrent(backup_current_context);
        }



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
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);

    // glfw: terminate, clearing all previously allocated GLFW resources.
    // ------------------------------------------------------------------
    glfwTerminate();
    return 0;
}

// process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
// ---------------------------------------------------------------------------------------------------------
// process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
// ---------------------------------------------------------------------------------------------------------

bool capture_mouse = false;
bool keyPressed = false;
bool scroll_callback_installed = false;
bool shift_pressed = false;

void processInput(GLFWwindow *window)
{

    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        camera.ProcessKeyboard(FORWARD, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        camera.ProcessKeyboard(BACKWARD, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        camera.ProcessKeyboard(LEFT, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        camera.ProcessKeyboard(RIGHT, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS)
        camera.ProcessKeyboard(CAMERA_YAW_NEG, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS)
        camera.ProcessKeyboard(CAMERA_YAW_POS, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_R) == GLFW_PRESS)
        camera.ProcessKeyboard(CAMERA_PITCH_POS, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_F) == GLFW_PRESS)
        camera.ProcessKeyboard(CAMERA_PITCH_NEG, deltaTime);

    if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS && !isDragging)
        shift_pressed = true;
    else if(glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_RELEASE && !isDragging)
        shift_pressed = false; 
        
    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS && !keyPressed && scene_hovered){

        keyPressed = true;
        camera.firstMouse = true;

        glfwGetCursorPos(window, &global_cursor_xpos, &global_cursor_ypos);

        glfwSetCursorPosCallback(window, mouse_callback);
        glfwSetCursorPosCallback(window, mouse_callback);
        glfwSetMouseButtonCallback(window, nullptr);

        ImGui::GetIO().MouseDown[0] = 0;
        ImGui::GetIO().MouseDown[1] = 0;

        //change cursor icon
        isDragging = true;
        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
        //glfwSetCursor(window, invisible_cursor);
        glfwSetCursor(window, dragCursor);
    } 
    //or just consider processing only at each frame 
    //else if (glfwGetKey(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS && keyPressed && scene_hovered){
    //    //camera.ProcessMouseMovement();
    //}
    else if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_RELEASE || !scene_hovered){

        keyPressed=false;
        glfwSetCursorPosCallback(window, ImGui_ImplGlfw_CursorPosCallback);
        glfwSetMouseButtonCallback(window, ImGui_ImplGlfw_MouseButtonCallback);

        //restore cursor icon
        isDragging = false;
        glfwSetCursor(window, arrowCursor);
        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
    }
    

    if(scene_hovered && !scroll_callback_installed){
        glfwSetScrollCallback(window, scroll_callback);
        scroll_callback_installed = true;
    }
    else if(!scene_hovered && scroll_callback_installed){
        glfwSetScrollCallback(window, ImGui_ImplGlfw_ScrollCallback);
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



// glfw: whenever the window size changed (by OS or user resize) this callback function executes
// ---------------------------------------------------------------------------------------------
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    // make sure the viewport matches the new window dimensions; note that width and 
    // height will be significantly larger than specified on retina displays.
    window_current_width = width;
    window_current_height = height;
    //glViewport(0, 0, width, height);
}


void update_imgui_mouse_input(GLFWwindow* window)
{
    ImGuiIO& io = ImGui::GetIO();

    double mouseX, mouseY;
    glfwGetCursorPos(window, &mouseX, &mouseY);
    io.MousePos = ImVec2((float)mouseX, (float)mouseY); // Restore ImGui's mouse position

    io.MouseDown[0] = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
    io.MouseDown[1] = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
}


// glfw: whenever the mouse moves, this callback is called
// -------------------------------------------------------
void mouse_callback(GLFWwindow* window, double xposIn, double yposIn)
{
    float xpos = static_cast<float>(xposIn);
    float ypos = static_cast<float>(yposIn);


    //cout << "("<<xpos<<","<<ypos<<")"<<"("<<ViewportPos.x<<","<<ViewportPos.y<<")" << std::endl;
    
    //camera.ProcessMouseMovement(xoffset, yoffset);
    camera.ProcessMouseMovement(xpos, ypos, shift_pressed);
    //if out of bounds teleport && camera set initial position

    //if(xpos > ){
    //xpos = fmod(xpos - ViewportPos.x, WindowSize.x) + ViewportPos.x;
    //ypos = fmod(ypos - ViewportPos.y, WindowSize.y) + ViewportPos.y;
    //glfwSetCursorPos(window, xpos, ypos);
    //camera.firstMouse;
    //}

    if(xpos < ViewportPos.x){
        glfwSetCursorPos(window, xpos + WindowSize.x, ypos);
        xpos += WindowSize.x;
        camera.firstMouse = true;
    } else if(xpos > (ViewportPos.x+WindowSize.x)){
        glfwSetCursorPos(window, xpos - WindowSize.x, ypos);
        xpos -= WindowSize.x;
        camera.firstMouse = true;
    }
    if(ypos < ViewportPos.y){
        glfwSetCursorPos(window, xpos, ypos + WindowSize.y);
        camera.firstMouse = true;
    } else if(ypos > (ViewportPos.y+WindowSize.y)){
        glfwSetCursorPos(window, xpos, ypos - WindowSize.y);
        camera.firstMouse = true;
    }
    
    global_cursor_xpos = static_cast<double>(xpos);
    global_cursor_ypos = static_cast<double>(ypos);

}

// glfw: whenever the mouse scroll wheel scrolls, this callback is called
// ----------------------------------------------------------------------
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    camera.ProcessMouseScroll(static_cast<float>(yoffset));
}

void create_framebuffer()
{   

    // framebuffer configuration
    // -------------------------
    glGenFramebuffers(1, &framebuffer);
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);
    // create a color attachment texture
    glGenTextures(1, &textureColorbuffer);
    glBindTexture(GL_TEXTURE_2D, textureColorbuffer);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, SCR_WIDTH, SCR_HEIGHT, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, textureColorbuffer, 0);
    // create a renderbuffer object for depth and stencil attachment (we won't be sampling these)
    glGenRenderbuffers(1, &rbo);
    glBindRenderbuffer(GL_RENDERBUFFER, rbo);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, SCR_WIDTH, SCR_HEIGHT); // use a single renderbuffer object for both a depth AND stencil buffer.
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, rbo); // now actually attach it
    // now that we actually created the framebuffer and added all attachments we want to check if it is actually complete now
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        std::cout << "ERROR::FRAMEBUFFER:: Framebuffer is not complete!" << std::endl;
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    previousImGuiWindowSize = ImVec2(SCR_WIDTH, SCR_HEIGHT);
}

// and we rescale the buffer, so we're able to resize the window
void rescale_framebuffer(int width, int height)
{
	glBindTexture(GL_TEXTURE_2D, textureColorbuffer);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, textureColorbuffer, 0);

	glBindRenderbuffer(GL_RENDERBUFFER, rbo);
	glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, width, height);
	glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, rbo);
}

void create_info_framebuffer()
{   

    // framebuffer configuration
    // -------------------------
    glGenFramebuffers(1, &m_info_framebuffer);
    glBindFramebuffer(GL_FRAMEBUFFER, m_info_framebuffer);
    // create a color attachment texture
    glGenTextures(1, &m_picking_texture);
    glBindTexture(GL_TEXTURE_2D, m_picking_texture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB32UI, SCR_WIDTH, SCR_HEIGHT, 0, GL_RGB_INTEGER, GL_UNSIGNED_INT, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_picking_texture, 0);
    // create a renderbuffer object for depth and stencil attachment (we won't be sampling these)
    glGenTextures(1, &m_picking_depth_tex);
    glBindTexture(GL_TEXTURE_2D, m_picking_depth_tex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, SCR_WIDTH, SCR_HEIGHT, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
    glFramebufferTexture2D(GL_FRAMEBUFFER,GL_DEPTH_ATTACHMENT,GL_TEXTURE_2D,m_picking_depth_tex,0);
    // now that we actually created the framebuffer and added all attachments we want to check if it is actually complete now
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        std::cout << "ERROR::FRAMEBUFFER:: Framebuffer is not complete!" << std::endl;
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    previousPickingImGuiWindowSize = ImVec2(SCR_WIDTH, SCR_HEIGHT);
}


void rescale_info_framebuffer(int width, int height)
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

inline void draw_main_popup(ImGuiIO& io){


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
        bool show_lidar_settings;
        if (ImGui::BeginMenu("View"))
        {
            if (ImGui::Checkbox("Show Lidar Settings", &show_lidar_settings)) { /* Do stuff */ }
            if (ImGui::MenuItem("Save", "Ctrl+S"))   { /* Do stuff */ }
            if (ImGui::MenuItem("Close", "Ctrl+W"))  { /*my_tool_active = false; */}
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

        static ImGuiID dock_id_left;
        static ImGuiID dock_id_right;
        static ImGuiID dock_id_center;
        static ImGuiID dock_id_down;

        ImGui::DockBuilderSplitNode(dockspace_id, ImGuiDir_Down, 0.3f, &dock_id_down, &dock_id_center);
        ImGui::DockBuilderSplitNode(dock_id_center, ImGuiDir_Left, 0.2f, &dock_id_left, &dock_id_center);
        ImGui::DockBuilderSplitNode(dock_id_center, ImGuiDir_Right, 0.2f, &dock_id_right, &dock_id_center);

        ImGui::DockBuilderDockWindow("Camera Viewport", dock_id_down);
        ImGui::DockBuilderDockWindow("Camera Settings", dock_id_left);
        ImGui::DockBuilderDockWindow("View", dock_id_right);
        ImGui::DockBuilderDockWindow("Viewport", dock_id_center); //Locked in center

        ImGui::DockBuilderFinish(dockspace_id);
    }

    ImGui::End();

    style.Colors[ImGuiCol_WindowBg] = ImVec4(0.1882352977991104f, 0.1882352977991104f, 0.1882352977991104f, 1.0f);
}

void draw_guizmo_toolbar(const ImVec2 &contentMin, const ImVec2 &contentMax){
    ImGui::PushFont(icons);
    ImVec2 toolbarPos = ImVec2(contentMin.x + 10, contentMin.y + 10); // 10 px padding
    ImGui::SetCursorScreenPos(toolbarPos);

    // Begin a transparent child window for the button bar
    ImGuiWindowFlags toolbarFlagsWindow =
        ImGuiWindowFlags_NoDecoration |
        ImGuiWindowFlags_NoSavedSettings |
        ImGuiWindowFlags_NoFocusOnAppearing |
        ImGuiWindowFlags_NoNav |
        windowFlags_global;

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

    int button_size = screenHeight*0.03;

    ImGui::BeginChild("GizmoToolbar", ImVec2(3*button_size +4*2, button_size), toolbarFlagsChild, toolbarFlagsWindow);

    

    static ImGuizmo::OPERATION currentOp = ImGuizmo::TRANSLATE;

    ImGuiStyle& style = ImGui::GetStyle();
    style.Colors[ImGuiCol_Button] = ImVec4(0.85f, 0.85f, 0.85f, 1.0f); // Default button color
    style.Colors[ImGuiCol_ButtonHovered] = ImVec4(0.75f, 0.75f, 0.75f, 1.0f); // Hover color
    style.Colors[ImGuiCol_ButtonActive] = ImVec4(1.0f, 0.0f, 0.0f, 1.0f); // Active color

    if (ImGui::Button("0", ImVec2(button_size, button_size))) currentOp = ImGuizmo::TRANSLATE;
    ImGui::SameLine();
    if (ImGui::Button("1", ImVec2(button_size, button_size))) currentOp = ImGuizmo::ROTATE;
    ImGui::SameLine();
    if (ImGui::Button("2", ImVec2(button_size, button_size))) currentOp = ImGuizmo::SCALE;


    ImGui::EndChild();

    toolbarPos = ImVec2(contentMin.x + 20 + 4*button_size, contentMin.y + 10); // 10 px padding
    ImGui::SetCursorScreenPos(toolbarPos);

    ImGui::BeginChild("world_transform", ImVec2(button_size, button_size), toolbarFlagsChild, toolbarFlagsWindow);

    if (ImGui::Button("6", ImVec2(button_size, button_size))) currentOp = ImGuizmo::TRANSLATE;
    ImGui::EndChild();

    toolbarPos = ImVec2(contentMin.x + 20 + 7*button_size, contentMin.y + 10); // 10 px padding
    ImGui::SetCursorScreenPos(toolbarPos);

    ImGui::BeginChild("camera_mode", ImVec2(button_size, button_size), toolbarFlagsChild, toolbarFlagsWindow);

    if (ImGui::Button("4", ImVec2(button_size, button_size)));
    ImGui::EndChild();

    toolbarPos = ImVec2(contentMin.x + 20 + 8.5*button_size, contentMin.y + 10); // 10 px padding
    ImGui::SetCursorScreenPos(toolbarPos);

    ImGui::BeginChild("perspective_angle", ImVec2(2*button_size,button_size), toolbarFlagsChild, toolbarFlagsWindow);

    //ImGui::DragFloat("7 : ", &board_orientation.x,0.1f,-arcSize, arcSize, "X: %.1f");

    ImGui::EndChild();


    style.Colors[ImGuiCol_Button] = global_style.Colors[ImGuiCol_Button];
    style.Colors[ImGuiCol_ButtonHovered] = global_style.Colors[ImGuiCol_ButtonHovered];
    style.Colors[ImGuiCol_ButtonActive] = global_style.Colors[ImGuiCol_ButtonActive];

    ImGui::PopStyleVar(6);

    ImGui::PopFont();
}

void draw_imoguizmo(const float *proj_m,ArcBallCamera &camera,const ImVec2 &WindowSize){

    ImOGuizmo::config.axisLengthScale = 1.0f;
    const float square_size = 120.0f;
    // specify position and size of gizmo (and its window when using ImOGuizmo::BeginFrame())
    ImOGuizmo::SetRect(ViewportPos.x +WindowSize.x-square_size -5 /* x */, ViewportPos.y + 5 /* y */, square_size /* square size */);
    //ImOGuizmo::BeginFrame(); // to use you own window remove this call 
    // and wrap everything in between ImGui::Begin() and ImGui::End() instead

    float viewMatrix[16];
    memcpy(viewMatrix, glm::value_ptr(camera.GetViewMatrix()), sizeof(float) * 16);
    // optional: set distance to pivot (-> activates interaction)
    if(ImOGuizmo::DrawGizmo(viewMatrix, proj_m, camera.m_pivot_distance /* optional: default = 0.0f */))
    {   
    	// in case of user interaction viewMatrix gets updated
        glm::vec3 right_v(viewMatrix[0],viewMatrix[4],viewMatrix[8]);
        glm::vec3 up_v(viewMatrix[1],viewMatrix[5],viewMatrix[9]);
        glm::vec3 front_v(-viewMatrix[2],-viewMatrix[6],-viewMatrix[10]);
        camera.SetVectors(right_v,up_v,front_v);
    }

}

void render_cube(const unsigned int &framebuffer,const unsigned int &width,const unsigned int &height){
    
}

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

    global_style = style;

}


