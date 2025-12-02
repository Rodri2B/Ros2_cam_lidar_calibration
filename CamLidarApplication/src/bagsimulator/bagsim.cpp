#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#define GL_SILENCE_DEPRECATION
#define IMGUI_DEFINE_MATH_OPERATORS

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <lib_opengl/shader.h>
#include <lib_opengl/shader_compute.h>
#include <lib_opengl/camera_quat.h>

#include <iostream>
#include <stdexcept>

#include <binding_vector.hpp>
#include <vector>

#include <texture_loader.hpp>

#include <object_loader.hpp>

#include <lib_opengl/model_original.h>

#include <lidar_scan.hpp>

#include <unistd.h>

#include <ImGuizmo.h>

//#include <pcl/sample_consensus/sac_model_plane.h>

#include <boost/interprocess/ipc/message_queue.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
//#include <cmath>


#define DEBUG 1


void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void processInput(GLFWwindow *window);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);

void update_imgui_mouse_input(GLFWwindow* window);

unsigned int loadTexture(const char *path);

//inline void draw_play_popup(ImGuiIO& io);
inline void draw_main_popup(ImGuiIO& io, const glm::mat4& proj_matrix,const glm::mat4& view_matrix,Object& object);

// settings
const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;

// camera
Camera camera(SCR_WIDTH, SCR_HEIGHT, glm::vec3(-3.0f, 1.0f, 0.0f),glm::vec3(0.0f, 1.0f, 0.0f),0.0f,0.0f); //up vector here is useless
//float lastX = (float)SCR_WIDTH  / 2.0;
//float lastY = (float)SCR_HEIGHT / 2.0;
//bool firstMouse = true;

bool fly_mode = 0;

// timing
float deltaTime = 0.0f;
float lastFrame = 0.0f;

//constants

bool show_raw_lidar_cloud = 0;
bool show_lidar_settings = 0;
bool show_board_settings = 0; 
bool show_lidar_points_borders = 0;
float lidar_uncertainty = 0.0f; 

static glm::vec3 board_position = glm::vec3( 0.0f, 0.0f, 0.0f );
static glm::vec3 board_orientation = glm::vec3 (0.0f, 0.0f, 0.0f);
static glm::vec3 board_scale = glm::vec3 (1.0f, 1.0f, 1.0f);


glm::vec4 clear_color = glm::vec4(0.69f, 0.69f, 0.93f, 1.0f);


int main(){
    
    // glfw: initialize and configure
    // ------------------------------
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    // glfw window creation
    // --------------------
    GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "Cloth Self Collision Handling", NULL, NULL);
    if (window == NULL)
    {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);

    glfwSwapInterval(1); // Enable vsync

    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetCursorPosCallback(window, nullptr);
    glfwSetScrollCallback(window, scroll_callback);
    //glfwSetKeyCallback(window, key_callback);

    // tell GLFW to capture our mouse
    //glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    // glad: load all OpenGL function pointers
    // ---------------------------------------
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return -1;
    }

    // Setup ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    
    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    //ImGui::StyleColorsLight();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 460");


    stbi_set_flip_vertically_on_load(true); // tell stb_image.h to flip loaded texture's on the y-axis.

    // configure global opengl state
    // -----------------------------
    glEnable(GL_DEPTH_TEST);
    //glDepthFunc(GL_ALWAYS); // always pass the depth test (same effect as glDisable(GL_DEPTH_TEST))

    glDepthFunc(GL_LESS);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Set pixel storage mode to match your image data layout
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glFrontFace(GL_CCW);
    
    // setting points
    glEnable(GL_PROGRAM_POINT_SIZE);
    //glPointSize(100.0f);
    //initializing binding vector

    std::vector<unsigned int> global_uniform_binding_indexes;

    global_uniform_binding_indexes.reserve(100);

    for(unsigned int i = 99; i > 0 ; i--){
        global_uniform_binding_indexes.push_back(i);
    }
    global_uniform_binding_indexes.push_back(0);

    
    // build and compile shaders
    // -------------------------

    std::string default_shader_path = "../shaders/default/";
    Shader default_shader((default_shader_path+"default_sh.vs").c_str(), (default_shader_path+"default_sh.fs").c_str());

    Shader base_shader((default_shader_path+"base.vs").c_str(), (default_shader_path+"base.fs").c_str());

    Shader lidar_shader((default_shader_path+"lidar.vs").c_str(), (default_shader_path+"lidar.fs").c_str());
    

    std::string path = "../shaders/project2/";
    Shader plane_shader((path+"plane.vs").c_str(), (path+"plane.fs").c_str());

    
    Shader point_sh((default_shader_path+"point_sh.vs").c_str(), (default_shader_path+"point_sh.fs").c_str());

    // build and compile compute shaders
    // -------------------------
    string cpath = "../shaders/compute/";
    Shader_compute lscanCompute((cpath+"lscan.comp").c_str());

    plane Plane(glm::vec3(0.0,0.0,0.0), plane_shader,DRAW_TEXTURED);
    Plane.Change_scale(glm::vec3(10.0,1.0,10.0));
    Plane.ScaleTexture(glm::vec2(2.0,2.0));

    //Plane.SetTransform();
    //Plane.SetTextureTransform();

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

    plane Board(glm::vec3(0.0,1.0,0.0), plane_shader,DRAW_TEXTURED);
    Board.Change_scale(glm::vec3(board_width,1.0,board_height));
    //Board.Rotate(glm::radians(-45.0),glm::vec3(0.0,1.0,0.0));
    Board.Rotate(glm::radians(-90.0),glm::vec3(1.0,0.0,0.0));
    Board.Rotate(glm::radians(90.0),glm::vec3(0.0,1.0,0.0));
    Board.ApplyRotation();
    Board.Rotate(glm::radians(45.0),glm::vec3(1.0,0.0,0.0));
    //Board.RotateTexture(glm::radians(-90.0));
    //Board.ApplyRotationTexture();
    Board.ScaleTexture(board_tex_scale);
    Board.PositionateTexture(center_dist - norm_tranlation_error);

    Board.SetTransform();
    Board.SetTextureTransform();

    board_position = Board.Get_position();
    glm::vec3 board_angles_rpy = glm::degrees(Board.Get_rotation());
    board_orientation = glm::vec3(board_angles_rpy.y, board_angles_rpy.z, board_angles_rpy.x);
    board_scale = Board.Get_scale();
    // load textures
    // -------------
    unsigned int floorTexture  = loadTexture("../resources/textures/marble.jpg");
    Plane.SetTexture(floorTexture);
    //glFlush();
    unsigned int chessTexture  = loadTexture("../resources/textures/chessboard.jpg");
    Board.SetTexture(chessTexture);

    // load models
    // -----------
    Model arrows("../resources/models/arrows/arrows.obj");
    Model lidar_mesh("../resources/models/lidar/lidar.obj");

    // configure a uniform buffer object
    // ---------------------------------
    // first. We get the relevant block indices

    // Now actually create the buffer
    unsigned int uboViewUniforms;
    glGenBuffers(1, &uboViewUniforms);
    glBindBuffer(GL_UNIFORM_BUFFER, uboViewUniforms);
    //unsigned long ubuffer_size =  2 * sizeof(glm::mat4) + 2*sizeof(float);
    unsigned long ubuffer_size =  2 * sizeof(glm::mat4) + sizeof(glm::vec3);
    glBufferData(GL_UNIFORM_BUFFER, ubuffer_size, NULL, GL_STATIC_DRAW);
    glBindBuffer(GL_UNIFORM_BUFFER, 0);
    
    //////////////////////////////
    unsigned int global_binding_proj_view;
    ///////////////////////////////

    //if (!global_uniform_binding_indexes.empty())
    if (global_uniform_binding_indexes.size() >= 1)
    {   
        unsigned int last = global_uniform_binding_indexes.back();

        glBindBufferRange(GL_UNIFORM_BUFFER, last, uboViewUniforms, 0, 2 * sizeof(glm::mat4));
        Plane.SetViewProjUBuffer(last);
        Board.SetViewProjUBuffer(last);

        //////////////////////
        global_binding_proj_view = last;
        //////////////////////

        global_uniform_binding_indexes.pop_back();
    }
    else 
        throw std::runtime_error("There is no uniform buffer biding availiable!");
    // define the range of the buffer that links to a uniform binding point

    // store the projection matrix (we only do this once now) (note: we're not using zoom anymore by changing the FoV)
    float far = 100.0f;
    float near = 0.1f;
    glm::mat4 projection = glm::perspective(45.0f, (float)SCR_WIDTH / (float)SCR_HEIGHT, near, far);
    glBindBuffer(GL_UNIFORM_BUFFER, uboViewUniforms);
    glBufferSubData(GL_UNIFORM_BUFFER, sizeof(glm::mat4),sizeof(glm::mat4), glm::value_ptr(projection));
    //glBufferSubData(GL_UNIFORM_BUFFER, 2*sizeof(glm::mat4),sizeof(float), &near);
    //glBufferSubData(GL_UNIFORM_BUFFER, 2*sizeof(glm::mat4) + sizeof(float),sizeof(float), &far);
    glBindBuffer(GL_UNIFORM_BUFFER, 0);
  
    /////////////////////////////////////////////
    //ARROWS//

    base_shader.use();
    unsigned int uniformBlockIndexObject = glGetUniformBlockIndex(base_shader.ID, "Matrixes_vertex");

    // then we link each shader's uniform block to this uniform binding point
    glUniformBlockBinding(base_shader.ID, uniformBlockIndexObject, global_binding_proj_view);

    base_shader.setMat4("model", glm::mat4(1.0));


    //////////////////////////////////////////

    /////////////////////////////////////////////
    //Lidar Model//

    lidar_shader.use();

    uniformBlockIndexObject = glGetUniformBlockIndex(lidar_shader.ID, "Matrixes_vertex");

    // then we link each shader's uniform block to this uniform binding point
    glUniformBlockBinding(lidar_shader.ID, uniformBlockIndexObject, global_binding_proj_view);


    //////////////////////////////////////////

    Lidar_config l_cfg = {
        .model = "VLP16",
        .ring_count = 16,
        .pos_horizontal_fov = 180.0f,
        .pos_vertical_fov = 15.0f,
        .neg_horizontal_fov = -180.0f,
        .neg_vertical_fov = -15.0f,
        .range = 100.0f,
        .rpm = 300.0f,
        .points_sec = 300000.0f,
        .is_360 = true,
        .lidar_precision = 0.03f
    };

    lidar_uncertainty = l_cfg.lidar_precision*1000.0f;
    //std::cout << "check" << std::endl;
    //usleep(100000);
    SensorScene velodyne(lscanCompute,l_cfg,glm::vec3(-3.0f,1.5f,0.0f));
    //std::cout << "check" << std::endl;
    //usleep(100000);
    velodyne.GenerateDirBuffer();
    //std::cout << "checkdir" << std::endl;
    //usleep(100000);
    velodyne.addObject(&Plane,1,"ground");
    velodyne.addObject(&Board,1,"chessboard");

    #if (DEBUG == 1)
    
    std::vector<glm::vec3> lidar_inliers(velodyne.direction_buf.size());

    ////////////////////// RENDER INLIERS ///////////////////////////////
    GLuint inliersVAO;
    GLuint inliersVBO;

    glGenVertexArrays(1, &inliersVAO);
    glGenBuffers(1, &inliersVBO);
    glBindVertexArray(inliersVAO);
    glBindBuffer(GL_ARRAY_BUFFER, inliersVBO);
    glBufferData(GL_ARRAY_BUFFER, velodyne.direction_buf.size()*sizeof(glm::vec3), NULL, GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    
    glBindVertexArray(0);



    
    Shader inliers_sh((default_shader_path+"inliers.vs").c_str(), (default_shader_path+"inliers.fs").c_str());
    inliers_sh.use();
    uniformBlockIndexObject = glGetUniformBlockIndex(inliers_sh.ID, "Matrixes_vertex");

    // then we link each shader's uniform block to this uniform binding point
    glUniformBlockBinding(inliers_sh.ID, uniformBlockIndexObject, global_binding_proj_view);
    /////////////////////////////////////////////////////////////////////

    #endif

    //std::cout << "checkdir" << std::endl;
    //usleep(100000);
    //////////////////////////////////////////
    //POINT CLOUD//

    // setup cloud VAO
    /*
    unsigned int cloudVAO, cloudVBO;
    glGenVertexArrays(1, &cloudVAO);
    glGenBuffers(1, &cloudVBO);
    glBindVertexArray(cloudVAO);
    glBindBuffer(GL_ARRAY_BUFFER, cloudVBO);
    glBufferData(GL_ARRAY_BUFFER, velodyne.direction_buf.size()*sizeof(Point_XYZIR), NULL, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Point_XYZIR), (void*)0);
    glBindVertexArray(0);
    */
    //glm::mat4 lidar_model = glm::rotate(glm::mat4(1.0f),glm::radians(0.0f),glm::vec3(0.0f,1.0f,0.0f));
    glm::mat4 lidar_model = glm::translate(glm::mat4(1.0f),glm::vec3(-3.0f,1.0f,0.0f));
    
    lidar_shader.use();

    lidar_shader.setMat4("model", lidar_model);
    
    point_sh.use();
    uniformBlockIndexObject = glGetUniformBlockIndex(point_sh.ID, "Matrixes_vertex");

    // then we link each shader's uniform block to this uniform binding point
    glUniformBlockBinding(point_sh.ID, uniformBlockIndexObject, global_binding_proj_view);

    point_sh.setMat4("model_lidar", lidar_model);

    GLuint dummyVAO;
    glGenVertexArrays(1, &dummyVAO);
    
    std::cout << velodyne.points.size() << std::endl;
    //////////////////////////////////////////

    ///////////////////////////////////////////////


    //float points[] = {
    //    // positions     // colors
    //    -0.5f,  0.5f,    1.0f, 0.0f, 0.0f,  // top-left  (red)
    //     0.5f,  0.5f,    0.0f, 1.0f, 0.0f,  // top-right (green)
    //     0.5f, -0.5f,    0.0f, 0.0f, 1.0f,  // bottom-right (blue)
    //    -0.5f, -0.5f,    1.0f, 1.0f, 0.0f   // bottom-left (yellow)
    //};

    //unsigned int VBOx, VAOx;
    //glGenVertexArrays(1, &VAOx);
    //glGenBuffers(1, &VBOx);

    //glBindVertexArray(VAOx);
      //glBindBuffer(GL_ARRAY_BUFFER, VBOx);
      //glBufferData(GL_ARRAY_BUFFER, velodyne.direction_buf.size()*sizeof(Point_XYZIR), NULL, GL_STATIC_DRAW);

      // position attribute (location = 0)
      //glEnableVertexAttribArray(0);
      //glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Point_XYZIR), (void*)0);

      // color attribute (location = 1)
      //glEnableVertexAttribArray(1);
      //glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float),
                            //(void*)(2 * sizeof(float)));
    //glBindVertexArray(0);


    //Shader default_points((default_shader_path+"points.vs").c_str(), (default_shader_path+"points.fs").c_str());

    //default_points.use();

    //uniformBlockIndexObject = glGetUniformBlockIndex(default_points.ID, "Matrixes_vertex");

    // then we link each shader's uniform block to this uniform binding point
    //glUniformBlockBinding(default_points.ID, uniformBlockIndexObject, global_binding_proj_view);

    //default_points.setMat4("model_lidar", lidar_model);

    // render loop
    // -----------

    camera.setConstrainAngle(true);
    
    lastFrame = static_cast<float>(glfwGetTime());

    //std::cout << "checkdir" << std::endl;
    //usleep(100000);

    /////////////////////////////   FIFO   //////////////////////////////////

    uint32_t fifo_msg_size = velodyne.points.size()*sizeof(Point_XYZIR);
    uint32_t fifo_recieve_size = velodyne.points.size()*3*sizeof(float);

    boost::interprocess::message_queue::remove("pts_queue");
    boost::interprocess::message_queue mq(boost::interprocess::create_only,
                                                               "pts_queue",
                                                               /*max msgs*/  5,
                                                               /*max size*/  fifo_msg_size);


    boost::interprocess::message_queue::remove("pts_queue_result");
    boost::interprocess::message_queue mq_inliers(boost::interprocess::create_only,
                                                               "pts_queue_result",
                                                               /*max msgs*/  10,
                                                               /*max size*/  fifo_recieve_size);
    /////////////////////////////////////////////////////////////////////////

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
        camera.OnRender(deltaTime);

        glClearColor(clear_color.x*clear_color.w, clear_color.y*clear_color.w, clear_color.z*clear_color.w, clear_color.w);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


        //setting the scene

        glm::mat4 view = camera.GetViewMatrix();
        glm::vec3 camera_pos = camera.GetCameraPosition();

        glBindBuffer(GL_UNIFORM_BUFFER, uboViewUniforms);
        glBufferSubData(GL_UNIFORM_BUFFER, 0, sizeof(glm::mat4), glm::value_ptr(view));
        glBufferSubData(GL_UNIFORM_BUFFER, 2*sizeof(glm::mat4), sizeof(glm::vec3), glm::value_ptr(camera_pos));
        glBindBuffer(GL_UNIFORM_BUFFER, 0);

        Board.Change_position(board_position);

        Board.Change_scale(board_scale);

        Board.Change_rotation(glm::radians(glm::vec3(board_orientation.z,board_orientation.x,board_orientation.y)));

        //glm::vec3 normal_board_vec = glm::rotate(glm::mat3(1.0f),glm::vec3(-1.0f,0.0f,0.0f));

        //Board.Rotate();

        Board.SetTransform();
        Board.SetTextureTransform();
        Board.Draw();

        Plane.SetTransform();
        Plane.SetTextureTransform();
        Plane.Draw();

        //base_shader.use();
        //arrows.Draw(base_shader);

        lidar_shader.use();
        lidar_mesh.Draw(lidar_shader);

        //std::cout << "checkloop1" << std::endl;
        //usleep(10000);

        velodyne.config.lidar_precision = lidar_uncertainty/1000.0f;

        velodyne.LidarScan(glm::inverse(lidar_model), currentFrame);
        
        if(mq.get_max_msg() > mq.get_num_msg()){

            velodyne.TransferLidarScan();

                std::cout << velodyne.points.at(55500).x << ", " 
                << velodyne.points.at(55500).y << ", "
                << velodyne.points.at(55500).z << ", "
                << velodyne.points.at(55500).ring << std::endl;

            mq.try_send(velodyne.points.data(), fifo_msg_size, /*priority*/ 0);
        }

        //std::cout <<currentFrame<<std::endl;

        //glDisable(GL_CULL_FACE);
        //glDisable(GL_DEPTH_TEST);
        //std::cout << "checkloop1" << std::endl;
        //usleep(10000);
        glBindVertexArray(dummyVAO);   
        point_sh.use();
        uniformBlockIndexObject = glGetUniformBlockIndex(point_sh.ID, "Matrixes_vertex");

        // then we link each shader's uniform block to this uniform binding point
        glUniformBlockBinding(point_sh.ID, uniformBlockIndexObject, global_binding_proj_view);

        point_sh.setMat4("model_lidar", lidar_model);

        if(show_raw_lidar_cloud) RenderCloud(point_sh,velodyne.points.size());

        glBindVertexArray(0);
        //RenderCloud(velodyne.points,point_sh,cloudVAO,cloudVBO);
        //RenderCloud(velodyne.points,default_points,VAOx,VBOx);
        //glEnable(GL_DEPTH_TEST);
        //glEnable(GL_CULL_FACE);

        /*
        glm::vec4 point  = glm::vec4(velodyne.points[10000].x,
                                     velodyne.points[10000].y,
                                     velodyne.points[10000].z, 1.0f);



        std::cout << (lidar_model*point).x << " " ;
        std::cout << (lidar_model*point).y << " " ;
        std::cout << (lidar_model*point).z << std::endl;
        */

        #if (DEBUG == 1)
        uint64_t recvd_size;
        uint32_t priority;
        if(mq_inliers.try_receive(lidar_inliers.data(), lidar_inliers.size()*sizeof(glm::vec3), recvd_size, priority)){
            //lidar_points.resize(recvd_size / sizeof(Point_XYZIR));

            glBindVertexArray(inliersVAO);
            glBindBuffer(GL_ARRAY_BUFFER, inliersVBO);
            glBufferSubData(GL_ARRAY_BUFFER, 0, recvd_size, lidar_inliers.data());

            inliers_sh.use();

            inliers_sh.setMat4("model_lidar", lidar_model);

            if(show_lidar_points_borders) glDrawArrays(GL_POINTS, 0, recvd_size/sizeof(glm::vec3));

            glBindVertexArray(0);

        }
        #endif

        //update_imgui_mouse_input(window);

        // Start ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        draw_main_popup(io,projection, view, Board);

        // Rendering
        ImGui::Render();

        glDisable(GL_DEPTH_TEST);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glEnable(GL_DEPTH_TEST);



        // glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
        // -------------------------------------------------------------------------------
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    // glfw: terminate, clearing all previously allocated GLFW resources.
    // ------------------------------------------------------------------
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;


}

// process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
// ---------------------------------------------------------------------------------------------------------

bool capture_mouse = false;
bool keyPressed = false;

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
    if (glfwGetKey(window, GLFW_KEY_G) == GLFW_PRESS && !keyPressed){
        
        capture_mouse = !capture_mouse;
        keyPressed = true;

        if(capture_mouse){
            glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
            glfwSetCursorPosCallback(window, mouse_callback);
            //glfwSetCursorPosCallback(window, nullptr);
            glfwSetMouseButtonCallback(window, nullptr);
            ImGui::GetIO().MouseDown[0] = 0;
            ImGui::GetIO().MouseDown[1] = 0;
            camera.firstMouse = true;
            fly_mode = true;
        }
        else{
            glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
            //glfwSetCursorPosCallback(window, nullptr);
            glfwSetCursorPosCallback(window, ImGui_ImplGlfw_CursorPosCallback);
            glfwSetMouseButtonCallback(window, ImGui_ImplGlfw_MouseButtonCallback);
            fly_mode = false;
        }
    } 
    else if (glfwGetKey(window, GLFW_KEY_G) == GLFW_RELEASE){
        keyPressed = false;  // Reset when key is released
    }

}



// glfw: whenever the window size changed (by OS or user resize) this callback function executes
// ---------------------------------------------------------------------------------------------
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    // make sure the viewport matches the new window dimensions; note that width and 
    // height will be significantly larger than specified on retina displays.
    glViewport(0, 0, width, height);
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

    //camera.ProcessMouseMovement(xoffset, yoffset);
    camera.ProcessMouseMovement(xpos, ypos);
}

// glfw: whenever the mouse scroll wheel scrolls, this callback is called
// ----------------------------------------------------------------------
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    camera.ProcessMouseScroll(static_cast<float>(yoffset));
}



inline void draw_main_popup(ImGuiIO& io, const glm::mat4& proj_matrix,const glm::mat4& view_matrix,Object& object){

    ImGuiWindowFlags windowFlags = ImGuiWindowFlags_None | ImGuiWindowFlags_MenuBar;

    //ImGuiWindowFlags_NoBringToFrontOnFocus;//(ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);
    if(fly_mode) windowFlags |= ImGuiWindowFlags_NoInputs;

    static bool opt_fullscreen = true;
    static ImGuiDockNodeFlags dockspace_flags = ImGuiDockNodeFlags_None;

    ImGuiWindowFlags window_flags = ImGuiWindowFlags_MenuBar | ImGuiWindowFlags_NoDocking;
    if (opt_fullscreen)
    {
        const ImGuiViewport* viewport = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(viewport->Pos);
        ImGui::SetNextWindowSize(viewport->Size);
        ImGui::SetNextWindowViewport(viewport->ID);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
        window_flags |= ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove;
        window_flags |= ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus;
    }

    ImGui::Begin("MainDockSpace", nullptr, window_flags);
    ImGui::PopStyleVar(2);

    // ImGui UI elements
    //ImGui::Begin("Settings",nullptr,windowFlags);
    ImGuiID dockspace_id = ImGui::GetID("MyDockSpace");
    ImGui::DockSpace(dockspace_id, ImVec2(0.0f, 0.0f), dockspace_flags);
    if (ImGui::BeginMenuBar())
    {
        if (ImGui::BeginMenu("View"))
        {
            if (ImGui::Checkbox("Show Lidar Settings", &show_lidar_settings)) { /* Do stuff */ }
            if (ImGui::MenuItem("Save", "Ctrl+S"))   { /* Do stuff */ }
            if (ImGui::MenuItem("Close", "Ctrl+W"))  { /*my_tool_active = false; */}
            ImGui::EndMenu();
        }
        ImGui::EndMenuBar();
    }


    //ImGui::BeginDisabled(true); // Disable all widgets inside this scope

    ImGui::Checkbox("Show Lidar Settings", &show_lidar_settings);
    ImGui::SameLine();
    ImGui::Checkbox("Show Board Settings", &show_board_settings);

    ImGui::Text("Lidar uncertanty (mm): ");
    ImGui::SameLine();
    ImGui::SliderFloat(" ",&lidar_uncertainty, 0.0f, 300.0f);            // Edit 1 float using a slider from 0.0f to 1.0f
    ImGui::Checkbox("Show Lidar Cloud", &show_raw_lidar_cloud);       
    
    
    
    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);
    //ImGui::EndDisabled(); // Re-enable widgets after this scope 
    ImGui::End();
    
    if(show_lidar_settings){
        ImGui::Begin("Lidar Settings",nullptr,windowFlags);
        ImGui::Checkbox("Show Detected Borders", &show_lidar_points_borders);
        ImGui::End();
    }

    const float pos_amplitude = 10.0f; // Size of the area for the 2D slider
    const float arcSize = 360.0f;
    const float scale_amplitude = 10.0f;

    if(show_board_settings){
        ImGui::Begin("Board Settings",nullptr,windowFlags);

        ImGui::Text("Translation");
        // Create sliders for X and Y coordinates
        //ImGui::SliderFloat("Z Position", &board_position.z, -pos_amplitude, pos_amplitude, "Z: %.1f");
        ImGui::DragFloat("X Position", &board_position.x, 0.01f,-pos_amplitude,pos_amplitude,"X: %.2f");
        ImGui::DragFloat("Y Position", &board_position.y, 0.01f,-pos_amplitude,pos_amplitude,"Y: %.2f");
        ImGui::DragFloat("Z Position", &board_position.z, 0.01f,-pos_amplitude,pos_amplitude,"Z: %.2f");

        ImGui::Text("Rotation");

        ImGui::DragFloat("X Rotation", &board_orientation.x,0.1f,-arcSize, arcSize, "X: %.1f");
        ImGui::DragFloat("Y Rotation", &board_orientation.y,0.1f,-arcSize, arcSize, "Y: %.1f");
        ImGui::DragFloat("Z Rotation", &board_orientation.z,0.1f,-arcSize, arcSize, "Z: %.1f");

        ImGui::Text("Scale");

        ImGui::DragFloat("X Scale", &board_scale.x,0.01f,-scale_amplitude, scale_amplitude, "X: %.3f");
        ImGui::DragFloat("Y Scale", &board_scale.y,0.01f,-scale_amplitude, scale_amplitude, "Y: %.3f");
        ImGui::DragFloat("Z Scale", &board_scale.z,0.01f,-scale_amplitude, scale_amplitude, "Z: %.3f");

        ImGui::End();
    }

    ImGuizmo::BeginFrame();
    ImGuizmo::SetOrthographic(false);
    ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);

    glm::mat4 model_m(1.0);

    // 5. Convert matrices to float arrays
    float viewMatrix[16], projMatrix[16], modelMatrix[16], deltaMatrix[16];
    memcpy(viewMatrix, glm::value_ptr(view_matrix), sizeof(float) * 16);
    memcpy(projMatrix, glm::value_ptr(proj_matrix), sizeof(float) * 16);
    memcpy(modelMatrix, glm::value_ptr(object.Get_model_matrix()), sizeof(float) * 16);

    ImGuizmo::Manipulate(viewMatrix, projMatrix,
                 ImGuizmo::TRANSLATE, // or ROTATE, SCALE
                 ImGuizmo::LOCAL,
                 modelMatrix);

    // 6. Call ImGuizmo
    //ImGuizmo::Manipulate(viewMatrix, projMatrix,
    //                     ImGuizmo::TRANSLATE, // or ROTATE, SCALE
    //                     ImGuizmo::WORLD,
    //                     modelMatrix);

    // 7. Update model matrix from manipulation
    if (ImGuizmo::IsUsing())
    {   
        float translation[3], rotation[3], scale[3];
        ImGuizmo::DecomposeMatrixToComponents(modelMatrix, translation, rotation, scale);
        //board_position[0] = translation[0];
        //board_position[1] = translation[1];
        //board_position[2] = translation[2];

        //board_orientation[0] = rotation[0];
        //board_orientation[1] = rotation[1];
        //board_orientation[2] = rotation[2];

        //std::cout << "x: " << rotation[0] << std::endl;
        //std::cout << "y: " << rotation[1] << std::endl;
        //std::cout << "z: " << rotation[2] << std::endl;

        //std::cout << "x: " << translation[0] << std::endl;
        //std::cout << "y: " << translation[1] << std::endl;
        //std::cout << "z: " << translation[2] << std::endl;

        //glm::mat4 newModel = glm::make_mat4(modelMatrix);

        glm::vec3 scale_v = glm::make_vec3(scale);
        glm::vec3 rotation_v = glm::make_vec3(rotation);
        glm::vec3 translation_v = glm::make_vec3(translation);

        //scale.x = glm::length(glm::vec3(modelMatrix[0],modelMatrix[1],modelMatrix[2]));
        //scale.y = glm::length(glm::vec3(modelMatrix[4],modelMatrix[5],modelMatrix[6]));
        //scale.z = glm::length(glm::vec3(modelMatrix[8],modelMatrix[9],modelMatrix[10]));

        //std::cout << "x: " << scale.x << std::endl;
        //std::cout << "y: " << scale.y << std::endl;
        //std::cout << "z: " << scale.z << std::endl;

        object.Change_position(translation_v);
        object.Change_rotation(rotation_v);
        object.Change_scale(scale_v);
        
        
    }

} 

