cmake_minimum_required (VERSION 3.16)

project ("opengl_test" VERSION 1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall")
#set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")

cmake_policy(SET CMP0072 NEW)
#cmake_policy(SET CMP00144 NEW)
find_package(OpenGL REQUIRED)

##################### PCL ###################################
# Make sure CMake will search your local PCL install
# (uses the HOME environment variable so it works on any machine/user)
list(APPEND CMAKE_PREFIX_PATH
     "$ENV{HOME}/Libraries/pcl1.15.0"
     "$ENV{HOME}/Libraries/eigen3.4"
     "$ENV{HOME}/Libraries/VTK9.4.2"
     "$ENV{HOME}/Libraries/flann"
     "$ENV{HOME}/Libraries/boost1.88"
     "$ENV{HOME}/Libraries/qhull"
     "$ENV{HOME}/Libraries/OpenCV"
     "$ENV{HOME}/Libraries/yaml_cpp"
)   

find_package( Eigen3 3.4 REQUIRED )

find_package( PCL 1.15 REQUIRED )
#find_package(PCL 1.15 REQUIRED COMPONENTS common io visualization)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#############################################################

### OpenCV

find_package(OpenCV REQUIRED )

############################################################

set(CURRENT_USER $ENV{USER})

set(LIB_DIR "/home/${CURRENT_USER}/Libraries")

add_subdirectory("${LIB_DIR}/glfw" "${CMAKE_BINARY_DIR}/glfw")

include_directories("${LIB_DIR}/glfw/include")

add_subdirectory("${LIB_DIR}/glm" "${CMAKE_BINARY_DIR}/glm")

include_directories("${LIB_DIR}")

set(GLAD_DIR "${LIB_DIR}/Glad")
set(IMGUI_DIR "${LIB_DIR}/imgui")

add_library(
    glad
    ${GLAD_DIR}/glad/glad.h
    ${GLAD_DIR}/glad/glad.c
    ${GLAD_DIR}/KHR/khrplatform.h
)

file(GLOB IMGUI_SOURCES "${IMGUI_DIR}/*.cpp")
file(GLOB IMGUI_HEADERS "${IMGUI_DIR}/*.h")

#file(GLOB IMGUI_SOURCES_BACK "${IMGUI_DIR}/backends/*.cpp")
#file(GLOB IMGUI_HEADERS_BACK "${IMGUI_DIR}/backends/*.h")


# Exclude a specific file
#list(FILTER IMGUI_SOURCES EXCLUDE REGEX ".*imgui_impl_vulkan\\.cpp$")
#list(FILTER IMGUI_SOURCES EXCLUDE REGEX ".*imgui_impl_allegro5\\.cpp$")

#list(REMOVE_ITEM IMGUI_SOURCES "${IMGUI_DIR}/backends/imgui_impl_allegro5.cpp")

set(VHACD_DIR "${LIB_DIR}/v-hacd/include")




add_library(
    imgui
    ${IMGUI_HEADERS}
    ${IMGUI_SOURCES}
    ${IMGUI_DIR}/backends/imgui_impl_glfw.h
    ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
    ${IMGUI_DIR}/backends/imgui_impl_opengl3.h
    ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
    
)

target_include_directories(imgui PUBLIC ${IMGUI_DIR} ${IMGUI_DIR}/backends)

add_library(
    camera
    #include/camera/lib_opengl/camera.h
    include/camera/lib_opengl/quaternion_custom.h
    include/camera/lib_opengl/camera_quat.h
    include/camera/lib_opengl/camera_quat.cpp
    include/camera/lib_opengl/camera_quat_arc.cpp
    include/camera/lib_opengl/camera_quat_virtual.cpp


)

add_library(
    mesh
    include/mesh/lib_opengl/mesh_original.h

)

add_library(
    model
    include/model/lib_opengl/model_original.h

)

add_library(
    shader
    include/shader/lib_opengl/shader.h
    include/shader/lib_opengl/shader_compute.h
)

add_library(
    utility
    include/utility/binding_vector.hpp
    include/utility/object_loader.hpp
    
)

add_library(
    texture_loader
    include/texture_loader/texture_loader.hpp
)

add_library(
    grid
    include/grid/grid.hpp
    include/grid/grid.cpp
)


add_library(
    lidar_scan
    include/lidar/lidar_scan.hpp
)

add_library(
    main_scene
    include/scene/scene.hpp
    include/scene/scene.cpp
)

add_library(
    calib_scene
    include/scene/calib_scene.hpp
    include/scene/calib_scene.cpp
)

add_library(
    ImGuizmo
    include/utility/Imguizmo/ImGuizmo.h
    include/utility/Imguizmo/imoguizmo.hpp
    include/utility/Imguizmo/ImGuizmo.cpp
)

target_include_directories(glad PUBLIC "${GLAD_DIR}")
target_include_directories(camera PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include/camera")
target_include_directories(mesh PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include/mesh")
target_include_directories(model PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include/model")
target_include_directories(shader PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include/shader")

target_include_directories(utility PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include/utility")
target_include_directories(texture_loader PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include/texture_loader")
target_include_directories(grid PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include/grid")

target_include_directories(lidar_scan PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include/lidar")

target_include_directories(ImGuizmo PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include/utility/Imguizmo")

set_target_properties(camera PROPERTIES LINKER_LANGUAGE CXX)
set_target_properties(mesh PROPERTIES LINKER_LANGUAGE CXX)
set_target_properties(model PROPERTIES LINKER_LANGUAGE CXX)
set_target_properties(shader PROPERTIES LINKER_LANGUAGE CXX)

set_target_properties(utility PROPERTIES LINKER_LANGUAGE CXX)
set_target_properties(texture_loader PROPERTIES LINKER_LANGUAGE CXX)
set_target_properties(grid PROPERTIES LINKER_LANGUAGE CXX)

set_target_properties(lidar_scan PROPERTIES LINKER_LANGUAGE CXX)


set_target_properties(ImGuizmo PROPERTIES LINKER_LANGUAGE CXX)

set_target_properties(main_scene PROPERTIES LINKER_LANGUAGE CXX)

set_target_properties(calib_scene PROPERTIES LINKER_LANGUAGE CXX)

# Set the path to Assimp's local installation
set(ASSIMP_DIR "${LIB_DIR}/assimp")

# Add the Assimp include directory
include_directories(${ASSIMP_DIR}/include)

# Add the Assimp library directory
link_directories(${ASSIMP_DIR}/lib)

# Set the path to Freetype's local installation
set(FREETYPE_DIR "${LIB_DIR}/freetype")

# Add the FreeType include directory
include_directories(${FREETYPE_DIR}/include/freetype2/)

# Add the FreeType library directory
link_directories(${FREETYPE_DIR}/lib)

# Link against the FreeType library
find_library(FREETYPE_LIBRARY freetype PATHS ${FREETYPE_DIR}/lib)

#target_include_directories(glad PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src")

# Platform-specific dependencies
#if (APPLE)
#    set(PLATFORM_SPECIFIC "\"-framework OpenGL\" \"-framework Cocoa\" \"-framework IOKit\" \"-framework CoreVideo\"")
#elseif (UNIX)
#    set(PLATFORM_SPECIFIC "GL X11 pthread Xrandr Xi dl")
#elseif (WIN32)
#    set(PLATFORM_SPECIFIC "opengl32")
#endif()



add_executable (bagsim
    #src/pbr/specular/pbrs2.cpp
    src/bagsimulator/bagsim_gui.cpp
)

add_executable (calib
    #src/pbr/specular/pbrs2.cpp
    include/utility/yaml/calib_yaml_config.hpp
    src/bagsimulator/calib_gui.cpp
)

add_executable (bridge
    #src/pbr/specular/pbrs2.cpp
    include/utility/yaml/calib_yaml_config.hpp
    src/bagsimulator/sim_calib_bridge.cpp
)

###add_executable (${PROJECT_NAME}
###    #src/pbr/specular/pbrs2.cpp
###    src/bagsimulator/bagsim.cpp
###)
###
add_executable (ransac
    #src/pbr/specular/pbrs2.cpp
    #src/bagsimulator/cluster_extraction.cpp
    include/utility/yaml/lidar_yaml_config.hpp
    src/bagsimulator/bagsim_ransac.cpp
)

add_executable (bag_pnp
    #src/pbr/specular/pbrs2.cpp
    #src/bagsimulator/cluster_extraction.cpp
    include/utility/yaml/yaml_config.hpp
    src/bagsimulator/bagsim_pnp.cpp
)

###add_executable (imgui_test
###    #src/pbr/specular/pbrs2.cpp
###    #src/bagsimulator/cluster_extraction.cpp
###    src/imgui_test/gui_main.cpp
###)


#add_executable (${PROJECT_NAME}
#    src/load_model2.cpp
#)

#add_executable (hello2
#    src/hello2.cpp
#)

include_directories("${CMAKE_CURRENT_SOURCE_DIR}")

set(STB_INCLUDE_PATH "${LIB_DIR}/stb")

target_include_directories(bagsim PUBLIC "${STB_INCLUDE_PATH}")

target_include_directories(calib PUBLIC "${STB_INCLUDE_PATH}")

target_include_directories(bridge PUBLIC "${STB_INCLUDE_PATH}")
###target_include_directories(${PROJECT_NAME} PUBLIC "${STB_INCLUDE_PATH}")
###target_include_directories(imgui_test PUBLIC "${STB_INCLUDE_PATH}")

set(glm_DIR "${LIB_DIR}/glm") # if necessary

#find_package(glm CONFIG REQUIRED)

target_link_libraries(camera PUBLIC glad glm::glm-header-only shader)

target_link_libraries(utility PUBLIC shader)

target_link_libraries(grid PUBLIC glad glm::glm-header-only shader)

target_link_libraries(ImGuizmo PUBLIC imgui)



############################## BOOST #####################################

#set(Boost_USE_STATIC_LIBS OFF) 
#set(Boost_USE_MULTITHREADED ON)  
#set(Boost_USE_STATIC_RUNTIME OFF) 
find_package(Boost 1.88 REQUIRED COMPONENTS thread) 

include_directories(${Boost_INCLUDE_DIRS}) 

##########################################################################

####################### yaml ###########################################

find_package(yaml-cpp REQUIRED)

########################################################################

target_include_directories(main_scene PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include/scene" "${STB_INCLUDE_PATH}" )
target_link_libraries(main_scene PUBLIC lidar_scan glad shader grid utility texture_loader camera mesh model glfw ${GLFW_LIBRARIES} OpenGL::GL  glm::glm-header-only  ${FREETYPE_LIBRARY} imgui ImGuizmo)

target_include_directories(calib_scene PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include/scene" "${STB_INCLUDE_PATH}" "include/utility/yaml/")
target_link_libraries(calib_scene PUBLIC opencv_core opencv_imgproc opencv_calib3d ${PCL_LIBRARIES} ${Boost_LIBRARIES} Boost::thread lidar_scan glad shader grid utility yaml-cpp::yaml-cpp texture_loader camera mesh model glfw ${GLFW_LIBRARIES} OpenGL::GL  glm::glm-header-only imgui ImGuizmo)

target_include_directories(bridge PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include/scene" "${STB_INCLUDE_PATH}" "include/utility/yaml/")
target_link_libraries(bridge PUBLIC  ${Boost_LIBRARIES} Boost::thread Eigen3::Eigen yaml-cpp::yaml-cpp  )

########################################################

target_link_libraries(bagsim PUBLIC ${Boost_LIBRARIES} main_scene lidar_scan glad shader grid utility texture_loader camera mesh model glfw ${GLFW_LIBRARIES} OpenGL::GL glm::glm-header-only assimp ${FREETYPE_LIBRARY} imgui ImGuizmo)

target_link_libraries(calib PUBLIC opencv_core opencv_imgproc opencv_calib3d ${PCL_LIBRARIES} ${Boost_LIBRARIES} Boost::thread calib_scene lidar_scan glad shader grid utility yaml-cpp::yaml-cpp texture_loader camera mesh model glfw ${GLFW_LIBRARIES} OpenGL::GL glm::glm-header-only assimp imgui ImGuizmo)

###target_link_libraries(${PROJECT_NAME} PUBLIC ${Boost_LIBRARIES} lidar_scan v_hacd glad shader grid utility texture_loader camera mesh model glfw ${GLFW_LIBRARIES} OpenGL::GL glm::glm-header-only assimp ${FREETYPE_LIBRARY} imgui ImGuizmo)
target_include_directories(ransac PUBLIC "include/utility/yaml/" )
target_link_libraries(ransac PUBLIC ${PCL_LIBRARIES} ${Boost_LIBRARIES} Boost::thread yaml-cpp::yaml-cpp lidar_scan glad shader grid utility texture_loader camera mesh model glfw ${GLFW_LIBRARIES} OpenGL::GL glm::glm-header-only assimp ${FREETYPE_LIBRARY} imgui)

target_include_directories(bag_pnp PUBLIC "include/utility/yaml/" )
target_link_libraries(bag_pnp PUBLIC ${OpenCV_LIBS} ${Boost_LIBRARIES} Boost::thread yaml-cpp::yaml-cpp glad glfw ${GLFW_LIBRARIES} OpenGL::GL glm::glm-header-only)
#target_link_libraries(${PROJECT_NAME} PUBLIC glad shader_loader camera mesh model glfw ${GLFW_LIBRARIES} OpenGL::GL glm::glm-header-only assimp)
#target_link_libraries(hello2 PUBLIC glad glfw ${GLFW_LIBRARIES} OpenGL::GL)


###target_link_libraries(imgui_test PUBLIC  glad shader grid utility texture_loader camera mesh model glfw ${GLFW_LIBRARIES} OpenGL::GL glm::glm-header-only assimp ${FREETYPE_LIBRARY} imgui ImGuizmo)






#target_link_libraries(i2c.out PUBLIC bsc_controller)
