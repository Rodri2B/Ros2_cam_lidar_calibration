#pragma once

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

//#include <unistd.h>

struct object_opt{
    bool visible;
    Object* object;
};

struct Lidar_config{
    string model;
    uint32_t ring_count;
    float pos_horizontal_fov;
    float pos_vertical_fov;
    float neg_horizontal_fov;
    float neg_vertical_fov;
    float range;
    float rpm;
    float points_sec;
    bool is_360;
    float lidar_precision;
};

struct Point_XYZIR{
    float x;
    float y;
    float z;
    float intensity;
    uint32_t ring;
    // Padding to match std430 alignment
    uint32_t padding[3];
};

struct Triangle{
    glm::vec4 p0;
    glm::vec4 p1;
    glm::vec4 p2;
    glm::vec3 n;
    
};

class SensorScene{

    public:

    SensorScene(Shader_compute &cp_sh, const Lidar_config &l_config, const glm::vec3 &l_origin);
    
    
    //void RenderScene();
    //void RayIntersect(Point_XYZIR &point,glm::vec3 &dir);
    void LidarScan(const glm::mat4 &lidar_model_matrix, const float &time);
    void GenerateDirBuffer(const bool &generate_from_scratch);
    void BindStorageBuffers();

    void TransferLidarScan();
    unsigned int GetPointBufferOBJ();

    void addObject(Object* object,const bool &visible,const string &object_name);
    void removeObject(const string &object_name);


    Lidar_config config; 

    uint32_t objects_num;
    uint32_t work_groups_num;
    std::map<string,object_opt> objects;
    std::vector<Point_XYZIR> points;
    std::vector<glm::vec4> direction_buf;

    glm::vec3 origin;
    private:

    Shader_compute &localCompute;
    unsigned int ssbo_dir_bo;
    unsigned int ssbo_point_bo;

    uint32_t dispatch_x;
    uint32_t dispatch_y;
    uint32_t dispatch_z;

    
};

SensorScene::SensorScene(Shader_compute &cp_sh,const Lidar_config &l_config, const glm::vec3 &l_origin) :
localCompute(cp_sh)
{
    config = l_config;
    origin = l_origin;
    objects_num = 0;
}

void SensorScene::addObject(Object* object,const bool &visible,const string &object_name)
{   
    //object_loader::Buffers bufs = object->getBuffers();
    //glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, bufs.ObjectVBO);
    object_opt opt = {
        .visible = visible,
        .object = object
    };
    objects.insert({object_name,opt});
    objects_num++;
}

void SensorScene::removeObject(const string &object_name)
{   
    objects.erase(object_name);
    objects_num--;
}


void SensorScene::GenerateDirBuffer(const bool &generate_from_scratch)
{

    double vertical_step = static_cast<double>(config.pos_vertical_fov - config.neg_vertical_fov)/static_cast<double>(config.ring_count - 1);
    uint32_t ring_num_dots = static_cast<uint32_t>(static_cast<double>(config.points_sec) * 60.0/(static_cast<double>(config.ring_count)* config.rpm));
    double horizontal_step;

    if(!config.is_360){
        horizontal_step = static_cast<double>(config.pos_horizontal_fov - config.neg_horizontal_fov)/static_cast<double>(ring_num_dots-1);
    }
    else{
        horizontal_step = 360.0/static_cast<double>(ring_num_dots);
    }

    double horizontal_angle = static_cast<double>(-config.pos_horizontal_fov);
    double vertical_angle = static_cast<double>(90.0f - config.pos_vertical_fov);

    direction_buf.resize(config.ring_count*ring_num_dots);

    std::size_t dir_index_conter = 0;

    for(uint32_t vertical_axis = 0; vertical_axis < config.ring_count; vertical_axis++){
        for(uint32_t horizontal_axis = 0; horizontal_axis < ring_num_dots; horizontal_axis++){

            glm::vec4 direction;
            direction.z = static_cast<float>(std::cos(glm::radians(horizontal_angle))*std::sin(glm::radians(vertical_angle)));
            direction.x = static_cast<float>(std::sin(glm::radians(horizontal_angle))*std::sin(glm::radians(vertical_angle)));
            direction.y = static_cast<float>(std::cos(glm::radians(vertical_angle)));
            direction.w = 0.0f;

            direction_buf.at(dir_index_conter) = direction;

            //Point_XYZIR dot;
            //RayIntersect(dot,direction);

            //dot.ring = vertical_axis;

            //points.push_back(dot);

            horizontal_angle += horizontal_step;

            dir_index_conter++;
        }

        vertical_angle += vertical_step;
    }
    
    localCompute.use();

    uint32_t dir_buf_size = direction_buf.size();

    std::cout << "dirs number: "<<dir_buf_size << std::endl;

    //points.reserve(dir_buf_size);
    points.resize(dir_buf_size);
    for (uint32_t i = 0; i < dir_buf_size;i++){
        glm::vec4 point_infinity = (config.range + 50.0f)*direction_buf[i];
        points[i] = {
            .x = point_infinity.x,
            .y = point_infinity.y,
            .z = point_infinity.z,
            .ring = i/ring_num_dots,
        };

    }
    std::cout << "points number: "<<points.size() << std::endl;
    // ---------------------------------
    // first. We get the relevant block indices
    unsigned int BlockIndex = glGetProgramResourceIndex(localCompute.ID, GL_SHADER_STORAGE_BLOCK, "data_dir");

    // then we link each shader's uniform block to this uniform binding point
    glShaderStorageBlockBinding(localCompute.ID, BlockIndex, 0);

    // Now actually create the buffer
    
    if(generate_from_scratch) glGenBuffers(1, &ssbo_dir_bo);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo_dir_bo);
    //glBufferData(GL_SHADER_STORAGE_BUFFER, 10*sizeof(uint), NULL, GL_DYNAMIC_COPY);
    glBufferData(GL_SHADER_STORAGE_BUFFER, dir_buf_size*sizeof(glm::vec4), direction_buf.data(), GL_DYNAMIC_DRAW);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
    // define the range of the buffer that links to a uniform binding point
    //glBindBufferRange(GL_SHADER_STORAGE_BUFFER, 0, ssbo_dir_bo, 0, 10*sizeof(uint));
    glBindBufferRange(GL_SHADER_STORAGE_BUFFER, 0, ssbo_dir_bo, 0, dir_buf_size*sizeof(glm::vec4));

    // first. We get the relevant block indices
    BlockIndex = glGetProgramResourceIndex(localCompute.ID, GL_SHADER_STORAGE_BLOCK, "data_point");

    // then we link each shader's uniform block to this uniform binding point
    glShaderStorageBlockBinding(localCompute.ID, BlockIndex, 1);

    if(generate_from_scratch) glGenBuffers(1, &ssbo_point_bo);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo_point_bo);
    //glBufferData(GL_SHADER_STORAGE_BUFFER, 10*sizeof(uint), NULL, GL_DYNAMIC_COPY);
    glBufferData(GL_SHADER_STORAGE_BUFFER, dir_buf_size*sizeof(Point_XYZIR), points.data(), GL_DYNAMIC_DRAW);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
    // define the range of the buffer that links to a uniform binding point
    //glBindBufferRange(GL_SHADER_STORAGE_BUFFER, 0, ssbo_dir_bo, 0, 10*sizeof(uint));
    glBindBufferRange(GL_SHADER_STORAGE_BUFFER, 1, ssbo_point_bo, 0, dir_buf_size*sizeof(Point_XYZIR));

    work_groups_num = dir_buf_size/WARP_SIZE;
    work_groups_num += (dir_buf_size%WARP_SIZE > 0) ? 1UL : 0UL;

    // query limitations
	// -----------------
	uint32_t max_compute_work_group_count[3];
	uint32_t max_compute_work_group_size[3];
	uint32_t max_compute_work_group_invocations;

    Query_compute_limitations(max_compute_work_group_count,
                               max_compute_work_group_size,
                               max_compute_work_group_invocations);
    
    uint64_t max_workgroup_num = static_cast<uint64_t>(max_compute_work_group_count[0])*
                                 static_cast<uint64_t>(max_compute_work_group_count[1]);

    if(static_cast<uint64_t>(work_groups_num) > max_workgroup_num) throw std::runtime_error("Max workgroup number exeded");

    if(work_groups_num <= max_compute_work_group_count[0]){ 
        dispatch_x = work_groups_num;
        dispatch_y = 1;
        dispatch_z = 1;
    }
    else if((work_groups_num/max_compute_work_group_count[0] + 
             ((work_groups_num%max_compute_work_group_count[0] > 0) ? 1UL : 0UL) ) <= max_compute_work_group_count[1]){

        dispatch_x = max_compute_work_group_count[0];
        dispatch_y = (work_groups_num/max_compute_work_group_count[0] + 
                     ((work_groups_num%max_compute_work_group_count[0]  > 0) ? 1UL : 0UL) ) ;
        dispatch_z = 1;

    }
    else throw std::runtime_error("Max workgroup number exeded");

}

void SensorScene::BindStorageBuffers(){
    uint32_t points_buf_size = points.size();
    glBindBufferRange(GL_SHADER_STORAGE_BUFFER, 0, ssbo_dir_bo, 0, points_buf_size*sizeof(glm::vec4));
    glBindBufferRange(GL_SHADER_STORAGE_BUFFER, 1, ssbo_point_bo, 0, points_buf_size*sizeof(Point_XYZIR));
}

/*
void SensorScene::RayIntersect(Point_XYZIR &point,glm::vec3 &dir)
{
    for(uint32_t i = 0; i < objects_num; i++){
        if(objects[i].visible){
            
            switch(objects[i].object->object_type){

                case (ObjTypes::Object_types::Plane) :
                    plane* obj = static_cast<plane*>(objects[i].object);
                    glm::mat3 normal_matrix = glm::transpose(glm::inverse(glm::mat3(obj->model_m)));

                    Triangle tri1;
                    Triangle tri2;

                    tri1.p0.x = Plane::Vertices[Plane::Indices[0]*8 + 0];
                    tri1.p0.y = Plane::Vertices[Plane::Indices[0]*8 + 1];
                    tri1.p0.z = Plane::Vertices[Plane::Indices[0]*8 + 2];
                    tri1.p0.w = 1.0f;
                    tri1.p0 = obj->model_m*tri1.p0;

                    tri1.p1.x = Plane::Vertices[Plane::Indices[1]*8 + 0];
                    tri1.p1.y = Plane::Vertices[Plane::Indices[1]*8 + 1];
                    tri1.p1.z = Plane::Vertices[Plane::Indices[1]*8 + 2];
                    tri1.p1.w = 1.0f;
                    tri1.p0 = obj->model_m*tri1.p1;

                    tri1.p2.x = Plane::Vertices[Plane::Indices[2]*8 + 0];
                    tri1.p2.y = Plane::Vertices[Plane::Indices[2]*8 + 1];
                    tri1.p2.z = Plane::Vertices[Plane::Indices[2]*8 + 2];
                    tri1.p2.w = 1.0f;
                    tri1.p2 = obj->model_m*tri1.p2;

                    tri1.n = normal_matrix*tri1.n;

                    tri2.p0.x = Plane::Vertices[Plane::Indices[3]*8 + 0];
                    tri2.p0.y = Plane::Vertices[Plane::Indices[3]*8 + 1];
                    tri2.p0.z = Plane::Vertices[Plane::Indices[3]*8 + 2];
                    tri2.p0.w = 1.0f;
                    tri2.p0 = obj->model_m*tri2.p0;

                    tri2.p1.x = Plane::Vertices[Plane::Indices[4]*8 + 0];
                    tri2.p1.y = Plane::Vertices[Plane::Indices[4]*8 + 1];
                    tri2.p1.z = Plane::Vertices[Plane::Indices[4]*8 + 2];
                    tri2.p1.w = 1.0f;
                    tri2.p1 = obj->model_m*tri1.p1;

                    tri2.p2.x = Plane::Vertices[Plane::Indices[5]*8 + 0];
                    tri2.p2.y = Plane::Vertices[Plane::Indices[5]*8 + 1];
                    tri2.p2.z = Plane::Vertices[Plane::Indices[5]*8 + 2];
                    tri2.p2.w = 1.0f;
                    tri2.p2 = obj->model_m*tri1.p2;

                    tri2.n = normal_matrix*tri2.n;

                    GetIntersection(tri1);



                    break;
                case (ObjTypes::Object_types::Mesh) :
                    //static_cast<plane*>(objects[i].object)->Draw();
                    break;
            }
            
        }
    }
     
}
*/

void SensorScene::LidarScan(const glm::mat4 &lidar_model_matrix, const float &time)
{

    map<string,object_opt>::iterator it;

    localCompute.use();
    localCompute.setUint("compute_op_flag", 1U);
    localCompute.setUint("points_num",direction_buf.size());
    localCompute.setFloat("range",config.range);
    localCompute.dispatch(dispatch_x,dispatch_y,dispatch_z);
    localCompute.memory_barrier(GL_ALL_BARRIER_BITS);

    for (it = objects.begin(); it != objects.end(); it++)
    {
        if(it->second.visible){
            object_loader::Buffers buf;
            buf = it->second.object->getBuffers();
            // Bind the same VBO as an SSBO for compute shader access
            glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, buf.ObjectVBO);
            glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, buf.ObjectEBO); // Binding SSBO to binding point

            localCompute.use();
            //localCompute.setVec3("l_origin",origin);
            localCompute.setUint("compute_op_flag", 0U);
            localCompute.setUint("points_num",direction_buf.size());
            localCompute.setUint("triangles_num",buf.ObjectEBSize/3);
            localCompute.setMat4("model",it->second.object->Get_model_matrix());
            localCompute.setMat4("inverse_lidar_model",lidar_model_matrix);
            //std::cout << "checkloop2" << std::endl;
            //usleep(10000);
            localCompute.dispatch(dispatch_x,dispatch_y,dispatch_z);
            localCompute.memory_barrier(GL_ALL_BARRIER_BITS);
            //std::cout << "checkloop3" << std::endl;
            //usleep(10000);

        }

    }

    if(config.lidar_precision > 0.0f){
        localCompute.use();
        localCompute.setUint("points_num",direction_buf.size());
        localCompute.setUint("compute_op_flag", 2U);
        localCompute.setFloat("lidar_precision",config.lidar_precision);
        localCompute.setFloat("time",time);

        localCompute.dispatch(dispatch_x,dispatch_y,dispatch_z);
        localCompute.memory_barrier(GL_ALL_BARRIER_BITS);
    }
    /////////////////////////////////// Transfer Data ////////////////////////////////////////
    //glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo_point_bo);
    //glGetBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, direction_buf.size()*sizeof(Point_XYZIR), points.data());

    //glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);

    ////////////////////////////////////////////////////////////////////////////////////////
}

void SensorScene::TransferLidarScan(){

    /////////////////////////////////// Transfer Data ////////////////////////////////////////
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo_point_bo);
    glGetBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, direction_buf.size()*sizeof(Point_XYZIR), points.data());

    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);

    ////////////////////////////////////////////////////////////////////////////////////////

}

/*
void Scene::RenderScene(){
    for(uint i = 0; i < objects_num; i++)
    {
        if(objects[i].visible){
            switch(objects[i].object->object_type){

                case (ObjTypes::Object_types::Plane) :
                    static_cast<plane*>(objects[i].object)->Draw();
                    break;
                case (ObjTypes::Object_types::Circle) :

                    break;
                case (ObjTypes::Object_types::Cube) :
                    static_cast<cube*>(objects[i].object)->Draw();
                    break;
                case (ObjTypes::Object_types::Sphere) :
                    static_cast<sphere*>(objects[i].object)->Draw();
                    break;
                case (ObjTypes::Object_types::Elipse) :
                    
                    break;
                case (ObjTypes::Object_types::Icosphere) :
                    
                    break;
                case (ObjTypes::Object_types::Cylinder) :
                    
                    break;
                case (ObjTypes::Object_types::Cone) :
                    
                    break;
                case (ObjTypes::Object_types::Torus) :
                    
                    break;
                case (ObjTypes::Object_types::Mesh) :
                    static_cast<object_model*>(objects[i].object)->Draw();
                    break;
            }
        }
    }
}
*/


//void SensorScene::RenderScan(const glm::mat4 &lidar_model_matrix){

//}

void RenderCloud(const std::vector<Point_XYZIR> &points,Shader &point_sh,unsigned int pointsVAO,unsigned int pointsVBO){

    uint32_t point_cloud_size = points.size();

    point_sh.use();

    glBindVertexArray(pointsVAO);
    glBindBuffer(GL_ARRAY_BUFFER, pointsVBO);
    glBufferData(GL_ARRAY_BUFFER, point_cloud_size*sizeof(Point_XYZIR), points.data(), GL_STATIC_DRAW);
    glDrawArrays(GL_POINTS, 0, point_cloud_size);
    glBindVertexArray(0); 


}

void RenderCloud(Shader &point_sh,const uint32_t &point_cloud_size){

    point_sh.use();

    glDrawArrays(GL_POINTS, 0, point_cloud_size);


}

unsigned int SensorScene::GetPointBufferOBJ(){
    return ssbo_point_bo;
}