#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#ifndef GLM_ENABLE_EXPERIMENTAL
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/matrix_transform_2d.hpp>
#include <glm/gtx/euler_angles.hpp>
#endif

#include <lib_opengl/shader.h>
#include <lib_opengl/model_original.h>


//std::string default_shader_path = "../shaders/default/";
//Shader default_shader((default_shader_path+"default_sh.vs").c_str(), (default_shader_path+"default_sh.fs").c_str());

/* one pissibles solution would be flip the x axis or render from the other side because that orietation was thought to
be looked from below and not from above
namespace Plane{

    float Vertices[32] = {
    // positions      //normals     //coords
   -0.5,  0.0, -0.5,  0.0, 1.0, 0.0 ,0.0, 0.0, // bottom left
    0.5,  0.0, -0.5,  0.0, 1.0, 0.0 ,1.0, 0.0, // bottom right
    0.5,  0.0,  0.5,  0.0, 1.0, 0.0 ,1.0, 1.0, // top right
   -0.5,  0.0,  0.5,  0.0, 1.0, 0.0 ,0.0, 1.0, // top left
							
    };

    unsigned int Indices[6] = {3, 2, 0, 1, 0, 2};
};

*/

namespace Plane{

    float Vertices[32] = {
    // positions      //normals     //coords
   -0.5,  0.0, -0.5,  0.0, 1.0, 0.0 ,1.0, 0.0, // bottom left
    0.5,  0.0, -0.5,  0.0, 1.0, 0.0 ,0.0, 0.0, // bottom right
    0.5,  0.0,  0.5,  0.0, 1.0, 0.0 ,0.0, 1.0, // top right
   -0.5,  0.0,  0.5,  0.0, 1.0, 0.0 ,1.0, 1.0, // top left
							
    };

    float Vertices_vertical[32] = {
    // positions      //normals     //coords
    0.0, -0.5, -0.5, -1.0, 0.0, 0.0 ,1.0, 0.0, // bottom left
    0.0,  0.5, -0.5, -1.0, 0.0, 0.0 ,0.0, 0.0, // bottom right
    0.0,  0.5,  0.5, -1.0, 0.0, 0.0 ,0.0, 1.0, // top right
    0.0, -0.5,  0.5, -1.0, 0.0, 0.0 ,1.0, 1.0, // top left
							
    };

    unsigned int Indices[6] = {3, 2, 0, 1, 0, 2};
};

namespace Cube{

    float Vertices[] = {
        // Back face
        -0.5f, -0.5f, -0.5f,  0.0, 0.0, -1.0,  0.0f, 0.0f, // Bottom-left
         0.5f,  0.5f, -0.5f,  0.0, 0.0, -1.0,  1.0f, 1.0f, // top-right
         0.5f, -0.5f, -0.5f,  0.0, 0.0, -1.0,  1.0f, 0.0f, // bottom-right         
        -0.5f,  0.5f, -0.5f,  0.0, 0.0, -1.0,  0.0f, 1.0f, // top-left
        // Front face
        -0.5f, -0.5f,  0.5f,  0.0, 0.0,  1.0,  0.0f, 0.0f, // bottom-left 
         0.5f, -0.5f,  0.5f,  0.0, 0.0,  1.0,  1.0f, 0.0f, // bottom-right
         0.5f,  0.5f,  0.5f,  0.0, 0.0,  1.0,  1.0f, 1.0f, // top-right
        -0.5f,  0.5f,  0.5f,  0.0, 0.0,  1.0,  0.0f, 1.0f, // top-left  
        // Left face
        -0.5f,  0.5f,  0.5f,  -1.0, 0.0, 0.0,  1.0f, 0.0f, // top-right 
        -0.5f,  0.5f, -0.5f,  -1.0, 0.0, 0.0,  1.0f, 1.0f, // top-left
        -0.5f, -0.5f, -0.5f,  -1.0, 0.0, 0.0,  0.0f, 1.0f, // bottom-left
        -0.5f, -0.5f,  0.5f,  -1.0, 0.0, 0.0,  0.0f, 0.0f, // bottom-right 
        // Right face
         0.5f,  0.5f,  0.5f,  1.0, 0.0,  0.0,  1.0f, 0.0f, // top-left 
         0.5f, -0.5f, -0.5f,  1.0, 0.0,  0.0,  0.0f, 1.0f, // bottom-right
         0.5f,  0.5f, -0.5f,  1.0, 0.0,  0.0,  1.0f, 1.0f, // top-right         
         0.5f, -0.5f,  0.5f,  1.0, 0.0,  0.0,  0.0f, 0.0f, // bottom-left      
        // Bottom face
        -0.5f, -0.5f, -0.5f,  0.0, -1.0, 0.0,  0.0f, 1.0f, // top-right 
         0.5f, -0.5f, -0.5f,  0.0, -1.0, 0.0,  1.0f, 1.0f, // top-left
         0.5f, -0.5f,  0.5f,  0.0, -1.0, 0.0,  1.0f, 0.0f, // bottom-left
        -0.5f, -0.5f,  0.5f,  0.0, -1.0, 0.0,  0.0f, 0.0f, // bottom-right 
        // Top face
        -0.5f,  0.5f, -0.5f,  0.0, 1.0,  0.0,  0.0f, 1.0f, // top-left 
         0.5f,  0.5f,  0.5f,  0.0, 1.0,  0.0,  1.0f, 0.0f, // bottom-right
         0.5f,  0.5f, -0.5f,  0.0, 1.0,  0.0,  1.0f, 1.0f, // top-right     
        -0.5f,  0.5f,  0.5f,  0.0, 1.0,  0.0,  0.0f, 0.0f  // bottom-left        
    };
    

    unsigned int Indices[] = {
        // Back face
        0, 1, 2,
        1, 0, 3,
        // Front face
        4, 5, 6,
        6, 7, 4,
        // Left face
        8, 9, 10,
        10, 11, 8,
        // Right face
        12, 13, 14,
        13, 12, 15,
        // Top face
        16, 17, 18,
        18, 19, 16,
        // Bottom face
        20, 21, 22,
        21, 20, 23
    };
};


struct object_config_t 
{

    glm::vec3 position;
    glm::vec3 rotation;
    glm::vec3 scale;
    glm::vec2 position_tex;
    float rotation_tex; 
    glm::vec2 scale_tex;
    glm::mat4 model_m_ref;
    Shader &dshader;

};


enum draw_mode {
    DRAW_UNTEXTURED,
    DRAW_TEXTURED,
    DRAW_UNTEXTURED_INSTANCED,
    DRAW_TEXTURED_INSTANCED,
    DRAW_MODEL,
    DRAW_MODEL_INSTANCED,
    DRAW_INFO,
    DRAW_MODEL_INFO,
    DRAW_OUTLINE,
    DRAW_MODEL_OUTLINE
};

namespace ObjTypes
{
enum Object_types {
    Plane,
    Circle,
    Cube,
    Sphere,
    Elipse,
    Icosphere,
    Cylinder,
    Cone,
    Torus,
    Mesh
};

}

namespace object_loader{
struct Buffers
{
    unsigned int ObjectVAO;
    unsigned int ObjectVBO;
    unsigned int ObjectEBO;
    unsigned int ObjectABO;
    unsigned int ObjectEBSize; 
};

}
class Object
{

public:
    Object(Shader &dshader, unsigned int dmode = DRAW_UNTEXTURED);
    Object(const glm::vec3 &pos, Shader &dshader, unsigned int dmode = DRAW_UNTEXTURED);
    Object(const glm::vec3 &pos, const glm::vec3 &rot, Shader &dshader, unsigned int dmode = DRAW_UNTEXTURED);
    Object(const glm::vec3 &pos, const glm::vec3 &rot, const glm::vec3 &scale, Shader &dshader, unsigned int dmode = DRAW_UNTEXTURED);
    Object(const object_config_t &config, unsigned int dmode = DRAW_UNTEXTURED);
    
    void Draw();
    void DrawInfo(const unsigned int &index);
    void DrawOutline();

    void setInstancePosBuffer(const unsigned int &buffer, const unsigned int &amount);

    void SetDrawMode(unsigned int mode);
    void SetDrawInfoMode(unsigned int mode);
    void SetDrawOutlineMode(unsigned int mode);
    void SetInfoShader(Shader *info_shader);
    void SetOutlineShader(Shader *outline_shader);
    void SetTexture(unsigned int TexIndex);
    void SetTransform();
    void SetProjection(const glm::mat4 &projection);
    void SetView(const glm::mat4 &projection);
    void SetInfoProjection(const glm::mat4 &projection);
    void SetOutlineProjection(const glm::mat4 &projection);
    void SetInfoView(const glm::mat4 &projection);
    void SetOutlineView(const glm::mat4 &view);
    void SetTextureTransform();
    void SetInfoTransform();
    void SetOutlineTransform();
    void SetTextureInfoTransform();
    void SetViewProjUBuffer(unsigned int biding);
    void ScaleTexture(const glm::vec2 &scale_vector);
    void RotateTexture(const float &angle);
    void PositionateTexture(const glm::vec2 &position_new);
    void ApplyRotationTexture();
    void GlobalScale(const glm::vec3 &scale_vector);
    void GlobalRotate(const float roll, const float pitch, const float yaw);
    void Scale(const glm::vec3 &scale_vector);
    void Scale(glm::mat4 &delta_scale);
    void Rotate(float angle,const glm::vec3 &rot_vector);
    void Rotate(const float roll, const float pitch, const float yaw);
    void Rotate(const glm::vec3 rot);
    void Rotate(glm::mat4 &delta_rot);
    void Tanslate(const glm::vec3 &translation);
    void ApplyAllTransfoms();
    void ApplyRotation();
    void ApplyTranslation();
    void ApplyScale();
    glm::vec3 Get_rotation();
    glm::vec3 Get_position();
    glm::vec3 Get_scale();
    glm::mat4 Get_model_matrix();
    glm::mat4 Get_reference_model_matrix();
    void Change_rotation(const glm::vec3 &rotation_new);
    void Change_position(const glm::vec3 &position_new);
    void Change_scale(const glm::vec3 &scale_vector);
    void Change_reference_model_matrix(const glm::mat4 &new_model);
    void Reset_reference_model_matrix();
    void Reset_reference_model_texture_matrix();

    object_loader::Buffers getBuffers();

    unsigned int Texture;
    Shader &localShader;
    Shader *localShader_info;
    Shader *localShader_outline;
    Model *localModel = nullptr;

    ObjTypes::Object_types object_type;

    glm::mat4 model_m = glm::mat4(1.0f);
    glm::mat3 model_m_tex = glm::mat3(1.0f);

    glm::vec3 position = glm::vec3(0.0f,0.0f,0.0f);
    glm::vec3 rotation = glm::vec3(0.0f,0.0f,0.0f); //roll, pith, yaw or x,y,z
    glm::vec3 scale = glm::vec3(1.0f,1.0f,1.0f);

protected:

    virtual void Init() = 0;

    void draw_untextured();
    void draw_textured();
    void draw_untextured_instanced();
    void draw_textured_instanced();
    void draw_model();
    void draw_model_instanced();
    void draw_info(const unsigned int &index);
    void draw_model_info(const unsigned int &index);
    void draw_outline();
    void draw_model_outline();

    //use (p.*Draw)(); out of the class
    void (Object::*Draw_method)();
    void (Object::*Draw_method_info)(const unsigned int &index);
    void (Object::*Draw_method_outline)();

    unsigned int ObjectVAO;
    unsigned int ObjectVBO;
    unsigned int ObjectEBO;

    unsigned int ObjectABO;
    unsigned int amount_i;

    unsigned int element_buffer_size;

    //glm::mat4 model_m = glm::mat4(1.0f);
    glm::mat4 model_m_ref = glm::mat4(1.0f);

    //glm::vec3 position = glm::vec3(0.0f,0.0f,0.0f);
    glm::mat4 local_pos_matrix = glm::mat4(1.0f);

    //glm::vec3 rotation = glm::vec3(0.0f,0.0f,0.0f); //roll, pith, yaw or x,y,z
    glm::mat4 local_rot_matrix = glm::mat4(1.0f);

    //glm::vec3 scale = glm::vec3(1.0f,1.0f,1.0f);
    glm::mat4 local_scale_matrix = glm::mat4(1.0f);

    //glm::mat3 model_m_tex = glm::mat3(1.0f);
    glm::mat3 model_m_tex_ref = glm::mat3(1.0f);

    glm::vec2 position_tex = glm::vec2(0.0f,0.0f);
    glm::mat3 local_pos_matrix_tex = glm::mat3(1.0f);

    float rotation_tex = 0.0f; 
    glm::mat3 local_rot_matrix_tex = glm::mat3(1.0f);

    glm::vec2 scale_tex = glm::vec2(1.0f,1.0f);
    glm::mat3 local_scale_matrix_tex = glm::mat3(1.0f);


};

Object::Object(Shader &dshader, unsigned int dmode):
localShader(dshader)
{
    SetDrawMode(dmode);
}

Object::Object(const glm::vec3 &pos,Shader &dshader, unsigned int dmode):
localShader(dshader)
{
    Change_position(pos);
    SetDrawMode(dmode);
}

Object::Object(const glm::vec3 &pos, const glm::vec3 &rot, Shader &dshader, unsigned int dmode):
localShader(dshader)
{   
    Change_rotation(rot);
    Change_position(pos);
    SetDrawMode(dmode);
}

Object::Object(const glm::vec3 &pos, const glm::vec3 &rot, const glm::vec3 &scale, Shader &dshader, unsigned int dmode):
localShader(dshader)
{   
    Change_scale(scale);
    Change_rotation(rot);
    Change_position(pos);
    SetDrawMode(dmode);
}

Object::Object(const object_config_t &config, unsigned int dmode):
localShader(config.dshader)
{

    Change_scale(config.scale);
    Change_rotation(config.rotation);
    Change_position(config.position);

    ScaleTexture(config.scale_tex);
    RotateTexture(config.rotation_tex);
    PositionateTexture(config.position_tex);

    model_m_ref = config.model_m_ref;
    SetDrawMode(dmode);
}

object_loader::Buffers Object::getBuffers(){
    object_loader::Buffers bufs = {
        .ObjectVAO = ObjectVAO,
        .ObjectVBO = ObjectVBO,
        .ObjectEBO = ObjectEBO,
        .ObjectABO = ObjectABO,
        .ObjectEBSize = element_buffer_size 
    };

    return bufs;
}

void Object::setInstancePosBuffer(const unsigned int &buffer, const unsigned int &amount){

        ObjectABO = buffer;
        amount_i = amount;
        
        glBindVertexArray(ObjectVAO);

        glBindBuffer(GL_ARRAY_BUFFER, ObjectABO);

        // set attribute pointers for matrix (4 times vec4)

        glEnableVertexAttribArray(8);
        glVertexAttribPointer(8, 4, GL_FLOAT, GL_FALSE, sizeof(glm::mat4), (void*)0);
        glEnableVertexAttribArray(9);
        glVertexAttribPointer(9, 4, GL_FLOAT, GL_FALSE, sizeof(glm::mat4), (void*)(sizeof(glm::vec4)));
        glEnableVertexAttribArray(10);
        glVertexAttribPointer(10, 4, GL_FLOAT, GL_FALSE, sizeof(glm::mat4), (void*)(2 * sizeof(glm::vec4)));
        glEnableVertexAttribArray(11);
        glVertexAttribPointer(11, 4, GL_FLOAT, GL_FALSE, sizeof(glm::mat4), (void*)(3 * sizeof(glm::vec4)));

        glVertexAttribDivisor(8, 1);
        glVertexAttribDivisor(9, 1);
        glVertexAttribDivisor(10, 1);
        glVertexAttribDivisor(11, 1);
        
        /*
        glEnableVertexAttribArray(7);
        glVertexAttribPointer(7, 4, GL_FLOAT, GL_FALSE, sizeof(glm::mat4), (void*)0);
        glEnableVertexAttribArray(8);
        glVertexAttribPointer(8, 4, GL_FLOAT, GL_FALSE, sizeof(glm::mat4), (void*)(sizeof(glm::vec4)));
        glEnableVertexAttribArray(9);
        glVertexAttribPointer(9, 4, GL_FLOAT, GL_FALSE, sizeof(glm::mat4), (void*)(2 * sizeof(glm::vec4)));
        glEnableVertexAttribArray(10);
        glVertexAttribPointer(10, 4, GL_FLOAT, GL_FALSE, sizeof(glm::mat4), (void*)(3 * sizeof(glm::vec4)));

        glVertexAttribDivisor(7, 1);
        glVertexAttribDivisor(8, 1);
        glVertexAttribDivisor(9, 1);
        glVertexAttribDivisor(10, 1);
        */

        glBindVertexArray(0);

}

void Object::SetDrawMode(unsigned int mode){

    switch(mode){
        case (DRAW_UNTEXTURED) :
            Draw_method = &Object::draw_untextured;
            break;
        case (DRAW_TEXTURED) :
            Draw_method = &Object::draw_textured;
            break;
        case (DRAW_UNTEXTURED_INSTANCED) :
            Draw_method = &Object::draw_untextured_instanced;
            break;
        case (DRAW_TEXTURED_INSTANCED) :
            Draw_method = &Object::draw_textured_instanced;
            break;
        case (DRAW_MODEL) :
            Draw_method = &Object::draw_model;
            break;
        case (DRAW_MODEL_INSTANCED) :
            Draw_method = &Object::draw_model_instanced;
            break;
        default:
            Draw_method = &Object::draw_untextured;
            break;

    }

}

void Object::SetDrawInfoMode(unsigned int mode){

    switch(mode){
        case (DRAW_INFO) :
            Draw_method_info = &Object::draw_info;
            break;
        case (DRAW_MODEL_INFO) :
            Draw_method_info = &Object::draw_model_info;
            break;
        default:
            Draw_method_info = &Object::draw_info;
            break;

    }

}

void Object::SetDrawOutlineMode(unsigned int mode){

    switch(mode){
        case (DRAW_OUTLINE) :
            Draw_method_outline = &Object::draw_outline;
            break;
        case (DRAW_MODEL_OUTLINE) :
            Draw_method_outline = &Object::draw_model_outline;
            break;
        default:
            Draw_method_outline = &Object::draw_outline;
            break;

    }

}

void Object::draw_untextured()
{ 
    localShader.use();

    glBindVertexArray(ObjectVAO);
    glDrawElements(GL_TRIANGLES, element_buffer_size, GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
}

void Object::draw_textured()
{
    localShader.use();

    glActiveTexture(GL_TEXTURE0);
    glBindVertexArray(ObjectVAO);
    glBindTexture(GL_TEXTURE_2D, Texture);
    //glDrawElements(GL_TRIANGLE_STRIP, element_buffer_size, GL_UNSIGNED_INT, 0);
    glDrawElements(GL_TRIANGLES, element_buffer_size, GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
}
void Object::draw_untextured_instanced()
{ 

    localShader.use();
    glBindVertexArray(ObjectVAO);
    glDrawElementsInstanced(GL_TRIANGLES, element_buffer_size, GL_UNSIGNED_INT, 0, amount_i);
    glBindVertexArray(0);
  

}
void Object::draw_textured_instanced()
{

    localShader.use();
    glActiveTexture(GL_TEXTURE0);
    glBindVertexArray(ObjectVAO);
    glBindTexture(GL_TEXTURE_2D, Texture);
    glDrawElementsInstanced(GL_TRIANGLES, element_buffer_size, GL_UNSIGNED_INT, 0, amount_i);
    glBindVertexArray(0);

}

void Object::draw_model()
{ 
    localShader.use();

    localModel->Draw(localShader);
}

void Object::draw_model_instanced()
{ 
    localShader.use();

    localModel->DrawInstanced(localShader,amount_i);
}

void Object::draw_info(const unsigned int &index)
{ 
    localShader_info->use();
    localShader_info->setUint("gObjectIndex",index);
    localShader_info->setUint("gDrawIndex",index);
    glBindVertexArray(ObjectVAO);
    glDrawElements(GL_TRIANGLES, element_buffer_size, GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
}

void Object::draw_model_info(const unsigned int &index)
{ 
    localShader_info->use();
    localShader_info->setUint("gObjectIndex",index);
    localShader_info->setUint("gDrawIndex",index);
    localModel->Draw_info(localShader_info);
}

void Object::draw_outline()
{ 
    localShader_outline->use();

    glBindVertexArray(ObjectVAO);
    glDrawElements(GL_TRIANGLES, element_buffer_size, GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);

}

void Object::draw_model_outline()
{ 
    localShader_outline->use();
    localModel->Draw_info(localShader_outline);
}

void Object::Draw(){
    (this->*Draw_method)();
}

void Object::DrawInfo(const unsigned int &index){
    (this->*Draw_method_info)(index);
}

void Object::DrawOutline(){
    (this->*Draw_method_outline)();
}

void Object::SetTexture(unsigned int TexIndex){
    localShader.use();
    localShader.setInt("texture_diffuse1", 0);
    Texture = TexIndex;

}

void Object::SetTransform(){

    localShader.use();
    localShader.setMat4("model", model_m);


}


void Object::SetProjection(const glm::mat4 &projection){

    localShader.use();
    localShader.setMat4("projection", projection);


}

void Object::SetView(const glm::mat4 &view){

    localShader.use();
    localShader.setMat4("view", view);


}

void Object::SetInfoProjection(const glm::mat4 &projection){

    localShader_info->use();
    localShader_info->setMat4("projection", projection);


}

void Object::SetOutlineProjection(const glm::mat4 &projection){

    localShader_outline->use();
    localShader_outline->setMat4("projection", projection);


}

void Object::SetInfoView(const glm::mat4 &view){

    localShader_info->use();
    localShader_info->setMat4("view", view);


}


void Object::SetOutlineView(const glm::mat4 &view){

    localShader_outline->use();
    localShader_outline->setMat4("view", view);


}

void Object::SetTextureTransform(){

    localShader.use();
    localShader.setMat3("model_tex", model_m_tex);


}

void Object::SetInfoTransform(){

    localShader_info->use();
    localShader_info->setMat4("model", model_m);


}

void Object::SetOutlineTransform(){

    localShader_outline->use();
    localShader_outline->setMat4("model", model_m);


}

void Object::SetTextureInfoTransform(){

    localShader_info->use();
    localShader_info->setMat3("model_tex", model_m_tex);


}

void Object::SetInfoShader(Shader *info_shader){

    localShader_info = info_shader;

}

void Object::SetOutlineShader(Shader *outline_shader){

    localShader_outline = outline_shader;

}

void Object::SetViewProjUBuffer(unsigned int biding){
    
    localShader.use();
    unsigned int uniformBlockIndexObject = glGetUniformBlockIndex(localShader.ID, "Matrices_cam");

    // then we link each shader's uniform block to this uniform binding point
    glUniformBlockBinding(localShader.ID, uniformBlockIndexObject, biding);
}

//void Object::SetViewProjUBuffer(unsigned int ubuffer, unsigned int offset, unsigned int biding){
//    
//    glBindBuffer(GL_UNIFORM_BUFFER, ubuffer);
//    glBufferSubData(GL_UNIFORM_BUFFER, offset,sizeof(glm::mat4), glm::value_ptr(model_m));
//    glBindBuffer(GL_UNIFORM_BUFFER, 0);
//    
//    unsigned int uniformBlockIndexPlane = glGetUniformBlockIndex(localShader.ID, "Matrixes_vertex");
//
//    // then we link each shader's uniform block to this uniform binding point
//    glUniformBlockBinding(localShader.ID, uniformBlockIndexPlane, biding);
//}


//void Object::SetViewProjUBuffer(unsigned int ubuffer, unsigned int offset){
//
//    glBindBuffer(GL_UNIFORM_BUFFER, ubuffer);
//    glBufferSubData(GL_UNIFORM_BUFFER, offset,sizeof(glm::mat4), glm::value_ptr(model_m));
//    glBindBuffer(GL_UNIFORM_BUFFER, 0);
//}


void Object::ScaleTexture(const glm::vec2 &scale_vector){

    glm::vec2 scale_temp = scale_vector;
    scale_temp.x = std::max(0.0001f,scale_temp.x);
    scale_temp.y = std::max(0.0001f,scale_temp.y);

    scale_tex = scale_temp;
    

    local_scale_matrix_tex = glm::scale(glm::mat3(1.0),scale_temp);
    model_m_tex = local_pos_matrix_tex*local_rot_matrix_tex*local_scale_matrix_tex*model_m_tex_ref;

}

void Object::RotateTexture(const float &angle){

    rotation_tex = angle;

    local_rot_matrix_tex = glm::rotate(glm::mat3(1.0),angle);


    model_m_tex = local_pos_matrix_tex*local_rot_matrix_tex*local_scale_matrix_tex*model_m_tex_ref;
}

void Object::PositionateTexture(const glm::vec2 &position_new){

    position_tex = position_new;

    local_pos_matrix_tex = glm::translate(glm::mat3(1.0),position_new);

    model_m_tex = local_pos_matrix_tex*local_rot_matrix_tex*local_scale_matrix_tex*model_m_tex_ref;

}


void Object::GlobalScale(const glm::vec3 &scale_vector){


    local_scale_matrix = glm::transpose(local_rot_matrix)*
    glm::scale(glm::mat4(1.0),scale_vector)*local_rot_matrix*
    local_scale_matrix;

    scale.x = local_scale_matrix[0][0];
    scale.y = local_scale_matrix[1][1];
    scale.z = local_scale_matrix[2][2];

    model_m = local_pos_matrix*local_rot_matrix*local_scale_matrix*model_m_ref;

}

void Object::GlobalRotate(const float roll, const float pitch, const float yaw){


    glm::mat4 rot_temp = glm::rotate(glm::mat4(1.0),yaw,glm::vec3(0.0,1.0,0.0));
    rot_temp = glm::rotate(rot_temp,pitch,glm::vec3(1.0,0.0,0.0));
    rot_temp = glm::rotate(rot_temp,roll,glm::vec3(0.0,0.0,1.0));


    //glm::mat4 rot_temp = glm::rotate(glm::mat4(1.0),roll,glm::vec3(0.0,0.0,1.0));
    //rot_temp = glm::rotate(rot_temp,pitch,glm::vec3(1.0,0.0,0.0));
    //rot_temp = glm::rotate(rot_temp,yaw,glm::vec3(0.0,1.0,0.0));

    local_rot_matrix = rot_temp*local_rot_matrix;

    model_m = local_pos_matrix*local_rot_matrix*local_scale_matrix*model_m_ref;
    
    //extract rpy
    float yaw_t, pitch_t, roll_t;

    glm::extractEulerAngleYXZ(local_rot_matrix, yaw_t, pitch_t, roll_t);

    rotation = glm::vec3(roll_t,pitch_t,yaw_t);

}


void Object::Scale(const glm::vec3 &scale_vector){

    glm::vec3 scale_temp = scale_vector;
    scale_temp.x = std::max(0.0001f,scale_temp.x);
    scale_temp.y = std::max(0.0001f,scale_temp.y);
    scale_temp.z = std::max(0.0001f,scale_temp.z);

    scale *= scale_temp;
    

    local_scale_matrix = glm::scale(glm::mat4(1.0f),scale_temp)*local_scale_matrix;
    model_m = local_pos_matrix*local_rot_matrix*local_scale_matrix*model_m_ref;

}

void Object::Scale(glm::mat4 &delta_scale){

    glm::vec3 scale_temp; 

    scale_temp.x= glm::length(glm::vec3(delta_scale[0]));
    scale_temp.y= glm::length(glm::vec3(delta_scale[1]));
    scale_temp.z= glm::length(glm::vec3(delta_scale[2]));

    scale_temp.x = std::max(0.0001f,scale_temp.x);
    scale_temp.y = std::max(0.0001f,scale_temp.y);
    scale_temp.z = std::max(0.0001f,scale_temp.z);

    scale *= scale_temp;
    

    local_scale_matrix = glm::scale(glm::mat4(1.0f),scale_temp)*local_scale_matrix;
    model_m = local_pos_matrix*local_rot_matrix*local_scale_matrix*model_m_ref;

}

void Object::Rotate(float angle,const glm::vec3 &rot_vector){

    local_rot_matrix = glm::rotate(glm::mat4(1.0f), angle, rot_vector)*local_rot_matrix;

    model_m = local_pos_matrix*local_rot_matrix*local_scale_matrix*model_m_ref;
    
    //extract rpy
    float yaw, pitch, roll;

    glm::extractEulerAngleYXZ(local_rot_matrix, yaw, pitch, roll);

    rotation = glm::vec3(roll,pitch,yaw);
}

void Object::Rotate(const float roll, const float pitch, const float yaw){

    rotation += glm::vec3(roll,pitch,yaw);

    local_rot_matrix = glm::rotate(glm::mat4(1.0),roll,glm::vec3(0.0,0.0,1.0))*local_rot_matrix;
    local_rot_matrix = glm::rotate(glm::mat4(1.0),pitch,glm::vec3(1.0,0.0,0.0))*local_rot_matrix;
    local_rot_matrix = glm::rotate(glm::mat4(1.0),yaw,glm::vec3(0.0,1.0,0.0))*local_rot_matrix;
    

    model_m = local_pos_matrix*local_rot_matrix*local_scale_matrix*model_m_ref;

}

void Object::Rotate(const glm::vec3 rot){

    rotation += rot;

    local_rot_matrix = glm::rotate(glm::mat4(1.0),rot.x,glm::vec3(0.0,0.0,1.0))*local_rot_matrix;
    local_rot_matrix = glm::rotate(glm::mat4(1.0),rot.y,glm::vec3(1.0,0.0,0.0))*local_rot_matrix;
    local_rot_matrix = glm::rotate(glm::mat4(1.0),rot.z,glm::vec3(0.0,1.0,0.0))*local_rot_matrix;

    model_m = local_pos_matrix*local_rot_matrix*local_scale_matrix*model_m_ref;

}

void Object::Rotate(glm::mat4 &delta_rot){

    delta_rot[3] = glm::vec4(0.0f,0.0f,0.0f,1.0f);

    local_rot_matrix = delta_rot*local_rot_matrix;

    model_m = local_pos_matrix*local_rot_matrix*local_scale_matrix*model_m_ref;
    
    //extract rpy
    float yaw, pitch, roll;

    glm::extractEulerAngleYXZ(local_rot_matrix, yaw, pitch, roll);

    rotation = glm::vec3(roll,pitch,yaw);
}

void Object::Tanslate(const glm::vec3 &translation){
    
    position += translation;

    local_pos_matrix = glm::translate(glm::mat4(1.0f),translation)*local_pos_matrix;
    model_m = local_pos_matrix*local_rot_matrix*local_scale_matrix*model_m_ref;


}

void Object::ApplyAllTransfoms(){
    
    model_m_ref = model_m;

    local_pos_matrix = local_rot_matrix = local_scale_matrix = glm::mat4(1.0);

    scale = glm::vec3(1.0);
    position = rotation = glm::vec3(0.0);

}


void Object::ApplyRotation(){
    
    model_m_ref = local_rot_matrix*local_scale_matrix*model_m_ref;

    model_m_ref = glm::scale(glm::mat4(1.0f),glm::vec3(1.0f/scale.x,1.0f/scale.y,1.0f/scale.z))*model_m_ref;

    local_rot_matrix = glm::mat4(1.0);

    rotation = glm::vec3(0.0);

}

void Object::ApplyTranslation(){
    
    model_m_ref = model_m;
    
    model_m_ref = glm::transpose(local_rot_matrix)*model_m_ref;

    model_m_ref = glm::scale(glm::mat4(1.0f),glm::vec3(1.0f/scale.x,1.0f/scale.y,1.0f/scale.z))*model_m_ref;

    local_pos_matrix = glm::mat4(1.0);

    position = glm::vec3(0.0);

}

void Object::ApplyScale(){
    
    model_m_ref = local_scale_matrix*model_m_ref;

    local_scale_matrix = glm::mat4(1.0);

    scale = glm::vec3(1.0);

}

void Object::ApplyRotationTexture(){


    model_m_tex_ref = local_rot_matrix_tex*local_scale_matrix_tex*model_m_tex_ref;

    model_m_tex_ref = glm::scale(glm::mat3(1.0f),glm::vec2(1.0f/scale_tex.x,1.0f/scale_tex.y))*model_m_tex_ref;

    local_rot_matrix_tex = glm::mat3(1.0);

    rotation_tex = 0.0;
}


glm::vec3 Object::Get_rotation(){
    return rotation;
}

glm::vec3 Object::Get_position(){
    return position;
}

glm::vec3 Object::Get_scale(){
    return scale;
}

glm::mat4 Object::Get_model_matrix(){
    return model_m;
}

glm::mat4 Object::Get_reference_model_matrix(){
    return model_m_ref;
}

void Object::Change_rotation(const glm::vec3 &rotation_new){

    rotation = rotation_new;

    local_rot_matrix = glm::rotate(glm::mat4(1.0),rotation_new.x,glm::vec3(0.0,0.0,1.0));
    local_rot_matrix = glm::rotate(glm::mat4(1.0),rotation_new.y,glm::vec3(1.0,0.0,0.0))*local_rot_matrix;
    local_rot_matrix = glm::rotate(glm::mat4(1.0),rotation_new.z,glm::vec3(0.0,1.0,0.0))*local_rot_matrix;
 
    model_m = local_pos_matrix*local_rot_matrix*local_scale_matrix*model_m_ref;

}

void Object::Change_position(const glm::vec3 &position_new){

    position = position_new;

    local_pos_matrix = glm::translate(glm::mat4(1.0),position);
    model_m = local_pos_matrix*local_rot_matrix*local_scale_matrix*model_m_ref;

}

void Object::Change_scale(const glm::vec3 &scale_vector){

    glm::vec3 scale_temp = scale_vector;
    scale_temp.x = std::max(0.0001f,scale_temp.x);
    scale_temp.y = std::max(0.0001f,scale_temp.y);
    scale_temp.z = std::max(0.0001f,scale_temp.z);

    scale = scale_temp;
    

    local_scale_matrix = glm::scale(glm::mat4(1.0),scale_temp);
    model_m = local_pos_matrix*local_rot_matrix*local_scale_matrix*model_m_ref;

}

void Object::Change_reference_model_matrix(const glm::mat4 &new_model){
    model_m_ref = new_model;
}

void Object::Reset_reference_model_matrix(){
    model_m_ref = glm::mat4(1.0f);
}


void Object::Reset_reference_model_texture_matrix(){
    model_m_tex_ref = glm::mat3(1.0f);
}



class plane : public Object
{
    public:

    plane(Shader &dshader, unsigned int dmode = DRAW_UNTEXTURED);
    plane(const glm::vec3 &pos, Shader &dshader, unsigned int dmode = DRAW_UNTEXTURED);
    plane(const glm::vec3 &pos, Shader &dshader, unsigned int dmode, bool vertical);
    plane(const glm::vec3 &pos, const glm::vec3 &rot, Shader &dshader, unsigned int dmode = DRAW_UNTEXTURED);
    plane(const glm::vec3 &pos, const glm::vec3 &rot, const glm::vec3 &scale, Shader &dshader, unsigned int dmode = DRAW_UNTEXTURED);
    plane(const object_config_t &config, unsigned int dmode = DRAW_UNTEXTURED);

    bool back_face_culling = true;

    void Init(/* args */) override;
    void InitVertical();
};

plane::plane(Shader &dshader, unsigned int dmode): Object(dshader,dmode)
{
    object_type = ObjTypes::Object_types::Plane;
    element_buffer_size = sizeof(Plane::Indices);
    Init();
}

plane::plane(const glm::vec3 &pos, Shader &dshader, unsigned int dmode):
Object(pos, dshader, dmode)
{
    object_type = ObjTypes::Object_types::Plane;
    element_buffer_size = sizeof(Plane::Indices);
    Init();
}

plane::plane(const glm::vec3 &pos, Shader &dshader, unsigned int dmode, bool vertical):
Object(pos, dshader, dmode)
{
    object_type = ObjTypes::Object_types::Plane;
    element_buffer_size = sizeof(Plane::Indices);
    InitVertical();
}

plane::plane(const glm::vec3 &pos, const glm::vec3 &rot, Shader &dshader, unsigned int dmode):
Object(pos, rot, dshader, dmode)
{
    object_type = ObjTypes::Object_types::Plane;
    element_buffer_size = sizeof(Plane::Indices);
    Init();
}

plane::plane(const glm::vec3 &pos, const glm::vec3 &rot, const glm::vec3 &scale, Shader &dshader, unsigned int dmode):
Object(pos, rot, scale, dshader, dmode)
{
    object_type = ObjTypes::Object_types::Plane;
    element_buffer_size = sizeof(Plane::Indices);
    Init();
}

plane::plane(const object_config_t &config, unsigned int dmode):
Object(config, dmode)
{
    object_type = ObjTypes::Object_types::Plane;
    element_buffer_size = sizeof(Plane::Indices);
    Init();
}

void plane::Init()
{
    glGenVertexArrays(1, &ObjectVAO);
    glGenBuffers(1, &ObjectVBO);
    glGenBuffers(1, &ObjectEBO);
    glBindVertexArray(ObjectVAO);
    glBindBuffer(GL_ARRAY_BUFFER, ObjectVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(Plane::Vertices), &Plane::Vertices, GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ObjectEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(Plane::Indices), &Plane::Indices, GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
    //glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

void plane::InitVertical()
{
    glGenVertexArrays(1, &ObjectVAO);
    glGenBuffers(1, &ObjectVBO);
    glGenBuffers(1, &ObjectEBO);
    glBindVertexArray(ObjectVAO);
    glBindBuffer(GL_ARRAY_BUFFER, ObjectVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(Plane::Vertices_vertical), &Plane::Vertices_vertical, GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ObjectEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(Plane::Indices), &Plane::Indices, GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
    //glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

class sphere : public Object
{
    public:

    sphere(Shader &dshader, const float &Radius, unsigned int dmode = DRAW_UNTEXTURED);
    sphere(const glm::vec3 &pos, Shader &dshader, const float &Radius, unsigned int dmode = DRAW_UNTEXTURED);
    sphere(const glm::vec3 &pos, const glm::vec3 &rot, Shader &dshader, const float &Radius, unsigned int dmode = DRAW_UNTEXTURED);
    sphere(const glm::vec3 &pos, const glm::vec3 &rot, const glm::vec3 &scale, Shader &dshader, const float &Radius, unsigned int dmode = DRAW_UNTEXTURED);
    sphere(const object_config_t &config, const float &Radius, unsigned int dmode = DRAW_UNTEXTURED);

    void Init(/* args */) override;

    float getRadius();

    private:

    float radius;
};

sphere::sphere(Shader &dshader, const float &Radius, unsigned int dmode): Object(dshader,dmode)
{
    object_type = ObjTypes::Object_types::Sphere;
    radius = Radius;
    Init();
}

sphere::sphere(const glm::vec3 &pos, Shader &dshader, const float &Radius, unsigned int dmode):
Object(pos, dshader, dmode)
{
    object_type = ObjTypes::Object_types::Sphere;
    radius = Radius;
    Init();
}

sphere::sphere(const glm::vec3 &pos, const glm::vec3 &rot, Shader &dshader, const float &Radius, unsigned int dmode):
Object(pos, rot, dshader, dmode)
{
    object_type = ObjTypes::Object_types::Sphere;
    radius = Radius;
    Init();
}

sphere::sphere(const glm::vec3 &pos, const glm::vec3 &rot, const glm::vec3 &scale, Shader &dshader, const float &Radius, unsigned int dmode):
Object(pos, rot, scale, dshader, dmode)
{
    object_type = ObjTypes::Object_types::Sphere;
    radius = Radius;
    Init();
}

sphere::sphere(const object_config_t &config, const float &Radius, unsigned int dmode):
Object(config, dmode)
{
    object_type = ObjTypes::Object_types::Sphere;
    radius = Radius;
    Init();
}

float sphere::getRadius(){
    return radius;
}

void sphere::Init()
{
    glGenVertexArrays(1, &ObjectVAO);
    glGenBuffers(1, &ObjectVBO);
    glGenBuffers(1, &ObjectEBO);

    std::vector<glm::vec3> positions;
    std::vector<glm::vec2> uv;
    std::vector<glm::vec3> normals;
    std::vector<unsigned int> indices;

    const unsigned int X_SEGMENTS = 64;
    const unsigned int Y_SEGMENTS = 64;
    const float PI = 3.14159265359f;
    for (unsigned int x = 0; x <= X_SEGMENTS; ++x)
    {
        for (unsigned int y = 0; y <= Y_SEGMENTS; ++y)
        {
            float xSegment = (float)x / (float)X_SEGMENTS;
            float ySegment = (float)y / (float)Y_SEGMENTS;
            float xPos = std::cos(xSegment * 2.0f * PI) * std::sin(ySegment * PI);
            float yPos = std::cos(ySegment * PI);
            float zPos = std::sin(xSegment * 2.0f * PI) * std::sin(ySegment * PI);

            positions.push_back(glm::vec3(xPos, yPos, zPos));
            uv.push_back(glm::vec2(xSegment, ySegment));
            normals.push_back(glm::vec3(xPos, yPos, zPos));
        }
    }

    bool oddRow = false;
    for (unsigned int y = 0; y < Y_SEGMENTS; ++y)
    {
        if (!oddRow) // even rows: y == 0, y == 2; and so on
        {
            for (unsigned int x = 0; x <= X_SEGMENTS; ++x)
            {
                indices.push_back(y * (X_SEGMENTS + 1) + x);
                indices.push_back((y + 1) * (X_SEGMENTS + 1) + x);
            }
        }
        else
        {
            for (int x = X_SEGMENTS; x >= 0; --x)
            {
                indices.push_back((y + 1) * (X_SEGMENTS + 1) + x);
                indices.push_back(y * (X_SEGMENTS + 1) + x);
            }
        }
        oddRow = !oddRow;
    }
    element_buffer_size = static_cast<unsigned int>(indices.size());

    std::vector<float> data;
    for (unsigned int i = 0; i < positions.size(); ++i)
    {
        data.push_back(positions[i].x);
        data.push_back(positions[i].y);
        data.push_back(positions[i].z);
        if (normals.size() > 0)
        {
            data.push_back(normals[i].x);
            data.push_back(normals[i].y);
            data.push_back(normals[i].z);
        }
        if (uv.size() > 0)
        {
            data.push_back(uv[i].x);
            data.push_back(uv[i].y);
        }
    }


    glBindVertexArray(ObjectVAO);
    glBindBuffer(GL_ARRAY_BUFFER, ObjectVBO);
    glBufferData(GL_ARRAY_BUFFER, data.size() * sizeof(float), &data[0], GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ObjectEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), &indices[0], GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
    //glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}





// renders (and builds at first invocation) a sphere
// -------------------------------------------------

void renderSphere(unsigned int &sphereVAO, unsigned int &indexCount)
{   

    if (sphereVAO == 0)
    {
        glGenVertexArrays(1, &sphereVAO);

        unsigned int vbo, ebo;
        glGenBuffers(1, &vbo);
        glGenBuffers(1, &ebo);

        std::vector<glm::vec3> positions;
        std::vector<glm::vec2> uv;
        std::vector<glm::vec3> normals;
        std::vector<unsigned int> indices;

        const unsigned int X_SEGMENTS = 64;
        const unsigned int Y_SEGMENTS = 64;
        const float PI = 3.14159265359f;
        for (unsigned int x = 0; x <= X_SEGMENTS; ++x)
        {
            for (unsigned int y = 0; y <= Y_SEGMENTS; ++y)
            {
                float xSegment = (float)x / (float)X_SEGMENTS;
                float ySegment = (float)y / (float)Y_SEGMENTS;
                float xPos = std::cos(xSegment * 2.0f * PI) * std::sin(ySegment * PI);
                float yPos = std::cos(ySegment * PI);
                float zPos = std::sin(xSegment * 2.0f * PI) * std::sin(ySegment * PI);

                positions.push_back(glm::vec3(xPos, yPos, zPos));
                uv.push_back(glm::vec2(xSegment, ySegment));
                normals.push_back(glm::vec3(xPos, yPos, zPos));
            }
        }

        bool oddRow = false;
        for (unsigned int y = 0; y < Y_SEGMENTS; ++y)
        {
            if (!oddRow) // even rows: y == 0, y == 2; and so on
            {
                for (unsigned int x = 0; x <= X_SEGMENTS; ++x)
                {
                    indices.push_back(y * (X_SEGMENTS + 1) + x);
                    indices.push_back((y + 1) * (X_SEGMENTS + 1) + x);
                }
            }
            else
            {
                for (int x = X_SEGMENTS; x >= 0; --x)
                {
                    indices.push_back((y + 1) * (X_SEGMENTS + 1) + x);
                    indices.push_back(y * (X_SEGMENTS + 1) + x);
                }
            }
            oddRow = !oddRow;
        }
        indexCount = static_cast<unsigned int>(indices.size());

        std::vector<float> data;
        for (unsigned int i = 0; i < positions.size(); ++i)
        {
            data.push_back(positions[i].x);
            data.push_back(positions[i].y);
            data.push_back(positions[i].z);
            if (normals.size() > 0)
            {
                data.push_back(normals[i].x);
                data.push_back(normals[i].y);
                data.push_back(normals[i].z);
            }
            if (uv.size() > 0)
            {
                data.push_back(uv[i].x);
                data.push_back(uv[i].y);
            }
        }
        glBindVertexArray(sphereVAO);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, data.size() * sizeof(float), &data[0], GL_STATIC_DRAW);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), &indices[0], GL_STATIC_DRAW);
        unsigned int stride = (3 + 2 + 3) * sizeof(float);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, (void*)0);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride, (void*)(3 * sizeof(float)));
        glEnableVertexAttribArray(2);
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, stride, (void*)(6 * sizeof(float)));
    }

    glBindVertexArray(sphereVAO);
    glDrawElements(GL_TRIANGLE_STRIP, indexCount, GL_UNSIGNED_INT, 0);
}


class cube : public Object
{
    public:

    cube(Shader &dshader, unsigned int dmode = DRAW_UNTEXTURED);
    cube(const glm::vec3 &pos, Shader &dshader, unsigned int dmode = DRAW_UNTEXTURED);
    cube(const glm::vec3 &pos, const glm::vec3 &rot, Shader &dshader, unsigned int dmode = DRAW_UNTEXTURED);
    cube(const glm::vec3 &pos, const glm::vec3 &rot, const glm::vec3 &scale, Shader &dshader, unsigned int dmode = DRAW_UNTEXTURED);
    cube(const object_config_t &config, unsigned int dmode = DRAW_UNTEXTURED);

    void Init(/* args */) override;
};

cube::cube(Shader &dshader, unsigned int dmode): Object(dshader,dmode)
{
    object_type = ObjTypes::Object_types::Cube;
    element_buffer_size = sizeof(Cube::Indices);
    Init();
}

cube::cube(const glm::vec3 &pos, Shader &dshader, unsigned int dmode):
Object(pos, dshader, dmode)
{
    object_type = ObjTypes::Object_types::Cube;
    element_buffer_size = sizeof(Cube::Indices);
    Init();
}

cube::cube(const glm::vec3 &pos, const glm::vec3 &rot, Shader &dshader, unsigned int dmode):
Object(pos, rot, dshader, dmode)
{
    object_type = ObjTypes::Object_types::Cube;
    element_buffer_size = sizeof(Cube::Indices);
    Init();
}

cube::cube(const glm::vec3 &pos, const glm::vec3 &rot, const glm::vec3 &scale, Shader &dshader, unsigned int dmode):
Object(pos, rot, scale, dshader, dmode)
{
    object_type = ObjTypes::Object_types::Cube;
    element_buffer_size = sizeof(Cube::Indices);
    Init();
}

cube::cube(const object_config_t &config, unsigned int dmode):
Object(config, dmode)
{
    object_type = ObjTypes::Object_types::Cube;
    element_buffer_size = sizeof(Cube::Indices);
    Init();
}

void cube::Init()
{
    glGenVertexArrays(1, &ObjectVAO);
    glGenBuffers(1, &ObjectVBO);
    glGenBuffers(1, &ObjectEBO);
    glBindVertexArray(ObjectVAO);
    glBindBuffer(GL_ARRAY_BUFFER, ObjectVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(Cube::Vertices), &Cube::Vertices, GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ObjectEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(Cube::Indices), &Cube::Indices, GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
    //glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

class object_model : public Object
{
    public:

    object_model(Shader &dshader, Model &dmodel, unsigned int dmode = DRAW_MODEL);
    object_model(const glm::vec3 &pos, Shader &dshader, Model &dmodel, unsigned int dmode = DRAW_MODEL);
    object_model(const glm::vec3 &pos, const glm::vec3 &rot, Shader &dshader, Model &dmodel, unsigned int dmode = DRAW_MODEL);
    object_model(const glm::vec3 &pos, const glm::vec3 &rot, const glm::vec3 &scale, Shader &dshader, Model &dmodel, unsigned int dmode = DRAW_MODEL);
    object_model(const object_config_t &config, Model &dmodel, unsigned int dmode = DRAW_MODEL);

    void Init() override;

};

object_model::object_model(Shader &dshader, Model &dmodel, unsigned int dmode): Object(dshader,dmode)
{
    object_type = ObjTypes::Object_types::Mesh;
    ObjectVAO = 0;
    ObjectVBO = 0;
    ObjectEBO = 0;
    element_buffer_size = 0;
    localModel = &dmodel;
}

object_model::object_model(const glm::vec3 &pos, Shader &dshader, Model &dmodel, unsigned int dmode):
Object(pos, dshader, dmode)
{
    object_type = ObjTypes::Object_types::Mesh;
    ObjectVAO = 0;
    ObjectVBO = 0;
    ObjectEBO = 0;
    element_buffer_size = 0;
    localModel = &dmodel;
}

object_model::object_model(const glm::vec3 &pos, const glm::vec3 &rot, Shader &dshader, Model &dmodel, unsigned int dmode):
Object(pos, rot, dshader, dmode)
{
    object_type = ObjTypes::Object_types::Mesh;
    ObjectVAO = 0;
    ObjectVBO = 0;
    ObjectEBO = 0;
    element_buffer_size = 0;
    localModel = &dmodel;
}

object_model::object_model(const glm::vec3 &pos, const glm::vec3 &rot, const glm::vec3 &scale, Shader &dshader, Model &dmodel, unsigned int dmode):
Object(pos, rot, scale, dshader, dmode)
{
    object_type = ObjTypes::Object_types::Mesh;
    ObjectVAO = 0;
    ObjectVBO = 0;
    ObjectEBO = 0;
    element_buffer_size = 0;
    localModel = &dmodel;
}

object_model::object_model(const object_config_t &config, Model &dmodel, unsigned int dmode):
Object(config, dmode)
{
    object_type = ObjTypes::Object_types::Mesh;
    ObjectVAO = 0;
    ObjectVBO = 0;
    ObjectEBO = 0;
    element_buffer_size = 0;
    localModel = &dmodel;
}

void object_model::Init()
{

}

