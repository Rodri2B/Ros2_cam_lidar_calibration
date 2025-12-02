#ifndef CUSTOM_GRID_H
#define CUSTOM_GRID_H

#include <glad/glad.h>
#include <GLFW/glfw3.h> // Will drag system OpenGL headers

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <lib_opengl/shader.h>
#include <vector>
#include <stdexcept>

#define UBUFFER_GRID_SIZE (3*sizeof(float) + 2*sizeof(glm::vec4))
#define UBUFFER_GRID_SIZE_B0 sizeof(float)
#define UBUFFER_GRID_SIZE_B1 (3*sizeof(float) + 2*sizeof(glm::vec4))
#define UBUFFER_GRID_B0_OFFSET 0
#define UBUFFER_GRID_B1_OFFSET 0
#define UBUFFER_OFFSET 0

namespace custom_grid{


class grid
{

public:

grid(std::vector<unsigned int> &global_binding_vector, Shader &grid_sh, const float &grid_size_p, const float &GridCellSize_p,const glm::vec4 &GridColorThick_p,const glm::vec4 &GridColorThin_p, float GridMinPixelsBetweenCells_p = 2.0f);


grid(std::vector<unsigned int> &global_binding_vector, Shader &grid_sh,const float &grid_size_p,const float &GridCellSize_p);


grid(std::vector<unsigned int> &global_binding_vector, Shader &grid_sh);

~grid();


void init(const unsigned int &projview_binding, const unsigned int &camera_binding);
void ChangeBidingVector(std::vector<unsigned int> &global_binding_vector);
void updateVar();
void Draw();

float GridSize;
float GridCellSize;
float GridMinPixelsBetweenCells;
glm::vec4 GridColorThick;
glm::vec4 GridColorThin;

Shader &localShader;

private:

unsigned int uboGridUniforms;
unsigned int uboParamVertexBinding;
unsigned int uboParamFragmentBinding;
unsigned int Grid_vao;
std::vector<unsigned int> &local_binding_vector;

};


}

#endif