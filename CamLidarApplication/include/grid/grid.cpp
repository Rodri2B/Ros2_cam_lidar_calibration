#include "grid.hpp"

custom_grid::grid::grid(std::vector<unsigned int> &global_binding_vector, Shader &grid_sh, const float &grid_size_p, const float &GridCellSize_p,const glm::vec4 &GridColorThick_p,const glm::vec4 &GridColorThin_p, float GridMinPixelsBetweenCells_p):
localShader(grid_sh), local_binding_vector(global_binding_vector)
{

    GridSize = grid_size_p;
    GridCellSize = GridCellSize_p;
    GridMinPixelsBetweenCells = GridMinPixelsBetweenCells_p;
    GridColorThick = GridColorThick_p;
    GridColorThin = GridColorThin_p;

}

custom_grid::grid::grid(std::vector<unsigned int> &global_binding_vector, Shader &grid_sh,const float &grid_size_p,const float &GridCellSize_p):
GridMinPixelsBetweenCells(2.0f), GridColorThick(glm::vec4(0.0f, 0.0f, 0.0f, 1.0f)), GridColorThin(glm::vec4(0.5f, 0.5f, 0.5f, 1.0f)),
localShader(grid_sh), local_binding_vector(global_binding_vector)
{

    GridSize = grid_size_p;
    GridCellSize = GridCellSize_p;

}

custom_grid::grid::grid(std::vector<unsigned int> &global_binding_vector, Shader &grid_sh):
GridCellSize(0.025f),
GridMinPixelsBetweenCells(2.0f), GridColorThick(glm::vec4(0.0f, 0.0f, 0.0f, 1.0f)), GridColorThin(glm::vec4(0.5f, 0.5f, 0.5f, 1.0f)),
localShader(grid_sh), local_binding_vector(global_binding_vector)
{
    GridSize = 100.0f;
}

custom_grid::grid::~grid(){
    local_binding_vector.push_back(uboParamFragmentBinding);
    local_binding_vector.push_back(uboParamVertexBinding);
}
void custom_grid::grid::init(const unsigned int &projview_binding, const unsigned int &camera_binding)
{

    // configure a uniform buffer object
    // ---------------------------------
    // first. We get the relevant block indices

    localShader.use();

    unsigned int uniformBlockIndexMatrixesVertex = glGetUniformBlockIndex(localShader.ID, "Matrixes_vertex");
    unsigned int uniformBlockIndexMatrixesFragment = glGetUniformBlockIndex(localShader.ID, "Matrixes_fragment");
    unsigned int uniformBlockIndexParamVertex = glGetUniformBlockIndex(localShader.ID, "ParamUniforms_vertex");
    unsigned int uniformBlockIndexParamFragment = glGetUniformBlockIndex(localShader.ID, "ParamUniforms_fragment");



    // then we link each shader's uniform block to this uniform binding point
    glUniformBlockBinding(localShader.ID, uniformBlockIndexMatrixesVertex, projview_binding);
    glUniformBlockBinding(localShader.ID, uniformBlockIndexMatrixesFragment, camera_binding);



    // Now actually create the buffer
    glGenBuffers(1, &uboGridUniforms);
    glBindBuffer(GL_UNIFORM_BUFFER, uboGridUniforms);
    glBufferData(GL_UNIFORM_BUFFER, UBUFFER_GRID_SIZE, NULL, GL_STATIC_DRAW);
    glBindBuffer(GL_UNIFORM_BUFFER, 0);


    updateVar();

    if (local_binding_vector.size() >= 2)
    {   
        // define the range of the buffer that links to a uniform binding point
        uboParamVertexBinding = local_binding_vector.back();
        glBindBufferRange(GL_UNIFORM_BUFFER, uboParamVertexBinding, uboGridUniforms, UBUFFER_GRID_B0_OFFSET, UBUFFER_GRID_SIZE_B0);
        glUniformBlockBinding(localShader.ID, uniformBlockIndexParamVertex, uboParamVertexBinding);
        local_binding_vector.pop_back();

        uboParamFragmentBinding = local_binding_vector.back();
        glBindBufferRange(GL_UNIFORM_BUFFER, uboParamFragmentBinding, uboGridUniforms, UBUFFER_GRID_B1_OFFSET, UBUFFER_GRID_SIZE_B1);
        glUniformBlockBinding(localShader.ID, uniformBlockIndexParamFragment, uboParamFragmentBinding);
        local_binding_vector.pop_back();
    }
    else 
        throw std::runtime_error("There is no uniform buffer biding availiable!");


    //gen grid vao
    glGenVertexArrays(1, &Grid_vao);
    glBindVertexArray(0);


}

void custom_grid::grid::updateVar(){

    glBindBuffer(GL_UNIFORM_BUFFER, uboGridUniforms);

    glBufferSubData(GL_UNIFORM_BUFFER, UBUFFER_OFFSET,sizeof(float), &GridSize);
    glBufferSubData(GL_UNIFORM_BUFFER, UBUFFER_OFFSET + sizeof(float),sizeof(float), &GridMinPixelsBetweenCells);
    glBufferSubData(GL_UNIFORM_BUFFER, UBUFFER_OFFSET + 2*sizeof(float),sizeof(float), &GridCellSize);
    glBufferSubData(GL_UNIFORM_BUFFER, UBUFFER_OFFSET + 3*sizeof(float),sizeof(glm::vec4), glm::value_ptr(GridColorThin));
    glBufferSubData(GL_UNIFORM_BUFFER, UBUFFER_OFFSET + 3*sizeof(float) + sizeof(glm::vec4),sizeof(glm::vec4), glm::value_ptr(GridColorThick));

    glBindBuffer(GL_UNIFORM_BUFFER, 0);
}

void custom_grid::grid::Draw(){

    localShader.use();
    glBindVertexArray(Grid_vao);
    glDrawArrays(GL_TRIANGLES, 0, 6);
    glBindVertexArray(0);
}

void custom_grid::grid::ChangeBidingVector(std::vector<unsigned int> &global_binding_vector){

    local_binding_vector.push_back(uboParamFragmentBinding);
    local_binding_vector.push_back(uboParamVertexBinding);

    local_binding_vector = global_binding_vector;

    if (local_binding_vector.size() >= 2)
    {   
        localShader.use();
        // define the range of the buffer that links to a uniform binding point
        uboParamVertexBinding = local_binding_vector.back();
        glBindBufferRange(GL_UNIFORM_BUFFER, uboParamVertexBinding, uboGridUniforms, UBUFFER_GRID_B0_OFFSET, UBUFFER_GRID_SIZE_B0);
        unsigned int uniformBlockIndexParamVertex = glGetUniformBlockIndex(localShader.ID, "ParamUniforms_vertex");
        glUniformBlockBinding(localShader.ID, uniformBlockIndexParamVertex, uboParamVertexBinding);
        local_binding_vector.pop_back();

        uboParamFragmentBinding = local_binding_vector.back();
        glBindBufferRange(GL_UNIFORM_BUFFER, uboParamFragmentBinding, uboGridUniforms, UBUFFER_GRID_B1_OFFSET, UBUFFER_GRID_SIZE_B1);
        unsigned int uniformBlockIndexParamFragment = glGetUniformBlockIndex(localShader.ID, "ParamUniforms_fragment");
        glUniformBlockBinding(localShader.ID, uniformBlockIndexParamFragment, uboParamFragmentBinding);
        local_binding_vector.pop_back();
    }
    else 
        throw std::runtime_error("There is no uniform buffer biding avaliable!");

}