#version 460 core

layout (location = 0) in vec3 aPos;
layout (location = 2) in vec2 aTexCoords;

out vec2 TexCoords;

uniform mat4 model;
uniform mat3 model_tex;
//uniform mat4 view;
//uniform mat4 projection;

uniform Matrices_cam
{
    mat4 view;
    mat4 projection;
};

void main()
{
    TexCoords = (model_tex*vec3(aTexCoords,1.0)).xy;    
    gl_Position = projection * view * model * vec4(aPos, 1.0);
}