#version 460 core
layout (location = 0) in vec3 aPos;

uniform Matrices_cam
{
    mat4 view;
    mat4 projection;
};


uniform mat4 model;

uniform float distant;

mat4 scale = mat4(1.0 + 0.004*distant*2.0, 0.0, 0.0, 0.0, 0.0, 1.0 + 0.004*distant*2.0, 0.0, 0.0, 0.0, 0.0, 1.0 + 0.004*distant*2.0, 0.0, 0.0, 0.0, 0.0, 1.0);

void main()
{   
    gl_Position = projection * view * model *scale* vec4(aPos, 1.0f);
}