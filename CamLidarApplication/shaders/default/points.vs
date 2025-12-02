#version 460 core
layout(location = 0) in vec3 aPos;
//layout(location = 1) in vec3 aColor;

out vec3 vColor;


/////////////////////]

uniform mat4 model_lidar;

uniform Matrices_cam
{
    mat4 view;
    mat4 projection;
};

//////////////////////]

void main()
{
    //vColor = aColor;
    gl_Position = projection*view*model_lidar*vec4(aPos, 1.0);
    gl_PointSize = 10.0;  // Set point size here
}