#version 460 core
layout (location = 0) in vec3 aPos;

uniform mat4 lidar_model;

uniform Matrices_cam
{
    mat4 view;
    mat4 projection;
};


//float offset = - 0.01;
float offset = 0.0;



void main()
{   

    float norma = sqrt(dot(aPos,aPos));
    vec3 direction = normalize(aPos);

    vec3 offs_pos = aPos + direction*offset;

    gl_PointSize = 2.0;
    if(norma < 100){
        gl_Position = projection*view*lidar_model*vec4(offs_pos, 1.0);
    }
    else{
        gl_Position = vec4(2.0, 2.0, 2.0, 1.0); 
    }
}
