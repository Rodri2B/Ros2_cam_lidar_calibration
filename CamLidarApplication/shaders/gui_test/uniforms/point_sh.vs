#version 460 core
//layout (location = 0) in vec3 aPos;

out float normalized_ring;

struct Direction {
    vec4 direction;
};


struct Point_XYZIR {
    float x;
    float y;
    float z;
    float intensity;
    uint ring;
    uint padding[3];
};

layout (std430,binding = 0) buffer data_dir
{
    Direction directions[];
};


layout (std430, binding = 1) buffer data_point
{
    Point_XYZIR points[];
};

uniform mat4 model_lidar;

uniform Matrices_cam
{
    mat4 view;
    mat4 projection;
};


//float offset = - 0.01;
float offset = 0.0;
uniform float range; //unifom
uniform uint ring_count; //uniform



void main()
{   

    vec3 aPos = vec3(points[gl_VertexID].x, points[gl_VertexID].y, points[gl_VertexID].z);
    //vec3 aPos = directions[gl_VertexID].direction.xyz;
    //vec3 aPos = vec3(0.0,0.0,0.0);
    float norma = sqrt(dot(aPos,aPos));
    vec3 direction = normalize(aPos);

    vec3 offs_pos = aPos + direction*offset;

    gl_PointSize = 2.0;
    normalized_ring = (2.0*float(ring_count - points[gl_VertexID].ring))/float(ring_count);
    if(norma < range){
        gl_Position = projection*view*model_lidar*vec4(offs_pos, 1.0);
    }
    else{
        gl_Position = vec4(2.0, 2.0, 2.0, 1.0); 
    }
    //gl_Position = vec4(0.0,0.0,0.0, 1.0); 
    //gl_Position = projection*view*vec4(0.0,3.0,0.0, 1.0);
}

