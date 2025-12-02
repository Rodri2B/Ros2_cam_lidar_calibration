#version 460

uniform Matrices_cam
{
    mat4 view;
    mat4 projection;
};

uniform vec3 cameraPos;

out vec3 WorldPos;
out vec3 gCameraWorldPos;

const vec3 Pos[4] = vec3[](
    vec3(-100.0, 0.0, -100.0),      // bottom left
    vec3( 100.0, 0.0, -100.0),      // bottom right
    vec3( 100.0, 0.0,  100.0),      // top right
    vec3(-100.0, 0.0,  100.0)       // top left
);

const int Indices[6] = int[](0, 2, 3, 2, 0, 1);

void main() {
    int Index = Indices[gl_VertexID];
    vec3 vPos3 = Pos[Index];

    vPos3.x += cameraPos.x;
    vPos3.z += cameraPos.z;

    vec4 vPos4 = projection*view*vec4(vPos3,1.0);
    gl_Position = vPos4;

    WorldPos = vPos3;
    gCameraWorldPos = cameraPos;
}