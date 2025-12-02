#version 460

uniform mat4 view;
uniform mat4 projection;

uniform vec3 cameraPos;

out vec3 WorldPos;
out vec3 gCameraWorldPos;
out vec4 color;

const vec3 Pos[4] = vec3[](
    vec3(-100.0, 0.0, 0.01),      // bottom left
    vec3( 100.0, 0.0, 0.01),      // bottom right
    vec3( 0.01, 0.0,  -100.0),      // top right
    vec3( 0.01, 0.0,  100.0)       // top left
);

void main() {
    vec3 vPos3 = Pos[gl_VertexID];

    if(gl_VertexID == 0 || gl_VertexID == 1){
    vPos3.x += cameraPos.x;
    color = vec4(1.0,0.0,0.0,1.0);
    }
    else{
    vPos3.z += cameraPos.z;
    color = vec4(0.0,0.0,1.0,1.0);
    }

    gl_Position = projection*view*vec4(vPos3,1.0);

    WorldPos = vPos3;
    gCameraWorldPos = cameraPos;
}