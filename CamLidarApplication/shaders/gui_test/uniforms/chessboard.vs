#version 460

layout(location = 0) in vec3 aPos;
//layout(location = 1) in vec3 aNormal;

layout(std140) uniform Matrices_cam
{
    mat4 view;
    mat4 projection;
};


uniform mat4 model;

//out vec3 fNormal_to_cam;
out vec2 fPos; 
//out vec3 fPos_to_cam;

void main() {

    fPos = aPos.zy;
    mat4 view_model = view*model;
    //vec4 Pos_to_cam = view_model*vec4(-aPos.y,aPos.x,aPos.z,1.0);
    vec4 Pos_to_cam = view_model*vec4(aPos,1.0);
    //fPos_to_cam = Pos_to_cam.xyz;
    //mat3 normalMatrix = mat3(transpose(inverse(view_model)));
    //fNormal_to_cam = normalize(vec3(normalMatrix * vec3(-aNormal.y,aNormal.x,aNormal.z)));
    //fNormal_to_cam = normalize(vec3(normalMatrix * aNormal));
    gl_Position = projection*Pos_to_cam;
}