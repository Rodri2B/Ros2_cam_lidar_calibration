#version 460


uniform float offset;


const vec4 Pos[4] = vec4[](
    vec4(-1.0, -1.0, 0.0, 1.0),      // bottom left
    vec4( 1.0, -1.0, 0.0, 1.0),      // bottom right
    vec4( 1.0,  1.0, 0.0, 1.0),      // top right
    vec4(-1.0,  1.0, 0.0, 1.0)       // top left
);


vec2 Tex_Coord[4] = vec2[](
    vec2( 0.0, 0.0),      // bottom left
    vec2( 1.0, 0.0),      // bottom right
    vec2( 1.0, 1.0),      // top right
    vec2( 0.0, 1.0)       // top left
);


out vec2 texCoord;

const int Indices[6] = int[](0, 2, 3, 2, 0, 1);

void main() {
    int Index = Indices[gl_VertexID];

    gl_Position = Pos[Index];

    texCoord = Tex_Coord[Index];
}