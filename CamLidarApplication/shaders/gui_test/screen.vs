#version 460


//uniform float offset;

uniform vec3 radialDistortionParams;
uniform vec2 tangentialDistortionParams;

uniform vec2 start_pos;
uniform vec2 start_pos_tex;

uniform vec2 tile_step;
uniform vec2 tile_step_tex;

uniform uint tile_quantity_x;
uniform uint tile_quantity_y;

uniform vec2 scale_to_meters;



vec2 RadialDistortion(vec2 coord, float k1, float k2, float k3) {
    float r = length(coord);
    float distortionFactor = 1.0 + k1 * pow(r, 2) + k2 * pow(r, 4) + k3 * pow(r, 6);
    return distortionFactor * coord;
}

vec2 TangentialDistortion(vec2 coord, float p1, float p2) {
    float x = coord.x;
    float y = coord.y;
    float r2 = x * x + y * y;
    float dx = 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
    float dy = p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;
    return vec2(dx, dy);
}


vec2 Pos[4] = vec2[](
    vec2( 0.0,          0.0        ),      // bottom left
    vec2( tile_step.x,  0.0        ),      // bottom right
    vec2( tile_step.x,  tile_step.y),      // top right
    vec2( 0.0,          tile_step.y)       // top left
);

vec2 Tex_Coord[4] = vec2[](
    vec2( 0.0,              0.0            ),      // bottom left
    vec2( tile_step_tex.x,  0.0            ),      // bottom right
    vec2( tile_step_tex.x,  tile_step_tex.y),      // top right
    vec2( 0.0,              tile_step_tex.y)       // top left
);

const int Indices[6] = int[](0, 2, 3, 2, 0, 1);


out vec2 texCoord;

void main() {
    vec2 position;
    position.x = ((gl_VertexID/6)%tile_quantity_x)*tile_step.x + start_pos.x;
    position.y = ((gl_VertexID/6)/tile_quantity_x)*tile_step.y + start_pos.y;

    vec2 position_tex;
    position_tex.x = ((gl_VertexID/6)%tile_quantity_x)*tile_step_tex.x + start_pos_tex.x;
    position_tex.y = ((gl_VertexID/6)/tile_quantity_x)*tile_step_tex.y + start_pos_tex.y;

    uint Index = Indices[gl_VertexID%6];

    position += Pos[Index];
    position_tex += Tex_Coord[Index];

    //gl_Position = vec4(position,0.0,1.0);

    //float depth = 0.01*dot(position,position)-1.0;

    position = position*scale_to_meters;
    position.y = -position.y;

    // Apply radial distortion
    vec2 radialDistortionCoords = RadialDistortion(position, radialDistortionParams.x, radialDistortionParams.y, radialDistortionParams.z);
    // Apply tangential distortion
    vec2 tangentialDistortionCoords = TangentialDistortion(position, tangentialDistortionParams.x, tangentialDistortionParams.y);

    // Combine the distortions
    vec2 distortedCoord = radialDistortionCoords + tangentialDistortionCoords;

    distortedCoord.y = -distortedCoord.y;
    distortedCoord = distortedCoord/scale_to_meters;

    gl_Position = vec4(distortedCoord,0.0,1.0);
    //gl_Position = vec4(distortedCoord,depth,1.0);

    texCoord = position_tex;
}