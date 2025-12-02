#version 460 core

uniform mat4 proj_view_inverse_virtual_view;
uniform uint render_mode;
uniform float normal_size;

const int IndicesPlane[6] = int[](1, 0, 3, 3, 2, 1);
const int IndicesLines[4] = int[](0, 1, 2, 3);

struct BoardDetection {
    vec4 border_points[6];   // 5 points x vec4 + 1 (center+normal) x vec4 (i.e. 24 floats total)
};

// Using std140 layout
layout(std140) uniform BoardBlock {
    BoardDetection board;
};

void main()
{   //render plane
    if(render_mode == 0){
        int Index = IndicesPlane[gl_VertexID];
        gl_Position = proj_view_inverse_virtual_view*board.border_points[Index];
    }//render line loop
    else if(render_mode == 1){
        int Index = IndicesLines[gl_VertexID];
        gl_Position = proj_view_inverse_virtual_view*board.border_points[Index];
    }//render points
    else if(render_mode == 2){
        gl_Position = proj_view_inverse_virtual_view*board.border_points[gl_VertexID];
        gl_PointSize = 15.0;
    }//render normal vector
    else{
        vec4 vector[2];
        vector[0] = board.border_points[4];
        vector[1] = normal_size*board.border_points[5] + vector[0];
        gl_Position = proj_view_inverse_virtual_view*vector[gl_VertexID];
    }
}