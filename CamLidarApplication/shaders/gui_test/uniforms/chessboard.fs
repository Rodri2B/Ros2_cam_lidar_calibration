#version 460 core

out vec4 FragColor;

//uniform uvec2 pattern_size;
uniform vec2 pattern_size;
uniform float square_length;
uniform vec2 board_dimension;
uniform vec2 translation_error;

//in vec3 fNormal_to_cam;
in vec2 fPos;
//in vec3 fPos_to_cam;

void main()
{    

    vec2 chess_pattern_size =  (pattern_size + vec2(1.0,1.0))*square_length;
    vec2 patter_origin = (board_dimension - chess_pattern_size)*0.5;
    patter_origin.y += chess_pattern_size.y;
    patter_origin += translation_error;
    //mat3 frag_to_pattern_matrix = mat3(1.0, 0.0, 0.0, 0.0, -1.0, 0.0, -patter_origin.x ,patter_origin.y, 1.0);
    vec2 frag_pos = (fPos + vec2(0.5,0.5))*board_dimension;
    //vec2 frag_patter_pos = (frag_to_pattern_matrix*vec3(frag_pos,1.0)).xy;
    vec2 frag_patter_pos = vec2(frag_pos.x-patter_origin.x,patter_origin.y-frag_pos.y);

    //if(dot(-fPos_to_cam,fNormal_to_cam) > 0){
    if(gl_FrontFacing){

        if(frag_patter_pos.x < 0.0 || frag_patter_pos.y < 0.0 ||
           frag_patter_pos.x > chess_pattern_size.x || frag_patter_pos.y > chess_pattern_size.y){

            FragColor = vec4(1.0);
        }else{
            uint index = uint(floor(frag_patter_pos.x/square_length) + floor(frag_patter_pos.y/square_length));

            FragColor = (index % 2 == 0) ? vec4(0.0,0.0,0.0,1.0):vec4(1.0);
        }

    }
    else{
        FragColor = vec4(0.1,0.1,0.1,1.0);
    }
}