#version 460 core

out vec4 FragColor;

uniform uint color_mode;

void main()
{   
    if(color_mode == 0) FragColor = vec4(1.0,0.0,0.0,1.0); //red
    else if(color_mode == 1) FragColor = vec4(0.0,1.0,0.0,1.0); //green
    else if(color_mode == 2) FragColor = vec4(0.0,0.0,1.0,1.0); //blue
    else if(color_mode == 3) FragColor = vec4(0.0,1.0,1.0,1.0); //cyan
    else if(color_mode == 4) FragColor = vec4(1.0,0.0,1.0,1.0); //pink magenta
    else if(color_mode == 5) FragColor = vec4(1.0,1.0,0.0,1.0); //yellow
    else if(color_mode == 6) FragColor = vec4(0.4,0.2,0.0,1.0); //brown
    else FragColor = vec4(1.0,1.0,1.0,1.0); //white
  
}