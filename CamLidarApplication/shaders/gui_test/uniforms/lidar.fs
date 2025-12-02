#version 460 core

out vec4 FragColor;

in vec3 Color;

void main()
{    
    //float depth = LinearizeDepth(gl_FragCoord.z) / far; // divide by far for demonstration, normalize from 0 to 1
    //FragColor = vec4(vec3(depth), 1.0); //to visualize depth
    FragColor = vec4(Color,1.0);
    //FragColor = vec4(0,0,0,1.0);
}