#version 460 core
out vec4 FragColor;

uniform sampler2D texture_diffuse1;

in vec2 TexCoord;

void main()
{    
    //FragColor = vec4(0.4,0.4,0.4,1.0);
    FragColor = texture(texture_diffuse1, TexCoord);
}