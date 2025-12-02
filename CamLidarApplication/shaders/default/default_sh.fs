#version 460 core

out vec4 FragColor;
  
in vec3 Normal;

//uniform sampler2D ourTexture;

uniform sampler2D texture1;

void main()
{
    //FragColor = texture(ourTexture, TexCoord) * vec4(ourColor, 1.0);
    FragColor = vec4(0.95,0.95,0.95,1.0);  
}

