#version 460 core

out vec4 FragColor;
  
in vec2 TexCoord;

//uniform sampler2D ourTexture;

uniform sampler2D texture_diffuse1;
//uniform sampler2D texture2;

void main()
{   
    //FragColor = vec4(1.0,1.0,1.0,1.0);
    FragColor = texture(texture_diffuse1, TexCoord);
   //FragColor = mix(texture(texture1, TexCoord), texture(texture2, TexCoord), 0.2);  
}

