#version 460

in vec2 texCoord;

uniform sampler2D screenTexture;

layout(location = 0) out vec4 FragColor;

void main() {

    FragColor = texture(screenTexture, texCoord);
    
    //FragColor = vec4(vec3(gl_FragCoord.z),1.0);
    //FragColor = vec4(1.0,0.0,0.0,1.0);
}