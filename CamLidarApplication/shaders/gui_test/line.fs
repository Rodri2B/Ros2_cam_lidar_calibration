#version 460 core

in vec3 gCameraWorldPos;
in vec3 WorldPos;
in vec2 screenPos;

in vec4 color;

out vec4 FragColor;

float gGridSize=100.0;

float satf(float x)
{
    float f = clamp(x, 0.0, 1.0);
    return f;
}

void main()
{



    vec4 fColor = color;
    float OpacityFalloff = (1.0 - satf(1.4*length(WorldPos.xyz - gCameraWorldPos.xyz) / gGridSize));
    fColor.a *= OpacityFalloff;
    FragColor = fColor;//Color;
}