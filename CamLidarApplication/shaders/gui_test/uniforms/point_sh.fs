#version 460 core

out vec4 FragColor;

in float normalized_ring;

vec3 hsv2rgb(vec3 color) {
    float h = mod(color.x * 6.0, 6.0);
    float s = color.y;
    float v = color.z;

    float c = v*s;

    float x = c * (1.0 - abs(mod(h, 2.0) - 1.0));
    float m = v - c;

    uint i = uint(floor(h));

    vec3 rgb;
    if (i == 0) {
        rgb = vec3(c, x, 0.0);
    } else if (i == 1) {
        rgb = vec3(x, c, 0.0);
    } else if (i == 2) {
        rgb = vec3(0.0, c, x);
    } else if (i == 3) {
        rgb = vec3(0.0, x, c);
    } else if (i == 4) {
        rgb = vec3(x, 0.0, c);
    } else {
        rgb = vec3(c, 0.0, x);
    }

    return rgb;
}

void main()
{    

    vec3 color = hsv2rgb(vec3(normalized_ring,1.0,1.0));
    FragColor = vec4(color,1.0);
}