#version 460

out vec4 FragColor;

in vec2 texCoord;

uniform vec2 viewportSize;

#define LINE_WEIGHT 1.0

uniform sampler2D bufferMask;

void main()
{
    float dx = (1.0 / viewportSize.x) * LINE_WEIGHT;
    float dy = (1.0 / viewportSize.y) * LINE_WEIGHT;

    vec2 uvCenter   = texCoord;
    vec2 uvRight    = vec2(uvCenter.x + dx, uvCenter.y);
    vec2 uvTop      = vec2(uvCenter.x,      uvCenter.y + dy);
    vec2 uvTopRight = vec2(uvCenter.x + dx, uvCenter.y + dy);
    //vec2 uvBotton = vec2(uvCenter.x, uvCenter.y - dy);
    //vec2 uvBottonLeft = vec2(uvCenter.x - dx, uvCenter.y - dy);
    //vec2 uvLeft    = vec2(uvCenter.x - dx, uvCenter.y);
    //vec2 uvBottonRight = vec2(uvCenter.x + dx, uvCenter.y - dy);
    //vec2 uvTopLeft = vec2(uvCenter.x - dx, uvCenter.y + dy);

    float mCenter   = texture(bufferMask, uvCenter).r;
    float mTop      = texture(bufferMask, uvTop).r;
    float mRight    = texture(bufferMask, uvRight).r;
    float mTopRight = texture(bufferMask, uvTopRight).r;
    //float mBotton = texture(bufferMask, uvBotton).r;
    //float mBottonLeft = texture(bufferMask, uvBottonLeft).r;
    //float mLeft    = texture(bufferMask, uvLeft).r;
    //float mBottonRight = texture(bufferMask, uvBottonRight).r;
    //float mTopLeft    = texture(bufferMask, uvTopLeft).r;

    float dT  = abs(mCenter - mTop);
    float dR  = abs(mCenter - mRight);
    float dTR = abs(mCenter - mTopRight);
    //float dB = abs(mCenter - mBotton);
    //float dBL = abs(mCenter - mBottonLeft);
    //float dL = abs(mCenter - mLeft);
    //float dBR = abs(mCenter - mBottonRight);
    //float dTL = abs(mCenter - mTopLeft);

    float delta = 0.0;
    delta = max(delta, dT);
    delta = max(delta, dR);
    delta = max(delta, dTR);
    //delta = max(delta, dB);
    //delta = max(delta, dBL);
    //delta = max(delta, dL);
    //delta = max(delta, dBR);
    //delta = max(delta, dTL);

    if(delta > 0.5) FragColor = vec4(0.96, 0.61, 0.22, 1.0);
    else discard;
}