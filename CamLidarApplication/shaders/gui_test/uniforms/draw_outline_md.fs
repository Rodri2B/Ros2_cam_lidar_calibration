#version 460

out vec4 FragColor;

in vec2 texCoord;

uniform vec2 viewportSize;

#define LINE_WEIGHT 2.0

uniform sampler2D bufferMask;
uniform sampler2D bufferDepthMask;
uniform sampler2D bufferDepth;

void main()
{
    float dx = (1.0 / viewportSize.x) * LINE_WEIGHT;
    float dy = (1.0 / viewportSize.y) * LINE_WEIGHT;

    vec2 uvCenter   = texCoord;
    vec2 uvRight    = vec2(uvCenter.x + dx, uvCenter.y);
    vec2 uvTop      = vec2(uvCenter.x,      uvCenter.y - dx);
    vec2 uvTopRight = vec2(uvCenter.x + dx, uvCenter.y - dx);

    float mCenter   = texture(bufferMask, uvCenter).r;
    float mTop      = texture(bufferMask, uvTop).r;
    float mRight    = texture(bufferMask, uvRight).r;
    float mTopRight = texture(bufferMask, uvTopRight).r;

    float depthCenter   = texture(bufferDepthMask, uvCenter).r;
    float depthTop      = texture(bufferDepthMask, uvTop).r;
    float depthRight    = texture(bufferDepthMask, uvRight).r;
    float depthTopRight = texture(bufferDepthMask, uvTopRight).r;

    float fragdp = texture(bufferDepth, uvCenter).r;

    float dT  = abs(mCenter - mTop);
    float dR  = abs(mCenter - mRight);
    float dTR = abs(mCenter - mTopRight);

    float delta = 0.0;
    delta = max(delta, dT);
    delta = max(delta, dR);
    delta = max(delta, dTR);

    if(delta > 0.5){

        if(mCenter < 0.5){
            
            float sum = 0.0;
            float num = 0.0;

            if(mTop>0.5){
                num += 1.0;
                sum += depthTop;
            }
            if(mRight>0.5){
                num += 1.0;
                sum += depthRight;
            }
            if(mTopRight>0.5){
                num += 1.0;
                sum += depthTopRight;
            }

            float depthValue = sum/num;

            if(fragdp > depthValue){
                //FragColor = vec4(0.96, 0.61, 0.22, 1.0);
                gl_FragDepth = depthValue; 
            }
        }else{
            if(fragdp >= depthCenter){
                //FragColor = vec4(0.96, 0.61, 0.22, 1.0);
                gl_FragDepth = depthCenter; 
            }
        }


    }

    //vec4 outline = vec4(delta, delta, delta, 1.0);
    //vec4 albedo  = texture(gbufferAlbedo, vsUV);

    //fsColor = albedo - outline;
    
    //FragColor = vec4(fragdp, fragdp, fragdp, 1.0);
    ///FragColor = vec4(depthCenter, depthCenter, depthCenter, 1.0);
    //FragColor = vec4(mCenter , mCenter , mCenter , 1.0);
    if(delta > 0.5) FragColor = vec4(0.96, 0.61, 0.22, 1.0);
    else
    FragColor = vec4(1.0,1.0,1.0, 0.0);
}