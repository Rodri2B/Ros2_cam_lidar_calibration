#version 460 core

in vec3 gCameraWorldPos;



float gGridSize=100.0;
float gGridMinPixelsBetweenCells = 2.0;
float lineThickness = 3.0;
uint  LOD_enabled = 1;
float gGridCellSize = 0.5;
vec4 gGridColorThin = vec4(0.5f, 0.5f, 0.5f, 0.4f);
vec4 gGridColorThick = vec4(0.5f, 0.5f, 0.5f, 0.4f);


in vec3 WorldPos;


layout(location = 0) out vec4 FragColor;

float log10(float x)
{
    float f = log(x) / log(10.0);
    return f;
}


float satf(float x)
{
    float f = clamp(x, 0.0, 1.0);
    return f;
}


vec2 satv(vec2 x)
{
    vec2 v = clamp(x, vec2(0.0), vec2(1.0));
    return v;
}


float max2(vec2 v)
{
    float f = max(v.x, v.y);
    return f;
}


void main()
{
    vec2 dvx = vec2(dFdx(WorldPos.x), dFdy(WorldPos.x));
    vec2 dvy = vec2(dFdx(WorldPos.z), dFdy(WorldPos.z));

    float lx = length(dvx);
    float ly = length(dvy);

    vec2 dudv = vec2(lx, ly);

    float l = length(dudv);

    float LOD = (LOD_enabled == 1) ? max(0.0, log10(l * gGridMinPixelsBetweenCells / gGridCellSize) + 1.0) : 0.0;

    dudv *= lineThickness;
    
    float GridCellSizeLod0 = gGridCellSize * pow(10.0, floor(LOD));
    float GridCellSizeLod1 = GridCellSizeLod0 * 10.0;
    float GridCellSizeLod2 = GridCellSizeLod1 * 10.0;

    vec2 mod_div_dudv = mod(WorldPos.xz, GridCellSizeLod0) / dudv;
    float Lod0a = max2(vec2(1.0) - abs(satv(mod_div_dudv) * 2.0 - vec2(1.0)) );

    mod_div_dudv = mod(WorldPos.xz, GridCellSizeLod1) / dudv;
    float Lod1a = max2(vec2(1.0) - abs(satv(mod_div_dudv) * 2.0 - vec2(1.0)) );
    
    mod_div_dudv = mod(WorldPos.xz, GridCellSizeLod2) / dudv;
    vec2 Lod2veca = vec2(1.0) - abs(satv(mod_div_dudv) * 2.0 - vec2(1.0));
    float Lod2a = max2(Lod2veca);

    float LOD_fade = fract(LOD);


    vec4 Color;

    if (Lod2a > 0.0) {
        if(WorldPos.z <= 0.3 && WorldPos.z >= -0.3){
            float activate =  ((Lod2veca.y == 0.0) ? 1.0:0.0);
            if(WorldPos.x <= 0.3 && WorldPos.x >= -0.3){
                Color = vec4(1.0 - activate,0.21,0.33,Lod2veca.y/Lod2a) + vec4(0.17,0.56,1.0,1.0)*activate;
            }
            else{
                Color = vec4(1.0 - activate,0.21,0.33,Lod2veca.y/Lod2a) + gGridColorThick*activate;
            }
            Color.a *= Lod2a;
        }
        else if(WorldPos.x <= 0.3 && WorldPos.x >= -0.3){
            float activate =  ((Lod2veca.x == 0.0) ? 1.0:0.0);
            Color = vec4(0.17,0.56,1.0 - activate,Lod2veca.x/Lod2a) + gGridColorThick*activate;
            Color.a *= Lod2a;
        }
        else{
            Color = gGridColorThick;
            Color.a *= Lod2a;
        }
    } else {
        if (Lod1a > 0.0) {
            Color = mix(gGridColorThick, gGridColorThin, LOD_fade);
            //Color = gGridColorThick;
	        //Color.a *= Lod1a;
            Color.a *= Lod1a*(1.0 - satf(0.2*length(WorldPos.xyz - gCameraWorldPos.xyz) / gGridSize));
        } else {
            Color = gGridColorThin;
	        //Color.a *= (Lod0a*(1.0 - LOD_fade)) ;
            //Color.a *= Lod0a*(1.0 - satf(2*length(WorldPos.xyz - gCameraWorldPos.xyz) / gGridSize));
            Color.a *= (Lod0a*(1.0 - LOD_fade))*(1.0 - satf(2*length(WorldPos.xyz - gCameraWorldPos.xyz) / gGridSize));
        }
    }
    
    float OpacityFalloff = (1.0 - satf(length(WorldPos.xyz - gCameraWorldPos.xyz) / gGridSize));

    Color.a *= OpacityFalloff;

    FragColor = Color;
}