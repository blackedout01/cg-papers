#version 410 core

in vec3 FragP;
in vec3 FragN;
in vec4 LightP;

out vec4 Result;

uniform vec4 Color;
uniform vec3 LightDir;

uniform sampler2D LightDepthTex;

void main() {
    vec3 N = normalize(FragN);
    vec3 L = -normalize(LightDir);
    
    vec3 Ambient = 0.5*vec3(0.68, 0.76, 0.99);
    float Diffuse = max(dot(N, L), 0);

    vec3 LightXYZ = LightP.xyz/LightP.w;
    LightXYZ = (LightXYZ + 1.0)/2.0;

    // 1. get depth of the point in light space
    float PointDepth = LightXYZ.z;

    // 2. use to sample from deth texture
    float ClosestDepth = texture(LightDepthTex, LightXYZ.xy).r;

    float ShadowFactor = (PointDepth > ClosestDepth)? 0.0 : 1.0;

    Diffuse *= ShadowFactor;

    //float Intensity = Ambient + (1.0 - Ambient)*Diffuse;
    vec3 Intensity = mix(Ambient, vec3(1.0), Diffuse);

    Result = vec4(Intensity*Color.rgb, 1.0);
    //Result = vec4(vec3(), 1.0);
}
