#version 410 core

in vec3 FragP;
in vec3 FragN;

out vec4 Result;

uniform vec4 Color;
uniform vec3 LightDir;

void main() {
    vec3 N = normalize(FragN);
    vec3 L = -normalize(LightDir);
    
    vec3 Ambient = 0.5*vec3(0.68, 0.76, 0.99);
    float Diffuse = max(dot(N, L), 0);
    
    //float Intensity = Ambient + (1.0 - Ambient)*Diffuse;
    vec3 Intensity = mix(Ambient, vec3(1.0), Diffuse);
    
    Result = vec4(Intensity*Color.rgb, 1.0);
}
