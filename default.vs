#version 410 core

layout(location=0) in vec3 P;
layout(location=1) in vec3 N;

out vec3 FragP;
out vec3 FragN;

uniform vec2 ModelP;
uniform mat4 R;

uniform mat4 MatM;
uniform mat4 MatV;
uniform mat4 MatP;

void main() {
    vec4 P4 = MatP*MatV*MatM*vec4(P, 1.0);
    FragP = P4.xyz;
    FragN = N;
    gl_Position = P4;
}
