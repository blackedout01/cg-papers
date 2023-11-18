#version 410 core

layout(location=0) in vec3 P;
layout(location=1) in vec3 N;

uniform mat4 MatM;
uniform mat4 MatPV;

void main() {
    vec4 P4 = MatPV*MatM*vec4(P, 1.0);
    gl_Position = P4;
}
