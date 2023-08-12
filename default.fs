#version 410 core

in vec3 FragC;
in vec3 FragP;

out vec4 Result;

void main() {
   Result = vec4(FragC, 1.0);
}
