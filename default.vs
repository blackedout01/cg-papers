#version 410 core

layout(location=0) in vec3 P;
layout(location=1) in vec3 C;

out vec3 FragC;
out vec3 FragP;

uniform vec2 ModelP;
uniform mat3 R;

void main() {
   FragP = R*P + vec3(ModelP, 0.0f);
   FragC = C;
   gl_Position = vec4(FragP, 1.0);
}
