#version 420

layout(binding = 0) uniform samplerCube space;
layout(binding = 1) uniform sampler2D uvs;

in sample vec2 pos;
out vec4 o_color;

void main() {
	vec4 c = texture(uvs, pos);
	o_color = c.w == 0 ? texture(space, c.xyz) : vec4(0, 0, 0, 0);
}
