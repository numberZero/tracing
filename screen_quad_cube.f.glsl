#version 420

layout(binding = 0) uniform samplerCube space;
layout(binding = 1) uniform sampler2D uvs;

in sample vec2 pos;
out vec4 o_color;

void main() {
	vec4 c = texture(uvs, pos);
	o_color = texture(space, c.xyz);
// 	if (c.w != int(c.w))
// 		o_color = vec4(0, 0, 0, 0);
}
