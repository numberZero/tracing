#version 420

layout(binding = 0) uniform sampler2D colors;
layout(binding = 1) uniform sampler2D uvs;

in vec2 pos;
out vec4 o_color;

void main() {
	vec2 uv = texture(uvs, pos).xy;
	o_color = texture(colors, uv);
}
