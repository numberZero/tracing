#version 430

layout(location = 4) uniform vec3 camera_position = vec3(0.0, 0.0, -1.5);
layout(binding = 0) uniform sampler2D colors;

in vec3 dir;
out vec4 o_color;

bool trace2(inout vec3 p, in vec3 d);

void main() {
	vec3 p = camera_position;
	bool ok = trace2(p, dir);

	vec2 uv = 0.25 * vec2(1.0, -1.0) * p.xy - 0.5;
	o_color = texture(colors, uv);
	if (!ok)
		o_color.b = clamp(o_color.b + 0.5, 0.0, 1.0);
}
