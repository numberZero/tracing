#version 430

layout(binding = 0) uniform sampler2D colors;

const float target_z = 4.0;
const float outer_radius = 1.00;

in vec2 pos;
out vec4 o_color;

float scale(vec3 p) {
	const float inner_scale = 2.0;
	const float inner_radius = 0.50;
	float r = length(p.yz);
	return mix(inner_scale, 1.0, smoothstep(inner_radius, outer_radius, r));
}

bool trace(inout vec3 p, inout vec3 d);

void main() {
	vec3 p = vec3(0.0, 0.0, -1.5);
	vec3 d = vec3(pos, 1.0);
	bool ok = trace(p, d);
	p += (target_z - p.z) / d.z * d;

	vec2 uv = 0.25 * vec2(1.0, -1.0) * p.xy - 0.5;
	o_color = texture(colors, uv);
	if (!ok)
		o_color.b = clamp(o_color.b + 0.5, 0.0, 1.0);
}
