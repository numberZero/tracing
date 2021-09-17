#version 430

layout(binding = 0) uniform sampler2D colors;

float target_z = 1.0;

in vec2 pos;
out vec4 o_color;

float scale(vec3 p) {
	const float inner_scale = 2.0;
	const float inner_radius = 0.5;
	const float outer_radius = 0.75;
	float r = length(p.yz);
	return mix(inner_scale, 1.0, smoothstep(inner_radius, outer_radius, r));
}

bool trace(inout vec3 p, inout vec3 d);

void main() {
	vec3 p = vec3(0.0, 0.0, -1.5);
// 	vec3 p = vec3(0.0, 0.0, 0.0);
	vec3 d = vec3(pos, 1.0);
	bool ok = trace(p, d);
	p += (target_z - p.z) / d.z * d;
	vec2 uv = mod(p.xy / 0.125, 2.0);
	bvec2 ab = lessThan(uv, vec2(1.0));
	uv = mod(uv, 1.0);
	bool c = ab.x == ab.y;
	o_color = vec4(uv.x, uv.y, !ok, 1.0);
}
