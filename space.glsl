#version 430

layout(location = 5) uniform vec4 params = vec4(1.0, 0.5, 0.0, 2.0);

float scale(vec3 p) {
	const float outer_radius = params.x;
	const float inner_radius = params.y;
	const float inner_scale = params.w;
	float r = length(p.yz);
	float l = length(p.x);
	float a = smoothstep(inner_radius, outer_radius, r);
	float b = smoothstep(9.0, 10.0, l);
	return mix(inner_scale, 1.0, a + b - a * b);
}
