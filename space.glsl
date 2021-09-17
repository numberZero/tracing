#version 430

float outer_radius = 1.00;

float scale(vec3 p) {
	const float inner_scale = 2.0;
	const float inner_radius = 0.50;
	float r = length(p.yz);
	return mix(inner_scale, 1.0, smoothstep(inner_radius, outer_radius, r));
}
