#version 430

layout(std140, binding=2) uniform ParamsBlock {
// Params
	float outer_radius;
	float inner_radius;
	float outer_half_length;
	float inner_half_length;
	float inner_pad;
// Coefs
	float x1, y1, x2, y2;
	float x0, y0, w;
};

struct decomp3 {
	mat3 ortho;
	vec3 diag;
};

bool contain(vec3 pos) {
	const float eps = 1.0e-3; // должен быть больше, чем на CPU
	float r = length(pos.yz);
	return abs(pos.x) <= outer_half_length + eps && r <= outer_radius + eps && r >= inner_radius - eps;
}

decomp3 halfmetric(vec3 pos) {
	float x = pos.x;
	float dx = 1.0;
	if (abs(x) < y1) {
		dx = x1 / y1;
	} else if (abs(x) < y2) {
		x = (x0 + sqrt((abs(x) - y0) / w)) * sign(x);
		dx /= -2 * w * (abs(x) - x0);
	}
	float c = smoothstep(inner_radius, outer_radius, length(pos.yz));
	return decomp3(mat3(1.0), vec3(mix(dx, 1.0, c), 1.0, 1.0));
}
