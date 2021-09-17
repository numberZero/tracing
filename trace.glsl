#version 450

layout(location = 1) uniform float dt = 1.0 / 128.0;

float target_z;

float scale(vec3 p);

vec3 grad(vec3 p) {
	const float h = 1.0 / 1024.0;
	float Dx = scale(p + vec3(h, 0.0, 0.0)) - scale(p - vec3(h, 0.0, 0.0));
	float Dy = scale(p + vec3(0.0, h, 0.0)) - scale(p - vec3(0.0, h, 0.0));
	float Dz = scale(p + vec3(0.0, 0.0, h)) - scale(p - vec3(0.0, 0.0, h));
	return vec3(Dx, Dy, Dz) / (h / 2.0);
}

bool trace(inout vec3 p, inout vec3 d) {
	d = normalize(d);
	for (int k = 0; p.z < target_z; k++) {
		if (k >= 1000)
		return false;
		float c = scale(p);
		if (c != 1.0) {
			vec3 dc = grad(p);
			vec3 dd;
			dd.x = (0.5 / c) * (dc.x * d.x - dc.y * d.y - dc.z * d.z) * d.x;
			dd.y = 0.5 * dc.y * d.x * d.x;
			dd.z = 0.5 * dc.z * d.x * d.x;
			d += dt * dd;
		}
		p += dt * d;
	}
	return true;
}