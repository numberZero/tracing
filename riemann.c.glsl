#version 430
// #extension GL_ARB_compute_variable_group_size: require

// layout(local_size_variable) in;
layout(local_size_x = 64) in;

struct Ray {
	vec4 pos; // только xyz используются
	vec4 dir; // только xyz используются
};

layout(std430, binding=0) restrict readonly buffer Input {
	Ray ins[];
};

layout(std430, binding=1) restrict writeonly buffer Output {
	Ray outs[]; // pos.w используется для длины пути
};

// uniform layout(location=0) float dt = 0.1;
const float dt = 1.0e-2;

bool contain(vec3 pos);
mat3 metric(vec3 pos);
mat3[3] krist(vec3 pos);

float len(vec3 pos, vec3 vec) {
	mat3 g = metric(pos);
	return sqrt(dot(vec, g * vec));
}

vec3 covar(mat3[3] G, vec3 v) {
	vec3 ret;
	for (int k = 0; k < 3; k++)
		ret[k] = -dot(v, G[k] * v);
	return ret;
}

void main() {
	Ray from = ins[gl_GlobalInvocationID.x];

	float t = 0.0;
	vec3 p = from.pos.xyz;
	vec3 v = from.dir.xyz;
	v /=  len(p, v);
	while (contain(p)) {
		vec3 a = covar(krist(p), v);
		v += dt * a;
		p += dt * v;
		t += dt;
	}
	Ray end;
	end.pos.xyz = p;
	end.pos.w = t;
	end.dir.xyz = normalize(v);
	end.dir.w = 0.0;

	outs[gl_GlobalInvocationID.x] = end;
}
