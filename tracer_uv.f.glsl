#version 430

layout(location = 4) uniform vec3 camera_position = vec3(0.0, 0.0, -1.5);
in vec3 dir;
out vec2 o_uv;

bool trace2(inout vec3 p, in vec3 d);

void main() {
	vec3 p = camera_position;
	if (dir.z <= 0.0) {
		o_uv = vec2(0.0);
		return;
	}
	bool ok = trace2(p, dir);

	vec2 uv = 0.25 * vec2(1.0, -1.0) * p.xy - 0.5;
	if (!ok)
		uv += vec2(0.5);
	o_uv = uv;
}
