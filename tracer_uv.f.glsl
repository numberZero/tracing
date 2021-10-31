#version 430

layout(location = 4) uniform vec3 camera_position = vec3(0.0, 0.0, -1.5);
in vec3 dir;
out vec3 o_uv;

bool trace2(inout vec3 p, in vec3 d);
vec3 trace3(vec3 p, vec3 d);

void main() {
// 	vec3 uvw = trace3(camera_position, dir);
// 	o_uv = vec3(0.5) + vec3(uvw.y, -uvw.z, 0) / uvw.x;
	vec3 pos = camera_position;
	trace2(pos, dir);
	o_uv = vec3(0.5) + vec3(-pos.z, -pos.y, 0) / 64.0;
}
