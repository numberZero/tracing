#version 430

layout(location = 4) uniform vec3 camera_position = vec3(0.0, 0.0, -1.5);
in vec3 dir;
out vec3 o_uv;

vec3 trace3(vec3 p, vec3 d);

void main() {
	o_uv = vec3(0.5) + vec3(1,-1,0) * trace3(camera_position, dir);
}
