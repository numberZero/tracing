#version 430

layout(points) in;
layout(triangle_strip, max_vertices = 4) out;
layout(location = 0) uniform mat3 camera;
out vec3 dir;

void main() {
	dir = camera * vec3(1.0, -1.0, 1.0);
	gl_Position = vec4(1.0, -1.0, 0.0, 1.0);
	EmitVertex();

	dir = camera * vec3(1.0, 1.0, 1.0);
	gl_Position = vec4(1.0, 1.0, 0.0, 1.0);
	EmitVertex();

	dir = camera * vec3(-1.0, -1.0, 1.0);
	gl_Position = vec4(-1.0, -1.0, 0.0, 1.0);
	EmitVertex();

	dir = camera * vec3(-1.0, 1.0, 1.0);
	gl_Position = vec4(-1.0, 1.0, 0.0, 1.0);
	EmitVertex();
}
