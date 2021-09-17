#version 430

layout(points) in;
layout(triangle_strip, max_vertices = 4) out;
layout(location = 0) uniform vec2 halfsize;
out vec2 pos;

void main() {
	pos = vec2(halfsize.x, -halfsize.y);
	gl_Position = vec4(1.0, -1.0, 0.0, 1.0);
	EmitVertex();

	pos = vec2(halfsize.x, halfsize.y);
	gl_Position = vec4(1.0, 1.0, 0.0, 1.0);
	EmitVertex();

	pos = vec2(-halfsize.x, -halfsize.y);
	gl_Position = vec4(-1.0, -1.0, 0.0, 1.0);
	EmitVertex();

	pos = vec2(-halfsize.x, halfsize.y);
	gl_Position = vec4(-1.0, 1.0, 0.0, 1.0);
	EmitVertex();
}
