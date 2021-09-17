#version 430

layout(points) in;
layout(triangle_strip, max_vertices = 4) out;
out sample vec2 pos;

void main() {
	pos = vec2(1.0, 0.0);
	gl_Position = vec4(1.0, -1.0, 0.0, 1.0);
	EmitVertex();

	pos = vec2(1.0, 1.0);
	gl_Position = vec4(1.0, 1.0, 0.0, 1.0);
	EmitVertex();

	pos = vec2(0.0, 0.0);
	gl_Position = vec4(-1.0, -1.0, 0.0, 1.0);
	EmitVertex();

	pos = vec2(0.0, 1.0);
	gl_Position = vec4(-1.0, 1.0, 0.0, 1.0);
	EmitVertex();
}
