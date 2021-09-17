#version 430

in vec2 pos;
out vec2 o_uv;

bool trace2(inout vec3 p, inout vec3 d);

void main() {
	vec3 p = vec3(0.0, 0.0, -1.5);
	vec3 d = vec3(pos, 1.0);
	bool ok = trace2(p, d);

	vec2 uv = 0.25 * vec2(1.0, -1.0) * p.xy - 0.5;
	if (!ok)
		uv += vec2(0.5);
	o_uv = mod(uv, 1.0);
}
