#pragma once
#include "riemann.hxx"
#include "math.hxx"

class CoilMetric: public RiemannMetric<2> {
public:
	float coil_scale = 3.0f;
	float coil_r = 3.0f;
	float coil_w = 0.5f;
	float coil_m = 0.1f;

	decomp halfmetric(vec pos) const noexcept final override {
		float r = glm::length(pos);
		vec2 dir = glm::normalize(pos);

		float s = box(r, {coil_r - coil_w, coil_r + coil_w}, coil_m);
		float t = glm::mix(1.0f, coil_r/r/coil_scale, s);

		return {
			.ortho = {
				dir.x, -dir.y,
				dir.y, dir.x,
			},
			.diag = {1.0f, t},
		};
	}
};

