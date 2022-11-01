#pragma once
#include "riemann.hxx"
#include "math.hxx"

class BoxSmoother: public RiemannMetric<2> {
public:
	RiemannMetric<2> *base;
	float halfwidth = 2.0f;
	float pad = 0.5f;

	decomp halfmetric(vec pos) const noexcept final override {
// 		if (abs(pos.y) > halfwidth)
// 			return {mat2(1.0f), vec2(1.0f)};
		auto g = base->halfmetric(pos);
		float c = clamp((halfwidth - abs(pos.y)) / pad, 0.0f, 1.0f);
// 		if (c == 0.0f)
// 			return g;
		c = smoothstep(c);
		return {g.ortho, mix(vec2(1.0f), g.diag, c)};
	}
};
