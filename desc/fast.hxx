#pragma once
#include "flat.hxx"
#include "riemann.hxx"
#include "math.hxx"

class FastSpace: public SpaceDesc, public RiemannMetric<2> {
public:
	float outer_hl = 2.0;
	float inner_hl = 1.25;
	float inner_pad = 0.25;

	Ray fromGlobal(Ray ray) const noexcept final override {
		Coefs cs(*this);
		if (abs(ray.base.x) < cs.y1) {
			ray.base.x *= cs.x1 / cs.y1;
			ray.dir.x *= cs.x1 / cs.y1;
		} else if (abs(ray.base.x) < cs.y2) {
			ray.base.x = copysign(sqrt((abs(ray.base.x) - cs.y0) / cs.w) - cs.x0, ray.base.x);
			ray.dir.x /= 2 * cs.w * (abs(ray.base.x) - cs.x0);
		} else {
			ray.base.x -= copysign(cs.y2 - cs.x2, ray.base.x);
		}
		return ray;
	}

	Ray toGlobal(Ray ray) const noexcept final override {
		Coefs cs(*this);
		if (abs(ray.base.x) < cs.x1) {
			ray.base.x *= cs.y1 / cs.x1;
			ray.dir.x *= cs.y1 / cs.x1;
		} else if (abs(ray.base.x) < cs.x2) {
			ray.dir.x *= 2 * cs.w * (abs(ray.base.x) - cs.x0);
			ray.base.x = copysign(cs.y0 + cs.w * sqr(abs(ray.base.x) - cs.x0), ray.base.x);
		} else {
			ray.base.x += copysign(cs.y2 - cs.x2, ray.base.x);
		}
		return ray;
	}

	decomp halfmetric(vec2 pos) const noexcept final override {
		Coefs cs(*this);
		float x = pos.x;
		float dx = 1.0f;
		if (abs(x) < cs.y1) {
			dx = cs.x1 / cs.y1;
		} else if (abs(x) < cs.y2) {
			x = copysign(cs.x0 + sqrt((abs(x) - cs.y0) / cs.w), x);
			dx /= -2 * cs.w * (abs(x) - cs.x0);
		}
		return {mat2(1.0f), {dx, 1.0f}};
	}

protected:
	struct Coefs {
		float x1, y1, x2, y2;
		float x0, y0, w;

		Coefs(FastSpace const &params) {
			x2 = params.inner_hl;
			y2 = params.outer_hl;
			x1 = params.inner_hl - params.inner_pad;

			// TODO исправить неустойчивость при y2 ≈ x2
			y1 = x1 * (x1 - x2 + 2*y2) / (x1 + x2);
			x0 = 0.5 * (sqr(x2) + sqr(x1) - 2 * x2 * y2) / (x2 - y2);
			y0 = y2 - 0.25 * (sqr(x2) - sqr(x1)) / (x2 - y2);
			w = (x2 - y2) / (sqr(x2) - sqr(x1));
		}
	};
};
