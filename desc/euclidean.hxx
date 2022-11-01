#pragma once
#include "flat.hxx"
#include "riemann.hxx"
#include "math.hxx"

class EuclideanSpace final: public SpaceDesc, public RiemannMetric<2> {
public:
	Ray fromGlobal(Ray ray) const noexcept override {
		return ray;
	}

	Ray toGlobal(Ray ray) const noexcept override {
		return ray;
	}

	decomp2 halfmetric([[maybe_unused]] vec2 pos) const noexcept override {
		return {mat2(1.0f), vec2(1.0f, 1.0f)};
	}

	static const EuclideanSpace instance;
};
inline const EuclideanSpace EuclideanSpace::instance;
