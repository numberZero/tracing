#pragma once
#include "math.hxx"

class SpaceVisual {
public:
	vec3 color = {0.9f, 0.1f, 0.4f};

	SpaceVisual() = default;
	SpaceVisual(vec3 color) : color(color) {}
	virtual ~SpaceVisual() = default;

	virtual vec3 where(vec3 pos) const {
		return pos;
	}

	virtual mat3 jacobi(vec3 pos) const {
		return mat3(1.0f);
	}
};
