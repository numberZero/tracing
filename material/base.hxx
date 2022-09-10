#pragma once
#include <glm/glm.hpp>

using glm::vec3;

struct Light {
	vec3 light = {0,0,0};
	vec3 filter = {1,1,1};

	operator vec3() const noexcept {
		return light;
	}

	Light &operator*= (vec3 f) noexcept {
		filter *= f;
		return *this;
	}

	Light &operator+= (vec3 l) noexcept {
		light += filter * l;
		return *this;
	}
};

class Material {
public:
	virtual void hit(Light &light, vec3 &dir, vec3 normal) const noexcept = 0;

	vec3 color;
	vec3 emission;
	float softness;
};
