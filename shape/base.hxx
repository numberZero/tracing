#pragma once
#include <glm/glm.hpp>
#include <optional>

using std::optional, std::nullopt;
using glm::vec3, glm::vec4;

struct Ray {
	vec3 pos;
	vec3 dir;
};

class Shape {
public:
	struct Hit {
		vec3 pos;
		float dist;
		vec3 normal;
	};

	virtual optional<Hit> hit(Ray const &ray) const noexcept = 0;
};
