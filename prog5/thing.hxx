#pragma once
#include <cmath>
#include <limits>
#include "prog5/subspace.hxx"

struct ThingInfo {
	FlatSubspace const *thing;
	vec2 pos;
	float radius;
};

class ThingyBoundary: public SubspaceBoundary {
public:
	SubspaceBoundary *base;
	std::vector<ThingInfo> things;

	Transition findBoundary(Ray ray) const override {
		Transition t = base->findBoundary(ray);
		float dist = t.into ? distance(ray.pos, t.atPos) : std::numeric_limits<float>::infinity();

		for (auto &&info: things) {
			const vec2 rel = info.pos - ray.pos;
			const float t_center = dot(rel, ray.dir);
			const float d2 = dot(rel, rel) - sqr(t_center);
			if (d2 > sqr(info.radius))
				continue;
			const float t_diff = std::sqrt(sqr(info.radius) - d2);
			const float t_near = t_center - t_diff;
			const float t_far = t_center + t_diff;
			if (t_far >= 0.0f && t_near < dist) { // Да, именно так. Если дальняя точка впереди, пишем расстояние до ближней, даже если она позади.
				dist = t_near;
				const vec2 pos = ray.pos + dist * ray.dir;
				t = {info.thing, pos, pos, mat2(1.0f)};
			}
		}

		return t;
	}
};
