#pragma once
#include <cmath>
#include <limits>
#include "prog5/subspace.hxx"

struct ThingInfo {
	FlatSubspace const *thing;
	vec2 pos;
	float radius;
};

class ThingyFlatSubspace: public FlatSubspace {
public:
	std::vector<ThingInfo> things;

	SwitchPoint trace(Ray ray) const override {
		SwitchPoint sp = FlatSubspace::trace(ray);
		FlatSubspace const *thing = nullptr;
		float dist = sp.to.space ? distance(ray.pos, sp.from.pos) : std::numeric_limits<float>::infinity();

		for (auto &&info: things) {
			const vec2 rel = info.pos - ray.pos;
			const float t_center = dot(rel, ray.dir);
			const float d2 = dot(rel, rel) - sqr(t_center);
			if (d2 > sqr(info.radius))
				continue;
			const float t_diff = std::sqrt(sqr(info.radius) - d2);
			const float t_near = t_center - t_diff;
			const float t_far = t_center + t_diff;
			if (t_far >= 0.0f && t_near < dist) {
				// Да, именно так. Если дальняя точка впереди, пишем расстояние до ближней, даже если она позади.
				dist = t_near;
				thing = info.thing;
			}
		}

		if (thing) {
			sp.from.pos = ray.pos + dist * ray.dir;
			sp.to = {Ray(sp.from), thing};
		}

		return sp;
	}
};
