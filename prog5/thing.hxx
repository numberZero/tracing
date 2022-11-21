#pragma once
#include <cmath>
#include <limits>
#include "prog5/subspace.hxx"

struct Location {
	FlatSubspace const *space;
	vec2 pos; ///< Положение (в координатах пространства @p space)
	mat2 rot; ///< Матрица поворота (ортогональная!)
};

class SubspaceBoundaryEx: public SubspaceBoundary, public SwitchMap {
public:
	virtual std::vector<Transition> findOverlaps(vec2 pos, float max_distance) const = 0;
};

class Thing {
public:
	float radius;
	Location loc; ///< Текущее положение центра
};

struct ThingInfo {
	FlatSubspace const *thing;
	vec2 pos;
	float radius;
};

class ThingyBoundary: public SubspaceBoundaryEx {
public:
	SubspaceBoundaryEx *base;
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

	std::vector<Transition> findOverlaps(vec2 at, float max_distance) const override {
		return base->findOverlaps(at, max_distance);
	}

	bool contains(vec2 point) const override {
		return base->contains(point);
	}

	Transition leave(Ray at) const override {
		return base->leave(at);
	}
};

struct ThingLocation {
	ThingyBoundary *space;
	vec2 pos;
};
