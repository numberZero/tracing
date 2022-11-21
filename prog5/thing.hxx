#pragma once
#include <cmath>
#include <limits>
#include "prog5/subspace.hxx"

class ThingySubspace;

struct Location {
	ThingySubspace const *space;
	vec2 pos; ///< Положение (в координатах пространства @p space)
	mat2 rot; ///< Матрица поворота (ортогональная!)
};

class SubspaceBoundaryEx: public SubspaceBoundary, public SwitchMap {
public:
	virtual std::vector<Transition> findOverlaps(vec2 pos, float max_distance) const = 0;
};

class Thing {
public:
	Location loc; ///< Текущее положение центра

	virtual float hit(Ray ray) const = 0;
};

struct ThingInfo {
	Thing const *thing;
	vec2 pos;
	float radius;
};

struct ThingTraceResult {
	Thing const *thing = nullptr;
	Ray incident;
	float distance;
};

class ThingySubspace: public Subspace {
public:
	SubspaceBoundaryEx *boundary;
	std::vector<ThingInfo> things;

	TraceResult trace(Ray ray) const final override {
		return thingless().trace(ray);
	}

	std::vector<ThingInfo> findThingsOnRay(Ray ray) const {
		std::vector<ThingInfo> result;
		Transition t = boundary->findBoundary(ray);
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
			if (t_far >= 0.0f && t_near < dist) {
				result.push_back(info);
			}
		}
		return result;
	}

	ThingTraceResult traceToThing(Ray ray) const {
		ThingTraceResult result;
		Transition t = boundary->findBoundary(ray);
		float max_dist = t.into ? distance(ray.pos, t.atPos) : std::numeric_limits<float>::infinity();
		for (auto &&info: findThingsOnRay(ray)) {
			const float dist = info.thing->hit({ray.pos - info.pos, ray.dir});
			if (dist < max_dist) {
				max_dist = dist;
				const vec2 pos = ray.pos + dist * ray.dir;
				result = {
					.thing = info.thing,
					.incident = {pos, ray.dir},
					.distance = dist,
				};
			}
		}
		return result;
	}

private:
	FlatSubspace thingless() const {
		FlatSubspace result;
		result.boundary = boundary;
		return result;
	}
};

struct ThingLocation {
	ThingySubspace *space;
	vec2 pos;
};
