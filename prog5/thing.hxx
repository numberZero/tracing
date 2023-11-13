#pragma once
#include <cmath>
#include <limits>
#include "prog5/subspace.hxx"

class ThingySubspace;

struct Location {
	ThingySubspace const *space;
	vecd pos; ///< Положение (в координатах пространства @p space)
	matd rot; ///< Матрица поворота (ортогональная!)
};

class SubspaceBoundaryEx: public SubspaceBoundary, public SwitchMap {
public:
	virtual std::vector<Transition> findOverlaps(vecd pos, float max_distance) const = 0;
};

class Thing {
public:
	Location loc; ///< Текущее положение центра
	uint32_t id;
	virtual float hit(Ray ray) const = 0;
	virtual float getRadius() const noexcept = 0;
	void move(vecd off);
	void rotate(matd rot);
};

struct ThingInfo {
	Thing const *thing;
	vecd pos;
	matd rot;
	float radius;
};

struct ThingTraceResult {
	Thing const *thing = nullptr;
	Ray incident;
	Ray thingspace_incident;
	float distance;
};

class ThingySubspace: public Subspace {
	friend class Universe;

public:
	SubspaceBoundaryEx *boundary;

	TraceResult trace(Ray ray) const final override {
		return thingless().trace(ray);
	}

	std::vector<ThingInfo> findThingsOnRay(Ray ray) const {
		std::vector<ThingInfo> result;
		Transition t = boundary->findBoundary(ray);
		float dist = t.into ? distance(ray.pos, t.atPos) : std::numeric_limits<float>::infinity();
		for (auto &&info: things) {
			const vecd rel = info.pos - ray.pos;
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
			const matd rot = transpose(info.rot);
			const float dist = info.thing->hit({rot * (ray.pos - info.pos), rot * ray.dir});
			if (dist < max_dist) {
				max_dist = dist;
				const vecd pos = ray.pos + dist * ray.dir;
				result = {
					.thing = info.thing,
					.incident = {pos, ray.dir},
					.thingspace_incident = {rot * (pos - info.pos), rot * ray.dir},
					.distance = dist,
				};
			}
		}
		return result;
	}

// private:
	mutable std::vector<ThingInfo> things;

	FlatSubspace thingless() const {
		FlatSubspace result;
		result.boundary = boundary;
		return result;
	}
};

/// Сдвигает объект на @p off, в координатах объекта
inline void Thing::move(vecd off) {
	loc.pos += loc.rot * off;
	while (!loc.space->boundary->contains(loc.pos)) {
		auto next = loc.space->boundary->leave({loc.pos, off});
		if (auto flat = dynamic_cast<ThingySubspace *>(next.into)) {
			loc.space = flat;
			loc.pos = next.intoPos;
			loc.rot = next.jacobi * loc.rot;
		} else {
			throw "Oops! A thing is destroyed by the space curvature";
		}
	}
}

/// Доворачивает объект на @p rot, в координатах объекта
inline void Thing::rotate(matd rot)
{
	loc.rot *= rot;
}
