#pragma once
#include <limits>
#include <vector>
#include "math.hxx"
#include "desc/riemann.hxx"

class Subspace;

struct TrackPoint {
	Subspace *space;
	vec2 pos;
	vec2 dir;
};

class Subspace {
public:
	virtual TrackPoint trace(vec2 from, vec2 dir) const = 0;
};

/// Выход из плоского участка пространства.
class FlatExit {
public:
	virtual float distance(vec2 from, vec2 dir) const = 0;
	virtual TrackPoint leave(vec2 pos, vec2 dir) const = 0;
};

class FlatSubspace: public Subspace {
public:
	std::vector<FlatExit *> exits;

	TrackPoint trace(vec2 from, vec2 dir) const override {
		float dist = std::numeric_limits<float>::infinity();
		FlatExit *exit = nullptr;
		for (auto *exit2: exits) {
			float dist2 = exit2->distance(from, dir);
			if (dist2 < 0.0f)
				continue;
			if (dist2 < dist) {
				exit = exit2;
				dist = dist2;
			}
		}
		if (exit) {
			from += dist * dir;
			return exit->leave(from, dir);
		} else {
			return {nullptr, vec2(std::numeric_limits<float>::quiet_NaN()), dir};
		}
	}
};

/// Карта окрестности искривлённого участка пространства.
class SwitchMap {
public:
	virtual bool contains(vec2 point) const = 0;
	virtual TrackPoint leave(vec2 pos, vec2 dir) const = 0;
};

class RiemannSubspace: public Subspace {
public:
	SwitchMap *map;
	RiemannMetric<2> *metric;

	TrackPoint trace(vec2 from, vec2 dir) const override {
		static constexpr float dt = 1e-2;
		static constexpr float eta = 1e-2;

		vec2 p = from;
		vec2 v = dir;
		v /=  length(p, v);
		while (map->contains(p)) {
			auto a = covar(metric->krist(p), v);
			if (dt * ::length(a) > eta) {
				int substeps = ceil(dt * ::length(a) / eta);
				substeps |= substeps >> 16;
				substeps |= substeps >> 8;
				substeps |= substeps >> 4;
				substeps |= substeps >> 2;
				substeps |= substeps >> 1;
				substeps++;
				float subdt = dt / substeps;
				for (int l = 0; l < substeps; l++) {
					auto a = covar(metric->krist(p), v);
					v += subdt * a;
					p += subdt * v;
				}
			} else {
				v += dt * a;
				p += dt * v;
			}
		}
		return map->leave(p, normalize(v));
	}

	float length(vec2 pos, vec2 vec) const {
		mat2 g = metric->metric(pos);
		return sqrt(dot(vec, g * vec));
	}
};
