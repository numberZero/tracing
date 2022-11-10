#pragma once
#include <limits>
#include <vector>
#include "math.hxx"
#include "desc/riemann.hxx"

class Subspace;

struct TrackPoint {
	const Subspace *space;
	vec2 pos;
	vec2 dir;
};

/// Общая для двух пространств точка и вектор в ней
struct SwitchPoint {
	TrackPoint from;	///< Запись в координатах исходного пространства
	TrackPoint to;	///< Запись в координатах следующего пространства
};

/// Участок пути, пролегающий по одному пространству
struct TrackSegment {
	const Subspace *space;
	std::vector<vec2> points;
	vec2 dirBegin;	///< Направление в начале участка (точка `points.front()`)
	vec2 dirEnd;	///< Направление в конце участка (точка `points.back()`)
	TrackPoint end;	///< Конец участка (в координатах следующего пространства)
};

class Subspace {
public:
	virtual TrackPoint trace(vec2 from, vec2 dir) const = 0;
	virtual TrackSegment traceEx(vec2 from, vec2 dir) const = 0;
};

/// Граница пространства
class SubspaceBoundary {
public:
	virtual SwitchPoint leave(vec2 from, vec2 dir) const = 0;
};

class FlatSubspace: public Subspace {
public:
	static constexpr float dt = 1e-1;
	SubspaceBoundary *boundary;

	TrackPoint trace(vec2 from, vec2 dir) const override {
		SwitchPoint sp = boundary->leave(from, dir);
		assert(!sp.from.space || sp.from.space == this);
		return sp.to;
	}

	TrackSegment traceEx(vec2 from, vec2 dir) const override {
		SwitchPoint sp = boundary->leave(from, dir);
		assert(!sp.from.space || sp.from.space == this);
		TrackSegment track;
		track.space = this;
		track.dirBegin = dir;
		track.dirEnd = sp.from.dir;
		track.end = sp.to;
		float len = distance(from, sp.from.pos);
		int n = ceil(len / dt);
		track.points.resize(n + 1);
		for (int k = 0; k <= n; k++) {
			float t = k / float(n);
			track.points[k] = mix(from, sp.from.pos, t);
		}
		if (!n) {
			track.points[0] = from;
		}
		return track;
	}
};

/// Карта окрестности искривлённого пространства
class SwitchMap {
public:
	virtual bool contains(vec2 point) const = 0;
	virtual TrackPoint leave(vec2 pos, vec2 dir) const = 0;
};

class RiemannSubspace: public Subspace {
public:
	static constexpr float dt = 1e-2;
	static constexpr float eta = 1e-2;

	const SwitchMap *map;
	const RiemannMetric<2> *metric;

	TrackPoint trace(vec2 from, vec2 dir) const override {
		TrackPoint end = trace(from, dir, [&](vec2 p, vec2 v) {
			return map->contains(p);
		});
		return map->leave(end.pos, end.dir);
	}

	TrackSegment traceEx(vec2 from, vec2 dir) const override {
		TrackSegment track;
		TrackPoint end = trace(from, dir, [&](vec2 p, vec2 v) {
			track.points.push_back(p);
			return map->contains(p);
		});
		track.points.shrink_to_fit();
		track.space = this;
		track.dirBegin = dir;
		track.dirEnd = end.dir;
		track.end = map->leave(end.pos, end.dir);
		return track;
	}

	float length(vec2 pos, vec2 vec) const {
		mat2 g = metric->metric(pos);
		return sqrt(dot(vec, g * vec));
	}

private:
	template <typename F>
	TrackPoint trace(vec2 from, vec2 dir, F &&pointCb) const {
		vec2 p = from;
		vec2 v = dir;
		v /=  length(p, v);

		while (pointCb(p, v)) {
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

		return {this, p, normalize(v)};
	}
};
