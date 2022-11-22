#pragma once
#include <limits>
#include <vector>
#include "math.hxx"
#include "desc/riemann.hxx"

class Subspace;

/// Переход в другое пространство
struct Transition {
	Subspace *into;	///< Новое пространство
	vec2 atPos;	///< Координаты точки перехода в исходном пространстве
	vec2 intoPos;	///< Координаты точки перехода в новом пространстве
	mat2 jacobi;	///< Матрица преобразования векторов
};

struct BoundaryPoint: Transition {
	float distance;
};

/// Точка с направлением
struct Ray {
	vec2 pos;	///< Положение точки
	vec2 dir;	///< Направляющий вектор (обязательно единичный!)

	Ray() = default;
	Ray(Ray const &) = default;
	Ray(vec2 pos, vec2 dir)
		: pos(pos)
		, dir(normalize(dir))
	{
	}
};

struct TrackPoint: Ray {
	const Subspace *space;
};

struct TraceResult {
	Ray end;
	TrackPoint to;
	float distance;
};

class Subspace {
public:
	virtual TraceResult trace(Ray from) const = 0;
};

/// Граница пространства
class SubspaceBoundary {
public:
	virtual BoundaryPoint findBoundary(Ray from) const = 0;
};

class FlatSubspace: public Subspace {
public:
	static constexpr float dt = 1e-1;
	SubspaceBoundary *boundary;

	TraceResult trace(Ray from) const override {
		auto t = boundary->findBoundary(from);
		return {
			{t.atPos, from.dir},
			{{t.intoPos, normalize(t.jacobi * from.dir)}, t.into},
			t.distance,
		};
	}
};

/// Карта окрестности искривлённого пространства
class SwitchMap {
public:
	virtual bool contains(vec2 point) const = 0;
	virtual Transition leave(Ray at) const = 0;
};

class RiemannSubspace: public Subspace {
public:
	static constexpr float dt = 0.01;
	static constexpr float eta = 0.02;
	inline static int large_steps = 0;
	inline static int regular_steps = 0;

	const SwitchMap *map;
	const RiemannMetric<2> *metric;

	float length(vec2 pos, vec2 vec) const {
		mat2 g = metric->metric(pos);
		return sqrt(dot(vec, g * vec));
	}

	TraceResult trace(Ray from) const override {
		float t = 0.0f;
		vec2 p = from.pos;
		vec2 v = from.dir;
		v /=  length(p, v);
		while (map->contains(p)) {
			auto a = covar(metric->krist(p), v);
			if (dt * ::length(a) > eta) {
				large_steps++;
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
				regular_steps++;
				v += dt * a;
				p += dt * v;
			}
			t += dt;
		}
		Ray end = {p, normalize(v)};
		return {end, leave(end), t};
	}

	TrackPoint leave(Ray at) const {
		Transition t = map->leave(at);
		return {{t.intoPos, normalize(t.jacobi * at.dir)}, t.into};
	}
};
