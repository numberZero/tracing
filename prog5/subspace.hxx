#pragma once
#include <limits>
#include <vector>
#include "math.hxx"
#include "desc/riemann.hxx"

class Subspace;

/// Переход в другое пространство
struct Transition {
	const Subspace *into;	///< Новое пространство
	vec2 atPos;	///< Координаты точки перехода в исходном пространстве
	vec2 intoPos;	///< Координаты точки перехода в новом пространстве
	mat2 jacobi;	///< Матрица преобразования векторов
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

/// Точка перехода между пространствами
struct SwitchPoint {
	TrackPoint from;
	TrackPoint to;
};

class Subspace {
public:
	virtual SwitchPoint trace(Ray from) const = 0;
};

/// Граница пространства
class SubspaceBoundary {
public:
	virtual Transition findBoundary(Ray from) const = 0;
};

class FlatSubspace: public Subspace {
public:
	static constexpr float dt = 1e-1;
	SubspaceBoundary *boundary;

	SwitchPoint trace(Ray from) const override {
		Transition t = boundary->findBoundary(from);
		return {
			{{t.atPos, from.dir}, this},
			{{t.intoPos, normalize(t.jacobi * from.dir)}, t.into},
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
	static constexpr float dt = 1e-2;
	static constexpr float eta = 1e-2;

	const SwitchMap *map;
	const RiemannMetric<2> *metric;

	SwitchPoint trace(Ray from) const override {
		Ray end = trace(from, [&](vec2 p, vec2 v) {
			return map->contains(p);
		});
		return {{end, this}, leave(end)};
	}

	float length(vec2 pos, vec2 vec) const {
		mat2 g = metric->metric(pos);
		return sqrt(dot(vec, g * vec));
	}

private:
	template <typename F>
	Ray trace(Ray from, F &&pointCb) const {
		vec2 p = from.pos;
		vec2 v = from.dir;
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

		return {p, normalize(v)};
	}

	TrackPoint leave(Ray at) const {
		Transition t = map->leave(at);
		return {{t.intoPos, normalize(t.jacobi * at.dir)}, t.into};
	}
};
