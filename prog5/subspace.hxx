#pragma once
#include <limits>
#include <vector>
#include "math.hxx"
#include "desc/riemann.hxx"

class Subspace;

/// Переход в другое пространство
struct Transition {
	Subspace *into;	///< Новое пространство
	vecd atPos;	///< Координаты точки перехода в исходном пространстве
	vecd intoPos;	///< Координаты точки перехода в новом пространстве
	matd jacobi;	///< Матрица преобразования векторов
};

struct BoundaryPoint: Transition {
	float distance;
};

/// Точка с направлением
struct alignas(32) Ray {
	vecd pos;	///< Положение точки
	vecd dir;	///< Направляющий вектор (обязательно единичный!)

	Ray() = default;
	Ray(Ray const &) = default;
	Ray(vecd pos, vecd dir)
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

	virtual std::vector<TraceResult> trace(std::vector<Ray> from) const {
		int nrays = from.size();
		std::vector<TraceResult> result;
		result.resize(nrays);
		for (int k = 0; k < nrays; k++)
			result[k] = trace(from[k]);
		return result;
	}
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
	virtual bool contains(vecd point) const = 0;
	virtual Transition leave(Ray at) const = 0;
};

class RiemannSubspace: public Subspace {
public:
#ifdef DIM
	static constexpr int dim = DIM;
#else
	static constexpr int dim = 2;
#endif
	static constexpr float dt = 0.1;
	static constexpr float eta = 0.2;
	inline static int large_steps = 0;
	inline static int regular_steps = 0;

	const SwitchMap *map;
	const RiemannMetric<dim> *metric;

	float length(vecd pos, vecd vec) const {
		matd g = metric->metric(pos);
		return sqrt(dot(vec, g * vec));
	}

	TraceResult trace(Ray from) const override {
		float t = 0.0f;
		vecd p = from.pos;
		vecd v = from.dir;
		v /=  length(p, v);
		while (map->contains(p)) {
			auto a = covar(metric->krist(p), v);
			if (dt * ::length(a) > eta) {
				large_steps++;
				int substeps = next_pow2(std::ceil(dt * ::length(a) / eta));
				float subdt = dt / substeps;
				assert(subdt * ::length(a) <= eta);
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
