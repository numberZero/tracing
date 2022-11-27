#pragma once
#include "math.hxx"

template <int N>
class RiemannMetric {
public:
	using vec = vecn<N>;
	using mat = matn<N>;
	using tens = tensn<N>;
	using decomp = decompn<N>;

	virtual decomp halfmetric(vec pos) const = 0;

	mat metric(vec pos) const {
		decomp m = halfmetric(pos);
		m.diag *= m.diag;
		return m;
	}

	tens dmetric(vec pos, float eps = 1e-3f) const {
		tens result;
		for (int k = 0; k < N; k++) {
			vec delta = {};
			delta[k] = eps;
			result[k] = (metric(pos + delta) - metric(pos - delta)) / (2.0f * eps);
		}
		return result;
	}

	tens krist(vec pos) const {
		// Γ^i_k_l = .5 * g^i^m * (g_m_k,l + g_m_l,k - g_k_l,m)
		mat g = glm::inverse(metric(pos)); // с верхними индексами
		tens d = dmetric(pos);
		tens ret;
		// ret[i][l][k] = sum((m) => .5f * g[m][i] * (d[k][l][m] + d[l][k][m] - d[m][k][l]))
		for (int i = 0; i < N; i++)
		for (int l = 0; l < N; l++)
		for (int k = 0; k < N; k++) {
			float v = 0.0f;
			for (int m = 0; m < N; m++)
				v += g[m][i] * (d[l][k][m] + d[k][m][l] - d[m][k][l]);
			ret[i][l][k] = .5f * v;
		}
		return ret;
	}
};

template <int N>
class MovedRiemannMetric: public RiemannMetric<N> {
public:
	RiemannMetric<N> *base = nullptr;
	matn<N> inv_transform = matn<N>(1.0f); // must be orthogonal
	vecn<N> origin = vecn<N>(0.0f);

	decompn<N> halfmetric(vecn<N> pos) const override {
		decompn<N> d = base->halfmetric(inv_transform * (pos - origin));
		return {d.ortho * inv_transform, d.diag};
	}
};
