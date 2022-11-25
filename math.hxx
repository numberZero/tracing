#pragma once
#include <glm/glm.hpp>

using namespace glm;

template <int N>
using vecn = glm::vec<N, float>;

template <int N>
using matn = glm::mat<N, N, float>;

template <int N>
matn<N> diagonal(vecn<N> v) {
	matn<N> ret{0.0f};
	for (int k = 0; k < N; k++)
		ret[k][k] = v[k];
	return ret;
}

auto diagonal(float x, float y) {
	return diagonal(vecn<2> {x, y});
}

auto diagonal(float x, float y, float z) {
	return diagonal(vecn<3>{x, y, z});
}

auto diagonal(float x, float y, float z, float w) {
	return diagonal(vecn<4>{x, y, z, w});
}

template <typename T>
inline static T sqr(T x) {
	return x * x;
}

inline static float smoothstep(float x) {
	return 3 * x * x - 2 * x * x * x;
}

inline static float box(float val, vec2 range, float pad) {
	float slope1 = 1.0f + (val - range.x) / pad;
	float slope2 = 1.0f - (val - range.y) / pad;
	float lin = std::min(slope1, slope2);
	return smoothstep(glm::clamp(lin, 0.0f, 1.0f));
}

template <int N>
struct tensn {
	using mat = matn<N>;
	mat data[N];

	mat &operator[] (int index) {
		return data[index];
	}
};
using tens2 = tensn<2>;
using tens3 = tensn<3>;

template <int N>
vecn<N> covar(tensn<N> G, vecn<N> v) {
	vecn<N> ret;
	for (int k = 0; k < N; k++)
		ret[k] = -dot(v, G[k] * v);
	return ret;
}

// Decomposed (O^−1 D O) symmetrical matrix
template <int N>
struct decompn {
	matn<N> ortho; // must be orthogonal
	vecn<N> diag; // eigenvalues

	operator matn<N> () const {
		return transpose(ortho) * diagonal(diag) * ortho;
	}
};

using decomp2 = decompn<2>;
using decomp3 = decompn<3>;

inline static uint32_t next_pow2(uint32_t v) {
	v--;
	v |= v >> 1;
	v |= v >> 2;
	v |= v >> 4;
	v |= v >> 8;
	v |= v >> 16;
	v++;
	return v;
}

inline static vec2 cross(vec2 v) {
	return {-v.y, v.x};
}
