#include <cstdio>
#include <cmath>
#include <span>
#include <vector>
#include <libgen.h>
#include <unistd.h>
#include <glm/glm.hpp>
#include <png++/png.hpp>

using glm::vec2, glm::mat2;
using glm::dot;
using std::sqrt;

template <int N>
using vec = glm::vec<N, float>;

template <int N>
using mat = glm::mat<N, N, float>;

inline static float sqr(float x) {
	return x * x;
}

static mat2 metric(vec2 pos) { // с нижними индексами!
	return {
		{1.0f, 0.0f},
		{0.0f, sqr(pos.x)},
	};
}

template <int N>
struct tens {
	using mat = glm::mat<N, N, float>;
	mat data[N];

	mat &operator[] (int index) {
		return data[index];
	}
};
using tens2 = tens<2>;
using tens3 = tens<3>;

template <int N, typename T>
struct tensor_traits;

template <int N>
struct tensor_traits<N, float> {
	using type = float;
	using next = vec<N>;
};

template <int N>
struct tensor_traits<N, vec<N>> {
	using type = vec<N>;
	using next = mat<N>;
	using prev = float;
};

template <int N>
struct tensor_traits<N, mat<N>> {
	using type = mat<N>;
	using next = tens<N>;
	using prev = vec<N>;
};

template <int N, typename T>
static auto part_deriv(T f(vec<N>), vec<N> pos, float eps = 1e-3f) {
	typename tensor_traits<N, T>::next result;
	for (int k = 0; k < N; k++) {
		vec<N> delta = {};
		delta[k] = eps;
		result[k] = (f(pos + delta) - f(pos - delta)) / (2.0f * eps);
	}
	return result;
}

template <int N>
static tens<N> dmetric(vec<N> pos, float eps = 1e-3f) {
	return part_deriv(metric, pos, eps);
}

template <int N>
static tens<N> krist(vec<N> pos) {
	// Γ^i_k_l = .5 * g^i^m * (g_m_k,l + g_m_l,k - g_k_l,m)
	mat<N> g = glm::inverse(metric(pos)); // с верхними индексами
	tens<N> d = dmetric(pos);
	tens<N> ret;
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

template <int N>
static float length(vec<N> pos, vec<N> vec) {
	mat<N> g = metric(pos);
	return sqrt(dot(vec, g * vec));
}

template <int N>
static float length(std::span<vec<N>> line) {
	double d = 0.0;
	if (line.empty())
		return 0.0;
	vec<N> p1 = line[0];
	mat<N> g1 = metric(p1);
	for (auto p2: line.subspan(1)) {
		mat<N> g2 = metric(p2);
		vec<N> step = p2 - p1;
		float dd1 = sqrt(dot(step, g1 * step));
		float dd2 = sqrt(dot(step, g2 * step));
		d += 0.5f * (dd1 + dd2);
		p1 = p2;
		g1 = g2;
	}
	return d;
}

template <int N>
vec<N> covar(tens<N> G, vec<N> v) {
	vec<N> ret;
	for (int k = 0; k < N; k++)
		ret[k] = -dot(v, G[k] * v);
	return ret;
}

template <int N>
static std::vector<vec<N>> trace(vec<N> base, vec<N> dir, float distance, float dt = 1e-3) {
	int steps = distance / dt;
	std::vector<vec<N>> result;
	result.reserve(steps + 1);
	auto p = base;
	auto v = dir;
	v /=  length(p, v);
	result.push_back(p);
	for (int k = 0; k < steps; k++) {
		vec<N> a = covar(krist(p), v);
		v += dt * a;
		p += dt * v;
		result.push_back(p);
	}
	return result;
}

vec2 to_cartesian(vec2 polar) {
	return {polar.x * std::cos(polar.y), polar.x * std::sin(polar.y)};
}

int main() {
	std::vector<vec2> line;
	for (int k = 0; k <= 1024; k++) {
		vec2 xy = {3.0f, k / 256.0f};
		float r = glm::length(xy);
		line.emplace_back(r, std::atan2(xy.y, xy.x));
	}
	printf("4.0 = %.3f\n", length(std::span{line}));

	line = trace(vec2{3.0f, 0.0f}, vec2{0.0f, 1.0f}, 4.0f);
	for (vec2 p: line) {
		vec2 xy = to_cartesian(p);
		printf("%.3f, %.3f <- %.3f, %.3f\n", xy.x, xy.y, p.x, p.y);
	}
	printf("%.3f = %.3f\n", glm::distance(to_cartesian(line.front()), to_cartesian(line.back())), length(std::span{line}));
}
