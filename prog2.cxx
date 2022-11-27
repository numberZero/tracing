#include <cstdio>
#include <cmath>
#include <span>
#include <vector>
#include <libgen.h>
#include <unistd.h>
#include <glm/glm.hpp>
#include <png++/png.hpp>

#define COIL 1

using glm::vec2, glm::mat2;
using glm::vec3, glm::mat3;
using glm::vec4, glm::mat4;
using glm::dot;
using std::sqrt;

template <int N>
using vecd = glm::vec<N, float>;

template <int N>
using mat = glm::mat<N, N, float>;

template <int N>
mat<N> diagonal(vecd<N> v) {
	mat<N> ret{0.0f};
	for (int k = 0; k < N; k++)
		ret[k][k] = v[k];
	return ret;
}

auto diagonal(float x, float y) {
	return diagonal(vecd<2> {x, y});
}

auto diagonal(float x, float y, float z) {
	return diagonal(vecd<3>{x, y, z});
}

auto diagonal(float x, float y, float z, float w) {
	return diagonal(vecd<4>{x, y, z, w});
}

inline static float sqr(float x) {
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

// Decomposed (O^−1 D O) symmetrical matrix
struct decomp2 {
	mat2 ortho; // must be orthogonal
	vec2 diag; // eigenvalues
};

#if COIL

static const float coil_scale = 3.0f;
static const float coil_r = 3.0f;
static const float coil_w = 0.5f;
static const float coil_m = 0.1f;

static decomp2 halfmetric(vec2 pos) {
	float r = glm::length(pos);
	vec2 dir = glm::normalize(pos);

	float s = box(r, {coil_r - coil_w, coil_r + coil_w}, coil_m);
	float t = glm::mix(1.0f, coil_r/r/coil_scale, s);

	return {
		.ortho = {
			dir.x, -dir.y,
			dir.y, dir.x,
		},
		.diag = {1.0f, t},
	};
}

#else

static const float box_scale = 3.0f;
static const vec2 box_a = {-0.3f, -3.0f};
static const vec2 box_b = {0.3f, 3.0f};
static const vec2 box_m = {0.1f, 0.5f};

static decomp2 halfmetric(vec2 pos) {
	float s = box(pos.x, {box_a.x, box_b.x}, box_m.x) * box(pos.y, {box_a.y, box_b.y}, box_m.y);
	return {
		.ortho = mat2(1.f),
		.diag = {1.f, pow(box_scale, -s)},
	};
}

#endif

static mat2 metric(vec2 pos) { // с нижними индексами!
	decomp2 h = halfmetric(pos);
	return glm::transpose(h.ortho) * diagonal(h.diag * h.diag) * h.ortho;
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
	using next = vecd<N>;
};

template <int N>
struct tensor_traits<N, vecd<N>> {
	using type = vecd<N>;
	using next = mat<N>;
	using prev = float;
};

template <int N>
struct tensor_traits<N, mat<N>> {
	using type = mat<N>;
	using next = tens<N>;
	using prev = vecd<N>;
};

template <int N, typename T>
static auto part_deriv(T f(vecd<N>), vecd<N> pos, float eps = 1e-3f) {
	typename tensor_traits<N, T>::next result;
	for (int k = 0; k < N; k++) {
		vecd<N> delta = {};
		delta[k] = eps;
		result[k] = (f(pos + delta) - f(pos - delta)) / (2.0f * eps);
	}
	return result;
}

template <int N>
static tens<N> dmetric(vecd<N> pos, float eps = 1e-3f) {
	return part_deriv(metric, pos, eps);
}

template <int N>
static tens<N> krist(vecd<N> pos) {
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
static float length(vecd<N> pos, vecd<N> vec) {
	mat<N> g = metric(pos);
	return sqrt(dot(vec, g * vec));
}

template <int N>
static float length(std::span<vecd<N>> line) {
	double d = 0.0;
	if (line.empty())
		return 0.0;
	vecd<N> p1 = line[0];
	mat<N> g1 = metric(p1);
	for (auto p2: line.subspan(1)) {
		mat<N> g2 = metric(p2);
		vecd<N> step = p2 - p1;
		float dd1 = sqrt(dot(step, g1 * step));
		float dd2 = sqrt(dot(step, g2 * step));
		d += 0.5f * (dd1 + dd2);
		p1 = p2;
		g1 = g2;
	}
	return d;
}

template <int N>
vecd<N> covar(tens<N> G, vecd<N> v) {
	vecd<N> ret;
	for (int k = 0; k < N; k++)
		ret[k] = -dot(v, G[k] * v);
	return ret;
}

struct ConvexShape {
	std::vector<vec2> rel_points;
	std::vector<vec2> normals;
	std::vector<vec2> abs_points;
	std::vector<vec3> colliders;
	vec2 pos;
};

ConvexShape place_shape(vec2 p, std::span<vec2 const> contour) {
	auto h = halfmetric(p);
	mat2 m = transpose(h.ortho) * diagonal(1.f / h.diag) * h.ortho;
	ConvexShape ret;

	ret.pos = p;

	ret.rel_points.insert(end(ret.rel_points), begin(contour), end(contour));

	ret.normals.reserve(contour.size());
	vec2 a = ret.rel_points.back();
	for (vec2 b: ret.rel_points) {
		vec2 delta = b - a;
		vec2 n = normalize(vec2{delta.y, -delta.x});
		ret.normals.push_back(n);
		a = b;
	}

	for (vec2 off: contour)
		ret.abs_points.push_back(p + m * off);

	ret.colliders.reserve(contour.size());
	a = ret.abs_points.back();
	for (vec2 b: ret.abs_points) {
		vec2 delta = b - a;
		vec2 n = normalize(vec2{delta.y, -delta.x});
		ret.colliders.push_back({-n, dot(n, a)});
		a = b;
	}

	return ret;
}

static auto const ts1 = place_shape({0, coil_r}, {{{-.125, -.125}, {.000, .125}, {.125, -.125}}});

static vec2 globalize(vec2 p, vec2 v) {
	auto h = halfmetric(p);
	return transpose(h.ortho) * diagonal(1.f / h.diag) * h.ortho * v;
}

template <int N>
static std::vector<vecd<N>> trace(vecd<N> base, vecd<N> dir, float distance, float dt = 1e-3) {
	int steps = distance / dt;
	std::vector<vecd<N>> result;
	result.reserve(steps + 1);
	auto p = base;
	auto v = globalize(base, dir);
	v /=  length(p, v);
	result.push_back(p);
	for (int k = 0; k < steps; k++) {
		vecd<N> a = covar(krist(p), v);
		v += dt * a;
		p += dt * v;
/*
		for (auto cdr: ts1.colliders) {
			if (dot({p2, 1.f}, cdr) > 0.f)
				goto no_collision;
		}

		for (int k = 0; k < ts1.colliders.size(); k++) {
			if (dot({p, 1.f}, ts1.colliders[k]) > 0.f) {
				vec2 n = ts1.normals[k];
				goto collision_processed;
			}
		}
		printf("WARNING: Collided part not found!\n");*/

// 	collision_processed:
// 		break;
/*
		static vec2 const pp = vec2(-2, 2);
		static vec2 const n = normalize(coglobalize(pp, vec2(1)));
		static vec3 const nn = {n, -dot(n, pp)};
		if (dot({p2, 1.f}, nn) <= 0 && dot({p, 1.f}, nn) > 0) {
			float t0 = -dot({p, 1.f}, nn) / dot({v, 0.f}, nn);
			vec2 p0 = p + t0 * v;
// 			if (t0 > dt * 1.000001)
// 				printf("p(%.6f, %.6f) p2(%.6f, %.6f) p0(%.6f, %.6f) dt(%.6f) t0(%.6f)\n", p.x, p.y, p2.x, p2.y, p0.x, p0.y, dt, t0);
			if (p0.x > pp.x - .2 && p0.x < pp.x + .1) {
				p2 = p;
			}
		}
*/
// 	no_collision:
// 		p = p2;
		result.push_back(p);
	}
	return result;
}

vec2 to_cartesian(vec2 polar) {
	return {polar.x * std::cos(polar.y), polar.x * std::sin(polar.y)};
}

void draw_path(FILE *svg, char const *cls, std::span<vec2 const> line, bool close = false, int skip = 1) {
	fprintf(svg, "<path class=\"%s\" d=\"M", cls);
	int n = 0;
	for (vec2 p: line) {
		if (n % skip == 0)
			fprintf(svg, " %.3f,%.3f", p.x, p.y);
		if (!n)
			fprintf(svg, " L");
		n++;
	}
	if (close)
		fprintf(svg, " Z");
	fprintf(svg, "\" />\n");
}

void draw_box(FILE *svg, char const *cls, vec2 a, vec2 b) {
	draw_path(svg, cls, {std::vector{a, {a.x, b.y}, b, {b.x, a.y}}}, true);
}

int main() {
	FILE *svg = fopen("out.svg", "wb");
	fprintf(svg, "<?xml version=\"1.0\" encoding=\"utf-8\" ?>\n");
	fprintf(svg, "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"-6 -4 12 8\">");
	fprintf(svg, "<style>@import url(style.css);</style>");
	for (int j = -64; j <= 64; j++) {
		float y = j / 64.0f;
		printf("%d: %.3f\n", j, y);
		auto line = trace(vec2{-5.0f, 0.0f}, vec2{1.0f, y},20.0f);
		draw_path(svg, "ray", {line}, false, 10);
	}
#if COIL
	for (int j = -64; j <= 64; j++) {
		float x = j / 64.0f;
		printf("%d: %.3f\n", j, x);
		auto line = trace(vec2{coil_r, 0}, vec2{x, 1.0f}, 6.28f * coil_r / coil_scale);
		draw_path(svg, "ray2", {line}, false, 10);
	}
	fprintf(svg, "<circle class=\"inner\" r=\"%.3f\" />", coil_r - coil_w);
	fprintf(svg, "<circle class=\"inner\" r=\"%.3f\" />", coil_r + coil_w);
	fprintf(svg, "<circle class=\"outer\" r=\"%.3f\" />", coil_r - coil_w - coil_m);
	fprintf(svg, "<circle class=\"outer\" r=\"%.3f\" />", coil_r + coil_w + coil_m);
#else
	for (int j = -64; j <= 64; j++) {
		float x = j / 64.0f;
		printf("%d: %.3f\n", j, x);
		auto line = trace(vec2{0.5f * box_a.x + 0.5f * box_b.x, 0.7f * box_a.y + 0.3f * box_b.y}, vec2{x, 1.0f}, 10.0f);
		draw_path(svg, "ray2", {line}, false, 10);
	}
	draw_box(svg, "inner", box_a, box_b);
	draw_box(svg, "outer", box_a - box_m, box_b + box_m);
#endif
	for (int j = -8; j <= 8; j++) {
		float x = j;
		draw_path(svg, j ? "grid" : "grid major", {{{-8.f, x}, {8.f, x}}}, false);
		draw_path(svg, j ? "grid" : "grid major", {{{x, -8.f}, {x, 8.f}}}, false);
	}
	for (int j = -64; j <= 64; j++)
	for (int i = -64; i <= 64; i++) {
		vec2 p = vec2(i, j) / 8.f;
		decomp2 h = halfmetric(p);
		mat2 s = transpose(h.ortho) * diagonal(1.f / h.diag) / 64.f;
		draw_path(svg, "mark", {{p + s[0], p + s[1], p - s[0], p - s[1]}}, true);
	}
	draw_path(svg, "object", ts1.abs_points, true);
	fprintf(svg, "</svg>");
	fclose(svg);
}
