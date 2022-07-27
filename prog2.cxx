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

inline static float sqr(float x) {
	return x * x;
}

static mat2 metric(vec2 pos) { // с нижними индексами!
	return {
		{1.0f, 0.0f},
		{0.0f, sqr(pos.x)},
	};
}

struct tens2 {
	mat2 x, y;
};

static tens2 dmetric(vec2 pos, float eps = 1e-3f) {
	mat2 xp = metric(pos + vec2(eps, 0.0f));
	mat2 xn = metric(pos - vec2(eps, 0.0f));
	mat2 yp = metric(pos + vec2(0.0f, eps));
	mat2 yn = metric(pos - vec2(0.0f, eps));
	return {
		(xp - xn) / (2.0f * eps),
		(yp - yn) / (2.0f * eps),
	};
}
/*
static tens2 dmetric(vec2 pos) {
	return {
		{0.0f, 0.0f, 0.0f, 2 * pos.x},
		{0.0f, 0.0f, 0.0f, 0.0f},
	};
}*/

static tens2 krist(vec2 pos) {
	mat2 g = glm::inverse(metric(pos)); // с верхними индексами
	tens2 d = dmetric(pos);
	// Γ^i_k_l = .5 * g^i^m * (g_m_k,l + g_m_l,k - g_k_l,m)
	// ret[i][l][k] = sum((m) => .5f * g[m][i] * (d[m][k][l] + d[m][l][k] - d[k][l][m]))
	// Γ^i_k_l = .5 * g^i^x * (g_x_k,l + g_x_l,k - g_k_l,x) + .5 * g^i^y * (g_y_k,l + g_y_l,k - g_k_l,y)
	// ret.i[l].k = .5f * g[0].i * (d.x[k].l + d.x[l].k - d.k[l].x) + .5f * g[1].i * (d.y[k].l + d.y[l].k - d.k[l].y);
	return {{
		.5f * g[0].x * (d.x[0].x + d.x[0].x - d.x[0].x) + .5f * g[1].x * (d.x[0].y + d.x[0].y - d.y[0].x),
		.5f * g[0].x * (d.x[1].x + d.y[0].x - d.x[0].y) + .5f * g[1].x * (d.x[1].y + d.y[0].y - d.y[0].y),
		.5f * g[0].x * (d.y[0].x + d.x[1].x - d.x[1].x) + .5f * g[1].x * (d.y[0].y + d.x[1].y - d.y[1].x),
		.5f * g[0].x * (d.y[1].x + d.y[1].x - d.x[1].y) + .5f * g[1].x * (d.y[1].y + d.y[1].y - d.y[1].y),
	},{
		.5f * g[0].y * (d.x[0].x + d.x[0].x - d.x[0].x) + .5f * g[1].y * (d.x[0].y + d.x[0].y - d.y[0].x),
		.5f * g[0].y * (d.x[1].x + d.y[0].x - d.x[0].y) + .5f * g[1].y * (d.x[1].y + d.y[0].y - d.y[0].y),
		.5f * g[0].y * (d.y[0].x + d.x[1].x - d.x[1].x) + .5f * g[1].y * (d.y[0].y + d.x[1].y - d.y[1].x),
		.5f * g[0].y * (d.y[1].x + d.y[1].x - d.x[1].y) + .5f * g[1].y * (d.y[1].y + d.y[1].y - d.y[1].y),
	}};
}

static float length(vec2 pos, vec2 vec) {
	mat2 g = metric(pos);
	return sqrt(dot(vec, g * vec));
}

static float length(std::span<vec2> line) {
	double d = 0.0;
	if (line.empty())
		return 0.0;
	vec2 p1 = line[0];
	mat2 g1 = metric(p1);
	for (auto p2: line.subspan(1)) {
		mat2 g2 = metric(p2);
		vec2 step = p2 - p1;
		float dd1 = sqrt(dot(step, g1 * step));
		float dd2 = sqrt(dot(step, g2 * step));
		d += 0.5f * (dd1 + dd2);
		p1 = p2;
		g1 = g2;
	}
	return d;
}

static std::vector<vec2> trace(vec2 base, vec2 dir, float distance, float dt = 1e-3) {
	int steps = distance / dt;
	std::vector<vec2> result;
	result.reserve(steps + 1);
	vec2 p = base;
	vec2 v = dir;
	v /=  length(p, v);
	result.push_back(p);
	for (int k = 0; k < steps; k++) {
		tens2 G = krist(p);
		vec2 a = {-dot(v, G.x * v), -dot(v, G.y * v)};
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
	printf("4.0 = %.3f\n", length({line}));

	line = trace({3.0f, 0.0f}, {0.0f, 1.0f}, 4.0f);
	for (vec2 p: line) {
		vec2 xy = to_cartesian(p);
		printf("%.3f, %.3f <- %.3f, %.3f\n", xy.x, xy.y, p.x, p.y);
	}
	printf("%.3f = %.3f\n", glm::distance(to_cartesian(line.front()), to_cartesian(line.back())), length({line}));
}
