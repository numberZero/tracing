#include <cassert>
#include <cstdio>
#include <memory>
#include <limits>
#include <random>
#include <thread>
#include <unordered_map>
#include <vector>
#include <asyncpp/generator.h>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <GLFW/glfw3.h>
#include <GL/gl.h>
#include <GL/glext.h>
#include "averager.hxx"
#include "math.hxx"
#include "texture.hxx"
#include "shader.hxx"
#include "io.hxx"
#include "prog5/glshape.hxx"
#include "prog5/gpu.hxx"
#include "prog5/subspace.hxx"
#include "prog5/thing.hxx"
#include "prog5/universe.hxx"
#include "prog5/visual.hxx"
#include "iters.hxx"

#define TEST 0
// #define debugf(...) printf(__VA_ARGS__)
#define debugf(...)

using namespace std::literals;

struct Params {
	float outer_radius = 5.0f;
	float inner_radius = 4.0f;
	float outer_half_length = 1500.0f;
	float inner_half_length = 100.0f;
	float inner_pad = 0.25f;
};

struct Coefs {
	float x1, y1, x2, y2;
	float x0, y0, w;

	Coefs(Params const &params) {
		static constexpr float eps = 1e-3;

		x2 = params.inner_half_length;
		y2 = params.outer_half_length;
		x1 = params.inner_half_length * (1.0f - params.inner_pad);

		if (y2 - x2 < eps) {
			if (y2 - x2 < -eps)
				throw "Invalid channel properties";
			x1 = y1 = x2 = y2;
			x0 = y0 = w = std::numeric_limits<float>::signaling_NaN();
			return;
		}

		y1 = x1 * (x1 - x2 + 2*y2) / (x1 + x2);
		x0 = 0.5 * (sqr(x2) + sqr(x1) - 2 * x2 * y2) / (x2 - y2);
		y0 = y2 - 0.25 * (sqr(x2) - sqr(x1)) / (x2 - y2);
		w = (x2 - y2) / (sqr(x2) - sqr(x1));
	}
};

static constexpr float eps = 1e-4;

enum class Side {
	Outside = 1,
	Inside = -1,
};

struct CylinderDist {
	float cap, side;
};

static CylinderDist cylinderDist(Ray from, float halflength, float radius, Side side = Side::Outside) {
	CylinderDist dist{std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity()};

	{ // Крышки цилиндра
		const float d = -int(side) * sign(from.dir.x) * halflength - from.pos.x;
		const float t = d / from.dir.x;
		const vec3 p = from.pos + t * from.dir;
		if (t > 0.0f && length(vec2{p.y, p.z}) <= radius)
			dist.cap = t - eps;
	}

	{ // Боковая стенка цилиндра
		const vec2 pos = {from.pos.y, from.pos.z};
		const vec2 dir = {from.dir.y, from.dir.z}; // не единичный!
		const float t_center = -dot(pos, dir);
		const float scale = dot(dir, dir);
		const float off = sqr(radius) - dot(pos, pos);
		if (const float d2 = sqr(t_center) + scale * off; d2 >= 0.0f) {
			const float t_diff = std::sqrt(d2);
			const float t = (t_center - int(side) * t_diff) / scale;
			if (t >= -eps) {
				vec3 p = from.pos + t * from.dir;
				if (abs(p.x) <= halflength)
					dist.side = t - eps;
			}
		}
	}

	return dist;
}

class InwardsBoundary final: public SubspaceBoundaryEx {
public:
	Params const &params;
	Subspace *side;
	Subspace *channel;

	InwardsBoundary(Params &_params): params(_params) {}

	BoundaryPoint findBoundary(Ray from) const override {
		auto dists = cylinderDist(from, params.outer_half_length, params.outer_radius);
		float dist = min(dists.cap, dists.side);
		if (!std::isfinite(dist))
			return {{nullptr, from.pos, from.pos, mat3(1)}, dist};
		const vec3 pos = from.pos + dist * from.dir;
		return {leave({pos, from.dir}), 0.0};
	}

	std::vector<Transition> findOverlaps(vec3 pos, float max_distance) const override {
		std::vector<Transition> overlaps;
		if (length(pos.x) <= params.outer_half_length + max_distance && length(vec2{pos.y, pos.z}) <= params.outer_radius + max_distance) {
			if (length(vec2{pos.y, pos.z}) - max_distance <= params.inner_radius) {
				assert(abs(pos.x) >= params.outer_half_length);
				vec3 into = pos;
				into.x -= copysign(params.outer_half_length - params.inner_half_length, pos.x);
				overlaps.push_back({channel, pos, into, mat3(1)});
			}
			if (length(vec2{pos.y, pos.z}) + max_distance >= params.inner_radius) {
				overlaps.push_back({side, pos, pos, mat3(1)});
			}
		}
		return overlaps;
	}

	bool contains(vec3 point) const override {
		return length(point.x) > params.outer_half_length || length(vec2{point.y, point.z}) > params.outer_radius;
	}

	Transition leave(Ray at) const override {
		vec3 pos = at.pos;
		if (length(vec2{pos.y, pos.z}) < params.inner_radius) {
			vec3 into = pos;
			into.x -= copysign(params.outer_half_length - params.inner_half_length, pos.x);
			return {channel, pos, into, mat3(1)};
		} else {
			return {side, pos, pos, mat3(1)};
		}
	}
};

class ChannelBoundary final: public SubspaceBoundaryEx {
public:
	Params const &params;
	Subspace *outer;
	Subspace *side;

	ChannelBoundary(Params &_params): params(_params) {}

	BoundaryPoint findBoundary(Ray from) const override {
		auto dist = cylinderDist(from, params.inner_half_length, params.inner_radius, Side::Inside);
		const vec3 pos = from.pos + min(dist.cap, dist.side) * from.dir;
		vec3 into = pos;

		if (dist.cap <= dist.side) {
			into.x += copysign(params.outer_half_length - params.inner_half_length, pos.x);
			return {{outer, pos, into, mat3(1)}, dist.cap};
		} else {
			Coefs cs(params);
			float m = 1.0f;
			if (abs(pos.x) < cs.x1) {
				into.x *= cs.y1 / cs.x1;
				m = cs.y1 / cs.x1;
			} else if (abs(pos.x) < cs.x2) {
				m = 2 * cs.w * (abs(pos.x) - cs.x0);
				into.x = copysign(cs.y0 + cs.w * sqr(abs(pos.x) - cs.x0), pos.x);
			} else {
				into.x += copysign(cs.y2 - cs.x2, pos.x);
			}
			return {{side, pos, into, diagonal(m, 1)}, dist.side};
		}
	}

	std::vector<Transition> findOverlaps(vec3 pos, float max_distance) const override {
		std::vector<Transition> overlaps;
		if (abs(pos.x) + max_distance >= params.inner_half_length) {
			vec3 into = pos;
			into.x += copysign(params.outer_half_length - params.inner_half_length, pos.x);
			overlaps.push_back({outer, pos, into, mat3(1)});
		}
		if (length(vec2{pos.y, pos.z}) + max_distance >= params.inner_radius) {
			Coefs cs(params);
			vec3 into = pos;
			float m = 1.0f;
			if (abs(pos.x) < cs.x1) {
				into.x *= cs.y1 / cs.x1;
				m = cs.y1 / cs.x1;
			} else if (abs(pos.x) < cs.x2) {
				m = 2 * cs.w * (abs(pos.x) - cs.x0);
				into.x = copysign(cs.y0 + cs.w * sqr(abs(pos.x) - cs.x0), pos.x);
			} else {
				into.x += copysign(cs.y2 - cs.x2, pos.x);
			}
			overlaps.push_back({side, pos, into, diagonal(m, 1)});
		}
		return overlaps;
	}

	bool contains(vec3 point) const override {
		return length(point.x) < params.inner_half_length && length(vec2{point.y, point.z}) < params.inner_radius;
	}

	Transition leave(Ray at) const override {
		const vec3 pos = at.pos;
		vec3 into = pos;

		if (length(pos.x) <= params.inner_half_length || length(vec2{pos.y, pos.z}) <= params.inner_radius) {
			into.x += copysign(params.outer_half_length - params.inner_half_length, pos.x);
			return {outer, pos, into, mat3(1)};
		} else {
			Coefs cs(params);
			float m = 1.0f;
			if (abs(pos.x) < cs.x1) {
				into.x *= cs.y1 / cs.x1;
				m = cs.y1 / cs.x1;
			} else if (abs(pos.x) < cs.x2) {
				m = 2 * cs.w * (abs(pos.x) - cs.x0);
				into.x = copysign(cs.y0 + cs.w * sqr(abs(pos.x) - cs.x0), pos.x);
			} else {
				into.x += copysign(cs.y2 - cs.x2, pos.x);
			}
			return {side, pos, into, diagonal(m, 1)};
		}
	}
};

class ChannelMetric: public RiemannMetric<3> {
public:
	Params const &params;

	ChannelMetric(Params &_params): params(_params) {}

	decomp halfmetric(vec3 pos) const noexcept override {
		Coefs cs(params);
		float x = pos.x;
		float dx = 1.0f;
		if (abs(x) < cs.y1) {
			dx = cs.x1 / cs.y1;
		} else if (abs(x) < cs.y2) {
			x = copysign(cs.x0 + sqrt((abs(x) - cs.y0) / cs.w), x);
			dx /= -2 * cs.w * (abs(x) - cs.x0);
		}
		return {mat3(1.0f), {dx, 1.0f, 1.0f}};
	}
};

class ChannelSideMetric: public ChannelMetric {
	using ChannelMetric::ChannelMetric;

	decomp halfmetric(vec pos) const noexcept override {
		auto g = ChannelMetric::halfmetric(pos);
		float c = clamp((params.outer_radius - length(vec2{pos.y, pos.z})) / (params.outer_radius - params.inner_radius), 0.0f, 1.0f);
		c = smoothstep(c);
		return {g.ortho, mix(vec3(1.0f), g.diag, c)};
	}
};

class SideBoundary: public SwitchMap {
public:
	Params const &params;
	Subspace *outer;
	Subspace *channel;

	SideBoundary(Params &_params): params(_params) {}

	bool contains(vec3 point) const override {
		float r = length(vec2{point.y, point.z});
		return abs(point.x) <= params.outer_half_length + eps && r <= params.outer_radius + eps && r >= params.inner_radius - eps;
	}

	Transition leave(const Ray at) const override {
		vec3 pos = at.pos;
		vec3 dir = at.dir;
		Coefs cs(params);
		if (length(vec2{pos.y, pos.z}) >= params.inner_radius)
			return {outer, pos, pos, mat3(1)};
		if (abs(pos.x) >= params.outer_half_length && sign(dir.x) == sign(pos.x))
			return {outer, pos, pos, mat3(1)};
		float m = 1.0f;
		if (abs(pos.x) < cs.y1) {
			pos.x *= cs.x1 / cs.y1;
			m = cs.x1 / cs.y1;
		} else if (abs(pos.x) < cs.y2) {
			pos.x = copysign(sqrt((abs(pos.x) - cs.y0) / cs.w) - cs.x0, pos.x);
			m = 1 / (2 * cs.w * (abs(pos.x) - cs.x0));
		} else {
			pos.x -= copysign(cs.y2 - cs.x2, pos.x);
		}
		return {channel, at.pos, pos, diagonal(m, 1.0f)};
	}
};

void test() {
}

class ChannelVisual: public SpaceVisual {
public:
	Params const &params;

	ChannelVisual(vec3 color, Params &_params): SpaceVisual(color), params(_params) {}

	vec3 where(vec3 pos) const override {
		Coefs cs(params);
		if (abs(pos.x) < cs.x1) {
			pos.x *= cs.y1 / cs.x1;
		} else if (abs(pos.x) < cs.x2) {
			pos.x = copysign(cs.y0 + cs.w * sqr(abs(pos.x) - cs.x0), pos.x);
		} else {
			pos.x += copysign(cs.y2 - cs.x2, pos.x);
		}
		return pos;
	}

	mat3 jacobi(vec3 pos) const override {
		Coefs cs(params);
		if (abs(pos.x) < cs.x1) {
			return diagonal(cs.y1 / cs.x1, 1.0f);
		} else if (abs(pos.x) < cs.x2) {
			return diagonal(2 * cs.w * (abs(pos.x) - cs.x0), 1.0f);
		} else {
			return mat3(1.0f);
		}
	}
};

class PreviewableThing: public Thing {
public:
	virtual void preview(SpaceVisual const *visual) const = 0;
};

class Sphere: public PreviewableThing {
public:
	float radius;

	Sphere() = default;
	Sphere(uint32_t _id, float _radius, ThingySubspace const *space, vec3 pos) {
		id = _id;
		radius = _radius;
		loc = {
			space,
			pos,
			mat3(1.0f),
		};
	}

	ThingHit hit(Ray ray) const override {
		const vec3 rel = -ray.pos;
		const float t_center = -dot(ray.pos, ray.dir);
		const float d2 = dot(rel, rel) - sqr(t_center);
		if (d2 > sqr(radius))
			return {};
		const float t_diff = std::sqrt(sqr(radius) - d2);
		const float t_near = t_center - t_diff;
		const float t_far = t_center + t_diff;
		if (t_near < -eps)
			return {};
		auto pos = ray.pos + t_near * ray.dir;
		return {pos, t_near, pos / radius};
	}

	float getRadius() const noexcept final override {
		return radius;
	}

	void preview(SpaceVisual const *visual) const override {
		ellipsoid(visual, loc.pos, radius * loc.rot);
	}
};

asyncpp::generator<std::pair<vec3, vec3>> edges(std::vector<vec3> const &points) {
	vec3 a = points.back();
	for (vec3 b: points) {
		co_yield {a, b};
		a = b;
	}
}

vec3 dirToColor(vec3 dir) {
	vec3 v = mat3(1, -1, -1, -1, 1, -1, -1, -1, 1) * dir;
	return .5f + .5f * v / max(abs(v));
}

class Mesh: public PreviewableThing {
public:
	Mesh(uint32_t _id, std::vector<vec3> _points, std::vector<ivec3> _tris, ThingySubspace const *space, vec3 pos) {
		id = _id;
		loc = {
			space,
			pos,
			mat3(1.0f),
		};
		points = std::move(_points);
		tris = std::move(_tris);
		assert(points.size() >= 4);
		for (vec3 p: points)
			radius = max(radius, length(p));
	}

	ThingHit hit(Ray ray) const override {
		float min_dist = std::numeric_limits<float>::infinity();
		vec3 normal = {};
		for (ivec3 tri: tris) {
			vec3 a = points[tri.x];
			vec3 b = points[tri.y];
			vec3 c = points[tri.z];
			vec3 n = cross(b - a, c - a);
			vec3 rel_a = a - ray.pos;
			vec3 rel_b = b - ray.pos;
			vec3 rel_c = c - ray.pos;
			mat3 rel = transpose(mat3{
				cross(rel_b, rel_c),
				cross(rel_c, rel_a),
				cross(rel_a, rel_b),
			});
			vec3 flags = rel * ray.dir;
			if (all(lessThanEqual(flags, vec3(0.0f)))) {
				float k = dot(a, n); // a⋅n = b⋅n = c⋅n
				float dist = (k - dot(ray.pos, n)) / dot(ray.dir, n);
				if (dist < -eps)
					continue;
				if (dist >= min_dist)
					continue;
				min_dist = dist; // Для выпуклого контура можно было бы сразу `return dist`.
				normal = n;
			}
		}
		return {ray.pos + min_dist * ray.dir, min_dist, normalize(normal)};
	}

	float getRadius() const noexcept final override {
		return radius;
	}

	void preview(SpaceVisual const *visual) const override {
		glEnable(GL_DEPTH_TEST);
		glBegin(GL_TRIANGLES);
		for (ivec3 tri: tris) {
			vec3 a = loc.pos + loc.rot * points[tri.x];
			vec3 b = loc.pos + loc.rot * points[tri.y];
			vec3 c = loc.pos + loc.rot * points[tri.z];
			vec3 n = normalize(cross(b - a, c - a));
			glColor3fv(value_ptr(dirToColor(n)));
			glVertex3fv(value_ptr(visual->where(a)));
			glVertex3fv(value_ptr(visual->where(b)));
			glVertex3fv(value_ptr(visual->where(c)));
		}
		glEnd();
		glDisable(GL_DEPTH_TEST);
	}

private:
	std::vector<vec3> points;
	std::vector<ivec3> tris;

	float radius = 0.0f;
};

using std::shared_ptr, std::make_shared;

double t_frozen = 0.0;
double t_offset = 0.0;
bool active = true;
double rt_time = 0.0;
int rt_rays = 0;

class MyUniverse: public Universe {
public:
	Params params;
	ThingySubspace outer, channel;
	GpuRiemannSubspace side;
	ChannelSideMetric side_metric{params};
	SideBoundary sbnd{params};
	InwardsBoundary ibnd{params};
	ChannelBoundary cbnd{params};

public:
	MyUniverse() {
		side.metric = &side_metric;
		side.map = &sbnd;
		ibnd.side = &side;
		ibnd.channel = &channel;
		cbnd.outer = &outer;
		cbnd.side = &side;
		sbnd.outer = &outer;
		sbnd.channel = &channel;
		outer.boundary = &ibnd;
		channel.boundary = &cbnd;

		thingySpaces.push_back(&outer);
		thingySpaces.push_back(&channel);
	}
};

MyUniverse uni;
const float off = 2.5f;
const float A = uni.params.inner_half_length + off;
const float omega = 1.0f;
const float a = .3f, b = 0.5f * a;
Sphere spheres[] = {
	// {0.25f, &uni.outer, {-(uni.params.outer_half_length + off), -0.5f, 0.0f}},
	// {0.10f, &uni.outer, {-(uni.params.outer_half_length + off), 0.0f, 0.0f}},
	{1, 300.0f, &uni.outer, {(uni.params.outer_half_length + 320.0f), -100.0f, 0.0f}},
};
Mesh meshes[] = {
	{
		2,
		{{-a, -a, 0.f}, {0.f, -0.500f * a, 0.f}, {a, -a, 0.f}, {0.f, 1.414f * a, 0.f}, {0.f, -a, -0.500f * a}, {0.f, -a, 0.500f * a}},
		{{0, 5, 3}, {5, 2, 3}, {2, 4, 3}, {4, 0, 3}, {5, 0, 1}, {2, 5, 1}, {4, 2, 1}, {0, 4, 1}},
		&uni.outer, {-(uni.params.outer_half_length + off), -2.0f, 0.0f},
	},
	{
		2,
		{{-a, -a, 0.f}, {0.f, -0.500f * a, 0.f}, {a, -a, 0.f}, {0.f, 1.414f * a, 0.f}, {0.f, -a, -0.500f * a}, {0.f, -a, 0.500f * a}},
// 		{{0, 5, 3}, {5, 2, 3}, {5, 0, 1}, {2, 5, 1}}, // верхняя половина — для тестирования
		{{0, 5, 3}, {5, 2, 3}, {2, 4, 3}, {4, 0, 3}, {5, 0, 1}, {2, 5, 1}, {4, 2, 1}, {0, 4, 1}},
		&uni.outer, {-(uni.params.outer_half_length + off), -1.0f, 0.0f}
	},
	{
		0,
		{{-1, -1, -1}, {-1, -1, 1}, {-1, 1, -1}, {-1, 1, 1}, {1, -1, -1}, {1, -1, 1}, {1, 1, -1}, {1, 1, 1}},
		{{0, 1, 2}, {1, 3, 2}, {0, 2, 4}, {2, 6, 4}, {0, 4, 1}, {1, 4, 5}, {5, 6, 7}, {4, 6, 5}, {3, 5, 7}, {1, 5, 3}, {3, 7, 6}, {2, 3, 6}},
		&uni.outer, {-(uni.params.outer_half_length + off + 3), -1.0f, 0.0f}
	},
};
Thing *me = &meshes[0];

void init() {
	for (auto &sphere: spheres)
		uni.things.push_back(&sphere);
	for (auto &th: meshes)
		uni.things.push_back(&th);
	for (auto &th: uni.things)
		th->loc.rot = mat3(1.0f);
}

namespace settings {
	float rays = 120;
	int refine = 4;
	int trace_limit = 10;
	bool show_frame = false;
	bool show_previews = false;
	bool physical_acceleration = false;
	bool mouse_control = false;
	bool jet_control = false;

	float movement_acceleration = 6.0f;
	vec3 movement_speed = {1.0f, 6.0f, 1.0f};
	vec3 rotation_speed = {2.5f, 2.5f, 0.5f};
}

bool scale_space = false;

/// Поворот на углы @p angle (yaw, pitch, roll — рысканье, тангаж, крен).
///
/// Направления осей:
///   * X — вправо
///   * Y — вперёд
///   * Z — вверх
///
/// Положительные повороты:
///   * рысканье (X) — влево
///   * тангаж (Y) — вниз
///   * крен (Z) — против часовой
mat3 rotate(vec3 angle) {
	vec3 s = sin(angle), c = cos(angle);
	mat3 yaw = {
		c.x, s.x, 0,
		-s.x, c.x, 0,
		0, 0, 1,
	};
	mat3 pitch = {
		1, 0, 0,
		0, c.y, -s.y,
		0, s.y, c.y,
	};
	mat3 roll = {
		c.z, 0, s.z,
		0, 1, 0,
		-s.z, 0, c.z,
	};
	return yaw * pitch * roll;
}

/// Возвращает форму окна (x/y = ширина/высота, x⋅y = 1)
vec2 getWinShape(GLFWwindow *wnd) {
	ivec2 size;
	glfwGetWindowSize(wnd, &size.x, &size.y);
	vec2 v = vec2(size);
	return sqrt(vec2{v.x / v.y, v.y / v.x});
}

void update(GLFWwindow *wnd) {
	static double t0 = 0.0;
	double t = active ? glfwGetTime() - t_offset : t_frozen;
	float dt = active ? t - t0 : 0.0;
	t0 = t;

	vec3 mov{0.0f};
	vec3 rot{0.0f};
	static vec3 v = {0.0f, 0.0f, 0.0f};
	ivec2 wsize;
	dvec2 mouse;
	glfwGetCursorPos(wnd, &mouse.x, &mouse.y);
	glfwGetWindowSize(wnd, &wsize.x, &wsize.y);
	if (settings::mouse_control) {
		vec2 ctl = clamp(2.0f * vec2(mouse) / vec2(wsize) - 1.0f, -1.0f, 1.0f);
		rot.x = -ctl.x;
		rot.y = ctl.y;
	} else {
		if (glfwGetKey(wnd, GLFW_KEY_LEFT) == GLFW_PRESS) rot.x += 1.0f;
		if (glfwGetKey(wnd, GLFW_KEY_RIGHT) == GLFW_PRESS) rot.x -= 1.0f;
		if (glfwGetKey(wnd, GLFW_KEY_UP) == GLFW_PRESS) rot.y -= 1.0f;
		if (glfwGetKey(wnd, GLFW_KEY_DOWN) == GLFW_PRESS) rot.y += 1.0f;
	}
	if (glfwGetKey(wnd, GLFW_KEY_W) == GLFW_PRESS) mov.y += 1.0f;
	if (glfwGetKey(wnd, GLFW_KEY_S) == GLFW_PRESS) mov.y -= 1.0f;
	if (glfwGetKey(wnd, GLFW_KEY_A) == GLFW_PRESS) mov.x -= 1.0f;
	if (glfwGetKey(wnd, GLFW_KEY_D) == GLFW_PRESS) mov.x += 1.0f;
	if (glfwGetKey(wnd, GLFW_KEY_Q) == GLFW_PRESS) rot.z += 1.0f;
	if (glfwGetKey(wnd, GLFW_KEY_E) == GLFW_PRESS) rot.z -= 1.0f;

	mat3 rmat = rotate(dt * settings::rotation_speed * rot);
	if (settings::physical_acceleration) {
		v += dt * settings::movement_acceleration * mov;
		v = transpose(rmat) * v;
	} else if (settings::jet_control) {
		v += dt * settings::movement_acceleration * mov;
		v = clamp(v, -settings::movement_speed, settings::movement_speed);
	} else {
		v = settings::movement_speed * mov;
	}
	me->rotate(rmat);
	auto loc = me->loc;
	try {
		me->move(dt * v);
	} catch (char const *) {
		me->loc = loc;
		v = {};
	}

	if (scale_space) {
		static float x = 1.0f;
		static float v = 0.0f;
		v -= dt * omega * omega * x;
		x += dt * v;
		uni.params.inner_half_length = 3.0f + 2.0f * x;
		uni.params.inner_pad = 0.25f;
	}
	try {
		uni.updateCaches();
	} catch (char const *) {
		me->loc = loc;
		v = {};
	}
}

using ProgramID = GLuint;

namespace prog {
	ProgramID quad;
	ProgramID uv_quad;
}

namespace comp {
	ProgramID side;
}

namespace tex {
	TextureID objs;
}

void load_shaders() {
	prog::quad = link_program({
		compile_shader(GL_VERTEX_SHADER, read_file("empty.v.glsl")),
		compile_shader(GL_GEOMETRY_SHADER, read_file("screen_quad.g.glsl")),
		compile_shader(GL_FRAGMENT_SHADER, read_file("simple.f.glsl")),
	});
	prog::uv_quad = link_program({
		compile_shader(GL_VERTEX_SHADER, read_file("empty.v.glsl")),
		compile_shader(GL_GEOMETRY_SHADER, read_file("screen_quad.g.glsl")),
		compile_shader(GL_FRAGMENT_SHADER, read_file("screen_quad_cube.f.glsl")),
	});
	comp::side = link_program({
		compile_shader(GL_COMPUTE_SHADER, read_file("riemann.c.glsl")),
		compile_shader(GL_COMPUTE_SHADER, read_file("dmetric.c.glsl")),
		compile_shader(GL_COMPUTE_SHADER, read_file("side.c.glsl")),
	});
}

void load_textures() {
	unsigned size = 2048;
	glCreateTextures(GL_TEXTURE_CUBE_MAP_ARRAY, 1, &tex::objs);
	glTextureStorage3D(tex::objs, int(std::log2(size) + 1.5), GL_SRGB8_ALPHA8, size, size, 6*4);
	load_cube_texture_layer(tex::objs, 0, "grid");
	load_cube_texture_layer(tex::objs, 1, "jupiter");
	load_cube_texture_layer(tex::objs, 2, "saturn");
	load_cube_texture_layer(tex::objs, 3, "venus");
	glGenerateTextureMipmap(tex::objs);
	glTextureParameteri(tex::objs, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTextureParameteri(tex::objs, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
}

class ball_distribution {
public:
	ball_distribution(float _radius = 1.0f) : radius(_radius), inner(-1.0f, 1.0f) {}

	template <typename Generator>
	glm::vec3 operator()(Generator &gen) {
		for (;;) {
			glm::vec3 v{inner(gen), inner(gen), inner(gen)};
			float len = glm::length(v);
			if (len <= 1.0f && len >= 1e-3) {
				return radius * v;
			}
		}
	}

	float radius;

private:
	std::uniform_real_distribution<float> inner;
};

glm::vec4 sample(glm::vec3 dir) {
	static const mat4 conv = {
		{.5, -.5, -.5, 0},
		{-.5, .5, -.5, 0},
		{-.5, -.5, .5, 0},
		{.5, .5, .5, 1},
	};
	return conv * vec4(dir / max(abs(dir)), 1.0f);
}

struct VisualTraceResult {
	Ray incident;
	vec3 normal;
	Subspace const *space = nullptr;
	Thing const *thing = nullptr;
};

std::vector<VisualTraceResult> trace(std::vector<TrackPoint> rays) {
	struct Job {
		int at;
		TrackPoint pt;
	};
	std::vector<VisualTraceResult> result(rays.size());
	std::vector<Job> jobs(rays.size());
	for (int k = 0; k < rays.size(); k++)
		jobs[k] = {k, rays[k]};
	for (int n = 0; !jobs.empty(); n++) {
		if (n >= settings::trace_limit) {
			fprintf(stderr, "Warning: tracing loop aborted with %zu jobs still pending\n", jobs.size());
			break;
		}

		struct Batch {
			std::vector<int> indices;
			std::vector<Ray> rays;
		};
		std::unordered_map<Subspace const *, Batch> batches;

		for (auto &&job: jobs) {
			Batch &batch = batches[job.pt.space];
			batch.indices.push_back(job.at);
			batch.rays.push_back(job.pt);
		}
		jobs.clear();

		for (auto &&[space, batch]: batches) {
			assert(batch.indices.size() == batch.rays.size());
			auto results = space->trace(batch.rays);
			auto flat = dynamic_cast<ThingySubspace const *>(space);
			assert(results.size() == batch.rays.size());
			for (int k = 0; k < batch.rays.size(); k++) {
				const int at = batch.indices[k];
				auto const &traced = results[k];
				if (flat) {
					if (auto t = flat->traceToThing(batch.rays[k]); t.thing) {
						result[at] = {
							.space = space,
							.thing = t.thing,
							.incident = t.incident,
							.normal = t.normal,
						};
						continue;
					}
				}
				if (traced.to.space) {
					jobs.push_back({at, traced.to});
				} else {
					result[at] = {
						.space = space,
						.thing = nullptr,
						.incident = traced.end,
					};
				}
			}
		}
	}
	return result;
}

void render(GLFWwindow *wnd) {
	{
		struct {
			Params params;
			Coefs cs;
		} data{
			uni.params,
			uni.params,
		};
		uni.side.prog = comp::side;
		uni.side.params.resize(sizeof(data));
		std::memcpy(uni.side.params.data(), &data, sizeof(data));
	}

	const vec2 shape = getWinShape(wnd);
	const ivec2 ihalfsize = settings::rays * shape;
	const ivec2 fine_size = 2 * settings::refine * ihalfsize;
	const double rtt1 = glfwGetTime();

	std::vector<vec4> uvws;
	std::vector<char> objects_mask;
	std::vector<vec4> colors;
	std::vector<vec4> fine_colors;
	uvws.resize(4 * ihalfsize.x * ihalfsize.y);
	objects_mask.resize(4 * ihalfsize.x * ihalfsize.y);
	colors.resize(4 * ihalfsize.x * ihalfsize.y);
	fine_colors.resize(fine_size.x * fine_size.y);

	std::vector<TrackPoint> jobs;
	jobs.reserve(4 * ihalfsize.x * ihalfsize.y);
	for (ivec2 ipos: irange(-ihalfsize, ihalfsize)) {
		vec2 wpos = (vec2(ipos) + .5f) / vec2(ihalfsize);
		vec2 spos = shape * wpos;
		TrackPoint pt;
		pt.pos = me->loc.pos;
		pt.dir = me->loc.rot * normalize(vec3(spos.x, 1.0f, spos.y));
		pt.space = me->loc.space;
		jobs.push_back(pt);
	}
	auto trace_result = trace(std::move(jobs));
	jobs.clear();

	static thread_local std::mt19937 gen{(unsigned long)time(nullptr)};
	ball_distribution metal{0.5f};
	ball_distribution plastic{1.0f};
	for (int k = 0; k < 4 * ihalfsize.x * ihalfsize.y; k++) {
		auto const &t = trace_result[k];
		if (t.thing) {
			objects_mask[k] = 1;
			const int n_samples = 4;
			vec4 color = {};
			for (int s = 0; s < n_samples; s++) {
#if PLASTIC
				vec3 dir = normalize(plastic(gen));
				if (dot(dir, t.normal) < 0.0f)
					dir -= 2.0f * t.normal * dot(dir, t.normal);
#else
				vec3 dir = normalize(reflect(t.incident.dir, t.normal) + metal(gen));
#endif
				color += sample(dir);
			}
			colors[k] = color / float(n_samples);
		} else {
			uvws[k] = vec4(t.incident.dir, 0.0f);
		}
	}
	trace_result = {};

	std::vector<char> objects_mask_2;
	objects_mask_2.resize(objects_mask.size());
	for (ivec2 ipos: irange(-ihalfsize + 1, ihalfsize - 1)) {
		int index = (ipos.y + ihalfsize.y) * 2 * ihalfsize.x + (ipos.x + ihalfsize.x);
		objects_mask_2[index] = objects_mask[index]
			+ objects_mask[index - 1]
			+ objects_mask[index + 1]
			+ objects_mask[index - 2 * ihalfsize.x]
			+ objects_mask[index + 2 * ihalfsize.x];
	}

	std::vector<uint32_t> indices;
	indices.reserve(fine_size.x * fine_size.y);
	for (ivec2 ipos: irange(-ihalfsize, ihalfsize)) {
		int coarse_index = (ipos.y + ihalfsize.y) * 2 * ihalfsize.x + (ipos.x + ihalfsize.x);
		int mask = objects_mask_2[coarse_index];
		if (mask == 0 || mask == 5)
			continue;
		colors[coarse_index] = {};
		for (ivec2 sub: irange(ivec2(settings::refine))) {
			const ivec2 fine_ipos = settings::refine * (ihalfsize + ipos) + sub;
			const int fine_index = fine_size.x * fine_ipos.y + fine_ipos.x;
			const vec2 off = (vec2(sub) + .5f) / float(settings::refine);
			const vec2 wpos = (vec2(ipos) + off) / vec2(ihalfsize);
			const vec2 spos = shape * wpos;
			TrackPoint pt;
			pt.pos = me->loc.pos;
			pt.dir = me->loc.rot * normalize(vec3(spos.x, 1.0f, spos.y));
			pt.space = me->loc.space;
			jobs.push_back(pt);
			indices.push_back(fine_index);
		}
	}
	trace_result = trace(std::move(jobs));
	jobs.clear();
	for (auto [job_index, fine_index]: enumerate(indices)) {
		if (trace_result[job_index].thing) {
			fine_colors[fine_index] = vec4(0.5f + 0.5f * trace_result[job_index].normal, 1.0f);
		}
	}
	trace_result = {};

	double rtt2 = glfwGetTime();
	rt_rays += uvws.size();
	rt_time += rtt2 - rtt1;

	TextureID rt_result = 0;
	glCreateTextures(GL_TEXTURE_2D, 1, &rt_result);
	glTextureParameteri(rt_result,  GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTextureParameteri(rt_result,  GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTextureParameteri(rt_result,  GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTextureStorage2D(rt_result, 1, GL_RGBA16F, 2 * ihalfsize.x, 2 * ihalfsize.y);
	glTextureSubImage2D(rt_result, 0, 0, 0, 2 * ihalfsize.x, 2 * ihalfsize.y, GL_RGBA, GL_FLOAT, uvws.data());
	glUseProgram(prog::uv_quad);
	glBindTextureUnit(0, tex::objs);
	glBindTextureUnit(1, rt_result);
	glDisable(GL_BLEND);
	glDrawArrays(GL_POINTS, 0, 1);
	glDeleteTextures(1, &rt_result);

	TextureID rt_colors = 0;
	glCreateTextures(GL_TEXTURE_2D, 1, &rt_colors);
	glTextureParameteri(rt_colors,  GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTextureParameteri(rt_colors,  GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTextureParameteri(rt_colors,  GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTextureStorage2D(rt_colors, 1, GL_RGBA8, 2 * ihalfsize.x, 2 * ihalfsize.y);
	glTextureSubImage2D(rt_colors, 0, 0, 0, 2 * ihalfsize.x, 2 * ihalfsize.y, GL_RGBA, GL_FLOAT, colors.data());
	glBindTextureUnit(0, rt_colors);
	glEnable(GL_BLEND);
	glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
	glUseProgram(prog::quad);
	glDrawArrays(GL_POINTS, 0, 1);
	glDeleteTextures(1, &rt_colors);

	TextureID rt_fine_colors = 0;
	glCreateTextures(GL_TEXTURE_2D, 1, &rt_fine_colors);
	glTextureParameteri(rt_fine_colors,  GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTextureParameteri(rt_fine_colors,  GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTextureParameteri(rt_fine_colors,  GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTextureStorage2D(rt_fine_colors, 1, GL_RGBA8, fine_size.x, fine_size.y);
	glTextureSubImage2D(rt_fine_colors, 0, 0, 0, fine_size.x, fine_size.y, GL_RGBA, GL_FLOAT, fine_colors.data());
	glBindTextureUnit(0, rt_fine_colors);
	glEnable(GL_BLEND);
	glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
	glUseProgram(prog::quad);
	glDrawArrays(GL_POINTS, 0, 1);
	glDeleteTextures(1, &rt_fine_colors);

	glUseProgram(0);

	std::unordered_map<Subspace const *, shared_ptr<SpaceVisual>> visuals = {
		{nullptr, make_shared<SpaceVisual>(vec3{1.0f, 0.1f, 0.4f})},
		{&uni.outer, make_shared<SpaceVisual>(vec3{0.1f, 0.4f, 1.0f})},
		{&uni.channel, make_shared<ChannelVisual>(vec3{0.4f, 1.0f, 0.1f}, uni.params)},
		{&uni.side, make_shared<SpaceVisual>(vec3{1.0f, 0.4f, 0.1f})},
	};

	glLoadIdentity();
	{
		float near = 1e-2;
		float far = 1e2;
		auto &&visual = visuals.at(me->loc.space);
		vec3 off = visual->where(me->loc.pos);
		mat3 mat = transpose(me->loc.rot) * inverse(visual->jacobi(me->loc.pos));
		mat = mat3(1, 0, 0, 0, 0, -1, 0, 1, 0) * mat;
		glFrustum(-near * shape.x, near * shape.x, -near * shape.y, near * shape.y, near, far);
		glMultMatrixf(value_ptr(mat4(mat)));
		glTranslatef(-off.x, -off.y, -off.z);
	}

	if (settings::show_frame) {
		for (auto *bnd: uni.thingySpaces) {
			auto &&visual = visuals.at(bnd);
			for (auto &&info: bnd->things) {
				glColor3fv(value_ptr(visual->color));
				ellipsoid(visual.get(), info.pos, info.radius * info.rot);
			}
		}
	}

	if (settings::show_previews) {
		for (auto *thing: uni.things) {
			auto &&visual = visuals.at(thing->loc.space);
			if (auto p = dynamic_cast<PreviewableThing *>(thing)) {
				glColor4f(.8, .8, .8, .75);
				p->preview(visual.get());
			}
		}
	}

	if (settings::show_frame) {
		int N = 12;
		Coefs cs(uni.params);
		auto *visual = visuals.at(&uni.outer).get();
		glBegin(GL_LINES);
		glColor4f(.0f, .9f, .9f, .75f);
		for (int k = -N; k < N; k++) {
			float phi = (.5 + k) * (M_PI / N);
			vec3 r{0.0f, cos(phi), sin(phi)};
			vec3 l{1.0f, 0.0f, 0.0f};
			glVertex3fv(value_ptr(uni.params.outer_radius * r - uni.params.outer_half_length * l));
			glVertex3fv(value_ptr(uni.params.outer_radius * r + uni.params.outer_half_length * l));
		}
		glEnd();
		circleX(visual, vec3(-uni.params.outer_half_length, 0.0f, 0.0f), uni.params.outer_radius);
		circleX(visual, vec3(uni.params.outer_half_length, 0.0f, 0.0f), uni.params.outer_radius);
		glColor4f(.0f, .2f, .7f, .75f);
		circleX(visual, vec3(-uni.params.outer_half_length, 0.0f, 0.0f), uni.params.inner_radius);
		circleX(visual, vec3(uni.params.outer_half_length, 0.0f, 0.0f), uni.params.inner_radius);
		circleX(visual, vec3(-cs.y1, 0.0f, 0.0f), uni.params.inner_radius);
		circleX(visual, vec3(cs.y1, 0.0f, 0.0f), uni.params.inner_radius);
	}

	glLoadIdentity();
	if (settings::mouse_control) {
		glColor4f(.5f, .5f, .5f, .5f);
		glBegin(GL_LINES);
		glVertex2f(-20.f, 0.f);
		glVertex2f(20.f, 0.f);
		glVertex2f(0.f, -20.f);
		glVertex2f(0.f, 20.f);
		glEnd();
	}
}

float background_lightness = 0.1;

float winsize;

int frames = 0;

void paint(GLFWwindow* window) {
	glClearColor(0.0, 0.0, 0.0, 0.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_CULL_FACE);

	glLineWidth(winsize * 0.0085);
	glPointSize(winsize * 0.085);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE);
	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_MULTISAMPLE);
	glEnable(GL_TEXTURE_CUBE_MAP_SEAMLESS);

	render(window);

	glfwSwapBuffers(window);
	frames++;
}

GLint GetNamedFramebufferAttachmentParameter(GLuint framebuffer, GLenum attachment, GLenum pname) {
	GLint result = 0;
	glGetNamedFramebufferAttachmentParameteriv(framebuffer, attachment, pname, &result);
	return result;
}

void initGL() {
	glEnable(GL_FRAMEBUFFER_SRGB);
	for (auto attach: (GLenum[]){GL_FRONT_LEFT, GL_FRONT_RIGHT, GL_BACK_LEFT, GL_BACK_RIGHT, GL_DEPTH, GL_STENCIL}) {
		printf("Probing %04x...", attach);
		GLenum type = GetNamedFramebufferAttachmentParameter(0, attach, GL_FRAMEBUFFER_ATTACHMENT_OBJECT_TYPE);
		if (type == GL_NONE) {
			printf(" absent\n");
			continue;
		}
		GLenum ctype = GetNamedFramebufferAttachmentParameter(0, attach, GL_FRAMEBUFFER_ATTACHMENT_COMPONENT_TYPE);
		GLenum enc = GetNamedFramebufferAttachmentParameter(0, attach, GL_FRAMEBUFFER_ATTACHMENT_COLOR_ENCODING);
		printf(" %04x:%04x:%04x", type, enc, ctype);

		int red = GetNamedFramebufferAttachmentParameter(0, attach, GL_FRAMEBUFFER_ATTACHMENT_RED_SIZE);
		int green = GetNamedFramebufferAttachmentParameter(0, attach, GL_FRAMEBUFFER_ATTACHMENT_GREEN_SIZE);
		int blue = GetNamedFramebufferAttachmentParameter(0, attach, GL_FRAMEBUFFER_ATTACHMENT_BLUE_SIZE);
		int alpha = GetNamedFramebufferAttachmentParameter(0, attach, GL_FRAMEBUFFER_ATTACHMENT_ALPHA_SIZE);
		int depth = GetNamedFramebufferAttachmentParameter(0, attach, GL_FRAMEBUFFER_ATTACHMENT_DEPTH_SIZE);
		int stencil = GetNamedFramebufferAttachmentParameter(0, attach, GL_FRAMEBUFFER_ATTACHMENT_STENCIL_SIZE);
		printf(" %d:%d:%d:%d:%d:%d\n",red, green, blue, alpha, depth, stencil);
	}
// 	exit(0);

	load_shaders();
	load_textures();
}

void resized(GLFWwindow* window, int width, int height) {
	glViewport(0, 0, width, height);
	::winsize = sqrt(width * height / 2) / 4;
}

static const char *title = "Space Refraction 3D v2";

void toggle_active(GLFWwindow *window) {
	double t0 = glfwGetTime();
	active = !active;
	if (active) {
		t_offset = t0 - t_frozen;
		glfwSetWindowTitle(window, title);

	} else {
		char title[256];
		snprintf(title, sizeof(title), "%s (paused)", ::title);
		t_frozen = t0 - t_offset;
		glfwSetWindowTitle(window, title);
	}
}

void toggle_fullscreen(GLFWwindow *window) {
	static bool fullscreen = false;
	static int x, y, w, h;
	fullscreen = !fullscreen;
	if (fullscreen) {
		glfwGetWindowPos( window, &x, &y);
		glfwGetWindowSize( window, &w, &h);
		GLFWmonitor *monitor = glfwGetPrimaryMonitor();
		const GLFWvidmode *mode = glfwGetVideoMode(monitor);
		glfwSetWindowMonitor(window, monitor, 0, 0, mode->width, mode->height, mode->refreshRate);
	} else {
		glfwSetWindowMonitor(window, nullptr, x, y, w, h, GLFW_DONT_CARE);
	}
}

void keyed(GLFWwindow *window, int key, int scancode, int action, int mods) {
	if (key == GLFW_KEY_ESCAPE)
		glfwSetWindowShouldClose(window, GLFW_TRUE);
	if (action != GLFW_PRESS)
		return;

	switch (key) {
	case GLFW_KEY_PAUSE:
		toggle_active(window);
		break;

	case GLFW_KEY_ENTER:
		if (mods == GLFW_MOD_ALT)
			toggle_fullscreen(window);
		break;

	case GLFW_KEY_B:
		background_lightness = 1.0f - background_lightness;
		paint(window);
		break;

	case GLFW_KEY_F: settings::show_frame = !settings::show_frame; break;
	case GLFW_KEY_P: settings::physical_acceleration = !settings::physical_acceleration; break;
	case GLFW_KEY_SPACE: settings::mouse_control = !settings::mouse_control; break;
	case GLFW_KEY_J: settings::jet_control = !settings::jet_control; break;
	case GLFW_KEY_K: settings::show_previews = !settings::show_previews; break;
	}
}

void APIENTRY debug(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length, GLchar const *message, void const *userParam) {
	switch (severity) {
	case GL_DEBUG_SEVERITY_NOTIFICATION:
		return; // ignore
	case GL_DEBUG_SEVERITY_LOW:
		return; // ignore
	case GL_DEBUG_SEVERITY_MEDIUM:
		if (type == GL_DEBUG_TYPE_PERFORMANCE)
			return; // ignore anyway, they’re useless
		break; // show
	case GL_DEBUG_SEVERITY_HIGH:
		break; // show
	}
	std::fprintf(stderr, "%04x->%04x(%04x) %08x: %.*s\n", source, type, severity, id, (int)length, message);
}

int main() try {
#if TEST
	test();
#else
	init();
	glfwInit();
	glfwWindowHint(GLFW_ALPHA_BITS, 8);
	glfwWindowHint(GLFW_DEPTH_BITS, 16);
	glfwWindowHint(GLFW_SAMPLES, 8);
	glfwWindowHint(GLFW_SRGB_CAPABLE, GLFW_TRUE);
	glfwWindowHint(GLFW_DOUBLEBUFFER, GLFW_TRUE);
	glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_API);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
	glfwWindowHint(GLFW_CONTEXT_ROBUSTNESS, GLFW_NO_RESET_NOTIFICATION);
	glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, GLFW_TRUE);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_COMPAT_PROFILE);
	auto wnd = glfwCreateWindow(1600, 1200, title, nullptr, nullptr);
	glfwMakeContextCurrent(wnd);
	glDebugMessageCallback(debug, nullptr);
	initGL();
	glfwSwapInterval(1);
	glfwSetWindowRefreshCallback(wnd, paint);
	glfwSetFramebufferSizeCallback(wnd, resized);
	glfwSetKeyCallback(wnd, keyed);
	resized(wnd, 1600, 1200);
	glfwShowWindow(wnd);
	double t0 = glfwGetTime();
	while (!glfwWindowShouldClose(wnd)) {
		if (active) {
			glfwPollEvents();
			update(wnd);
			paint(wnd);
			double t1 = glfwGetTime();
			if (t1 - t0 >= 1.0) {
				double fps = frames / (t1 - t0);
				t0 = t1;
				frames = 0;
				char title[256];
				float den = RiemannSubspace::large_steps + RiemannSubspace::regular_steps;
				float larges = RiemannSubspace::large_steps / den;
				RiemannSubspace::large_steps = 0;
				RiemannSubspace::regular_steps = 0;
				snprintf(title, sizeof(title), "%s @ %.1f FPS, %.1f μs/ray, %.1f steps/ray, %.0f%% steps subdivided",
					::title, fps, 1e6 * rt_time / rt_rays, den / rt_rays, 100.0f * larges);
				rt_time = 0.0;
				rt_rays = 0;
				glfwSetWindowTitle(wnd, title);
			}
		} else {
			glfwWaitEvents();
			t0 = glfwGetTime();
			frames = 0;
		}
	}
	glfwDestroyWindow(wnd);
#endif
} catch (char const *str) {
	fflush(stdout);
	fprintf(stderr, "Got fatal exception: %s\n", str);
	throw;
}
