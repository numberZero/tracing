#include <cassert>
#include <cstdio>
#include <memory>
#include <unordered_map>
#include <vector>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <GLFW/glfw3.h>
#include <GL/gl.h>
#include <GL/glext.h>
#include "averager.hxx"
#include "math.hxx"
#include "desc/riemann.hxx"
#include "desc/ray.hxx"
#include "desc/euclidean.hxx"
#include "desc/flat.hxx"
#include "desc/fast.hxx"
#include "desc/box.hxx"

struct Hit {
	vec2 point;
};

class Shape {
public:
	virtual bool hit(Hit *hit, Ray ray) const {
		return false;
	}

	virtual void preview() const {}
};

class TransformedShape final: public Shape {
public:
	bool hit(Hit *hit, Ray ray) const noexcept final override {
		ray.base = inverse(tfm) * (ray.base - pos);
		ray.dir = normalize(inverse(tfm) * ray.dir);
		if (!base->hit(hit, ray))
			return false;
		hit->point = tfm * hit->point + pos;
		return true;
	}

	void preview() const noexcept final override {
		glPushMatrix();
		mat4 m = tfm;
		glTranslatef(pos.x, pos.y, 0.0);
		glMultMatrixf(value_ptr(m));
		base->preview();
		glPopMatrix();
	}

	Shape const *base;
	mat2 tfm;
	vec2 pos;
};

class UnitCircle final: public Shape {
public:
	bool hit(Hit *hit, Ray ray) const noexcept override {
		static constexpr float eps = 1e-3f;
		vec2 L = -ray.base;
		float tca = dot(L, ray.dir);
		if (tca < 0) return false;
		float d2 = dot(L, L) - tca*tca;
		if (d2 > 1.0f) return false;
		float thc = std::sqrt(1.0f - d2);
		float t0 = tca-thc, t1 = tca+thc;
		if (t0 > eps) return make(hit, ray, t0);
		if (t1 > eps) return make(hit, ray, t1);
		return false;
	}

	bool make(Hit *hit, Ray ray, float t) const noexcept {
		hit->point = ray.base + t * ray.dir;
		return true;
	}

	void preview() const noexcept override {
		glBegin(GL_LINE_LOOP);
		for (int k = 0; k < 120; k++) {
			float phi = M_PI * k / 60.0;
			glVertex2f(cos(phi), sin(phi));
		}
		glEnd();
	}

	static const UnitCircle instance;
};
const UnitCircle UnitCircle::instance;

class UnitLine final: public Shape {
public:
	bool hit(Hit *hit, Ray ray) const noexcept override {
		if (ray.dir.x > 0.0f && ray.base.x < 0.0f) {
			float dx = -ray.base.x;
			float dy = dx * ray.dir.y / ray.dir.x;
			float y = ray.base.y + dy;
			if (std::abs(y) <= 1.0f) {
				hit->point = {0.0f, y};
				return true;
			}
		}
		return false;
	}

	void preview() const noexcept override {
		glBegin(GL_LINES);
		glVertex2f(0.0, -1.0);
		glVertex2f(0.0, 1.0);
		glEnd();
	}

	static const UnitLine instance;
};
const UnitLine UnitLine::instance;

class UnitSquare final: public Shape {
public:
	bool hit(Hit *hit, Ray ray) const noexcept override {
		if (ray.dir.x > 0.0f && ray.base.x < -1.0f) {
			float dx = -1.0f - ray.base.x;
			float dy = dx * ray.dir.y / ray.dir.x;
			float y = ray.base.y + dy;
			if (std::abs(y) <= 1.0f) {
				hit->point = {-1.0f, y};
				return true;
			}
		}
		if (ray.dir.x < 0.0f && ray.base.x > 1.0f) {
			float dx = ray.base.x - 1.0f;
			float dy = -dx * ray.dir.y / ray.dir.x;
			float y = ray.base.y + dy;
			if (std::abs(y) <= 1.0f) {
				hit->point = {1.0f, y};
				return true;
			}
		}
		if (ray.dir.y > 0.0f && ray.base.y < -1.0f) {
			float dy = -1.0f - ray.base.y;
			float dx = dy * ray.dir.x / ray.dir.y;
			float x = ray.base.x + dx;
			if (std::abs(x) <= 1.0f) {
				hit->point = {x, -1.0f};
				return true;
			}
		}
		if (ray.dir.y < 0.0f && ray.base.y > 1.0f) {
			float dy = ray.base.y - 1.0f;
			float dx = -dy * ray.dir.x / ray.dir.y;
			float x = ray.base.x + dx;
			if (std::abs(x) <= 1.0f) {
				hit->point = {x, 1.0f};
				return true;
			}
		}
		return false;
	}

	void preview() const noexcept override {
		glBegin(GL_LINE_LOOP);
		glVertex2f(-1.0, -1.0);
		glVertex2f(1.0, -1.0);
		glVertex2f(1.0, 1.0);
		glVertex2f(-1.0, 1.0);
		glEnd();
	}

	static const UnitSquare instance;
};
const UnitSquare UnitSquare::instance;

class UnitSquareInv final: public Shape {
public:
	bool hit(Hit *hit, Ray ray) const noexcept override {
// 		assert(abs(ray.base.x) < 1.0f + 1e-2);
// 		assert(abs(ray.base.y) < 1.0f + 1e-2);

		if (ray.dir.x < 0.0f) {
			float dx = -1.0f - ray.base.x;
			float dy = dx * ray.dir.y / ray.dir.x;
			float y = ray.base.y + dy;
			if (std::abs(y) <= 1.0f) {
				hit->point = {-1.0f, y};
				return true;
			}
		}
		if (ray.dir.x > 0.0f) {
			float dx = ray.base.x - 1.0f;
			float dy = -dx * ray.dir.y / ray.dir.x;
			float y = ray.base.y + dy;
			if (std::abs(y) <= 1.0f) {
				hit->point = {1.0f, y};
				return true;
			}
		}
		if (ray.dir.y < 0.0f) {
			float dy = -1.0f - ray.base.y;
			float dx = dy * ray.dir.x / ray.dir.y;
			float x = ray.base.x + dx;
			if (std::abs(x) <= 1.0f) {
				hit->point = {x, -1.0f};
				return true;
			}
		}
		if (ray.dir.y > 0.0f) {
			float dy = ray.base.y - 1.0f;
			float dx = -dy * ray.dir.x / ray.dir.y;
			float x = ray.base.x + dx;
			if (std::abs(x) <= 1.0f) {
				hit->point = {x, 1.0f};
				return true;
			}
		}
		abort();
	}

	void preview() const noexcept override {
		glBegin(GL_LINE_LOOP);
		glVertex2f(-1.0, -1.0);
		glVertex2f(1.0, -1.0);
		glVertex2f(1.0, 1.0);
		glVertex2f(-1.0, 1.0);
		glEnd();
	}

	static const UnitSquareInv instance;
};
const UnitSquareInv UnitSquareInv::instance;
/*
class CoordinateSystem {
public:
	virtual decomp2 halfmetric(vec2 pos) = 0;

	mat2 metric(vec2 pos) {
		decomp2 h = halfmetric(pos);
		return transpose(h.ortho) * diagonal(h.diag * h.diag) * h.ortho;
	}
};

static constexpr float eps = 1e-3f;

class Universe;

class Region {
public:
	vec3 color;

	struct Next {
		Region *region;
		vec2 pos;
		float dist;
	};

	virtual Next next(Ray ray) = 0;
};

class Object: public Region {
public:
};


class Space: public Region {
public:
	Shape *shape;
};

class Flat: public Space {
public:
	struct Location {
		Object *object;
		vec2 pos;
		float radius;
	};

	std::vector<Location> objects;

	Next next(Ray ray) override {
		Next n;
		n.dist = MAXFLOAT;

		for (auto loc: objects) {
			vec2 rel = loc.pos - ray.base;

			float a = -dot(rel, ray.dir);
			if (a < 0) continue;

			float d2 = dot(rel, rel) - a*a;
			float b = loc.radius * loc.radius - d2;
			if (b < 0) continue;

			float t = a-std::sqrt(b);
			if (t > n.dist)
				continue;
			if (t > eps) {
				n.region = loc.object;
				n.dist = t;
				n.pos = ray.base + t * ray.dir;
			}
		}

		return n;
	}
};

class Universe: public Flat {
public:
	std::vector<Space *> subspaces;
	std::vector<Object *> objects;

	Next next(Ray ray) override {
		Next n = Flat::next(ray);

		for (auto *space: subspaces) {
			Hit h;
			if (space->shape->hit(&h, ray)) {
				float t = distance(ray.base, h.point);
				if (t > n.dist)
					continue;
				n.region = space;
				n.pos = h.point;
				n.dist = t;
			}
		}

		return n;
	}
};
*/
/*
class Gluing {
public:
	virtual void convertForward(Ray *ray) = 0;
	virtual void convertBackward(Ray *ray) = 0;
};
*/


class Universe;
class Space;
/*
class Object {
};
*/

struct Track {
	std::vector<vec2> points;
	vec2 dir;
	Space *next = nullptr;
	float length;
};

class Space {
public:
	virtual Track trace(Ray ray, float maxd) const = 0;
};

struct Transition {
	Space *into;
	Shape *shape;
};

class ConvexBoundary {
private:
	struct Segment {
		Space *target;
		vec2 normal;
		float threshold;
	};

	std::vector<vec2> points;
	std::vector<Segment> segments;

public:
	ConvexBoundary(std::vector<vec2> &&_points, std::vector<Space *> const &_targets)
		: points(std::move(_points))
	{
		if (points.size() != _targets.size())
			throw std::invalid_argument("There must be exactly one target space per boundary segment");
		int n = points.size();
		if (n < 3)
			throw std::invalid_argument("There must be at least three segments");
		segments.resize(n);
		for (int k = 0; k < n; k++) {
			vec2 a = points[k];
			vec2 b = points[(k + 1) % n];
			vec2 u = b - a;
			vec2 v = {-u.y, u.x};
			segments[k].target = _targets[k];
			segments[k].normal = v;
			segments[k].threshold = dot(a, v);
		}
	}

	bool inside(vec2 p) const noexcept {
		for (auto &&seg: segments) {
			if (dot(p, seg.normal) > seg.threshold)
				return false;
		}
		return true;
	}

	Space *next(Ray from) const noexcept {
		int n = segments.size();
		for (int k = 0; k < n; k++) {
			vec2 a = points[k] - from.base;
			vec2 b = points[(k + 1) % n] - from.base;
// 			if (dot(a, from.dir) <= 0.0f)
// 				continue;
// 			if (dot(b, from.dir) <= 0.0f)
// 				continue;
			float da = dot(vec2(-a.y, a.x), from.dir);
			float db = dot(vec2(-b.y, b.x), from.dir);
			if (da <= 0.0f && db >= 0.0f)
				return segments[k].target;
		}
		return nullptr;
	}
};

class FlatSpace: public Space {
public:
	std::vector<Transition> transitions;
	SpaceDesc const *desc = &EuclideanSpace::instance;

	Track trace(Ray ray, float maxd) const final override {
		Track track = {};

		ray = desc->fromGlobal(ray);
		vec2 hitpoint = ray.base + maxd * ray.dir;
		for (auto &trans: transitions) {
			Hit h;
			if (!trans.shape->hit(&h, ray))
				continue;
			float dist = distance(ray.base, h.point);
			if (dist > maxd)
				continue;
			hitpoint = h.point;
			maxd = dist;
			track.next = trans.into;
		}

		track.length = maxd;

// 		Ray r = desc->toGlobal({hitpoint, ray.dir});
// 		return {{ray.base, r.base}, r.dir, next};

		static constexpr float dt = 1e-2;
		int steps = max(1, int(maxd / dt));
		track.points.reserve(steps + 1);
		for (int k = 0; k <= steps; k++) {
			float t = k * maxd / steps;
			vec2 p = ray.base + t * ray.dir;
			Ray r = desc->toGlobal({p, ray.dir});
			track.points.push_back(r.base);
			track.dir = r.dir;
		}

		return track;
	}
};

class RiemannSpace: public Space {
public:
	RiemannMetric<2> *metric;

	Track trace(Ray ray, float maxd) const final override {
		static constexpr float dt = 1e-2;
		static constexpr float eta = 1e-2;
		Track t;
		t.length = maxd;

		int steps = maxd / dt;
		t.points.reserve(steps + 1);
		auto p = ray.base;
		auto v = ray.dir;
		v /=  length(p, v);
		t.points.push_back(p);
		for (int k = 0; k < steps; k++) {
			auto p1 = p;
			auto v1 = v;
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
			t.points.push_back(p);
			if (Space *s = next({p1, v1}, p)) {
				t.next = s;
				t.length = k * dt;
				break;
			}
		}
		t.dir = v;
		return t;
	}

	float length(vec2 pos, vec2 vec) const {
		mat2 g = metric->metric(pos);
		return sqrt(dot(vec, g * vec));
	}

	virtual Space *next([[maybe_unused]] Ray ray, [[maybe_unused]] vec2 p2) const {
		return nullptr;
	}
};

class ConvexRiemannSubspace: public RiemannSpace {
public:
	ConvexBoundary *boundary;

	Space *next(Ray ray, vec2 p2) const noexcept final override {
		if (boundary->inside(p2))
			return nullptr;
		return boundary->next(ray);
	}
};
/*
struct ObjectLocation {
	Object *object;
	Space *space;
	vec2 pos;
	float radius;
};

struct Boundary {
	int space1;
	int space2;
	Shape *shape1;
	Shape *shape2;
	Gluing *gluing;
};
*/
class Universe {
public:
/*
	std::vector<ObjectLocation> objectLocations;
	std::vector<Boundary> flatRiemann;
	std::vector<Boundary> flatFlat;

	std::vector<Space *> subspaces;
	std::vector<Object *> objects;

	Next next(Ray ray) override {
		Next n = Flat::next(ray);

		for (auto *space: subspaces) {
			Hit h;
			if (space->shape->hit(&h, ray)) {
				float t = distance(ray.base, h.point);
				if (t > n.dist)
					continue;
				n.region = space;
				n.pos = h.point;
				n.dist = t;
			}
		}

		return n;
	}
*/
};

class SegmentedTransitions {
private:
	std::vector<TransformedShape> tfms;
	std::vector<Transition> trans;

public:
	SegmentedTransitions(std::vector<vec2> const &points, std::vector<Space *> const &targets) {
		if (points.size() != targets.size())
			throw std::invalid_argument("There must be exactly one target space per boundary segment");
		int n = points.size();
		if (n < 3)
			throw std::invalid_argument("There must be at least three segments");

		tfms.resize(n);
		for (int k = 0; k < n; k++) {
			vec2 a = points[k];
			vec2 b = points[(k + 1) % n];
			vec2 p = 0.5f * (a + b);
			vec2 v = b - p;
			vec2 u = {-v.y, v.x};
			tfms[k].base = &UnitLine::instance;
			tfms[k].pos = p;
			tfms[k].tfm = mat2(u, v);
		}

		trans.resize(n);
		for (int k = 0; k < n; k++) {
			trans[k].into = targets[k];
			trans[k].shape = &tfms[k];
		}
	}

	std::vector<Transition> const &transitions() const noexcept {
		return trans;
	}

	auto begin() const noexcept {
		return trans.begin();
	}

	auto end() const noexcept {
		return trans.end();
	}
};

class MovedSpace: public Space {
public:
	Space *base = nullptr;
	mat2 transform = mat2(1.0f);
	vec2 origin = vec2(0.0f);

	Track trace(Ray ray, float maxd) const override {
		ray.base -= origin;
		ray.base = inverse(transform) * ray.base;
		ray.dir = inverse(transform) * ray.dir;
		Track t = base->trace(ray, maxd);
		t.dir = transform * t.dir;
		for (auto &pt: t.points)
			pt = transform * pt + origin;
		return t;
	}
};

void test() {
// 	FlatSpace s[5];
// 	ConvexBoundary cb({
// 		{-2.0f, 0.5f},
// 		{-2.0f, 1.0f},
// 		{0.0f, 1.7f},
// 		{3.0f, 1.0f},
// 		{3.0f, 0.5f},
// 	}, {
// 		&s[0], &s[1], &s[2], &s[3], &s[4],
// 	});
// 	for (vec2 pos: {vec2{0.0f, 0.7f}}) {
// 		for (vec2 dir: {vec2{1.0f, 0.0f}, vec2{0.707f, 0.707f}, vec2{-1.0f, 0.0f}, vec2{0.0f, 1.0f}}) {
// 			printf("%6.3f,%6.3f:%6.3f,%6.3f -> %ld\n", pos.x, pos.y, dir.x, dir.y, (FlatSpace *)cb.next({pos, dir}) - s);
// 		}
// 	}
/*
	printf("       ");
	for (int j = 0; j <= 15; j++) {
		float y = 0.1f * j;
		printf(" %6.3f", y);
	}
	puts("");
	for (int i = -30; i <= 40; i++) {
		float x = 0.1f * i;
		printf("%6.3f ", x);
		for (int j = 0; j <= 15; j++) {
			float y = 0.1f * j;
			printf(" %6.0f", cb.f({x, y}));
		}
		puts("");
	}
*/
// 	exit(0);
}

#define USE_TRANSFORM 1
#define RENDER_FULL_RIEMANN 0

void render() {
	double t0 = glfwGetTime();

// 	CoilMetric cm;
// 	cm.coil_scale = 2.0 + sin(.2 * t0);

	FlatSpace eucl;

	FastSpace sd;
// 	sd.inner_hl = clamp(1.125f - .875f * sinf(.2 * t0), sd.inner_pad, sd.outer_hl);

	FlatSpace fs;
	fs.desc = &sd;

	BoxSmoother bs;
	bs.base = &sd;
	bs.halfwidth = 0.5f;
	bs.pad = 0.125f;

	ConvexRiemannSubspace rsn;
	rsn.metric = &bs;

	ConvexRiemannSubspace rsp;
	rsp.metric = &bs;

#if RENDER_FULL_RIEMANN
	RiemannSpace rs;
	rs.metric = &bs;
#endif

#if USE_TRANSFORM
	float phi = .23 * t0;
	vec2 u{cos(phi), sin(phi)};
	mat2 tf = {u.x, u.y, -u.y, u.x};
	vec2 pos = {0.0f, 0.0f};

#if RENDER_FULL_RIEMANN
	MovedRiemannMetric<2> rm;
	rm.base = rs.metric;
	rm.origin = pos;
	rm.inv_transform = inverse(tf);
	rs.metric = &rm;
#endif

	MovedSpace fs1;
	fs1.base = &fs;

	MovedSpace rsn1;
	rsn1.base = &rsn;

	MovedSpace rsp1;
	rsp1.base = &rsp;

	fs1.transform = rsn1.transform = rsp1.transform = tf;
	fs1.origin = rsn1.origin = rsp1.origin = pos;

	SegmentedTransitions eucl_to_pipe({
		tf * vec2{-sd.outer_hl, -bs.halfwidth} + pos,
		tf * vec2{sd.outer_hl, -bs.halfwidth} + pos,
		tf * vec2{sd.outer_hl, -(bs.halfwidth - bs.pad)} + pos,
		tf * vec2{sd.outer_hl, (bs.halfwidth - bs.pad)} + pos,

		tf * vec2{sd.outer_hl, bs.halfwidth} + pos,
		tf * vec2{-sd.outer_hl, bs.halfwidth} + pos,
		tf * vec2{-sd.outer_hl, (bs.halfwidth - bs.pad)} + pos,
		tf * vec2{-sd.outer_hl, -(bs.halfwidth - bs.pad)} + pos,
	}, {
		&rsn1, &rsn1, &fs1, &rsp1, &rsp1, &rsp1, &fs1, &rsn1,
	});
	eucl.transitions.insert(eucl.transitions.end(), eucl_to_pipe.begin(), eucl_to_pipe.end());

	SegmentedTransitions pipe_out({
		{-sd.inner_hl, -(bs.halfwidth - bs.pad)},
		{-sd.inner_hl, (bs.halfwidth - bs.pad)},
		{sd.inner_hl, (bs.halfwidth - bs.pad)},
		{sd.inner_hl, -(bs.halfwidth - bs.pad)},
	}, {
		&eucl, &rsp1, &eucl, &rsn1,
	});
	fs.transitions.insert(fs.transitions.end(), pipe_out.begin(), pipe_out.end());

	ConvexBoundary pipe_top({
		{-sd.outer_hl, bs.halfwidth - bs.pad},
		{-sd.outer_hl, bs.halfwidth},
		{sd.outer_hl, bs.halfwidth},
		{sd.outer_hl, bs.halfwidth - bs.pad},
	}, {
		&eucl, &eucl, &eucl, &fs1,
	});
	rsp.boundary = &pipe_top;

	ConvexBoundary pipe_bottom({
		{sd.outer_hl, -(bs.halfwidth - bs.pad)},
		{sd.outer_hl, -bs.halfwidth},
		{-sd.outer_hl, -bs.halfwidth},
		{-sd.outer_hl, -(bs.halfwidth - bs.pad)},
	}, {
		&eucl, &eucl, &eucl, &fs1,
	});
	rsn.boundary = &pipe_bottom;

	std::unordered_map<Space *, vec4> colors = {
		{&eucl, {0.1f, 0.4f, 1.0f, 0.75f}},
		{&fs1, {0.4f, 1.0f, 0.1f, 0.75f}},
		{&rsp1, {1.0f, 0.4f, 0.1f, 0.75f}},
		{&rsn1, {1.0f, 0.1f, 0.4f, 0.75f}},
	};
#else
	SegmentedTransitions eucl_to_pipe({
		{-sd.outer_hl, -bs.halfwidth},
		{sd.outer_hl, -bs.halfwidth},
		{sd.outer_hl, -(bs.halfwidth - bs.pad)},
		{sd.outer_hl, (bs.halfwidth - bs.pad)},

		{sd.outer_hl, bs.halfwidth},
		{-sd.outer_hl, bs.halfwidth},
		{-sd.outer_hl, (bs.halfwidth - bs.pad)},
		{-sd.outer_hl, -(bs.halfwidth - bs.pad)},
	}, {
		&rsn, &rsn, &fs, &rsp, &rsp, &rsp, &fs, &rsn,
	});
	eucl.transitions.insert(eucl.transitions.end(), eucl_to_pipe.begin(), eucl_to_pipe.end());

	SegmentedTransitions pipe_out({
		{-sd.inner_hl, -(bs.halfwidth - bs.pad)},
		{-sd.inner_hl, (bs.halfwidth - bs.pad)},
		{sd.inner_hl, (bs.halfwidth - bs.pad)},
		{sd.inner_hl, -(bs.halfwidth - bs.pad)},
	}, {
		&eucl, &rsp, &eucl, &rsn,
	});
	fs.transitions.insert(fs.transitions.end(), pipe_out.begin(), pipe_out.end());

	ConvexBoundary pipe_top({
		{-sd.outer_hl, bs.halfwidth - bs.pad},
		{-sd.outer_hl, bs.halfwidth},
		{sd.outer_hl, bs.halfwidth},
		{sd.outer_hl, bs.halfwidth - bs.pad},
	}, {
		&eucl, &eucl, &eucl, &fs,
	});
	rsp.boundary = &pipe_top;

	ConvexBoundary pipe_bottom({
		{sd.outer_hl, -(bs.halfwidth - bs.pad)},
		{sd.outer_hl, -bs.halfwidth},
		{-sd.outer_hl, -bs.halfwidth},
		{-sd.outer_hl, -(bs.halfwidth - bs.pad)},
	}, {
		&eucl, &eucl, &eucl, &fs,
	});
	rsn.boundary = &pipe_bottom;

	std::unordered_map<Space *, vec4> colors = {
		{&eucl, {0.1f, 0.4f, 1.0f, 0.75f}},
		{&fs, {0.4f, 1.0f, 0.1f, 0.75f}},
		{&rsp, {1.0f, 0.4f, 0.1f, 0.75f}},
		{&rsn, {1.0f, 0.1f, 0.4f, 0.75f}},
	};
#endif

#if RENDER_GRID
	glBegin(GL_LINES);
	for (int i = -50; i <= 50; i++)
		for (int j = -50; j <= 50; j++) {
			float len = 0.1f;
			vec2 dir = vec2(0.707f, 0.707f);
			vec2 pt = 0.1f * vec2{i, j};
			auto r = fs.desc->toGlobal(fs.desc->fromGlobal(Ray{pt, dir}));
// 			auto r = fs.desc->fromGlobal(Ray{pt, dir});
			glColor4f(0.5f, 0.5f, 0.5f, 1.0f);
			glVertex2fv(value_ptr(pt));
			glColor4f(0.0, 1.0f, 0.0f, 1.0f);
			glVertex2fv(value_ptr(pt + len * dir));
// 			pt += 0.05f * fs.desc->fromGlobal(Ray{pt, vec2(0.707f, 0.707f)}).dir;
			pt = r.base;

			glColor4f(0.5f, 0.5f, 0.5f, 1.0f);
			glVertex2fv(value_ptr(pt));
			glColor4f(1.0, 0.0f, 0.0f, 1.0f);
			glVertex2fv(value_ptr(pt + len * r.dir));
		}
	glEnd();
#endif

	for (int k = -120; k <= 120; k++) {
		Space *s = &eucl;
		Ray r;
		r.base = {-5.2f, 0.0f};
		r.dir = normalize(vec2(1.0, k / 120.0));
		float rem = 20.0f;
		while (rem > 1.0f) {
			vec4 color = colors[s];
			Track track = s->trace(r, rem);
			assert(track.points.size() > 0);
			glColor4fv(value_ptr(color));
			glBegin(GL_LINE_STRIP);
			for (auto pt: track.points)
				glVertex2f(pt.x, pt.y);
			glEnd();

#if RENDER_SWITCH_DIRS
			float in_len = 0.05f;
			float out_len = 0.1f;
			glColor4f(1.0, 1.0, 0.0, 1.0);
			glBegin(GL_LINE_STRIP);
			glVertex2fv(value_ptr(track.points.back() - in_len * track.dir));
			glVertex2fv(value_ptr(track.points.back() + out_len * track.dir));
			glEnd();
#endif

			if (!track.next) {
				float r = 1.0f / 16.0f;
				vec2 pt = track.points.back();
				glBegin(GL_LINES);
				glVertex2f(pt.x - r, pt.y - r);
				glVertex2f(pt.x + r, pt.y + r);
				glVertex2f(pt.x - r, pt.y + r);
				glVertex2f(pt.x + r, pt.y - r);
				glEnd();
				break;
			}
			rem -= track.length;
			r.base = track.points.back();
			r.dir = track.dir;
			s = track.next;
		}
	}
// 	printf("%.3f: |a| ≈ %.3f(1 ± %.3f) ∈ [%.3f, %.3f]\n",
// 	       sd.inner_hl, rsp.a_len.mean(), rsp.a_len.reldev(), rsp.a_len.min(), rsp.a_len.max());

#if RENDER_FULL_RIEMANN
	glColor4f(0.4, 0.4, 0.4, 0.75);
	for (int k = -120; k <= 120; k++) {
		Ray r;
		r.base = {-5.2f, 0.0f};
		r.dir = normalize(vec2(1.0, k / 120.0));
		Track track = rs.trace(r, 20.0f);
		glBegin(GL_LINE_STRIP);
		for (auto pt: track.points)
			glVertex2f(pt.x, pt.y);
		glEnd();
	}
#endif
}

float background_lightness = 0.1;

int width, height;
float winsize;
GLuint fbs[2] = {0, 0};
GLuint bb = 0;
GLuint mb = 0;

void paint(GLFWwindow* window) {
	glBindFramebuffer(GL_FRAMEBUFFER, fbs[0]);
	glClearColor(0.0, 0.0, 0.0, 0.0);
	glClear(GL_COLOR_BUFFER_BIT);

	glLineWidth(2.3f);
	glLineWidth(winsize * 0.0085);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE);
	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_MULTISAMPLE);

	render();

	glPushMatrix();
	glLoadIdentity();

	glDisable(GL_MULTISAMPLE);
	glBindFramebuffer(GL_DRAW_FRAMEBUFFER, fbs[1]);
	glBlitFramebuffer(0, 0, width, height, 0, 0, width, height, GL_COLOR_BUFFER_BIT, GL_NEAREST);

	glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
	glClearColor(background_lightness, background_lightness, background_lightness, 1.0);
	glClear(GL_COLOR_BUFFER_BIT);
	glEnable(GL_BLEND);
	glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);

	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, mb);
	glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
	glBegin(GL_QUADS);
	glTexCoord2f(0.0f, 0.0f);
	glVertex2f(-1.0f, -1.0f);
	glTexCoord2f(1.0f, 0.0f);
	glVertex2f(1.0f, -1.0f);
	glTexCoord2f(1.0f, 1.0f);
	glVertex2f(1.0f, 1.0f);
	glTexCoord2f(0.0f, 1.0f);
	glVertex2f(-1.0f, 1.0f);
	glEnd();
	glBindTexture(GL_TEXTURE_2D, 0);
	glDisable(GL_TEXTURE_2D);

	glPopMatrix();

	glfwSwapBuffers(window);
}

void init() {
	glCreateFramebuffers(2, fbs);
}

void resized(GLFWwindow* window, int width, int height) {
	::width = width;
	::height = height;
	glViewport(0, 0, width, height);
	::winsize = sqrt(width * height / 2) / 4;
	glLoadIdentity();
	glScalef(winsize / width, winsize / height, 1.0);
	if (bb) {
		glDeleteTextures(1, &bb);
		glDeleteTextures(1, &mb);
	}
	glCreateTextures(GL_TEXTURE_2D_MULTISAMPLE, 1, &bb);
	glCreateTextures(GL_TEXTURE_2D, 1, &mb);
	glTextureStorage2DMultisample(bb, 8, GL_RGBA8, width, height, GL_FALSE);
	glTextureStorage2D(mb, 1, GL_RGBA8, width, height);
	glNamedFramebufferTexture(fbs[0], GL_COLOR_ATTACHMENT0, bb, 0);
	glNamedFramebufferTexture(fbs[1], GL_COLOR_ATTACHMENT0, mb, 0);
// 	paint(window);
}

void APIENTRY debug(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length, GLchar const *message, void const *userParam) {
	std::printf("%.*s\n", (int)length, message);
}

int main() {
	test();
	glfwInit();
	glfwWindowHint(GLFW_ALPHA_BITS, 8);
	glfwWindowHint(GLFW_DEPTH_BITS, 0);
	glfwWindowHint(GLFW_SAMPLES, 16);
	glfwWindowHint(GLFW_SRGB_CAPABLE, GLFW_TRUE);
	glfwWindowHint(GLFW_DOUBLEBUFFER, GLFW_TRUE);
	glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_API);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
	glfwWindowHint(GLFW_CONTEXT_ROBUSTNESS, GLFW_NO_RESET_NOTIFICATION);
	glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, GLFW_TRUE);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_COMPAT_PROFILE);
	auto wnd = glfwCreateWindow(1600, 1200, "Space Refraction 2D v1", nullptr, nullptr);
	glfwMakeContextCurrent(wnd);
	glDebugMessageCallback(debug, nullptr);
	init();
	glfwSwapInterval(0);
	glfwSetWindowRefreshCallback(wnd, paint);
	glfwSetFramebufferSizeCallback(wnd, resized);
	resized(wnd, 1600, 1200);
	glfwShowWindow(wnd);
	while (!glfwWindowShouldClose(wnd)) {
// 		glfwPollEvents();
		glfwWaitEventsTimeout(1.0 / 60.0);
		paint(wnd);
	}
	glfwDestroyWindow(wnd);
}
