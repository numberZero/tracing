#undef NDEBUG
#include <cassert>
#include <cstdio>
#include <memory>
#include <limits>
#include <unordered_map>
#include <vector>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <GLFW/glfw3.h>
#include <GL/gl.h>
#include <GL/glext.h>
#include "averager.hxx"
#include "math.hxx"
#include "prog5/subspace.hxx"
#include "prog5/thing.hxx"

#define TEST 0
// #define debugf(...) printf(__VA_ARGS__)
#define debugf(...)

using namespace std::literals;

struct Params {
	float outer_radius = 3.0f;
	float inner_radius = 2.0f;
	float outer_half_length = 5.0f;
	float inner_half_length = 2.0f;
	float inner_pad = 1.25;
};

struct Coefs {
	float x1, y1, x2, y2;
	float x0, y0, w;

	Coefs(Params const &params) {
		x2 = params.inner_half_length;
		y2 = params.outer_half_length;
		x1 = params.inner_half_length - params.inner_pad;

		// TODO исправить неустойчивость при y2 ≈ x2
		y1 = x1 * (x1 - x2 + 2*y2) / (x1 + x2);
		x0 = 0.5 * (sqr(x2) + sqr(x1) - 2 * x2 * y2) / (x2 - y2);
		y0 = y2 - 0.25 * (sqr(x2) - sqr(x1)) / (x2 - y2);
		w = (x2 - y2) / (sqr(x2) - sqr(x1));
	}
};

static constexpr float eps = 1e-4;

class InwardsBoundary final: public SubspaceBoundaryEx {
public:
	Params params;
	Subspace *side;
	Subspace *channel;

	BoundaryPoint findBoundary(Ray from) const override {
		const vec2 radius = {params.outer_half_length, params.outer_radius};
		const vec2 d = -sign(from.dir) * radius - from.pos;
		const vec2 dists = d / from.dir;
		const vec2 px = from.pos + dists.x * from.dir;
		const vec2 py = from.pos + dists.y * from.dir;
		float dist = std::numeric_limits<float>::infinity();
		if (dists.x > 0.0f && abs(px.y) <= radius.y)
			dist = dists.x - eps;
		if (dists.y > 0.0f && abs(py.x) <= radius.x)
			dist = dists.y - eps;
		if (!std::isfinite(dist))
			return {{nullptr, from.pos, from.pos, mat2(1)}, dist};
		const vec2 pos = from.pos + dist * from.dir;
		return {leave({pos, from.dir}), 0.0};
	}

	std::vector<Transition> findOverlaps(vec2 pos, float max_distance) const override {
		const vec2 radius = {params.outer_half_length, params.outer_radius};
		std::vector<Transition> overlaps;
		if (all(lessThanEqual(abs(pos), radius + max_distance))) {
			if (abs(pos.y) - max_distance <= params.inner_radius) {
				assert(abs(pos.x) >= params.outer_half_length);
				vec2 into = pos;
				into.x -= copysign(params.outer_half_length - params.inner_half_length, pos.x);
				overlaps.push_back({channel, pos, into, mat2(1)});
			}
			if (abs(pos.y) + max_distance >= params.inner_radius) {
				overlaps.push_back({side, pos, pos, mat2(1)});
			}
		}
		return overlaps;
	}

	bool contains(vec2 point) const override {
		const vec2 radius = {params.outer_half_length, params.outer_radius};
		return any(greaterThan(abs(point), radius));
	}

	Transition leave(Ray at) const override {
		vec2 pos = at.pos;
		if (abs(pos.y) < params.inner_radius) {
			vec2 into = pos;
			into.x -= copysign(params.outer_half_length - params.inner_half_length, pos.x);
			return {channel, pos, into, mat2(1)};
		} else {
			return {side, pos, pos, mat2(1)};
		}
	}
};

class ChannelBoundary final: public SubspaceBoundaryEx {
public:
	Params params;
	Subspace *outer;
	Subspace *side;

	BoundaryPoint findBoundary(Ray from) const override {
		const vec2 radius = {params.inner_half_length, params.inner_radius};
		const vec2 d = sign(from.dir) * radius - from.pos;
		const vec2 dist = d / from.dir;
		const float dist_outer = dist.x + eps;
		const float dist_side = dist.y;// + eps;

		const vec2 pos = from.pos + min(dist_outer, dist_side) * from.dir;
		vec2 into = pos;

		if (dist_outer <= dist_side) {
			into.x += copysign(params.outer_half_length - params.inner_half_length, pos.x);
			return {{outer, pos, into, mat2(1)}, dist_outer};
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
			return {{side, pos, into, diagonal(m, 1)}, dist_side};
		}
	}

	std::vector<Transition> findOverlaps(vec2 pos, float max_distance) const override {
		std::vector<Transition> overlaps;
		if (abs(pos.x) + max_distance >= params.inner_half_length) {
			vec2 into = pos;
			into.x += copysign(params.outer_half_length - params.inner_half_length, pos.x);
			overlaps.push_back({outer, pos, into, mat2(1)});
		}
		if (abs(pos.y) + max_distance >= params.inner_radius) {
			Coefs cs(params);
			vec2 into = pos;
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

	bool contains(vec2 point) const override {
		const vec2 radius = {params.inner_half_length, params.inner_radius};
		return all(lessThan(abs(point), radius));
	}

	Transition leave(Ray at) const override {
		const vec2 radius = {params.inner_half_length, params.inner_radius};
		const vec2 pos = at.pos;
		vec2 into = pos;

		if (any(lessThanEqual(abs(pos), radius))) {
			into.x += copysign(params.outer_half_length - params.inner_half_length, pos.x);
			return {outer, pos, into, mat2(1)};
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

class ChannelMetric: public RiemannMetric<2> {
public:
	Params params;

	decomp halfmetric(vec2 pos) const noexcept override {
		Coefs cs(params);
		float x = pos.x;
		float dx = 1.0f;
		if (abs(x) < cs.y1) {
			dx = cs.x1 / cs.y1;
		} else if (abs(x) < cs.y2) {
			x = copysign(cs.x0 + sqrt((abs(x) - cs.y0) / cs.w), x);
			dx /= -2 * cs.w * (abs(x) - cs.x0);
		}
		return {mat2(1.0f), {dx, 1.0f}};
	}
};

class ChannelSideMetric: public ChannelMetric {
	decomp halfmetric(vec pos) const noexcept override {
		auto g = ChannelMetric::halfmetric(pos);
		float c = clamp((params.outer_radius - abs(pos.y)) / (params.outer_radius - params.inner_radius), 0.0f, 1.0f);
		c = smoothstep(c);
		return {g.ortho, mix(vec2(1.0f), g.diag, c)};
	}
};

class SideBoundary: public SwitchMap {
public:
	Params params;
	Subspace *outer;
	Subspace *channel;

	bool contains(vec2 point) const override {
		return abs(point.x) <= params.outer_half_length + eps && abs(point.y) <= params.outer_radius + eps && abs(point.y) >= params.inner_radius - eps;
	}

	Transition leave(const Ray at) const override {
		vec2 pos = at.pos;
		vec2 dir = at.dir;
		Coefs cs(params);
		if (abs(pos.y) >= params.inner_radius)
			return {outer, pos, pos, mat2(1)};
		if (abs(pos.x) >= params.outer_half_length && sign(dir.x) == sign(pos.x))
			return {outer, pos, pos, mat2(1)};
		debugf("side->channel (% .1f, % .1f)>>(% .3f, % .3f) -> ", pos.x, pos.y, dir.x, dir.y);
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
		debugf("(% .1f, % .1f)>>(% .3f, % .3f)\n", pos.x, pos.y, dir.x, dir.y);
		return {channel, at.pos, pos, diagonal(m, 1.0f)};
	}
};

void test() {
}

class SpaceVisual {
public:
	vec3 color = {0.9f, 0.1f, 0.4f};

	SpaceVisual() = default;
	SpaceVisual(vec3 color) : color(color) {}
	virtual ~SpaceVisual() = default;

	virtual vec2 where(vec2 pos) const {
		return pos;
	}

	virtual mat2 jacobi(vec2 pos) const {
		return mat2(1.0f);
	}
};

class ChannelVisual: public SpaceVisual {
public:
	using SpaceVisual::SpaceVisual;

	Params params;

	vec2 where(vec2 pos) const override {
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

	mat2 jacobi(vec2 pos) const override {
		Coefs cs(params);
		if (abs(pos.x) < cs.x1) {
			return diagonal(cs.y1 / cs.x1, 1.0f);
		} else if (abs(pos.x) < cs.x2) {
			return diagonal(2 * cs.w * (abs(pos.x) - cs.x0), 1.0f);
		} else {
			return mat2(1.0f);
		}
	}
};

using std::shared_ptr, std::make_shared;

double t_frozen = 0.0;
double t_offset = 0.0;
bool active = true;

class MyUniverse {
public:
	Params params;
	Coefs cs{params};
	Thing a_thing;
	ThingySubspace outer, channel;
	RiemannSubspace side;
	ChannelSideMetric side_metric;
	SideBoundary sbnd;
	InwardsBoundary ibnd;
	ChannelBoundary cbnd;

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
	}
};

MyUniverse uni;
float off = .5f * uni.params.outer_half_length;
float A = uni.params.inner_half_length + off;
float omega = 1.0f;
ThingLocation locs[] = {
	{&uni.outer, {-(uni.params.outer_half_length + off), 0.0f}},
	{&uni.outer, {-A, uni.params.outer_radius + off}},
};

void render() {
	static double t0 = 0.0;
	double t = active ? glfwGetTime() - t_offset : t_frozen;
	float dt = active ? t - t0 : 0.0;
	t0 = t;

	std::unordered_map<Subspace const *, shared_ptr<SpaceVisual>> visuals = {
		{nullptr, make_shared<SpaceVisual>(vec3{1.0f, 0.1f, 0.4f})},
		{&uni.outer, make_shared<SpaceVisual>(vec3{0.1f, 0.4f, 1.0f})},
		{&uni.channel, make_shared<ChannelVisual>(vec3{0.4f, 1.0f, 0.1f})},
		{&uni.side, make_shared<SpaceVisual>(vec3{1.0f, 0.4f, 0.1f})},
	};

	auto a_thing = reinterpret_cast<Thing *>(&uni.a_thing);
	const float thing_radius = 0.5f;
	uni.outer.things.clear();
	uni.channel.things.clear();
	for (auto &loc: locs) {
		loc.space->things.push_back({a_thing, loc.pos, thing_radius});
		for (auto over: loc.space->boundary->findOverlaps(loc.pos, thing_radius)) {
			if (auto flat = dynamic_cast<ThingySubspace *>(over.into)) {
				flat->things.push_back({a_thing, over.intoPos, .5});
			} else {
			}
		}
		vec2 v = vec2(A * omega * sin(omega * t), 0.0f);
		loc.pos += dt * v;
		if (!loc.space->boundary->contains(loc.pos)) {
			auto next = loc.space->boundary->leave({loc.pos, v});
			if (auto flat = dynamic_cast<ThingySubspace *>(next.into)) {
				loc.space = const_cast<ThingySubspace *>(flat); // все ThingySubspace неконстантны
				loc.pos = next.intoPos;
			} else {
				throw "Oops! A thing is destroyed by the space curvature";
			}
		}
	}

	float theta = .3 * t;
	int N = 120;
	for (int k = -N; k < N; k++) {
		float phi = (.5 + k) * (M_PI / N);
		TrackPoint pt;
		pt.pos = 8.0f * vec2(cos(theta), sin(theta));
		pt.dir = vec2(cos(phi), sin(phi));
		pt.space = &uni.outer;
		int n = 0;
		vec2 p, v;
		glBegin(GL_LINE_STRIP);
		for (;;) {
			auto visual = visuals[pt.space];
			auto traced = pt.space->trace(pt);
			glColor4fv(value_ptr(vec4{visual->color, 0.75f}));
			glVertex2fv(value_ptr(visual->where(pt.pos)));
			p = visual->where(traced.end.pos);
			v = visual->jacobi(traced.end.pos) * pt.dir;
			if (auto flat = dynamic_cast<ThingySubspace const *>(pt.space)) {
				if (auto t = flat->traceToThing(pt); t.thing) {
					// TODO Если track.to.space — ThingySubspace, там может найтись Thing ещё ближе; надо проверять.
					p = visual->where(t.incident.pos);
					v = visual->jacobi(t.incident.pos) * pt.dir;
					glVertex2fv(value_ptr(p));
					glColor3f(1, 1, 0);
					glVertex2fv(value_ptr(p));
					glVertex2fv(value_ptr(p + .5f * v));
					break;
				}
			}
			glVertex2fv(value_ptr(p));
			if (traced.to.space) {
				pt = traced.to;
			} else {
				glVertex2fv(value_ptr(p + 50.f * v));
				break;
			}
			if (n++ > 10) {
				glColor3f(1, 0, 0);
				glVertex2fv(value_ptr(p));
				glVertex2fv(value_ptr(p + 1.f * v));
				break;
			}
		}
		glEnd();
	}

	vec4 colors[] = {
		{.2, .2, .2, .75},
		{.8, .2, .2, .75},
		{.2, .8, .2, .75},
		{.8, .8, .2, .75},
		{.2, .2, .8, .75},
		{.8, .2, .8, .75},
		{.2, .8, .8, .75},
		{.8, .8, .8, .75},
	};
	int k = 0;
	int M = 30;
	for (auto const &bnd: {&uni.outer, &uni.channel}) {
		auto &&visual = visuals.at(bnd);
		for (auto &&info: bnd->things) {
			glColor3fv(value_ptr(visual->color));
// 			glColor4fv(value_ptr(colors[k++%8]));
// 			glColor4f(.8, .8, .8, .75);
			glBegin(GL_LINE_LOOP);
			for (int k = -M; k < M; k++) {
				float phi = (.5 + k) * (M_PI / M);
				glVertex2fv(value_ptr(visual->where(info.pos + info.radius * vec2(cos(phi), sin(phi)))));
			}
			glEnd();
		}
	}

	glBegin(GL_LINE_LOOP);
	glColor3f(.0f, .9f, .9f);
	glVertex2f(-uni.params.outer_half_length, -uni.params.outer_radius);
	glVertex2f(-uni.params.outer_half_length, uni.params.outer_radius);
	glVertex2f(uni.params.outer_half_length, uni.params.outer_radius);
	glVertex2f(uni.params.outer_half_length, -uni.params.outer_radius);
	glEnd();
	glBegin(GL_LINES);
	glColor3f(.0f, .2f, .7f);
	glVertex2f(-uni.params.outer_half_length, -uni.params.inner_radius);
	glVertex2f(uni.params.outer_half_length, -uni.params.inner_radius);
	glVertex2f(-uni.params.outer_half_length, uni.params.inner_radius);
	glVertex2f(uni.params.outer_half_length, uni.params.inner_radius);
	glVertex2f(-uni.cs.y1, -uni.params.outer_radius);
	glVertex2f(-uni.cs.y1, uni.params.outer_radius);
	glVertex2f(uni.cs.y1, uni.params.outer_radius);
	glVertex2f(uni.cs.y1, -uni.params.outer_radius);
	glEnd();
}

float background_lightness = 0.1;

int width, height;
float winsize;
GLuint fbs[2] = {0, 0};
GLuint bb = 0;
GLuint mb = 0;

int frames = 0;

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

	glLoadIdentity();
	glScalef(0.5f, 0.5f, 0.5f);

	render();

	glLoadIdentity();

	glMatrixMode(GL_PROJECTION);
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
	glMatrixMode(GL_MODELVIEW);

	glfwSwapBuffers(window);
	frames++;
}

void init() {
	glCreateFramebuffers(2, fbs);
}

void resized(GLFWwindow* window, int width, int height) {
	::width = width;
	::height = height;
	glViewport(0, 0, width, height);
	::winsize = sqrt(width * height / 2) / 4;
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glScalef(winsize / width, winsize / height, 1.0);
	glMatrixMode(GL_MODELVIEW);
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
}

static const char *title = "Space Refraction 2D v2";

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
	if (key == GLFW_KEY_SPACE) {
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
	if (key == GLFW_KEY_ENTER && mods == GLFW_MOD_ALT)
		toggle_fullscreen(window);
	if (key == GLFW_KEY_B) {
		background_lightness = 1.0f - background_lightness;
		paint(window);
	}
}

void APIENTRY debug(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length, GLchar const *message, void const *userParam) {
	std::printf("%.*s\n", (int)length, message);
}

int main() {
#if TEST
	test();
#else
	glfwInit();
	glfwWindowHint(GLFW_ALPHA_BITS, 8);
	glfwWindowHint(GLFW_DEPTH_BITS, 0);
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
	init();
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
			paint(wnd);
			double t1 = glfwGetTime();
			if (t1 - t0 >= 1.0) {
				double fps = frames / (t1 - t0);
				t0 = t1;
				frames = 0;
				char title[256];
				snprintf(title, sizeof(title), "%s @ %.1f FPS", ::title, fps);
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
}
