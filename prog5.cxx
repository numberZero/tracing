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
#include "desc/riemann.hxx"
#include "desc/ray.hxx"
#include "desc/euclidean.hxx"
#include "desc/flat.hxx"
#include "desc/fast.hxx"
#include "desc/box.hxx"
#include "prog5/subspace.hxx"

#define TEST 0
// #define debugf(...) printf(__VA_ARGS__)
#define debugf(...)

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

class InwardsBoundary final: public FlatExit {
public:
	Params params;
	Subspace *side;
	Subspace *channel;

	float distance(vec2 from, vec2 dir) const override {
		vec2 radius = {params.outer_half_length, params.outer_radius};
		vec2 d = -sign(dir) * radius - from;
		vec2 dists = d / dir;
		vec2 px = from + dists.x * dir;
		vec2 py = from + dists.y * dir;
		if (abs(px.y) <= radius.y)
			return dists.x - eps;
		if (abs(py.x) <= radius.x)
			return dists.y - eps;
		return std::numeric_limits<float>::infinity();
	}

	TrackPoint leave(vec2 pos, vec2 dir) const override {
		if (abs(pos.y) < params.inner_radius) {
			pos.x -= copysign(params.outer_half_length - params.inner_half_length, pos.x);
			return {channel, pos, dir};
		} else {
			return {side, pos, dir};
		}
	}
};

class ChannelOutwardsBoundary final: public FlatExit {
public:
	Params params;
	Subspace *outer;

	float distance(vec2 from, vec2 dir) const override {
		float d = sign(dir.x) * params.inner_half_length - from.x;
		float dist = d / dir.x;
		debugf("channel->outer (% .1f, % .1f)>>(% .3f, % .3f) @ %.3f\n", from.x, from.y, dir.x, dir.y, dist);
		return dist + eps;
	}

	TrackPoint leave(vec2 pos, vec2 dir) const override {
		debugf("channel->outer (% .1f, % .1f)>>(% .3f, % .3f) -> ", pos.x, pos.y, dir.x, dir.y);
		pos.x += copysign(params.outer_half_length - params.inner_half_length, pos.x);
		debugf("(% .1f, % .1f)>>(% .3f, % .3f)\n", pos.x, pos.y, dir.x, dir.y);
		return {outer, pos, dir};
	}
};

class ChannelSideBoundary final: public FlatExit {
public:
	Params params;
	Subspace *side;

	float distance(vec2 from, vec2 dir) const override {
		float d = sign(dir.y) * params.inner_radius - from.y;
		float dist = d / dir.y;
		debugf("channel->side (% .1f, % .1f)>>(% .3f, % .3f) @ %.3f\n", from.x, from.y, dir.x, dir.y, dist);
		return dist - eps;
	}

	TrackPoint leave(vec2 pos, vec2 dir) const override {
		Coefs cs(params);
		debugf("channel->side (% .1f, % .1f)>>(% .3f, % .3f) -> ", pos.x, pos.y, dir.x, dir.y);
		if (abs(pos.x) < cs.x1) {
			pos.x *= cs.y1 / cs.x1;
			dir.x *= cs.y1 / cs.x1;
		} else if (abs(pos.x) < cs.x2) {
			dir.x *= 2 * cs.w * (abs(pos.x) - cs.x0);
			pos.x = copysign(cs.y0 + cs.w * sqr(abs(pos.x) - cs.x0), pos.x);
		} else {
			pos.x += copysign(cs.y2 - cs.x2, pos.x);
		}
		debugf("(% .1f, % .1f)>>(% .3f, % .3f)\n", pos.x, pos.y, dir.x, dir.y);
		return {side, pos, normalize(dir)};
	}
};

class ChannelSideMetric: public RiemannMetric<2> {
public:
	Params params;

	decomp halfmetric(vec2 pos) const noexcept final override {
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

class SideBoundary: public SwitchMap {
public:
	Params params;
	Subspace *outer;
	Subspace *channel;

	bool contains(vec2 point) const override {
		return abs(point.x) <= params.outer_half_length + eps && abs(point.y) <= params.outer_radius + eps && abs(point.y) >= params.inner_radius - eps;
	}

	TrackPoint leave(vec2 pos, vec2 dir) const override {
		Coefs cs(params);
		if (abs(pos.y) >= params.inner_radius)
			return {outer, pos, dir};
		debugf("side->channel (% .1f, % .1f)>>(% .3f, % .3f) -> ", pos.x, pos.y, dir.x, dir.y);
		if (abs(pos.x) < cs.y1) {
			pos.x *= cs.x1 / cs.y1;
			dir.x *= cs.x1 / cs.y1;
		} else if (abs(pos.x) < cs.y2) {
			pos.x = copysign(sqrt((abs(pos.x) - cs.y0) / cs.w) - cs.x0, pos.x);
			dir.x /= 2 * cs.w * (abs(pos.x) - cs.x0);
		} else {
			pos.x -= copysign(cs.y2 - cs.x2, pos.x);
		}
		debugf("(% .1f, % .1f)>>(% .3f, % .3f)\n", pos.x, pos.y, dir.x, dir.y);
		return {channel, pos, normalize(dir)};
	}
};

void test() {
	Params params;
	Coefs cs(params);
	FlatSubspace outer, channel;
	RiemannSubspace side;
	ChannelSideMetric side_metric;
	SideBoundary sbnd;
	side.metric = &side_metric;
	side.map = &sbnd;
	InwardsBoundary ibnd;
	ibnd.side = &side;
	ibnd.channel = &channel;
	ChannelOutwardsBoundary cobnd;
	ChannelSideBoundary csbnd;
// 	SideOutwardsBoundary sobnd;
// 	SideChannelBoundary scbnd;
	cobnd.outer = &outer;
	csbnd.side = &side;
	sbnd.outer = &outer;
	sbnd.channel = &channel;
	outer.exits.push_back(&ibnd);
// 	side.exits.push_back(&sobnd);
// 	side.exits.push_back(&scbnd);
	channel.exits.push_back(&cobnd);
	channel.exits.push_back(&csbnd);

	std::unordered_map<Subspace const *, char const *> names = {
		{nullptr, "none"},
		{&outer, "outer"},
		{&channel, "channel"},
		{&side, "side"},
	};

	TrackPoint pt0 = {nullptr, {-4.8, 0.0}, {-0.168, 0.986}};
	TrackPoint pt1 = sbnd.leave(pt0.pos, pt0.dir);
	TrackPoint pt2 = csbnd.leave(pt1.pos, pt1.dir);
	debugf("(% .1f, % .1f)>>(% .3f, % .3f) %s\n", pt0.pos.x, pt0.pos.y, pt0.dir.x, pt0.dir.y, "0");
	debugf("(% .1f, % .1f)>>(% .3f, % .3f) %s\n", pt1.pos.x, pt1.pos.y, pt1.dir.x, pt1.dir.y, "1");
	debugf("(% .1f, % .1f)>>(% .3f, % .3f) %s\n", pt2.pos.x, pt2.pos.y, pt2.dir.x, pt2.dir.y, "2");

	TrackPoint pt;
	pt.pos = {-4.0, -6.0};
	pt.dir = normalize(vec2(-1, 5));
	pt.space = &outer;
	int n = 0;
	for (;;){
		debugf("(% .1f, % .1f)>>(% .3f, % .3f) %s\n", pt.pos.x, pt.pos.y, pt.dir.x, pt.dir.y, names[pt.space]);
		if (!pt.space)
			break;
		auto next = pt.space->trace(pt.pos, pt.dir);
		if (!next.space)
			next.pos = pt.pos;
		pt = next;
		if (n++ > 10)
			break;
	}
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
};

using namespace std;

void render() {
	double t0 = glfwGetTime();

	Params params;
	Coefs cs(params);
	FlatSubspace outer, channel;
	RiemannSubspace side;
	ChannelSideMetric side_metric;
	SideBoundary sbnd;
	side.metric = &side_metric;
	side.map = &sbnd;
	InwardsBoundary ibnd;
	ibnd.side = &side;
	ibnd.channel = &channel;
	ChannelOutwardsBoundary cobnd;
	ChannelSideBoundary csbnd;
	cobnd.outer = &outer;
	csbnd.side = &side;
	sbnd.outer = &outer;
	sbnd.channel = &channel;
	outer.exits.push_back(&ibnd);
	channel.exits.push_back(&cobnd);
	channel.exits.push_back(&csbnd);

	std::unordered_map<Subspace const *, shared_ptr<SpaceVisual>> visuals = {
		{nullptr, make_shared<SpaceVisual>(vec3{0.9f, 0.1f, 0.4f})},
		{&outer, make_shared<SpaceVisual>(vec3{0.1f, 0.4f, 0.9f})},
		{&channel, make_shared<ChannelVisual>(vec3{0.4f, 0.9f, 0.1f})},
		{&side, make_shared<SpaceVisual>(vec3{1.0f, 0.4f, 0.1f})},
	};

	float theta = .3*t0;
	int N = 120;
	for (int k = -N; k < N; k++) {
		float phi = (.5 + k) * (M_PI / N);
		TrackPoint pt;
		pt.pos = 8.0f * vec2(cos(theta), sin(theta));
		pt.dir = vec2(cos(phi), sin(phi));
		pt.space = &outer;
		int n = 0;
		vec2 p;
		glBegin(GL_LINE_STRIP);
		while (pt.space) {
			auto visual = visuals[pt.space];
			p = visual->where(pt.pos);
			glColor3fv(value_ptr(visual->color));
			glVertex2fv(value_ptr(p));
			auto next = pt.space->trace(pt.pos, pt.dir);
			if (!next.space)
				next.pos = pt.pos;
			pt = next;
			visual = visuals[pt.space];
			p = visual->where(pt.pos);
			glVertex2fv(value_ptr(p));
			if (n++ > 10) {
				glColor3f(1, 0, 0);
				break;
			}
		}
		glVertex2fv(value_ptr(p + 50.f * pt.dir));
		glEnd();
	}

	glBegin(GL_LINE_LOOP);
	glColor3f(.0f, .9f, .9f);
	glVertex2f(-params.outer_half_length, -params.outer_radius);
	glVertex2f(-params.outer_half_length, params.outer_radius);
	glVertex2f(params.outer_half_length, params.outer_radius);
	glVertex2f(params.outer_half_length, -params.outer_radius);
	glEnd();
	glBegin(GL_LINES);
	glColor3f(.0f, .2f, .7f);
	glVertex2f(-params.outer_half_length, -params.inner_radius);
	glVertex2f(params.outer_half_length, -params.inner_radius);
	glVertex2f(-params.outer_half_length, params.inner_radius);
	glVertex2f(params.outer_half_length, params.inner_radius);
	glVertex2f(-cs.y1, -params.outer_radius);
	glVertex2f(-cs.y1, params.outer_radius);
	glVertex2f(cs.y1, params.outer_radius);
	glVertex2f(cs.y1, -params.outer_radius);
	glEnd();
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

void keyed(GLFWwindow *window, int key, int scancode, int action, int mods) {
	if (key == GLFW_KEY_ESCAPE)
		glfwSetWindowShouldClose(window, GLFW_TRUE);
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
	glfwWindowHint(GLFW_SAMPLES, 16);
	glfwWindowHint(GLFW_SRGB_CAPABLE, GLFW_TRUE);
	glfwWindowHint(GLFW_DOUBLEBUFFER, GLFW_TRUE);
	glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_API);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
	glfwWindowHint(GLFW_CONTEXT_ROBUSTNESS, GLFW_NO_RESET_NOTIFICATION);
	glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, GLFW_TRUE);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_COMPAT_PROFILE);
	auto wnd = glfwCreateWindow(1600, 1200, "Space Refraction 2D v2", nullptr, nullptr);
	glfwMakeContextCurrent(wnd);
	glDebugMessageCallback(debug, nullptr);
	init();
	glfwSwapInterval(0);
	glfwSetWindowRefreshCallback(wnd, paint);
	glfwSetFramebufferSizeCallback(wnd, resized);
	glfwSetKeyCallback(wnd, keyed);
	resized(wnd, 1600, 1200);
	glfwShowWindow(wnd);
	while (!glfwWindowShouldClose(wnd)) {
// 		glfwPollEvents();
		glfwWaitEventsTimeout(1.0 / 60.0);
		paint(wnd);
	}
	glfwDestroyWindow(wnd);
#endif
}

