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

class InwardsBoundary final: public SubspaceBoundary {
public:
	Params params;
	Subspace *side;
	Subspace *channel;

	SwitchPoint leave(vec2 from, vec2 dir) const override {
		const vec2 radius = {params.outer_half_length, params.outer_radius};
		const vec2 d = -sign(dir) * radius - from;
		const vec2 dists = d / dir;
		const vec2 px = from + dists.x * dir;
		const vec2 py = from + dists.y * dir;
		float dist = std::numeric_limits<float>::infinity();
		if (dists.x > 0.0f && abs(px.y) <= radius.y)
			dist = dists.x - eps;
		if (dists.y > 0.0f && abs(py.x) <= radius.x)
			dist = dists.y - eps;
		if (!std::isfinite(dist))
			return {{nullptr, from, dir}, {nullptr, from, dir}};
		vec2 pos = from + dist * dir;
		const TrackPoint pt1 = {nullptr, pos, dir};
		if (abs(pos.y) < params.inner_radius) {
			pos.x -= copysign(params.outer_half_length - params.inner_half_length, pos.x);
			return {pt1, {channel, pos, dir}};
		} else {
			return {pt1, {side, pos, dir}};
		}
	}
};

class ChannelBoundary final: public SubspaceBoundary {
public:
	Params params;
	Subspace *outer;
	Subspace *side;

	SwitchPoint leave(vec2 from, vec2 dir) const override {
		const vec2 radius = {params.inner_half_length, params.inner_radius};
		const vec2 d = sign(dir) * radius - from;
		const vec2 dist = d / dir;
		const float dist_outer = dist.x + eps;
		const float dist_side = dist.y;// + eps;

		vec2 pos = from + min(dist_outer, dist_side) * dir;
		const TrackPoint pt1 = {nullptr, pos, dir};

		if (dist_outer <= dist_side) {
			debugf("channel->outer (% .1f, % .1f)>>(% .3f, % .3f) -> ", pos.x, pos.y, dir.x, dir.y);
			pos.x += copysign(params.outer_half_length - params.inner_half_length, pos.x);
			debugf("(% .1f, % .1f)>>(% .3f, % .3f)\n", pos.x, pos.y, dir.x, dir.y);
			return {pt1, {outer, pos, dir}};
		} else {
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
			return {pt1, {side, pos, normalize(dir)}};
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

	TrackPoint leave(vec2 pos, vec2 dir) const override {
		Coefs cs(params);
		if (abs(pos.y) >= params.inner_radius)
			return {outer, pos, dir};
		if (abs(pos.x) >= params.outer_half_length && sign(dir.x) == sign(pos.x))
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

using std::shared_ptr, std::make_shared;

double t_frozen = 0.0;
double t_offset = 0.0;
bool active = true;

void render() {
 	double t0 = active ? glfwGetTime() - t_offset : t_frozen;

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
	ChannelBoundary cbnd;
	cbnd.outer = &outer;
	cbnd.side = &side;
	sbnd.outer = &outer;
	sbnd.channel = &channel;
	outer.boundary = &ibnd;
	channel.boundary = &cbnd;

	std::unordered_map<Subspace const *, shared_ptr<SpaceVisual>> visuals = {
		{nullptr, make_shared<SpaceVisual>(vec3{1.0f, 0.1f, 0.4f})},
		{&outer, make_shared<SpaceVisual>(vec3{0.1f, 0.4f, 1.0f})},
		{&channel, make_shared<ChannelVisual>(vec3{0.4f, 1.0f, 0.1f})},
		{&side, make_shared<SpaceVisual>(vec3{1.0f, 0.4f, 0.1f})},
	};

	float theta = .3 * t0;
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
		for (;;) {
			auto visual = visuals[pt.space];
			glColor4fv(value_ptr(vec4{visual->color, 0.75f}));
			auto track = pt.space->traceEx(pt.pos, pt.dir);
			assert(!track.points.empty());
			for (auto tp: track.points) {
				p = visual->where(tp);
				glVertex2fv(value_ptr(p));
			}
			if (track.end.space) {
				pt = track.end;
			} else {
				break;
			}
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
			paint(wnd);
			glfwPollEvents();
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
		}
	}
	glfwDestroyWindow(wnd);
#endif
}
