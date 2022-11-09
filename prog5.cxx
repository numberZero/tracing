#undef NDEBUG
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

void test() {
// 	exit(0);
}

void render() {
	double t0 = glfwGetTime();
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
}

void keyed(GLFWwindow *window, int key, int scancode, int action, int mods) {
	if (key == GLFW_KEY_ESCAPE)
		glfwSetWindowShouldClose(window, GLFW_TRUE);
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
}

