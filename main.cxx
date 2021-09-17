#include <cstdio>
#include <cmath>
#include <libgen.h>
#include <unistd.h>
#include <GL/gl.h>
#include <GL/glext.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "shader.hxx"
#include "io.hxx"

namespace prog {

unsigned quad;
unsigned tracer;

}

void load_shaders() {
	prog::quad = link_program({
		compile_shader(GL_VERTEX_SHADER, read_file("empty.v.glsl")),
		compile_shader(GL_GEOMETRY_SHADER, read_file("screen_quad.g.glsl")),
		compile_shader(GL_FRAGMENT_SHADER, read_file("screen_quad.f.glsl")),
	});
	prog::tracer = link_program({
		compile_shader(GL_VERTEX_SHADER, read_file("empty.v.glsl")),
		compile_shader(GL_GEOMETRY_SHADER, read_file("tracer_quarter.g.glsl")),
		compile_shader(GL_FRAGMENT_SHADER, read_file("tracer.f.glsl")),
		compile_shader(GL_FRAGMENT_SHADER, read_file("trace.glsl")),
	});
}

float c(glm::vec2 p) {
	return 1.0f / (1.0f + 1.0f * exp(-10.0f * p.x * p.x));
}

float dt = 1.0f / 128.0f;

float c_x(glm::vec2 p) {
	auto h = 0.25f * dt;
// 	static constexpr float h = 1.0f / (1 << 12);
	return (c({p.x + h, p.y}) - c({p.x - h, p.y})) / (2.0f * h);
}

float c_y(glm::vec2 p) {
	auto h = 0.25f * dt;
// 	static constexpr float h = 1.0f / (1 << 12);
	return (c({p.x, p.y + h}) - c({p.x, p.y - h})) / (2.0f * h);
}

void on_scroll(GLFWwindow *window, double dx, double dy) {
	char buf[128];
	dt *= pow(2.0, -0.25 * dy);
	sprintf(buf, "dt: 1/%.4g", 1.0 / dt);
	glfwSetWindowTitle(window, buf);
}

void APIENTRY debug(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length, GLchar const *message, void const *userParam) {
	std::printf("%.*s\n", (int)length, message);
}

int main(int argc, char *argv[])
{
	chdir(dirname(dirname(argv[0])));
	glfwInit();
	glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_API);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_COMPAT_PROFILE);
	glfwWindowHint(GLFW_CONTEXT_ROBUSTNESS, GLFW_LOSE_CONTEXT_ON_RESET);
	glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, GLFW_TRUE);
	glfwWindowHint(GLFW_SAMPLES, 4);
	GLFWwindow *window = glfwCreateWindow(800, 600, "Space Refraction", nullptr, nullptr);
	glfwShowWindow(window);
	glfwSetScrollCallback(window, on_scroll);
	glfwMakeContextCurrent(window);
	glDebugMessageCallback(debug, nullptr);
	load_shaders();
	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	while (!glfwWindowShouldClose(window)) {
		glfwPollEvents();
		int width, height;
		glfwGetWindowSize(window, &width, &height);
		glViewport(0, 0, width, height);
		float size = fmin(width, height);

		glClear(GL_COLOR_BUFFER_BIT);

		glUseProgram(prog::tracer);
		glUniform2f(0, width / size, height / size);
		glUniform1f(1, dt);
		glDrawArrays(GL_POINTS, 0, 1);
		glUseProgram(0);
/*
		glLoadIdentity();
		glOrtho(-width / size, width / size, -height / size, height / size, -1.0f, 1.0f);
		glColor4f(0.0f, 0.5f, 1.0f, 0.5f);
		for (float x = -1.0; x <= 1.0; x += 1.0f / 16.0f) {
			for (float y = -1.0; y <= 1.0; y += 1.0f / 16.0f) {
				float s = 1.0f / 128.0f;
				glBegin(GL_LINE_LOOP);
				glVertex2f(x - s, y);
				glVertex2f(x, y - s * c({x, y}));
				glVertex2f(x + s, y);
				glVertex2f(x, y + s * c({x, y}));
				glEnd();
			}
		}

// 		for (int angle = -60; angle <= 60; angle += 5) {
		for (int i = 0; i < 160; i++) {
			constexpr float t = 3.0;
// 			const float dt = 0.001;
			const int N = t / dt;
// 			const float phi = glm::radians<float>(angle);
// 			const float a = cos(phi);
// 			const float s = sin(phi);
			const float u = (i + 0.5f) / 80.0f - 1.0f;
			glm::vec2 p0 = {-1.0f, 0.0f};

			float dx = 1.0f;
			float dy = u;
			glm::vec2 p = p0;
			glColor4f(0.5f, 0.5f, 0.5f, 0.5f);
			glBegin(GL_LINE_STRIP);
			glVertex2fv(glm::value_ptr(p));
			glVertex2fv(glm::value_ptr(p + 3.0f * glm::vec2{dx, dy}));
			glEnd();

			glColor4f(1.0f, 1.0f, 0.0f, 1.0f);
			glBegin(GL_LINE_STRIP);
			glVertex2fv(glm::value_ptr(p));
			while (p.x < 1.0f && fabs(p.y) < 1.0f) {
				float ddx = 0.5f * c_x(p) * dy * dy;
				float ddy = -c_x(p) / c(p) * dx * dy + 0.5f * c_y(p) / c(p) * dy * dy;
				dx += dt * ddx;
				dy += dt * ddy;
				p += dt * glm::vec2{dx, dy};
				glVertex2fv(glm::value_ptr(p));
			}
			glEnd();
		}
*/
		glfwSwapBuffers(window);
	}
	glfwDestroyWindow(window);
	return 0;
}
