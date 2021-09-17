#include <cstdio>
#include <cmath>
#include <libgen.h>
#include <unistd.h>
#include <GL/gl.h>
#include <GL/glext.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <png++/png.hpp>
#include "shader.hxx"
#include "io.hxx"

namespace prog {
	unsigned quad;
	unsigned tracer;
}

namespace tex {
	unsigned grid;
}

void load_shaders() {
	prog::quad = link_program({
		compile_shader(GL_VERTEX_SHADER, read_file("empty.v.glsl")),
		compile_shader(GL_GEOMETRY_SHADER, read_file("screen_quad.g.glsl")),
		compile_shader(GL_FRAGMENT_SHADER, read_file("screen_quad.f.glsl")),
	});
	prog::tracer = link_program({
		compile_shader(GL_VERTEX_SHADER, read_file("empty.v.glsl")),
		compile_shader(GL_GEOMETRY_SHADER, read_file("tracer.g.glsl")),
		compile_shader(GL_FRAGMENT_SHADER, read_file("tracer.f.glsl")),
		compile_shader(GL_FRAGMENT_SHADER, read_file("space.glsl")),
		compile_shader(GL_FRAGMENT_SHADER, read_file("trace.glsl")),
	});
}

static unsigned load_texture(std::string const &filename, GLenum format = GL_RGBA8) {
	unsigned texture;
	png::image<png::rgba_pixel, png::solid_pixel_buffer<png::rgba_pixel>> image(filename);
	printf("Got texture %dx%d from %s\n", image.get_width(), image.get_height(), filename.c_str());
	glCreateTextures(GL_TEXTURE_2D, 1, &texture);
	auto &&data = image.get_pixbuf().get_bytes();
	glTextureStorage2D(texture, 1, format, image.get_width(), image.get_height());
	glTextureSubImage2D(texture, 0, 0, 0, image.get_width(), image.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, data.data());
	printf("Got pixel %d, %d, %d, %d\n", data[0], data[1], data[2], data[3]);
	return texture;
}

void load_textures() {
	tex::grid = load_texture("grid.png");
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
	load_textures();
	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	float fps = 0.0f;
	double t0 = glfwGetTime();
	int n = 0;
	while (!glfwWindowShouldClose(window)) {
		glfwPollEvents();
		int width, height;
		glfwGetWindowSize(window, &width, &height);
		glViewport(0, 0, width, height);
		float size = fmin(width, height);

		glClear(GL_COLOR_BUFFER_BIT);

		glUseProgram(prog::tracer);
		glBindTextureUnit(0, tex::grid);
		glUniform2f(0, width / size, height / size);
		glUniform1f(1, dt);
		glDrawArrays(GL_POINTS, 0, 1);

		glfwSwapBuffers(window);
		n++;
		double t1 = glfwGetTime();
		if (t1 - t0 >= 1.0) {
			fps = n / (t1 - t0);
			t0 = t1;
			n = 0;
			char buf[128];
			sprintf(buf, "FPS: %.0f", fps);
			glfwSetWindowTitle(window, buf);
		}
	}
	glfwDestroyWindow(window);
	return 0;
}
