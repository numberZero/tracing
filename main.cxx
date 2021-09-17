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
	unsigned uv_quad;
	unsigned uv_tracer;
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
	prog::uv_quad = link_program({
		compile_shader(GL_VERTEX_SHADER, read_file("empty.v.glsl")),
		compile_shader(GL_GEOMETRY_SHADER, read_file("screen_quad.g.glsl")),
		compile_shader(GL_FRAGMENT_SHADER, read_file("screen_quad_uv.f.glsl")),
	});
	prog::tracer = link_program({
		compile_shader(GL_VERTEX_SHADER, read_file("empty.v.glsl")),
		compile_shader(GL_GEOMETRY_SHADER, read_file("tracer.g.glsl")),
		compile_shader(GL_FRAGMENT_SHADER, read_file("tracer.f.glsl")),
		compile_shader(GL_FRAGMENT_SHADER, read_file("space.glsl")),
		compile_shader(GL_FRAGMENT_SHADER, read_file("trace.glsl")),
	});
	prog::uv_tracer = link_program({
		compile_shader(GL_VERTEX_SHADER, read_file("empty.v.glsl")),
		compile_shader(GL_GEOMETRY_SHADER, read_file("tracer.g.glsl")),
		compile_shader(GL_FRAGMENT_SHADER, read_file("tracer_uv.f.glsl")),
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
	return texture;
}

void load_textures() {
	tex::grid = load_texture("grid.png");
}

bool indirect = true;
int scale = 2;
float dt = 1.0f / 128.0f;

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
	float fps = 0.0f;
	double t0 = glfwGetTime();
	int n = 0;

	int last_width = 0, last_height = 0;
	unsigned uvmap = 0;
	unsigned fb;
	glCreateFramebuffers(1, &fb);

	while (!glfwWindowShouldClose(window)) {
		glfwPollEvents();
		int width, height;
		glfwGetWindowSize(window, &width, &height);
		glViewport(0, 0, width, height);
		float size = fmin(width, height);
		if (width != last_width || height != last_height) {
			last_width = width;
			last_height = height;
			glDeleteTextures(1, &uvmap);
			glCreateTextures(GL_TEXTURE_2D, 1, &uvmap);
			glTextureStorage2D(uvmap, 1, GL_RG32F, width / scale, height / scale);
			glNamedFramebufferTexture(fb, GL_COLOR_ATTACHMENT0, uvmap, 0);
		}

		if (indirect) {
			glViewport(0, 0, width / scale, height / scale);
			glBindFramebuffer(GL_DRAW_FRAMEBUFFER, fb);
			glClear(GL_COLOR_BUFFER_BIT);
			glUseProgram(prog::uv_tracer);
			glUniform2f(0, width / size, height / size);
			glUniform1f(1, dt);
			glDrawArrays(GL_POINTS, 0, 1);

			glEnable(GL_MULTISAMPLE);
			glViewport(0, 0, width, height);
			glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
			glClear(GL_COLOR_BUFFER_BIT);
			glUseProgram(prog::uv_quad);
			glBindTextureUnit(0, tex::grid);
			glBindTextureUnit(1, uvmap);
			glDrawArrays(GL_POINTS, 0, 1);
		} else {
			glDisable(GL_MULTISAMPLE);
			glUseProgram(prog::tracer);
			glBindTextureUnit(0, tex::grid);
			glUniform2f(0, width / size, height / size);
			glUniform1f(1, dt);
			glDrawArrays(GL_POINTS, 0, 1);
		}

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
