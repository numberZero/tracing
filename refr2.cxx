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
	glTextureStorage2D(texture, int(log2(image.get_width()) + 1.5), format, image.get_width(), image.get_height());
	glTextureSubImage2D(texture, 0, 0, 0, image.get_width(), image.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, data.data());
	glGenerateTextureMipmap(texture);
	return texture;
}

void load_textures() {
	tex::grid = load_texture("grid.png");
}

constexpr float outer_radius = 1.00f;
float inner_scale = 1.0f;
constexpr float inner_radius = 0.50f;

float g11(glm::vec3 p) {
	float r = glm::length(glm::vec2{p.y, p.z});
	float l = glm::length(p.x);
	float a = glm::smoothstep(inner_radius, outer_radius, r);
	float b = glm::smoothstep(9.0f, 10.0f, l);
	return glm::mix(inner_scale, 1.0f, a + b - a * b);
}

glm::vec3 dg11(glm::vec3 p) {
	static constexpr float h = 1.0f / (1 << 12);
	float x = (g11({p.x + h, p.y, p.z}) - g11({p.x - h, p.y, p.z})) / (2.0f * h);
	float y = (g11({p.x, p.y + h, p.z}) - g11({p.x, p.y - h, p.z})) / (2.0f * h);
	float z = (g11({p.x, p.y, p.z + h}) - g11({p.x, p.y, p.z - h})) / (2.0f * h);
	return {x, y, z};
}

glm::mat3 make_camera_orientation_matrix(float yaw, float pitch = 0.0, float roll = 0.0) {
	yaw = glm::radians(yaw);
	pitch = glm::radians(pitch);
	roll = glm::radians(roll);
	float ys = std::sin(yaw), yc = std::cos(yaw);
	float ps = std::sin(pitch), pc = std::cos(pitch);
	float rs = std::sin(roll), rc = std::cos(roll);
	glm::mat3 ym = {
		yc, 0.0f, -ys,
		0.0f, 1.0f, 0.0f,
		ys, 0.0f, yc,
	};
	glm::mat3 pm = {
		1.0f, 0.0f, 0.0f,
		0.0f, pc, ps,
		0.0f, -ps, pc,
	};
	glm::mat3 rm = {
		rc, -rs, 0.0f,
		rs, rc, 0.0f,
		0.0f, 0.0f, 1.0f,
	};
	return rm * pm * ym;
}

constexpr float MOUSE_SPEED = 0.2f;
constexpr float MOVE_SPEED = 10.0f;

glm::vec3 position = {0.0, 3.0, -3.0};
glm::vec3 rotation_ypr = {0.0, 0.0, 0.0};
bool indirect = true;
int scale = 4;
float dt = 1.0f / 32.0f;

void on_scroll(GLFWwindow *window, double dx, double dy) {
	char buf[128];
	inner_scale *= pow(2.0, -0.25 * dy);
	sprintf(buf, "dt: 1/%.4g", 1.0 / dt);
	glfwSetWindowTitle(window, buf);
}

void on_cursor_pos(GLFWwindow *window, double x, double y) {
	rotation_ypr.x = -MOUSE_SPEED * x;
	rotation_ypr.y = -MOUSE_SPEED * y;
	rotation_ypr.y = glm::clamp(rotation_ypr.y, -80.0f, 80.0f);
	rotation_ypr.z = glm::clamp(rotation_ypr.z, -60.0f, 60.0f);
	glfwSetCursorPos(window, rotation_ypr.x / -MOUSE_SPEED, rotation_ypr.y / -MOUSE_SPEED);
}

void on_key(GLFWwindow *window, int key, int scancode, int action, int mods) {
	if (key == GLFW_KEY_ESCAPE)
		glfwSetWindowShouldClose(window, GLFW_TRUE);
}

void after_frame(GLFWwindow *window, float dt) {
	auto move_matrix = glm::transpose(make_camera_orientation_matrix(rotation_ypr.x, 0.0f, 0.0f));
	if (glfwGetKey(window, GLFW_KEY_D)) position += dt * MOVE_SPEED * move_matrix[0];
	if (glfwGetKey(window, GLFW_KEY_A)) position -= dt * MOVE_SPEED * move_matrix[0];
	if (glfwGetKey(window, GLFW_KEY_W)) position += dt * MOVE_SPEED * move_matrix[2];
	if (glfwGetKey(window, GLFW_KEY_S)) position -= dt * MOVE_SPEED * move_matrix[2];
	if (glfwGetKey(window, GLFW_KEY_R)) position += dt * MOVE_SPEED * move_matrix[1];
	if (glfwGetKey(window, GLFW_KEY_F)) position -= dt * MOVE_SPEED * move_matrix[1];
	if (glfwGetKey(window, GLFW_KEY_SPACE)) position += dt * MOVE_SPEED * move_matrix[1];
	if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)) position -= dt * MOVE_SPEED * move_matrix[1];
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
	glfwSetInputMode(window, GLFW_RAW_MOUSE_MOTION, GLFW_TRUE);
	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
	glfwSetScrollCallback(window, on_scroll);
	glfwSetCursorPosCallback(window, on_cursor_pos);
	glfwSetKeyCallback(window, on_key);
	glfwShowWindow(window);
	glfwMakeContextCurrent(window);
	glDebugMessageCallback(debug, nullptr);
	load_shaders();
	load_textures();
	float fps = 0.0f;
	double t00 = glfwGetTime();
	double t0 = t00;
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

		glm::mat3 camera_matrix_1 = make_camera_orientation_matrix(rotation_ypr.x, rotation_ypr.y, rotation_ypr.z);

		glm::mat3 scale_matrix{};
		scale_matrix[0].x = width / size;
		scale_matrix[1].y = height / size;
		scale_matrix[2].z = 1.0f;

		glm::mat3 space_scale_matrix{};
		space_scale_matrix[0].x = 1.0f;// * g11(position);
		space_scale_matrix[1].y = 1.0f;
		space_scale_matrix[2].z = 1.0f;

		glm::mat3 camera_matrix = space_scale_matrix * glm::inverse(camera_matrix_1) * scale_matrix;

		if (indirect) {
			glViewport(0, 0, width / scale, height / scale);
			glBindFramebuffer(GL_DRAW_FRAMEBUFFER, fb);
			glClear(GL_COLOR_BUFFER_BIT);
			glUseProgram(prog::uv_tracer);
			glUniformMatrix3fv(0, 1, GL_FALSE, glm::value_ptr(camera_matrix));
			glUniform1f(1, dt);
			glUniform1f(2, 15.0f);
			glUniform1i(3, 30.0f / dt);
			glUniform4f(5, outer_radius, inner_radius, 0.0f, inner_scale);
			glUniform3fv(4, 1, glm::value_ptr(position));
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
			glUniformMatrix3fv(0, 1, GL_FALSE, glm::value_ptr(camera_matrix));
			glUniform1f(1, dt);
			glUniform4f(5, outer_radius, inner_radius, 0.0f, inner_scale);
			glUniform3fv(4, 1, glm::value_ptr(position));
			glDrawArrays(GL_POINTS, 0, 1);
		}
/*
		glUseProgram(0);
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glFrustum(-width / size, width / size, -height / size, height / size, 1.0, 16384.0);
		glScalef(128.0, 128.0, -128.0);
		glMatrixMode(GL_MODELVIEW);
		glLoadMatrixf(glm::value_ptr(glm::mat4(camera_matrix_1)));
		glTranslatef(-position.x, -position.y, -position.z);

		glLineWidth(1.5f);
		glBegin(GL_LINES);
		glColor3f(1, 0, 0);
		glVertex3f(0, 0, 0);
		glVertex3f(10, 0, 0);
		glColor3f(0, 1, 0);
		glVertex3f(0, 0, 0);
		glVertex3f(0, 10, 0);
		glColor3f(0, 0, 1);
		glVertex3f(0, 0, 0);
		glVertex3f(0, 0, 10);
		glEnd();

		glLineWidth(1.0f);
		glColor4f(0.0f, 0.5f, 1.0f, 0.5f);
		for (float x = -1.0; x <= 1.0; x += 1.0f / 16.0f) {
			for (float z = -1.0; z <= 1.0; z += 1.0f / 16.0f) {
				float s = 1.0f / 128.0f;
				glBegin(GL_LINE_LOOP);
				glVertex3f(x, 0.0f, z - s);
				glVertex3f(x - s / g11({x, 0.0f, z}), 0.0f, z);
				glVertex3f(x, 0.0f, z + s);
				glVertex3f(x + s / g11({x, 0.0f, z}), 0.0f, z);
				glEnd();
			}
		}

		glLineWidth(2.0f);
		float y = 0.0f;
		float z = -2.0f;
//		for (float y: {0.0f, 0.5f})
		{
			glColor4f(0.0f, 1.0f, 1.0f, 1.0f);
			glBegin(GL_LINES);
			glVertex3f(-10.0f, 0.0f, -outer_radius);
			glVertex3f(10.0f, 0.0f, -outer_radius);
			glVertex3f(-10.0f, 0.0f, -inner_radius);
			glVertex3f(10.0f, 0.0f, -inner_radius);
			glVertex3f(-10.0f, 0.0f, inner_radius);
			glVertex3f(10.0f, 0.0f, inner_radius);
			glVertex3f(-10.0f, 0.0f, outer_radius);
			glVertex3f(10.0f, 0.0f, outer_radius);
			glEnd();
			float z = 0.0f;
			float x = -11.0f;
// 			for (float x: {-11.0f, 0.0f})
// 			for (float z: {0.0f, 0.2f})
				for (int angle = 80; angle <= 90; angle += 1) {
					const float phi = glm::radians<float>(angle);
					const float a = cos(phi);
					const float s = sin(phi);
					glm::vec3 p0 = {x, y, z};

					glm::vec3 p = p0;
					glm::vec3 d{s / g11(p), 0.0f, a};

					glLineWidth(3.0f);
					glColor4f(1.0f, 1.0f, 0.0f, 1.0f);
					glBegin(GL_LINES);
					glVertex3fv(glm::value_ptr(p));
					while (p.z < 3.0f && glm::length(p) < 20.0f) {
						float c = g11(p);
						glm::vec3 dc = dg11(p);
						glm::vec3 dd;
						dd.x = (0.5f / c) * (dc.x * d.x - dc.y * d.y - dc.z * d.z) * d.x;
						dd.y = 0.5f * dc.y * d.x * d.x;
						dd.z = 0.5f * dc.z * d.x * d.x;
						d += dt * dd;
						float len = glm::length(glm::vec3{d.x * g11(p), d.y, d.z});
						p += dt * d;
						glColor4f(2.0f - len, len, 0.0f, 1.0f);
						glVertex3fv(glm::value_ptr(p));
					}
					glEnd();
				}
		}*/

		glfwSwapBuffers(window);
		n++;
		double t1 = glfwGetTime();
		if (t1 - t00 >= 1.0) {
			fps = n / (t1 - t00);
			t00 = t1;
			n = 0;
		}
		char buf[128];
		sprintf(buf, "FPS: %.0f / (%.1f, %.1f, %.1f)", fps, position.x, position.y, position.z);
		glfwSetWindowTitle(window, buf);
		float dt = t1 - t0;
		t0 = t1;
		after_frame(window, dt);
	}
	glfwDestroyWindow(window);
	return 0;
}
