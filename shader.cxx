#include "shader.hxx"
#include <cassert>
#include <sstream>
#include <fmt/printf.h>
#include <GL/glext.h>

unsigned compile_shader(GLenum type, bytearray const &source) {
	return compile_shader(type, source.data(), source.size());
}

template <typename LoadSource>
static unsigned compile_shader(GLenum type, LoadSource load_source) {
	unsigned shader = glCreateShader(type);
	if (!shader)
		throw gl_exception(fmt::sprintf("Can't create shader (of type %#x)", type));
	load_source(shader);
	glCompileShader(shader);
	int compiled = 0;
	glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);
	if (compiled)
		return shader;
	int log_length = 0;
	glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &log_length);
	if (!log_length)
		throw gl_exception(fmt::sprintf("Can't compile shader (of type %#x); no further information available.", type));
	std::vector<char> log(log_length);
	glGetShaderInfoLog(shader, log.size(), &log_length, log.data());
	assert(log.size() == log_length + 1);
	throw gl_exception(fmt::sprintf("Can't compile shader (of type %#x): %s", type, log.data()));
}

unsigned compile_shader(GLenum type, void const *source, std::size_t size) {
	char const *source_data = reinterpret_cast<char const *>(source);
	int source_length = size;
	return compile_shader(type, [&] (int shader) {
		glShaderSource(shader, 1, &source_data, &source_length);
	});
}

unsigned compile_shader(GLenum type, bytearray const &source, int version, std::vector<std::pair<std::string, std::string>> const &defines) {
	std::stringstream header;
	header << "#version " << version << "\n";
	for (auto &&[name, value]: defines)
		header << "#define " << name << " " << value << "\n";
	auto head = header.str();
	char const *source_data[] = { head.data(), reinterpret_cast<char const *>(source.data())};
	int source_length[] = { (int)head.size(), (int)source.size() };
	return compile_shader(type, [&] (int shader) {
		glShaderSource(shader, 2, source_data, source_length);
	});
}

unsigned link_program(std::vector<unsigned> shaders, bool delete_shaders) {
	unsigned program = glCreateProgram();
	if (!program)
		throw gl_exception("Can't create program");
	for (GLenum shader: shaders)
		glAttachShader(program, shader);
	glLinkProgram(program);
	for (GLenum shader: shaders)
		glDetachShader(program, shader);
	int linked = 0;
	glGetProgramiv(program, GL_LINK_STATUS, &linked );
	if (linked) {
		if (delete_shaders)
			for (GLenum shader: shaders)
				glDeleteShader(shader);
		return program;
	}
	int log_length = 0;
	glGetProgramiv(program, GL_INFO_LOG_LENGTH, &log_length);
	if (!log_length)
		throw gl_exception("Can't link program; no further information available.");
	std::vector<char> log(log_length);
	glGetProgramInfoLog(program, log.size(), &log_length, log.data());
	assert(log.size() == log_length + 1);
	throw gl_exception(fmt::sprintf("Can't link program: %s", log.data()));
}
