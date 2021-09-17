#pragma once
#include <stdexcept>
#include <utility>
#include <vector>
#include <GL/gl.h>

using bytearray = std::vector<unsigned char>;
class gl_exception: public std::runtime_error { using runtime_error::runtime_error; };

unsigned compile_shader(GLenum type, bytearray const &source);
extern "C" unsigned compile_shader(GLenum type, void const *source, std::size_t size);
unsigned compile_shader(GLenum type, bytearray const &source, int version, std::vector<std::pair<std::string, std::string>> const &defines);
unsigned link_program(std::vector<unsigned> shaders, bool delete_shaders = true);
