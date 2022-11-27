#pragma once
#include <cmath>
#include <GL/gl.h>
#include <png++/png.hpp>

using TextureID = GLuint;

static TextureID load_texture(std::string const &filename, GLenum format = GL_RGBA8) {
	TextureID texture;
	png::image<png::rgba_pixel, png::solid_pixel_buffer<png::rgba_pixel>> image(filename);
	glCreateTextures(GL_TEXTURE_2D, 1, &texture);
	auto &&data = image.get_pixbuf().get_bytes();
	glTextureStorage2D(texture, int(std::log2(image.get_width()) + 1.5), format, image.get_width(), image.get_height());
	glTextureSubImage2D(texture, 0, 0, 0, image.get_width(), image.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, data.data());
	glGenerateTextureMipmap(texture);
	return texture;
}

static TextureID load_cube_texture(std::string const &basename, GLenum format = GL_RGBA8) {
	TextureID texture;
	glCreateTextures(GL_TEXTURE_CUBE_MAP, 1, &texture);
	unsigned size;
	for (int k = 0; k < 6; k++) {
		png::image<png::rgba_pixel, png::solid_pixel_buffer<png::rgba_pixel>> image(basename + std::to_string(k) + ".png");
		auto &&data = image.get_pixbuf().get_bytes();
		if (k == 0) {
			size = image.get_width();
			glTextureStorage2D(texture, int(std::log2(size) + 1.5), format, size, size);
		}
		if (image.get_width() != size || image.get_height() != size) {
			glDeleteTextures(1, &texture);
			throw "Invalid cubemap data";
		}
		glTextureSubImage3D(texture, 0, 0, 0, k, image.get_width(), image.get_height(), 1, GL_RGBA, GL_UNSIGNED_BYTE, data.data());
	}
	glGenerateTextureMipmap(texture);
	return texture;
}
