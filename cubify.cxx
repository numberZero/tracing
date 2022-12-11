#include <cmath>
#include <cstdio>
#include <random>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <SDL2/SDL_surface.h>
#include <SDL2/SDL_image.h>
#include "iters.hxx"

using namespace glm;

using Dirmaker = vec3(vec2);

Dirmaker *dirmakers[6] = {
	[] (vec2 uv) -> vec3 { return {1, -uv.y, -uv.x}; },
	[] (vec2 uv) -> vec3 { return {-1, -uv.y, uv.x}; },
	[] (vec2 uv) -> vec3 { return {uv.x, 1, uv.y}; },
	[] (vec2 uv) -> vec3 { return {uv.x, -1, -uv.y}; },
	[] (vec2 uv) -> vec3 { return {uv.x, -uv.y, 1}; },
	[] (vec2 uv) -> vec3 { return {-uv.x, -uv.y, -1}; },
};

int main(int argc, char **argv) {
	if (argc != 3) {
		printf("Usage: %s [input filename] [output basename]\n", argv[0]);
		exit(1);
	}
	char const *input_name = argv[1];
	char const *output_base = argv[2];
	int output_namelen = strlen(output_base) + 6;

// 	const int halfsize = 1024;
// 	const int subsamples = 16;
	const int halfsize = 128;
	const int subsamples = 4;

	SDL_Init(0);
	IMG_Init(IMG_INIT_PNG | IMG_INIT_JPG);

	SDL_Surface *in = IMG_Load(input_name);
	if (in->format->format != SDL_PIXELFORMAT_RGBA32) {
		SDL_Surface *copy = SDL_ConvertSurfaceFormat(in, SDL_PIXELFORMAT_RGBA32, 0);
		SDL_FreeSurface(in);
		in = copy;
	}

	using Pixel = glm::tvec4<std::uint8_t>;
	const Pixel *const inpixels = reinterpret_cast<Pixel *>(in->pixels);
	const int instride = in->pitch / sizeof(*inpixels);
	const ivec2 insize = {in->w, in->h};

	auto get = [&] (ivec2 c) -> vec4 {
		c.x %= insize.x;
		if (c.x < 0)
			c.x += insize.x;
		c.y = clamp(c.y, 0, insize.y - 1);
		return vec4(inpixels[c.x + instride * c.y]) / 256.0f;
	};
	auto interp = [&] (vec2 c) -> vec4 {
		vec2 base = floor(c);
		vec2 b = c - base;
		vec2 a = 1.0f - b;
		vec4 aa = a.x * a.y * get(ivec2(base));
		vec4 ba = b.x * a.y * get(ivec2(base) + ivec2(1, 0));
		vec4 ab = a.x * b.y * get(ivec2(base) + ivec2(0, 1));
		vec4 bb = b.x * b.y * get(ivec2(base) + ivec2(1, 1));
		return aa + ba + ab + bb;
	};

	std::ranlux24 gen;
	std::uniform_real_distribution<float> dSample{0.0f, 1.0f};
	for (auto [side, dirmaker]: enumerate(dirmakers)) {
		static glm::tvec4<std::uint8_t> pixels[2 * halfsize][2 * halfsize];
		SDL_Surface *out = SDL_CreateRGBSurfaceWithFormatFrom(pixels, 2 * halfsize, 2 * halfsize, 32, 2 * sizeof(std::uint32_t) * halfsize, SDL_PIXELFORMAT_RGBA32);
		char output_name[output_namelen];
		snprintf(output_name, output_namelen, "%s%d.png", output_base, side);
		printf("Generating %s\n", output_name);
		for (ivec2 pt: irange(ivec2(2 * halfsize))) {
			vec3 color{0};
			for (int _: irange(subsamples)) {
				vec2 off{dSample(gen), dSample(gen)};
				vec2 sample = (vec2(pt - halfsize) + off) / float(halfsize);
				vec3 dir = dirmaker(sample);
				float phi = atan2f(dir.y, dir.x);
				float theta = atan2f(dir.z, length(vec2(dir.x, dir.y)));
				vec2 uv{
					0.5f + 0.5f * phi / M_PIf32,
					0.5f + theta / M_PIf32,
				};
				color += vec3(interp(uv * vec2(insize)));
			}
			uvec4 pdata = clamp(ivec4(256.0f / subsamples * color, 255), 0, 255);
			pixels[pt.y][pt.x] = pdata;
		}
		IMG_SavePNG(out, output_name);
		SDL_FreeSurface(out);
	}
}
