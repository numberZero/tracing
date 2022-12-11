#include <cmath>
#include <cstdio>
#include <random>
#include <thread>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <SDL2/SDL_surface.h>
#include <SDL2/SDL_image.h>
#include "iters.hxx"

using namespace glm;

using Pixel = glm::tvec4<std::uint8_t>;
using Dirmaker = vec3(vec2);

const int halfsize = 1024;
const int subsamples = 16;
// const int halfsize = 128;
// const int subsamples = 4;

static thread_local std::ranlux24 gen;
static thread_local std::uniform_real_distribution<float> dSample{0.0f, 1.0f};

int main(int argc, char **argv) {
	if (argc != 3) {
		printf("Usage: %s [input filename] [output basename]\n", argv[0]);
		exit(1);
	}
	char const *input_name = argv[1];
	char const *output_base = argv[2];
	int output_namelen = strlen(output_base) + 6;

	SDL_Init(0);
	IMG_Init(IMG_INIT_PNG | IMG_INIT_JPG);

	SDL_Surface *in = IMG_Load(input_name);
	if (in->format->format != SDL_PIXELFORMAT_RGBA32) {
		SDL_Surface *copy = SDL_ConvertSurfaceFormat(in, SDL_PIXELFORMAT_RGBA32, 0);
		SDL_FreeSurface(in);
		in = copy;
	}

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

	int const N = std::thread::hardware_concurrency();
	static Pixel pixels[2 * halfsize][2 * halfsize];
	auto render_side = [&] (auto dirmaker) -> SDL_Surface * {
		SDL_Surface *out = SDL_CreateRGBSurfaceWithFormatFrom(pixels, 2 * halfsize, 2 * halfsize, 32, 2 * sizeof(std::uint32_t) * halfsize, SDL_PIXELFORMAT_RGBA32);
		auto render_lines = [&] (int k) {
			for (ivec2 pt: irange({0, k}, ivec2(2 * halfsize), {1, N})) {
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
		};
		std::vector<std::thread> threads(N);
		for (auto &&[k, thread]: enumerate(threads))
			thread = std::thread(render_lines, k);
		for (auto &&thread: threads)
			thread.join();
		return out;
	};

	auto make_side = [&] (int side, auto dirmaker) {
		char output_name[output_namelen];
		snprintf(output_name, output_namelen, "%s%d.png", output_base, side);
		fprintf(stderr, "Generating side %d\n", side);
		SDL_Surface *out = render_side(dirmaker);
		fprintf(stderr, "Saving to %s\n", output_name);
		IMG_SavePNG(out, output_name);
		SDL_FreeSurface(out);
	};

	make_side(0, [] (vec2 uv) -> vec3 { return {1, -uv.y, -uv.x}; });
	make_side(1, [] (vec2 uv) -> vec3 { return {-1, -uv.y, uv.x}; });
	make_side(2, [] (vec2 uv) -> vec3 { return {uv.x, 1, uv.y}; });
	make_side(3, [] (vec2 uv) -> vec3 { return {uv.x, -1, -uv.y}; });
	make_side(4, [] (vec2 uv) -> vec3 { return {uv.x, -uv.y, 1}; });
	make_side(5, [] (vec2 uv) -> vec3 { return {-uv.x, -uv.y, -1}; });

	return 0;
}
