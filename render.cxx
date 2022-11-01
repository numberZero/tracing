#include <cmath>
#include <vector>
#include <thread>
#include <glm/glm.hpp>
#include "render.hxx"
#include "rand.hxx"
#include "scene.hxx"
#include "trace.hxx"
#include "material/base.hxx"
#include "shape/base.hxx"

uint8_t image[height][width][3];

using std::vector, std::thread;
using glm::vec2, glm::vec3, glm::ivec2, glm::ivec3;

#if BEST
#define makePixel makePixelFine
#else
#define makePixel makePixelFast
#endif

inline static float to_sRGB(float channel) {
	if (channel <= 0.0031308f)
		return 12.92f * channel;
	return 1.055f * pow(channel, 1.0f/2.4f) - 0.055f;
}

inline static ivec3 to_sRGB(vec3 color) {
	vec3 srgb = {
		to_sRGB(color.x),
		to_sRGB(color.y),
		to_sRGB(color.z),
	};
	return glm::clamp(ivec3(256.0f * srgb), 0, 255);
}

inline static vec3 compress_color(vec3 color) {
	float Y = dot({0.2126, 0.7152, 0.0722}, color);
	float Yc = atan(Y) / M_PI_2;
	return color * (Yc / Y);
}

static ivec3 makePixelFine(ivec2 pos) {
	vec3 color = {};
	int n = 0;
	for (int k = 0; k < rays; k++) {
		vec2 focal = (focal_distance / depth) * vec2{pos.x + randd() - width/2.f, height/2.f - (pos.y + randd())};
		vec2 lens = lens_rad * rand_disc();
		vec3 dir = normalize(vec3{focal, -focal_distance} - vec3{lens, 0.f});
		Ray ray{camera_pos + vec3{lens, 0.f}, dir};
		Light light;
		for (int k = 0; k < max_reflections; k++) {
			auto hit = trace(ray);
			if (!hit.surface) {
				light += background(ray.dir);
				color += vec3(light);
				n++;
				break;
			}
			hit.surface->material.hit(light, ray.dir, hit.normal);
			ray.pos = hit.pos;
		}
	}
	return to_sRGB(compress_color(brightness / n * color));
}

static vec3 renderPixelFast(vec3 dir) {
	Ray ray1{camera_pos, dir};

	auto hit = trace(ray1);
	if (!hit.surface)
		return background(dir);

	vec3 color = {};
	int n = 0;
	for (int k = 0; k < rays; k++) {
		Ray ray{hit.pos, dir};
		Light light;
		hit.surface->material.hit(light, ray.dir, hit.normal);

		for (int k = 1; k < max_reflections; k++) {
			auto hit = trace(ray);
			if (!hit.surface) {
				light += background(ray.dir);
				color += vec3(light);
				n++;
				break;
			}
			hit.surface->material.hit(light, ray.dir, hit.normal);
			ray.pos = hit.pos;
			if (all(lessThan(light.filter, vec3{1e-2f}))) {
				color += vec3(light);
				n++;
				break;
			}
		}
	}
	return color / float(n);
}

static ivec3 makePixelFast(ivec2 pos) {
	vec3 dir = {pos.x - width/2.f + .5f, height/2.f - pos.y - .5f, -depth};
	return to_sRGB(compress_color(brightness * renderPixelFast(normalize(dir))));
}

static void render_line(int j) {
	for (int i = 0; i < width; i++) {
		ivec3 pixel = makePixel({i, j});
		for (int k = 0; k < 3; k++)
			image[j][i][k] = pixel[k];
	}
}

static void render_band(int j1, int j2) {
	rgen.seed(0x12345678, (j1 << 16) ^ j2);
	printf("Me renders %d-%d only\n", j1, j2);
	for (int j = j1; j < j2; j++)
		render_line(j);
	printf("Me rendered %d-%d\n", j1, j2);
}

static void render_lines(int start, int step) {
	rgen.seed(0x12345678, (start << 16) ^ step);
	printf("Me renders every %d from %d only\n", step, start);
	for (int j = start; j < height; j += step)
		render_line(j);
	printf("Me rendered every %d from %d only\n", step, start);
}

#if THREADS
static vector<thread> threads_bands(int n) {
	vector<thread> threads;
	vector<int> splits;
	for (int k = 0; k <= n; k++)
		splits.push_back(k * height / n);
	for (int k = 0; k < n; k++)
		threads.emplace_back(render_band, splits[k], splits[k + 1]);
	return threads;
}

static vector<thread> threads_interlaced(int n) {
	vector<thread> threads;
	for (int k = 0; k < n; k++)
		threads.emplace_back(render_lines, k, n);
	return threads;
}

void render() {
	int n = thread::hardware_concurrency();
#if BEST
	n /= 2; // HACK to not overheat the CPU
#endif
	if (n <= 0)
		n = 1;
	auto threads = threads_interlaced(n);
	for (thread &th: threads)
		th.join();
}
#else
void render() {
	render_band(0, height);
}
#endif
