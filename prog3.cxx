#include <cstdio>
#include <optional>
#include <vector>
#if THREADS
#include <thread>
#endif
#include <glm/glm.hpp>
#include <random>
#include "shape.hxx"
#include "material.hxx"

using namespace std;
using namespace glm;

class Surface {
public:
	Shape const &shape;
	Material const &material;
};

static constexpr float basement_radius = 1000.0f;
static constexpr int additional_spheres = 300;

std::vector<Surface> surfaces = {
	{*new Sphere{{-3.0, 4.0,  3.0}, 2}, *new Metallic({0.5, 0.5, 0.6}, 0.1)},
	{*new Sphere{{-1.0, 2.5, -1.0}, 2}, *new Metallic({0.7, 0.1, 0.1}, 0.5)},
	{*new Sphere{{ 2.5, 3.5,  1.0}, 2}, *new Diffuse({0.3, 0.2, 0.7})},
	{*new Sphere{{ 7.0, 5.0,  12.0}, 2}, *new Shiny({3.7, 1.7, 1.7})},
	{*new Conicoid({-30.0, 3.0, -30.0},  {-30.0, basement_radius+3.0, -30.0}, 4.0, -0.3), *new Metallic({0.5, 0.5, 0.8}, 0.3)},
	{*new Conicoid({9.0, 3.0, 0.0}, {3, basement_radius+1, 0}, 1.5, 0.0), *new Metallic({0.5, 0.8, 0.6}, 0.3)},
	{*new Sphere{{0.0, -basement_radius, 0.0}, basement_radius}, *new Diffuse({0.65, 0.65, 0.65})},
};

void prepare() {
	rgen.seed(0x12345678);
	std::normal_distribution<float> ndist{0.0f, 10.0f};
	for (int k = 0; k < additional_spheres; k++) {
		float u = randd();
		float v = randd();
		float radius = 0.1f + 0.2f * u + 0.6f * pow(v, 10);
		vec2 pos = {ndist(rgen), ndist(rgen)};
		vec3 pos3 = (basement_radius + radius) * normalize(vec3{pos.x, basement_radius, pos.y}) - vec3{0, basement_radius, 0};
		Shape *shape = new Sphere{pos3, radius};

		float softness = randd();
		float hue = 2 * M_PI * randd();
		vec3 pure = .5f + .5f * vec3{cos(hue), cos(hue + 2/3. * M_PI), cos(hue + 4/3. * M_PI)};
		float sat = randd();
		vec3 color = mix(vec3(1), pure, sat);
		float darkness = .8f * randd();
		float brightness = 1.f - darkness;
		Material *material;
		if (softness > .5f)
			material = new Diffuse(brightness * color);
		else
			material = new Metallic(brightness * color, softness);

		surfaces.push_back({*shape, *material});
	}
}

static vec3 const sun_dir = normalize(vec3{100.0f, 200.0f, 100.0f});
static float const sun_size = 0.02f;
static vec3 const sun_color = {20.0f, 16.0f, 8.0f};

vec3 background(vec3 dir) {
// 	return{};
// 	if (dir.y < -.1)
// 		return {0.1, 0.5, 0.0};
	vec3 sky_color = mix(vec3{0.6, 0.8, 0.9}, vec3{0.6, 0.7, 1.2}, dir.y);
// 	sky_color += .3f * mix(vec3{.5, .6, .8}, vec3{.7, .9, 1.}, dot(dir, sun_dir));
	float sun_d = (1.f - dot(dir, sun_dir)) / sun_size;
	sky_color += sun_color * glm::clamp(1.f - sun_d, 0.f, 1.f);
	return sky_color;
}

struct Hit {
	Surface const *surface;
	vec3 pos;
	vec3 normal;
};

Hit trace(Ray const &ray) {
	Hit hit = {};
	float d = 1.0 / 0.0;
	for (Surface const &surf: surfaces) {
		auto h = surf.shape.hit(ray);
		if (!h)
			continue;
		if (h->dist > d)
			continue;
		d = h->dist;
		hit = {
			&surf,
			h->pos,
			h->normal,
		};
	}
	return hit;
}

float to_sRGB(float channel) {
	if (channel <= 0.0031308f)
		return 12.92f * channel;
	return 1.055f * pow(channel, 1.0f/2.4f) - 0.055f;
}

ivec3 to_sRGB(vec3 color) {
	vec3 srgb = {
		to_sRGB(color.x),
		to_sRGB(color.y),
		to_sRGB(color.z),
	};
	return glm::clamp(ivec3(256.0f * srgb), 0, 255);
}

vec3 compress_color(vec3 color) {
	float Y = dot({0.2126, 0.7152, 0.0722}, color);
	float Yc = atan(Y) / M_PI_2;
	return color * (Yc / Y);
}

static constexpr float brightness = 2.0f;
#if BEST
#define makePixel makePixelFine
static constexpr int width = 1920;
static constexpr int height = 1080;
static constexpr int depth = height;
static constexpr int rays = 256;
#else
#define makePixel makePixelFast
static constexpr int width = 800;
static constexpr int height = 450;
static constexpr int depth = height;
static constexpr int rays = 16;
#endif
static constexpr int max_reflections = 16;

static vec3 const camera_pos = {0.0, 4.0, 15.0};
static float const aperture = 0.1f;
static float const lens_rad = aperture / 2.f;
static float const focal_distance = 15.0f;

ivec3 makePixelFine(ivec2 pos) {
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

vec3 renderPixelFast(vec3 dir) {
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

ivec3 makePixelFast(ivec2 pos) {
	vec3 dir = {pos.x - width/2.f + .5f, height/2.f - pos.y - .5f, -depth};
	return to_sRGB(compress_color(brightness * renderPixelFast(normalize(dir))));
}

static uint8_t image[height][width][3];

void render_line(int j) {
	for (int i = 0; i < width; i++) {
		ivec3 pixel = makePixel({i, j});
		for (int k = 0; k < 3; k++)
			image[j][i][k] = pixel[k];
	}
}

void render_band(int j1, int j2) {
	rgen.seed(0x12345678, (j1 << 16) ^ j2);
	printf("Me renders %d-%d only\n", j1, j2);
	for (int j = j1; j < j2; j++)
		render_line(j);
	printf("Me rendered %d-%d\n", j1, j2);
}

void render_lines(int start, int step) {
	rgen.seed(0x12345678, (start << 16) ^ step);
	printf("Me renders every %d from %d only\n", step, start);
	for (int j = start; j < height; j += step)
		render_line(j);
	printf("Me rendered every %d from %d only\n", step, start);
}

#if THREADS
vector<thread> threads_bands(int n) {
	vector<thread> threads;
	vector<int> splits;
	for (int k = 0; k <= n; k++)
		splits.push_back(k * height / n);
	for (int k = 0; k < n; k++)
		threads.emplace_back(render_band, splits[k], splits[k + 1]);
	return threads;
}

vector<thread> threads_interlaced(int n) {
	vector<thread> threads;
	for (int k = 0; k < n; k++)
		threads.emplace_back(render_lines, k, n);
	return threads;
}

void render() {
	auto threads = threads_interlaced(thread::hardware_concurrency());
	for (thread &th: threads)
		th.join();
}
#else
void render() {
	render_band(0, height);
}
#endif

void save() {
	FILE *ppm = fopen("out.ppm", "wb");
	fprintf(ppm, "P6\n%d %d\n255\n", width, height);
	fwrite(image, sizeof(image), 1, ppm);
	fclose(ppm);
}

int main() {
	prepare();
	printf("Prepared %zu surfaces\n", surfaces.size());
	render();
	printf("Rendered %d lines\n", height);
	save();
	printf("Saved\n");
}
