#include <cstdio>
#include <optional>
#if THREADS
#include <thread>
#include <vector>
#endif
#include <glm/glm.hpp>

using namespace std;
using namespace glm;

static constexpr float eps = 1e-3;

struct Ray {
	vec3 pos;
	vec3 dir;
};

class Shape {
public:
	struct Hit {
		vec3 pos;
		float dist;
		vec3 normal;
	};

	virtual optional<Hit> hit(Ray const &ray) const noexcept = 0;
};

class Sphere: public Shape {
public:
	Sphere(vec3 center, float radius) : center(center), radius(radius) {}

	optional<Hit> hit(Ray const &ray) const noexcept override {
		vec3 L = center - ray.pos;
		float tca = dot(L, ray.dir);
		float d2 = dot(L, L) - tca*tca;
		if (d2 > radius * radius) return nullopt;
		float thc = std::sqrt(radius * radius - d2);
		float t0 = tca-thc, t1 = tca+thc;
		if (t0 > eps) return make(ray, t0);
		if (t1 > eps) return make(ray, t1);
		return nullopt;
	}

	vec3 center;
	float radius;

private:
	Hit make(Ray const &ray, float t) const noexcept {
		Hit hit;
		hit.dist = t;
		hit.pos = ray.pos + t * ray.dir;
		hit.normal = normalize(hit.pos - center);
		return hit;
	}
};

class Plane: public Shape {
public:
	Plane(vec3 origin, vec3 normal) : eq(normal, -dot(origin, normal)) {}

	optional<Hit> hit(Ray const &ray) const noexcept override {
		float t = -dot(eq, {ray.pos, 1}) / dot(vec3(eq), ray.dir);
		if (t > eps)
			return make(ray, t);
		return nullopt;
	}

	vec4 eq;

private:
	Hit make(Ray const &ray, float t) const noexcept {
		Hit hit;
		hit.dist = t;
		hit.pos = ray.pos + t * ray.dir;
		hit.normal = eq;
		return hit;
	}
};

struct Light {
	vec3 light = {0,0,0};
	vec3 filter = {1,1,1};

	operator vec3() const noexcept {
		return light;
	}

	Light &operator*= (vec3 f) noexcept {
		filter *= f;
		return *this;
	}

	Light &operator+= (vec3 l) noexcept {
		light += filter * l;
		return *this;
	}
};

static thread_local uint16_t randstate[3];

int32_t rand31() {
	return nrand48(randstate);
}

double randd() {
	return erand48(randstate);
}

vec3 rand_spherical() {
	for (;;) {
		vec3 ret = {randd(), randd(), randd()};
		ret = 2.0f * ret - 1.0f;
		float len = length(ret);
		if (len >= 1.0f)
			continue;
		if (len < eps)
			continue;
		return ret / len;
	}
}

class Material {
public:
	virtual void hit(Light &light, vec3 &dir, vec3 normal) const noexcept = 0;

	vec3 color;
	vec3 emission;
	float softness;
};

class Metallic: public Material {
public:
	void hit(Light &light, vec3 &dir, vec3 normal) const noexcept override {
		light *= color;
		dir = reflect(dir, normal) + softness * rand_spherical();
		if (dot(dir, normal) <= 0.0f)
			light *= {0,0,0};
	}

	Metallic(vec3 color, float softness = 0.2f) : color(color), softness(softness) {}

	vec3 color;
	float softness;
};

class Specular: public Material {
public:
	Specular(vec3 color): color(color) {}

	void hit(Light &light, vec3 &dir, vec3 normal) const noexcept override {
		light *= color;
		dir = reflect(dir, normal);
	}

	vec3 color;
};

class Diffuse: public Material {
public:
	Diffuse(vec3 color): color(color) {}

	void hit(Light &light, vec3 &dir, vec3 normal) const noexcept override {
		light *= color;
		dir = normalize(normal + rand_spherical());
	}

	vec3 color;
};

class Shiny: public Material {
public:
	Shiny(vec3 color): color(color) {}

	void hit(Light &light, vec3 &dir, vec3 normal) const noexcept override {
		light += color;
		light *= {0,0,0};
	}

	vec3 color;
};

class Custom: public Material {
public:
	void hit(Light &light, vec3 &dir, vec3 normal) const noexcept override {
		light *= color;
		light += emission;
		dir = reflect(dir, normal) + softness * rand_spherical();
		if (dot(dir, normal) <= 0.0f)
			light *= {0,0,0};
	}

	Custom(vec3 color, float softness = 0.5f, vec3 emission = {}) : color(color), emission(emission), softness(softness) {}

	vec3 color;
	vec3 emission;
	float softness;
};

class Surface {
public:
	Shape const &shape;
	Material const &material;
};

Surface const surfaces[] = {
	{Sphere{{-3.0,  0.0, -12}, 2}, Metallic({0.5, 0.5, 0.6}, 0.1)},
	{Sphere{{-1.0, -1.5, -16}, 2}, Metallic({0.7, 0.1, 0.1}, 0.5)},
	{Sphere{{ 2.5, -0.5, -14}, 2}, Diffuse({0.3, 0.2, 0.7})},
	{Sphere{{ 7.0,  5.0, -18}, 2}, Specular({0.7, 0.7, 0.7})},
	{Sphere{{ 7.0,  5.0, -6}, 2}, Shiny({3.7, 1.7, 1.7})},
	{Plane{{0.0, -4.0, 0.0}, {0.0, 1.0, 0.0}}, Diffuse({0.12, 0.40, 0.02})},
// 	{Sphere{{-3.0,  0.0, -16}, 2}, Custom{{0.3, 0.5, 0.2}}},
// 	{Sphere{{-1.0, -1.5, -12}, 2}, Custom{{0.7, 0.1, 0.1}}},
// 	{Sphere{{ 1.5, -0.5, -18}, 2}, Custom{{0.3, 0.2, 0.7}}},
// 	{Sphere{{ 7.0,  5.0, -18}, 2}, Custom{{1.7, 1.7, 1.7}}},
};

static vec3 const sun_dir = normalize(vec3{100.0f, 200.0f, 100.0f});
static float const sun_size = 0.002f;
static vec3 const sun_color = {20.0f, 18.0f, 12.0f};

vec3 background(vec3 dir) {
// 	return{};
// 	if (dir.y < -.1)
// 		return {0.1, 0.5, 0.0};
	vec3 sky_color = mix(vec3{.6, .7, 1.}, vec3{.9, .9, 1.}, dir.y);
	return sky_color;
// 	float sun_d = (1.f - dot(dir, sun_dir)) / sun_size;
// 	return mix(sun_color, sky_color, glm::clamp(sun_d - 1.f, 0.f, 1.f));
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
static constexpr int width = 800;
static constexpr int height = 600;
static constexpr int depth = height;
static constexpr int rays = 16;
static constexpr int max_reflections = 16;

ivec3 makePixel(ivec2 pos) {
	vec3 color = {};
	int n = 0;
	for (int k = 0; k < rays; k++) {
		vec3 dir = normalize(vec3{pos.x + randd() - width/2.f, height/2.f - (pos.y + randd()), -depth});
		Ray ray{{}, dir};
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

static uint8_t image[height][width][3];

void render_line(int j) {
	for (int i = 0; i < width; i++) {
		ivec3 pixel = makePixel({i, j});
		for (int k = 0; k < 3; k++)
			image[j][i][k] = pixel[k];
	}
}

void render_band(int j1, int j2) {
	randstate[0] = 0x1234 ^ j1;
	randstate[1] = 0x5678 ^ j2;
	randstate[2] = 0x9ABC;
	rand31();
	rand31();
	printf("Me renders %d-%d only\n", j1, j2);
	for (int j = j1; j < j2; j++)
		render_line(j);
	printf("Me rendered %d-%d\n", j1, j2);
}

void render_lines(int start, int step) {
	randstate[0] = 0x1234 ^ start;
	randstate[1] = 0x5678 ^ step;
	randstate[2] = 0x9ABC;
	rand31();
	rand31();
	printf("Me renders every %d from %d only\n", step, start);
	for (int j = start; j < height; j += step)
		render_line(j);
	printf("Me rendered every %d from %d only\n", step, start);
}

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

#if THREADS
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
