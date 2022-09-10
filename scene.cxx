#include "scene.hxx"
#include "shape/sample.hxx"
#include "material/sample.hxx"

static constexpr float basement_radius = 1000.0f;
static constexpr int additional_spheres = 300;

std::vector<Surface> surfaces = {
	{*new Sphere{{-3.0, 4.0,  3.0}, 2}, *new Metallic({0.5, 0.5, 0.6}, 0.1)},
	{*new Sphere{{-1.0, 2.5, -1.0}, 2}, *new Metallic({0.7, 0.1, 0.1}, 0.5)},
	{*new Sphere{{ 2.5, 3.5,  1.0}, 2}, *new Metallic({0.8, 0.6, 0.3}, 0.3)},
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
