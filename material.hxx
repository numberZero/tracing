#pragma once
#include <optional>
#include <random>
#include <glm/glm.hpp>
#include <pcg_random.hpp>

using glm::vec2, glm::vec3;

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

class Material {
public:
	virtual void hit(Light &light, vec3 &dir, vec3 normal) const noexcept = 0;

	vec3 color;
	vec3 emission;
	float softness;
};

static thread_local pcg32 rgen;

inline static double randd() {
	static thread_local std::uniform_real_distribution<> dist;
	return dist(rgen);
}

inline static vec2 rand_disc() {
	for (;;) {
		vec2 ret = {randd(), randd()};
		ret = 2.0f * ret - 1.0f;
		float len = length(ret);
		if (len >= 1.0f)
			continue;
		return ret;
	}
}

inline static vec3 rand_spherical() {
	static constexpr float eps = 1e-3;
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

class Glossy: public Material {
public:
	Glossy(float shinness, vec3 refl_color, vec3 back_color): shinness(shinness), varnish(refl_color), back(back_color) {}

	void hit(Light &light, vec3 &dir, vec3 normal) const noexcept override {
		if (randd() < shinness)
			varnish.hit(light, dir, normal);
		else
			back.hit(light, dir, normal);
	}

	float shinness;
	Specular varnish;
	Diffuse back;
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
