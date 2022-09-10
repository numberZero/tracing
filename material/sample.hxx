#pragma once
#include <glm/glm.hpp>
#include "base.hxx"
#include "rand.hxx"

using glm::vec2, glm::vec3;

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
