#pragma once
#include <glm/glm.hpp>
#include <optional>

using std::optional, std::nullopt;
using glm::vec3, glm::vec4, glm::mat3, glm::mat4;

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
		if (tca < 0) return nullopt;
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

class Quadric: public Shape {
public:
	Quadric(mat4 m) : m(0.5f * (m + glm::transpose(m))) {}

	optional<Hit> hit(Ray const &ray) const noexcept override {
		vec4 va{ray.dir, 0.0f};
		vec4 vb{ray.pos, 1.0f};
		float a = glm::dot(va, m * va);
		float b = glm::dot(va, m * vb) / a;
		float c = glm::dot(vb, m * vb) / a;

		float D = b*b - c;
		if (D < 0.0f)
			return nullopt;
		float d = std::sqrt(D);
		float t0 = -b-d;
		float t1 = -b+d;
		if (t0 > eps) return make(ray, t0);
		if (t1 > eps) return make(ray, t1);
		return nullopt;
	}

	mat4 m;

private:
	Hit make(Ray const &ray, float t) const noexcept {
		Hit hit;
		hit.dist = t;
		hit.pos = ray.pos + t * ray.dir;
		hit.normal = glm::normalize(vec3(m * vec4(hit.pos, 1.0f)));
		return hit;
	}
};

class Triangle: public Shape {
public:
	Triangle(vec3 a, vec3 b, vec3 c)
		: hm(inverse(mat4{{a, 1}, {b, 1}, {c, 1}, {0, 0, 0, -1}}))
		, n(normalize(cross(c-a, b-a)))
	{}

	optional<Hit> hit(Ray const &ray) const noexcept override {
		float t = (1 - (hm * ray.pos).w) / (hm * ray.dir).w;
		if (t <= 0)
			return nullopt;
		Hit hit;
		hit.dist = t;
		hit.pos = ray.pos + t * ray.dir;
		hit.normal = n;
		vec4 h = hm * hit.pos;
		if (h.w < 0)
			h = -h;
		if (!all(greaterThanEqual(vec3(h), vec3(0))))
			return nullopt;
		return {hit};
	}

	glm::mat3x4 hm;
	vec3 n;
};
