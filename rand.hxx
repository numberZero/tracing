#pragma once
#include <random>
#include <glm/glm.hpp>
#include <pcg_random.hpp>

inline thread_local pcg32 rgen;

using glm::vec2, glm::vec3;

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
