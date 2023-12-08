#pragma once
#include <cstdint>
#include <bit>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <pcg_random.hpp>

std::uint64_t rseeder();
inline thread_local pcg32 rgen{rseeder()};

using glm::vec2, glm::vec3;

inline static float rand_canonical() {
	std::uint64_t rand_int = rgen();
	std::uint64_t one_repr = 0x3ff0000000000000ul;
	std::uint64_t val_repr = one_repr | (rand_int << 20) | (1ull << 19);
	double val = std::bit_cast<double>(val_repr);
	return val - 1.0;
}

inline static glm::vec2 rand_disc() {
	for (;;) {
		glm::vec2 ret = {rand_canonical(), rand_canonical()};
		ret = 2.0f * ret - 1.0f;
		float len = length(ret);
		if (len >= 1.0f)
			continue;
		return ret;
	}
}

inline static glm::vec3 rand_ball() {
	for (;;) {
		glm::vec3 ret = {rand_canonical(), rand_canonical(), rand_canonical()};
		ret = 2.0f * ret - 1.0f;
		float len = length(ret);
		if (len >= 1.0f)
			continue;
		return ret;
	}
}

inline static glm::vec3 rand_spherical() {
	static constexpr float eps = 1e-3;
	for (;;) {
		glm::vec3 ret = {rand_canonical(), rand_canonical(), rand_canonical()};
		ret = 2.0f * ret - 1.0f;
		float len = length(ret);
		if (len >= 1.0f)
			continue;
		if (len < eps)
			continue;
		return ret / len;
	}
}
