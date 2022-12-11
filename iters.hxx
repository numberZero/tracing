#pragma once
#include <asyncpp/generator.h>
#include <glm/vec2.hpp>

template <typename T>
asyncpp::generator<T> irange(T start, T stop, T step = T{1});

template <>
asyncpp::generator<int> irange<int>(int start, int stop, int step) {
	for (int x = start; x < stop; x += step)
		co_yield {x};
}

template <>
asyncpp::generator<glm::ivec2> irange<glm::ivec2>(glm::ivec2 start, glm::ivec2 stop, glm::ivec2 step) {
	for (int y = start.y; y < stop.y; y += step.y)
	for (int x = start.x; x < stop.x; x += step.x)
		co_yield {x, y};
}

template <typename T>
asyncpp::generator<T> irange(T stop) {
	return irange<T>({}, stop);
}

template <typename C>
auto enumerate(C &&container) -> asyncpp::generator<std::pair<int, decltype(*std::begin(container))>> {
	int index = 0;
	for (auto &&item: container)
		co_yield {index++, item};
}
