#pragma once
#include <asyncpp/generator.h>
#include <glm/vec2.hpp>

asyncpp::generator<glm::ivec2> irange(glm::ivec2 start, glm::ivec2 stop, glm::ivec2 step = {1, 1}) {
	for (int y = start.y; y < stop.y; y += step.y)
	for (int x = start.x; x < stop.x; x += step.x)
		co_yield {x, y};
}

asyncpp::generator<glm::ivec2> irange(glm::ivec2 stop) {
	return irange({}, stop);
}
