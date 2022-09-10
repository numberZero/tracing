#pragma once
#include <vector>
#include <glm/glm.hpp>
#include "trace.hxx"

extern std::vector<Surface> surfaces;

void prepare();
glm::vec3 background(glm::vec3 dir);
