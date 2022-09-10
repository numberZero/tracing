#pragma once
#include <glm/glm.hpp>

using glm::vec3, glm::ivec2, glm::ivec3;

static constexpr float brightness = 2.0f;
#if BEST
static constexpr int width = 1920;
static constexpr int height = 1080;
static constexpr int depth = height;
static constexpr int rays = 256;
#else
static constexpr int width = 800;
static constexpr int height = 450;
static constexpr int depth = height;
static constexpr int rays = 16;
#endif
static constexpr int max_reflections = 16;

static vec3 const camera_pos = {0.0, 4.0, 15.0};
static float const aperture = 0.1f;
static float const lens_rad = aperture / 2.f;
static float const focal_distance = 15.0f;

extern uint8_t image[height][width][3];

void render();
