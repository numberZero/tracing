#pragma once
#include <GL/gl.h>
#include "math.hxx"
#include "visual.hxx"

void ellipse(SpaceVisual const *visual, vec3 center, vec3 u, vec3 v) {
	static constexpr int M = 30;
	glBegin(GL_LINE_LOOP);
	for (int k = -M; k < M; k++) {
		float phi = (.5 + k) * (M_PI / M);
		glVertex3fv(value_ptr(visual->where(center + cos(phi) * u + sin(phi) * v)));
	}
	glEnd();
}

void circleX(SpaceVisual const *visual, vec3 center, float radius) {
	ellipse(visual, center, {0, radius, 0}, {0, 0, radius});
}

void circleY(SpaceVisual const *visual, vec3 center, float radius) {
	ellipse(visual, center, {0, 0, radius}, {radius, 0, 0});
}

void circleZ(SpaceVisual const *visual, vec3 center, float radius) {
	ellipse(visual, center, {radius, 0, 0}, {0, radius, 0});
}

void sphere(SpaceVisual const *visual, vec3 center, float radius) {
	circleX(visual, center, radius);
	circleY(visual, center, radius);
	circleZ(visual, center, radius);
}

void ellipsoid(SpaceVisual const *visual, vec3 center, mat3 transform) {
	ellipse(visual, center, transform * vec3{0, 1, 0}, transform * vec3{0, 0, 1});
	ellipse(visual, center, transform * vec3{0, 0, 1}, transform * vec3{1, 0, 0});
	ellipse(visual, center, transform * vec3{1, 0, 0}, transform * vec3{0, 1, 0});
}
