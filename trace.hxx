#pragma once
#include "material/base.hxx"
#include "shape/base.hxx"

class Surface {
public:
	Shape const &shape;
	Material const &material;
};

struct Hit {
	Surface const *surface;
	vec3 pos;
	vec3 normal;
};

Hit trace(Ray const &ray);
