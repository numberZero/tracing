#pragma once
#include "math.hxx"
#include "ray.hxx"

class SpaceDesc {
public:
	virtual Ray toGlobal(Ray ray) const = 0;
	virtual Ray fromGlobal(Ray ray) const = 0;
};
