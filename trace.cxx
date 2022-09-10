#include "trace.hxx"
#include "scene.hxx"

Hit trace(Ray const &ray) {
	Hit hit = {};
	float d = 1.0 / 0.0;
	for (Surface const &surf: surfaces) {
		auto h = surf.shape.hit(ray);
		if (!h)
			continue;
		if (h->dist > d)
			continue;
		d = h->dist;
		hit = {
			&surf,
			h->pos,
			h->normal,
		};
	}
	return hit;
}
