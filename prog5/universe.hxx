#pragma once
#include "math.hxx"
#include "subspace.hxx"
#include "thing.hxx"

class Universe {
public:
	// TODO unique_ptr
	std::vector<ThingySubspace *> thingySpaces;
	std::vector<Thing *> things;

	void updateCaches() {
		for (auto *space: thingySpaces)
			space->things.clear();
		for (auto *thing: things)
			encacheThing(thing);
	}

private:
	static void encacheThing(Thing const *thing) {
		float radius = thing->getRadius();
		thing->loc.space->things.push_back({thing, thing->loc.pos, thing->loc.rot, radius});
		for (auto over: thing->loc.space->boundary->findOverlaps(thing->loc.pos, radius)) {
			if (auto flat = dynamic_cast<ThingySubspace *>(over.into)) {
				flat->things.push_back({thing, over.intoPos, thing->loc.rot * over.jacobi, radius});
			} else {
				throw "Oops! A thing is destroyed by the space curvature";
			}
		}
	}
};
