#include <cstdio>
#include <time.h>
#include "scene.hxx"
#include "render.hxx"

using namespace std;
using namespace glm;

void save() {
	char filename[100] = "render.ppm";
#if BEST
	time_t timestamp = time(nullptr);
	tm split;
	if (localtime_r(&timestamp, &split) != &split)
		assert(false);
	strftime(filename, sizeof(filename), "render-%FT%T%z.ppm", &split);
#endif
	FILE *ppm = fopen(filename, "wb");
	fprintf(ppm, "P6\n%d %d\n255\n", width, height);
	fwrite(image, sizeof(image), 1, ppm);
	fclose(ppm);
}

int main() {
	tzset();
	prepare();
	printf("Prepared %zu surfaces\n", surfaces.size());
	render();
	printf("Rendered %d lines\n", height);
	save();
	printf("Saved\n");
}
