#include <cstdio>
#include "scene.hxx"
#include "render.hxx"

using namespace std;
using namespace glm;

void save() {
	FILE *ppm = fopen("out.ppm", "wb");
	fprintf(ppm, "P6\n%d %d\n255\n", width, height);
	fwrite(image, sizeof(image), 1, ppm);
	fclose(ppm);
}

int main() {
	prepare();
	printf("Prepared %zu surfaces\n", surfaces.size());
	render();
	printf("Rendered %d lines\n", height);
	save();
	printf("Saved\n");
}
