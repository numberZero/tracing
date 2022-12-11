#include <cmath>
#include <cstdio>
#include <random>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

int main() {
	static const float delta = 1.0;
	static const int samples = 1024;
	static const float scatter = glm::radians(15.0f);
	std::ranlux24 gen;
	std::normal_distribution<float> dScatter{0.0f, scatter};
	fprintf(stderr, "scatter: %.3f\n", scatter);
	puts("Angle\t\tLightness\t");
	puts("Deg\tRad\tBase\tScatter");
	float ltotbase = 0.0f;
	float ltotscatter = 0.0f;
	for (float angle = 0.0; angle <= 180.0f; angle += delta) {
		float theta = glm::radians(angle);
		float area = 2.0f * M_PIf32 * std::sin(theta) * glm::radians(delta);
		float lbase = std::max(0.0f, std::cos(theta));
		float lscatter = 0.0f;
		glm::mat3 baserot = glm::rotate(glm::mat4(1.0f), theta, {1.0f, 0.0f, 0.0f});
		for (int k = 0; k < samples; k++) {
			glm::vec2 v{dScatter(gen), dScatter(gen)};
			glm::mat3 randrot = glm::rotate(glm::mat4(1.0f), glm::length(v), {v, 0.0f});
			glm::vec3 pt = randrot * baserot * glm::vec3{0.0f, 0.0f, 1.0f};
			float light = std::max(0.0f, pt.z);
			lscatter += light;
		}
		lscatter /= samples;
		printf("%6.1f°\t%6.4f\t%6.4f\t%6.4f\n",
			angle, theta, lbase, lscatter);
		ltotbase += area * lbase;
		ltotscatter += area * lscatter;
	}
	fprintf(stderr, "Total base light: %.6f\n", ltotbase);
	fprintf(stderr, "Total scatter light: %.6f\n", ltotscatter);
}
