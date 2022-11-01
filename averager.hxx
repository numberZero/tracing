#pragma once
#include <limits>
#include "math.hxx"

class Averager {
private:
	int sum0;
	float sum1, sum2;
	float minval, maxval;

public:
	Averager() noexcept {
		reset();
	}

	void reset() noexcept {
		sum0 = 0;
		sum1 = 0.0f;
		sum2 = 0.0f;
		minval = std::numeric_limits<float>::max();
		maxval = std::numeric_limits<float>::lowest();
	}

	void sample(float v) noexcept {
		sum0 += 1;
		sum1 += v;
		sum2 += v*v;
		minval = ::min(minval, v);
		maxval = ::max(maxval, v);
	}

	float min() const noexcept {
		return minval;
	}

	float max() const noexcept {
		return maxval;
	}

	float mean() const noexcept {
		return sum1 / sum0;
	}

	float stddev() const noexcept {
		return sqrt(sum2 / sum0 - sqr(sum1 / sum0));
	}

	float reldev() const noexcept {
		return sqrt(sum0 * sum2 / sqr(sum1) - 1.0f);
	}
};
