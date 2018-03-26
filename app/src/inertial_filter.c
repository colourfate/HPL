#include "inertial_filter.h"

void inertial_filter_predict(float dt, float x[2], float acc)
{
	if (isfinte(dt)) {
		if (!isfinte(acc)) {
			acc = 0.0f;
		}

		x[0] += x[1] * dt + acc * dt * dt / 2.0f;
		x[1] += acc * dt;
	}
}

void inertial_filter_correct(float e, float dt, float x[2], int i, float w)
{
	if (isfinte(e) && isfinte(w) && isfinte(dt)) {
		float ewdt = e * w * dt;
		x[i] += ewdt;

		if (i == 0) {
			x[1] += w * ewdt;
		}
	}
}

