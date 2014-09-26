/*
 * geo.cpp
 *
 *  Created on: Sep 25, 2014
 *      Author: roman
 */

#include <geo/geo.h>
#include <unistd.h>
#include <pthread.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>
#include <float.h>

#define M_PI_F 3.14159265358979323846f
#define M_TWOPI_F       (M_PI_F * 2.0f)


float _wrap_pi(float bearing)
{
	/* value is inf or NaN */
	if (!isfinite(bearing)) {
		return bearing;
	}

	int c = 0;
	while (bearing >= M_PI_F) {
		bearing -= M_TWOPI_F;

		if (c++ > 3) {
			return NAN;
		}
	}

	c = 0;
	while (bearing < -M_PI_F) {
		bearing += M_TWOPI_F;

		if (c++ > 3) {
			return NAN;
		}
	}

	return bearing;
}

float _wrap_2pi(float bearing)
{
	/* value is inf or NaN */
	if (!isfinite(bearing)) {
		return bearing;
	}

	int c = 0;
	while (bearing >= M_TWOPI_F) {
		bearing -= M_TWOPI_F;

		if (c++ > 3) {
			return NAN;
		}
	}

	c = 0;
	while (bearing < 0.0f) {
		bearing += M_TWOPI_F;

		if (c++ > 3) {
			return NAN;
		}
	}

	return bearing;
}

float _wrap_180(float bearing)
{
	/* value is inf or NaN */
	if (!isfinite(bearing)) {
		return bearing;
	}

	int c = 0;
	while (bearing >= 180.0f) {
		bearing -= 360.0f;

		if (c++ > 3) {
			return NAN;
		}
	}

	c = 0;
	while (bearing < -180.0f) {
		bearing += 360.0f;

		if (c++ > 3) {
			return NAN;
		}
	}

	return bearing;
}

float _wrap_360(float bearing)
{
	/* value is inf or NaN */
	if (!isfinite(bearing)) {
		return bearing;
	}

	int c = 0;
	while (bearing >= 360.0f) {
		bearing -= 360.0f;

		if (c++ > 3) {
			return NAN;
		}
	}

	c = 0;
	while (bearing < 0.0f) {
		bearing += 360.0f;

		if (c++ > 3) {
			return NAN;
		}
	}

	return bearing;
}
