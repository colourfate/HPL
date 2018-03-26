#include <api_os.h>
#include <api_debug.h>
#include <time.h>
#include <math.h>

int map_projection_init(struct map_projection_reference_s *ref, double lat_0,
				 double lon_0) //lat_0, lon_0 are expected to be in correct format: -> 47.1234567 and not 471234567
{
	ref->lat_rad = lat_0 * M_DEG_TO_RAD;
	ref->lon_rad = lon_0 * M_DEG_TO_RAD;
	ref->sin_lat = sin(ref->lat_rad);
	ref->cos_lat = cos(ref->lat_rad);

	ref->timestamp = clock() / CLOCKS_PER_MSEC;
	ref->init_done = true;

	return 0;
}

int map_projection_project(const struct map_projection_reference_s *ref, double lat, double lon, float *x,
				    float *y)
{
	if (!ref->init_done) {
		return -1;
	}

	double lat_rad = lat * M_DEG_TO_RAD;
	double lon_rad = lon * M_DEG_TO_RAD;

	double sin_lat = sin(lat_rad);
	double cos_lat = cos(lat_rad);
	double cos_d_lon = cos(lon_rad - ref->lon_rad);

	double arg = ref->sin_lat * sin_lat + ref->cos_lat * cos_lat * cos_d_lon;

	if (arg > 1.0) {
		arg = 1.0;

	} else if (arg < -1.0) {
		arg = -1.0;
	}

	double c = acos(arg);
	double k = (fabs(c) < DBL_EPSILON) ? 1.0 : (c / sin(c));

	*x = k * (ref->cos_lat * sin_lat - ref->sin_lat * cos_lat * cos_d_lon) * CONSTANTS_RADIUS_OF_EARTH;
	*y = k * cos_lat * sin(lon_rad - ref->lon_rad) * CONSTANTS_RADIUS_OF_EARTH;

	return 0;
}


