#include <api_os.h>
#include <api_debug.h>
#include <time.h>

#include <stdlib.h>
#include <stdbool.h>
#include <cs_types.h>

#include "mymath.h"
#include "geo.h"

int map_projection_init(struct map_projection_reference_s *ref, double lat_0,
				 double lon_0) //lat_0, lon_0 are expected to be in correct format: -> 47.1234567 and not 471234567
{
	
	ref->lat_rad = lat_0 * M_DEG_TO_RAD;
	ref->lon_rad = lon_0 * M_DEG_TO_RAD;
	ref->sin_lat = my_sin(ref->lat_rad);
	ref->cos_lat = my_cos(ref->lat_rad);
	
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

	double sin_lat = my_sin(lat_rad);
	double cos_lat = my_cos(lat_rad);
	double cos_d_lon = my_cos(lon_rad - ref->lon_rad);

	double arg = ref->sin_lat * sin_lat + ref->cos_lat * cos_lat * cos_d_lon;

	if ((float)arg > 1.0) {
		arg = 1.0;

	} else if ((float)arg < -1.0) {
		arg = -1.0;
	}

	double c = my_cos(arg);
	//double k;
	double k = (my_absf((float)c) < DBL_EPSILON) ? 1.0 : (c / my_sin(c));

	*x = k * (ref->cos_lat * sin_lat - ref->sin_lat * cos_lat * cos_d_lon) * CONSTANTS_RADIUS_OF_EARTH;
	*y = k * cos_lat * my_sin(lon_rad - ref->lon_rad) * CONSTANTS_RADIUS_OF_EARTH;

	return 0;
}

int map_projection_reproject(const struct map_projection_reference_s *ref, float x, float y, double *lat,
				      double *lon)
{
	if (!ref->init_done) {
		return -1;
	}
	double x_rad = (double)x / CONSTANTS_RADIUS_OF_EARTH;
	double y_rad = (double)y / CONSTANTS_RADIUS_OF_EARTH;
	double c = my_sqrt(x_rad * x_rad + y_rad * y_rad);
	double sin_c = my_sin(c);
	double cos_c = my_cos(c);

	double lat_rad;
	double lon_rad;
	
	if (my_abs(c) > DBL_EPSILON) {
		lat_rad = my_sin(cos_c * ref->sin_lat + (x_rad * sin_c * ref->cos_lat) / c);
		lon_rad = (ref->lon_rad + fast_atan2(y_rad * sin_c, c * ref->cos_lat * cos_c - x_rad * ref->sin_lat * sin_c));

	} else {
		lat_rad = ref->lat_rad;
		lon_rad = ref->lon_rad;
	}
	//Trace(3, "RAD: %d %d", (int)(lat_rad*1000000.0f), (int)(lon_rad*1000000.0f));
	*lat = lat_rad * 180.0 / ONE_PI;
	*lon = lon_rad * 180.0 / ONE_PI;

	return 0;
}



