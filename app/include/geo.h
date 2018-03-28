#ifndef _GEO_H_
#define _GEO_H_
#define M_DEG_TO_RAD 0.0174533
#define DBL_EPSILON 2.2204460492503131e-16  /* 1E-9 */
#define CONSTANTS_RADIUS_OF_EARTH 6371000


/* lat/lon are in radians */
struct map_projection_reference_s {
	uint64_t timestamp;
	double lat_rad;
	double lon_rad;
	double sin_lat;
	double cos_lat;
	bool init_done;
};
int map_projection_init(struct map_projection_reference_s *ref, double lat_0,
				 double lon_0);
int map_projection_project(const struct map_projection_reference_s *ref, double lat, double lon, float *x,
				    float *y);


#endif
