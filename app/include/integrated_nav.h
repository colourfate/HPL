#ifndef _INTEGRATED_NAV_H_
#define _INTEGRATED_NAV_H_
#include <stdbool.h>
#include <time.h>

#define INAV_TASK_STACK_SIZE    (1024 * 10)		// 10K
#define INAV_TASK_PRIORITY      1
#define INAV_TASK_NAME "position estimation"

#define CONSTANTS_ONE_G 9.80665f
#define PI 3.141592654
// The inav update rate is 20Hz and the buffer length is 40, so the buffer can store data within 2 second
#define UPDATE_HZ		2
#define EST_BUF_SIZE 	40
#define MIN_VALID_W 	0.00001f
#define PUB_INTERVAL	5000 // the send interval is 5 second

#define PARAMS_W_XY_P 1
#define PARAMS_W_XY_V 0.5
#define PARAMS_W_ACC_BIAS 0.064
#define PARAMS_W_XY_RES_V 1
#define PARAMS_DALAY_GPS (1*UPDATE_HZ)	// GPS delay = 1 s

struct location_position{
	double lat;
	double lon;
	bool gps_valid;
	clock_t timestamp;
};
extern struct location_position local_pos;

void position_estimator_testTask();
int isfinite(float v);



#endif
