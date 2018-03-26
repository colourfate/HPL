#ifndef _INTEGRATED_NAV_H_
#define _INTEGRATED_NAV_H_

#define INAV_TASK_STACK_SIZE    (2048 * 5)		// 1MB
#define INAV_TASK_PRIORITY      0
#define INAV_TASK_NAME "inertial navigation and position estimation"

#define CONSTANTS_ONE_G 9.80665f
#define PI 3.141592654f
// The inav update rate is 20Hz and the buffer length is 40, so the buffer can store data within 1.6 second
#define EST_BUF_SIZE 40
#define MIN_VALID_W 0.00001f

#define PARAMS_W_XY_P 
#define PARAMS_W_XY_V
#define PARAMS_W_ACC_BIAS
#define PARAMS_W_XY_RES_V

#define min(a,b) (a<b ? a : b)
#define max(a,b) (a>b ? a : b)
void position_estimator_testTask(void *pData);


#endif
