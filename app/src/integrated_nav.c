#include <api_os.h>
#include <api_gps.h>
#include <api_debug.h>
#include <api_socket.h>

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <time.h>
#include <cs_types.h>
#include <string.h>

#include "integrated_nav.h"
#include "inertial_filter.h"
#include "geo.h"
#include "JY901_i2c.h"
#include "buffer.h"
#include "gps_parse.h"
#include "mymath.h"

struct location_position local_pos;
extern bool gps_ready;
extern int socketFd;


// transform ddmm.mmmm to dd.dddddd
double gps_dm_2_dd(double dm)
{
	int d = (int)(dm / 100.0f);
	double m = dm / 100.0f - d;
	return d+m/60.0f;
}

// Come from the PX4 position_estimator_inav_thread_main(),
// which is a complementary filter for 3D position and velocity states.
// The position estimator has been deprecated in new PX4 Firmware.

// Because of the 10K stack, we assign these variables in bss segment, not in stack.
// calculation
float q[4];
float A[3][3];		// the attitude maxtix
clock_t t_prev = 0;

float x_est[2] = { 0.0f, 0.0f };	// pos, vel
float y_est[2] = { 0.0f, 0.0f };	// pos, vel
float x_est_prev[2], y_est_prev[2];


float est_buf[EST_BUF_SIZE][3][2];
float R_buf[EST_BUF_SIZE][3][3];
float R_gps[3][3];					// rotation matrix for GPS correction moment

uint8_t buf_ptr = 0;

float corr_gps[3][2] = {
	{ 0.0f, 0.0f },		// N (pos, vel)
	{ 0.0f, 0.0f },		// E (pos, vel)
	{ 0.0f, 0.0f },		// D (pos, vel)
};
float w_gps_xy = 1.0f;

// GPS
GPS_Information_t* gpsInfo;
bool gps_valid = false; 		// GPS is valid
bool ref_inited = false;		
clock_t ref_init_start = 0;
const clock_t ref_init_delay = 1000;	// wait for 1s after 3D fix
struct map_projection_reference_s ref;

static const float min_eph_epv = 2.0f;	// min EPH/EPV, used for weight calculation
static const float max_eph_epv = 20.0f;	// max EPH/EPV acceptable for estimation

float eph;
float epv;

// sensor
float a[] = {0.0f, 0.0f, 0.0f};			// body frame
float acc[] = { 0.0f, 0.0f, 0.0f };		// NED frame
float acc_bias[] = { 0.0f, 0.0f, 0.0f };	// body frame
struct SQ stcQ;
struct SAcc stcAcc;
// socket
clock_t pub_last;

void position_estimator_testTask()
{
	/*
	memset(x_est_prev, 0, sizeof(x_est_prev));
	memset(y_est_prev, 0, sizeof(y_est_prev));
	memset(est_buf, 0, sizeof(est_buf));
	memset(R_buf, 0, sizeof(R_buf));
	memset(R_gps, 0, sizeof(R_gps));
	memset(&ref, 0, sizeof(ref));
	memset(&stcQ, 0, sizeof(stcQ));
	memset(&stcAcc, 0, sizeof(stcAcc));
	*/
	
	gpsInfo = Gps_GetInfo();
	eph = max_eph_epv;
	epv = 1.0;
	pub_last = clock() / CLOCKS_PER_MSEC;
	
	while(1){
		// the start time of each loop, unit: ms
		clock_t t = clock() / CLOCKS_PER_MSEC;

		JY901_read_q(&stcQ);
		// get the current attitude
		q[0] = stcQ.q[0] / 32768.0f;
		q[1] = stcQ.q[1] / 32768.0f;
		q[2] = stcQ.q[2] / 32768.0f;
		q[3] = stcQ.q[3] / 32768.0f;
		// get attitude maxtix
		A[0][0] = q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3];
		A[0][1] = 2 * (q[1]*q[2] - q[0]*q[3]);
		A[0][2] = 2 * (q[1]*q[3] + q[0]*q[2]);
		A[1][0] = 2 * (q[1]*q[2] + q[0]*q[3]);
		A[1][1] = q[0]*q[0] - q[1]*q[1] + q[2]*q[2] - q[3]*q[3];
		A[1][2] = 2 * (q[2]*q[3] - q[0]*q[1]);
		A[2][0] = 2 * (q[1]*q[3] - q[0]*q[2]);
		A[2][1] = 2 * (q[2]*q[3] + q[0]*q[1]);
		A[2][2] = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
		
		/* update the sensor data */
		JY901_read_acc(&stcAcc);
		// get current accelerate unit: m/s^2
		a[0] = stcAcc.a[0] / 32768.0f * 16 * CONSTANTS_ONE_G;
		a[1] = stcAcc.a[1] / 32768.0f * 16 * CONSTANTS_ONE_G;
		a[2] = stcAcc.a[2] / 32768.0f * 16 * CONSTANTS_ONE_G;
		//Trace(3, "ACC: %d %d %d", (int)(a[0]*1000), (int)(a[1]*1000), (int)(a[2]*1000));
		// correct accel bias
		a[0] -= acc_bias[0];
		a[1] -= acc_bias[1];
		a[2] -= acc_bias[2];
		// transform acceleration vector from body frame to NED frame
		for(uint8_t i=0; i<3; i++){
			acc[i] = 0.0f;
			for(uint8_t j=0; j<3; j++){
				acc[i] += A[i][j] * a[j];
			}
		}
		acc[2] -= CONSTANTS_ONE_G;
		//Trace(3, "NED ACC: %d %d %d", (int)(acc[0]*1000), (int)(acc[1]*1000), (int)(acc[2]*1000));
		//Trace(2, "1");
		// update gps data
		if(gps_ready){
			bool reset_est = false;
			
			// hysteresis for GPS quality
			if(gps_valid){
				// double must cast to float, if not the compiler will display following warning:
				/*  /cygdrive/d/Workspace/AiThinker_A9G/GPRS_C_SDK_V1505/hex/app/app.elf: In function `__ltdf2':
				*	/home/lee/gnu_src/gcc_build/mips-elf/libgcc/../.././gcc/dp-bit.c:1276: multiple definition of `__ltdf2'
				*	/cygdrive/d/Workspace/AiThinker_A9G/GPRS_C_SDK_V1505/hex/app/SW_V1505_csdk.elf:( .ramtext+0x13c99): first defined here
				*/
				// And .elf file will not generate.
				if((float)gpsInfo->HDOP > max_eph_epv || (float)gpsInfo->VDOP > max_eph_epv || 
					gpsInfo->fixGPS == 1 || gpsInfo->fixBDS == 1){
					gps_valid = false;
					Trace(3, "GPS signal lost");
				}
			}else{
				if((float)gpsInfo->HDOP < max_eph_epv*0.7f && (float)gpsInfo->VDOP < max_eph_epv*0.7f && 
					gpsInfo->fixGPS > 1 && gpsInfo->fixBDS > 1){
					reset_est = true;
					gps_valid = true;
					Trace(3, "GPS signal found");
				}
			}
			//Trace(2, "2");
			// confirm the refer location
			if(gps_valid){
				double lat = my_atof(gpsInfo->latitude);
				double lon = my_atof(gpsInfo->longitude);
				lat = gps_dm_2_dd(lat);
				lon = gps_dm_2_dd(lon);
				double track_rad = gpsInfo->Course * PI / 180.0;
				double vel_n_m_s = gpsInfo->Speed / 3.6 * my_cos(track_rad);	// N
				double vel_e_m_s = gpsInfo->Speed / 3.6 * my_sin(track_rad);	// E

				//Trace(3, "RAW GPS: %s %s", gpsInfo->latitude, gpsInfo->longitude);
				//Trace(3, "cur GPS: %d %d", (int)(lat*1000000), (int)(lon*1000000));			

				if(!ref_inited){
					if (ref_init_start == 0) {
						ref_init_start = t;
					// get the refer location after 1 second
					} else if (t > ref_init_start + ref_init_delay) {
						ref_inited = true;
						// set position estimate to (0, 0, 0), use GPS velocity for XY
						x_est[0] = 0.0f;
						x_est[1] = (float)vel_n_m_s;
						y_est[0] = 0.0f;
						y_est[1] = (float)vel_e_m_s;
						// initialize projection
						map_projection_init(&ref, lat, lon);
						Trace(3, "init ref: lat=%d, lon=%d", (int)(lat*1000000), (int)(lon*1000000));
					
					}
					
				}
				//Trace(2, "3");
				// calculate the correction and weight for position
				if(ref_inited){
					// project GPS lat lon to plane
					float gps_proj[2];
					map_projection_project(&ref, lat, lon, &(gps_proj[0]), &(gps_proj[1]));
					//Trace(3, "GPS proj: %d %d", (int)(gps_proj[0]*1000), 
					//	(int)(gps_proj[1]*1000));
					
					// reset position estimate when GPS becomes good
					if (reset_est) {
						x_est[0] = gps_proj[0];
						x_est[1] = vel_n_m_s;
						y_est[0] = gps_proj[1];
						y_est[1] = vel_e_m_s;
					}

					// calculate index of estimated values in buffer
					// the max(0, ...) is to insure the PARAMS_DALAY_GPS is not littler than 0
					// and the min(EST_BUF_SIZE, ..) is to insure the PARAMS_DALAY_GPS is not greater than EST_BUF_SIZE
					int est_i = buf_ptr - 1 - MIN(EST_BUF_SIZE - 1, MAX(0, (int)(PARAMS_DALAY_GPS)));
					if (est_i < 0) {
						est_i += EST_BUF_SIZE;
					}

					// calculate correction for position
					corr_gps[0][0] = gps_proj[0] - est_buf[est_i][0][0];
					corr_gps[1][0] = gps_proj[1] - est_buf[est_i][1][0];
					// calculate correction for velocity
					corr_gps[0][1] = vel_n_m_s - est_buf[est_i][0][1];
					corr_gps[1][1] = vel_e_m_s - est_buf[est_i][1][1];
					// take rotation matrix at this moment
					memcpy(R_gps, R_buf[est_i], sizeof(R_gps));
					// calculate weight, fmaxf-->max
					w_gps_xy = min_eph_epv / MAX(min_eph_epv, gpsInfo->HDOP);
					
				}
				//Trace(2, "4");
			}else{
				// no GPS lock
				memset(corr_gps, 0, sizeof(corr_gps));
				ref_init_start = 0;
			}
			gps_ready = false;
		}
		//Trace(2, "5");

		// check for timeout on GPS topic
		float dt = t_prev > 0 ? (t - t_prev) / 1000.0f : 0.0f;
		// fmaxf-->max
		dt = MAX(MIN(0.02, dt), 0.0002);		// constrain dt from 0.2 to 20 ms
		t_prev = t;

		// increase EPH/EPV on each step
		if (eph < 0.000001f) { //get case where eph is 0 -> would stay 0
			eph = 0.001;
		}

		if (eph < max_eph_epv) {
			eph *= 1.0f + dt;
		}

		if (epv < 0.000001f) { //get case where epv is 0 -> would stay 0
			epv = 0.001;
		}

		if (epv < max_eph_epv) {
			epv += 0.005f * dt; // add 1m to EPV each 200s (baro drift)
		}

		// use GPS if it's valid and reference position initialized
		bool use_gps_xy = ref_inited && gps_valid && PARAMS_W_XY_P > MIN_VALID_W;

		float w_xy_gps_p = PARAMS_W_XY_P * w_gps_xy;
		float w_xy_gps_v = PARAMS_W_XY_V * w_gps_xy;

		// accelerometer bias correction for GPS (use buffered rotation matrix)
		float accel_bias_corr[3] = { 0.0f, 0.0f, 0.0f };
		if (use_gps_xy) {
			accel_bias_corr[0] -= corr_gps[0][0] * w_xy_gps_p * w_xy_gps_p;
			accel_bias_corr[0] -= corr_gps[0][1] * w_xy_gps_v;
			accel_bias_corr[1] -= corr_gps[1][0] * w_xy_gps_p * w_xy_gps_p;
			accel_bias_corr[1] -= corr_gps[1][1] * w_xy_gps_v;
		}
		//Trace(2, "6");
		// transform error vector from NED frame to body frame
		for (int i = 0; i < 3; i++) {
			float c = 0.0f;

			for (int j = 0; j < 3; j++) {
				c += R_gps[j][i] * accel_bias_corr[j];
			}

			if (isfinite(c)) {
				acc_bias[i] += c * PARAMS_W_ACC_BIAS * dt;
			}
		}


		//Trace(2, "7");
		if(use_gps_xy){
			// inertial filter prediction for position
			inertial_filter_predict(dt, x_est, acc[0]);
			inertial_filter_predict(dt, y_est, acc[1]);

			if (!(isfinite(x_est[0]) && isfinite(x_est[1]) && isfinite(y_est[0]) && isfinite(y_est[1]))) {
				Trace(3, "BAD ESTIMATE AFTER PREDICTION");
				memcpy(x_est, x_est_prev, sizeof(x_est));
				memcpy(y_est, y_est_prev, sizeof(y_est));
			}

			// fminf-->min
			eph = MIN(eph, gpsInfo->HDOP);

			inertial_filter_correct(corr_gps[0][0], dt, x_est, 0, w_xy_gps_p);
			inertial_filter_correct(corr_gps[1][0], dt, y_est, 0, w_xy_gps_p);

			// timestamp
			inertial_filter_correct(corr_gps[0][1], dt, x_est, 1, w_xy_gps_v);
			inertial_filter_correct(corr_gps[1][1], dt, y_est, 1, w_xy_gps_v);

			if (!(isfinite(x_est[0]) && isfinite(x_est[1]) && isfinite(y_est[0]) && isfinite(y_est[1]))) {
				Trace(3, "BAD ESTIMATE AFTER CORRECTION");
				memcpy(x_est, x_est_prev, sizeof(x_est));
				memcpy(y_est, y_est_prev, sizeof(y_est));
				memset(corr_gps, 0, sizeof(corr_gps));
			} else {
				memcpy(x_est_prev, x_est, sizeof(x_est));
				memcpy(y_est_prev, y_est, sizeof(y_est));
			}
		}else{
			// gradually reset xy velocity estimates 
			//inertial_filter_correct(-x_est[1], dt, x_est, 1, PARAMS_W_XY_RES_V);
			//inertial_filter_correct(-y_est[1], dt, y_est, 1, PARAMS_W_XY_RES_V);
		}

		//Trace(2, "8");
		// push current estimate to buffer
		est_buf[buf_ptr][0][0] = x_est[0];
		est_buf[buf_ptr][0][1] = x_est[1];
		est_buf[buf_ptr][1][0] = y_est[0];
		est_buf[buf_ptr][1][1] = y_est[1];
		// push current rotation matrix to buffer
		memcpy(R_buf[buf_ptr], &A[0][0], sizeof(A));
		buf_ptr++;
		if (buf_ptr >= EST_BUF_SIZE) {
			buf_ptr = 0;
		}
		//Trace(2, "9");
		// send the location to server

		if(t > pub_last + PUB_INTERVAL){
			pub_last = t;
			double est_lat = 0, est_lon = 0;
			map_projection_reproject(&ref, x_est[0], y_est[0], &est_lat, &est_lon);
			Trace(3, "(x, y): (%d, %d)    (vx, vy): (%d, %d)", (int)(x_est[0]*10000), 
				(int)(y_est[0]*10000), (int)(x_est[1]*10000), (int)(y_est[1]*10000));
			//Trace(3, "est lat: %s", my_ftoa(est_lat));
			//Trace(3, "est lon: %s", my_ftoa(est_lon));
			//Trace(3, "est GPS: %d %d", (int)(est_lat*1000000), (int)(est_lon*1000000));
			//Trace(3, "GPS: %s %s SPEED: %d W:%d", gpsInfo->latitude, gpsInfo->longitude,
			//	(int)(gpsInfo->Speed*10000/3.6f), (int)(w_gps_xy*100));
			// assign global viariable and wait to send.
			local_pos.lat = est_lat;
			local_pos.lon = est_lon;
			local_pos.gps_valid = gps_valid;
			local_pos.timestamp = t;
		}
		//Trace(2, "10");
		OS_Sleep(1000/UPDATE_HZ);	
	}
}


