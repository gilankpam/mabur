/* Boot-time sensor frame-rate trim.
 *
 * The sensor driver answers an integer fps request with a line count
 * rounded to whole lines, and the auto-exposure can stretch that count in
 * dim light, so the sensor's real rate lands a little above or below the
 * configured venc.fps. Any excess over the rate the next stage accepts
 * costs one dropped frame per beat period, which the GS sees as a periodic
 * repeated frame. This controller measures the real rate from the encoded
 * frames' pts and re-requests the sensor rate in milli-fps until the real
 * rate sits in a band under venc.fps, within the GS phase lock's steering
 * reach; the lock does the rest. Pure logic: the caller
 * applies each request to the sensor (MI_SNR_SetFps) and feeds every
 * frame's pts. */
#ifndef MABUR_VENC_SENSOR_CAL_H
#define MABUR_VENC_SENSOR_CAL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SENSOR_CAL_MAX_ROUNDS 8
#define SENSOR_CAL_MAX_STEPS 1024
/* One measurement window: at 60 fps ~60 frame intervals, whose median
 * sits under one sensor line's worth of rate (0.018 Hz). */
#define SENSOR_CAL_WINDOW_US 1000000u

typedef enum {
	SENSOR_CAL_NONE = 0,  /* nothing to do */
	SENSOR_CAL_REQUEST,   /* apply sensor_cal_request_mfps() to the sensor */
	SENSOR_CAL_LOCKED,    /* measured rate is inside the band; done */
	SENSOR_CAL_GAVE_UP,   /* round cap hit: apply sensor_cal_request_mfps()
	                       * (the fastest request measured under the
	                       * target, else the boot request) and stop */
} SensorCalAction;

typedef enum {
	SENSOR_CAL_STATE_IDLE = 0,
	SENSOR_CAL_STATE_MEASURING,
	SENSOR_CAL_STATE_LOCKED,
	SENSOR_CAL_STATE_GAVE_UP,
} SensorCalState;

typedef struct {
	uint32_t target_fps;
	uint32_t request_mfps;   /* the request currently applied to the sensor */
	uint32_t prev_request_mfps;
	uint32_t initial_mfps;   /* the boot request, the give-up fallback when
	                          * no request measured under the target */
	uint32_t best_mfps;      /* fastest request measured under the target;
	                          * 0 = none yet */
	double best_hz;
	uint32_t max_mfps;       /* the mode's ceiling; 0 = none */
	SensorCalState state;
	int rounds;
	double measured_hz;
	int have_prev;
	uint32_t prev_pts;
	uint32_t win_start_pts;
	int settle_left;
	int nsteps;
	uint32_t steps[SENSOR_CAL_MAX_STEPS];
} SensorCal;

/* target_fps = venc.fps; initial_mfps = the request already applied to
 * the sensor (normally target_fps * 1000); max_mfps = the sensor mode's
 * ceiling (maxFps * 1000; 0 = none), above which the driver refuses a
 * request. Leaves the controller idle. */
void sensor_cal_init(SensorCal *c, uint32_t target_fps, uint32_t initial_mfps,
	uint32_t max_mfps);
/* Start (or restart) measuring from the current request. */
void sensor_cal_arm(SensorCal *c);
/* One encoded frame's pts (microseconds, wraps freely). */
SensorCalAction sensor_cal_feed(SensorCal *c, uint32_t pts_us);
/* The encoded stream is not carrying the sensor's rate right now (the bind
 * decimates, e.g. low-power 30 fps): drop the partial window, cost no
 * round. The caller simply stops feeding while held; the next feed starts
 * a fresh window with a settle period. No-op unless measuring. */
void sensor_cal_hold(SensorCal *c);
/* The sensor refused the last SENSOR_CAL_REQUEST: keep the previous
 * request and stop. */
void sensor_cal_request_rejected(SensorCal *c);

uint32_t sensor_cal_request_mfps(const SensorCal *c);
double sensor_cal_measured_hz(const SensorCal *c);
int sensor_cal_rounds(const SensorCal *c);
SensorCalState sensor_cal_state(const SensorCal *c);

/* Destination rate for a VPE->VENC (or VPE->recorder) bind. The SDK's
 * frame-rate control on a bind passes at most dst frames per second by its
 * own clock, which is never quite the sensor's: at full rate that costs one
 * dropped frame per beat. So a request at or above the sensor's set rate
 * binds at src:src (no limiter) and only a request below it, the low-power
 * decimation, keeps dst = request. src_fps 0 (unknown) returns the request. */
uint32_t sensor_cal_bind_dst_fps(uint32_t requested_fps, uint32_t full_fps,
	uint32_t src_fps);

/* The AE shutter ceiling, in microseconds, for a sensor running at fps. */
uint32_t sensor_cal_shutter_cap_us(uint32_t fps);

#ifdef __cplusplus
}
#endif
#endif
