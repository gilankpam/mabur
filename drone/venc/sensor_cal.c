#include "sensor_cal.h"

#include <stdlib.h>
#include <string.h>

/* A window must span SENSOR_CAL_WINDOW_US and hold this many clean steps
 * before its median is trusted. */
#define WINDOW_US SENSOR_CAL_WINDOW_US
#define MIN_STEPS 32
/* Steps skipped after a request: the driver applies new timing at a frame
 * boundary and the first intervals straddle both. */
#define SETTLE_STEPS 10
/* A step this long is a producer restart, not a frame interval. */
#define RESYNC_US 1000000u
/* Band under the target, in milli-Hz. The trim only has to hand the GS
 * phase lock a sensor it can steer: under the target (the steer cannot
 * pull a sensor down past its +-500 mfps reach, and the GS will not steer
 * one more than 0.55 Hz off its panel) and close enough that the steer's
 * offset stays small. The lock does the last fraction of a Hz, so the
 * band is wide: several lines on every mode, whatever its line length
 * (the binned modes ~18 mHz a line at 60 fps, native 2944x1656 ~32 mHz,
 * native 2272x1278 ~74 mHz at 100). A band narrower than a line can be
 * straddled forever. The bottom edge is fixed; the top edge scales. */
#define BAND_LO_MHZ 300
#define BAND_HI_MHZ 4
/* The proportional step aims well inside the band, so the driver's
 * rounding to a whole line and the whole-microsecond pts land the first
 * request in the band on any mode: one request, then lock. */
#define BAND_AIM_MHZ 40
/* One binned line is ~18 mfps at 60 fps: the fallback nudge when the
 * proportional step rounds to the same request. */
#define NUDGE_MFPS 18

/* The top edge, the aim and the nudge are sized for 60 fps. One line is
 * fps^2 / line_rate, so above 60 fps they grow by (fps/60)^2 to stay the
 * same number of lines. Below 60 fps they stay at their 60 fps size. */
static double line_scale(uint32_t target_fps)
{
	const double r = (double)target_fps / 60.0;
	return r > 1.0 ? r * r : 1.0;
}

static void start_window(SensorCal *c, uint32_t pts, int settle)
{
	c->win_start_pts = pts;
	c->nsteps = 0;
	c->settle_left = settle;
}

void sensor_cal_init(SensorCal *c, uint32_t target_fps, uint32_t initial_mfps,
	uint32_t max_mfps)
{
	memset(c, 0, sizeof(*c));
	c->max_mfps = max_mfps;
	c->target_fps = target_fps;
	c->request_mfps = initial_mfps;
	c->prev_request_mfps = initial_mfps;
	c->initial_mfps = initial_mfps;
	c->best_mfps = 0;
	c->state = SENSOR_CAL_STATE_IDLE;
}

void sensor_cal_arm(SensorCal *c)
{
	c->state = SENSOR_CAL_STATE_MEASURING;
	c->rounds = 0;
	c->have_prev = 0;
	c->measured_hz = 0.0;
	c->best_mfps = 0;
	c->best_hz = 0.0;
	start_window(c, 0, 0);
}

static int cmp_u32(const void *a, const void *b)
{
	uint32_t x = *(const uint32_t *)a, y = *(const uint32_t *)b;
	return x < y ? -1 : x > y;
}

static double window_median_hz(SensorCal *c)
{
	uint32_t sorted[SENSOR_CAL_MAX_STEPS];
	memcpy(sorted, c->steps, (size_t)c->nsteps * sizeof(uint32_t));
	qsort(sorted, (size_t)c->nsteps, sizeof(uint32_t), cmp_u32);
	return 1e6 / (double)sorted[c->nsteps / 2];
}

SensorCalAction sensor_cal_feed(SensorCal *c, uint32_t pts_us)
{
	if (c->state != SENSOR_CAL_STATE_MEASURING)
		return SENSOR_CAL_NONE;
	if (!c->have_prev) {
		c->have_prev = 1;
		c->prev_pts = pts_us;
		start_window(c, pts_us, c->settle_left);
		return SENSOR_CAL_NONE;
	}
	const uint32_t delta = pts_us - c->prev_pts;
	c->prev_pts = pts_us;
	if (delta == 0 || delta > RESYNC_US) {
		start_window(c, pts_us, SETTLE_STEPS);
		return SENSOR_CAL_NONE;
	}
	if (c->settle_left > 0) {
		--c->settle_left;
		c->win_start_pts = pts_us;
		return SENSOR_CAL_NONE;
	}
	if (c->nsteps < SENSOR_CAL_MAX_STEPS)
		c->steps[c->nsteps++] = delta;
	if (pts_us - c->win_start_pts < WINDOW_US || c->nsteps < MIN_STEPS)
		return SENSOR_CAL_NONE;

	const double hz = window_median_hz(c);
	c->measured_hz = hz;
	const double target = (double)c->target_fps;
	const double scale = line_scale(c->target_fps);
	const double lo = target - BAND_LO_MHZ / 1000.0;
	const double hi = target - BAND_HI_MHZ * scale / 1000.0;
	if (hz <= hi && (!c->best_mfps || hz > c->best_hz)) {
		c->best_mfps = c->request_mfps;
		c->best_hz = hz;
	}
	if (hz >= lo && hz <= hi) {
		c->state = SENSOR_CAL_STATE_LOCKED;
		return SENSOR_CAL_LOCKED;
	}
	const double aim = target - BAND_AIM_MHZ * scale / 1000.0;
	const double err = hz - aim;
	/* Below the band with the request already at the mode's ceiling: this
	 * mode cannot run faster (its fastest line sits under the target), so
	 * its fastest rate is the trim's answer. */
	if (err < 0 && c->max_mfps && c->request_mfps >= c->max_mfps) {
		c->state = SENSOR_CAL_STATE_LOCKED;
		return SENSOR_CAL_LOCKED;
	}
	/* Not converging means the driver does not answer the request the
	 * way this model expects. Fall back to the request that measured
	 * fastest under the target: the integer boot request can run above
	 * it (the native modes are timed ~0.5 Hz fast for the steer's
	 * headroom), and a sensor above the target is out of the phase
	 * lock's reach. The boot request only when nothing measured under. */
	if (++c->rounds >= SENSOR_CAL_MAX_ROUNDS) {
		c->state = SENSOR_CAL_STATE_GAVE_UP;
		c->prev_request_mfps = c->request_mfps;
		c->request_mfps = c->best_mfps ? c->best_mfps : c->initial_mfps;
		return SENSOR_CAL_GAVE_UP;
	}
	/* Proportional step: the driver's rate is close to linear in the
	 * request over this tiny range. A step that rounds to the same
	 * request moves by one line instead. */
	uint32_t next = (uint32_t)((double)c->request_mfps * aim / hz + 0.5);
	if (next == c->request_mfps) {
		const uint32_t nudge = (uint32_t)(NUDGE_MFPS * scale + 0.5);
		next += err > 0 ? -nudge : nudge;
	}
	const uint32_t floor_mfps = c->target_fps * 950;
	uint32_t ceil_mfps = c->target_fps * 1050;
	if (c->max_mfps && c->max_mfps < ceil_mfps)
		ceil_mfps = c->max_mfps;
	if (next < floor_mfps) next = floor_mfps;
	if (next > ceil_mfps) next = ceil_mfps;
	c->prev_request_mfps = c->request_mfps;
	c->request_mfps = next;
	start_window(c, pts_us, SETTLE_STEPS);
	return SENSOR_CAL_REQUEST;
}

void sensor_cal_hold(SensorCal *c)
{
	if (c->state != SENSOR_CAL_STATE_MEASURING)
		return;
	c->have_prev = 0;
	start_window(c, 0, SETTLE_STEPS);
}

void sensor_cal_request_rejected(SensorCal *c)
{
	c->request_mfps = c->prev_request_mfps;
	c->state = SENSOR_CAL_STATE_GAVE_UP;
}

uint32_t sensor_cal_request_mfps(const SensorCal *c) { return c->request_mfps; }
double sensor_cal_measured_hz(const SensorCal *c) { return c->measured_hz; }
int sensor_cal_rounds(const SensorCal *c) { return c->rounds; }
SensorCalState sensor_cal_state(const SensorCal *c) { return c->state; }

uint32_t sensor_cal_bind_dst_fps(uint32_t requested_fps, uint32_t full_fps,
	uint32_t src_fps)
{
	if (src_fps == 0)
		return requested_fps;
	if (full_fps && requested_fps >= full_fps)
		return src_fps;
	return requested_fps;
}

/* In a dim scene the AE pins the shutter at its ceiling and the sensor
 * stretches the frame to the shutter plus ~22 lines of overhead. At the
 * full frame period that stretch sits under the trimmed line up to 90 fps,
 * but at 100 fps it is ~107 us past the period (98.94 Hz whatever is
 * requested), so the trim can never reach its band. Above 90 fps the
 * ceiling leaves 10 % of the period for the overhead. */
#define SHUTTER_FULL_PERIOD_MAX_FPS 90
#define SHUTTER_HIGH_FPS_PCT 90

uint32_t sensor_cal_shutter_cap_us(uint32_t fps)
{
	if (fps == 0)
		return 0;
	if (fps > SHUTTER_FULL_PERIOD_MAX_FPS)
		return 1000000u / fps * SHUTTER_HIGH_FPS_PCT / 100;
	return 1000000u / fps;
}
