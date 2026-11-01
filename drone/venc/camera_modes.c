#include "camera_modes.h"

/* Up to 90 fps: the 2x2-binned full view. Above it the sensor cannot read
 * the full height fast enough (a wide binned line needs ~4.9 us and the
 * full height ~2250 lines), so 91-100 and 101-120 use the widest binned
 * crops that stay clean at those rates. */
static const CameraMode kFullView = {1920, 1080, 90};
static const CameraMode kWide100 = {1728, 972, 100};
static const CameraMode kWide120 = {1728, 816, 120};
/* The native (non-binned) readout: a narrower fps range than the binned
 * one, and nothing above 100 fps (kNone — the loader refuses it). */
static const CameraMode kNative60 = {2944, 1656, 60};
static const CameraMode kNative100 = {2304, 1296, 100};
static const CameraMode kNone = {0, 0, 0};
static const CameraMode k4K = {3840, 2160, 33};

CameraMode camera_mode_for(int binning, uint32_t venc_fps, uint32_t rec_w, uint32_t rec_h)
{
	CameraMode tier;
	if (binning) {
		if (venc_fps > kWide100.max_fps)
			tier = kWide120;
		else if (venc_fps > kFullView.max_fps)
			tier = kWide100;
		else
			tier = kFullView;
	} else {
		if (venc_fps > kNative100.max_fps)
			tier = kNone;
		else if (venc_fps > kNative60.max_fps)
			tier = kNative100;
		else
			tier = kNative60;
	}
	if ((rec_w > 1920 || rec_h > 1080) && (rec_w > tier.w || rec_h > tier.h))
		return k4K;
	return tier;
}

int camera_mode_find(const CameraMode *list, int n, CameraMode want)
{
	for (int i = 0; i < n; ++i)
		if (list[i].w == want.w && list[i].h == want.h &&
		    list[i].max_fps == want.max_fps)
			return i;
	return -1;
}

int camera_mode_size_ok(CameraMode mode, uint32_t w, uint32_t h)
{
	return (uint64_t)w * mode.h == (uint64_t)h * mode.w &&
		w <= mode.w && h <= mode.h;
}
