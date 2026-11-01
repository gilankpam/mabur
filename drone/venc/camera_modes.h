/* Which IMX415 sensor mode serves which venc.fps, and which encode sizes
 * that mode can serve. The table is waybeam's star6e IMX415 driver lineup
 * (sensor_imx415_star6e.ko), the driver the drone image ships; modes are
 * found in the driver's list by geometry and ceiling, never by index.
 * Two readouts share the lineup: binned (2x2, the full view) and native
 * (full-resolution, a narrower fps range). camera_mode_for's binning flag
 * picks which readout's tiers apply; a zero-size result (w == 0) means
 * there is no mode for that readout at that rate, and the config loader
 * refuses it so the pipeline never sees it.
 * docs/camera-modes.md has the per-mode view, binning and limits. Pure C,
 * shared by the pipeline and the config loader. */
#ifndef MABUR_VENC_CAMERA_MODES_H
#define MABUR_VENC_CAMERA_MODES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	uint16_t w, h;     /* the mode's output size */
	uint16_t max_fps;  /* the driver's maxFps for it */
} CameraMode;

/* The mode for a configured venc.fps and readout (binning non-zero selects
 * the binned tiers, zero selects the native tiers), or the 4K mode when
 * the recorder wants more than 1920x1080 and more than the tier's mode in
 * either dimension (the loader then requires venc.fps <= 30). */
CameraMode camera_mode_for(int binning, uint32_t venc_fps, uint32_t rec_w, uint32_t rec_h);

/* Index of the mode with want's size and ceiling in a driver's list, or -1. */
int camera_mode_find(const CameraMode *list, int n, CameraMode want);

/* 1 when w x h has the mode's aspect ratio and fits inside it (the VPE
 * scales down; a crop or an upscale is not offered). */
int camera_mode_size_ok(CameraMode mode, uint32_t w, uint32_t h);

#ifdef __cplusplus
}
#endif
#endif
