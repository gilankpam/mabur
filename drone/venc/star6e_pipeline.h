/* ported from waybeam_venc f956a52:include/star6e_pipeline.h */
#ifndef STAR6E_PIPELINE_H
#define STAR6E_PIPELINE_H

#include "sdk_quiet.h"
#include "sensor_select.h"
#include "star6e_output.h"
#include "star6e_video.h"
#include "venc_cfg.h"

#include <pthread.h>
#include <signal.h>
#include <time.h>

/* Hard VENC encoder-input frame-rate ceiling on Infinity6E.  The SDK's
 * _MI_VENC_VerifyFps rejects any input FPS > 120 and silently resets it to
 * 30/1 ("Input invalid FPS:144/1 is over 120, set 30/1 by default"), which
 * makes CBR rate control budget for 30fps while ~143 frames/s actually arrive
 * -> ~4.7x bitrate overshoot.  Clamp the VPE->VENC bind dst AND the encoder
 * rate-control fps to this so a >120 sensor mode (e.g. 1600x900@144) encodes
 * cleanly at 120fps instead of overshooting.  Sensor/VIF may still run 143;
 * the framebase bind drops the surplus. */
#define STAR6E_VENC_INPUT_FPS_MAX 120u

typedef struct {
	uint16_t x;
	uint16_t y;
	uint16_t w;
	uint16_t h;
} Star6ePrecropRect;

/* Audio, the hevc/ts recorders, the IMU ring, the dual (gemini) VENC
 * channel, the debug OSD, the IPU detector and the QR luma tap are all
 * deleted with their subsystems. */
typedef struct {
	SensorSelectResult sensor;
	MI_VENC_CHN venc_channel;
	MI_SYS_ChnPort_t vif_port;
	MI_SYS_ChnPort_t vpe_port;
	MI_SYS_ChnPort_t venc_port;
	int bound_vif_vpe;
	int bound_vpe_venc;
	Star6eOutput output;
	Star6eVideoState video;
	uint32_t image_width;
	uint32_t image_height;
	MI_VENC_Pack_t *stream_packs;     /* pre-allocated for main loop */
	uint32_t stream_packs_cap;
	Star6ePrecropRect active_precrop; /* precrop currently programmed into VIF
	                                   * (includes overscan offsets) */
} Star6ePipelineState;

/** Initialize and start the full encoder pipeline (sensor → VENC). */
int star6e_pipeline_start(Star6ePipelineState *state, const VencCfg *cfg,
	SdkQuietState *sdk_quiet);

/** Stop streaming, unbind hardware, and release pipeline resources.
 *  Also clears pipeline-level persist state so the next start is cold. */
void star6e_pipeline_stop(Star6ePipelineState *state);

/** Disable VPE prescaler (cleanup during shutdown). */
void star6e_pipeline_vpe_scl_preset_shutdown(void);

/** One-shot cold-boot fps re-kick.  Call once ~1.5s after pipeline start from
 *  the run loop; re-issues MI_SNR_SetFps to force the sensor timing register to
 *  the configured fps (the init-time kick fires before the ISP bin settles and
 *  can leave the sensor locked low on a cold boot).  No-op when fps is 0. */
void star6e_pipeline_cold_boot_fps_rekick(const Star6ePipelineState *state);

/** Service custom 3A (AWB/AE) at regular intervals. */
void star6e_pipeline_cus3a_tick(SdkQuietState *sdk_quiet,
	struct timespec *ts_last);

/** Choose which algorithm owns AWB: 1 = userspace (the star6e_awb loop),
 *  0 = ISP-internal.  Applied live when the CUS3A handoff has already run. */
void star6e_pipeline_set_awb_userspace(int on);

/** Reset CUS3A handoff state (call on pipeline reinit). */
void star6e_pipeline_cus3a_reset(void);

/** Calculate max exposure time to avoid frame drops at target FPS.
 *  shutter_rule_180 = true halves the cap (180° shutter rule). */
int star6e_pipeline_cap_exposure_for_fps(uint32_t fps,
	bool shutter_rule_180);

#endif /* STAR6E_PIPELINE_H */
