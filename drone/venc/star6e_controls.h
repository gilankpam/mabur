/* ported from waybeam_venc f956a52:include/star6e_controls.h */
#ifndef STAR6E_CONTROLS_H
#define STAR6E_CONTROLS_H

#include "star6e_pipeline.h"
#include "venc_cfg.h"

/* waybeam's VencApplyCallbacks table (the HTTP /api/v1/set live-apply glue)
 * is deleted along with httpd; the surviving controls are called directly.
 * Everything here is pure mechanism — no encoder-local policy (spec §4). */

/** Bind pipeline and config to runtime control state. */
void star6e_controls_bind(Star6ePipelineState *pipeline, const VencCfg *cfg);

/** Reset control state to defaults (called between pipeline restarts). */
void star6e_controls_reset(void);

/** Program the encoder rate.  The ONLY rate source is RcAgent — there is
 *  no venc.bitrate key and no encoder-local clamp policy beyond the
 *  VENC_BITRATE_{MIN,MAX}_KBPS rails and the >120 fps exact-CBR
 *  compensation.  Does NOT request an IDR (spec §4). */
int star6e_controls_apply_bitrate(uint32_t kbps);

/** Apply ROI-based QP adjustment for FPV center emphasis. */
int star6e_controls_apply_roi_qp(int qp);

/** Apply relative I/P QP delta to the running encoder (boot-time). */
int star6e_controls_apply_qp_delta(int delta);

/** Explicit IDR request.  Goes through the shared per-channel 100 ms
 *  rate limiter (idr_rate_limit.h); a coalesced request returns 0, not an
 *  error.  This is the only place in venc_core that calls
 *  MI_VENC_RequestIdr on behalf of a caller.
 *
 *  Unguarded: it dereferences the dlopen'd MI vtable, so it is only legal
 *  between pipeline start and stop.  The public verb venc_request_idr()
 *  (venc_core.h) is this call plus the started guard — external callers
 *  want that one. */
int star6e_controls_request_idr(void);

#endif /* STAR6E_CONTROLS_H */
