/* ported from waybeam_venc f956a52:include/star6e_runtime.h */
#ifndef STAR6E_RUNTIME_H
#define STAR6E_RUNTIME_H

#include "star6e_pipeline.h"
#include "venc_cfg.h"

/* waybeam's BackendOps table, the SIGHUP fork+exec respawn, the mDNS
 * beacon and the httpd are all deleted: maburd owns the process, its
 * signals and its lifetime.  What survives is the four-call encoder
 * lifecycle the facade (task B4) drives. */

typedef struct {
	VencCfg cfg;
	Star6ePipelineState ps;
	int system_initialized;
	int pipeline_started;
} Star6eRunnerContext;

#ifdef __cplusplus
extern "C" {
#endif

/** Reset per-run state.  Call once before star6e_runtime_init(). */
int star6e_runtime_prepare(void);

/** Load the MI libraries, MI_SYS_Init, bring the pipeline up and apply the
 *  boot-time controls.  ctx->cfg must already be filled.  Returns 0 on
 *  success; on failure the caller must still run the teardown. */
int star6e_runtime_init(Star6eRunnerContext *ctx);

/** Encoder thread body: drains the VENC channel into the frame-shm ring
 *  until star6e_runtime_request_stop().  `opaque` is the
 *  Star6eRunnerContext.
 *
 *  Returns NULL when the loop exited because a stop was requested, or a
 *  `const char *` describing the MI failure that killed it (valid until the
 *  next star6e_runtime_prepare()).  venc_core turns the non-NULL case into
 *  the VencCallbacks.on_fault callback — maburd keeps flying with a dead encoder,
 *  so "stopped" and "died" must not look alike. */
void *star6e_runtime_encoder_loop(void *opaque);

/** Ask the encoder loop to return.  Safe from any thread / a signal
 *  handler. */
void star6e_runtime_request_stop(void);

/** Tear the pipeline and the MI libraries down. Idempotent per field. */
void star6e_runtime_teardown(Star6eRunnerContext *ctx);

#ifdef __cplusplus
}
#endif

#endif /* STAR6E_RUNTIME_H */
