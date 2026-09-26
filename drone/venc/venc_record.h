/* venc_record.h -- pipeline-facing half of the record channel (the public
 * half is in venc_core.h). */
#pragma once
#include "star6e.h"
#include <stdint.h>

#define STAR6E_RECORD_CHANNEL 1   /* ch0 link, ch7 JPEG snapshot */

/* Create + bind the record channel to vpe_port, idle. Call BEFORE the link
 * channel's VPE->VENC bind. Returns 0 (also when disabled) or -1 (logged;
 * the recorder then reports Disabled). */
int venc_record_attach(const MI_SYS_ChnPort_t *vpe_port, uint32_t width,
	uint32_t height, uint32_t src_fps);
/* Stop, join the drain thread, unbind, destroy. Idempotent. Call before the
 * VPE port is unbound. */
void venc_record_detach(void);
