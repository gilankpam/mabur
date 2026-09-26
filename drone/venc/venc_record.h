/* venc_record.h -- pipeline-facing half of the record channel (the public
 * half is in venc_core.h). */
#pragma once
#include "star6e.h"
#include <stdint.h>

#define STAR6E_RECORD_CHANNEL 1   /* ch0 link, ch7 JPEG snapshot */

/* Create + bind the record channel, idle. Call BEFORE the link channel's
 * VPE->VENC bind. vpe_port is the link's port (0) at link_w x link_h; a
 * configured size other than that enables port 1 of the same VPE channel at
 * the record size (at most cap_w x cap_h, the VPE capture window) and binds
 * there. Returns 0 (also when disabled) or -1 (logged; the recorder then
 * reports Disabled). */
int venc_record_attach(const MI_SYS_ChnPort_t *vpe_port, uint32_t link_w,
	uint32_t link_h, uint32_t cap_w, uint32_t cap_h, uint32_t src_fps);
/* The configured recording size, or 0x0 when the recorder is disabled or
 * follows the link's size. Valid after venc_record_configure. */
void venc_record_size(uint32_t *width, uint32_t *height);
/* Stop, join the drain thread, unbind, destroy. Idempotent. Call before the
 * VPE port is unbound. */
void venc_record_detach(void);
