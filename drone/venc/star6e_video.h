/* ported from waybeam_venc f956a52:include/star6e_video.h */
#ifndef STAR6E_VIDEO_H
#define STAR6E_VIDEO_H

#include "star6e.h"
#include "star6e_output.h"

#include <stddef.h>
#include <stdint.h>

/* RTP packetizer state, the H.26x parameter-set cache, the sidecar sender
 * and the verbose stream-metrics sampler are all deleted: mabur's only
 * transport is the frame-shm ring, which takes whole access units. */
typedef struct {
	uint32_t sensor_framerate;
	unsigned int frame_counter;
} Star6eVideoState;

/** Reset video state to uninitialized (safe to reuse). */
void star6e_video_reset(Star6eVideoState *state);

/** Initialize per-run video state. */
void star6e_video_init(Star6eVideoState *state, uint32_t sensor_framerate);

/** Send one encoded frame to the configured output. */
size_t star6e_video_send_frame(Star6eVideoState *state,
	Star6eOutput *output, const MI_VENC_Stream_t *stream);

#endif /* STAR6E_VIDEO_H */
