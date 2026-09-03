/* ported from waybeam_venc f956a52:src/star6e_video.c */
#include "star6e_video.h"

#include <stdio.h>
#include <string.h>

void star6e_video_reset(Star6eVideoState *state)
{
	if (!state)
		return;

	memset(state, 0, sizeof(*state));
}

void star6e_video_init(Star6eVideoState *state, uint32_t sensor_framerate)
{
	if (!state)
		return;

	memset(state, 0, sizeof(*state));
	state->sensor_framerate = sensor_framerate;
}

size_t star6e_video_send_frame(Star6eVideoState *state,
	Star6eOutput *output, const MI_VENC_Stream_t *stream)
{
	if (!state || !output || !stream)
		return 0;

	state->frame_counter++;
	__atomic_store_n(&state->last_qp, stream->h265Info.startQual,
		__ATOMIC_RELAXED);
	/* Which MI_VENC_StreamInfoH265_t fields this firmware actually fills:
	 * startQual read 0 on every frame at the first deploy (2026-09-03)
	 * while refType was live, so dump the whole struct for the first few
	 * frames and then once a minute (stderr -> /tmp/mabur.log). */
	if (state->frame_counter <= 3 || state->frame_counter % 3600 == 0) {
		const i6_venc_strminfo_h265 *si = &stream->h265Info;
		fprintf(stderr, "venc_strminfo: n=%u size=%u iCu64/32/16/8=%u/%u/%u/%u "
			"pCu32/16/8/4=%u/%u/%u/%u refType=%u updAttrCnt=%u startQual=%u\n",
			state->frame_counter, si->size, si->iCu64x64, si->iCu32x32,
			si->iCu16x16, si->iCu8x8, si->pCu32x32, si->pCu16x16,
			si->pCu8x8, si->pCu4x4, si->refType, si->updAttrCnt,
			si->startQual);
	}

	return star6e_output_send_frame(output, stream);
}
