/* ported from waybeam_venc f956a52:src/star6e_video.c */
#include "star6e_video.h"

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

	return star6e_output_send_frame(output, stream);
}
