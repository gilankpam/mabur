/* ported from waybeam_venc f956a52:include/star6e_output.h */
#ifndef STAR6E_OUTPUT_H
#define STAR6E_OUTPUT_H

#include <stddef.h>
#include <stdint.h>

#include "star6e.h"
#include "venc_frame_ring.h"

/* frame-shm:// is the ONLY transport (spec 2026-08-28 §3): the RTP
 * packetizer, the sendmmsg batch, the udp:// / unix:// / shm:// socket
 * paths and the audio output all belonged to waybeam's HTTP-configured
 * multi-transport surface and are deleted.  mabur publishes whole access
 * units into VENC_RING_NAME and drone/src/frame_source.cpp consumes them. */

typedef struct {
	venc_frame_ring_t *frame_ring;
	/* Transport-pressure observation cache (telemetry only — never gates
	 * frame transmission).  Populated by star6e_output_observe_pressure
	 * once per frame on the producer thread.
	 *
	 * Hysteresis flag enters at fill_pct >= VENC_PRESSURE_HIGH_WATER_PCT
	 * (75) and exits at fill_pct < LOW (50).  pressure_drops counts
	 * frames observed in pressure.
	 *
	 * last_full_drops / last_writes / last_oversize_drops carry the ring's
	 * lifetime counters cached at observation time. */
	int in_pressure;
	uint32_t pressure_drops;
	uint8_t last_fill_pct;
	uint32_t last_full_drops;
	uint32_t last_writes;
	uint32_t last_oversize_drops;
	int gdr_active;
	int svct_active;
	uint8_t gdr_cycle_len;
	uint8_t gdr_counter;
	/* One WARN per pipeline start (reset by star6e_output_reset's memset)
	 * when packet metadata is incomplete or invalid — the frame is aborted,
	 * never shipped truncated. */
	uint8_t trunc_warned;
} Star6eOutput;

/** Reset output state to uninitialized. */
void star6e_output_reset(Star6eOutput *output);

/** Create the frame-shm ring at `ring_name` (VENC_RING_NAME). */
int star6e_output_init(Star6eOutput *output, const char *ring_name);

/** Observe transport pressure for telemetry. Updates the hysteresis
 *  flag (`output->in_pressure`), the in-pressure counter
 *  (`output->pressure_drops`), and caches the latest fill_pct + ring
 *  lifetime counters (`output->last_*`).  Never directs the caller to
 *  skip — the caller MUST always emit the frame (a producer-side skip
 *  breaks the H.265 reference chain). */
void star6e_output_observe_pressure(Star6eOutput *output);

/* Snapshot the frame-shm ring occupancy.  Returns -1 when the ring is not
 * up. */
int star6e_output_frame_ring_fill(
	const Star6eOutput *output, venc_frame_ring_fill_t *out);

/** Validate that every vendor pack exposes complete, in-bounds NAL
 * descriptors. All consumers must reject the AU when this returns zero. */
int star6e_output_stream_packet_info_complete(
	const MI_VENC_Stream_t *stream);

/** Reject an incomplete AU before output. Returns 1 when rejected,
 * emits a one-time warning, and signals a chain break for reference
 * frames. */
int star6e_output_reject_incomplete_access_unit(Star6eOutput *output,
	const MI_VENC_Stream_t *stream);

/** Publish one encoder access unit into the frame ring. */
size_t star6e_output_send_frame(Star6eOutput *output,
	const MI_VENC_Stream_t *stream);

/** Destroy the ring and release output resources. */
void star6e_output_teardown(Star6eOutput *output);

#endif
