/* ported from waybeam_venc f956a52:src/star6e_output.c */
#include "star6e_output.h"

#include "venc_core_internal.h"

#include <stdio.h>
#include <string.h>

/* STAR6E_REFTYPE_ENHANCE_P_NOTFORREF (=5) is defined in star6e.h. */

/* ported from waybeam_venc f956a52:include/venc_ring.h — the pressure
 * hysteresis constants and observer.  Inlined here because mabur vendors
 * only the frame ring, not waybeam's packet ring.
 *
 * History: the observer used to return 1 to mean "skip-this-frame" and the
 * runtime used that to bypass the encoder output under backpressure.  That
 * was wrong: H.265 inter-frame coding requires the reference chain to be
 * intact, so dropping a P-frame post-encode left every following P-frame in
 * the GOP undecodable at the receiver.  Adaptation belongs upstream of
 * encode (lower bitrate, lower fps) — RcAgent's job.  This just maintains
 * the hysteresis flag and the "frames observed in pressure" counter. */
#define VENC_PRESSURE_HIGH_WATER_PCT 75u
#define VENC_PRESSURE_LOW_WATER_PCT  50u

static void venc_observe_pressure(uint8_t fill_pct,
	int *in_pressure, uint32_t *pressure_drops)
{
	if (!in_pressure || !pressure_drops)
		return;

	if (*in_pressure) {
		if (fill_pct < VENC_PRESSURE_LOW_WATER_PCT)
			*in_pressure = 0;
	} else {
		if (fill_pct >= VENC_PRESSURE_HIGH_WATER_PCT)
			*in_pressure = 1;
	}

	if (*in_pressure)
		(*pressure_drops)++;
}

/* ported from waybeam_venc f956a52:include/venc_frame_ring.h — does a
 * ring-full drop of a frame carrying these flags break the decoder's
 * reference chain?
 *
 * A ring-full drop lands AFTER encode, so unless the frame was
 * non-referenced the decoder renders garbage until the next IDR — with a
 * long GOP, seconds.  An SVC-T enhance frame is droppable by construction
 * (nothing predicts from it), so discarding one costs exactly one frame and
 * must NOT provoke a recovery IDR: that would spend the largest frame in the
 * stream to repair damage that never happened, into a ring already full. */
static int venc_frame_drop_breaks_chain(uint8_t flags)
{
	return (flags & VENC_FRAME_FLAG_ENHANCE) ? 0 : 1;
}

int star6e_output_stream_packet_info_complete(
	const MI_VENC_Stream_t *stream)
{
	unsigned int i;

	if (!stream || stream->count == 0 || !stream->packet)
		return 0;
	for (i = 0; i < stream->count; ++i) {
		const MI_VENC_Pack_t *pack = &stream->packet[i];
		const unsigned int info_cap = (unsigned int)(
			sizeof(pack->packetInfo) / sizeof(pack->packetInfo[0]));
		unsigned int k;

		if (!pack->data || pack->packNum > info_cap)
			return 0;
		if (pack->packNum == 0) {
			if (pack->length <= pack->offset)
				return 0;
			continue;
		}
		for (k = 0; k < (unsigned int)pack->packNum; ++k) {
			MI_U32 offset = pack->packetInfo[k].offset;
			MI_U32 length = pack->packetInfo[k].length;

			if (length == 0 || offset >= pack->length ||
			    length > pack->length - offset)
				return 0;
		}
	}
	return 1;
}

/* An already-encoded access unit was discarded.  Report the chain break
 * upward instead of requesting an IDR here — pure mechanism, spec §4: the
 * facade owns the recovery policy (the 1 s chain-break holdoff layered on
 * the shared 100 ms IDR spacing). */
static void star6e_output_recover_dropped_access_unit(Star6eOutput *output,
	const MI_VENC_Stream_t *stream)
{
	uint8_t flags = 0;

	if (!output || !stream)
		return;
	if (output->svct_active &&
	    stream->h265Info.refType == STAR6E_REFTYPE_ENHANCE_P_NOTFORREF)
		flags |= VENC_FRAME_FLAG_ENHANCE;
	if (venc_frame_drop_breaks_chain(flags))
		venc_core_signal_chain_break();
}

int star6e_output_reject_incomplete_access_unit(Star6eOutput *output,
	const MI_VENC_Stream_t *stream)
{
	if (star6e_output_stream_packet_info_complete(stream))
		return 0;
	if (output && !output->trunc_warned) {
		output->trunc_warned = 1;
		fprintf(stderr,
			"WARN: Star6E packetInfo table is incomplete or invalid; "
			"dropping whole access unit\n");
	}
	star6e_output_recover_dropped_access_unit(output, stream);
	return 1;
}

void star6e_output_reset(Star6eOutput *output)
{
	if (!output)
		return;

	memset(output, 0, sizeof(*output));
}

int star6e_output_init(Star6eOutput *output, const char *ring_name)
{
	if (!output)
		return -1;

	star6e_output_reset(output);
	if (!ring_name || !ring_name[0])
		return -1;

	output->frame_ring = venc_frame_ring_create(ring_name, 8, 384 * 1024);
	if (!output->frame_ring) {
		fprintf(stderr, "ERROR: venc_frame_ring_create(%s) failed\n",
			ring_name);
		return -1;
	}
	return 0;
}

int star6e_output_frame_ring_fill(
	const Star6eOutput *output, venc_frame_ring_fill_t *out)
{
	if (!output || !output->frame_ring || !out)
		return -1;
	return venc_frame_ring_get_fill(output->frame_ring, out);
}

void star6e_output_observe_pressure(Star6eOutput *output)
{
	venc_frame_ring_fill_t fill;

	if (!output)
		return;

	if (!output->frame_ring ||
	    venc_frame_ring_get_fill(output->frame_ring, &fill) != 0) {
		/* No ring (or the query failed) — clear the flag so we don't
		 * get stuck reporting pressure across teardown.  Cached
		 * fill_pct stays at its last value; readers see in_pressure=0
		 * either way. */
		__atomic_store_n(&output->in_pressure, 0, __ATOMIC_RELAXED);
		return;
	}

	venc_observe_pressure(fill.fill_pct,
		&output->in_pressure, &output->pressure_drops);

	__atomic_store_n(&output->last_fill_pct, fill.fill_pct, __ATOMIC_RELAXED);
	__atomic_store_n(&output->last_full_drops, (uint32_t)fill.full_drops,
		__ATOMIC_RELAXED);
	__atomic_store_n(&output->last_writes, (uint32_t)fill.writes,
		__ATOMIC_RELAXED);
	__atomic_store_n(&output->last_oversize_drops,
		(uint32_t)fill.oversize_drops, __ATOMIC_RELAXED);
}

static size_t star6e_output_send_frame_ring(Star6eOutput *output,
	const MI_VENC_Stream_t *stream)
{
	VencFrameMeta meta;
	size_t total_bytes = 0;
	unsigned int i;
	int is_idr = 0;

	if (!output || !output->frame_ring || !stream)
		return 0;

	/* IDR detection from packType.h265Nalu (types 19, 20) */
	for (i = 0; i < stream->count && !is_idr; ++i) {
		const MI_VENC_Pack_t *pack = &stream->packet[i];
		if (pack->packNum > 0) {
			unsigned int nal_count = (unsigned int)pack->packNum;
			unsigned int k;
			for (k = 0; k < nal_count; ++k) {
				uint8_t nt = (uint8_t)pack->packetInfo[k]
					.packType.h265Nalu;
				if (nt == 19 || nt == 20) {
					is_idr = 1;
					break;
				}
			}
		}
	}

	memset(&meta, 0, sizeof(meta));
	meta.pts = (stream->count > 0 && stream->packet)
		? (uint32_t)stream->packet[0].timestamp : 0;
	meta.codec = VENC_FRAME_CODEC_H265;
	meta.flags = is_idr ? VENC_FRAME_FLAG_IDR : 0;
	if (!is_idr && output->gdr_active) {
		meta.flags |= VENC_FRAME_FLAG_GDR;
		meta.gdr_pos = output->gdr_counter;
		meta.gdr_len = output->gdr_cycle_len;
		output->gdr_counter++;
		if (output->gdr_counter >= output->gdr_cycle_len)
			output->gdr_counter = 0;
	} else if (is_idr) {
		output->gdr_counter = 0;
	}
	if (output->svct_active &&
	    stream->h265Info.refType == STAR6E_REFTYPE_ENHANCE_P_NOTFORREF)
		meta.flags |= VENC_FRAME_FLAG_ENHANCE;

	if (venc_frame_ring_begin_write(output->frame_ring, &meta) != 0) {
		if (venc_frame_drop_breaks_chain(meta.flags))
			venc_core_signal_chain_break();
		return 0;
	}

	for (i = 0; i < stream->count; ++i) {
		const MI_VENC_Pack_t *pack = &stream->packet[i];

		if (!pack->data)
			continue;

		if (pack->packNum > 0) {
			unsigned int nal_count = (unsigned int)pack->packNum;
			unsigned int k;

			for (k = 0; k < nal_count; ++k) {
				MI_U32 length = pack->packetInfo[k].length;
				MI_U32 offset = pack->packetInfo[k].offset;

				if (length == 0 || offset >= pack->length ||
				    length > (pack->length - offset))
					continue;

				if (venc_frame_ring_append(output->frame_ring,
				    pack->data + offset, length) != 0) {
					venc_frame_ring_abort_write(
						output->frame_ring);
					star6e_output_recover_dropped_access_unit(
						output, stream);
					return 0;
				}
				total_bytes += length;
			}
			continue;
		}

		if (pack->length > pack->offset) {
			MI_U32 length = pack->length - pack->offset;

			if (venc_frame_ring_append(output->frame_ring,
			    pack->data + pack->offset, length) != 0) {
				venc_frame_ring_abort_write(output->frame_ring);
				star6e_output_recover_dropped_access_unit(output,
					stream);
				return 0;
			}
			total_bytes += length;
		}
	}

	venc_frame_ring_commit_write(output->frame_ring);
	return total_bytes;
}

size_t star6e_output_send_frame(Star6eOutput *output,
	const MI_VENC_Stream_t *stream)
{
	if (!output || !stream)
		return 0;
	if (star6e_output_reject_incomplete_access_unit(output, stream))
		return 0;

	return star6e_output_send_frame_ring(output, stream);
}

void star6e_output_teardown(Star6eOutput *output)
{
	if (!output)
		return;

	if (output->frame_ring) {
		venc_frame_ring_destroy(output->frame_ring);
		output->frame_ring = NULL;
	}
}
