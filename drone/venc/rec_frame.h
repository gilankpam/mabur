/* rec_frame.h -- pure, host-testable helpers for the VTX recorder's drain
 * thread (drone/venc/venc_record.c). No MI types, no state.
 *
 * The SDK hands each frame as packs whose packetInfo[] table already names
 * every NAL slice (offset, length, type); each slice starts with an Annex-B
 * start code. The MP4 muxer wants each NAL behind a 4-byte big-endian length
 * instead. Rewriting that while copying the slice out of the SDK buffer --
 * a copy the drain makes anyway -- spares the recorder a full start-code
 * scan and a second full copy per frame (tests/test_rec_frame.cpp). */
#ifndef REC_FRAME_H
#define REC_FRAME_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>

/* Length of the Annex-B start code at p (3 or 4), or 0 if there is none. */
static inline size_t rec_start_code_len(const uint8_t *p, size_t n)
{
	if (n >= 4 && p[0] == 0 && p[1] == 0 && p[2] == 0 && p[3] == 1)
		return 4;
	if (n >= 3 && p[0] == 0 && p[1] == 0 && p[2] == 1)
		return 3;
	return 0;
}

/* Append one SDK NAL slice (start code + NAL) to dst as a 4-byte big-endian
 * length followed by the NAL bytes. Returns the bytes written (4 + NAL
 * size); 0 when the slice has no start code, no NAL header after it, or
 * does not fit in cap. *type gets the HEVC NAL type on success. */
static inline size_t rec_put_nal(uint8_t *dst, size_t cap, const uint8_t *slice,
	size_t n, int *type)
{
	const size_t sc = rec_start_code_len(slice, n);
	size_t nal;

	if (sc == 0 || n < sc + 2)
		return 0;
	nal = n - sc;
	if (nal > 0xFFFFFFFFu || cap < 4 || nal > cap - 4)
		return 0;
	dst[0] = (uint8_t)(nal >> 24);
	dst[1] = (uint8_t)(nal >> 16);
	dst[2] = (uint8_t)(nal >> 8);
	dst[3] = (uint8_t)nal;
	memcpy(dst + 4, slice + sc, nal);
	if (type)
		*type = (slice[sc] >> 1) & 0x3F;
	return 4 + nal;
}

#endif /* REC_FRAME_H */
