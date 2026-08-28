/* venc_core internal seam — mabur-local, no waybeam_venc counterpart.
 *
 * The encoder is a pure mechanism (spec 2026-08-28 §4): when a frame-shm
 * ring-full drop breaks the decoder's reference chain, star6e_output does
 * NOT decide to spend an IDR on it.  It reports the fact upward and the
 * facade owns the policy (the 1 s chain-break holdoff on top of the
 * shared 100 ms IDR spacing). */
#ifndef VENC_CORE_INTERNAL_H
#define VENC_CORE_INTERNAL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Called on the encoder thread from star6e_output when an already-encoded
 * REFERENCE frame was discarded (ring full / append failure).  A dropped
 * non-referenced SVC-T enhance frame costs exactly one frame and never
 * reaches here. */
void venc_core_signal_chain_break(void);

#ifdef __cplusplus
}
#endif

#endif /* VENC_CORE_INTERNAL_H */
