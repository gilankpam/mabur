/* venc_core internal seam stub — mabur-local, no waybeam_venc counterpart.
 *
 * Task B4 replaces this translation unit with the real callback dispatch
 * (VencCoreCallbacks.on_chain_break, paced by the 1 s chain-break holdoff
 * from spec §4).  Until then the library links standalone and a chain
 * break is silently absorbed, exactly as if no callback were registered. */
#include "venc_core_internal.h"

void venc_core_signal_chain_break(void)
{
	/* Intentionally empty — see file comment (B4 wires the facade). */
}
