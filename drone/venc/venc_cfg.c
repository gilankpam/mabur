/* ported-from/derived-from waybeam_venc f956a52:src/venc_config.c —
 * only the resilience-preset table and its expansion survive; every
 * cJSON parse/save/load and live-apply grouping is deleted (mabur's
 * config parser fills VencCfg, see spec 2026-08-28 §3). */
#include "venc_cfg.h"

#include <errno.h>
#include <stdlib.h>
#include <string.h>

static void preset_strcpy(char *dst, size_t dst_sz, const char *src)
{
	if (!dst || dst_sz == 0)
		return;
	if (!src) {
		dst[0] = '\0';
		return;
	}
	strncpy(dst, src, dst_sz - 1);
	dst[dst_sz - 1] = '\0';
}

/* Absent-key defaults (spec 2026-08-28 venc-foldin §3).  See venc_cfg.h.
 *
 * These are the values the shipped bundle carries, so a config that omits
 * a venc key gets the flown configuration rather than a zero.  sensor_bin
 * is deliberately left EMPTY: it is the one device-specific field (the ISP
 * calibration blob path), and the config loader treats an empty sensor_bin
 * as a boot failure.
 *
 * Kept in venc_cfg.c, next to the preset table, rather than as C++ member
 * initialisers in config.h, so the vendored C struct owns its own defaults
 * and any future non-mabur caller of venc_core_start() gets them too. */
void venc_cfg_defaults(VencCfg *cfg)
{
	if (!cfg)
		return;
	memset(cfg, 0, sizeof(*cfg));
	/* sensor_bin: intentionally left "" — required, no default. */
	cfg->width = 1920;
	cfg->height = 1080;
	cfg->fps = 60;
	cfg->gop_s = 2.0;
	cfg->qp_delta = -4;
	cfg->max_ipprop = 0; /* leave the firmware default (unbounded) */
	cfg->min_qp = 0;     /* leave the firmware u32MinQp */
	preset_strcpy(cfg->resilience, sizeof(cfg->resilience), "rally");
	cfg->roi_enabled = true;
	cfg->roi_steps = 2;
	cfg->roi_center = 0.4;
	cfg->ae_fps = 15;
	cfg->awb_fps = 15;
	cfg->snapshot_quality = 80;
}

/* Resilience preset expansion.
 *
 * The 2x2 matrix is:                                              .
 *                  | Best efficiency      | Most resilient        .
 *   ---------------+----------------------+------------------     .
 *   Fast recovery  | racing               | fpv (drone FPV)       .
 *   Slow recovery  | quality              | range                 .
 *
 * Named presets set intra_refresh_*, ref_*, AND gop_size, so picking a
 * preset fully determines all recovery / GOP behaviour.  Only `off`
 * leaves gop_size untouched — that mode is the escape hatch where the
 * user's `gopSize` field drives.
 *
 * Returns 0 on success ("off" or recognised preset), or -1 if the name
 * is unrecognised (a boot failure for mabur — there is no fall back to
 * "off" here, unknown config must not boot).
 */
int venc_cfg_expand_preset(const VencCfg *cfg, VencPresetOut *out)
{
	struct preset {
		const char *name;
		const char *intra;
		uint8_t ref_base;
		uint8_t ref_enhance;
		double gop_sec;           /* 0 = preserve caller's gop_size */
	};
	/* Resilience preset table.
	 *
	 * Stripe-only recovery (no IDR needed) requires the effective
	 * wavefront to fit inside the GOP.  Effective wavefront is
	 *   nominal_wavefront × (ref_enhance + 1)
	 * because the TRAIL_N rewrite drops `ref_enhance` of every
	 * `(ref_enhance + 1)` frames from the decoder's reference list.
	 *
	 * OSD-safe column: presets with `ref_enhance > 0` (SVC-T) leave
	 * persistent chroma artefacts in static high-contrast overlays
	 * (OSD text).  Confirmed by bench testing: even `rally` (1:1
	 * SVC-T) shows green smear over the OSD area that won't clear
	 * via stripes — only via IDR.  Root cause: chroma stays in
	 * skip mode for static-content MBs, and intra-refresh stripes
	 * landing in TRAIL_N frames don't propagate the chroma fix
	 * into the DPB.  SDK exposes no force-intra-MB knob to fix it
	 * (ROI is delta-QP only, doesn't override skip-mode for
	 * zero-residual blocks).
	 *
	 *   preset      nominal   enh  GOP    eff.wave  OSD-safe?
	 *   ─────────────────────────────────────────────────────
	 *   off         off       0    user   -         yes (no refresh)
	 *   rescue      off       0    0.25s  -         yes (IDR-spam, lowest latency)
	 *   quality     off       0    4.0s   -         yes (IDR-based)
	 *   sprint      150ms     0    0.5s   150ms     yes (intra+short GOP)
	 *   racing      150ms     0    2.0s   150ms     yes
	 *   endurance   500ms     0    2.0s   500ms     yes
	 *   patrol      500ms     0    4.0s   500ms     yes
	 *   rally       150ms     1    2.0s   300ms     no  (light refPred)
	 *   range       500ms     4    2.0s   2500ms    no  (heavy refPred)
	 *   fpv        1000ms     4    2.0s   5000ms    no  (heaviest refPred)
	 *
	 * The "ltr" family is handled below the table, not in it: its
	 * enhance ratio is derived from the GOP (which the user owns) and
	 * so cannot be a static column.  See the block above the loop.
	 */
	static const struct preset table[] = {
		{ "off",        "off",      0, 0, 0.0 },   /* gopSize honoured */
		{ "rescue",     "off",      0, 0, 0.25 },  /* IDR-spam, ~35% bitrate to IDRs */
		{ "quality",    "off",      0, 0, 4.0 },
		{ "sprint",     "fast",     0, 0, 0.5 },   /* intra-refresh + aggressive IDR */
		{ "racing",     "fast",     0, 0, 2.0 },
		{ "endurance",  "balanced", 0, 0, 2.0 },
		{ "patrol",     "balanced", 0, 0, 4.0 },
		{ "rally",      "fast",     1, 1, 2.0 },
		{ "range",      "balanced", 1, 4, 2.0 },
		{ "fpv",        "robust",   1, 4, 2.0 },
	};

	const char *want;
	size_t i;

	if (!cfg || !out)
		return -1;

	memset(out, 0, sizeof(*out));
	want = cfg->resilience[0] ? cfg->resilience : "off";

	/* "ltr" / "ltr:<N>" — maximum non-reference density, long GOP.
	 *
	 * Device-measured on Star6E 2026-08-06 (raw-ES NAL census).
	 * MI_VENC_ParamRef_t.u32Enhance is a PERIOD: the encoder emits
	 * exactly one non-referenced frame in every (u32Enhance + 1).
	 * enhance=4 yields `IRRRnRRRRn…` (20 % droppable), enhance=299
	 * yields 0.3 %.  So enhance=1 — `InRnRnRn…`, 50 % droppable — is
	 * the most resilient structure this SoC can express, and it is
	 * what bare "ltr" selects.  "ltr:<N>" pins the period for sweeps;
	 * larger N is strictly LESS resilient.
	 *
	 * A lost non-referenced frame costs exactly one frame.  The other
	 * half of the stream is an ordinary P-chain, so losing one of those
	 * still cascades to the next IDR — there is no long-term-reference
	 * API on Infinity6E to do better (the full SDK's MI_VENC_Set*
	 * surface has no LTR/SmartP/GOP-mode entry point).
	 *
	 * Unlike "rally" (the same 1:1 ratio) this preserves the caller's
	 * gop_size and forces intra-refresh off, so it can be paired with a
	 * long GOP and asymmetric transport FEC.  Stripes landing in
	 * non-referenced frames never enter the DPB, so with half the stream
	 * non-referenced they cost bitrate and repair nothing.
	 */
	if (strncmp(want, "ltr", 3) == 0 && (want[3] == '\0' || want[3] == ':')) {
		unsigned long enh = 1ul;

		if (want[3] == ':') {
			char *end = NULL;

			errno = 0;
			enh = strtoul(want + 4, &end, 10);
			if (errno != 0 || end == want + 4 || *end != '\0' ||
			    enh < 1ul || enh > 255ul)
				return -1;
		}
		preset_strcpy(out->intra_refresh_mode,
			sizeof(out->intra_refresh_mode), "off");
		out->intra_refresh_lines = 0;
		out->intra_refresh_qp = 0;
		out->ref_base = 1;
		out->ref_enhance = (uint8_t)enh;
		out->ref_pred = false;
		return 0;
	}

	for (i = 0; i < sizeof(table) / sizeof(table[0]); ++i) {
		if (strcmp(want, table[i].name) != 0)
			continue;
		preset_strcpy(out->intra_refresh_mode,
			sizeof(out->intra_refresh_mode), table[i].intra);
		out->intra_refresh_lines = 0;
		out->intra_refresh_qp = 0;
		out->ref_base = table[i].ref_base;
		out->ref_enhance = table[i].ref_enhance;
		out->ref_pred = true;
		if (table[i].gop_sec > 0.0) {
			out->gop_s = table[i].gop_sec;
			out->gop_overridden = true;
		}
		return 0;
	}
	return -1;
}

/* See venc_cfg.h: the config loader's sole authority for "is this a known
 * resilience preset". Delegates to venc_cfg_expand_preset() itself rather
 * than re-walking the table, so there is exactly one place that knows the
 * preset names (the table above, plus the "ltr"/"ltr:<N>" special case). */
int venc_cfg_preset_known(const char *name)
{
	VencCfg cfg;
	VencPresetOut out;

	memset(&cfg, 0, sizeof(cfg));
	if (name)
		preset_strcpy(cfg.resilience, sizeof(cfg.resilience), name);
	return venc_cfg_expand_preset(&cfg, &out) == 0;
}
