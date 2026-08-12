# Frames vanish in the drone's venc ring — invisible by construction

Bench, 2026-08-12. This documents the root cause of the recurring
"persistent smear on a perfectly healthy link", found at the end of the
IDR-back-channel work. It is a standalone finding because it outlives that
feature: **the wire protocol cannot represent this loss class, so every
instrument in the system reads clean while the picture is corrupt.**

## TL;DR

On a quiet, pinned (static mcs5) bench link, encoded frames occasionally
vanish **inside the drone, before maburd reads them** — at ~2–3/min. A
vanished frame gets no `frame_id`, so the wire sequence closes seamlessly
over the hole: no FEC loss, no truncation, no drop counter, no gap. When
the victim is a base-layer picture, later frames reference a picture the
decoder never received; rkvdec's reference *content* silently diverges
(the reference *list* stays valid, so `DISABLE_ERROR` logs nothing) and
the picture smears until an IDR resets the DPB.

The likely stall trigger is an IDR's own ~10× frame-size burst filling the
8-slot (133 ms) venc ring — meaning **a granted IDR can seed the next
smear**. This explains the operator-observed loop ("clears top-to-bottom,
then smears again") and why covering the lens always healed what IDRs
seemingly couldn't: a scene change repaints through ordinary-sized frames —
no burst, no ring lap, no new loss.

## The proof

Capture on the GS AU ring recording the wire `frame_id` (fid) alongside
sid for every AU (25 s, quiet link, sidesnap/ausniff-based script):

```
AUs 1501, ring records contiguous
fid holes (air loss):                          []
consecutive sid3 with CONTIGUOUS fid:          (85075, 85076)
consecutive sid1 with CONTIGUOUS fid:          2 pairs
sid transitions: (1,3): 735, (3,1): 734, (3,0)/(0,3): 14 each
```

The stream strictly alternates base/enhance (sid1/sid0 carry even POCs,
sid3 odd — verified by slice-header parse of a synchronized capture). Two
consecutive sid3 AUs with **adjacent** frame_ids therefore mean a base
frame existed between them — later frames reference it, so the encoder
coded it — but it was never read by maburd and never numbered. Zero fid
holes in the same window: this was not air loss. The `(1,1)` pairs are
enhance frames vanishing the same way (harmless: TRAIL_N, non-referenced).

Decoder-side signature, same day: `h265d: refs: cur_frm N missing ref poc
N-1` bursts with all missing POCs even (= base pictures), while the GS
reported 0 relevant loss and `maburplay`'s decode-health tracer showed
`errors=0, concealed=0` (rkvdec discarded nothing at its input). A 40 s
annex-B dump from the ring decoded **pristine in ffmpeg** while the screen
smeared — the delivered bitstream references pictures it never contains.

## Why nothing can see it

- `VencFrameMeta` (drone/vendor/venc_frame_ring.h) carries **no sequence
  number** — pts, codec, flags, gdr fields only.
- The venc ring's `full_drops` counter is incremented by the **write
  path**, i.e. lives in waybeam's process. `drone/src/main.cpp` documents
  it: "waybeam's own drop count has to come from waybeam". maburd's
  `enc.ring_drops` telemetry counts only consumer-side oversize/bad-slot
  and is structurally zero for this class.
- waybeam's binary exposes **no metric and no log line** for a ring-full
  drop (verified: no matching route, no matching strings).
- maburd assigns `frame_id` at read time, so the sequence it transmits is
  gapless by definition. Downstream, FrameStream/FEC/ring/player all see a
  perfectly healthy stream.

## What was eliminated on the way (each measured, same day)

1. **Transport fragility of large IDR AUs** — disproven: ffmpeg decodes
   the captured stream pristine; grant path is byte-identical to the
   manual `curl /request/idr` that heals.
2. **rkvdec input drops** (`submit_au` BUFFER_FULL exhaustion) — disproven:
   decode-health tracer flat during a live smear.
3. **GS air-loss blind spot** — real but a *different* class: FrameStream's
   gap-skip path never fired `frame_lost` for wholly-lost frames
   ("accepted limitation" in the 2026-08-11 spec). Fixed in `e32ce99`
   (sid inferred from SVC-T alternation, s3-immunity preserved). Covers
   fid-hole losses only; the venc-ring class produces no fid hole.
4. **Ladder churn** (operator's experimental 8-rung mcs6@ov0.15 config) —
   a real aggravator (promote↔demote limit cycle re-damaging the picture)
   but not the cause: smear recurred with the op pinned static mcs5.

## Implications

- **"Wire clean" on the sideport does NOT mean "no frame loss".** Any
  past recording showing smear with clean counters is explained by this.
- The IDR request feature (GS wire-loss trigger + player back-channel)
  works as designed; this loss class is upstream of everything it can see.
- Any future resilience mechanism relying on "the drone sent it" must
  account for the venc ring being lossy and unaccounted.

## Fix directions (status as of 2026-08-12 evening)

1. **maburd pts-jump detection — BUILT + DEPLOYED same day** (commit
   `65c94fd`): period EMA-derived (never hardcoded 60 fps), u32-wrap-safe,
   shed-immune; class inferred from the neighbours' `ENHANCE` flags via
   the strict alternation; exported as `enc.vanished` (split base/enh) on
   telemetry + sideport (`drone.enc.vanished_base/vanished_enh/
   self_idr_refused`) + the 5 s `frame_ring:` stderr line. Base-class
   vanishes latch a self-IDR request through the shared grant cooldown.
   **The re-seed caveat above was prophetic — see the storm section
   below.** The detection/export half is unconditionally valuable; the
   self-IDR half needs redesign.
2. **Enlarge the venc ring** 8→32 slots (533 ms of buffer) — attacks the
   stall margin itself; needs a waybeam rebuild (openipc-builder, see
   waybeam-venc-build notes). After the storm below, this is promoted from
   "structural option" to **the** fix.
3. Optionally: find maburd's transient drain stall (suspect: the FEC+
   inject burst of an IDR frame on the hot path) and bound it.

## Live confirmation — and the self-IDR storm (2026-08-12 evening)

Detection went live and immediately confirmed the mechanism: vanishes
counted at ~2–3/min on a quiet link, several refused as IDR-adjacent
(direct evidence for the burst-seeding theory), drone stderr and sideport
byte-identical.

Then the operator's hand-wave test at the pinned mcs5 / 16 Mbps operating
point closed the loop this doc warned about, at full scale:

- Motion pushed the encoder to its 16 Mbps target → bigger frames →
  longer per-frame drain on the hot path → IDR bursts started overrunning
  the ring reliably instead of occasionally.
- Every base-class vanish raised a self-IDR; the 500 ms guard assumed
  vanishes cluster tight behind the IDR *read*, but under sustained
  pressure they spread out — so grants fired at cooldown rate (1/s).
- At ~1.5 IDRs/s (grants + rally's natural 2 s GOP) the stream became
  I-frame-dominated: the encoder inflated to a **sustained 17.1 Mbps**
  (CBR overshoot), P-frames got crushed (stutter), and every burst seeded
  the next vanish. Steady state: **~40 vanishes/min, 124 grants
  (~100+ self-requested), rolling smear**.
- Decisive negative: `link.air_pct` was **45%** throughout — the RF link
  had headroom. This loss class is drain-bound, not airtime-bound; no
  MCS/overhead change would have helped.

**Mitigation (deployed):** `waybeam.idr_cooldown_ms: 1000 → 5000` in
`/etc/mabur.json` (backup `.pre-idrcool`). One shared cooldown gates all
grant paths (GS latch, player back-channel, self), so this caps total
request-driven IDR pressure at 1/5 s while rally's 2 s natural GOP keeps
smear lifetime bounded. Result within 90 s: zero new grants, zero new
vanishes, encoder back to ~11 Mbps scene-bound.

**Self-IDR redesign notes (queued):** needs a drone-side kill switch
(the GS latch has `link.idr_req`; self has none — an oversight); should
self-suppress entirely when the natural GOP is short (waybeam's `rally`
resilience preset pins GOP at 2.0 s by design — see
`../waybeam_venc/src/venc_config.c` preset table — so a natural IDR is
never more than 2 s away and a self-request buys ≤1 s at real burst
cost); the guard should key on vanish *rate*, not proximity to the last
IDR read.

Same evening, distinct bug, same symptom neighborhood:
`docs/waybeam-bitrate-wedge-2026-08-12.md` — the encoder can be left at
maburd's 1400 kbps boot-floor bitrate after link-up (<1 Mbps video while
everything reads healthy).

## Related documents

- `docs/idr-decoder-blindspot-2026-08-11.md` — the original blind-spot
  investigation (no IRAP in the stream; GDR restores the reference list
  but never repaints).
- `docs/idr-backchannel-acceptance-2026-08-12.md` — the feature acceptance
  whose §4 follow-ups trace the elimination chain that ended here.
- `docs/idr-request-validation-2026-08-11.md` — loss-sim validation of the
  wire-loss trigger (s1 causes smear, s3 cannot).
