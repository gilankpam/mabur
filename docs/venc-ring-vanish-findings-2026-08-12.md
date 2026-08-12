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
2. **Enlarge the venc ring** 8→32 slots — ~~promoted to "the" fix after
   the storm~~ **RETIRED same evening by direct measurement** (see the
   bottleneck section below): when the CPU isn't starved, an IDR burst
   never pushes the ring past 2/8 slots, and when the CPU *is* starved a
   deeper ring is pure bufferbloat (the operator called this). Keep only
   the `full_drops` export from the planned rebuild.
3. Optionally: find maburd's transient drain stall — **RESOLVED below**:
   there is no maburd-local stall to find; the hot thread simply stops
   getting scheduled when the SoC is over its CPU envelope.

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

## The bottleneck, measured — it's CPU famine, not ring depth

Attribution experiment, same evening: a read-only 1 kHz sampler on the
ring header's `write_idx`/`read_idx` (third process, mmap PROT_READ —
occupancy = the index difference), run for identical 75 s windows with 4
deliberate IDR bursts, at two bitrate caps:

| | cap 16000 | cap 12000 |
|---|---|---|
| ring empty (0/8) | 22% of samples | 94% |
| ring ≥5 slots | 49% (15% pinned FULL) | 0.0% — never above 2/8 |
| writes into ring | 56.2/s (encoder starved too) | 59.5/s (full rate) |
| pressure episodes | 200+, up to 6.4 s at FULL | zero |
| `enc.vanished` delta | +281 | +1 |

Two readings force the conclusion:

- During full episodes, occupancy sat pinned at exactly 8 while
  `write_idx` froze for up to 1.5 s cumulative — which arithmetically
  means `read_idx` was frozen too. maburd's hot thread was not draining
  for hundreds of ms at a stretch. That is not the ~130 ms IDR-burst
  arithmetic; that is a thread not being scheduled. Box state at the
  time: 16.6% idle, maburd >1 core across its FEC/hot threads, kernel
  video threads eating the rest. (Load average ~14 is MEANINGLESS on this
  SoC — vpe/vif/venc kernel threads idle in D-state and inflate it; read
  idle%.)
- At the 12k cap the four IDR bursts — the worst-case stressor — never
  pushed occupancy past **2 of 8 slots**. With CPU available, maburd
  drains a whole IDR burst in under two frame-times. The 8-slot ring is
  *generous*; depth was never the problem, and a deeper ring under
  sustained overload would only add standing latency before dropping
  anyway.

So the venc-ring vanish class is **drain-bound via CPU famine**: above
~12 Mbps at this op point (mcs5, FEC ov 0.25, rally, 2×A7) the SoC runs
out — encoder, TRAIL_N rewrite, FEC, USB all scale with video bytes
together — and the first victim is the hot thread's scheduling, whose
backlog lands on the 133 ms ring, whose overflow is the silent base-frame
drop. Everything in this doc downstream of "IDR burst fills the ring" is
that mechanism; the burst is just the moment demand peaks.

Hand-wave at the 12k cap: zero smear (operator-confirmed), +14 vanished
during the peak vs +281 — the residual transient class (motion peak
coinciding with an IDR) survives at 12k and would be addressed by
`maxIBytes` (live-settable, caps the I-frame burst at the source) if it
ever matters at the chosen operating point.

## Where this landed (end of 2026-08-12)

The operator stepped the bench back to **master (`6086880`) on both ends
with `bitrate_max_kbps: 10000`** — a CPU-honest operating point (verified:
encoder 9.8 Mbps at the cap, 60.3 fps, 0 gaps/incomplete/resyncs). The
`idr-request` branch (all detection/self-IDR/back-channel machinery, 35
commits) is parked locally, unpushed; master has NO vanish observability
and NO IDR request paths, so any residual vanish heals via rally's
natural 2 s GOP only. Revised follow-up queue: self-IDR redesign (kill
switch + GOP-aware suppression + rate-based guard) before the branch
redeploys; waybeam trip reduced to `full_drops` export (+ optional
`maxIBytes`); maburd bitrate-collapse watchdog for the sibling wedge bug.

## Related documents

- `docs/idr-decoder-blindspot-2026-08-11.md` — the original blind-spot
  investigation (no IRAP in the stream; GDR restores the reference list
  but never repaints).
- `docs/idr-backchannel-acceptance-2026-08-12.md` — the feature acceptance
  whose §4 follow-ups trace the elimination chain that ended here.
- `docs/idr-request-validation-2026-08-11.md` — loss-sim validation of the
  wire-loss trigger (s1 causes smear, s3 cannot).
