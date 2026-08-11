# Persistent smear on a clean link: the IDR request's decoder blind spot

Bench, 2026-08-11, immediately after the IDR-request validation campaign
(`docs/idr-request-validation-2026-08-11.md`). The screen showed heavy
blocky smearing that did not go away, while the GS OSD reported a perfectly
healthy link — MCS 5 / FEC 25 %, AIR 50 %, **LOSS 0.1 % → 0.0 %**, 60 FPS,
JIT 10 ms, 15.9 Mbit/s, C0 −56 dBm/29 dB, C1 −58 dBm/31 dB — and **no IDR
was ever requested**.

## TL;DR

Two independent facts produced the symptom:

1. **This stream contains no IRAP at all in normal operation.** The ~2 s
   "refresh" on sid 0 is `VPS + SPS + PPS` plus an ordinary `TRAIL_R`
   P-picture — parameter sets, not a random-access point. So a broken
   decoder reference chain is **permanent**, not a ~2 s concealment window.
2. **The IDR request is open-loop with respect to the decoder.**
   `IdrRequester` fires only on `FrameStream::frame_lost` — a GS *wire-loss*
   event on sid ≤ 2. The decoder was broken; the wire was not. Nothing in
   the system connects those two facts, and there is no periodic IRAP as a
   backstop.

One forced IDR healed it completely and permanently. The feature works; its
trigger is narrower than the failure class it needs to cover.

## Evidence

Instruments added by this investigation (all read the AU ring read-only
from outside the daemon, reusing `ausniff.py`'s seqlock reader):
`tools/bench/aunal.py` (NAL-type / temporal-id census),
`tools/bench/auorder.py` (publication-order check),
`tools/bench/refcorr.py` (correlate decoder complaints against arrivals).

### 1. The stream has no random-access point

`aunal.py --seconds 25`, steady state:

```
sid 0:   5 AUs  nal types {VPS:5, SPS:5, PPS:5, TRAIL_R:45}   temporal_id {0:45}
sid 1: 429 AUs  nal types {TRAIL_R:429}                        temporal_id {0:429}
sid 3: 430 AUs  nal types {TRAIL_N:430}                        temporal_id {0:430}
IRAP NALs seen in window: 0
```

Zero IRAPs, across three separate windows. Everything is `temporal_id 0`;
the SVC-T enhance layer is signalled purely as `TRAIL_N` (non-reference),
not as a temporal sub-layer. This confirms — and sharpens — the comment at
`gs/player/src/main.cpp:905`: sid 0 is a *join/cut point*, not an IRAP. The
comment's expectation that a decoder joining there "should expect
concealment/errors for up to one refresh cycle (~2 s), then clean decode"
holds for a fresh join (§5), but it does **not** hold for a chain broken
mid-session — there is no mechanism to repair one.

### 2. The decoder was losing references continuously

`/tmp/maburplay.log` carried 370+ lines of

```
mpp[415]: h265d: refs: cur_frm 10443 missing ref poc 10442
```

still accruing at **5 events per 25 s** while the link was pristine, always
`missing ref poc = cur − 1`.

### 3. The GS saw essentially nothing

Same session, from the sideport: 20 154 clean AUs, **2 truncated, 4
dropped**, `q_drop 0`, `ring.dropped_oversize 0`, `stall_resets 0`; daemon
stats line `mis=0 badfrag=0`. `link.video.idr_req.episodes = 2`, both
latched and cleared normally (313 ms and 1250 ms). `ausniff` over the same
ring: 2 654 AUs / 45 s, **0 incomplete, 0 frame_id gaps, 0 resyncs**,
59.0 fps.

So ~370 decoder reference losses against 6 GS-visible frame losses.

### 4. Delivery is gap-free and in order (hypothesis refuted)

The obvious suspect was cross-stream reordering at the ring — sid 0/1/3
have different FEC overheads and therefore different repair latencies, so
AUs could in principle be published out of encode order. `auorder.py`
refutes it outright:

```
records seen=1503 in 1 contiguous run
frame_id step histogram: {1: 1502}
inversions: 0    forward holes: 0
```

Every AU, in order, no holes. The GS hands the player a perfect stream.

### 5. One IDR fixes it, permanently

Forced via the drone's own endpoint (`curl http://127.0.0.1/request/idr`,
`waybeam.idr_path`). The IDR appears exactly as designed —
`aunal.py` sees `IDR_W_RADL` on sid 0 with the ring's `IDR` flag set — and
then:

| after | missing-ref events |
|---|---|
| forced IDR, next 150 s (`refcorr.py`) | **0** |
| + maburgs restart, next 30 s | **0** |
| + maburplay restart, next 100 s | **0** |

The chain healed on the single IDR and stayed healed across both restarts.

### 6. What did NOT break it

Both reproducible candidate triggers came back clean, so both are refuted
as *the* cause:

- **maburgs restart** (ring recreate → player flush → re-arm at the next
  sid 0): 0 new missing-refs in 30 s.
- **maburplay restart** (mid-session join on a non-IRAP sid 0 — precisely
  the path `main.cpp:905` warns about): 0 in 100 s. The documented join
  path self-heals in practice.

Also refuted: a parameter-set-refresh correlation. The apparent rate match
between sid 0 arrivals and decoder complaints was an artifact of `aunal.py`
undercounting (a Python poller cannot keep up with a 60 AU/s ring — only
its *NAL-type census* is trustworthy, never its rates). `refcorr.py`, which
does keep up (59.6 AU/s captured), measured sid 0 at a true **0.50/s**
against 0.2/s complaints, and recorded **0 complaints in 150 s** after the
heal. Not correlated.

### 7. Positive control — the mechanism works when it fires

The session running at the time of writing has taken 9 truncated + 17
dropped AUs, latched **4** episodes, healed all 4, and sits at **0**
decoder missing-refs. When the GS sees the loss, the loop closes correctly.

## What remains unproven

The event that originally broke the chain. The old player log showed a
decoder sequence reset mid-session (frame counters restarting at
`cur_frm 2`), so *something* flushed the decoder, but both reproducible
paths recover cleanly. The 35 % s1 loss burst from the validation campaign
is the obvious suspect and cannot be confirmed: restarting maburplay for
test §6 truncated that log and destroyed the record. Copy
`/tmp/maburplay.log` before restarting the player next time.

## Why the trigger is too narrow

`IdrRequester::on_frame_lost()` is edge-triggered on GS wire loss, and
`on_frame_emitted()` clears on any complete IDR-flagged frame. Both live
entirely on the GS. The decoder's reference state is a *different* variable
that nothing observes:

- The GS cannot see a break it did not cause (player flush, MPP internal
  reset, decode watchdog, a repair that was "complete" but wrong).
- The player knows — MPP reports missing references, and the player has its
  own decode watchdog — but has no way to say so.
- There is no periodic IRAP to bound the damage either way.

So the failure is silent, permanent, and looks exactly like a healthy link
in every instrument the GS exports.

## Follow-up, 2026-08-12: why nothing heals, and which stream breaks it

### The encoder's intra refresh is on — and does not repaint pixels

`waybeam` runs a GDR sweep, correctly configured and applied at the
hardware level (`/api/v1/resilience/status`):

```json
"preset":"rally",
"intra":{"mode":"fast","active":true,"mi_supported":true,"apply_ok":true,
         "effective_lines":4,"effective_qp":36},
"refPred":{"active":true,"apply_ok":true,"base":1,"enhance":1,"pred":true},
"gop":{"effective_sec":2.000,"auto":false}
```

So the premise behind `MPP_DEC_SET_DISABLE_ERROR` in `mpp_backend.cpp`
("the intra sweep genuinely heals the picture") is *configured* to hold.
An A/B with `link.idr_req` disabled shows it does not, and separates two
things that were being conflated:

| after a 3 s / 35 % s1 burst | outcome |
|---|---|
| reference **list** (`missing ref` lines) | recovers — a short flurry, then silent for 160 s |
| reference **pixels** (what is on screen) | **never recovers** — smear persists indefinitely |

The GDR sweep re-establishes the DPB but does not repaint. That comment was
inherited from PixelPilot's decoder setup, not measured on this link.

### The recovery that does work is content overwrite, not refresh

Operator observation: covering the lens with a finger and releasing clears
the smear. Measured against the ring, a cover/release produces **0 IRAP
NALs in 90 s** — no IDR is generated (consistent with
`video0.sceneThreshold: 0`, which the config validator treats as disabled).

So nothing in the drone decides "this picture needs refreshing". Corruption
is overwritten only where scene content changes, which makes corruption
lifetime a function of scene motion: immortal on a static bench scene,
washed out quickly in flight. That is why this has never presented as a
permanent smear in the air, and why the bench is the worst case rather than
a representative one. It is also a weak guarantee precisely when it matters
— a hover, a slow approach, a static horizon.

### s1 causes the smear; s3 cannot

Two 5 s bursts at eff=35, one stream each, `idr_req` off so nothing repairs
the picture between them, each phase starting from an explicit IDR:

| stream | injected | missing-ref | trunc | drop | s1 abn | s3 abn | episodes |
|---|---:|---:|---:|---:|---:|---:|---:|
| **s3** | 1793 | **0** | 43 | 83 | 0 | 947 | 0 |
| **s1** | 1395 | **15** | 6 | 25 | 104 | 0 | 2 |

s3 took *more* injected loss and abandoned 947 bodies, costing 43 truncated
and 83 dropped frames — and did **zero** reference damage. s1 took less
loss, dropped a quarter as many frames, and broke the chain 15 times. This
is exactly what the NAL types predict (sid 1 = `TRAIL_R` reference, sid 3 =
`TRAIL_N` non-reference): **enhance-layer loss costs smoothness, never
smear.**

This is the companion to the validation campaign. That run proved s3 loss
does not *falsely* fire the latch; this one proves it *should not*, because
s3 loss cannot break the reference chain at all. The `sid > 2` guard in
`IdrRequester` is now measured, not assumed. Note also that the latch fired
in exactly the phase that caused reference damage (episodes 2 vs 0) — the
trigger is aimed correctly; its problem is only that it cannot see breaks
the GS did not itself observe.

### Kill-switch semantics worth knowing

`link.idr_req: false` gates *transmission* of `RCF_F_IDR_REQ`, not the
latch: `IdrRequester` still latches and still counts episodes (main.cpp
gates on `cfg.link.idr_req && idr_req.want()` at the send site only). So
`link.video.idr_req.episodes` keeps counting with the feature disabled, and
`episodes > 0` with `grants == 0` means "switched off here", not "the drone
is refusing".

## Observability gaps found along the way

- **`--fps-log` is off in production** (`S97maburplay`), so none of the
  player's own counters (`truncated_skipped`, `dropped_enhance_incomplete`,
  `delivered`, `backend_submits`) are visible on a live GS. The entire
  investigation ran on MPP's incidental log spam; had the decoder been less
  chatty there would have been no signal at all.
- **The drone counts grants, not requests** — a request refused by the
  cooldown is counted nowhere (also noted in the validation doc).
- **Only one process can bind the sideport port**, so an ad-hoc capture
  fails with `EADDRINUSE` whenever maburtop is open. `tools/bench/sidesnap.py`
  (added 2026-08-12) sniffs the datagrams off `lo` via AF_PACKET instead of
  binding, so it composes with a running maburtop rather than displacing it.
- Nothing exports decoder health to the sideport, so `maburtop` and the GS
  OSD both showed a green link while the picture was unwatchable.
