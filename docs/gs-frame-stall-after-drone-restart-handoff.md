# Handoff: restarting maburd stalls GS video output for minutes

**Status: FIXED + HARDWARE-VERIFIED 2026-07-25** (candidate fixes 1+2 below,
plus a third layer the rig demanded — see *Link-up anchor*). 3/3 maburd
restarts on the rig now recover in ~4–5 s (= the daemon + radio bring-up
downtime itself; was 52 s / 72 s / 460 s, worst case 18.2 min), `stall=0` new
increments across all three, drops booked per restart +18…+157 (was
~6 300…54 900), and a 30 s rtpsniff tap after reads the healthy signature
(8.21 Mbps, gaps=0, out_of_order=0, ok_fps=59.4, 0% bad).

**The open hypothesis is settled, and it was worse than guessed.** With the
1 s sticky discont window deployed, the first hardware stall's watchdog onset
line read:

```
maburgs: frame stall: no emit for 504 ms with frames arriving
  (next_emit=87218 last_id64=65706 discont_seen=0) -> reset
```

`discont_seen=0` after 60+ flagged frames were sent: the **entire first
second-plus of frames after a restart never reaches the GS** — maburd starts
encoding immediately, but BOOT→RENDEZVOUS→LINKED takes longer than any
fixed-duration flag window, so every flagged frame dies before the link is
up. Not "one unlucky lost frame" — systematic loss of the whole window. Hence
the third fix layer:

- **Link-up anchor** (`RcAgent::take_link_established()` → hot thread
  re-marks the window): the agent latches BOOT/RENDEZVOUS→LINKED transitions
  (deliberately NOT FAILSAFE→LINKED — a routine RF flap re-basing the id
  space would evict in-flight frames for nothing), and maburd's hot loop
  re-marks the discontinuity window when it fires, so the flag rides the
  first ~1 s of frames that can actually land. On the rig this resolves every
  restart through the normal discont re-base path — the watchdog never fires.

What shipped:

- **GS stall watchdog** (`gs/src/frame_stream.cpp`, `FrameStreamCfg::
  stall_reset_ms`, default 500 ms): if frames keep arriving with nothing
  emitted for 500 ms, `FrameStream` logs one rate-limited stderr line with the
  wedged-cursor state (`next_emit`, `last_id64`, `discont_seen` — the
  instrumentation asked for below) and self-`reset()`s. Backstops the whole
  class, whatever the trigger. Counted as `stall=` on the maburgs stats line.
- **Sticky discont** (`drone/src/frame_pipeline.h`, `kDiscontStickyMs` =
  1 s): `kFlagDiscont` rides on every frame for ~1 s after start/reattach
  instead of exactly one frame, so losing frames at radio bring-up cannot lose
  the re-base signal.
- **Run-aware unwrap** (`gs/src/frame_stream.cpp:unwrap_id`): only the first
  flagged frame of a discont run re-bases; the rest delta-track, and
  `Slot::discont` now means "this frame re-based" — so the sticky flag doesn't
  re-base per frame, reorder frames inside the window, or skip gap-holds.
- Tests: `lost_discont_restart_recovers_via_stall_watchdog` (the outage's
  exact signature: backward id64 rebase, eviction stall, watchdog recovery),
  `discont_run_rebases_once_and_holds_order`, the two rewritten
  sticky-window pipeline tests, and the two `link_established` latch tests
  in test_agent.cpp (RCF and DISC link-up paths, FAILSAFE-flap negative). `decode_bodies.py` now masks `FLAG_DISCONT`
  in its byte-exact comparison (which frames carry it is timing-dependent by
  design) while still requiring it on frame 0.

The ~119/s drop-rate mystery (below) is also explained: an evicted frame's
*remaining* fragments re-create its slot (`push_fragment`'s `slots_[key]`),
which never gets a header and is aged out by `poll()` with a second
`++dropped_` — exactly 2 per multi-fragment frame, ≈ 2 × 59.7 fps. The counter
over-books by up to 2× during eviction storms; with the watchdog capping
storms at 500 ms this was documented, not fixed.

One rig gotcha found during verification, unrelated to this bug: the drone
radio can come up deaf/quiet after an S96mabur restart (GS sees SNR collapse
64→14 dB on one card, drone flaps LINKED↔FAILSAFE at ~74 pps, no video). Cure
is another restart — same family as the linkbench RX wedge. Nothing here can
paper over that; it is a radio bring-up issue, not a transport one.

---

Original handoff follows.

**Original status:** root-caused to a specific code path, reproduced 3/3, **not fixed**.
The final link in the chain (why the discontinuity signal is missed) is
hypothesised, not proven — see *What is not proven*. Found 2026-07-25 while
deploying the `frame_ring` stats fix; the stall is **pre-existing** and
unrelated to that change.

**One-line summary:** a maburd restart does not drop the GS's session, so
`FrameStream`'s emit cursor survives into the new session while the drone's
16-bit `frame_id` restarts at 0. Every arriving frame then looks *late*, gets
evicted by the late-arrival pass, and is counted as dropped — the GS emits no
video at all until the frame counter climbs back to the stale cursor. Observed
outages: 52 s, 72 s and 7.7 min. Worst case is a full 16-bit wrap, **18.2 min**.

## Symptom

Video stops at the ground station while the radio link looks perfect. During
the outage:

- `rtp=` and `frames[clean/...]` are **frozen** — nothing is emitted.
- `frames[.../.../drop]` climbs at ~119/s.
- `s1[p=...]` keeps climbing at ~3150/s — packets *are* arriving.
- `rec=` climbs (FEC still repairing), `bc=0 sbf=0 fl=0`, `badfrag=0`,
  `ord[gap=0]`, `udp_fail=0 q_drop=0`, SNR 62–66 dB on both cards.

**No error counter moves.** Every health signal reads clean; only the absence
of output and the rising `drop` reveal it. Worse, `drop` is cumulative, so
after recovery it just looks like old history — the outage is invisible unless
you *difference* the counter. That is exactly how it went unnoticed here for
most of a session.

## Evidence (rig, 2026-07-25)

Three episodes in one 2-hour maburgs session, one per maburd restart that day.
Line numbers index the preserved log (below); the stats line is 1/s.

| # | lines | duration | drops booked | what I did |
|---|---|---|---|---|
| 1 | 5343–5414 | 72 s | ~8 540 | deploy #1 (maburd + S96mabur), restart |
| 2 | 5808–6267 | **460 s** | ~54 900 | deploy #2 (final S96mabur), restart |
| 3 | 6964–7015 | 52 s | ~6 300 | deploy #3 (frame_ring fix), restart |

584 frozen samples total, 69 474 of the session's 69 741 drops. Episodes 1 and
2 **self-healed**; episode 3 ended at the same sample the restart's traffic
resumed. Between episodes the link ran clean at 59.6 fps.

The sample immediately before each episode shows `p+0` — the video packet
counter freezing for exactly one sample, i.e. the drone going away and coming
back. That is the restart, and it is what starts the clock:

```
line 5341  p+0                 <- drone gone (restart)
line 5344  p+2800   drop+...   <- packets back, output dead
line 5807  p+0
line 5809  p+3120   drop+...
line 6963  p+0
line 6965  p+3198   drop+...
```

Inside an outage (rtp and clean pinned, drop climbing, packets flowing):

```
line 7013  rtp=4825682  s1[p=21987473  frames[clean/trunc/drop]=378889/10/69390
line 7014  rtp=4825682  s1[p=21990611  frames[clean/trunc/drop]=378889/10/69510
line 7015  rtp=4825682  s1[p=21993816  frames[clean/trunc/drop]=378889/10/69628
line 7016  rtp=4825736  s1[p=21996968  frames[clean/trunc/drop]=378893/10/69741  <- recovers
```

Recovery books **no** gap: `drop` simply stops rising and emission resumes.
That is the signature of the cursor being *reached from below*, not skipped to.

A 30 s `rtpsniff.py` taken 25 s into episode 3 read `2.67 Mbps, ok_fps=19.3`
with `gaps=0 out_of_order=0 bad=0` — a partial-output window that still
reports zero errors. A tap after recovery read the normal `8.21 Mbps,
ok_fps=59.4`. **A short tap right after a deploy can therefore certify a link
that is actually broken, or broken one that is fine.** Difference the maburgs
counters instead.

## Mechanism

1. `drone/src/frame_pipeline.h:45` — `uint16_t next_frame_id_ = 0`, incremented
   per frame at `frame_pipeline.cpp:26`. It is **per-process**, so a maburd
   restart rebases the wire id to 0. It also wraps every 65536 frames
   (18.2 min at 60 fps).
2. `gs/src/frame_stream.cpp:28` — `unwrap_id` lifts that 16-bit id to a 64-bit
   `id64`. Normal path (`:38`) is a **signed 16-bit delta** against
   `last_id64_`, so it tracks relatively and can step *backward*.
3. `gs/src/frame_stream.cpp:43` `try_emit` begins with a late-arrival eviction
   pass (`:45–56`): any slot with `id64 < next_emit_id64_ && !began` is erased
   with `++dropped_`. This runs **before** head-of-line selection.
4. Therefore, once `next_emit_id64_` sits above the incoming id64 stream, every
   frame is evicted on arrival. `head` is never selected, nothing is emitted,
   `clean_`/`truncated_` never move, and `dropped_` rises at the arrival rate.
   The state persists until the stream's id64 climbs back to the cursor — up to
   65536 frames later.

Two mechanisms exist to prevent exactly this, and **both were bypassed**:

- **`kFlagDiscont`.** maburd sets `discont_ = true` at construction
  (`frame_pipeline.h:46`, comment: "first frame after start"), and the GS
  rebases on it to `next_emit_id64_ + 0x20000` (`frame_stream.cpp:32–35`) —
  deliberately *above* the cursor so it becomes head and re-syncs. Correct by
  design, but **the signal rides on exactly one frame**. If that frame does not
  land, the next frame (no flag) takes the delta path and steps the cursor's
  view backward.
- **`FrameStream::reset()`** (`frame_stream.cpp:126`) clears `have_id_base_`,
  `last_id64_` and `next_emit_id64_` — a full cure. But its only caller is
  `gs/src/main.cpp:253`, gated on `fw != frame_wire`, i.e. a change in
  *session/CAP_FRAME_WIRE* state. **Confirmed not to have fired**: the log's
  `maburgs: video tail -> ...` lines appear at 453/461, 880/914 and 3058/3068
  only — nowhere near the three episodes. A ~2 s daemon restart does not drop
  the GS session (peer_caps is sticky), so the cursor survived it.

## What is proven vs. what is not

**Proven:** the outage signature; that it starts at a maburd restart, 3/3;
that packets keep arriving throughout; that `reset()` did not fire; that
recovery converges on the cursor from below with no gap booking; the code paths
above.

**Not proven:** *why* the discontinuity signal was missed all three times. The
leading hypothesis is that the first frame after restart is systematically lost
— it is transmitted immediately after radio bring-up, before the link has
settled, which would make a "rare" single-frame loss a near-certainty rather
than luck. An alternative is that the flag is set but the frame's fragment 0
never assembles (`have_hdr` false → never unwrapped → aged out). Both are
consistent with the evidence; nothing here distinguishes them. **The drone-side
log for each episode was destroyed by the restart that caused it** (S96mabur
truncates on respawn by design), which is why this needs instrumentation rather
than more log archaeology.

**Unexplained:** drops accrue at ~119/s while frames arrive at ~60/s — very
close to exactly 2 evictions per frame. `++dropped_` at `frame_stream.cpp:52`
is per *slot*, so something is producing two slots per frame (dual-card
delivery? a second key under `(sid<<16)|fseq`?). Worth understanding before
trusting the counter's magnitude; it does not affect the mechanism.

## Reproduction

1. Confirm a clean baseline: difference maburgs' `frames[clean/trunc/drop]`
   over 60 s — expect clean ≈ +60/s, drop +0.
2. `ssh root@192.168.10.152 '/etc/init.d/S96mabur restart'` — do **not** touch
   maburgs.
3. Difference the same counters every 10 s. Expect `clean` and `rtp` to freeze
   while `s1 p` keeps climbing and `drop` rises ~119/s.
4. The outage length varies per restart (52 s / 72 s / 460 s observed) because
   it equals the id64 deficit ÷ 60 fps, and the deficit depends on the low-16
   arithmetic at that instant. Budget up to 18.2 min.

## Suggested instrumentation (to settle the hypothesis)

In the eviction pass, emit a **rate-limited** stderr line the first time a slot
is evicted after a quiet period: `id64`, `next_emit_id64_`, `last_id64_`, the
raw 16-bit `frame_id`, and `discont`. One line per outage onset is enough to
show whether the cursor is stale-high and whether a discont frame was ever
seen. Do not log per eviction — that is 119 lines/s into a RAM-backed /tmp,
which is its own outage (see `drone-log-flood-handoff.md`).

## Candidate fixes

1. **GS watchdog (backstop).** If frames are arriving but nothing has been
   emitted for > N ms, call `reset()`. A receiver should never sit for minutes
   discarding a healthy stream, whatever the cause. This fixes the whole class,
   including causes not yet identified.
2. **Sticky discont (removes the trigger).** Set `kFlagDiscont` on every frame
   for the first ~1 s after start/reattach instead of exactly one, so losing a
   frame at bring-up does not lose the signal.
3. **Session epoch in the frame header.** Replace the one-shot flag with state
   carried on every frame; the receiver compares epochs and resets on change.
   Most robust, biggest wire change.

Recommend 1 + 2 together: 1 is the safety net, 2 removes the trigger.
Regardless of fix, a test should cover "cursor is ahead of the incoming stream"
directly — `tests/` has no coverage of a backward id64 rebase today.

## Preserved evidence

`docs/evidence/gs-frame-stall-2026-07-25.txt` — the three episodes with
context, reduced to the four load-bearing columns (line, `rtp`, `s1 p`,
`frames[clean/trunc/drop]`). The full 7308-line capture (2.8 MB) was not worth
committing; if more of it is needed it can be re-taken from the rig with
`ssh root@10.18.0.1 'cat /tmp/maburgs.log'` — but **capture before restarting
maburgs**, which truncates the log (`S96maburgs` redirects with `>`, the same
pattern as S96mabur). Note the counters are cumulative and never reset without
a restart, so a live capture still contains any episode since maburgs started.
