# Handover: rc_drain decouple (Part C) shows no close_ms improvement on the bench

2026-08-14, end of session. Branch `fade-demote`, PR #29 (open, not merged).
Both daemons deployed to the bench and left running on the new binaries.

**Status: OPEN. The feature is deployed, harmless, and unproven.** Part C's
entire justification is a latency reduction that did not appear in the one
measurement taken. Nothing is broken — video is clean, the hot thread is
fine — but the claim in the PR description and in CLAUDE.md is currently
unsupported by hardware data, and should not be repeated as fact until
someone closes this out.

## The claim

Before: `maburd`'s agent loop drained the queued rate-control frames once
per `link.tick_ms` (100), so op actuation latency was U(0, 100) ms.
Part C makes the loop wake every `link.rc_drain_ms` (new key, default 5)
to drain RCFs, with all per-tick housekeeping moved behind a `TickGate`
deadline so its cadence is bit-for-bit unchanged
(`drone/src/tick_gate.h`, `drone/src/main.cpp` agent loop).

Expected: mean actuation latency drops ~47.5 ms (U(0,100) mean 50 →
U(0,5) mean 2.5). The plan's acceptance criterion was
`link.attrib.close_ms` **median ≤ 30 ms, no tail > ~150 ms across ≥ 8
boundaries**, against a recorded baseline of median ~110 with tails
295/971 (2026-08-14 campaign, pre-branch).

## What was measured

Matched A/B on the bench, same session, same RF conditions, same forced
stimulus (`/etc/init.d/S96maburgs restart` → ladder re-climbs rung 0→5,
producing one boundary per rung). maburd was rolled back to the pre-branch
binary for the control arm and rolled forward again afterwards.

| → rung | old maburd (control) | new maburd (rc_drain_ms=5) |
|--------|---------------------|----------------------------|
| 2      | 84                  | 173                        |
| 3      | 41                  | 43                         |
| 4      | 100                 | 161                        |
| 5      | 75                  | 35                         |
| median | **79.5**            | **102**                    |
| mean   | 75                  | 103                        |
| range  | 41–100 (2.4×)       | 35–173 (4.9×)              |

No improvement. Median and mean both worse, variance roughly doubled.
Target was ≤30 ms; neither arm approaches it.

**This is n=4 per arm from a single climb each, and the within-arm spread
is 2.4–4.9×. The distributions overlap heavily. It is not evidence that
Part C made things worse — it is a failure to demonstrate that Part C made
things better, which is a different and weaker statement.** Do not quote
the "new is slower" reading as a finding; it is under-powered.

## What the metric actually measures (verified, not assumed)

Worth reading before designing the follow-up, because the first instinct —
"close_ms includes the feedback quantum, so ≤30 ms was never reachable" —
**is wrong**, and I checked:

- `close_ms = now_ms - bnd_arm_ms` (`common/src/uep_decoder.cpp:128`),
  armed by `UepDecoder::mark_transition()` (`:25-40`) and closed on the
  first body whose RX PHY rate matches the new commanded MCS (`:125-128`).
- The boundary arms in `gs/src/main.cpp:500-512`, which runs **before**
  `vrx.step()` at `:645` in the same loop body — so the arm lands one tight
  main-loop iteration after the op changed.
- Crucially, in `gs/src/vrx_controller.cpp:65-72`, the `feedback_ms` gate
  is checked *first*, and then `ctrl_.update()` (where `cur_op_` changes)
  runs inside the **same** call that immediately builds and returns the
  RCF. **Op-commit and RCF-emission are simultaneous.** `feedback_ms: 50`
  does not sit between them and is not an additive term in `close_ms`.

So `close_ms` ≈ RCF-send → drone drain → `apply_ladder_op` → drone TX at
the new rate → GS decodes a body at the new rate. **The drone-side drain
that Part C shortens is squarely inside the measured window.** The
expected ~47 ms mean improvement should have been visible. It wasn't.

## The single most valuable next check

**Nobody has verified that the RCF drain is actually running every 5 ms on
the device.** This is the gap I would close first.

What *is* verified: `TickGate` works — the drone's 1 Hz stats line still
prints at 1 Hz rather than 200 Hz, which proves the housekeeping gate
holds its cadence. That says nothing about the drain half.

The drain running at the intended rate is currently an inference from
"the binary contains the code and `rc_drain_ms` defaults to 5". Cheapest
confirmations, roughly in order of effort:

1. Add a wake counter and a drain counter to the drone's 1 Hz stats line
   (`drone/src/main.cpp`). At `rc_drain_ms: 5` the wake counter should
   advance ~200/s; if it advances ~10/s, the loop is not waking as
   intended and the whole feature is inert. This is a two-line change and
   it definitively settles the question.
2. Set `link.rc_drain_ms: 1` and `: 100` explicitly on the device and
   re-run the A/B. If close_ms is identical across a 100× change in drain
   period, the drain is not the binding constraint and Part C cannot help
   this metric — which would be a real, publishable negative result.
3. `tools/bench/ringwatch.c` (branch `bench/loss-sim-v2`) or strace on the
   agent thread.

## Other hypotheses worth testing (not yet examined)

- **The drone TX/FEC pipeline may quantize actuation.** `fec.window: 32`,
  `flush_ms: 25` on the drone. If a newly-applied op only takes effect at
  the next FEC block boundary, that quantization is common to both arms
  and would mask a 47 ms drain improvement without explaining why the new
  arm looked slower.
- **Promotes vs demotes.** Every boundary in this A/B is a *promote* from
  a post-restart climb, and promotes interact with the s3 probe
  (`vrx_controller.cpp:86-92` sets `probe3`/`probe_profile`). The recorded
  ~110 ms baseline population is of unknown composition. A campaign that
  separates promote boundaries from demote boundaries would be more
  honest than pooling them.
- **Rung-dependence.** Both arms show their worst numbers at →2 and →4 and
  their best at →3 and →5, in both arms. That alternation appearing in
  *both* arms suggests a systematic per-rung effect (overhead changes at
  those rungs: ov 0.5→0.5→0.25→0.25) rather than noise, and it is a bigger
  effect than the one being hunted. Worth understanding before adding
  more samples.

## What Part C definitively did NOT break

The real risk of a 5 ms wake on the drone's 2×A7 SoC was starving the hot
thread (this is the SoC where >12 Mbps causes venc-ring vanishes — see
`docs/venc-ring-vanish-findings-2026-08-12.md`). It did not happen:

- `maburd frame_ring: fill=0% (0/8)`, `vanished=0/0`,
  `self_idr_refused=0` throughout.
- `txq=0 txq_drop=0 tx_failed=0`, `hot_beat` advancing ~6840/s steadily.
- AU ring via `ausniff.py`, 30 s samples: **1800 AUs, 0 frame_id gaps,
  0 resyncs, 60.0 fps** — identical before the deploy, after the GS-only
  deploy, after the full deploy, and after the A/B roll-forward.

## Repro recipe

```sh
# 1. sampler on the GS (:8300 must be free — stop the flight recorder first)
ssh root@10.18.0.1 '/etc/init.d/S97flightrec stop'
ssh root@10.18.0.1 'timeout 130 python3 -u -c "
import socket,json,time
s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
s.bind((\"127.0.0.1\",8300)); s.settimeout(6)
seen=None; t0=time.time()
while time.time()-t0 < 120:
    try: d,_=s.recvfrom(65535)
    except Exception: continue
    r=json.loads(d)
    c=(r[\"link\"].get(\"attrib\") or {}).get(\"close_ms\")
    rung=(r[\"link\"][\"ctl\"].get(\"rung\") or {}).get(\"idx\")
    if c is not None and c != seen:
        seen=c; print(\"close_ms=%s rung=%s t=%.1f\" % (c,rung,time.time()-t0))
"' &

# 2. force the climb (one boundary per rung, 0->5)
ssh root@10.18.0.1 '/etc/init.d/S96maburgs restart'

# 3. afterwards, put the recorder back
ssh root@10.18.0.1 '/etc/init.d/S97flightrec start'
```

Ignore the first value printed (t<1 s): it is the carry-over from the
previous boundary, not a new one. `close_ms` only changes when a boundary
closes, so sampling a parked link returns one stale value forever — that
is expected, not a bug.

Swapping maburd arms (rollback binary is on the device):

```sh
ssh root@192.168.10.152 '/etc/init.d/S96mabur stop'
ssh root@192.168.10.152 'mv /usr/bin/maburd /usr/bin/maburd.fade'
ssh root@192.168.10.152 'mv /usr/bin/maburd.pre-fade /usr/bin/maburd'
ssh root@192.168.10.152 '/etc/init.d/S96mabur start'
# ...and the reverse to go back. Confirm with md5sum; the new binary is
# 1fccda526a72fbe306c81aefe9f2d0f9.
```

⚠ The drone rootfs holds **max 2** maburd binaries (5.7 M, ~1.94 M each,
1.7 M free with two present). `mv`, never `cp`. `df` before any scp.

## Bench end-state as left

- **Drone** `root@192.168.10.152`: `/usr/bin/maburd` = new
  (`1fccda526a72fbe306c81aefe9f2d0f9`, branch `fade-demote`), rollback
  `/usr/bin/maburd.pre-fade`. `/etc/mabur.json` **unmodified**
  (`tick_ms: 100`, no `rc_drain_ms` → default 5).
- **GS** `root@10.18.0.1`: `/usr/local/bin/maburgs` = new
  (`767db45d495a4f94531511c66bd2edaa`), rollback
  `/usr/local/bin/maburgs.pre-fade`. `/etc/maburgs.json` **unmodified**
  (no `link.fade` → all defaults, cascade+predict ON; `attrib` absent →
  true; `ctl_log: true`).
- Link parked at mcs5/ov0.25, ausniff clean, ctl log now `ctllog 3`.
- No config edits were needed on either device — both new keys are
  optional with live defaults. Deploy was binary-only in both directions.

⚠ **New boot-blocking validation on an existing key.** This branch bounds
`link.tick_ms` to [1,1000] and adds a `rc_drain_ms <= tick_ms` cross-check
(`drone/src/config.cpp`). The bench drone passes (`tick_ms: 100`). A drone
tuned with `tick_ms` outside that range, or below 5 without an explicit
`rc_drain_ms`, will **fail to boot and crash-loop every 2 s** — the
wrapper does not check exit codes. Check the target's live config before
deploying this branch anywhere else.

## Side finding, unrelated to this branch

**The GS flight recorder was dead when this session started.** `rec8300`
was not running and `/media/dvr/flight-0002.jsonl` had been stale for
~5 hours. I caught it only because a datagram I read lacked
`link.attrib`, which shipped in #28 last week — i.e. I was reading a
recording from a binary two deploys old and nearly mistook it for live
output. Restarted (`/etc/init.d/S97flightrec start`, now writing
`flight-0013.jsonl`, confirmed capturing the new `link.ctl.fade` keys).

**Root cause unknown, so it may die again.** This is the fourth recording
gap in the sequence CLAUDE.md documents as the reason S97flightrec exists
at all. Anyone relying on a flight recording should verify `rec8300` is
actually running *before* the flight, not after. Worth a separate
investigation.

## References

- PR #29, branch `fade-demote`. Part C is commit `1260c36`
  (`feat(drone): decoupled RCF drain — rc_drain_ms wake, tick_ms
  housekeeping gate`) plus `7a84a19` (the `tick_ms` bounds).
- Plan: `docs/superpowers/plans/2026-08-14-fade-demote.md` Task 8 and
  post-merge follow-up item 3 (gitignored).
- Spec: `docs/superpowers/specs/2026-08-14-fade-demote-design.md` §3b
  carries the original Part C evidence (gitignored).
- `common/src/uep_decoder.cpp:25-40,125-128` — boundary arm/close.
- `gs/src/vrx_controller.cpp:65-72` — op-commit and RCF-emission are the
  same call. This is the line that refutes the feedback-quantum theory.
- Parts A and B (GS-side fade regime + predictive trigger) are **not**
  affected by any of this and were verified live: `link.ctl.fade` block
  present and behaving on a steady link (`active: false`, deltas ~0),
  `ctllog 3` S lines carrying `drssi dsnr`, freshness gate visibly
  NaN-ing s1 labels on zero-frame windows. They cannot be exercised
  further without a real fade — that needs the walk-out/attenuation run
  in the plan's follow-ups.
