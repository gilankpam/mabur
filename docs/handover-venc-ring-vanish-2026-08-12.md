# Handover: venc-ring frame vanish — next session's work

2026-08-12, end of session. UNTRACKED on purpose (working handover, not a
finding — the findings are committed, see References). Branch `idr-request`,
all of today's commits local/unpushed.

> **UPDATE, same day evening:** Action 1 is BUILT + DEPLOYED (`65c94fd`)
> and the findings doc carries the full follow-through: detection works,
> but the self-IDR half caused an IDR storm under load (the §1 re-seed
> hazard, realized — guard too narrow, 124 grants, rolling smear) —
> mitigated with `idr_cooldown_ms: 5000` (drone config, backup
> `.pre-idrcool`). **LATER SAME NIGHT, ring 8→32 RETIRED**: ringwatch
> attribution proved the class is drone CPU famine, not ring depth (at a
> 12 Mbps cap the ring never passed 2/8 even under IDR bursts) — see the
> committed findings doc's bottleneck section. Bench end-state: **master
> `6086880` on both ends, `bitrate_max_kbps: 10000`**, this whole branch
> parked (rollbacks: drone `maburd.vanish`, GS `*.idrbranch`; the
> `.pre-idrcool`/`.pre-12kcap` config backups contain `idr_cooldown_ms`
> which master REJECTS). Action 2 shrinks to full_drops export (+
> optional maxIBytes). Self-IDR needs its redesign (kill switch,
> GOP-aware suppression) before this branch ever redeploys.
> Also found+documented the same evening: the 1400-boot-floor bitrate
> wedge (`docs/waybeam-bitrate-wedge-2026-08-12.md`). Acceptance item (c)
> — 5 cooldown-spaced manual IDRs, clean end — PASSED (pre-step-back).

## The bug, in one paragraph

Encoded frames occasionally vanish **inside the drone, between waybeam's
encoder and maburd's ring read** (~2–3/min on a quiet bench link). A
vanished frame never gets a `frame_id`, so the wire sequence closes
seamlessly over the hole — no FEC loss, no truncation, no drop counter, no
gap, anywhere. When the victim is a base-layer picture (sid1/sid0 class,
even POCs), every later frame references a picture the decoder never
received; rkvdec's reference *content* silently diverges (reference *list*
stays valid → `DISABLE_ERROR` logs nothing) and the picture smears until an
IDR resets the DPB. The likely stall trigger is the IDR's own ~10×
frame-size burst filling the 8-slot (133 ms) venc ring — so **a granted IDR
can seed the next smear**, which is why IDRs looked flaky all day and a
scene change (finger over the lens; ordinary-sized frames, no burst) always
healed.

Proof (fid-instrumented AU-ring capture, 25 s, quiet pinned link): zero
`frame_id` holes (no air loss) but a base frame between two contiguous-fid
sid3 AUs — encoded and referenced, never numbered. Plus 2× `(1,1)`
adjacencies = enhance frames vanishing the same way (harmless, TRAIL_N).

## Ownership framing (matters for the fix)

- **The interface design gap is mabur's.** `drone/vendor/venc_frame_ring.h`
  is our contract (frame-shm ingest, PR #1); waybeam implements the
  producer side. No sequence number in `VencFrameMeta`, no cross-process
  drop counter, no back-pressure — even a well-behaved producer dropping
  on a full ring is undetectable *by design*.
- **The drop executes in waybeam**, which is arguably correct behaviour
  (drop-new beats blocking the camera pipeline). Its real sin: silence —
  no metric, no log line (verified: no route, no strings).
- **The likely stall trigger is maburd's** (FEC + injection burst of an
  IDR frame on the hot path) — plausible, NOT proven.
- NOT yet confirmed: ring-full drop-new vs a deeper drop inside waybeam's
  encoder pipeline (`waiting for encoder data...` appears in its log).
  Distinguishing requires waybeam's own `full_drops` counter exposed.

## Actions, in order

### 1. maburd pts-jump detection (no waybeam change needed) — do this first

The ring meta already carries `pts` (µs, 32-bit truncated). At 60 fps,
consecutive reads step ~16 667 µs; a ~33 ms jump = a frame vanished
upstream. In maburd's ring reader (`drone/src/frame_source.cpp` /
`pipe` ingest in `drone/src/main.cpp`):

- Detect: `pts_delta > 1.5 * frame_period` (derive period from observed
  deltas, don't hardcode 60 fps; beware the 32-bit pts wrap).
- Classify: infer the vanished frame's class from the *neighbours'* real
  `VENC_FRAME_FLAG_ENHANCE` flags (strict base/enhance alternation,
  capture-proven) — not parity guesswork.
- Export: NEW telemetry counter (e.g. `enc.vanished` / split base/enh).
  `Telem.idr_grants`-style u16 saturating; sideport export additive under
  `v: 1`. Do NOT reuse `enc.ring_drops` (documented consumer-side-only
  semantics).
- Act: on a base-class vanish, self-request an IDR via the existing
  `WaybeamClient::request_idr()` + the existing cooldown
  (`RcAgent::grant_idr` path or a sibling — keep ONE shared cooldown so
  GS-requested and self-requested IDRs dedupe against each other).
- ⚠ **Re-seed loop hazard**: the IDR burst may itself cause the next
  vanish → detect → IDR → … at cooldown rate, forever. Mitigate: don't
  self-request while the ring still holds a recent IDR frame / within
  N ms of the last IDR emission; and count refused self-requests so the
  loop is visible if it happens anyway.
- Host test: fake ring meta sequence with a pts hole → counter + class +
  request fired; wrap-around case; no-fire on shed (sheds don't leave pts
  holes? VERIFY — shed happens in maburd AFTER read, so read-side pts is
  continuous; state this in the test).

### 2. waybeam rebuild (structural; can be parallel or later)

Via openipc-builder (see `waybeam-venc-build` memory: FHS shell.nix,
pipe-to-stdin gotcha; last bump 9c8d83c v0.51.1):

- Enlarge the venc ring 8 → 32 slots (533 ms buffer). waybeam creates the
  ring ("Created /mabur_f: 8 slots x 384 KB"); find the constant in its
  source. Check drone RAM headroom (32 × 384 KB = 12 MB shm).
- Expose `full_drops` (the write path already counts it) in waybeam's log
  and/or an API/metrics route — turns this invisible class into a counted
  one permanently, and settles ring-full vs deeper-internal.

### 3. Verification (bench)

- Repro instrument exists: the fid/sid capture script (also
  `tools/bench/sidesnap.py`, `aunal.py`, `refcorr.py`, committed). A
  `(3,3)`-adjacent-fid event rate before vs after is the metric.
- Acceptance: (a) with detection deployed, `enc.vanished` counts events
  and base-class ones produce a self-IDR + heal — smear lifetime bounded
  by cooldown; (b) soak: no self-IDR churn on a healthy link (the re-seed
  loop guard); (c) the old worst case — fire 5 manual IDRs back-to-back
  (cooldown-spaced), screen must end clean.
- Then restore the operator's 8-rung ladder (`/etc/maburgs.json.user-8rung`)
  as a stress test: its promote/demote churn was a great damage generator.

## Current bench state (as left tonight)

| thing | state |
|---|---|
| GS op | **pinned `static_mcs: 5`** in /etc/maburgs.json; operator's 8-rung ladder saved at `/etc/maburgs.json.user-8rung` |
| maburgs | gap-skip-fix + forensic `frame-lost:` log build (branch @ head); rollbacks `.fb` (pre-forensic feedback build), `.pre-fb` (pre-feature) |
| maburplay | feedback + decode-health tracer build; rollbacks `.fb`, `.pre-fb` |
| maburtop | new build with the player/idr cells; rollback `.pre-fb` |
| drone maburd | UNCHANGED all day (`/usr/bin/maburd`, rollback `.pre-idr`) |
| trace persister | `tail -F /tmp/maburplay.log >> /media/dvr/maburplay-trace.log` running (survives respawn truncation) |
| loss-sim rig | GS binary `maburgs.losssim` + init `.losssim` parked (pre-feedback build — rebuild from branch before reusing) |

## Gotchas for whoever picks this up

- **A curl IDR can seed its own smear** (the bug!). Don't use "fire IDR →
  clean" as proof of health; check missing-ref/`frame-lost:` for ~5 s after.
- maburtop owns UDP :8300 — use `python3 /root/sidesnap.py` (AF_PACKET,
  composes) for sideport reads; plain socat gets EADDRINUSE.
- Every maburgs restart makes the player flush+join at a non-IRAP →
  missing-ref burst + possible smear until the next IDR. Reset with a curl
  AFTER restarts, then treat the next ~5 s as contaminated.
- Respawn loops truncate /tmp logs; the DVR trace file is the durable copy.
- `scp -O`, and stop a daemon before overwriting its binary (Text file busy).
- `fl=` in the maburgs stats line is a FEC-layer stat, NOT the frame_lost
  callback count. Burned once already.
- The forensic `frame-lost:` log and the decode-health tracer are marked
  remove-after-clean-soak; keep them until the fix lands.

## Branch / process state

- `idr-request` @ `b4ed10f`, ~15 commits ahead of origin, unpushed. Merge/
  PR decision pending (user's call). The back-channel feature itself is
  accepted (3/3 hardware tests + reviews); the SDD ledger with parked
  minors lives at `.superpowers/sdd/2026-08-12-decoder-idr-backchannel/`.
- Deferred/open (beyond this bug): `--fps-log` off in production so player
  counters are invisible (tracer is a stopgap); drone counts grants not
  requests; `--decode-only`'s dead `"concealed"` field; why the GDR sweep
  restored the reference list but not pixels in the morning experiment
  (possibly continuous re-seeding by this same bug — unverified).

## References (committed)

- `docs/venc-ring-vanish-findings-2026-08-12.md` — the finding + proof
- `docs/idr-backchannel-acceptance-2026-08-12.md` — feature acceptance +
  the elimination chain (§4 follow-ups)
- `docs/idr-decoder-blindspot-2026-08-11.md` — the original blind spot
- CLAUDE.md ⚠ "wire clean ≠ no frame loss" caveat (top of the debugging
  section)
