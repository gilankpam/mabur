# Player→GS IDR back-channel: hardware acceptance (bench, 2026-08-12)

Acceptance for the back-channel that lets maburplay tell maburgs it has
broken its own decoder reference chain (spec
`docs/superpowers/specs/2026-08-12-decoder-idr-backchannel-design.md`,
gitignored; background `docs/idr-decoder-blindspot-2026-08-11.md`).

Deployed: `maburgs` + `maburplay` + `maburtop` built from `8d3191e`.
Rollbacks on the GS: `/usr/local/bin/{maburgs,maburplay}.pre-fb`,
`/usr/bin/maburtop.pre-fb`. Drone unchanged (no drone code in this work).

Both sides paired on the default port 8303 with **no config change** —
`link.player_fb_port` and `feedback.gs_port` both default there, and missing
keys take struct defaults, so the shipped configs kept working untouched.

## Results

| # | test | verdict |
|---|---|---|
| 1 | player restart comes up clean | **PASS** |
| 2 | flush recovery (maburgs restart) | **PASS (objective); visual unconfirmed** |
| 3 | no steady-state IDR churn over 5 min | **PASS** |
| — | ausniff AU-ring regression gate | **PASS** |

### 1. Player restart — the loop closes in 137 ms

A player-only restart rejoins mid-session at a sid 0 that is provably never
an IRAP. The full chain fired:

```
maburplay: idr-fb: set (join)            <- player latched on its own join
idr-req:   set (player join)             <- GS raised the NEW trigger
maburplay: idr-fb: cleared after 137 ms  <- drone granted, IDR decoded
```

Operator confirmed the picture visually clean afterwards. On the build
running minutes earlier the same restart left a smear that only cleared by
covering the lens.

Sideport, immediately after:

```json
"idr_req": {"episodes": 3, "episodes_player": 1, "pending": false},
"player":  {"idr": false, "age_ms": 129, "joins": 1, "reason": "join",
            "malformed": 0, "flushes": 0, "watchdogs": 0}
```

`episodes_player` is correctly separate from the wire-loss `episodes`.

### 2. Flush recovery — 49 ms, and the cooldown absorbs the duplicate

A maburgs restart recreates the AU ring, so the player flushes:

```
maburplay: idr-fb: set (flush)
idr-req:   set (player flush)   x2
maburplay: idr-fb: cleared after 49 ms
```

**Two requests, one grant** (`idr_grants` 9 → 10). This is the level
reconciliation behaving exactly as designed and is worth recording because
it looks alarming in a log: the GS raises, the drone grants, the GS emits
the IDR and *its* latch clears — but the player has not fed that IDR yet, so
it is still asserting, and reconciliation raises once more. The drone's 1 s
cooldown refuses the duplicate. Net cost: zero extra IDRs. This is also why
`episodes_player` counts *requests made*, not player episodes.

Visual state after this test was not confirmed by the operator (see §4).

### 3. No churn — the runaway guard

Five minutes of quiet link:

```
idr_grants       9 -> 9   delta 0
episodes_player  1 -> 1
player           {"idr": false, "age_ms": 284, "malformed": 0}
```

`age_ms: 284` is as important as the zeros: the heartbeat was flowing, so
"no churn" means correctly quiet, not a dead channel. This is the guard
against a stuck latch quietly costing one IDR per second forever.

### ausniff (external gate)

`2684 AUs / 45 s, 0 incomplete, 0 frame_id_gaps, 0 resyncs, 59.7 fps.`
Gating a maburgs change on the sideport would be circular; this is the
outside check and it is clean.

## 4. OPEN — the granted IDR does not always repair the picture

Operator observation, and the most important result here: with the feature
running, a smear now visibly *attempts* to clear about once a second
(previously it just sat there), the clear sweeps top-to-bottom over roughly
500 ms, **but the picture stays smeared** — while a manual
`GET http://<drone>/request/idr` clears it outright.

What is established:

- **The granted path is not the problem.** `RcAgent::grant_idr()` →
  `WaybeamClient::request_idr()` → `GET /request/idr` is byte-for-byte the
  same endpoint and method as the manual curl, and the drone reports
  `waybeam_failures=0` — every grant call succeeds.
- **Real IDRs are produced.** A ring census during an induced episode saw
  11 `IDR_W_RADL` NALs in 40 s, i.e. the drone is genuinely emitting IRAPs
  on request.
- **Some of them never reach the decoder.** The same census saw
  **7 INCOMPLETE sid 0 AUs**. maburplay drops an incomplete AU whole
  (`main.cpp`, "never feed a truncated AU to the decoder" — a truncated
  slice wedges rkvdec2), so a partially-delivered IDR is wasted entirely,
  the player keeps asserting, and the request repeats at the cooldown rate.
  That cadence matches the operator's "tries every second" exactly.

What is NOT established, and should not be assumed:

- **Why** those sid 0 AUs truncate. At the time of measurement the link was
  healthy — `s0 abandoned=0` (and s0 carries the heaviest protection,
  `ov=1.0`), `txq drops=0`, air 50 %, 13 truncated out of 47 469 clean. An
  IDR AU is far larger than a P frame, so the leading hypothesis is that it
  is more likely to hit the 50 ms reassembly deadline
  (`video.frame_gap_timeout_ms`) before its tail arrives — but that is a
  hypothesis, not a measurement.
- **The top-to-bottom sweep.** If the player drops incomplete AUs whole,
  a partial repaint should not be visible at all. That visual is
  unexplained; candidates are waybeam's always-on GDR sweep
  (`intraRefresh mode=fast, 4 lines/P`) repainting between attempts, or a
  complete IDR landing and being re-broken immediately. Not resolved.

**This is not a defect in the back-channel.** The back-channel's job is to
notice the break and ask; it does that correctly and stops when repaired.
The gap is downstream: a large IDR AU is exactly the payload least likely
to survive the transport, so the only deterministic repair in the system is
least reliable when it is most needed.

Next steps (not this work):

1. Instrument which sid 0 AUs truncate and why — deadline vs FEC vs size.
   `video.frame_gap_timeout_ms` (50 ms) is the first suspect for a large AU.
2. Consider whether the player can use a partially-delivered IDR at all, or
   whether the drone should split/pace an IDR so it survives the transport.
3. Re-run §2's visual and this section once (1) is understood.

## Deployment state

GS runs the new build; rollbacks listed above. `/etc/maburgs.json` and
`/etc/maburplay.json` are untouched — the feature is on by default on both
sides.
