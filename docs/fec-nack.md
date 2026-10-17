# Software NACK: base-layer selective repeat on top of the FEC

Spec: `docs/superpowers/specs/2026-10-05-fec-nack-design.md` (gitignored;
this page is the committed, as-built record). Spike:
`docs/findings/2026-10-05-fec-nack-spike.md`; as-built bench gate:
`docs/findings/2026-10-06-fec-nack-bench.md`. RC_VERSION 15.

## What it is

The GS asks the drone to re-send base-layer (sid 0) FEC **source** symbols
the sliding-window decoder could not recover, inside the time the GS
already waits before it truncates a frame. FEC overhead, the rung table and
the two-card receive diversity are untouched: the NACK only removes the
truncations and base smears that happen while the ladder is still deciding.
It is **option A, repair only**: a symbol a retransmit filled still counts
as loss for every ladder input, so the ladder demotes exactly as it would
without the NACK. Base layer only; the wire carries a `sid` byte so enh can
follow, but the drone refuses any `sid != 0` today.

**Scope: `maburgs` and the web GS in GS mode.** The
browser GS (`web/`, `docs/web-gs.md` "Software NACK") runs the same
`NackTracker` over the same decoder, FrameStream and ladder, sent directly
through its one card; its Config form's "Retransmit (NACK)" switch is the
`[link.nack] enable` overlay (default on). A spotter never sends one: it
has no tracker and no send path by construction. There is no capability
bit: a GS without `[link.nack] enable = true` simply never asks.

## Wire

**`T_NACK` = 7** (GS → drone, `common/include/mabur/rc_proto.h`,
`pack_nack`/`parse_nack` in `common/src/rc_proto.cpp`):

```
magic u16 | ver u8 | type u8 | flags u8 | counter u32 | sid u8 | n u8 |
n × (first_seq u32, bitmap u32) | tag u64 | crc u16
```

`kNackFixedLen` = 11 (through the `n` byte), `kNackEntryLen` = 8,
`n` ∈ [1, `kMaxNackEntries` = 4], so one frame names at most 128 wire seqs.
Bit *i* of a run's bitmap requests `first_seq + i`; bit 0 is always set on
the wire. `flags` bit 0 = `kNackFlagRepeat`: at least one seq inside is on
its second try, and the drone answers the whole frame twice. The tag is the
link-pairing SipHash (`rc::verify_control` finds it at
`kNackFixedLen + n × 8`) with ctx `{vrx_nonce, vtx_nonce, seq32 = counter}`
(`docs/link-pairing.md`).

**SBI retransmit mark.** `kSbiRetxMark` = `0x80` in the SBI header's
`stream_id` byte (`common/include/mabur/sbi.h`). `sbi_peek_stream_id` and
the per-layer decoder read the low 7 bits (`kSbiStreamIdMask`), so a retx
body routes as plain sid 0; `sbi_unpack` exposes `retx`, and the decoder's
accounting and the latency anchor read it. The mark needs no other header
field and no `SBI_VER` change.

**Telem** (+6 bytes, `TELEM_LEN` 48 → 54): `u16 nack_rx, retx_syms,
retx_refused`, per Telem period, saturating:

- `nack_rx` — requests that passed every check (below).
- `retx_syms` — **distinct** source symbols answered this period. A
  repeat-flagged (doubled) answer counts each symbol once, so symbols put
  on air = `retx_syms` + the ones of them answered under the repeat flag.
- `retx_refused` — symbols the air bucket refused. A refused symbol is
  never queued; nothing waits for tokens.

**RC_VERSION 15** carries T_NACK's layout and the 54-byte Telem; a pair on
different RC_VERSIONs has no control link ("Deploy" below).

## GS: NackTracker

`common/include/mabur/nack_tracker.h` + `common/src/nack_tracker.cpp`; the
header comment is the contract. Core-thread only, like the decoder it
reads; `gs/src/main.cpp` polls it once per core-loop iteration right after
the radio drain, sid 0 only, and only while `[link.nack] enable`.

**Inputs** (`NackInputs`): the decoder's erasure set
(`SwDecoder::missing_sources(lookback)`: unknown seqs behind newest, above
the live floor, ascending); `SwDecoder::source_state(seq)` (Unknown /
Direct / Recovered / Retx / BelowFloor); the FrameStream tail view of the
newest unfinished sid-0 frame (`count`, `max_idx`, `seq_at_max`,
`last_progress_ms`); the ladder's own demote util (`vrx.ctl().util()`)
against `link.down_util`; and the per-symbol deadline
`FrameStream::gap_ms(0)`.

**Triggers.**

- *Gap*: a seq in the erasure set is admitted with `t0` = the first poll
  that saw it.
- *Tail*: when the newest sid-0 frame's header says `count` fragments but
  only up to `max_idx` arrived, and `now ≥ last_progress_ms + settle`, the
  seqs `seq_at_max + 1 … seq_at_max + (count − max_idx − 1)` that the
  decoder still reports Unknown are admitted with `t0 = last_progress_ms`.
  It catches a frame whose tail never arrived, which the erasure set cannot
  see (nothing newer advanced past it). The seq arithmetic holds only while
  one sid-0 fragment == one source symbol, true by construction today
  (fragments are cut to `max_packet_size − Fragmenter::kHdrLen`).

**Adaptive settle.** A seq is first requested once `now ≥ t0 + settle`.
`settle` = the maximum natural lateness (how long a never-requested seq
stayed missing before its own copy arrived) over a sliding 10 s window,
+ 2 ms, clamped to **[4, 24] ms**; seeded at 12 ms until the window holds
50 samples. When the window later drops below 50 samples, settle keeps its
last value (no reseed). There is no config key. The clamp exists because the spike's
natural-lateness max read 12 ms under loss-sim but 30–37 ms under the
jammer (carrier-sense deferral reorders more); unclamped it would spend most
of the gap window waiting. A seq that arrives by itself after it was
requested is a `late_fill` and does not feed the window. `clear()` keeps the
estimate (it is a property of the link, not of the session).

**Repeat.** A requested seq still missing after `repeat_ms` (16) is asked
again, up to `max_tries` (2). A frame carrying any second-try seq sets the
repeat flag. `max_tries = 0` is observe-only: lateness stats, no sends.

**Minimum lead.** A request, first try or
repeat, is withheld when `now + min_lead_ms` (12) is past the entry's
deadline: the answer could not land in time, and in a demote cascade such
deadline-doomed seqs were packed first (ascending) and ate the drone's air
bucket ahead of seqs that could still have been filled. The entry is dead
on the spot and counts `lead_skipped` (per seq); it books
`dropped_deadline` later only if it had been requested before.

**Deadline.** An entry still Unknown `gap_ms(0)` after `t0` is **dead**:
never requested again. `gap_ms(0)` is the rate-aware sid-0 frame gap
timeout (`GapTimeoutPolicy`: `clamp(w / seq_rate + 15 ms,
video.frame_gap_timeout_ms, video.frame_gap_timeout_max_ms)`, 50..100 ms
as shipped in `gs/bundle/maburgs.default.toml`; the code default for the
max is 150). A dead entry stays tracked until the decoder's state for it is
terminal, so the erasure set cannot re-admit it.

**Stop rule.** While util ≥ `down_util`, a poll that has due entries
counts one `suppressed` and marks those entries dead. There is no catch-up
burst when util drops. Admission continues while stopped, so entries age
and resolve normally.

**Packing and send.** Due seqs, ascending, go into ≤ 4 runs of 32; what
does not fit stays due for the next poll. At most one frame per poll, with
`counter` = the tracker's counter (1 on each new vtx nonce, see below). It
is sent **directly** through `send_control_frame` on the selected TX card,
mid-burst, gated like every control frame by `ChannelCore::may_send` and
the calibration radio-silence gate. Never through the RcfSlotter: in the
spike the slotted send filled at p50 44 ms, by which time the frame had
usually been truncated.

**Session clears.** `nack.clear()` drops the tracked entries every poll
while there is no session nonce or the peer is not on the frame wire, and on
every `frame_wire` edge, together with the decoder continuity reset. It
keeps the counter: the drone's `accept_nack_counter` holds the last counter
for as long as the vtx nonce is unchanged, and a GS that drops to
BEACONING after `link_lost_ms` of video silence re-enters SESSION on the
first video body under that same nonce (`VrxRendezvous::feed_video`, no
DISC). The counter restarts (`NackTracker::restart_counter`, next NACK is
counter 1) only when `session_ctx().vtx_nonce` turns non-zero and differs
from the last one seen (`nack_vtx_seen` in `gs/src/main.cpp`) — the same
point the drone starts its own count over.

**Config** (`gs/src/config.cpp`, `gs/bundle/maburgs.default.toml`):

```toml
[link.nack]          # absent = off
enable    = false
lookback  = 256      # symbols behind newest the erasure view scans; 8..4096, must be < fec.seq_horizon
repeat_ms = 16       # 1..1000
max_tries = 2        # 0..16; 0 = observe only
min_lead_ms = 12     # 1..100; withhold a request this close to the deadline
```

## Drone: ring, handler, bucket

**RetxRing** (`common/include/mabur/retx_ring.h`). Fixed slots of one
sealed sid-0 envelope each (`kSwHeaderLen` 14 + `fec.symbol_size[0]`),
indexed by `seq & (slots − 1)`, preallocated at startup. Size
(`RetxRing::slots_for`):

```
slots = next pow2 ≥ bitrate_max_kbps × 1000/8 × 0.6 × ring_ms/1000 / symbol_size + 1
```

At the bundle's `bitrate_max_kbps` 24000, `ring_ms` 150 and symbol size 332
that is 814 → **1024 slots × 346 B ≈ 354 kB**. The hot thread fills it from
the UepEncoder source tap (base layer only). Seqlock-style: `put()` is one
memcpy plus three atomic stores (state = 1, seq, state = 2), lock-free and
allocation-free; `get()` rejects a slot that is mid-write or was
overwritten under the reader (acquire gate on the seq, an acquire fence
after the copy, an acquire recheck of state). A rejected or stale slot is a
miss.

**Handler** (`nack_hook` in `drone/src/main.cpp`, on the **RX thread**;
`rx_callback` routes `T_NACK` there instead of the agent queue):

1. `RcAgent::check_nack`: `parse_nack`, `sid == 0`,
   `RcAgent::verify_session_tagged(…, counter)` against the published
   session, `RcAgent::accept_nack_counter(counter, session)`. The counter is
   one atomic word `(vtx_nonce << 32) | last`: accept iff `counter > last`
   under the current vtx nonce; a new nonce starts again from 0. Every
   failure counts `nack_bad` and returns, but only a tag or counter failure
   raises `auth_reject` (Telem flags bit1). A parse failure (`rx_callback`
   routes frames regardless of `crc_err`, so this is mostly radio
   corruption) or `sid != 0` is not an auth failure and raises nothing,
   like the RCF path's silent parse drop.
2. `nack_rx++`; refill the bucket.
3. For each requested seq: ring `get` (miss → skip), bucket `take(cost)`
   (refused → `retx_refused++`, skip), `retx_syms++`, pack into a fresh SBI
   body with the sid-0 geometry and the retx mark.
4. `TxQueue::push_front` every body (pushed back-to-front, so they leave in
   request order ahead of queued video), each booked on the air clock; with
   the repeat flag the whole answer is pushed twice.

**Bucket** (`drone/src/nack_bucket.h`). Tokens are symbols. Depth is
`nack.burst_ms` (20) of air at the current op's delivered sid-0 rate,
re-set at every verified NACK (`TokenBucket::depth_for`; growing it only
raises the ceiling, shrinking it clamps what is held): about 75 symbols at
mcs0/40, 146 at mcs1/40, 433 at mcs4/40 with the bundle's efficiency
tables. A fixed symbol depth would be a few ms of air at the top rung and
tens of ms at the bottom one, refusing most of a demote cascade's requests;
sizing it in air time keeps the burst equal at every rung. Tokens start
full at 64 (before the first op is known). Refilled at each verified NACK at

```
rate = delivered_mbps(op.ladder[0]) × 1e6/8 / 346 B × nack.air_pct/100   symbols/s
```

(`delivered_mbps` = PHY rate × the air clock's per-MCS efficiency for the
applied op's sid-0 spec). A symbol costs 1 token, 2 under the repeat flag;
a symbol the tokens do not cover is refused, so a request is served in
order up to the tokens and the rest is counted `retx_refused`.

**Config** (`drone/src/config.cpp`, `bundle/mabur.default.toml`):

```toml
[nack]
ring_ms = 150   # history kept, at encoder.bitrate_max_kbps; 50..1000
air_pct = 5     # bucket refill, % of the rung's delivered sid-0 capacity; 0..50
burst_ms = 20   # bucket depth, ms of air at the current op; 1..200
```

## Accounting (option A)

- **Arrival tracker exclusion.** A retx-marked body never touches the
  `ArrivalTracker` (`SwDecoder::add_symbol(…, retx)`), so pre-FEC loss and
  both util inputs see the original loss.
- **Boundary-neutral.** A retx body never closes a rate transition: the
  drone re-sends old seqs at the current rate, so `UepDecoder` forces its
  boundary hint to `kNone` and `SwDecoder` does the same.
- **`retx` class.** A symbol first made known by a retx body books
  `syms_retx` (not delivered, not recovered); if its direct copy shows up
  later it also books `syms_retx_arrived`. A retx of an already-known seq
  is a stale drop. `LossEpisode` gains `retx` (feclog 3's `rtx` column).
  Seqs a retx body cascade-solves off a pending repair row (one row
  covering two holes, the retx fills one, the row then yields the other)
  inherit the retx class: they book `syms_retx`, read `kRetx` to the
  tracker (`filled`, not `wasted`) and count in the episode's `retx`
  (`SwDecoder::ingest`): without the retx the row could not have yielded
  them.
- **Residual inputs** use `abandoned + syms_retx` on both paths
  (`gs/src/ladder_residual.cpp`, the s3 path in `gs/src/link_health.cpp`).
  A retx-filled symbol counts as loss even if its direct copy arrives
  later, symmetric with `recovered`.
- **Latency anchor.** When fragment 0 of an AU arrived in a retx body
  (`hdr_retx`), the enc/q anchor sample is withheld: its stamps describe
  the original send, a NACK round trip earlier.
- **Tracker outcome buckets are not disjoint.** A requested entry books
  one of `filled` (Retx), `late_fill` (Direct), `wasted` (Recovered) when
  it resolves, and `dropped_deadline` when it is still Unknown at the
  deadline or falls below the floor. A seq that booked `dropped_deadline`
  and resolves later also books `filled`/`late_fill`/`wasted`, so the
  outcomes can sum above `syms_requested` under congestion. A requested
  seq the stop rule killed still books `dropped_deadline` at the
  deadline/floor. `suppressed` counts polls, not symbols.

## Observability

- **Sideport `link.nack`** (only while enabled; `gs/src/stats_exporter.cpp`):
  cumulative `requests, repeats, syms_requested, tail_requests, filled,
  late_fill, wasted, dropped_deadline, suppressed, lead_skipped`; per export window
  `fill_pps`, `fill_ms{p50,p90,max}` (first request → retx-filled,
  nearest-rank), `late_ms_max`; gauge `settle_ms`.
- **Sideport `drone.nack{rx, retx_syms, retx_refused}`**: the Telem fields,
  per Telem period and repeated on every record until the next Telem, so
  count once per `drone.tlm_seq`.
- **fec.log `feclog 3`** (`gs/src/fec_log.h`):
  `<t_ms> <sid> <mcs> <bw> <ov> <first_seq> <span> <m> <rec> <rtx> <aband>
  <stale> <r> <w>`; `rtx` = symbols of the episode a retransmit filled.
  feclog 1/2 rows read as `rtx` 0 (`docs/data-provenance.md`).
- **maburtop**: a `nack:` line (`req fill waste sup p50/p90/max ms settle |
  drone rx syms refused`) in the LADDER panel. That panel is hidden in
  static-pin mode, so on a pinned bench read `flight.jsonl` or
  `tools/bench/nack/arm_report.py` instead.
- **flightreport.py**: a NACK section (`print_nack_report`: requests/min,
  counter-reset tolerant; repeats, outcomes; drone `rx`/`retx_syms`/
  `refused` summed once per `tlm_seq`; fill p50/p90/max, settle) and a
  `retx=` count per group in FEC EPISODES; `load_feclog` reads feclog
  1/2/3 by marker.

## Bench

**Rig: `tools/bench/benchjam.sh`** (primary), a spare 8822EU on the host
flooding 1000 B QoS-Data at 6 Mbit/s on the op channel:
`tools/bench/benchjam.sh --channel <op> --pps 180`. Losses are whole PPDUs
on both GS cards, the drone's carrier sense defers to the jammer, and the
uplink NACK goes through the same air.

**Calibrate** the frame rate per bench geometry, at the pinned rung, 60 s
per step, until the control shows ~30 base abandoned symbols/min (spec §8);
the calibration runs so far are in the findings records below.

**Arms** (`tools/bench/nack/`): `cfg/A0_control.toml` (`[link.nack]
enable = false`) and `cfg/A1_nack.toml` (`enable = true`, lookback 256,
repeat_ms 16, max_tries 2), both the GS's live config pinned at
`static_mcs 2` / `static_bw 40` / `static_overhead_base 0.5` /
`static_overhead_enh 0.25` with `debug_log.enable`; `cfg/F0_control.toml` /
`cfg/F1_nack.toml` are the same pair with the ladder adaptive
(`static_mcs = -1`) for the fade arm. Stage the build under test
(`out/arm64/maburgs`) as `GSBIN` (default `/usr/local/bin/maburgs.fecnack`)
before the arms. Stop `S96maburgs`,
stage the configs in `/tmp/cfg/` on the GS, run `runjam.sh` (each ssh
retries 18 × 5 s because the jammer disturbs the GS's management Wi-Fi;
an arm whose restart did not write a new `/tmp/mabur-session` aborts the
run), then `arm_report.py <session dir>…`: au.log truncations and fid
gaps, fec.log abandoned episodes and `rtx` per sid, air_pct, the last
`link.nack` block, `drone.nack` summed per `tlm_seq`. `runarms.sh` is the
loss-sim variant (needs a `MABUR_LOSS_SIM` build). Bench arms are pinned,
so the stop rule is inert (below): expect `suppressed` 0.

**Pass criteria** (spec §8). *Loss arm*, control vs NACK, 5 min each: base
truncated + dropped −90 % or better; fill p90 < 25 ms; wasted < 25 %;
`retx_refused` 0; `air_pct` within 1 pt of control; `ausniff.py` clean;
`aucadence.py` within the per-rung baseline. *Fade arm* (adaptive ladder,
jammer 10 s on / 20 s off, three pulses): demote timing within one
feedback period of a control run; `suppressed` rises during pulses;
`retx_refused` bounded; fills visible at the lower rung after each pulse.
*Loss-sim* (secondary): `s0 eff=1.5 burst=4`, same criteria.

Bench records: spike `docs/findings/2026-10-05-fec-nack-spike.md`;
as-built gate against these criteria
`docs/findings/2026-10-06-fec-nack-bench.md`.

## Known limitations

- The tail-trigger geometry guard in `gs/src/main.cpp` is a tautology:
  one fragment == one symbol holds today by construction. A fragmenter
  change needs a test, not that guard.
- `NackWindow::fill_ms` is drained only by the sideport export. It is
  capped at `NackWindow::kMaxFillSamples` (4096): past that it stops
  appending, so the export's `fill_ms` percentiles cover the window's
  first 4096 fills (`filled` and `fill_pps` still count all). With NACK on
  and the stats sideport off it sits full at 16 kB instead of growing.
- `link.nack.lookback < fec.seq_horizon` is checked even without a
  `[link.nack]` section (default 256: a `seq_horizon` ≤ 256 fails boot).
- The stop rule reads the ladder controller's util, which static-pin mode
  never ticks: the stop rule is inert on a pinned link (all bench arms).
- `TxQueue::push` sheds from the front when over cap, so under a heavy
  backlog the retx bodies at the head are the first dropped; they count in
  `txq_drops`.
- The bucket starts full (64) and neither refills nor re-sizes until the
  first op is published (refill rate 0 without one).
- `nack_bad` and `retx_miss` are counted on the drone but reported nowhere
  (no Telem field, no log line); `auth_reject` is the only sign of a NACK
  that failed its tag or counter, a corrupted one is invisible, and so is
  a ring miss.

## Deploy

RC_VERSION 15 is a flag day: `maburd` and `maburgs` must both run it (a
mismatched pair has no control link), so swap the two back to back. On each
end the binary goes in before a config that carries `[nack]` /
`[link.nack]` — a build without those keys fails boot on them (strict
keys). Turning the feature off is config-only on the GS (`[link.nack]
enable = false`, the shipped default); a drone that is never asked never
re-sends.

## Demand refresh

The bundle runs the drone with no periodic refresh at all
(`venc.gop_s = 0` → the SigmaStar SDK's largest GOP, `VENC_GOP_NONE` =
65535 frames; `venc.intra_refresh_frames = 0`, since the GDR sweep only
fires at a GOP start and there is none). Healing a damaged base picture is
therefore on demand: `RefreshRequester` (`gs/src/refresh_requester.h`) in
`maburgs` decides when the RCF's `idr_epoch` byte changes, and
`RcAgent` on the drone serves one paced IDR per change
(`docs/link-adaptation.md` "RcAgent is the only IDR authority").

**Order.** FEC recovery and the software NACK retransmit (above) run
first, inside `FrameStream`'s reassembly; only once an access unit is
finished — complete, truncated, or slice-salvaged — does
`RefreshRequester::note_au()` see it and decide whether to arm. A request
then rides `idr_epoch` on every RCF until `RcAgent` serves the IDR and a
later complete base IRAP clears it: FEC → NACK → request → IDR → clear.

**Arms on:** a base AU (`sid 0`) that finishes incomplete (the NACK's
repair window has already passed; reason `hole`); a base AU rebuilt by
slice salvage (a filled band is a stale reference; `FrameStream` reports
it incomplete with the salvaged flag set, and it books as `salvaged`, not
`hole`); whole AUs the stream skipped (reason `lost`; their layer is
unknown, so they count as base); or one of four one-shot reports from
`maburplay` over
the same loopback control socket that carries the record button's VTX
wish (`gs/player/src/core_control_client.h`, `docs/vtx-recorder.md`
"Wire"): `start` (ring open, nothing to arm on yet), `resync` (the ring
reader skipped AUs), `decoder` (the decode watchdog reset or recreated
the backend — not raised on an ordinary decode error), and `dvr` (the
raw DVR opened and needs a key frame to begin on). Never arms on enhance
damage: nothing references an enhance frame.

A skip is reported on the AU emitted right after it
(`AuLatMeta::skipped_before`), in stream order. When that AU is a complete
base IRAP the skip is already healed and arms nothing, so the answering
IDR arriving behind a skipped AU does not buy a second one. A late copy of
a skipped AU is evicted, and a headerless slot aged out, without booking
anything: the skip itself already did (both still count in
`frames_dropped()`). AUs lost at the end of a stream with no successor arm
nothing until the next AU arrives.

**Clears on:** a complete base AU whose first slice NAL is an IRAP
(16..21), requested or not — a refresh-start (TRAIL_R) picture does not
clear it: only an IRAP restarts every reference. While armed the byte is
bumped again every 300 ms (`RefreshRequester::kRetryMs`): the expected heal is one RCF period plus
the IDR's transit, so a retry only means the IDR itself was lost or
truncated.

**Across an outage.** Retries only run while maburgs is in a session
(`vrx.link_state() == SESSION`): with no drone to hear them, they would
only pile up `I` rows. Arming still bumps the epoch at once whatever the
link state, and the first in-session tick retries at once if the last
bump is 300 ms old.
`heal_ms` counts from arming, though, so a request armed before the link
is up — the player's boot `start`, which usually lands long before the
drone powers up, or a hole at the start of an outage — reads as a long
heal that includes the dead time. `RcAgent` only serves a pending GS/chain request while
`state_ == LINKED`, so the request just waits; the entering-LINKED IDR it
serves on reconnect covers whatever broke while unlinked, and that same
IDR clears the request. A drone restart looks the same from the GS side:
the old in-flight base AU ends incomplete and arms one request, which the
new producer's link-up IDR clears. The GS's epoch is not reset per
session, while the drone's `idr_epoch_seen_` is reset to 0 at every
session edge, so the first RCF of a new session reads as a request:
absorbed by the link-up IDR when the session starts with one, otherwise
one extra IDR.

**The 65535-frame GOP is not forever.** With nothing else restarting it
first, the encoder still emits a GOP-start picture every ~65535 frames —
about 18 minutes at 60 fps
(`docs/findings/2026-10-17-sdk-gop-ceiling.md`). Any chain-break, rung
change, or demand-refresh IDR restarts the count, so this ceiling only
matters on a flight with none of those for that long.

**Observability:** `fec.log`'s `I <t_ms> <reason> <epoch>` / `H <t_ms>
<heal_ms> <retries>` rows (`feclog 4`) and the sideport's `link.refresh`
block (`docs/observability.md`) carry the same counters;
`flightreport.py`'s REFRESH section and maburtop's `refresh:` line read
them.

**What this gives up.** A periodic refresh used to bound how long
undetected wire corruption — a frame that decodes without error but is
wrong — could propagate: at most one `venc.gop_s` period. Demand refresh
has no such bound, because nothing arms it unless FEC/NACK/the stream
already know a frame is damaged, salvaged, or lost. The only mitigation
left is the player's own decode-watchdog report (`refresh decoder`),
which catches a decoder that is visibly failing, not a reference that is
silently wrong.
