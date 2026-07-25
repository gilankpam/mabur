# Frame-SHM Bench Acceptance + Deployment Notes

> **STATUS (2026-07-25): historical record — the A/B this runbook describes is
> finished and the old path is gone.** The bench gates below all passed, the
> deployed default was flipped, and the pre-frame-shm video path has since been
> deleted: there is no `video_input` key and no second arm to select. Read this
> for what was measured and how; read
> [frame-shm-old-path-removal.md](frame-shm-old-path-removal.md) for the
> current shape of the system and the config migration. Sections 1.1–1.3
> (waybeam bring-up) and 2 (fu_probe method) still apply verbatim.

Operational runbook for deploying and bench-accepting the `frame-shm://`
video path (Tasks 1–11 of the frame-shm-ingest work). Written to be followed
from a fresh session with no prior context, in the style of
`docs/bench-validation.md` — that document covers the original RTP-ring
transport's hardware bring-up; this one covers switching the video ingest to
waybeam's whole-frame ring and frame-aligned FEC, and proving it is at least
as good before it becomes the only path.

## What's changing (30-second recap)

Today maburd consumes waybeam's RTP-packet shm ring (`shm://mabur`),
reclassifies packets, and flushes the sliding-window FEC on a schedule
unrelated to frame boundaries. The new path has maburd consume waybeam's
whole-frame ring (`frame-shm://mabur_f`, waybeam ≥ v0.42.0), classify whole
Annex-B frames, and seal the FEC window at frame end — killing the
seq-invisible mid-frame-truncation failure class from the 2026-07-17
drain-ceiling incident. RTP generation for H.265 also moves to the GS side
(streaming `FrameStream` + `RtpPacketizer`), replacing drone-side
re-packetization for this path. Everything downstream of RTP (PixelPilot on
`:5600`, PT 97, ssrc `0x4D414252` "MABR") is unchanged.

This was a **temporary A/B gate**, same pattern as the async-FEC-worker
rollout (commit 5a30989): both paths shipped together behind a drone-side
config switch, got bench-accepted here, and the old path was then deleted in a
follow-up commit (see "Post-acceptance" below).

Rig for this runbook: drone `root@192.168.10.152` (SSC338Q), GS
`root@10.18.0.1`. Same devices, same cards, same physical setup as prior
bench sessions in `docs/bench-validation.md` — do not introduce a second
variable by changing rig at the same time as the transport.

## 1. Deployment prerequisites

Do these once, before any A/B flipping. Both binaries and both configs get
backed up on **both** boxes first — the whole point of this section is that
rollback is a restore, not a rebuild.

### 1.1 Back up current state (drone AND GS)

```bash
# Drone
ssh root@192.168.10.152 '
  cp -a /usr/bin/maburd /usr/bin/maburd.pre-frameshm
  cp -a /etc/mabur.json /etc/mabur.json.pre-frameshm
  cp -a /etc/waybeam.json /etc/waybeam.json.pre-frameshm
'

# GS
ssh root@10.18.0.1 '
  cp -a /usr/local/bin/maburgs /usr/local/bin/maburgs.pre-frameshm
  cp -a /etc/maburgs.json /etc/maburgs.json.pre-frameshm
'
```

Confirm all four `*.pre-frameshm` files exist before continuing — these are
the rollback in section 4.

### 1.2 Build + deploy waybeam ≥ v0.42.0 (commit `18e304cb`)

The frame-shm output (`venc_frame_ring`, whole Annex-B frames + 8-byte
`VencFrameMeta` in a 16×512 KB SPSC ring) is new in waybeam v0.42.0. Build
and deploy it to the drone per waybeam's own build docs, landing the binary
at whatever path the drone's init script already expects (same binary slot
waybeam has always occupied — this is a version bump, not a relocation).

Once the new waybeam binary is in place, point its output at the frame ring
and restart:

```bash
ssh root@192.168.10.152 '
  json_cli /etc/waybeam.json outgoing.server frame-shm://mabur_f
  /etc/init.d/S95waybeam restart
'
```

Do **not** flip maburd's `video_input` yet — waybeam can run with
`outgoing.server = frame-shm://mabur_f` while maburd is still on `"ring"`;
maburd just won't be consuming it. This lets you deploy waybeam and confirm
it's healthy (`frame-shm` ring created, encoder running) before touching the
mabur daemons at all.

**maburd's cross-check:** once you do flip `video_input` to `"frame_ring"`
(section 2), maburd verifies waybeam's `outgoing.server` at startup via
`get_param`. If it does **not** contain `"frame-shm://"` (e.g. waybeam is
still on `shm://mabur`, or the restart in this step didn't take), maburd
logs a loud line to stderr:

```
maburd: FATAL MISMATCH: video_input=frame_ring but waybeam outgoing.server
is not frame-shm:// (got: ...). Video will not flow until waybeam is
reconfigured and restarted.
```

and **keeps running** — it does not crash-loop, but no video flows. If you
see this line, fix waybeam's config and restart it; you do not need to
restart maburd afterward, but the fresh startup log will confirm the match
on the next respawn.

### 1.3 Deploy the new maburd / maburgs binaries

Build and deploy as usual (this is the same tooling as every prior bench
session, no new steps here):

```bash
bundle/install.sh root@192.168.10.152          # drone: builds + deploys maburd
gs/bundle/install.sh root@10.18.0.1             # GS: builds + deploys maburgs
```

`bundle/install.sh` stops `S96mabur`, scp's the static ARM binary, and
restarts; it will not clobber an existing `/etc/mabur.json` (only installs
the default if one is missing) — the config-key additions below need to be
applied by hand.

### 1.4 Drone config: `/etc/mabur.json` gains two keys

> Post-deletion: `video_input` no longer exists. A config that still carries it
> (or `ring_name`) loads fine — both are accepted-and-ignored with a WARNING
> line — but the frame path is unconditional. Only `frame_ring_name` remains.

The A/B binary understood two new top-level keys, with these defaults
baked into the code (`drone/src/config.h`) if omitted from the file:

| Key | Default | Meaning |
|---|---|---|
| `video_input` | `"ring"` | **removed** — was `"ring"` = old RTP-packet shm path, `"frame_ring"` = whole-frame path. |
| `frame_ring_name` | `"mabur_f"` | Name of the waybeam frame-shm ring to attach to. Must match waybeam's `outgoing.server = frame-shm://<name>`. |

**Start the baseline arm with `video_input: "ring"` explicitly** (or simply
omit the key — `"ring"` is the default) even though waybeam is already
pointed at `frame-shm://mabur_f`. This is deliberate: it lets you confirm
the new maburd binary is a clean drop-in on the *old* path (no regression
from the binary swap alone) before you introduce the new transport as a
second variable. Edit on the drone:

```bash
ssh root@192.168.10.152 'json_cli /etc/mabur.json video_input ring'
ssh root@192.168.10.152 '/etc/init.d/S96mabur restart'
```

Confirm baseline health (fu_probe, section 2) before flipping to
`frame_ring`.

### 1.5 GS config: no action required for the A/B

`video_out` gains two keys on the GS (`frame_gap_timeout_ms`, default 50ms,
range [10,1000]; `frame_lookahead`, default 8, range [2,64]) but you do
**not** need to touch `/etc/maburgs.json` to run the A/B — maburgs
negotiates format per session from the drone's advertised capability
(`CAP_FRAME_WIRE` in its DiscAck) and auto-follows. Leave the GS config at
defaults unless you specifically want to bench different gap-timeout or
lookahead values later.

## 2. A/B method

The A/B variable is **one config key on the drone only**:
`/etc/mabur.json` → `video_input`. Everything else — rig, cards, channel,
antennas, GS config, waybeam config, day/conditions — stays fixed between
arms. Do not restart the GS between arms; it doesn't need to know.

1. Set the drone to the arm under test and restart:
   ```bash
   ssh root@192.168.10.152 'json_cli /etc/mabur.json video_input ring'          # baseline
   # or
   ssh root@192.168.10.152 'json_cli /etc/mabur.json video_input frame_ring'    # new path
   ssh root@192.168.10.152 '/etc/init.d/S96mabur restart'
   ```
2. Confirm the drone relinks (RENDEZVOUS→LINKED per the usual stats-line
   witness) and, for the `frame_ring` arm, confirm there's no `FATAL
   MISMATCH` line in `/tmp/mabur.log`.
3. Sniff RTP on the GS, `:5600`, using `tools/bench/fu_probe.py` (the
   trustworthy successor to `rtpsniff.py` — 8 MB `SO_RCVBUF`, seq dedup,
   seq-space unwrap, marker-classified truncation; see its docstring):
   ```bash
   ssh root@10.18.0.1 'python3 tools/bench/fu_probe.py lo 5600 60'
   ```
   Run it for long enough to get a stable fps/loss read (60–120 s per arm
   is enough for the parity/marginal gates below; the soak gate in section
   3 needs its own much longer run).
4. Record `fps`, `hard` (mid-frame truncations with no transport loss
   around them — the failure class this work targets), and `missing`
   (seq-space loss) for each arm, same rig, same day. Flip only
   `video_input` between runs; don't change MCS/overhead/antenna between
   the two arms of the same comparison.
5. For the `frame_ring` arm specifically, also watch maburd's own 5-second
   stderr counter line and maburgs' periodic stats line (both described in
   section 3) — these are new observability this path adds and have no
   equivalent on the old path, so they're not part of the A/B diff itself
   but are essential context for interpreting it.

## 3. Acceptance gates

All gates below compare the `frame_ring` arm against the `ring` arm (or, for
the soak/restart gates, against the `frame_ring` arm's own behavior — there
is no direct old-path equivalent to compare those against).

### 3.1 Parity gate — mcs5 / ov0.25 (~10 Mbps air, the deployed pin)

This is the known-good operating point from prior benches (safe pin
~mcs5/ov0.25 at ~10 Mbps air; ceiling ~11 Mbps) — the frame path must not
regress it.

- Run `fu_probe.py` on both arms at this pin, same rig/day.
- **Pass:** `end_HARD = 0` (no mid-frame truncations) on the `frame_ring`
  arm, fps ≥ 59.5, and seq loss (`missing`) comparable between arms (not
  materially worse on `frame_ring`).
- If `frame_ring` shows `end_HARD > 0` at this pin where `ring` showed 0,
  that's a regression — do not proceed to the marginal probe until it's
  understood.

### 3.2 Marginal probe — mcs5 / ov0.375 (~11 Mbps air)

This pin is expected to be at or past the drain ceiling. The point of this
gate is not "does it stay clean" (it may not) but "does overload degrade the
*right* way": counted frame drops instead of invisible seq-truncation.

- Run `fu_probe.py` as before, and simultaneously watch:
  - **maburd's frame-ring stderr line** (drone side, every 5 s):
    ```
    maburd frame_ring: fill=NN% writes=N full_drops=N oversize=N idr_disagree=N
    ```
    Rising `full_drops` here means the ring is shedding whole frames under
    pressure — the intended failure mode.
  - **maburgs' periodic stats line** (GS side) gains a frame-counter
    suffix in frame mode:
    ```
    frames[clean/trunc/drop]=N/N/N
    ```
    (from `FrameStream::frames_clean/truncated/dropped`). Watch
    `trunc`/`drop` climb together with `fu_probe`'s `end_HARD` count.
- **Pass:** as the pin gets marginal, `fu_probe`'s truncations should show
  up as **counted** drops/truncations in maburgs' `frames[.../.../..]`
  line and maburd's `full_drops`/ring fill — not as silent transport loss
  with no corresponding counter movement. Note qualitatively whether the
  drain ceiling itself has moved up, down, or stayed at ~11 Mbps vs the old
  path's known ceiling; this is a record-only observation, not a hard gate.

### 3.3 Restart robustness

Mid-stream, on the `frame_ring` arm:

1. Restart waybeam on the drone (`/etc/init.d/S95waybeam restart`) while
   maburd keeps running. Confirm: maburd survives (rides through same as
   the old path — see `docs/bench-validation.md` B6), the frame ring is
   re-attached or re-created cleanly on waybeam's side, and RTP resumes on
   the GS with **no seq discontinuity artifact beyond the expected stall**
   (i.e. seq keeps incrementing monotonically once frames resume; no
   seq reset, no duplicate-seq burst).
2. Restart maburd itself (`/etc/init.d/S96mabur restart`) while waybeam
   keeps running. Confirm: maburd re-attaches to the existing frame ring
   (no waybeam restart needed), relinks to the GS, and RTP resumes with the
   same seq-continuity expectation as above.
3. In both cases, confirm the GS's `frames[clean/trunc/drop]` counters
   don't do anything surprising across the gap (a burst of `trunc`/`drop`
   during the outage is expected and fine; what you're checking is that
   counting resumes correctly afterward, not that the outage itself is
   silent).

### 3.4 Soak ≥ 90 minutes, crossing the pts wrap

The drone's frame pts is a u32 microsecond counter and wraps every ~71.6
minutes; the wrap point does not land on a frame boundary. Run the
`frame_ring` arm continuously for **at least 90 minutes** so the soak
crosses this wrap live.

- Watch for: any RTP timestamp discontinuity, freeze, or seq/ts
  inconsistency at the wrap point. The packetizer unwraps pts into a wider
  counter before deriving RTP ts (never from the GS's own clock — that's
  the stale-clock bug class from the earlier transport), so the wrap
  should be invisible on the wire; confirm this is actually true on air,
  not just in the unit tests.
- Also watch the steady-state `fu_probe` numbers and the two counter lines
  over the full 90+ minutes for any slow drift (ring fill creeping up,
  `oversize`/`idr_disagree` counts appearing) that a short bench run
  wouldn't surface.

### 3.5 Glass-to-glass latency spot-check

Spot-check end-to-end (camera-to-display) latency on the `frame_ring` arm
against a baseline measurement taken the same way on the `ring` arm (same
method used in any prior latency spot-check for this rig — e.g. a
clock/LED-in-frame comparison, or timestamp correlation between capture and
PixelPilot render). The frame-aligned FEC flush (sealing the window at
frame end instead of on a schedule) is expected to be latency-neutral or
slightly better (tail symbols hit the air sooner), not worse — flag it if
the new path is measurably worse.

### 3.6 TxQueue drops under IDR load

Watch `txq drops` (the drone's TX-queue drop-oldest counter, exercised
whenever an IDR frame bursts more data into the pipe than a single tick can
drain) during any of the above runs, especially the marginal probe (3.2)
and the parity run's occasional GOP boundaries. The frame FEC window seals
per-frame rather than on a fixed schedule, which changes the burst shape
into the TxQueue; confirm `txq drops` under an IDR burst on `frame_ring` is
not worse than the same IDR-burst behavior on `ring` at the same pin. This
is a watch item (the design already accepts drop-oldest + FEC absorption
here), not expected to gate a fail on its own unless it's a clear
regression.

## 4. Rollback

If any gate above fails and you need to get back to the known-good state
without waiting on a fix:

```bash
# Drone
ssh root@192.168.10.152 '
  /etc/init.d/S96mabur stop
  cp -a /usr/bin/maburd.pre-frameshm /usr/bin/maburd
  cp -a /etc/mabur.json.pre-frameshm /etc/mabur.json
  cp -a /etc/waybeam.json.pre-frameshm /etc/waybeam.json
  json_cli /etc/mabur.json video_input ring
  /etc/init.d/S95waybeam restart
  /etc/init.d/S96mabur start
'

# GS
ssh root@10.18.0.1 '
  /etc/init.d/S96maburgs stop
  cp -a /usr/local/bin/maburgs.pre-frameshm /usr/local/bin/maburgs
  cp -a /etc/maburgs.json.pre-frameshm /etc/maburgs.json
  /etc/init.d/S96maburgs start
'
```

Key points, restated because they're easy to get backwards under pressure:

- Drone `video_input` goes back to `"ring"` (explicit, don't rely on the
  restored config file alone if you hand-edited anything mid-session).
- Waybeam's `outgoing.server` goes back to `shm://mabur` (**not**
  `frame-shm://mabur_f`) — the restored `/etc/waybeam.json.pre-frameshm`
  should already have this, but if waybeam was reconfigured live rather
  than via file restore, set it explicitly:
  `json_cli /etc/waybeam.json outgoing.server shm://mabur` — and restart
  waybeam after.
- GS needs no config change to roll back (it auto-follows), but restoring
  the pre-frameshm binary is still correct if the new maburgs binary itself
  is suspect, not just the config.

## 5. Post-acceptance

Once all gates in section 3 pass:

1. **Flip the deployed default.** Change the *shipped* default (not just
   the live device config) so new deployments start on `video_input:
   "frame_ring"` — update `bundle/mabur.default.json` (and the in-code
   default in `drone/src/config.h` if the intent is to make `frame_ring`
   the default even when the key is omitted from `/etc/mabur.json`).
   Leave both devices in this bench running on `frame_ring` going forward;
   don't revert to `ring` for day-to-day use after acceptance.
2. **Schedule the old-path deletion as its own reviewed change.** Do not
   fold it into the acceptance commit. The old RTP-packet path has real
   surface area to remove carefully:
   - `RingSource` video wiring (the old shm-ring consumer for video).
   - `classify_rtp`'s video use (packet-level classification, superseded
     by whole-frame `classify_frame`).
   - `RtpReorder` (packet reordering on the old drone-side packetization
     path — the new path packetizes on the GS from ordered frames, so this
     has no equivalent to preserve).
   - Narrow-FRAG video config (the 4-byte frag format, superseded by the
     wide 6-byte FRAG format for frame mode; narrow FRAG stays for
     non-video streams so don't remove the format itself, only its video
     wiring).
   This mirrors the async-FEC-worker precedent (commit 5a30989 removed the
   transitional config gate in its own follow-up commit after the gated
   feature was proven) — same shape here: prove first, delete later, as a
   change someone can review in isolation from the feature work.
