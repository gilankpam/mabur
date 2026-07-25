# Follow-up: GS RtpPacketizer under-fills RTP packets (~2× packet rate)

**Status:** open finding from the frame-shm hardware A/B acceptance (2026-07-23), branch `frame-shm-ingest` / PR #1. Non-blocking (no video-quality impact), but worth fixing before pushing to higher bitrates.

**Owner:** unassigned — this doc is the hand-off for a follow-up agent.

---

## The finding

On the GS, the new frame-shm tail's `RtpPacketizer` (`gs/src/rtp_packetizer.cpp`) emits roughly **twice as many RTP packets** — each about **half full** — as waybeam's native RTP packetizer, for **identical video** (same fps, same application bitrate, 0% loss). This is a packetization-efficiency gap, not a correctness or quality problem.

### Measured evidence (A/B, mcs5/ov0.25, ~8.3 Mbps H.265, 60 fps)

Both arms measured with the same RTP sniffer bound to the GS's `127.0.0.1:5600` (where maburgs sends to PixelPilot), 30 s windows:

| Metric | Frame arm (new path) | Baseline (waybeam native RTP) |
|---|---|---|
| fps (RTP marker rate) | 59.5 | 59.5 |
| app bitrate | 8.28 Mbps | 8.20 Mbps |
| RTP loss | 0.000% | 0.000% |
| **packets/sec** | **~1449** | **~763** |
| **packets/frame** | **~24** | **~13** |
| **avg RTP payload** | **~720 B** | **~1350 B** |

The MTU budget is `max_payload = 1400` (set in `gs/src/main.cpp` where `RtpPacketizer` is constructed, and default in `RtpPacketizerCfg`). Baseline packs to ~1350 B (near-MTU); the frame path lands at ~720 B.

### Impact

- ~686 extra packets/s × ~40 B fixed per-packet overhead (12 B RTP + 8 B UDP + 20 B IP) ≈ **~220 kbps extra wire overhead** for the same video, plus the per-FU indicator bytes.
- ~2× the packet rate for PixelPilot's depacketizer to process.
- Scales worse at higher bitrates — at 2× the video rate the absolute pps gap doubles too. Fix before raising the operating bitrate.
- **No** impact on delivered video: fps, bitrate, and loss are all at parity.

---

## Root-cause hypotheses (not yet confirmed)

The finding was flagged from the pps measurement; the exact cause was **not** root-caused. Two likely contributors, both in `gs/src/rtp_packetizer.cpp`:

1. **FU packets emitted smaller than `max_payload`.** The packetizer is a *streaming* start-code scanner: `data()` is fed the frame as a byte stream (the GS `FrameStream` calls `frame_data()` with the contiguous chunk prefix, which arrives in wide-FRAG-fragment-sized pieces of ~158 B). To bound buffering and guarantee the closing FU-end is non-empty, it emits FU fragments as bytes accumulate. If the emission threshold/条件 flushes an FU before `pending_` reaches `max_payload` (e.g. flushing per input chunk, or a conservative "retain ≥1 byte" that under-fills), you get many ~half-MTU packets. **~720 B ≈ half of 1400 is a strong hint the flush cadence is the culprit** — check the FU-emission loop in `data()` and `close_nal()`.

2. **No aggregation of small NALs (RFC 7798 AP packets).** An HEVC access unit has small non-VCL NALs (VPS/SPS/PPS/SEI, tens of bytes each) plus the big slice NAL. The mabur packetizer sends each NAL ≤ MTU as its own single-NAL packet; waybeam's native packetizer may aggregate several small NALs into one AP packet. This alone only accounts for a few extra packets per frame (mostly on IDR frames), so it's likely secondary to (1).

The dominant factor is almost certainly (1), because the 2× gap is on **every** frame, not just IDRs.

---

## How to reproduce / root-cause

### 1. Packet-size histogram on the GS

Capture the actual RTP payload-size distribution to confirm whether it's uniform-half-MTU (points to FU under-fill) vs a bimodal small+large (points to per-NAL/no-aggregation). The GS has `python3` and nothing binds `:5600` when PixelPilot isn't running, so a UDP receiver can sniff directly. Sniffer used during acceptance (push to `/tmp/rtpwin.py` on the GS `root@10.18.0.1`, run `python3 /tmp/rtpwin.py <seconds>`):

```python
import socket, sys, time
dur = float(sys.argv[1]) if len(sys.argv) > 1 else 30
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(('127.0.0.1', 5600)); s.settimeout(2)
pkts=markers=byts=lost=0; last=None; pts=set(); sizes=[]; t0=time.time()
while time.time()-t0 < dur:
    try: d,_ = s.recvfrom(2048)
    except socket.timeout: continue
    if len(d) < 12: continue
    pkts+=1; byts+=len(d); markers += (d[1]>>7)&1; sizes.append(len(d)-12)
    seq=(d[2]<<8)|d[3]
    if last is not None:
        g=(seq-last-1)&0xffff
        if 0<g<30000: lost+=g
    last=seq; pts.add((d[4]<<24)|(d[5]<<16)|(d[6]<<8)|d[7])
el=time.time()-t0 or 1
print(f"WIN {el:.1f}s | pkts={pkts} pps={pkts/el:.0f} | frames={markers} fps={markers/el:.1f} | lost~{lost} loss={100*lost/max(1,pkts+lost):.3f}% | kbps={8*byts/el/1000:.0f}")
# ADD for this investigation: payload-size histogram
import collections
h=collections.Counter((sz//100)*100 for sz in sizes)
print("payload-size buckets (B):", dict(sorted(h.items())))
```

Expected reading: if most packets cluster around ~700 B with few near 1400, it's FU under-fill (hypothesis 1).

### 2. Read the emission logic

`gs/src/rtp_packetizer.cpp` — trace `data()` → the FU streaming loop and `close_nal()`/`drain_fu()` (the closing-FU path added `allow_end`). Confirm whether an FU is emitted before `pending_` reaches `max_payload`. The design intent (see `docs/superpowers/plans/2026-07-22-frame-shm-ingest.md`, Task 6) was: emit FU-start/middle packets of `max_payload` payload bytes while `pending_` holds more than `max_payload+1` unsent bytes, always retaining ≥1 unsent byte. If the implementation flushes more eagerly than that, that's the bug.

### 3. Unit-test angle

`tests/test_rtp_packetizer.cpp` already has `large_nal_fu_fragments`. Add/extend a test that feeds a large NAL **in small `data()` chunks** (simulating FrameStream's ~158 B fragment delivery) and asserts every emitted FU except the last has payload == `max_payload` (i.e. packets are MTU-filled regardless of input chunking). That test should fail today and pass after the fix, and it's the reliable regression guard. Note `byte_at_a_time_equals_bulk` already asserts chunking-invariance of *content*; this adds MTU-fill of *packet sizes*.

---

## Fix direction

Make FU emission MTU-greedy and chunking-independent: accumulate `data()` input into `pending_` and only emit an FU when there are `> max_payload` unsent bytes, emitting exactly `max_payload` payload bytes per FU (retaining the remainder), so packet size is decoupled from the input chunk size. Optionally add AP aggregation for consecutive small NALs (secondary).

### Constraints (do not regress)

- `max_payload` stays 1400; RTP **PT stays 97**, SSRC `0x4D414252` — PixelPilot needs no config change.
- Must not break `byte_at_a_time_equals_bulk` (byte-identical output regardless of input chunking) or any other `test_rtp_packetizer` case.
- Must not reintroduce the truncation/no-fake-E behavior (`truncated_frame_no_marker_no_e_bit`) or the pts unwrap.
- Full mabur suite stays green; host e2e (`test_frame_e2e`) stays green.

## Verification criteria

1. Unit: new FU-fill test passes; all existing `test_rtp_packetizer` + `test_frame_e2e` pass.
2. Hardware A/B re-run (method below): frame-arm **pps ≈ baseline** (~13 pkts/frame, ~1350 B avg), with fps/bitrate/loss unchanged at parity.

### A/B method (for the hardware re-check)

Flip **only** the drone's `video_input` between arms; the GS auto-follows via `CAP_FRAME_WIRE`. Same rig/day. Devices: drone `root@192.168.10.152`, GS `root@10.18.0.1`.
- Frame arm: drone `/etc/mabur.json` `video_input:"frame_ring"` + waybeam `/etc/waybeam.json` `outgoing.server:"frame-shm://mabur_f"`.
- Baseline arm: `video_input:"ring"` + waybeam `outgoing.server:"shm://mabur"`.
- Each format switch requires restarting **all three** (waybeam, maburd, then maburgs — the maburgs restart is required because `peer_caps` only refreshes on session establishment). Back up configs/binaries as `*.pre-*` first. See `docs/frame-shm-bench-acceptance.md` for the full protocol.

---

## Context

- The frame-shm feature (whole-frame ingest + frame-aligned FEC, GS `FrameStream` → `RtpPacketizer` → RTP :5600) is on branch `frame-shm-ingest`, PR #1, gated behind drone `video_input` (default `"ring"`; frame path only when `"frame_ring"`).
- As of this writing the rig runs the frame path deployed and hardware-validated at parity, with graceful degradation and both maburd- and waybeam-restart recovery confirmed. This pps gap and a glass-to-glass latency measurement (needs a physical timer rig) are the only open acceptance items.
- Design/spec: `docs/superpowers/specs/2026-07-22-frame-shm-ingest-design.md`, plan Task 6 in `docs/superpowers/plans/2026-07-22-frame-shm-ingest.md`.
