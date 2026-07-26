# CLAUDE.md

Guidance for Claude Code when working in this repository.

mabur is a two-daemon FPV video link: `maburd` (drone, OpenIPC/SigmaStar,
armv7) encodes and injects; `maburgs` (ground station, aarch64) receives,
FEC-decodes, and emits RTP to PixelPilot. `common/` holds the shared wire
formats and FEC; `third_party/devourer` (plus the sibling checkout
`../devourer`) is the userspace radio driver.

## Build & test

NixOS host: wrap every cmake/ctest invocation in
`nix-shell -p pkg-config libusb1 --run "..."`. Configure with
`-DDEVOURER_DIR=$PWD/../devourer` when the default `../devourer` doesn't
resolve. Host suite: `ctest --test-dir build -R 'test_|host_e2e'`.
Cross-builds: `tools/build-arm64.sh` (maburgs), `tools/build-arm.sh`
(maburd). Devices: drone `root@192.168.10.152`, GS `root@10.18.0.1`.
Never load the 8812eu kernel module on the GS cards.

## Debugging & observability — which tool for what

The **stats sideport** is the primary debug surface. maburgs pushes a JSON
datagram (schema v1) to UDP `127.0.0.1:8300` every 500 ms: per-card and
per-RF-class signal stats, link op point, per-stream FEC health, video/RTP
health, TX/injection rates, airtime estimate, and drone telemetry (state,
applied op, encoder, queues, uplink RF, temps — via the 1 Hz T_TELEM
control frame). Consume it with:

- `maburtop` on the GS (`tools/maburtop.py`) — full-screen console,
  grouped by link; color thresholds carry the judgment.
- Ad-hoc capture: any UDP listener on :8300, or passively via AF_PACKET on
  `lo` when a consumer already holds the port.
- Flight recorder: `socat -u udp-recv:8300 - | jq -c . >> flight.jsonl` on
  the GS logs every metric at 2 Hz for post-flight analysis. The `jq -c` is
  REQUIRED: sideport datagrams carry no trailing newline, so bare socat
  appends concatenated JSON, not JSONL (recover such a file with
  `jq -c . < file`). GS `/root/statsrec.py` does the same and also re-emits
  datagrams to :8301 so a live `maburtop --port 8301` can watch alongside.

**Rule of thumb: if you want to KNOW something about the running link,
read the sideport. Reach for other tools only in these cases:**

- **Verifying maburgs itself → `tools/bench/rtpsniff.py`** (on the GS:
  `python3 rtpsniff.py lo 5600 30`). It taps the emitted RTP stream from
  OUTSIDE the daemon. This is the standing regression gate for any change
  that touches maburgs: the sideport is maburgs self-reporting, so gating
  a maburgs change on the sideport is circular — a bug that mangles
  packets or lies in its own counters sails through. Never replace the
  rtpsniff gate with sideport numbers.
- **Packet-level forensics → capture tools** (`tools/bench/seqdump.py`,
  `fu_probe.py`, `decode_bodies.py`, `live_decode.py`/`live_play.py`).
  The sideport is aggregates; when a summary number looks wrong, these
  record raw bodies/RTP for offline dissection.
- **Post-mortem when no consumer was listening → the 1 Hz stderr stats
  line** in `/tmp/maburgs.log` (and maburd's in `/tmp/mabur.log`). It is
  numerically redundant with the sideport but persists on disk; the UDP
  feed is ephemeral.
- **Radio/PHY bring-up below mabur → devourer's own tools** (`rxdemo` with
  `DEVOURER_RX_ALLPATHS=1`, `doctor`, etc. — see
  `third_party/devourer/CLAUDE.md`). Use these when the question is about
  the chip/driver rather than the mabur link.
- **Bench harnesses** (`bench/linkbench`, `bench/txagcbench`) drive
  special TX/RX modes for characterization; they are not monitoring tools.
  Note: the drone radio RX can wedge after a linkbench run — restart
  maburd.

Schema/design references (local, gitignored):
`docs/superpowers/specs/2026-07-25-gs-stats-sideport-design.md` and
`docs/superpowers/specs/2026-07-26-drone-telemetry-design.md`. The schema
is additive-only under `v: 1`; consumers must ignore unknown keys. The
sideport config lives in `/etc/maburgs.json` under `stats`
(default-off in the shipped bundle; enabled on the bench GS).
