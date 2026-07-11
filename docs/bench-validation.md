# Mabur Bench Validation Guide

Hardware bring-up and validation for mabur v1. Written to be followed from a
fresh session with no prior context. Everything referenced here is committed;
paths are relative to the mabur repo root unless noted.

## What mabur is (30-second recap)

Drone-side FPV streamer bridge for an SSC338Q (SigmaStar Infinity6E) camera.
Pulls H.265 RTP from waybeam's shared-memory ring, applies per-layer
Reed-Solomon UEP FEC + SBI framing, and injects over a Realtek RTL8812EU USB
dongle using the **devourer** userspace driver (not the kernel module). It
consumes ground-station adaptive-link feedback (RCF/DISC frames on the same
radio) and falls back to a MAX_RANGE profile when feedback goes silent.

- Design spec: `docs/superpowers/specs/2026-07-10-mabur-system-bundle-design.md`
  (local only — `docs/superpowers/` is gitignored; the file is on disk).
- v1 is merged to mabur `master` (public: github.com/gilankpam/mabur) and all
  14 host test suites + the host E2E pass. The drone-side software is done;
  **this document is the remaining work: prove it on real hardware.**

## Status entering bench

Everything below the RF boundary is verified in software:
- Wire formats byte-pinned to devourer's Python references (golden vectors).
- Host E2E decodes maburd's actual emitted frames through devourer's own
  reference decoders: byte-exact 4-stream recovery, 100% critical delivery at
  20% loss, RCF-commanded MCS changes on the wire
  (`tests/integration/run_host_e2e.sh`).
- Fully static ARM binary builds (`tools/build-arm.sh` → `out/arm/maburd`,
  musl, ~1.8 MB, zero runtime deps).
- Firmware image builds: openipc-builder branch `feat/devourer` produces an
  `ssc338q_fpv_openipc-urllc-aio` image with waybeam + mabur wired to
  `shm://mabur` and booting via `S95waybeam` → `S96mabur`. Verified artifact:
  `openipc-builder/archive/ssc338q_fpv_openipc-urllc-aio/202607102145/`.

**Real-mode (threads / USB / radio) has never run on hardware.** That is the
entire point of bench validation.

## Bench results — 2026-07-11 (first hardware run)

Drone `root@192.168.10.152` (SSC338Q), dongle `0bda:a81a` = RTL8812EU / 8822E
(Jaguar3, chip-id 0x17), host USB `Sstar-ehci-1` (EHCI / USB2 high-speed).

**On-target smoke: B1 ✅ · B2 ✅ · B3 ✅ · B4 ✅ · B6 ✅** (B5 + E1–E6 pending —
need the proto-GS assembled on the host).

Three bugs found and fixed this session:

1. **🔴 TX-during-bring-up race (fixed — `drone/src/main.cpp`).** maburd's
   firmware download to the dongle failed on every boot
   (`bulk_send EP 5 FAIL rc=-7 got 0/387` → `firmware download FAILED`),
   leaving TX dead. Isolated with devourer's own `doctor`, which brings the
   *same* dongle up **HEALTHY** on the *same* host — so not the hardware, the
   EHCI/USB2 host, or devourer. Cause: mabur called `Init` (devourer's RX-only
   convenience wrapper) while the hot/agent threads were already pushing frames
   via `send_packet` *during* bring-up; premature frames clog the shared
   bulk-OUT FIFO (nothing drains them pre-FW-boot), which starves devourer's own
   reserved-page firmware download. Fix mirrors devourer's `doctor`/`streamtx`
   pattern: `DeviceConfig.rx.enable_with_tx = true`, then `InitWrite` → open a
   `device_ready` gate → `StartRxLoop`, with `DevourerSink::send` dropping until
   ready. Verified: FW boots, 11099 bulk sends OK / 0 fail, `tx_failed=0`.

2. **🟠 Firmware packaging (fixed — `openipc-builder` `mabur.mk`).** The image
   built maburd *dynamically* linked against `libdevourer.so` (Buildroot
   defaults `BUILD_SHARED_LIBS=ON`; devourer's `add_library` has no explicit
   STATIC) but never installed the `.so` → `error while loading shared
   libraries: libdevourer.so`. Fix: `-DBUILD_SHARED_LIBS=OFF` in the recipe
   (static-links devourer into maburd) + `MABUR_VERSION` bump to the fixed SHA.
   Worked around at the bench by side-loading `out/arm/maburd` (static musl).

3. **🟡 B4 waybeam IDR endpoint (fixed).** Route is `GET /request/idr`, not
   `/api/v1/idr`. Default corrected in `config.h` + `bundle/mabur.default.json`.

**Useful bench levers:** VBUS cold power-cycle (soft `libusb_reset` does NOT
clear a wedged Realtek chip) =
`echo soc:Sstar-ehci-1 > /sys/bus/platform/drivers/Sstar-ehci-1/{unbind,bind}`.
Full Jaguar3 bring-up trace = build with `DEVOURER_LOG_MAX_LEVEL=TRACE`
(`tools/build-arm.sh` hardcodes `WARN`, so no `[I]` lines by default).

## Two ways to get maburd onto the camera

Pick one:

1. **Firmware image (integrated).** Flash the openipc-builder image. On this
   machine, in `../openipc-builder`:
   ```
   git checkout feat/devourer        # feat/waybeam does NOT have mabur
   nix-shell --run "./builder.sh ssc338q_fpv_openipc-urllc-aio"
   ```
   Image lands in `archive/ssc338q_fpv_openipc-urllc-aio/<timestamp>/`. This
   boots straight into waybeam→mabur (both init scripts + configs baked in;
   `/etc/waybeam.json` already points `outgoing` at `shm://mabur`).

2. **Side-load onto a running OpenIPC camera (faster iteration).** Build the
   static binary and scp it over an existing image that already runs waybeam:
   ```
   bash tools/build-arm.sh            # → out/arm/maburd (static)
   bundle/install.sh root@<camera-ip> # scp binary + S96mabur + mabur.json,
                                      # and json_cli-wires waybeam to shm://mabur
   ```
   Note `bundle/install.sh` is manual-deploy convenience; it uses `json_cli`
   flag spelling that is UNVERIFIED (see bench item B4).

## Ground-station side (the "proto-GS")

There is no finished GS. The bench GS is assembled from devourer's own tools
(the spec calls this the proto-GS; `tools/bench/` is its starting point):

- **Receive + decode video:** a second RTL8812-class dongle on a Linux PC
  running devourer's `duplex` example (or `rxdemo`). It emits `rx.frame` JSONL
  events (per-frame `seq_num`, `rssi[2]`, `snr[2]`, `crc`) and can inject on
  the same handle. Pipe its output to a decoder built on
  `tools/bench/decode_bodies.py` logic (fec_subblock.unpack → stream_fec rs
  decoder → FRAG reassembly) to reconstruct the RTP/H.265 stream.
- **Send feedback (RCF/DISC):** devourer's `tools/precoder/adaptive_link.py`
  has an `AdaptiveVrx` class and a `--role vrx` CLI that consumes `rx.frame`
  events, scores the link, runs the energy-min controller, and emits RCF
  frames back through the duplex binary. This is the piece that closes the
  loop and drives mabur's RcAgent. **Assembling this live vrx pipeline against
  a real duplex subprocess is itself part of bench prep** — it is exercised
  in devourer by `tests/adaptive_onair.sh`; use that as the reference wiring.

The RC wire protocol both ends share is `tools/precoder/rc_proto.py`
(RCF/DISC/DISC_ACK) — mabur's `common/src/rc_proto.cpp` is byte-pinned to it,
so a vrx built on the Python side and mabur agree by construction.

## Config quick reference (`/etc/mabur.json`)

Defaults that matter for bench (full defaults in `bundle/mabur.default.json`):

| Field | Default | Bench relevance |
|---|---|---|
| `radio.usb_vid` | `0x0bda` (3034) | Realtek |
| `radio.usb_pid` | `0` = scan `{0xa81a, 0x881a, 0x8812}` | **confirm your 8812EU's PID (B2)** |
| `radio.channel` | `149` | 5 GHz; match the GS |
| `radio.width` | `20` | values ≠ 20 warn + fall back to 20 |
| `radio.bw_set` | `[20, 40]` | probe rungs (B7) |
| `radio.max_txagc` | `63` | power clamp |
| `link.vtx_id` | `1` | must match the GS's target |
| `link.failsafe_ms` | `1000` | silence → MAX_RANGE |
| `ring_name` | `"mabur"` | must equal waybeam `outgoing.server` `shm://mabur` |
| `waybeam.idr_path` | `/request/idr` | bench-confirmed route (B4); GET → `{"ok":true,"data":{"idr":true}}` |

maburd logs to `/tmp/mabur.log` (S96mabur truncates per respawn). `SIGUSR1`
dumps full counters to stderr.

## Bench checklist

Ordered smoke → integration. Stop and diagnose at the first failure.

### On-target smoke (camera only, no GS)

- [x] **B1 — boots and attaches.** DONE. Both daemons run; maburd opens the
  dongle, boots FW, and streams (`sent` climbing, `tx_failed=0`,
  `waybeam_failures=0`). **Required the TX-race fix (bug 1 above)** — before it,
  FW download failed and TX was dead. Also required side-loading the static
  binary (bug 2). RX alive (`rx_beat` climbing).
- [x] **B2 — USB PID.** DONE. `lsusb` = `0bda:a81a`, which is first in the scan
  list `{0xa81a, 0x881a, 0x8812}` — no `radio.usb_pid` override needed.
- [x] **B3 — CPU budget.** DONE. maburd sits at **~20% of one core** (`top`),
  well under the spec target < 35%. (No external video source at the bench yet,
  so this is the pipeline's own load, not a calibrated 8 Mbps.)
- [x] **B4 — waybeam control API.** DONE. The IDR route on this waybeam build
  is `GET /request/idr` (returns `{"ok":true,"data":{"idr":true}}`), **not**
  `/api/v1/idr` (that 404s: `{"ok":false,"error":{"code":"not_found"}}`).
  `waybeam.idr_path` default fixed to `/request/idr` in `drone/src/config.h` +
  `bundle/mabur.default.json`. Still TODO: confirm `bundle/install.sh`'s
  `json_cli` flag spelling matches the on-device `json_cli`.
- [ ] **B5 — monitor-mode capture.** From a nearby laptop with a monitor-mode
  dongle, capture a few seconds of maburd's injected frames. Verify radiotap
  MCS matches the commanded profile and the probe-bandwidth schedule
  (`seq % 32 ∈ {0,8,16}` fly the `bw_set` rungs). Confirms the air format
  before involving a decoder.
- [~] **B6 — kill/restart matrix.** PARTIAL. maburd restart DONE: soft
  `/etc/init.d/S96mabur restart` (no USB cold-cycle) re-boots FW and streams
  immediately — the clean de-init in the fixed binary lets the chip re-init
  without a VBUS cycle (the earlier need for cold-cycles was an artifact of the
  broken binary's crash-loop leaving the chip wedged). Still TODO: waybeam
  restart mid-stream (ring reattach), dongle re-plug.

### Bench end-to-end (camera + GS PC), the acceptance gate

Each scenario has a pass criterion from the spec's Testing section.

- [ ] **E1 — clean link.** GS decodes H.265, plays at commanded bitrate,
  post-FEC RTP loss = 0.
- [ ] **E2 — RC frame acceptance (verify B-item first).** Confirm a real
  received RC frame is accepted by mabur's parser. **Known risk:** devourer's
  `Packet.Data` on RX may include a trailing 4-byte FCS; if so, mabur's
  `on_rc_frame` computes `body_len = len - 24` including 4 junk bytes and the
  RCF CRC check rejects valid commands. Test: have the GS send one RCF; check
  maburd's log/`SIGUSR1` for `rc_records` accepted vs dropped. If dropped,
  the fix is a one-line tail-trim in `drone/src/main.cpp` rx_callback (trim
  4 bytes if the driver appends FCS) — file it back to the mabur repo.
- [ ] **E3 — degradation staircase.** Add attenuation / interference (devourer
  has `tests/sdr_interferer.py` for a calibrated co-channel source). GS
  commands down the ladder; observe T2 sheds first, then T1, waybeam bitrate
  follows within ~2 s, video never freezes.
- [ ] **E4 — feedback kill → failsafe.** Stop GS TX. maburd must reach
  MAX_RANGE within ~1.2 s (failsafe_ms 1000 + margin) — verify via log. Also
  confirm the encoder bitrate drops to the floor (~1400 kbps), not just the
  MCS. Resume GS → recovery + an IDR request.
- [ ] **E5 — cold rendezvous.** Boot the drone with the GS already beaconing
  DISC on the configured channel → drone links and streams at the init
  profile with no manual channel setup. Confirms DISC/DISC_ACK.
- [ ] **E6 — concurrent TX+RX sanity.** RC frames arrive while maburd is
  mid-injection (Jaguar3 does TX+RX on one handle via `StartRxLoop`). Watch
  for RX starvation or `GetTxStats`/`GetThermalStatus` contention under
  sustained bulk TX. No spec number here — characterize behavior.

### Config-edge / known non-blocking items

- [ ] **B7 — 40 MHz probes on a 20 MHz device.** `bw_set` includes 40 while
  `radio.width` is 20; probe frames inject at 40 MHz radiotap on a 20 MHz-tuned
  device. Confirm the GS still receives them (or drop 40 from `bw_set` if it
  causes trouble). This is the one place device tuning and per-packet
  bandwidth can disagree.
- [ ] **B8 — watchdog edge (only if you change config).** With default
  `failsafe_ms` (1000) < watchdog `stale_ms` (3000) the RX watchdog is safe.
  If you raise `failsafe_ms` above 3000, a quiet channel while LINKED could
  trip the watchdog before failsafe fires. Keep defaults unless you have a
  reason.

## Deferred to a future iteration (not bench-blocking)

- **Per-layer TX power (v1.1).** Needs the devourer Jaguar3 `TXPWR_OFSET` port
  first (8822E TX descriptor has the same 3-bit LUT field as Jaguar2, at
  `txdesc+0x14[30:28]`; devourer only wires it for Jaguar2 today — confirmed
  still true as of devourer `bb6c27e`). Mabur already carries
  `power_offset_db` per layer (defaulted 0, not emitted). After the devourer
  port + a bench A/B of GS-measured RSSI deltas per LUT step, enabling it is a
  mabur config change + one line in `RadioTx::to_tx_mode`.
- **Ground station proper** (`gs/` in the mabur repo, reserved). Would link
  `libmabur_common` so drone and GS can't drift. The bench proto-GS above is
  the interim.
- **Spec deferrals** (recorded in the spec's "v1 implementation deferrals"):
  SIGHUP config reload, ring-reattach IDR hook, non-20 MHz width tuning.

## If you find a drone-side bug at the bench

Fix it in the mabur repo on a branch, re-run the host suite
(`nix-shell -p pkg-config libusb1 --run "ctest --test-dir build -R 'test_|host_e2e'"`),
and — if the firmware pins to it — bump `MABUR_VERSION` in
`openipc-builder/package/mabur/mabur.mk` to the new SHA (the recipe fetches
the public repo by commit). The build environment is NixOS; devourer/libusb
builds need the `nix-shell -p pkg-config libusb1` wrapper.
