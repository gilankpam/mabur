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

### B5 session (2026-07-11 afternoon) — TX was never valid on air; root-caused

B5 initially read as "capture sees nothing". Root-caused to **two independent
problems**, one per side:

4. **✅ FIXED — devourer 8822E TX+RX mode breaks on-air TX (devourer fork
   `master` @ `c2f4dbd`).** maburd's frames have NEVER decoded on
   air: USB-level TX is green (`sent` climbing, `tx_failed=0`) but a monitor
   receiver sees only ~8% of frames as undecodable PHY hash (fa≈cca, no
   PLCP lock). Bisected with the same ARM `txdemo` on the drone dongle:
   pure TX (`InitWrite` only) = **7057/18s clean HT-MCS0 decodes**; identical
   flood with `DEVOURER_TX_WITH_RX=thread` (maburd's exact mode, from the
   fd1fa69 TX-race fix) = **0 decodes**. Mechanism: in TX+RX mode on the
   8822E, devourer deliberately skips the **path-B OFDM TXAGC reference
   (0x41e8)** on every TXAGC apply (`skip_path_b_ofdm_ref`, an RX-desense
   quirk) — but the chip transmits OFDM on both chains, and the unwritten
   path-B reference corrupts the combined waveform. Forcing the write
   (temp env knob `DEVOURER_FORCE_PATHB_REF=1`, uncommitted edit in
   `../devourer` `RtlJaguar3Device.cpp:apply_tx_power_current`) fixes TX
   completely: **19,814 frames / 20 s, 99.94% CRC-clean, rate=12 = HT MCS0**
   = the commanded MAX_RANGE profile. Bench-range RX check with the forced
   ref showed **no desense** (drone heard 4165/5000 of a host flood vs
   3872/5000 stock) — devourer's "any nonzero 0x41e8 → near-deaf RX"
   finding did NOT reproduce at 1 m; re-verify at range during E-series.
   Proper devourer fix TBD (candidates: TX-path-A-only mapping in TX+RX
   mode so 0x41e8 stays 0 for RX, or write the ref and re-validate desense).
   **Re-confirmed on devourer `master` (ab89f1b, evening retest)** with a
   single-binary env A/B — same txdemo build, TX+RX mode: stock = 0 frames
   on air, `DEVOURER_FORCE_PATHB_REF=1` = 5833 clean/15 s — removing any
   rebuild confound (the afternoon tests were built from the
   per-packet-txpower feature branch; behavior is identical on master).
   Master-built maburd + knob: 14,815 frames / 15 s, 99.6% CRC-clean, MCS0.
   **Landed for real on devourer fork `master` @ `c2f4dbd`**: the temp env
   hack is gone; the path-B OFDM TXAGC reference (0x41e8) is now written by
   default (`skip_path_b_ofdm_ref` inverted to opt-in legacy behavior via
   `DEVOURER_PROTECT_PATHB_AGC`, see bug 5's note below). Final A/B re-run
   against stock master-built maburd (no env at all): **5644 clean frames**
   vs **0 frames** with the legacy knob forced on — confirms the new default
   is the fix, not an artifact of the temp knob.
5. **✅ FIXED — devourer J3 radiotap parser drops LDPC/STBC on HT frames**
   (devourer fork `master` @ `c2f4dbd`). `send_packet`'s
   `IEEE80211_RADIOTAP_MCS` case read BW/SGI/MCS but ignored the FEC (0x10)
   and STBC (0x20) known/flag bits — only the VHT case parsed them. So
   mabur's MAX_RANGE `LDPC(+STBC)` robustness flags never reached the chip;
   everything flew as plain HT. Fixed via `RadiotapTxFlags.h` +
   `decode_radiotap_mcs_field` honouring the known/flag bits on Jaguar2/3
   (with a selftest). Re-validated live: **5701 frames, 100% `ldpc:1` /
   `stbc:1`**, with **no decode penalty vs plain MCS0** — this is bug 5's
   "RX-desense" concern from the note above answered in practice: writing
   the reference does not desense the on-air link at this range. (Note:
   `skip_path_b_ofdm_ref`'s **legacy** RX-desense-guard behavior is still
   available, opt-in, via `DEVOURER_PROTECT_PATHB_AGC` — see the open item
   below for the E-series range A/B.)
6. **✅ FIXED — devourer J3 coex thread dies silently on any register
   hiccup** (devourer fork `master` @ `c2f4dbd`). `coex_runtime_loop`'s 2 s
   tick body was `try { … } catch (...) { break; }` — one USB glitch
   permanently killed the thread that (per devourer docs) keeps sustained
   5 GHz TX alive (`coex_run_5g` + `pwr_track`), with no `[E]` log. Fixed:
   the tick body now retries transient register failures instead of dying
   silently. Coex soak re-run post-fix: clean, no thread death observed.
7. **✅ FIXED — devourer #253 zerocopy RX intermittently deaf on the GS
   host** (devourer fork `master` @ `c2f4dbd`). The default-on
   `DEVOURER_RX_ZEROCOPY` DMA RX path (master ab89f1b) delivered ZERO frames
   on one run and worked on another, same host/dongle/channel; the heap path
   was 100% reliable. Fixed by flipping the **default to off**
   (zerocopy is now opt-in); verified both directions (zerocopy explicitly
   off, and default-off with no env at all) give the same reliable heap-path
   behavior — bench captures no longer need `DEVOURER_RX_ZEROCOPY=0`.

**B7 answered (see checklist):** 40 MHz probe frames on a 20 MHz-tuned device
are aired but **unreceivable even by a 40 MHz-tuned monitor** — a 20 MHz BB
cannot emit a valid 40 MHz PPDU. Exactly 1/32 of frames (the `seq%32==8`
probe slot) are lost on air. FIXED in mabur 4252b02: config load
drops `bw_set` rungs > `radio.width` with a warning, and the default
`bw_set` is now `[20]`. Probing wider bandwidths needs `radio.width` ≥
the widest rung.

Also confirmed: the chip's `EN_HWSEQ=1` means the **on-air 802.11 seq is the
hardware counter**, offset from mabur's internal seq — GS-side gap detection
still works (lockstep +1) but slot arithmetic on captured seqs is shifted.

Earlier bugs (morning session):

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

**Bench state as of 2026-07-11 evening (B5 session):**

- The GS-side 8812EU (`0bda:a81a`) is **hard-wedged off the host's USB bus**
  (failed enumeration -71 at plug-in, dropped mid-re-enumeration at 12:44,
  sysfs per-port `disable` toggle did not revive it — desktop ports don't cut
  VBUS). **Physically re-plug it** before using it. Interim proto-GS receiver
  = the RTL8812AU (`0bda:8812`) — works well; its kernel driver
  (`rtw88_8812au`) was detached by rxdemo's libusb claim; rebind with
  `echo 2-2:1.0 | sudo tee /sys/bus/usb/drivers/rtw88_8812au/bind` if the
  kernel interface is wanted back.

**Bench state as of 2026-07-11, post-merge rollout (bugs 4–7 fixed):**

- Devourer fork `master` @ `c2f4dbd` (merge of
  `fix/8822e-txrx-tx-break`) is pushed to `origin/master`
  (`git@github.com:gilankpam/devourer.git`) — bugs 4/5/6/7 above are all
  fixed there, not workarounds. `../devourer` (the sibling checkout used by
  mabur's cross-build via `DEVOURER_DIR=../devourer`) is **clean `master`,
  no local edits** — the temp `DEVOURER_FORCE_PATHB_REF` getenv hack is
  gone entirely.
- The drone runs an **env-free**, master-built `maburd` from
  `/tmp/maburd-master` (rebuilt from mabur `master` after bumping
  `third_party/devourer` to `c2f4dbd`, cross-compiled with the standard
  `MABUR_CROSS_CC`/`MABUR_CROSS_CXX` toolchain env, no `DEVOURER_*` knobs at
  launch) — confirmed live: `tx_failed=0`, `sent` climbing steadily.
  `/tmp` is volatile; `/etc/init.d/S96mabur` is stopped (left alone per
  policy) and **still points at the old broken-TX binary** — rebuilding the
  openipc-builder image (which vendors `third_party/devourer` for the
  Buildroot recipe) so S96 ships the fixed binary is the one remaining
  deployment step.
- Bench captures **no longer need `DEVOURER_RX_ZEROCOPY=0`** — the default
  flipped to off with bug 7's fix, so a plain capture invocation (no env)
  now gets the reliable heap RX path.
- Final numbers from this rollout's live E2E re-validation (env-free
  master maburd → host `rxdemo` capture): fix-1 A/B = **5644 clean frames**
  stock vs **0** with the legacy knob forced on; fix-2 = **5701 frames,
  100% `ldpc:1`/`stbc:1`, no decode penalty vs plain MCS0**; coex soak
  clean (no thread death); zerocopy default-off verified both directions.
  Whole-branch final review: **clean, "Ready to merge."**
- **Open item carried forward:** RX-desense-at-range for the path-B OFDM
  TXAGC reference has NOT been re-verified beyond ~1 m bench range (see bug
  4/5 notes above — no desense observed at 1 m, but devourer's own
  historical concern was about range). The A/B lever for the E-series is
  now the properly named `DEVOURER_PROTECT_PATHB_AGC` env (restores the old
  legacy skip-the-reference behavior for comparison) — run E-series range
  tests with and without it set and compare RX sensitivity/desense before
  calling this fully closed.

## Bench results — 2026-07-12 (reflashed image; E-series session)

Drone reflashed with the openipc-builder image (kernel 2026-07-11 17:14 UTC).
The stock boot path now works end-to-end: **`/usr/bin/maburd` from the image
runs via S96mabur with no side-loading and no env** — TX decodes on air
(bug-2 packaging fix and devourer bugs 4–7 all confirmed present in the
shipped binary). Proto-GS = host 8812AU running devourer `duplex` driven by
`adaptive_link.py --role vrx` (`--vtx-id 1 --channel 149`), with duplex
stdout tee'd for capture.

**E2 ✅ · E4 ✅ · E5 ✅ · E6 ✅ · B6 ✅ (now complete).** No new mabur bugs.
Session evidence:

- **Post-flash TX sanity:** 16,454 frames / 15 s, **100% CRC-clean**, all
  MCS0 + `ldpc:1`/`stbc:1`. Pre-FEC air loss 7.6% (down from ~20% on
  2026-07-11); post-FEC stream 0 **0.000%**, stream 1 2.9% — strict E1
  "post-FEC 0" still needs link-budget work (E1 stays PARTIAL).
- **⚠️ Flashed binary predates the B7 clamp (4252b02).** Confirmed on air:
  lost frames cluster at exactly one mod-32 slot (556/1356 lost = 3.1% of
  all frames = the `seq%32==8` probe slot, observed at hw-seq slot 9 due to
  the EN_HWSEQ offset), and the boot log has no clamp warning. The image was
  built from the *committed* pin 641bded; the working-tree `mabur.mk` bump
  to 423d286 is uncommitted. **Remediation options:** edit `/etc/mabur.json`
  `bw_set` to `[20]` on the drone (works on any binary), and/or commit the
  mk bump + rebuild/reflash. Until then the drone burns 3.1% of frames as
  dead-air 40 MHz probes.
- **B6 complete.** Waybeam restart mid-stream: maburd unaffected — `sent`
  stalls ~6–9 s while waybeam re-inits, then resumes at full rate; no drop
  burst, `waybeam_failures=0`. Dongle "re-plug" (EHCI unbind/bind = VBUS
  cold-cycle with maburd holding the device): maburd dies with the bus,
  S96 respawn retries through `libusb_init failed (-99)` while USB is down,
  and full FW-reboot + streaming resume within ~3 s of rebind. No manual
  intervention.
- **E2 PASS.** Drone RENDEZVOUS→LINKED (state 1→2) on GS DISC/RCF across
  four separate sessions. **The FCS-tail risk did not materialize** — no
  tail-trim needed; received RC frames pass CRC + vtx_id as-is. Applied
  on-air witness: the tee'd GS capture shows the drone's TX rate move to
  exactly the vrx-commanded rungs (318k frames MCS0 + 1,774 MCS1 + 550 MCS2
  across sessions where the controller commanded MCS1/MCS2).
- **E4 PASS.** Timed kill (host↔drone clock mapped to ±5 ms via
  `/proc/uptime`, drone polled at 10 Hz): GS TX stop → FAILSAFE entry
  **≤1.05 s** (bound includes 1 Hz stats-line lag), within the ~1.2 s spec.
  Encoder floor confirmed: waybeam `video0.bitrate` = 1400 after failsafe.
  Resume: relaunched vrx re-linked the drone (FAILSAFE/RENDEZVOUS → LINKED);
  IDR-on-recovery is code-pinned (`entering_linked → request_idr()`), only
  weakly witnessable on air because this waybeam config's short GOP emits
  IRAPs every ~1 s anyway.
- **E5 PASS.** maburd restarted under an already-beaconing GS: exactly one
  RENDEZVOUS stats line before LINKED — **cold rendezvous in ~2 s**, no
  manual channel setup. (Contrast: re-link onto a vrx already in SESSION
  took 40–165 s — that's a proto-GS artifact: `adaptive_link` stops DISC
  beaconing once it hears video and relies on sparse RCFs getting through
  the marginal GS→drone direction. Not a mabur issue; mabur links instantly
  when a DISC arrives.)
- **E6 characterized, clean.** 30 s LINKED window under sustained injection:
  TX 1,168 fps steady, `tx_failed=0`, drops flat, RC frames kept landing
  mid-injection (state held LINKED ⇒ ≥1 valid RCF per 1 s failsafe window
  throughout), `GetTxStats`/`GetThermalStatus` polled every 100 ms tick with
  no stall. No RX starvation, no contention symptoms.
- **Bench observation for E-series at range:** the GS→drone feedback
  direction is the weak side of this bench link (vrx TXes RCF at low
  power/rate defaults; link-up latency varied 2 s–165 s depending on whether
  DISC beaconing was active). For E3/range work, favor forcing the vrx to
  keep beaconing or raise its feedback rate (`--feedback-ms`).

**Remaining after this session:** E1 strict pass (link budget / antenna),
E3 degradation staircase (needs attenuation or the SDR interferer),
path-B-AGC desense A/B at range (`DEVOURER_PROTECT_PATHB_AGC`), commit the
`mabur.mk` 423d286 bump + reflash (or `bw_set` config edit) for the B7
clamp, and the `bundle/install.sh` `json_cli` flag-spelling check (B4
leftover).

## Bench results — 2026-07-12 (late session): E1 STRICT PASS + receiver root-cause

`bw_set` remediated on the drone (`/etc/mabur.json` → `[20]`, restart);
confirmed on air: loss went flat across mod-32 slots (probe-slot spike
gone). Then the day's "link budget problem" got root-caused — it was never
the drone TX or the air link. Two GS-receiver findings:

1. **The RTL8812AU (`0bda:8812`) is a lossy receiver at this bench** —
   5–25% pre-FEC loss with slow multi-second swings at stable RSSI, all
   day, through every antenna/positioning change. Every "air loss" number
   measured through it (including 2026-07-11's ~20%) is AU-RX-bounded, not
   the link. Do not use it for quantitative captures.
2. **🐛 NEW devourer bug — 8822E RX-only bring-up is near-deaf.** The
   8812EU (`0bda:a81a`, same 8822E as the drone's) heard **2 frames/15 s**
   via `rxdemo` (RX-only `Init` path) but **20,143 frames/60 s** via
   `duplex` (TX+RX `InitWrite`+`StartRxLoop` — the same mode maburd uses,
   which is why the drone's RX always worked). Chip is HEALTHY per
   `doctor`; EU TX radiates fine (drone heard a 4k-frame `streamtx` flood).
   Not the path-B AGC ref: `DEVOURER_PROTECT_PATHB_AGC` A/B in RX-only mode
   made no difference (2 frames either way) — so that desense A/B remains
   untested (redo it in TX+RX mode at range). **File upstream to the
   devourer fork.** Bench workaround: capture through `duplex`
   (keep stdin open, e.g. `tail -f /dev/null | duplex`). Minor: `streamtx`
   segfaults at EOF shutdown on Jaguar3 (TX itself completes fine).

- [x] **E1 — clean link. STRICT PASS (2026-07-12, 8812EU + duplex).**
  ~34 s steady-state window at the full stream rate (~1,050 fps received ≈
  TX rate): 35,329 frames, **0 CRC errors**, **stream 0 post-FEC 0.000%,
  stream 1 post-FEC 0.000%** (the naive FRAG-span number shows 20.9% but
  it is entirely one discontinuity during duplex's ~25 s bring-up — the
  steady-state region 49776..54199 is gap-free). Recovered payload
  1.27 Mbps = MAX_RANGE floor ✓, T2/T1 shed ✓. Notes for repeat runs:
  duplex events carry `seq: 0` (no hw seq — use FRAG-level continuity, not
  802.11 seq gaps) and the 24-byte dot11 header is stripped from `body`.
  Cross-receiver RSSI caveat: raw RSSI is not comparable between chips —
  the EU reports lower raw values than the AU while receiving losslessly.

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
  running devourer's `duplex` example (or `rxdemo`). **Receiver choice
  matters (2026-07-12):** the 8812EU (`0bda:a81a`) through `duplex` is the
  lossless reference receiver; `rxdemo` on the EU is near-deaf (devourer
  RX-only-mode bug, see the late-session section), and the 8812AU
  (`0bda:8812`) loses 5–25% regardless of tool. It emits `rx.frame` JSONL
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
| `radio.bw_set` | `[20]` | probe rungs; rungs > `radio.width` are dropped at load (B7, 4252b02) |
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
- [x] **B5 — monitor-mode capture.** DONE (with the bug-4 workaround; see
  above). Receiver = the host's RTL8812AU (`0bda:8812`) running devourer's
  `rxdemo` (`DEVOURER_PID=0x8812 DEVOURER_CHANNEL=149 DEVOURER_STREAM_OUT=1
  DEVOURER_RX_DUMP_ALL=1 DEVOURER_RX_KEEP_CORRUPTED=1 DEVOURER_EVENTS=stdout`),
  no root needed once the kernel driver is detached. With maburd running
  under `DEVOURER_FORCE_PATHB_REF=1`: 19,814 frames / 20 s (~all of the
  ~1100 fps), 99.94% CRC-clean, **rate index 12 = HT MCS0 = the commanded
  MAX_RANGE profile** ✓. Probe schedule: the 20 MHz rungs (`seq%32 ∈ {0,16}`)
  fly (indistinguishable from the default 20 MHz — correct for this config);
  the 40 MHz rung (`seq%32==8`) is aired but unreceivable (B7). NB the
  stock committed binaries still have broken TX until the devourer fix
  lands (bug 4).
- [x] **B6 — kill/restart matrix.** DONE (2026-07-12). maburd restart: soft
  `/etc/init.d/S96mabur restart` (no USB cold-cycle) re-boots FW and streams
  immediately — the clean de-init in the fixed binary lets the chip re-init
  without a VBUS cycle (the earlier need for cold-cycles was an artifact of the
  broken binary's crash-loop leaving the chip wedged). Waybeam restart
  mid-stream: maburd rides through (~6–9 s stall while waybeam re-inits,
  then full-rate resume, no crash). Dongle re-plug (EHCI unbind/bind VBUS
  cycle): maburd dies with the bus, S96 respawn recovers fully within ~3 s
  of rebind. See the 2026-07-12 results section.

### Bench end-to-end (camera + GS PC), the acceptance gate

Each scenario has a pass criterion from the spec's Testing section.

- [x] **E1 — clean link.** STRICT PASS 2026-07-12 via 8812EU + duplex (see
  the late-session section above; post-FEC 0.000% both streams). History
  below is the 2026-07-11 partial (measured through the lossy AU receiver):
  PARTIAL (2026-07-11 first light, receive-only —
  no vrx feedback yet). Full pipeline proven live: drone camera → waybeam →
  maburd → air → host 8812AU rxdemo → SBI unpack → RS decode → FRAG
  reassembly → RTP → **HEVC 2560×1440@25 identified by ffprobe and decoded
  by ffmpeg** (`scratchpad live_decode.py` / `rtp_to_h265.py`; capture:
  `DEVOURER_RX_ZEROCOPY=0 DEVOURER_PID=0x8812 DEVOURER_CHANNEL=149
  DEVOURER_STREAM_OUT=1 DEVOURER_EVENTS=stdout rxdemo`). Numbers (15 s):
  11,652/~15,000 frames captured (~20% pre-FEC air/RX loss at 1 m —
  characteristic of this bench link, matches host→drone flood loss);
  post-FEC: **stream 0 (VPS/SPS/PPS) 0.000% loss**, stream 1 3.6% —
  exactly what k=8 + 0.25 overhead predicts at 20% symbol loss, i.e. the
  FEC is at its design edge. Recovered payload ≈ 1.15 Mbps = the MAX_RANGE
  floor bitrate ✓. **"post-FEC loss = 0" NOT yet met** — needs link-budget
  work (antenna orientation/positioning; RSSI at the receiver is only
  ~32 raw at 1 m, low) before the strict E1 pass. T2/T1 correctly shed
  (streams 2/3 empty in RENDEZVOUS). No DISC/RC frames on air — correct:
  the drone only answers a beaconing GS (E5). **2026-07-12 update
  (reflashed image):** pre-FEC air loss down to 7.6% (3.1 points of which
  is the B7 dead-air probe slot — see the 2026-07-12 section), 100%
  CRC-clean, stream 0 still 0.000%, stream 1 2.9%. Closer, but strict
  "post-FEC 0" still needs link-budget work + the bw_set remediation.
- [x] **E2 — RC frame acceptance.** PASS (2026-07-12). Drone
  RENDEZVOUS→LINKED on GS DISC/RCF, four sessions. The FCS-tail risk did
  NOT materialize — `Packet.Data` carries no trailing FCS on this path; no
  tail-trim needed. Applied-witness on air: drone TX rate followed the
  vrx-commanded rungs (MCS1/MCS2 frames in the GS capture).
- [ ] **E3 — degradation staircase.** Add attenuation / interference (devourer
  has `tests/sdr_interferer.py` for a calibrated co-channel source). GS
  commands down the ladder; observe T2 sheds first, then T1, waybeam bitrate
  follows within ~2 s, video never freezes.
- [x] **E4 — feedback kill → failsafe.** PASS (2026-07-12). GS TX stop →
  FAILSAFE ≤1.05 s (timed at 10 Hz with ±5 ms clock mapping); waybeam
  bitrate at the 1400 floor after entry; resume → re-link (recovery
  confirmed; IDR-on-recovery code-pinned, weak on-air witness due to the
  short-GOP encoder config).
- [x] **E5 — cold rendezvous.** PASS (2026-07-12). maburd (re)booted under
  an already-beaconing GS links in ~2 s (one RENDEZVOUS stats line before
  LINKED), no manual channel setup. DISC path confirmed.
- [x] **E6 — concurrent TX+RX sanity.** DONE (2026-07-12), clean. 30 s
  LINKED under sustained injection: 1,168 fps TX, `tx_failed=0`, RCFs
  accepted mid-injection continuously (state held LINKED the whole window),
  100 ms-tick `GetTxStats`/`GetThermalStatus` never stalled. No RX
  starvation or contention observed.

### Config-edge / known non-blocking items

- [x] **B7 — 40 MHz probes on a 20 MHz device.** ANSWERED (2026-07-11): they
  do NOT work. The frames are aired (seq consumed, HW seq advances) but are
  unreceivable even by a 40 MHz-tuned monitor — a 20 MHz-tuned baseband
  cannot emit a valid 40 MHz PPDU. Net effect: exactly 1/32 of frames
  (probe slot `seq%32==8`) are dead air. **FIXED (mabur 4252b02):** config
  load drops rungs > `radio.width` with a stderr warning; default `bw_set`
  is now `[20]`.
- [ ] **B8 — watchdog edge (only if you change config).** With default
  `failsafe_ms` (1000) < watchdog `stale_ms` (3000) the RX watchdog is safe.
  If you raise `failsafe_ms` above 3000, a quiet channel while LINKED could
  trip the watchdog before failsafe fires. Keep defaults unless you have a
  reason.

## Deferred to a future iteration (not bench-blocking)

- **Devourer follow-ups from the 2026-07-12 bench (file/fix on the fork):**
  1. **Host bring-up is 41 s on the 8822E** (measured, duplex on the GS
     host, timestamps from a clean profile run): power-on+FW 6.5 s →
     BB/AGC/RF register tables **15 s** (~40,300 one-at-a-time USB control
     transfers ≈ 370 µs each) → IQK **13 s** → TXGAPK **6 s** → RX loop at
     41 s. The drone's identical chip through the same code brings up in a
     couple of seconds (embedded EHCI round-trips are ~an order of
     magnitude faster), so the chip doesn't need 41 s — the host is
     per-transfer-latency bound. Improvement: **batch/aggregate the table
     writes** (would collapse the 15 s phase to ~1 s); bench palliatives:
     keep duplex resident instead of restarting per capture, and
     `DEVOURER_DISABLE_IQK=1` saves ~13 s when calibration accuracy doesn't
     matter (monitoring, not measurement).
  2. **8822E RX-only bring-up near-deaf** (the E1-session bug, see the
     late-session section above): `Init`/rxdemo path hears ~nothing while
     `InitWrite`+`StartRxLoop` (duplex/maburd mode) is lossless on the same
     device. Root-cause candidates start at whatever the RX-only path
     skips/adds vs the TX+RX path post-bring-up.
  3. **streamtx segfaults at EOF shutdown on Jaguar3** (TX completes fine;
     crash is in de-init after stdin closes).
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
