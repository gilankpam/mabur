# CPE510 remote RX card (`mabur-relay`)

A TP-Link CPE510 (AR9344, ath9k, 2×2 5 GHz panel) running `mabur-relay` acts
as a **remote radio card** for mabur: it hears the drone's downlink on its
panel antenna and forwards every mabur frame (FCS-failed ones included, for
SBI salvage) over Ethernet to a ground station, and — since protocol v3 —
injects uplink RCFs for the owning client too. The web GS consumes it today
(radio picker: USB card | CPE relay — see `docs/web-gs.md`); **`RemoteCard`
in `maburgs` does not exist yet**.

| | |
|---|---|
| Firmware repo | `../mabur-openwrt`, <https://github.com/gilankpam/mabur-openwrt> (OpenWrt 25.12.4 ath79, mabur-only image) |
| Daemon source | `feed/net/mabur-relay/src` in that repo (C, libc only, single `poll()` loop) |
| **Wire contract** | `docs/mabur-relay-protocol.md` in that repo — protocol **v3**. Code against that file, not this summary. |
| Bench record | `docs/verify-mabur-relay-on-device.md` in that repo (flash/boot check, v1/v2 full-rate runs, TX mode) |
| Device | `root@10.83.11.1`; DHCP on the LAN (10.83.11.100-199, no router/DNS — mabur-openwrt `90-mabur-lan`); failsafe 192.168.1.1; the bench CPE is a v3 |
| Ports | UDP **8310** (`maburgs`, native `webgs`), `ws://` **8311** (web GS) |
| Config | `/etc/mabur-relay.conf` (PHY, MON, REG, TXPOWER, BOOT_CHANNEL/SEC, ports); procd service `mabur-relay`, respawns forever |

## Why it works (spike, 2026-09-28)

- ath9k enables **RX LDPC** and **RX STBC** on AR9344, so every mabur rung is
  decodable.
- The CPE must follow the link's **width**, not just its channel: tuned
  `136 HT20` it heard almost nothing of an HT40 link; `136 HT40-` heard
  15,415 video frames in 5 s at 0.21 % dot11-seq loss.

## Protocol v3 in one paragraph

Clients subscribe with `HELLO` (every 500 ms; lapses after 2 s) and steer the
radio with `TUNE` (channel + `sec`: 0 HT20, 1 HT40+, 2 HT40-). Only the
**owner** (oldest UDP subscriber, else oldest WS client) may tune; others get
`STATUS state=3`. Each forwarded frame is a `FRAME`: a **20-byte header**
(`seq`, `rx_channel` — 0 mid-retune, `sec`, flags, `mcs`, per-chain
`rssi[2]`/`noise[2]` in dBm, `tsf_lo`) followed by the **802.11 frame with
the FCS already stripped**. The relay parses radiotap on the CPE; clients
never see radiotap on the RX side. All fields little-endian. `seq` gaps =
relay→client loss, distinct from air loss in the dot11 seq. New in v3: `TX`
(type 5) lets the **owner only** hand the relay `mcs + flags + an FCS-less
802.11 frame (24–1500 B)` for injection; anything not from the owner, or
with `mcs` > 7, a reserved flag bit set, or a bad length, is dropped and
counted in `tx_refused` — no per-frame `STATUS` reply, the RCF rate is too
high for that to be anything but spam. `STATUS` gained three `u32` counters
(`tx`, `tx_fail`, `tx_refused`) after `uptime_s`, same wrap rule as the rest.
On the relay's own `poll()` loop, client sockets (UDP/WS control + TX) are
read **before** the monitor socket, so an owner's TX/TUNE is never starved
behind a burst of inbound video.

## ath9k quirks a consumer must respect

- **`phy_valid` (flag bit2) is set only on the last subframe of an A-MPDU.**
  RSSI and the GI/STBC/LDPC/40 MHz bits are meaningful only then; `mcs` is
  valid on every frame. Take the **width from the tuned `sec`**, not the
  per-frame flag.
- Noise floor reads a constant **−95 dBm**, so SNR = RSSI + 95 — not
  comparable to Realtek SNR (`snr_units.h` half-dB scale).
- **No EVM** (ar9003 cannot measure long frames; not carried).
- No FA/CCA/NHM energy reads → the CPE can never be the **scout** card.

## Mapping to `maburgs` (`RemoteCard`, not built)

`gs/src/dot11.h` `RxMeta` + `fill_rx_body()` is the device-agnostic seam
(the web GS already uses it via `gs/src/relay_client.{h,cpp}`). A
`RemoteCard` would: receive v3 `FRAME`s → `crc_err` = flags bit0,
`data_rate` from `mcs` (convert to the devourer `DESC_RATE` code, `0x0C +
mcs` for HT), `rssi[]`/`snr[]` from rssi/noise, `physt` = bit2, `tsfl` =
`tsf_lo` → `fill_rx_body()` on the dot11 bytes → `BodyQueue`. It also has to
issue `TUNE` whenever the link moves (boot-scan pick, in-flight hop, width
change) and stamp `RxBody::rx_channel` from the frame's `rx_channel`, and —
now that TX exists — build outbound RCFs into `TX` messages the same way the
web GS's `RelayLink` already does (see TX mode, below).

## Measured limits (full rate, mcs4/40, ~3.2k frames/s, 36 Mb/s)

| Subscriber | CPE CPU | Loss |
|---|---|---|
| one `maburgs` over UDP | **58 %** | 0 |
| one browser over WebSocket | **71 %** | 0 |

The relay is **send-bound** (the AR9344's Ethernet has no checksum offload):
v1's raw-radiotap design fragmented 96 % of datagrams and cost 65 % / 95 %;
v2's compact header + `SO_NO_CHECK` + batched sends brought it to the
numbers above. UDP and WebSocket at the same time (93 %) is not a real use
case. Load scales with video bitrate + FEC; the next lever, if ever needed, is
a `PACKET_MMAP` receive ring (RX costs ~18 pts).

## Ops notes

- The GS (Radxa ZERO 3) has no Ethernet — it needs a USB-Ethernet adapter to
  talk to the CPE.
- After a `sysupgrade -n` the CPE's SSH host key changes:
  `ssh-keygen -R 10.83.11.1`.
- For a full-rate bench run the drone's FC reports DISARMED, so set
  `[low_power] enable = false` in `/etc/mabur.toml` temporarily (restore
  after).
- Building the firmware on this host: IPv6 to downloads.openwrt.org is
  broken, which stalls the ImageBuilder's wget; run the image stage with an
  IPv4-only `WGETRC` (see the relay repo README).

## TX mode (as built, protocol v3, 2026-09-29)

The relay builds the 13-byte radiotap header itself (TX_FLAGS NOACK + MCS)
around the client's `mcs + flags + FCS-less dot11` payload and injects on
`mon0` — clients never construct radiotap. Only the current tune owner's
`TX` is honoured; a non-owner's, or one with `mcs` > 7 / a reserved flag bit
/ a bad length, is silently dropped into `tx_refused` (no `STATUS` echo per
frame, see above).

**Own-echo filter.** Every injected frame comes back on `mon0` **twice**:
AF_PACKET's `PACKET_OUTGOING` loopback copy and mac80211's TX-status report,
both carrying radiotap `TX_FLAGS`. Bench-confirmed: `txecho == 2 × tx`
exactly (2422 vs 1211 in the TX-alone run below). The relay filters both by
that radiotap bit before treating anything as inbound video — without the
filter, every RCF the GS itself sends would loop back and be forwarded to
the client a second time as if it were drone RX.

mabur's client side is `gs/src/relay_wire.{h,cpp}` (the pure v3 codec) and
`gs/src/relay_client.{h,cpp}` (`RelayClient`, lib `mabur_gs_relay`):
HELLO every 500 ms; `TUNE` retried every 500 ms while not owner, but only
within a 2500 ms window of connecting (after that, a persistent refusal is
reported rather than retried forever); once owner but read back mistuned
(the relay rebooted onto its own default channel, etc.) `TUNE` is retried
every 500 ms with **no** window — an owner never gives up tuning its own
radio. `lost` = no `STATUS` for 2 s. `seq` gap tracking only counts forward
jumps (a reorder or a relay-side reset resyncs quietly rather than counting
a spurious gap).

### Bench record — TX mode, 2026-09-29 (CPE v3, `mabur-openwrt` 44f0190)

Setup: CPE on host USB-Ethernet (192.168.1.101 ↔ 192.168.1.1) (pre-2026-09-29
address); drone `.152`
on ch136 HT40-. Full numbers and the per-window breakdown are in the relay
repo's `docs/verify-mabur-relay-on-device.md` ("TX mode" section); this is
the summary.

- **TX alone** (native `webgs live --relay 192.168.1.1:8310 --mode gs --ch
  136 --w 40 --secs 60`, no other GS on air, drone `low_power` on): SESSION
  + `peer_acked` within the first second, ladder climbed to rung 4 (mcs4/40)
  by ~15 s. `drone_rcf_rx / rcf_sent` = 1129 / 1151 = **98.1 %**; relay
  `seq` gaps 0; relay `tx` 1211, `tx_fail` 0, `tx_refused` 0; RTT ~5–7 ms.
  Proves ath9k honours the injected MCS0 + LDPC + STBC combination (the
  drone decodes it) and confirms the own-echo filter (`txecho` = 2422 =
  2 × 1211, kept).
- **A/B vs. USB + CPE load** (drone `low_power` off, full rate, ch136/40,
  mcs4; 4 legs alternating USB `maburgs`-on-Radxa and the CPE relay via
  native `webgs --relay`): RCF-heard 94.5 % / 94.3 % over USB vs. 100.4 % /
  100.7 % over the relay (after the first 10 s) — relay ≥ USB, inside the
  ±5-point bar (the >100 % readings are telemetry-counter lag at window
  edges: USB's denominator counts every GS TX frame, the relay's counts
  RCFs only). At the same time the CPE carried full-rate video
  (3170–3250 frames/s over UDP) plus ~20 TX/s: CPU 53.5–59.6 % (5 s
  windows, ≤ the 75 % gate), `relay_gaps` 0, `rxdrop` 0, `tx_fail` 0, RTT
  7.5–8.7 ms — the same as the RX-only UDP baseline (58 %, see the table
  below): TX cost on the relay is negligible.
- **Not measured on hardware**: the browser path (page → WebSocket → relay
  Worker → ring → core) — no browser in the bench session. The native CLI
  exercises the same `RelayLink`/`RelayClient` over UDP instead. WS-mode
  full-rate CPE load was 71 % RX-only in an earlier run (see the table
  below); not re-measured with TX traffic added.

## Why not wss (spike, 2026-09-29)

Measured on the bench CPE (AR9344, 74Kc 560 MHz), relay idle:

| | result |
|---|---|
| mbedTLS 3.6.7 (device lib) cipher | ChaCha20-Poly1305 14.3 MB/s, AES-128-GCM 1.3 MB/s |
| OpenSSL 3.5 cipher | ChaCha 15.4 MB/s, AES-GCM 5.5 MB/s |
| one stream, plain TCP, 4.5 MB/s / 2 MB/s | 22 % / 10 % CPU |
| one stream, TLS ChaCha, 4.5 MB/s / 2 MB/s | 96 % / 42 % CPU (saturates 4.8 MB/s) |

TLS costs ~16 CPU points per MB/s. The relay's WS path is already 71 % at
full rate (4.5 MB/s), so wss cannot carry the top rungs. The mbedTLS test
server also hung in the handshake against OpenSSL clients (not debugged).
The web GS doesn't need it anyway: Chrome 142+ lets an `https://` page
(the hosted GitHub Pages build) open plain `ws://` to a private IP literal
or `.local` name once the user allows local network access — see
`docs/web-gs.md`, "CPE relay radio".

## Follow-ups

- **`RemoteCard`** in `maburgs`: receive `FRAME`s the way `RelayClient`
  already does for the web GS, so a native GS box can also run off the
  panel instead of its own USB cards.
- **wss for phones at capped rungs** (the mbedTLS test server hung in the
  handshake — solve first).
