# CPE510 remote RX card (`mabur-relay`)

A TP-Link CPE510 (AR9344, ath9k, 2×2 5 GHz panel) running `mabur-relay` acts
as a **remote receive card** for mabur: it hears the drone's downlink on its
panel antenna and forwards every mabur frame (FCS-failed ones included, for
SBI salvage) over Ethernet to a ground station. It is built and flashed; the
**mabur-side consumer does not exist yet**.

| | |
|---|---|
| Firmware repo | `../poc/wfb-ng-openwrt` (OpenWrt 25.12.4 ath79, image is mabur-only — wfb-ng removed) |
| Daemon source | `feed/net/mabur-relay/src` in that repo (C, libc only, single `poll()` loop) |
| **Wire contract** | `docs/mabur-relay-protocol.md` in that repo — protocol **v2**. Code against that file, not this summary. |
| Bench record | `docs/verify-mabur-relay-on-device.md` in that repo (flash/boot check, v1 + v2 full-rate runs) |
| Device | `root@192.168.1.1` (static, no DHCP); the bench CPE is a v3 |
| Ports | UDP **8310** (`maburgs`), `ws://` **8311** (web GS) |
| Config | `/etc/mabur-relay.conf` (PHY, MON, REG, TXPOWER, BOOT_CHANNEL/SEC, ports); procd service `mabur-relay`, respawns forever |

## Why it works (spike, 2026-09-28)

- ath9k enables **RX LDPC** and **RX STBC** on AR9344, so every mabur rung is
  decodable.
- The CPE must follow the link's **width**, not just its channel: tuned
  `136 HT20` it heard almost nothing of an HT40 link; `136 HT40-` heard
  15,415 video frames in 5 s at 0.21 % dot11-seq loss.

## Protocol v2 in one paragraph

Clients subscribe with `HELLO` (every 500 ms; lapses after 2 s) and steer the
radio with `TUNE` (channel + `sec`: 0 HT20, 1 HT40+, 2 HT40-). Only the
**owner** (oldest UDP subscriber, else oldest WS client) may tune; others get
`STATUS state=3`. Each forwarded frame is a `FRAME`: a **20-byte header**
(`seq`, `rx_channel` — 0 mid-retune, `sec`, flags, `mcs`, per-chain
`rssi[2]`/`noise[2]` in dBm, `tsf_lo`) followed by the **802.11 frame with
the FCS already stripped**. The relay parses radiotap on the CPE; clients
never see radiotap. All fields little-endian. `seq` gaps = relay→client loss,
distinct from air loss in the dot11 seq.

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
(the web GS already uses it). A `RemoteCard` would: receive v2 `FRAME`s →
`crc_err` = flags bit0, `data_rate` from `mcs` (convert to the devourer
`DESC_RATE` code, `0x0C + mcs` for HT), `rssi[]`/`snr[]` from rssi/noise,
`physt` = bit2, `tsfl` = `tsf_lo` → `fill_rx_body()` on the dot11 bytes →
`BodyQueue`. It also has to issue `TUNE` whenever the link moves (boot-scan
pick, in-flight hop, width change) and stamp `RxBody::rx_channel` from the
frame's `rx_channel`. The same mapping serves the web GS over WebSocket
(Spotter only today — see TX mode below).

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
  `ssh-keygen -R 192.168.1.1`.
- For a full-rate bench run the drone's FC reports DISARMED, so set
  `[low_power] enable = false` in `/etc/mabur.toml` temporarily (restore
  after).
- Building the firmware on this host: IPv6 to downloads.openwrt.org is
  broken, which stalls the ImageBuilder's wget; run the image stage with an
  IPv4-only `WGETRC` (see the relay repo README).

## Follow-ups

- **TX mode:** accept `TX` (message type 5, reserved and currently ignored)
  from the owner and inject it on `mon0`, so the web GS can run in *GS mode*
  and `maburgs` can send its uplink through the panel. Open risk: RCF slot
  timing jitter over the network hop — measure RCF delivery on the bench.
- **`RemoteCard`** in `maburgs` and a WebSocket source in the web GS.
