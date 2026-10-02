# CPE510 remote RX card (`mabur-relay`)

A TP-Link CPE510 (AR9344, ath9k, 2×2 5 GHz panel) running `mabur-relay` acts
as a **remote radio card** for mabur: it hears the drone's downlink on its
panel antenna and forwards every mabur frame (FCS-failed ones included, for
SBI salvage) over Ethernet to a ground station, and — since protocol v3 —
injects uplink RCFs for the owning client too. The web GS consumes it today
(radio picker: USB card | CPE relay — see `docs/web-gs.md`), and since
2026-10-02 so does `maburgs`: `[radio] relays` adds the relay as a
`RemoteCard` next to the USB cards (below).

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
  comparable to Realtek SNR (`snr_units.h` half-dB scale), and not a
  second measurement at all: see "The relay has no SNR" below.
- **No EVM** (ar9003 cannot measure long frames; not carried).
- No FA/CCA/NHM energy reads → the CPE can never be the **scout** card.

## `maburgs` `RemoteCard` (as built, 2026-10-02)

`gs/src/remote_card.{h,cpp}` makes the relay one more `maburgs` radio card
next to the USB cards: in the Aggregator, in the TX selector, a possible
hop lead. A relay-only GS (no USB card) is not supported — the USB scan's
"no supported radio found" exit runs before any relay is counted.

**Config.** `[radio] relays = ["10.83.11.1:8310"]` in `/etc/maburgs.toml`
(default `[]`). Each entry is `ipv4:port` — a numeric dotted IPv4 address
(`inet_pton`; a hostname fails boot, because the UDP transport's
`getaddrinfo` runs on the core thread on every 2 s reopen and a dead
resolver would stall video) — port 1..65535, duplicates rejected; no baked-in default address. Strict keys: an old `maburgs` fails
boot on the key, so config before binary (`docs/deploy.md`).

**Roster.** USB cards first (scan order, or `[[radio.cards]]` order),
indices `0..n_usb-1`; relays follow in config order, `n_usb..n_cards-1`
(`gs/src/main.cpp`, grep `n_usb`). Everything sized off `n_cards` is
unchanged. A `tx_card` pin may name a relay (an explicit `[[radio.cards]]`
list counts relays toward the bounds check), but under auto-scan the
relay's index is `n_usb + k`, so a second USB card appearing shifts it
(a pin past the cards found falls back to auto-select) — a relay pin is
only stable alongside an explicit card list.

**Interface.** `gs/src/link_card.h` `LinkCard` is what `main.cpp` drives
on every card; `RadioFrontend` and `RemoteCard` both implement it, with no
per-card `is_relay` branches. `can_scout()` (energy reads exist) is true
for USB, false for the relay; `relay_stats()` is `nullopt` except on a
relay. `caps()`: chip `ath9k`, gen `CPE510`, 2x2, 20|40, `fast_retune`
false, `fa_ok`/`igi_ok`/`nhm_ok`/`floor_ok`/`snr_ok` all false; the
`scan.log` `C` record logs it like any card. Scout picks are by predicate
(`gs/src/scout_pick.h`): the boot scout is the last scout-capable card and
the in-flight scout the last scout-capable non-TX card, so the relay never
scouts (`docs/channel-select.md`, `docs/inflight-channel-hop.md`).
Scout mode keys on the USB count, not the roster: one USB card is a
one-card scan (it interleaves home windows and beacons DISC itself), and
every `ready()` relay beacons DISC on home *in addition*
(`scan_disc_targets()`); a relay is never the only rendezvous path, since a
CPE that is still booting, unplugged or owned by another client would
otherwise mean no DISC ever leaves the GS. The in-flight hop lead is the
first `ready()` non-TX card of any type (`pick_hop_lead()`; a relay leads
via `TUNE`); with none ready the hop runs the one-card path.

**Lifecycle.** `open_and_start()` opens the UDP transport, re-asserts the
card's current target channel/width, `RelayClient::start()`s and spawns the
RX thread. `alive()` = RX thread running and not `lost()` (no `STATUS` for
2 s): a silent relay is a dead card and goes through the core loop's
existing stop/reopen path, exactly like a USB card that dropped off the
bus. `ready()` = `owned_and_tuned() && !lost()` — owned, `STATUS state` 0,
and the confirmed channel/`sec` equal to the target; a silent relay's last
`STATUS` still reads owned-and-tuned, so lost wins. One stderr line per
transition (`maburgs relay card N (addr): connecting` (each open) / `owned and tuned` / `refused
(another client owns the relay)` / `lost (no STATUS for 2 s)` / `waiting
for STATUS`).

**Not ready = dead card.** Frames that arrive while `!ready()` are counted
(`rx_frames`, `foreign`) and dropped before the `BodyQueue`: a relay tuned
elsewhere by another owner, or still swinging to our `TUNE`, must not feed
the aggregator. `send_control()` while not owned-and-tuned counts `tx_fail` and
sends nothing; since its frames no longer reach the aggregator, the TX
selector's dead-card rule (no frame for 1.5 s, or `!alive()`) moves the
uplink off it.

**Ownership policy — deliberately not the web GS's.** The relay's owner is
the oldest UDP subscriber, else the oldest WebSocket client, so a running
`maburgs` always outranks a web page; `maburgs` can only be refused by
another UDP client (native `webgs --relay`, a second `maburgs`).
`RelayClient` keeps its 2.5 s `TUNE` window; `RemoteCard::tick()` restarts
the client (`HELLO` + `TUNE`, fresh window) every **5 s** while refused or
after ownership was lost mid-session (`kRefusedRestartMs`). A relay that is
both silent and refused reads lost — lost takes precedence over refused, and
the reopen path handles it. The web GS does the opposite on purpose: it
keeps failing Connect with the reason instead of retrying, because it has
an operator in front of it, and a page that silently grabbed the relay later
would start commanding the drone without anyone choosing that. Spotter mode
proceeds non-owned, as before.

**Tuning.** `retune(ch)` / `set_width(ch, w)` (= `retune_width`) record the
target and send `TUNE` at once (`sec` = `mabur::ht40_offset(ch)` at 40 MHz,
0 at 20); they return true, `channel()` reads the commanded channel, and
`ready()` stays false until a `STATUS` confirms it — so every boot-scan
commit, split/reunite, in-flight hop and width resync reaches the relay
through the existing per-card retune loop. Bodies carry the relay's own
`rx_channel` stamp (0 mid-retune), which is what hop confirmation and
`is_link_video` read. TUNE→ready time on hardware: bench leg 5 below.

**Own airtime.** `OwnAirAcc` (the NHM `own_air_pct` input) is fed from the
relay's own frames: width from the relay's confirmed `STATUS` `sec`, never
the commanded width (40 on an unpaired channel tunes 20) and never the
per-frame 40 MHz bit; STBC/SGI from the frame flags, which are valid on the
`phy_valid` frames `OwnAirAcc` latches on. ath9k marks the **last** A-MPDU
subframe `phy_valid` (devourer marks the first), so the latch applies to
the following PPDU — one preamble per PPDU is still counted once, the
per-PPDU attribution is approximate.

**The relay has no SNR.** `../mabur-openwrt/patches/mac80211/999-ath9k-radiotap-antnoise.patch`
leaves ath9k's one per-frame measurement as it is and adds the noise
field next to it:

```c
	rxs->signal = ah->noise + rx_stats->rs_rssi;
+	rxs->noise = ah->noise;
```

`ah->noise` is the per-radio calibrated noise floor, written into every
chain. `relay_client.cpp` computes `snr = signal − noise`, which is just
`rs_rssi`: RSSI above a slowly calibrated floor — one measurement, not two —
and not comparable to Realtek's per-frame PHY SNR. So `CardCaps::snr_ok =
false` on the relay, and every consumer that compares or decides on SNR
across cards excludes it:

- `TxSelector` (`gs/src/tx_selector.h`) compares best-chain **RSSI** on
  every card (3 dB margin, 2 s hold, dead-card rule) — a dated behaviour
  change for USB-only GSes too (`docs/data-provenance.md`, 2026-10-02).
- RF labels / fade predictor (`gs/src/rf_labels.h`): `select_label_card`
  skips `!snr_ok` cards, so `rf_snr_db` never comes from a relay.
- Probe rows (`gs/src/link_health.h`): NaN SNR for a `!snr_ok` card.
- Hop verdict (`gs/src/hop_verdict.h`, `VerdictCardIn::snr_valid`): `weak`
  on a relay best card reads RSSI alone. FA/CCA read 0 and `busy_valid` is
  false; the relay contributes RSSI, foreign and CRC evidence.
- Sideport: `snr`/`snr_a`/`snr_b`/`evm*` are null on a relay card
  (`docs/observability.md`); maburtop and the player OSD draw a dash, and
  the player counts a relay card as heard on RSSI alone.

The aggregator still folds whatever SNR arrives; nothing exports or decides
on it for a `!snr_ok` card. The web GS still displays `RelayClient`'s
derived figure through its own assembler.

**Several relays.** `relays` is a list and the code builds one `RemoteCard`
per entry, but the CPE firmware fixes every unit at `10.83.11.1` with its
own DHCP server, so a second relay needs a `mabur-openwrt` addressing
change first. Only one CPE exists on the bench; multi-relay is covered by
the host tests (`test_remote_card`, `test_config`) only.

### Bench acceptance (spec §6)

Standing gate `tools/bench/ausniff.py`; no `aucadence` (nothing touches the
balancer, venc or UEP). One CPE, so every leg is single-relay. In order:

1. Host `maburgs` build, one USB card + the relay over host Ethernet: relay
   row owned+tuned, both rows hear, ausniff clean, relay `gaps` 0.
   Measured: not yet run (2026-10-02).
2. GS with `relays = []`: USB-only regression, ausniff clean, selector sane.
   Measured: not yet run (2026-10-02).
3. GS + relay (USB-Ethernet adapter on the Radxa): as 1.
   Measured: not yet run (2026-10-02).
4. Uplink: `tx_card` pinned to the relay — RCF-heard vs the 94–98 % bench
   record above; unpinned — attenuate the USB antenna, the selector moves to
   the relay on RSSI and back. Measured: not yet run (2026-10-02).
5. Hop with the relay as lead (USB card TX): hop inject test; record
   TUNE→ready time. If it dwarfs FastRetune, a relay-aware confirm
   allowance is a follow-up. Measured: not yet run (2026-10-02).
6. Failures: Ethernet pulled → dead card → replug → owned again; web page in
   GS mode against the same relay → page reports taken, `maburgs`
   unaffected; CPE reboot onto its default channel → `RemoteCard` re-tunes.
   Measured: not yet run (2026-10-02).
7. Boot scan with the roster: one USB card scouts at 20 MHz in one-card
   mode (beacons in its home windows), the relay beacons DISC on home
   whenever `ready()`; rendezvous time in the usual range, and with the
   CPE unplugged it still rendezvouses on the USB card alone.
   Measured: not yet run (2026-10-02).

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

- **TX alone** (native `webgs live --relay 192.168.1.1:8310` (pre-2026-09-29
  address) `--mode gs --ch
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

- **Multi-relay on hardware**: needs a second CPE and per-device addressing
  in `mabur-openwrt` (every unit ships at `10.83.11.1` with its own DHCP).
- **Relay-aware hop confirm window**, if bench leg 5 shows TUNE→ready
  dwarfing FastRetune.
- **ath9k `noise` as a slow in-band energy sensor** for the relay (does NF
  calibration run in monitor mode? log `noise` next to an interferer).
- **Web GS showing the relay's SNR as "RSSI above floor"** (its own
  assembler).
- **wss for phones at capped rungs** (the mbedTLS test server hung in the
  handshake — solve first).
