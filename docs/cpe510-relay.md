# CPE510 remote RX card (`mabur-relay`)

A TP-Link CPE510 (AR9344, ath9k, 2×2 5 GHz panel) running `mabur-relay` acts
as a **remote radio card** for mabur: it hears the drone's downlink on its
panel antenna and forwards every mabur frame (FCS-failed ones included, for
SBI salvage) over Ethernet to a ground station, injects uplink RCFs for the
owning client, and detects interference on the op channel and sweeps +
ranks the channel set for a hop ("Interference + hop on a relay", below).
Two consumers: the web GS (radio picker: USB card | CPE relay — see
`docs/web-gs.md`) and `maburgs`, where `[radio] relays` adds the relay as a
`RemoteCard` next to the USB cards (below).

| | |
|---|---|
| Firmware repo | `../mabur-openwrt`, <https://github.com/gilankpam/mabur-openwrt> (OpenWrt 25.12.4 ath79, mabur-only image) |
| Daemon source | `feed/net/mabur-relay/src` in that repo (C, libc only, single `poll()` loop) |
| **Wire contract** | `docs/mabur-relay-protocol.md` in that repo — protocol **v4**. Code against that file, not this summary. |
| Bench records | relay side: `docs/verify-mabur-relay-on-device.md` in that repo (flash/boot check, full-rate runs, TX mode); mabur side: `docs/findings/2026-09-28-cpe-relay-rx-spike.md`, `docs/findings/2026-09-29-cpe-relay-tx-wss.md`, `docs/findings/2026-10-02-cpe-relay-remote-card-bench.md`, `docs/findings/2026-10-03-cpe-relay-full-chantable.md`, `docs/findings/2026-10-05-cpe-relay-hop-bench.md` |
| Device | `root@10.83.11.1`; DHCP on the LAN (10.83.11.100-199, no router/DNS — mabur-openwrt `90-mabur-lan`); failsafe 192.168.1.1; the bench unit is hardware revision v3 |
| Ports | UDP **8310** (`maburgs`, native `webgs`), `ws://` **8311** (web GS) |
| Config | `/etc/mabur-relay.conf` (PHY, MON, REG, TXPOWER, BOOT_CHANNEL/SEC, ports); procd service `mabur-relay`, respawns forever |

## Protocol v4 in one paragraph

Clients subscribe with `HELLO` (every 500 ms; lapses after 2 s) and steer the
radio with `TUNE` (channel + `sec`: 0 HT20, 1 HT40+, 2 HT40-). Only the
**owner** (oldest UDP subscriber, else oldest WS client) may tune; others get
`STATUS state=3`. Each forwarded frame is a `FRAME`: a **20-byte header**
(`seq`, `rx_channel` — 0 mid-retune, `sec`, flags, `mcs`, per-chain
`rssi[2]`/`noise[2]` in dBm, `tsf_lo`) followed by the **802.11 frame with
the FCS already stripped**. The relay parses radiotap on the CPE; clients
never see radiotap on the RX side. All fields little-endian. `seq` gaps =
relay→client loss, distinct from air loss in the dot11 seq. `TX`
(type 5) lets the **owner only** hand the relay `mcs + flags + an FCS-less
802.11 frame (24–1500 B)` for injection; anything not from the owner, or
with `mcs` > 7, a reserved flag bit set, or a bad length, is dropped and
counted in `tx_refused` — no per-frame `STATUS` reply, the RCF rate is too
high for that to be anything but spam. `STATUS` carries `tx`, `tx_fail`,
`tx_refused` and `tx_scan_drop` (`u32`, same wrap rule as the rest) after
`uptime_s`. On the relay's own `poll()` loop, client sockets (UDP/WS
control + TX) are read **before** the monitor socket, so an owner's
TX/TUNE is never starved behind a burst of inbound video. `SURVEY` (type 6,
relay → every subscriber, periodic per-channel energy counters) and
`SCAN`/`SCAN_RESULT` (types 7/8, owner-only interference sweep) are covered
in "Interference + hop on a relay", below. `ver` is 4 on every message
(`gs/src/relay_wire.h`); a peer on another version drops every message as
a bad `ver` — there is no compatibility shim.

## ath9k quirks a consumer must respect

- **The CPE must follow the link's width**, not just its channel: tuned
  HT20 it hears almost nothing of an HT40 link. ath9k enables **RX LDPC**
  and **RX STBC** on AR9344, so every mabur rung is decodable once the
  width matches.
- **`phy_valid` (flag bit2) is set only on the last subframe of an A-MPDU.**
  RSSI and the GI/STBC/LDPC/40 MHz bits are meaningful only then; `mcs` is
  valid on every frame. Take the **width from the tuned `sec`**, not the
  per-frame flag.
- Noise floor reads a constant **−95 dBm**, so SNR = RSSI + 95 — not
  comparable to Realtek SNR (`snr_units.h` half-dB scale), and not a
  second measurement at all: see "The relay has no SNR" below.
- **No EVM** (ar9003 cannot measure long frames; not carried).
- No FA/CCA/NHM energy reads → the CPE can never be the **scout** card.
- **Channel set.** Stock ath9k lists only 36–140 and 149–165 (its static
  5 GHz table in `ath9k/common-init.c` predates channel 144). The
  `mabur-openwrt` image carries kmod patch `997-ath9k-full-5ghz-chantable`
  (kmods `-r5`): the phy lists the whole 20 MHz grid **36–177**, and the
  shipped regdb domain `REG=XM` opens all of it with no DFS/NO-IR (NO-IR
  would make mac80211 refuse the relay's TX inject). A CPE on an older
  image cannot tune 144 (TUNE fails with `-22`, the relay stays put); any
  channel in `radio.channels` or hop target the CPE cannot tune drops the
  relay for the session (gracefully: `owned=false`, USB cards carry the
  link). Record: `docs/findings/2026-10-03-cpe-relay-full-chantable.md`.

## `maburgs` `RemoteCard`

`gs/src/remote_card.{h,cpp}` makes the relay one more `maburgs` radio card
next to the USB cards: in the Aggregator, in the TX selector, a possible
hop lead. **A relay-only GS (no USB card) is supported**: `main.cpp`'s
"no supported radio found" exit only fires when the USB scan is empty
**and** `radio.relays` is empty too — an `n_usb = 0` roster is valid
(`cards: no USB radio; running relay-only (N relay)`). A relay-only roster's
boot scan stays search-only (no energy reads to rank with); see
"Interference + hop on a relay" below for what it gets instead.

**Config.** `[radio] relays = ["10.83.11.1:8310"]` in `/etc/maburgs.toml`
(default `[]`). Each entry is `ipv4:port` — a numeric dotted IPv4 address
(`inet_pton`; a hostname fails boot, because the UDP transport's
`getaddrinfo` runs on the core thread on every 2 s reopen and a dead
resolver would stall video) — port 1..65535, duplicates rejected; no baked-in default address.

**Roster.** USB cards first (scan order, or `[[radio.cards]]` order),
indices `0..n_usb-1`; relays follow in config order, `n_usb..n_cards-1`
(`gs/src/main.cpp`, grep `n_usb`). Everything sized off `n_cards` counts
the relays. A `tx_card` pin may name a relay (an explicit `[[radio.cards]]`
list counts relays toward the bounds check), but under auto-scan the
relay's index is `n_usb + k`, so a second USB card appearing shifts it
(a pin past the cards found falls back to auto-select) — a relay pin is
only stable alongside an explicit card list.

**Interface.** `gs/src/link_card.h` `LinkCard` is what `main.cpp` drives
on every card; `RadioFrontend` and `RemoteCard` both implement it, with no
per-card `is_relay` branches. `can_scout()` (energy reads exist) is true
for USB, false for the relay; `relay_stats()` is `nullopt` except on a
relay. `can_sweep()` (a `SCAN` the relay can run) is false for USB,
**statically true** for `RemoteCard`: a relay on another protocol version
cannot speak to the GS at all, so there is no "not yet v4" state to wait
for. `caps()`: chip `ath9k`, gen `CPE510`, 2x2, 20|40, `fast_retune`
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
RX thread. `alive()` = RX thread up AND at least one `STATUS` since
(re)start AND not lost (2 s): a silent relay is a dead card and goes
through the core loop's existing stop/reopen path, exactly like a USB
card that dropped off the bus. `ready()` = `owned_and_tuned() && !lost()`
— owned, `STATUS state` 0, and the confirmed channel/`sec` equal to the
target; a silent relay's last
`STATUS` still reads owned-and-tuned, so lost wins. One stderr line per
transition (`maburgs relay card N (addr): connecting` (each open) / `owned and tuned` / `refused
(another client owns the relay)` / `lost (no STATUS for 2 s)` / `waiting
for STATUS`). An owner mid-retune (a search-burst or hop `TUNE`) keeps
logging as owned for up to 1 s, so a retune does not print a
waiting/owned pair; a `TUNE` that never lands then logs `waiting for
STATUS` once (`RemoteCard::tick()`).

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
`is_link_video` read. TUNE→ready is ≈ 50 ms on hardware, far below
`hop.confirm_ms`, so the hop confirm needs no relay allowance
(`docs/findings/2026-10-02-cpe-relay-remote-card-bench.md`, legs 5 and 7).

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
  every card (3 dB margin, 2 s hold, dead-card rule), USB-only GSes
  included (scale note in `docs/data-provenance.md`).
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
change first. Multi-relay is covered by the host tests
(`test_remote_card`, `test_config`) only.

## Interference + hop on a relay

A GS whose only radio is the CPE gets the same verdict → freshness
burst → ranker → hop flow a one-/two-card USB GS runs
(`docs/inflight-channel-hop.md`): it detects interference on the op
channel, sweeps the channel set over the relay, ranks it and hops. Code:
GS side `gs/src/channel_core.{h,cpp}`, `gs/src/remote_card.{h,cpp}`,
`gs/src/relay_sweep_map.h`, `gs/src/scout_pick.h`,
`gs/src/hop_ranker.{h,cpp}`; relay side `../mabur-openwrt`
`feed/net/mabur-relay/src/`. Design spec:
`docs/superpowers/specs/2026-10-05-cpe-relay-hop-design.md` (gitignored).
Out of scope: a relay never joins the periodic in-flight scout (no
synchronous energy read) and a relay-only boot stays search-only (no
boot-scan measurement over the relay). Bench record and the evidence
behind the metric: `docs/findings/2026-10-05-cpe-relay-hop-bench.md`.

**`SURVEY`/`SCAN` in one line.** `6 SURVEY` is the relay pushing per-channel
hardware cycle counters (`active_ms`/`busy_ms`/`rx_ms`/`tx_ms`,
`ofdm_err`, `foreign`) to every subscriber every 50 ms, cumulative since a
`gen` counter that bumps on every retune and every sweep — it is how a
client reads the **op** channel's condition without owning a scout card.
`7 SCAN`/`8 SCAN_RESULT` is the owner asking the relay to leave the op
channel, dwell every candidate at HT20, and come back — it is how a
relay-only GS gets a ranking for a **candidate** it cannot otherwise
measure (no FA/CCA/NHM reads on this card, ever). Full wire layout:
`../mabur-openwrt/docs/mabur-relay-protocol.md`.

**`busy − rx` is the interferer signature.** `busy` alone cannot tell WiFi
from an interferer (both can read high); `rx` is the relay's own count of
frames it could actually decode, so `busy − rx` is non-decodable airtime —
near 0 for real WiFi, high for anything the relay can hear energy from but
not demodulate (an analog VTX carrier, DJI O4). The hardware cycle counters
**reset when the radio tunes onto a channel**, so a before/after snapshot
across a retune gives negative deltas; `SURVEY`'s `gen`-keyed baseline reads
after the tune instead. **Noise is not a usable signal** (calibration
rejects out-of-range NF; the same channel can read −46 one pass and −95 the
next) — `busy − rx` is the metric, not `ah->noise`.

**Verdict-input mapping** (`VerdictCardIn`, per 150 ms window, from a
relay card — `gs/src/channel_core.cpp`'s per-card verdict loop):

| field | source | feeds |
|---|---|---|
| `fa` | Δ`ofdm_err` (`RemoteCard::read_energy_scout()`) | `raised` |
| `foreign` | Δ`foreign` (`RemoteCard::frames()`) | `contended` |
| `busy_valid`, `nhm_busy_pct` | `SURVEY` delta valid (same `gen`, span ≥ 100 ms); `100·Δbusy/Δactive` | `blocked` |
| `own_air_pct` | `100·Δrx/Δactive` — on the op channel `rx` ≈ our own video | subtracted in `blocked` (decodable foreign WiFi is already `contended`) |
| `rssi`, `snr_valid=false` | RSSI from the FRAME header (no second measurement: "The relay has no SNR" above) | `weak`, `fading` |

`cca` reads 0 on a relay (ath9k has no CCA-event count the relay exposes),
so `raised` on a relay card is `fa` (OFDM/HT PHY errors) alone — and
`ofdm_err` barely moves under a decodable 802.11 jam (decodable frames are
not PHY errors), so in practice relay detection rests on `blocked`
(non-WiFi energy) and `contended` (WiFi).

**Async sweep (spec §4).** On a trigger, `ChannelCore` sends one `SCAN`
for `radio.channels` minus the op channel, `passes = 2`, `observe_ms = 20`
(`kSweepPasses`/`kSweepObserveMs`, `gs/src/channel_core.h`); the relay
dwells every candidate at HT20 both passes before returning. Budget
(4-channel set, 3 candidates): ~40 ms/channel (18 ms retune + 20 ms observe
+ 2 ms reads) × 3 × 2 passes + an 18 ms return ≈ **280 ms the relay is off
the op channel** — link gap on a relay-only GS, since there is no spare
card to carry video meanwhile. Two passes give two visits per candidate
with distinct timestamps, so **one burst ranks the whole set**
(`HopRanker` needs `fresh >= 2`). No result within the sweep timeout —
**`max(1000, passes·n·(observe_ms+40) + 300)` ms**, derived from the
request when the `SCAN` leaves (`n` = channels in the `SCAN`; a 4-member
set sits on the 1000 ms floor, an 8-member set gets 1140 ms) — → the burst
is dropped, `sweep_timeout` logged, `hop.sweep_timeouts` bumped on the
sideport, and a later burst may run again — this is also what covers a
relay reboot or a lost `SCAN_RESULT` mid-sweep.

**`hop.relay_burst_period_ms`** (default 1000, range 500–60000,
`gs/src/config.{h,cpp}`, `gs/bundle/maburgs.default.toml`): `hop_burst_due()`
uses this instead of `hop.burst_period_ms` (333 ms) when the burst card is
a relay — a 280 ms sweep every 333 ms in a sustained `Hold` would leave the
relay deaf ~85 % of the time. The first burst after a trigger is still
immediate, as for a USB burst.

**Burst-card order** (`pick_burst_card()`, `gs/src/scout_pick.h`), first
match wins:

1. scout-capable, not TX (USB spare card — 40 ms, the link card stays on air)
2. sweep-capable, not TX (relay spare — the USB TX card keeps the link while the relay sweeps)
3. scout-capable TX (one-card USB GS's existing acceptance)
4. sweep-capable TX (**relay-only GS**)
5. none → skip (`-1`)

"Sweep-capable" is evaluated **per tick**: a v4 relay (`can_sweep()`)
**that is `ready()`** — owned, tuned, not lost. A relay that is down,
booting or owned by another client therefore never takes the burst, and on
a relay + USB GS with USB as TX the USB card bursts itself (rule 3) instead
of the burst being spent on a dead relay. A `start_sweep()` that still
fails (a ready → down race, or a one-member set with nothing to sweep) does
not spend the burst (`last_burst_ms_` is restored). While a sweep is
pending the TX selector is frozen (`tx_selection_frozen`, like an in-flight
dwell), so TX never moves onto the sweeping relay.

**Relay `HopVisit` mapping** (`gs/src/relay_sweep_map.h`'s `sweep_visit()`):
`fa = ofdm_err`, `foreign = foreign`, `cca = own = 0` (so `fa + 4·foreign`
is the whole score contribution), `busy_valid = true`, `busy_pct = 100·busy/active`
— **raw busy, no rx subtraction** (none of our own frames land on a
candidate, same as the USB ranker's candidate NHM reading) — and `src =
VisitSrc::Relay`. `HopRanker::ranking()` (`gs/src/hop_ranker.cpp`) uses
only the **newest visit's source kind**: a USB 5 ms CCA-event visit and a
relay 20 ms survey visit are different scales, and one burst always covers
the whole candidate set with one card, so a ranking is never built mixing
the two.

**Where the code differs from the design spec** (as-built facts):

1. `TUNE` keeps the relay's `iw` fork/exec (the spec moved it to nl80211
   too); only the sweep and `SURVEY` use nl80211 directly. The relay loop
   tests stub `iw`, and 7 ms on a hop `TUNE` does not matter.
2. `RemoteCard::can_sweep()` is **statically true**, not "true once a v4
   `STATUS` is seen" — a relay on another version cannot speak to the GS
   at all (`ver` mismatch is dropped on both sides), so there is no
   intermediate state to detect.
3. `SURVEY` is **paused during a sweep** (the spec kept it running with
   `gen` bumps mid-sweep); `gen` bumps **once, on return** —
   `survey_rebase()` (`../mabur-openwrt/feed/net/mabur-relay/src/relay.c`)
   runs at startup, in `finish_tune()` and in `finish_sweep()`, never at
   sweep start (`handle_scan()` does not call it), and the SURVEY send
   itself is gated off while `R.sw.active` (`relay.c`'s poll loop). The GS
   already treats a sweeping card as busy/unusable for the op verdict
   regardless (below), so the paused stream costs nothing.
4. While a sweep is pending, `ChannelCore` feeds the hop controller
   `trigger = false` so it cannot enter `hold_exhausted` before the
   `SCAN_RESULT` (or the timeout) lands.
5. The spec's "resync `cur_ch_` from the `SCAN_RESULT`'s `back_channel`"
   was **not implemented**. Harmless: the core never retunes the relay as
   part of a sweep (the relay returns itself), so `cur_ch_` is still right
   after a normal return; a `status 3` return (radio read back elsewhere,
   `STATUS state 2`) is recovered by `RelayClient`'s re-`TUNE`, the same
   path as any failed retune.
6. The sweep timeout is **derived from the request**
   (`max(1000, passes·n·(observe_ms+40) + 300)` ms), not the spec's fixed
   1000 ms — a larger set would otherwise time out mid-sweep.
7. `hop.relay_burst_period_ms`'s minimum is **500**, not 100: under
   ~450 ms a burst can re-fire on the tick the trigger returns after a
   result and starve the controller.
8. Additions beyond the spec: the sideport `hop.sweep_timeouts`
   counter (`docs/observability.md`); the per-tick ready-gated burst pick
   and the TX freeze above; a
   `SURVEY` counter that goes **down within one `gen`** (a fast relay
   restart reusing the gen) is treated by `RemoteCard` as a gen change —
   window invalid, totals take the new sample's full count — instead of
   wrapping u32.

**Other as-built facts found during implementation** (not spec deviations,
behaviour the spec left underspecified):

- **A `SCAN_RESULT` to a WS client is priority** — it evicts a queued video
  frame when that client's WS send queue is full (`relay.c`'s prio
  enqueue; `SURVEY` is not prio and can itself be dropped under load).
- **A sweep that cannot return (`status 3`)**: the relay reads the radio
  back, **adopts** whatever channel that read shows, sets `STATUS state =
  2` and broadcasts it — the GS's `RelayClient` sees the mistune and
  re-`TUNE`s, the same recovery path an ordinary failed retune uses.
- **The relay forgets a sweep's requester if that client is reaped or its
  slot is reused** mid-sweep (UDP 2 s silence, WS socket closed): the
  finished sweep's result is computed and then simply dropped — nobody
  claims it, and a new client reusing the same subscriber slot never sees
  a stale result land on it.
- **`RemoteCard::on_survey_()`** (`gs/src/remote_card.cpp`): the **first**
  `SURVEY` after (re)connect is a baseline only — its counters are the
  relay's backlog for that `gen`, not this session's traffic, so nothing
  is added to the cumulative OFDM-error/foreign totals. A `gen` change
  mid-session (a sweep, or another client's retune) instead adds the **new
  gen's full counts** — there is no prior sample in that `gen` to delta
  against.
- **`ChannelCore`'s op verdict skips a card only while its own sweep is
  pending** (`sweep_.on && sweep_.card == i`), deliberately **not**
  `RemoteCard::sweeping()`: a lost `SCAN_RESULT` leaves the card-level flag
  on past `ChannelCore`'s own sweep timeout, and skipping the op verdict
  on it for good would blind a one-card (relay-only) GS permanently. The
  timeout (expiry) is checked **before** the result arrival, so a result
  that lands late loses to the timeout rather than reviving a dropped
  sweep.

## Measured limits (full rate, mcs4/40, ~3.2k frames/s, 36 Mb/s)

| Subscriber | CPE CPU | Loss |
|---|---|---|
| one `maburgs` over UDP | **58 %** | 0 |
| one browser over WebSocket | **71 %** | 0 |

The relay is **send-bound** (the AR9344's Ethernet has no checksum
offload), which is why the `FRAME` header is compact (no raw radiotap: that
fragmented almost every datagram) and the relay sends with `SO_NO_CHECK` in
batches. UDP and WebSocket at the same time (93 %) is not a real use case.
Load scales with video bitrate + FEC; the next lever, if ever needed, is a
`PACKET_MMAP` receive ring (RX costs ~18 pts). TX traffic (~20 RCF/s) adds
nothing measurable (`docs/findings/2026-09-29-cpe-relay-tx-wss.md`).

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

## TX mode

The relay builds the 13-byte radiotap header itself (TX_FLAGS NOACK + MCS)
around the client's `mcs + flags + FCS-less dot11` payload and injects on
`mon0` — clients never construct radiotap. Only the current tune owner's
`TX` is honoured; a non-owner's, or one with `mcs` > 7 / a reserved flag bit
/ a bad length, is silently dropped into `tx_refused` (no `STATUS` echo per
frame, see above).

**Own-echo filter.** Every injected frame comes back on `mon0` **twice**:
AF_PACKET's `PACKET_OUTGOING` loopback copy and mac80211's TX-status report,
both carrying radiotap `TX_FLAGS` (bench: `txecho == 2 × tx` exactly).
The relay filters both by
that radiotap bit before treating anything as inbound video — without the
filter, every RCF the GS itself sends would loop back and be forwarded to
the client a second time as if it were drone RX.

mabur's client side is `gs/src/relay_wire.{h,cpp}` (the pure v4 codec) and
`gs/src/relay_client.{h,cpp}` (`RelayClient`, lib `mabur_gs_relay`):
HELLO every 500 ms; `TUNE` retried every 500 ms while not owner, but only
within a 2500 ms window of connecting (after that, a persistent refusal is
reported rather than retried forever); once owner but read back mistuned
(the relay rebooted onto its own default channel, etc.) `TUNE` is retried
every 500 ms with **no** window — an owner never gives up tuning its own
radio. `lost` = no `STATUS` for 2 s. `seq` gap tracking only counts forward
jumps (a reorder or a relay-side reset resyncs quietly rather than counting
a spurious gap).

Bench record (TX alone 98.1 % RCF-heard; relay ≥ USB in the A/B; the
native `webgs --relay` CLI used for it):
`docs/findings/2026-09-29-cpe-relay-tx-wss.md`.

## Why not wss

TLS on the CPE costs ~16 CPU points per MB/s (mbedTLS ChaCha20-Poly1305;
AES-GCM is far slower on the 74Kc), and the relay's WS path is already
71 % at full rate (4.5 MB/s), so `wss://` cannot carry the top rungs. The
web GS doesn't need it: Chrome 142+ lets an `https://` page (the hosted
GitHub Pages build) open plain `ws://` to a private IP literal or `.local`
name once the user allows local network access — see `docs/web-gs.md`,
"CPE relay radio". Spike record: `docs/findings/2026-09-29-cpe-relay-tx-wss.md`.

## Follow-ups

- **Multi-relay on hardware**: needs a second CPE and per-device addressing
  in `mabur-openwrt` (every unit ships at `10.83.11.1` with its own DHCP).
- **Web GS showing the relay's SNR as "RSSI above floor"** (its own
  assembler).
- **wss for phones at capped rungs** (the mbedTLS test server hung in the
  handshake against OpenSSL clients — solve first).
- `RemoteCard::tick()` should log `tune failed` (RelayClient::tune_failed /
  STATUS state 2) instead of `waiting for STATUS` when the relay refuses
  the channel; the sideport `relay.state` already carries it.
- **~250 ms lost between `SCAN_RESULT` and the order** on every relay hop:
  the trigger is held off while the sweep is pending and the verdict then
  needs 2 interfered windows again. Keeping the trigger latched across the
  sweep would bring onset → video to ~0.8 s.
- **Relay-only auto boot relocates to `channels[0]`** after "link where
  found" with no measurement behind it — a pointless hop on each
  relay-only boot.
- **The web page shows a ~1.5 s video gap after a relay hop** where
  `maburgs` resumes within ~300 ms — likely the page's key-frame wait after
  a channel change.
