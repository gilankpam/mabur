# RC protocol: how the drone and the GS talk

This page is the drone ↔ GS contract on one page: every frame on the air,
its bytes, who sends it and when, how a session forms, and which versions
can talk to each other. It is the map; each topic's detail lives on the page
it links to. A GS implementation that follows this page and the pages it
links can link with `maburd`.

The bytes are defined in `common/` (both sides build against it) and pinned
by the goldens in `tests/test_rc.cpp`. When this page and the code disagree,
the code is right and this page is the bug.

## Two kinds of traffic

| | Control (RC) frames | Data (SBI) bodies |
|---|---|---|
| What | discovery, the session, link feedback, telemetry, NACK, MAVLink | video (H.265, two SVC layers), the probe stream, MSP OSD |
| Direction | both | drone → GS only |
| 802.11 header | 24 B probe request, frame control `0x40 0x00` | 26 B QoS-Data, frame control `0x88 0x00`, No-Ack, TID 0 |
| Body magic | u16 `0x5243` ("RC") | u16 `0xF5B0` (SBI) |
| PHY | HT MCS0, 20 MHz, on the primary of the channel | the ladder rung's MCS and width (`docs/link-adaptation.md`, `docs/bw40.md`) |
| Integrity | one CRC16-CCITT over the frame | a CRC16 per sub-block, then FEC (`docs/airtime-model.md`) |
| Auth | SipHash-2-4 tag on most types (below) | none |

Both headers use broadcast addr1 `ff:ff:ff:ff:ff:ff` and addr2 = addr3 =
`57:42:75:05:d6:00`. A receiver drops a CRC-clean frame from any other
source address as foreign, takes the body at byte 24 for `0x40` and at
byte 26 for `0x88`, and ignores the trailing 4-byte 802.11 FCS. All
multi-byte fields are little-endian, so the RC magic is `43 52` on the air
and the SBI magic is `b0 f5`.

Code: the control header is built in `drone/src/main.cpp` (drone) and
[`gs/src/dot11.cpp`](https://github.com/gilankpam/mabur-ground/blob/master/gs/src/dot11.cpp)
(GS); the QoS-Data header in `drone/src/radio_tx.cpp`.

## RC frames

Every RC frame starts with the same 4-byte prefix and ends with a u16
CRC16-CCITT (`mabur::crc16_ccitt`) over every byte before it:

```
u16 magic 0x5243 | u8 version (the sender's RC_VERSION) | u8 type | ... | u16 crc
```

| Type | Name | Direction | Size (B) | Tagged | Frozen | Cadence | Detail |
|---|---|---|---|---|---|---|---|
| 1 | RCF | GS → drone | 27 | session | no | every `link.feedback_ms` (code 100, bundle 50) in SESSION | `docs/link-adaptation.md`, `docs/tx-rx-timing.md` |
| 2 | DISC | GS → drone | 20 | key | **yes** | 20 ms while searching; 1 s keep-alive otherwise | `docs/link-pairing.md` |
| 3 | DISC_ACK | drone → GS | 26 | key | **yes** | one per DISC heard | `docs/link-pairing.md` |
| 4 | Telem | drone → GS | 59 | no | no | 1 Hz, plus one on session promote | below |
| 5, 6 | — | | | | | deleted (old calibration frames), never reused | |
| 7 | NACK | GS → drone | 21 + 8·n | session | no | on demand, at most one per poll | `docs/fec-nack.md` |
| 8 | `T_MAV` | GS → drone | 20 + len | session | **envelope** | on demand | `docs/mavlink.md` |
| 9 | `T_MAV_UP` | drone → GS | 20 + len | session | **envelope** | ≤ 1 per 50 ms with a published session; calibration replies and the update-only session unpaced | `docs/mavlink.md` |

Sizes are the RC body: prefix through CRC, without the 802.11 header or FCS.

### Version gating

- `rc::frame_type()` returns a frame's type only when its version byte is
  our `RC_VERSION` (19), else -1. Every **non-frozen** type is read through
  it, so an RCF, NACK or Telem from another version is dropped.
- `rc::frozen_type()` returns the type of DISC, DISC_ACK, `T_MAV` and
  `T_MAV_UP` at **any** version byte. Their parsers never read the version.
  The drone hands a `T_MAV` of another version to the MAV thread, which
  accepts it only under an update-only session ("Update-only session"
  below).
- `rc::is_foreign_rc_version()` lets an ingest path keep another version's
  RC frames off the video path and log that the peer is there.

### The tag

Tagged frames carry 8 bytes right before the CRC:

```
tag = SipHash-2-4(key, frame_bytes_before_tag ‖ u32 vrx_nonce ‖ u32 vtx_nonce ‖ u32 seq32)
```

The output is a u64, written little-endian. The key is the 128-bit link key
from `/etc/mabur.key` on both ends (`docs/link-pairing.md` "The key").

- **Key-tagged** (DISC, DISC_ACK): the context is all zero, because no
  session exists yet.
- **Session-tagged**: the context is the session pair plus a 32-bit
  sequence. For RCF that is its u16 `seq` widened by a locally kept wrap
  count; for NACK and MAV it is the frame's own u32 `counter`.

The tag sits at the frame's **structural** offset, worked out from its type
and length fields, never at `len - 10`. The type byte is inside the tagged
bytes, so a frame cannot verify as the other direction. `rc::verify_control`
is the only place a tag is checked.

Replay rules:

| Frame | Receiver accepts |
|---|---|
| RCF | `seq32` strictly above the session's last accepted |
| NACK, `T_MAV` | `counter` strictly above the last accepted under this pair |
| `T_MAV_UP` | each `counter` once, inside a 64-wide window per pair (the drone's two sending threads share one counter and can reach the air out of order) |
| DISC_ACK | its `seq` must echo a DISC the GS sent in the last 2 s |

### Byte layouts

Offsets in bytes. `tag` is 8 bytes; `crc` is u16.

**DISC** (frozen), 20 B
```
0 magic | 2 ver | 3 type=2 | 4 u32 vrx_nonce | 8 u16 seq | 10 tag | 18 crc
```

**DISC_ACK** (frozen), 26 B
```
0 magic | 2 ver | 3 type=3 | 4 u32 vrx_nonce | 8 u32 vtx_nonce | 12 u8 flags |
13 u8 channel | 14 u16 seq | 16 tag | 24 crc
```

- `flags`: bit0 = key mismatch. With it set, `vtx_nonce` is 0 and the tag
  is all zero. Other bits are reserved zero.
- `channel`: the channel the drone is tuned to (the primary of its pair).
- `seq`: an echo of the DISC's `seq`.

**RCF**, 27 B
```
0 magic | 2 ver | 3 type=1 | 4 u8 flags (0) | 5 u16 seq | 7 u8 profile |
8 u8 fec_overhead_base ×100 | 9 u8 fec_overhead_enh ×100 | 10 u8 probe_profile |
11 u8 hop_ch | 12 u8 hop_epoch | 13 u8 rec | 14 u8 idr_epoch | 15 i16 snr_off |
17 tag | 25 crc
```

**Telem**, 59 B (untagged)
```
0 magic | 2 ver | 3 type=4 | 4 u8 flags | 5 u16 tlm_seq | 7 u8 state |
8 u16 rcf_age_ms | 10 u16 rcf_seq_echo | 12 u64 pts_at_build | 20 u32 rcf_rx |
24 u16 cmd_kbps | 26 u32 txq_drops | 30 u16 txq_wait_max_ms | 32 u16 usb_fail |
34 u8 up_rssi[2] | 36 i8 up_snr[2] | 38 i8 soc_temp_c | 39 u16 cpu_busy_x100 |
41 u16 rx_own | 43 u16 rx_foreign | 45 u16 rx_crcfail | 47 u8 rec_status |
48 u16 nack_rx | 50 u16 retx_syms | 52 u16 retx_refused | 54 u16 chip_caps |
56 u8 width | 57 crc
```

- `state` is the drone's `RcAgent::State`: 0 BOOT, 1 RENDEZVOUS, 2 LINKED,
  3 FAILSAFE.
- `flags`:
  - bit0: failsafe shed;
  - bit1: auth reject (at least one GS → drone frame failed verification
    this period);
  - bit3: `rcf_seq_echo` is valid;
  - bit4: congestion shed;
  - bit7: low-power (disarmed) mode.
- `up_rssi` is raw: dBm = v − 110. `soc_temp_c` −128 and `cpu_busy_x100`
  65535 mean unavailable.

**NACK**, 21 + 8·n B
```
0 magic | 2 ver | 3 type=7 | 4 u8 flags | 5 u32 counter | 9 u8 sid | 10 u8 n (1..4) |
11 n × { u32 first_seq | u32 bitmap (bit0 always set) } | tag | crc
```

**`T_MAV` / `T_MAV_UP`** (frozen envelope), 20 + len B
```
0 magic | 2 ver | 3 type (8 or 9) | 4 u32 counter | 8 u16 len (≤ 600) |
10 payload: one or more MAVLink 2 packets | tag | crc
```

The envelope is frozen; the MAVLink messages inside are not. The mabur
dialect (calibration, ids 60000–60005) and the parameter table are in
`docs/mavlink.md` and `docs/vtx-config-mavlink.md`.

### What RCF fields mean

An RCF is the GS's whole wish, repeated: every field is the standing
truth, sent in every RCF. Losing one costs nothing; the next carries the
same values. Events ride an **epoch** that changes once per event, so a
repeat is idempotent.

| Field | Meaning |
|---|---|
| `seq` | the GS's RCF counter (the tag's `seq32` and the link-RTT echo) |
| `profile` | the encode profile (rung) to run |
| `fec_overhead_base` / `_enh` | the FEC overhead per stream, ×100, clamped to 5–200 |
| `probe_profile` | the rung to probe; `0xFF` = no probe stream |
| `hop_ch` / `hop_epoch` | the channel the drone must be on; 0 = no order. A new epoch is a new order, and the drone checks it lands within `link.move_confirm_ms`. This is the **only** way a linked drone changes channel. |
| `rec` | the VTX recorder wish: bit1 "known", bit0 "on"; 0 = leave the recorder alone |
| `idr_epoch` | a GS-requested IDR: one paced IDR per change |
| `snr_off` | phase-lock steering in milli-fps, clamped to ±500 (`docs/phase-lock.md`) |

## SBI bodies (drone → GS data)

Video, the probe stream and the MSP OSD share one body format: a 13-byte
SBI header, then sub-blocks, each with its own CRC so that a damaged body
still yields its good blocks.

```
SBI header, 13 B:
0 u16 magic 0xF5B0 | 2 u8 ver=2 | 3 u8 stream_id | 4 u16 block_payload |
6 u8 n_blocks | 7 u16 q_ms | 9 u16 enc_us | 11 u16 air_ms

then n_blocks × { u16 crc16(payload) | payload (block_payload bytes) }
```

`stream_id`, low 7 bits:

| Value | Stream |
|---|---|
| 0 | video base layer |
| 1 | video enhancement layer |
| 2–3 | reserved for video |
| 4 | MSP DisplayPort OSD |
| 5 | probe stream |

Bit 7 marks a body re-sent for a NACK. The header sits outside the FEC, so
the drone stamps `q_ms` (queue wait), `enc_us` (encoder latency) and
`air_ms` (air backlog) at send time.

For video and MSP, each block payload is one sliding-window FEC envelope:

```
SW header, 14 B: u16 magic 0xF541 | u8 flags (bit0 repair) | u16 symbol_size |
u32 seq (a source's own seq, or a repair's window start) | u8 window_len | u32 repair_key
then symbol_size bytes of symbol
```

- A source symbol carries one fragment: a 6-byte FRAG header (`u16 seq |
  u16 idx | u16 count`), then a chunk of the frame unit.
- The frame unit is an 8-byte `FrameHdr` (`u16 frame_id | u8 flags | u8
  slice_rows | u32 pts_us`) followed by the Annex-B access unit.
- A repair symbol is a GF(256) combination of the window. Its coefficients
  come from `repair_key` by a frozen rule (`common/include/mabur/sw_wire.h`).

The shipped geometry is 4 blocks of 332-byte symbols: 13 + 4 × (2 + 14 +
332) = 1405 B per body.

Probe bodies are SBI bodies, but each block holds a 9-byte probe header
plus deterministic fill instead of an FEC envelope
(`common/include/mabur/probe_wire.h`).

Detail: FEC and frame windows in `docs/airtime-model.md`; slices in
`docs/slices.md`; NACK and retransmit in `docs/fec-nack.md`.

## The session

A session is the pair (`vrx_nonce`, `vtx_nonce`):

- `vrx_nonce` is random for each GS process.
- `vtx_nonce` is picked by the drone for each new `vrx_nonce` it hears. A
  repeated DISC with the same `vrx_nonce` gets the same answer back.

Nothing is persisted on either end.

```
GS (BEACONING)                                   drone (RENDEZVOUS / FAILSAFE)
  DISC every 20 ms, on every searched channel ──►  verify tag (key)
                                              ◄──  DISC_ACK: vtx_nonce, channel, tagged
  verify tag + seq echo;
  same version  → SESSION, move cards to ack.channel
  other version → VERSION_MISMATCH (stop here)
  RCF every feedback_ms, session-tagged        ──►  first verified RCF promotes the pair
                                                    → LINKED, on the channel it already holds
                                              ◄──  video (SBI), Telem 1 Hz, T_MAV_UP
  keep-alive DISC every 1 s (same vrx_nonce)   ──►  same vtx_nonce back
```

The rules:

- **A DISC never moves the drone or links it.** It only gets an ack. The
  first RCF that verifies under the acked pair does the linking.
- **A channel change is always an RCF hop order.** The GS links wherever
  the drone is found, then relocates it with a hop order. The GS side is in
  [`docs/channel-select.md`](https://github.com/gilankpam/mabur-ground/blob/master/docs/channel-select.md).
- **Loss on the drone.** No verified RCF for `link.failsafe_ms` (1000)
  puts it in FAILSAFE and clears both sessions. Its next keep-alive DISC
  re-pairs it with a fresh `vtx_nonce`.
- **Loss on the GS.** No video for `link_lost_ms` (1000) drops SESSION
  back to BEACONING.
- **A restarted GS** brings a new `vrx_nonce`. A linked drone promotes the
  new pair on its first verified RCF without touching the current
  operating point, so a GS crash mid-flight keeps the rung.

### Update-only session

A GS at another `RC_VERSION` cannot link, but it can update the drone
(`docs/mavlink.md` "Over-the-air update"). The drone opens a separate
**update-only pair** for it:

- **Open.** A DISC whose tag verifies and whose version byte is not the
  drone's `RC_VERSION` is answered as always, and its acked pair
  (`vrx_nonce`, `vtx_nonce`) is also stored as the update pair. Every
  such DISC refreshes it; the GS repeats one every second.
- **Allowlist.** Under the update pair the drone accepts only the update
  lane's messages: `MABUR_IDENT_REQ`, `MABUR_UPDATE` and
  `FILE_TRANSFER_PROTOCOL` (the GS-to-drone ids; replies are
  `MABUR_IDENT`, `MABUR_UPDATE_STATUS` and the FTP ACK/NAK). A tagged
  frame carrying anything else is dropped whole and counted as
  `update_dropped=`; it is not an `auth_reject`, since it proves the GS
  holds the key. The counter rule is the `T_MAV` one (strictly above the
  last accepted under this pair).
- **Uplink.** Replies go out as `T_MAV_UP` tagged under the update pair,
  unpaced (no video to protect), whether or not a session is published.
- **Expiry.** The pair lapses 10 s (`RcAgent::kUpdateSessionMs`) after
  the last verified other-version DISC. The drone's other state is
  untouched: no op change, no retune, no LINKED entry.
- **Cleared** by the next normal promotion (the first RCF that verifies
  under a pair at our version), so a GS that is then updated and links
  leaves nothing behind.

GS states (`VrxState`):

| State | Meaning |
|---|---|
| BEACONING | searching |
| SESSION | linked |
| KEY_MISMATCH | a drone in range answers with flagged acks (another key) |
| VERSION_MISMATCH | a verified ack at another `RC_VERSION`; no RCF, NACK, MAV or video |

All four, with their entry and exit rules, are in `docs/link-pairing.md`
("Rendezvous, as built", "The GS's side of the handshake").

## Capabilities

Telem's `chip_caps` tells the GS what this drone build does. `maburd` sets
all four bits today.

| Bit | Name | GS use |
|---|---|---|
| `0x0001` | `CAP_FRAME_WIRE` | video bodies use the frame wire (`FrameHdr` + 6-byte FRAG); no GS reads it |
| `0x0002` | `CAP_TELEMETRY` | the drone sends Telem; display only |
| `0x0004` | `CAP_CALIBRATE` | calibration over the MAVLink cal lane; the GS refuses `maburcal start` without it |
| `0x0008` | `CAP_MAVLINK` | the MAVLink tunnel; logged once per session if missing, gates nothing |

The GS treats caps as unknown until the first Telem of a session
(`peer_caps_known()`).

## Versions: who can talk to whom

`RC_VERSION` (`common/include/mabur/rc_proto.h`, currently 19) is the wire
version. It is not the release version (`v0.MINOR.PATCH`,
`docs/releases.md`). Any `RC_VERSION` bump or `CAP_*` change is a minor
release.

| Drone \ GS | Same `RC_VERSION` | Different, both ≥ 19 | One side < 19 |
|---|---|---|---|
| Result | full link | the handshake completes; the GS shows `UPD?` / `version_mismatch` naming the side to update; no control link and no video | silence: before 19, DISC and DISC_ACK had other layouts |

**The frozen handshake** is the one compatibility promise. It covers:

- the 802.11 header of control frames;
- the RC prefix;
- DISC and DISC_ACK;
- the tag construction;
- the MAV envelope;
- the four mabur update messages (ids 60006-60009);
- the CRC.

These never change under any `RC_VERSION`, not even by appending; a new
need is a new frame type. `docs/link-pairing.md` "Frozen handshake" owns
the rule, and the goldens in `tests/test_rc.cpp` enforce it. Everything
else may change in any release.

## Changing the protocol

1. Change `common/`, and bump `RC_VERSION` with a one-line reason in the
   history comment in `rc_proto.h`. Retired type numbers are never reused.
2. Update the goldens and layout cases in `tests/test_rc.cpp`. A frozen
   golden failing means you broke the frozen handshake: undo it and add a
   new type instead.
3. Update this page and the topic page.
4. Open the mabur PR, saying "contract change: the next release is a minor
   bump". Then open the mabur-ground PR built against it with
   `-DMABUR_DIR`. mabur merges first; mabur-ground then moves its pin.
5. Deploy both ends (`docs/deploy.md`). Until both are swapped, the pair
   shows a version mismatch.

## Where the code lives

| Piece | mabur | mabur-ground |
|---|---|---|
| RC frames, tag, version gates | `common/include/mabur/rc_proto.h`, `common/src/rc_proto.cpp`, `siphash.*`, `link_key.*` | — |
| SBI / FEC / fragments / frame header / probe | `common/include/mabur/sbi.h`, `sw_wire.h`, `frag.h`, `frame_wire.h`, `probe_wire.h`, `uep_encoder.h`, `uep_decoder.h` | — |
| MAVLink | `common/include/mabur/mav.h`, `third_party/mavlink/` | `gs/src/mav_tunnel.*` |
| Drone side | `drone/src/rc_agent.*` (session, RCF apply), `drone/src/main.cpp` (RX callback, Telem, control TX), `drone/src/radio_tx.cpp` (data TX) | — |
| GS side | — | `gs/src/rendezvous.*`, `vrx_controller.*`, `aggregator.cpp`, `dot11.cpp`, `channel_core.*` |
