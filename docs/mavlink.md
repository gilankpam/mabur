# MAVLink tunnel

The radio link carries MAVLink 2 packets in both directions inside one RC
frame type, `T_MAV`. On the drone a MAVLink component answers on it. On
the GS, `maburgs` bridges the tunnel to a MAVLink UDP endpoint, so a
ground app talks to the drone as if it were a MAVLink system on the
network; the web GS runs its own ground client on the tunnel. This page is
the tunnel and what its tenants share; each tenant's protocol has its own
page.

- **VTX config** (`docs/vtx-config-mavlink.md`): the component serves
  fourteen drone config parameters as MAVLink extended parameters
  (`PARAM_EXT_*`). A ground end holding the link key can list, read and
  set them while linked; a set applies to the running `maburd` before it
  is published to `/etc/mabur.toml`. `tools/bench/mavparam.py` and the web
  GS's VTX tab drive it.
- **Calibration** (`docs/calibration.md`): `maburgs`' calibration session
  and the drone's sweep engine talk over the same tunnel in a mabur
  MAVLink dialect, on a lane of their own that survives the session lapse
  every sweep causes. See "Calibration over the tunnel" below.
- Designed for, not built: "Later tenants" below.

## Where the code lives

| Piece | File |
|---|---|
| `T_MAV` frame, `CAP_MAVLINK` | `common/include/mabur/rc_proto.h` |
| the mabur dialect's XML (`mabur.xml`, including `common.xml`) | `common/mavlink/` |
| the vendored generated headers (mabur, common, minimal, standard) | `third_party/mavlink/` (pymavlink version, regenerate command and the `protocol.h` patch in its `VERSION`), included through `common/include/mabur/mav.h` |
| calibration messages: plain structs and their MAVLink codec | `common/include/mabur/cal_proto.h`, `common/include/mabur/cal_mav.h` |
| drone: the component (routing, heartbeat; the parameter protocol in `docs/vtx-config-mavlink.md`) | `drone/src/mav_component.h` |
| drone: MAV thread, tag and counter check | `drone/src/main.cpp` (`mbr-mav`), `RcAgent::check_mav` |
| GS: the tunnel and its cal lane (shared GS core) | `gs/src/mav_tunnel.h` |
| GS: the UDP endpoint | `gs/src/mav_endpoint.h` |

## The wire: `T_MAV`

`T_MAV = 8`, `RC_VERSION` 18. Same layout both directions; only the GS to
drone direction carries the tag:

```
u16 magic "RC" | u8 version | u8 type = T_MAV |
u32 counter     GS -> drone: per-session, starts at 1, strictly increasing
                drone -> GS: per-process; the GS drops a frame repeating
                the last counter (the second card's copy)
u16 len         payload byte count, at most kMavMaxPayload (600)
u8[len]         one or more whole MAVLink 2 packets, back to back
[u8[8] tag]     GS -> drone only: SipHash over the bytes before it || TagCtx
u16 crc16
```

- A packet never spans two frames. Both ends pack as many whole packets as
  fit in 600 bytes (three 161-byte `PARAM_EXT_VALUE`s, or a 21-byte
  heartbeat plus three).
- The RC layer treats the payload as opaque. Each end splits it with a
  MAVLink parser; a bad MAVLink CRC (which is also what refuses a message
  id the mabur dialect does not know), a MAVLink 1 packet, or a packet
  cut off at the end of a payload is dropped and counted. MAVLink 2 only.
- **GS to drone** is tagged with `TagCtx{vrx_nonce, vtx_nonce, seq32 =
  counter}`, the NACK rule (`docs/link-pairing.md`). The drone accepts a
  frame only under its published session and only with a counter greater
  than the last one it accepted in that session. A frame that fails the
  tag or the counter, or arrives with no published session, is dropped
  whole and raises `auth_reject` (Telem flags bit1); a frame that fails to
  parse is dropped and counted without it. The GS restarts its counter at
  1 when the session's `vtx_nonce` changes. The one exception is a frame
  made entirely of calibration messages: its own counter domain, and a
  latch onto the calibration run's pair (below).
- **Drone to GS** is untagged, like `T_TELEM`, and goes out on the Telem
  path (control radiotap, the shared Telem 802.11 sequence) at most one
  frame every `kMavUplinkMinMs` (50 ms), which caps the tunnel near
  12 kB/s on the drone's uplink. Nothing from the component goes up while
  a calibration sweep airs (the periodic Telem's rule); replies wait
  queued. The calibration replies are the exception (below). A two-card
  GS receives each frame once
  per card: the tunnel drops a frame whose counter equals the last one it
  took, and forgets that counter when the session's `vtx_nonce` changes
  (a restarted drone counts from 0 again).
- The GS's other card also hears the GS's own tagged `T_MAV`. The tunnel
  tries the tagged layout first and counts a match apart, never as a
  reject and never delivered: an untagged frame is too short to parse as
  tagged, even with the trailing 4-byte FCS.
- **GS to drone** frames are offered to the RCF slotter like an RCF, so
  they land in the drone's inter-AU idle, and are never sent while a
  calibration sweep holds the GS radio-silent. A frame goes out only when
  a ground app has sent something, or the calibration session has a
  message due (its own lane, below).
- `CAP_MAVLINK` (`0x0008`) in DISC_ACK `chip_caps` says the drone answers
  `T_MAV`. It is display-only: with `[mavlink] enable`, `maburgs` logs
  `maburgs: mavlink: drone lacks CAP_MAVLINK` once per session when the bit
  is missing, and it gates nothing on it.

## Ids

- The drone's component answers as system 1, component 191
  (`MAV_COMP_ID_ONBOARD_COMPUTER`). A flight controller on the same drone
  would be component 1, the MAVLink convention.
- Ground senders use their own ids (QGroundControl sends 255/190).
  `maburgs` forwards a ground app's bytes unchanged in both directions;
  the packets it originates itself -- calibration's, and its hosted
  parameter client's (`docs/vtx-config-mavlink.md` "Ground client") --
  it sends as 255/190, same as any other ground app.

## The drone component

The MAV thread (`mbr-mav`) owns the component: it takes `T_MAV` frames off
their own queue, checks the tag and counter, feeds the payload in, and
sends queued replies. The config write fsyncs flash, so it stays off the
agent and TX threads.

- **Heartbeat.** Once a second while a session is published:
  `MAV_TYPE_ONBOARD_CONTROLLER`, `MAV_AUTOPILOT_INVALID`,
  `MAV_STATE_ACTIVE`. Suppressed while a calibration sweep airs, the
  same rule as the periodic Telem. This is what a ground app discovers
  the component by.
- **Routing.** A frame made entirely of calibration messages (ids
  60000-60005) never reaches the component: its messages go to the TX
  writer thread's calibration queue. Any other frame goes whole to the
  component. A message is for the component when its target system is 0
  or 1 and its target component is 0 or 191. Served:
  `HEARTBEAT` (ignored), `PARAM_EXT_REQUEST_LIST`,
  `PARAM_EXT_REQUEST_READ`, `PARAM_EXT_SET`. Anything else (a calibration
  id in a mixed frame included), or anything for another target, is
  dropped and counted.
- **Parameters.** List, read and set, the apply paths and what
  `ACCEPTED` promises: `docs/vtx-config-mavlink.md`.
- **Outbound queue.** At most 64 packets wait for the uplink; the cap
  applies to every reply and the heartbeat alike. A packet that would
  exceed it is dropped and counted in the `stats:` line's `drop`
  (`docs/observability.md`), never one already queued; the ground app's
  retry recovers it.

## The GS endpoint

`maburgs` opens one UDP socket when `[mavlink] enable`:

```toml
[mavlink]
enable = true
bind   = "127.0.0.1"   # the GS's LAN address to let a ground app on the LAN connect
port   = 14550         # MAVLink's conventional GCS port
```

`bind` is a dotted IPv4 address; `port` is 1..65535. A port that cannot be
bound is a boot warning, not fatal: the video link does not depend on it.
Boot log: `maburgs: mavlink endpoint udp <bind>:<port>` or
`maburgs: mavlink endpoint disabled`.

- **Last peer.** The peer is whoever sent the most recent datagram;
  QGroundControl, MAVProxy and pymavlink all open with a heartbeat. Each
  datagram's bytes go to the tunnel, which splits them into packets and
  queues whole ones.
- **Outbound** packets are packed into `T_MAV` frames once per core-loop
  tick. While `maburgs` is not in a session they are dropped and counted:
  a ground app's heartbeats before link-up are not replayed later.
- **Inbound**: each MAVLink packet from the drone goes to the peer as its
  own datagram. With no peer, or when `sendto` fails, the datagram is
  dropped: the socket never blocks and never retries. Retrying a request
  whose answer was lost is the ground app's job.
- Polled from the core loop next to the calibration control socket; at
  most 64 datagrams per tick, so a flooding client cannot hold the loop.
- Sideport: `link.mav{tx, rx, reject, peer}` (`docs/observability.md`);
  maburtop shows `mav <tx>/<rx>` on its top bar.

## Trust boundary

Over the air the tunnel is authenticated by mabur's SipHash tag, bound to
the session nonces (`docs/link-pairing.md`): only a GS holding the link
key, in the current session, can make the drone act. The UDP endpoint
sits inside that boundary, like the calibration control socket: **whoever
can reach the socket is trusted** and can rewrite the drone's config
(`docs/vtx-config-mavlink.md`). The default bind is loopback; binding the
GS's LAN address is a deliberate choice that lets anything on that LAN in.

MAVLink 2 message signing stays off in both directions. It needs a wall
clock the drone does not have, and a second key next to the link key; the
SipHash tag is the authentication. Signing between a ground app and a
flight controller would pass through the tunnel untouched: mabur never
rewrites packets.

## Calibration over the tunnel

### The mabur dialect

`common/mavlink/mabur.xml` includes `common.xml`, so every common message
still packs and decodes under it, and adds six messages in the private id
range. Both daemons and the web GS build against it (`mabur/mav.h`
includes `mavlink/mabur/mavlink.h`); a `common`-only ground app drops the
new ids at its parser (unknown CRC_EXTRA), which is what we want.

| message | id | dir | fields |
|---|---|---|---|
| `MABUR_CAL_SWEEP` | 60000 | GS → drone | `nonce u32, seq u8, kind u8 (1 power, 2 verify, 3 capacity), bw u8, frames_per_cell u16, settle_ms u16, gap_us u16, dwell_ms u16, idx_lo i8, idx_hi i8, mcs_mask u8` |
| `MABUR_CAL_ACK` | 60001 | drone → GS | `nonce u32, seq u8, result u8 (0 accepted, 1 anchor unreadable, 2 busy, 3 unsupported), start_in_ms u16, flags u8 (the stamp's ldpc/stbc mask)` |
| `MABUR_CAL_SWEEP_DONE` | 60002 | drone → GS | `nonce u32, seq u8, offered u32[8], blocked_pct u8[8]` |
| `MABUR_CAL_APPLY` | 60003 | GS → drone | `nonce u32, seq u8, bw u8, has u8 (bit0 walls, bit1 eff), walls i8[8] (−128 = leave), legacy i8, eff u16[8] (×1000, 0 = leave)` |
| `MABUR_CAL_APPLIED` | 60004 | drone → GS | `nonce u32, seq u8, bw u8, ok u8, reason char[50]` (why it refused; on success empty, or a note of what it kept) |
| `MABUR_CAL_ABORT` | 60005 | GS → drone | `nonce u32` |

`seq` is the sweep's position in the run, on SWEEP and on the APPLY that
sweep produced. The drone answers only `accepted` or `anchor unreadable`
today. What each message does in a run: `docs/calibration.md`. The
messages carry no RC version of their own; the dialect changes with the
code. `common/include/mabur/cal_proto.h` holds them as plain structs and
`common/src/cal_mav.cpp` is the one TU that packs and parses them
(`mav.h`'s per-TU channel rule).

The generated C headers are vendored under `third_party/mavlink/`
(`mabur/`, `common/`, `minimal/`, `standard/`, all from one pymavlink
version). Regenerate with the command in `third_party/mavlink/VERSION`:

```
nix-shell -p python3Packages.pymavlink --run "mavgen.py --lang C --wire-protocol 2.0 -o third_party/mavlink common/mavlink/mabur.xml"
```

then reapply the one hand patch: `protocol.h`'s `mav_array_assign_char`
and `_mav_put_char_array` copy with `mav_array_memcpy`, not the
generator's `strncpy`, because `PARAM_EXT_*` carry raw bytes in a
`char[]` field.

### The cal lane (GS)

`maburgs` encodes the session's due message as system 255 / component
190 and hands it to `MavTunnel::push_cal(packet, run_ctx)`. The cal lane
differs from the live lane in four ways:

- **Its own frames.** A calibration packet is never packed into a frame
  with a ground app's packet: the drone tells the two apart by content.
- **The frozen pair.** Frames are tagged under `run_ctx`, the pair the
  session froze when it first commanded the drone, whatever the GS's own
  session state — every sweep outlasts the drone's failsafe, and the GS
  may adopt a fresh pair between sweeps.
- **Its own counter**, restarting at 1 only when a frame's
  `run_ctx.vtx_nonce` differs from the last cal frame's; a second run on
  the same pair keeps counting (the drone would read a restart as a
  replay).
- **Its own path.** `flush()` sends cal frames first, through a sender of
  their own that skips the RCF slotter and the radio-silence gate (still
  behind `ChannelCore::may_send`): the session offers a message only when
  the air is the GS's (waiting for an ack or an APPLIED, or the closing
  Aborts), and repeats it every 200 ms until answered.

Inbound, the tunnel's `deliver` hands every drone packet to the UDP peer,
as before, and also to the calibration session (`feed_cal_packet`), which
ignores what does not fit its state; delivery does not depend on the
GS's session state. Counters: `tx_cal_frames` and `drop_cal_oversize` in
`MavTunnelStats` (not in `link.mav`).

### Two counter domains and the run-pair latch (drone)

`RcAgent::check_mav` (MAV thread) verifies the tag, then classifies the
frame by content (`cal::all_packets_are_cal`):

- **Cal-only** (every packet a calibration id, no trailing bytes): its
  counter is checked against `cal_last_`, a per-pair high-water mark of
  its own, so the cal lane's counter and the live lane's never refuse
  each other.
- **Anything else**: `mav_last_`, under the published session only, as
  above.

`kOkLatched`: a cal-only frame that fails the published session is
re-verified under the **run pair** — the pair of the last accepted cal-only
frame — while a calibration run is open (`cal_active`), with a counter
above that pair's `cal_last_`. The run pair is recorded on every accepted
cal-only frame, so the SWEEP that opens a run (accepted under the
published session) sets it, and a SWEEP accepted under a newer published
pair moves it. It is honoured only while a run is open. A frame carrying
anything but calibration messages never takes this path: a parameter set
in a latched frame drops the whole frame and raises `auth_reject`, so the
latch cannot change config outside a run. On the TX writer, a latched
SWEEP that names a nonce other than the open run's is dropped and counted
(`cal_latch_drop=` on `maburd tx_send:`): a run opens only from a SWEEP
verified under the published session.

`kLateCal`: a cal-only frame that fails the published session, arrives
with **no** run open, and still verifies under the run pair is the closed
run trailing off — the GS sends its closing Abort three times, the first
closes the run, and the repeats land after (often under a re-paired
session). It is dropped without `auth_reject` and counted (`cal_late=` in
the `stats:` line's `mav` part); its counter is neither checked nor
consumed. A non-cal frame under the old pair, or a frame that verifies
under no known pair, is still a reject.

The drone's replies (ACK, SWEEP_DONE, APPLIED) are packed by the TX
writer thread as untagged `T_MAV` frames, one message each, on the
control path with the shared Telem 802.11 sequence, whatever the session
state and outside the MAV thread's 50 ms pacing and sweep suppression.
The MAV thread and the TX writer share one atomic uplink counter, so the
GS's duplicate filter sees one monotonic sequence. They count in the
`stats:` line's `mav tx=`.

## Later tenants

Designed for, not built. Each is more traffic through the same tunnel and
one more case in the drone's routing; none needs a wire change.

- **Flight controller over MAVLink.** A drone `[fc]` section names the
  UART; the MAV thread runs a second parser on it, packets for component 1
  go to the UART, and the FC's packets go up the tunnel under a rate
  filter (a low stream profile requested with
  `MAV_CMD_SET_MESSAGE_INTERVAL`, or a byte budget by message id). Arm
  state comes from the FC heartbeat's `MAV_MODE_FLAG_SAFETY_ARMED` into
  the input low-power mode reads today.
- **Firmware upload / MAVLink FTP.** FTP packets are tunnel payload; the
  tenant is a file-service component on the drone.
