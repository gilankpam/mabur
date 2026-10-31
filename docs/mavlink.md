# MAVLink tunnel and drone parameters over the link

The radio link carries MAVLink 2 packets in both directions inside one RC
frame type, `T_MAV`. The first tenant is a parameter component on the drone
that serves twelve parameters over thirteen keys of `/etc/mabur.toml`
(`VENC_MODE` spells two) as MAVLink extended parameters (`PARAM_EXT_*`): a ground end holding the link key can list,
read and set them while linked. A set applies to the running `maburd`
before it is published to the file — a knob takes effect on `RcAgent`'s
next tick, a reprogram takes effect in place, a rebuild restarts the
encoder pipeline in-process — and `ACCEPTED` always means both: in
effect on the drone and on disk. See "Applying a set" below.

On the GS, `maburgs` bridges the tunnel to a MAVLink UDP endpoint, so a
ground app or `tools/bench/mavparam.py` talks to the drone as if it were a
MAVLink system on the network. The web GS runs its own ground client on
the tunnel, and its VTX tab is the operator screen onto the table
(`docs/web-gs.md` "VTX tab").

The second tenant is calibration (`docs/calibration.md`): `maburgs`'
calibration session and the drone's sweep engine talk over the same
tunnel in a mabur MAVLink dialect, on a lane of their own that survives
the session lapse every sweep causes. See "Calibration over the tunnel"
below.

## Where the code lives

| Piece | File |
|---|---|
| `T_MAV` frame, `CAP_MAVLINK` | `common/include/mabur/rc_proto.h` |
| the parameter table (ids, keys, types, help) | `common/include/mabur/config_params.h` |
| the mabur dialect's XML (`mabur.xml`, including `common.xml`) | `common/mavlink/` |
| the vendored generated headers (mabur, common, minimal, standard) | `third_party/mavlink/` (pymavlink version, regenerate command and the `protocol.h` patch in its `VERSION`), included through `common/include/mabur/mav.h` |
| calibration messages: plain structs and their MAVLink codec | `common/include/mabur/cal_proto.h`, `common/include/mabur/cal_mav.h` |
| drone: the component (list/read/set, heartbeat) | `drone/src/mav_component.h` |
| value coding (wire and text), shared by drone and GS | `common/include/mabur/param_value.h` |
| drone: per-row accessors, TOML rendering, the row patcher | `drone/src/mav_params.h` |
| drone: line patcher and verified publish | `drone/src/config_write.h` |
| drone: MAV thread, tag and counter check | `drone/src/main.cpp` (`mbr-mav`), `RcAgent::check_mav` |
| GS: the tunnel and its cal lane (shared GS core) | `gs/src/mav_tunnel.h` |
| GS: the UDP endpoint | `gs/src/mav_endpoint.h` |
| GS: the ground client (list/read/set, link and heartbeat state) | `gs/src/param_client.h` |

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
  the only packets it originates are calibration's, which it sends as
  255/190.

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
- **List.** One `PARAM_EXT_VALUE` per table row, in table order,
  `param_count` 12 and `param_index` = the row. Values are read by loading
  the file with the boot loader (`mabur::load_config`) on every request,
  so a list shows a set made a moment ago or an ssh edit. A file that no
  longer loads answers nothing and logs the loader's reason. A list
  request that arrives while an earlier list's values are still queued
  is answered by those values, not a second copy.
- **Outbound queue.** At most 64 packets wait for the uplink; the cap
  applies to every reply and the heartbeat alike. A packet that would
  exceed it is dropped and counted in the `stats:` line's `drop`
  (`docs/observability.md`), never one already queued; the ground app's
  retry recovers it.
- **Read.** By `param_index` (when not -1) or by `param_id`: one
  `PARAM_EXT_VALUE`. An unknown id or index gets no reply (the protocol
  has no negative read answer; the ground end times out) and is logged.
- **Set.** One key per set, answered with one `PARAM_EXT_ACK`, or two for
  a set that changes the file:
  - unknown id: `PARAM_ACK_VALUE_UNSUPPORTED`, value zeroed;
  - a `param_type` other than the row's, a Bool byte other than 0/1, a
    non-finite REAL32, or a CUSTOM value with no NUL in its 128 bytes or a
    non-printable byte: `PARAM_ACK_VALUE_UNSUPPORTED` with the current
    value; there is no cross-type coercion;
  - a `VENC_MODE` value that is not `<fps>@<W>x<H>` — each number a run
    of 1 to 6 digits, nothing before or after:
    `PARAM_ACK_VALUE_UNSUPPORTED` with the current value. A well-formed
    value out of range is the loader's `FAILED`, below;
  - a set to the value the file already holds writes nothing and answers
    `ACCEPTED` immediately, so a ground end may re-send a set whose ack
    it lost;
  - otherwise the line(s) the row owns are patched in place (spacing
    and trailing comment kept byte for byte) and the candidate is
    written next to the file and verified with `mabur::load_config`
    before anything is submitted for applying. A candidate the loader
    refuses is `PARAM_ACK_FAILED` immediately, with the current,
    unchanged value, followed by one `STATUSTEXT`
    (`MAV_SEVERITY_ERROR`) carrying the reason (`<field>: <why>`, the
    loader's message without its file and line, cut to 50 characters).
    A key whose line is missing or commented out is `FAILED` with
    `<section.key>: key not found`; nothing is written. A candidate
    the loader accepts, and that is a real change, is handed to
    `ConfigApplier` instead of being written straight to disk: the set
    answers `PARAM_ACK_IN_PROGRESS` with the current value at once,
    and a second `PARAM_EXT_ACK` follows once the applier finishes —
    `ACCEPTED` with the value read back from the now-published file,
    or `FAILED` with the current value and a `STATUSTEXT` reason. See
    "Applying a set" below.
  - Only one set is ever being applied at a time. A set that arrives
    while an earlier one is still being applied is answered at once,
    before the config-write lock the apply holds is touched (so the MAV
    thread keeps heartbeating through a rebuild): a repeat of the pending
    set — same key, same value, as a ground end sends after losing the
    `IN_PROGRESS` — gets `PARAM_ACK_IN_PROGRESS` again with the current
    value, and the one final ack answers both; any other set is
    `PARAM_ACK_FAILED` with the current value and a `STATUSTEXT`
    "apply in progress". A set is refused `FAILED` "calibration in
    progress" while a calibration session is running.
  - Each set is one or two `[mav] set ...` lines in `maburd`'s log
    (`docs/observability.md`).

`ACCEPTED` always means both: the value is in effect on the running
`maburd` and published to disk. The applier publishes a candidate only
after the process has accepted it — never before — and a publish that
fails afterwards (disk full, a lost race) re-applies the previous config
so the process and the file never disagree. The read-patch-check
sequence in `handle_set` and the applier's own read-patch-apply-publish
sequence both hold the process-wide config-write lock, so a calibration
publish on another thread and a parameter set never lose each other's
change. There is no arm-state gate: a set in flight is the operator's
call.

### Applying a set

A set that changes the file and passes the loader check is applied to
the running process by `ConfigApplier`, its own thread (`mbr-apply`,
`drone/src/config_apply.h`) — a rebuild stops the encoder for about half
a second, too long to run on the MAVLink thread that answers the set.
Each parameter takes one of three paths, the parameter table's "apply"
column below:

- **Knob** (`ENC_BITRATE_MAX`, `ENC_AIRTIME`, `LOWPWR_ENABLE`): the new
  value is written to a live atomic `RcAgent` reads on its next tick.
  Nothing stops.
- **Reprogram** (`RADIO_WALL_MARGN`, `RADIO_LDPC`, `MSP_ENABLE`): a radio
  or peripheral setting changes in place. `RADIO_WALL_MARGN` rebuilds
  both widths' TX-power plans and writes the live rung's under the retune
  gate (`docs/calibration.md` "Per-width tables on the drone");
  `RADIO_LDPC` rebuilds the control radiotaps and `RcAgent` re-applies
  the current operating point with the new flag on its next tick (every
  ladder it builds from then on uses it too); `MSP_ENABLE` starts or
  stops the MSP thread. A `RADIO_LDPC` set that leaves a flown wall table
  measured under the other LDPC setting (`radio.cal_flags_<bw>`) is still
  `ACCEPTED`, followed by a `STATUSTEXT` (`MAV_SEVERITY_WARNING`) per
  width: `radio: 20 MHz walls were measured with ldpc on; radio.ldpc is
  off — re-run maburcal`. It is longer than 50 bytes, so it goes out in
  MAVLink 2's chunked form (one nonzero id, `chunk_seq` from 0).
  `mavparam.py` prints the chunks as they come; the web GS's
  `ParamClient` reassembles them and attaches the whole text to the
  `RADIO_LDPC` row, which the VTX tab shows under it as a warning
  (`docs/web-gs.md`).
- **Rebuild** (`VENC_MODE`, `VENC_SLICES`, `REC_ENABLE`,
  `REC_BITRATE`, `REC_FPS`, `REC_SIZE`): the encoder pipeline is rebuilt
  in-process (`docs/link-adaptation.md` "Rebuilding the encoder pipeline
  in-process"), about 0.5 s plus ~0.1 s to the first frame of the new
  mode. A recording in progress is stopped first and the record channel
  is recreated in its bind order; the GS's standing record wish restarts
  it (`docs/vtx-recorder.md`).

**The rebuild failure ladder.** A rebuild tries the new config
(`venc_core_restart`); if the pipeline refuses it, the ladder restarts
the previous config the same way — a failed restart leaves the core
stopped but the MI_SYS session open, and a restart is gated on that
session, so the second restart builds on it; if that also fails, a full
`venc_core_stop()` + `venc_core_start()` of the previous config (about
8.5 s; the stop closes the session even though the core is not running,
so the start opens a fresh one); and if that also fails, `maburd` exits
for its init wrapper to respawn it from the unchanged file. Each rung
logs one line. A set that falls to any rung but the first still answers
`PARAM_ACK_FAILED` with the reason — `ACCEPTED` is only for the config
that was actually asked for, never a fallback. The contract is written
in `drone/venc/venc_core.h` and pinned on the host by `test_venc_core`
(the real `venc_core.c` over a fake runtime).

To walk the restore rungs on a device, start `maburd` with
`MABUR_VENC_FAIL_RESTARTS=N` in its environment: the next `N` pipeline
restarts fail after the old pipeline is stopped and before the SDK is
touched (`venc: injected pipeline start failure` in the log). `N=1`
makes a rebuild set land on the "restored previous" rung, `N=2` on the
cold rung. It is read once, by the process's first encoder start, and
only counts down — a respawn without the variable, or a later start in
the same process, never re-arms it.

Only one apply runs at a time (a second set is answered as described
under **Set** above), and an apply is refused outright while a
calibration session is running ("calibration in progress") —
calibration owns the TX-power table, and its file publish takes the
same config-write lock. A host build of `maburd` (no encoder) refuses
the rebuild keys at submit, "no encoder in this build". Logging and the
`stats:` line's `mav_apply=`/`mav_rebuild=` counters:
`docs/observability.md`. The undo, `.pre-param`: below.

### `.pre-param`

Every set that changes the file saves the previous file as
`/etc/mabur.toml.pre-param` before replacing it (written then renamed, so
a crash part way through leaves the previous backup intact). It is the
undo over ssh; over the air the undo is setting the old value again. It
sits beside `.pre-cal` in the `.pre-*` convention (`docs/deploy.md`).

## Parameters

The table in `common/include/mabur/config_params.h` is the single source:
its order is the `param_index` order on the wire, and the help strings
are what an operator UI shows. Types on the wire: Bool is
`MAV_PARAM_EXT_TYPE_UINT8` (byte 0 is 0 or 1), Int32 and Real32 are
`INT32`/`REAL32` little-endian in bytes 0..3, Str is `CUSTOM`
(NUL-terminated ASCII, at most 127 characters, written into the TOML as a
quoted string).

| # | param_id | TOML key | type | apply | help |
|---|---|---|---|---|---|
| 0 | `RADIO_WALL_MARGN` | `radio.wall_margin_db` | Real32 | reprogram | Safety margin below the calibrated TX-power walls; raise to lower TX power |
| 1 | `RADIO_LDPC` | `radio.ldpc` | Bool | reprogram | LDPC coding on every frame; turn off only for a ground card that cannot decode it |
| 2 | `VENC_MODE` | `venc.fps` + `venc.size` | Str | rebuild | Camera frame rate and encoder size, `<fps>@<WxH>`; the rate picks the sensor's field-of-view mode and the size must fit it |
| 3 | `VENC_SLICES` | `venc.slices` | Int32 | rebuild | H.265 row slices on the link channel; 1 disables slice salvage |
| 4 | `ENC_BITRATE_MAX` | `encoder.bitrate_max_kbps` | Int32 | knob | Bitrate ceiling, kbps |
| 5 | `ENC_AIRTIME` | `encoder.airtime_budget` | Real32 | knob | Share of the link's real capacity the video may use |
| 6 | `REC_ENABLE` | `record.enable` | Bool | rebuild | Enable the onboard microSD recorder |
| 7 | `REC_BITRATE` | `record.bitrate_kbps` | Int32 | rebuild | Constant bitrate of the onboard recording |
| 8 | `REC_FPS` | `record.fps` | Int32 | rebuild | Onboard recording frame rate, at most the camera's fps |
| 9 | `REC_SIZE` | `record.size` | Str | rebuild | Onboard recording resolution, same aspect ratio as the camera's |
| 10 | `LOWPWR_ENABLE` | `low_power.enable` | Bool | knob | Lower bitrate and frame rate while disarmed to keep the VTX cool; needs msp.enable for arm state |
| 11 | `MSP_ENABLE` | `msp.enable` | Bool | reprogram | Flight-controller link for the OSD and arm state |

"apply" comes from each row's `Apply` value in the table itself
(`cfgparam::apply_name`, `common/include/mabur/config_params.h`) — how the
set reaches the running process (`apply_path_for` in
`drone/src/config_apply.h` maps it to the same three paths for drone-side
dispatch; "Applying a set" above): knob is a live atomic `RcAgent` reads on
its next tick, reprogram changes a radio or peripheral setting in place,
rebuild restarts the encoder pipeline in-process. Ranges are the loader's,
not repeated here: whatever `maburd` refuses to boot on, a set refuses.

### Ordering rules

There is no transaction: each set is verified against the whole file as it
stands, so keys that depend on each other must be set in a working order.
A set out of order is refused with the loader's reason and changes
nothing; send the other key first. The rules the loader enforces:

- `REC_FPS` ≤ `VENC_MODE`'s fps (while the recorder is on): lower
  `REC_FPS` before lowering `VENC_MODE`'s fps; raise `VENC_MODE`'s fps
  before raising `REC_FPS`.
- `REC_SIZE` above 1920x1080 needs `VENC_MODE`'s fps ≤ 30 first.
- `LOWPWR_ENABLE = 1` needs `MSP_ENABLE = 1` first; `MSP_ENABLE = 0` needs
  `LOWPWR_ENABLE = 0` first.
- A `VENC_MODE` set that changes the sensor mode's aspect, while the
  recorder is on with an explicit `record.size`, also rewrites
  `record.size` to the new venc size in the same candidate; `record.fps`
  never follows.

Two loader rules involve keys the table does not expose:

- With `LOWPWR_ENABLE = 1`, `VENC_MODE`'s fps cannot drop below
  `low_power.fps` and `ENC_BITRATE_MAX` cannot drop below
  `low_power.bitrate_kbps`.
- `REC_SIZE` must keep `VENC_MODE`'s size's aspect ratio while the
  recorder is on.

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

## Ground client

`ParamClient` (`gs/src/param_client.h`) is the ground-side PARAM_EXT_*
protocol client: list, read, set and link/heartbeat state, driven only by
a send callback and `on_packet`/`tick(now)` -- pure data handling, no
socket or clock of its own. The web GS core owns one (`docs/web-gs.md`
"MAVLink seam"), fed by the tunnel's `deliver` callback, and its VTX tab
is the operator screen onto it. `maburgs` does not run one: it only
bridges the tunnel to the UDP endpoint.

- **Link.** `set_linked(true)` starts a fresh list: every row is cleared,
  `list` becomes `Pending`, and one `PARAM_EXT_REQUEST_LIST` goes to
  system 1 / component 191. `set_linked(false)` drops every row back to
  the unlinked defaults -- including a set in flight, whose row passes
  through `Timeout` with status "link lost" on the way to that clear, so
  the end state a caller observes is the plain unlinked defaults.
  Independent of this is the drone heartbeat: `alive()` is true while a
  `HEARTBEAT` from 1/191 arrived within the last 3 s; a heartbeat that
  arrives while already alive only refreshes the timestamp. Either
  `set_linked` call also zeroes `alive_ms`, so a fresh link has no
  heartbeat until the drone's next one (the web GS reports that as
  `alive: null`).
- **List.** While `list` is `Pending`, each row that has not answered
  since the round began (no `PARAM_EXT_VALUE` or ack carrying its value)
  is re-read individually by `PARAM_EXT_REQUEST_READ` (`param_index` set,
  the id left blank) once every 2 s, for up to 3 rounds; `list` reaches
  `Complete` as soon as every row has answered, or `Partial` once the
  rounds run out with rows still silent. A `PARAM_EXT_VALUE` whose
  `param_count` disagrees with the table's own row count flags
  `table_mismatch`, but the value is still stored when its id is known; an
  id the table doesn't have is dropped and counted. `refresh()` starts the
  same round again without clearing the rows already held, for a UI's
  manual re-list: a held row keeps its value on show but is re-read like a
  missing one until it answers again, and so is every row after the
  re-list a rebuild's `ACCEPTED` starts.
- **Set.** `set(row, text, now, &why)` refuses (`false`, `*why`) while
  unlinked ("not linked"), while another set is already in flight
  ("apply in progress"), for an out-of-range row ("no such parameter"),
  or for text that doesn't parse for the row's type ("bad value for
  `<type>`"); a refusal on a row the table has marks that row `Failed`
  with `*why` as its status. An accepted set sends one `PARAM_EXT_SET`,
  marks the row `Sending` and clears its status (so a reason from an
  earlier set never shows against this one), then resends it every 2 s,
  up to 3 times, while no ack for it has arrived; the row times out
  (`result_ms` stamped, one `PARAM_EXT_REQUEST_READ` follows) if nothing
  ever answers. The row's first `PARAM_ACK_IN_PROGRESS` moves it to
  `InProgress` and opens a 15 s deadline from that moment -- a repeated
  `IN_PROGRESS` updates the row's value but never pushes the deadline
  out, and the row times out the same way if the deadline passes first. `PARAM_ACK_ACCEPTED`/`FAILED`/
  `VALUE_UNSUPPORTED` for the row is terminal: the row's state and value
  are set and the client is free to set another row; an `ACCEPTED` on a
  row whose `apply` is `rebuild` also starts a fresh list round, since a
  `VENC_MODE` set can move `REC_SIZE`'s value along with it. An ack for
  any row other than the one being set only updates that row's value. A
  `STATUSTEXT` from 1/191 lands on the row currently being set, if there
  is one -- so a reason that reaches the GS before, or instead of, its
  ack still lands on the right row -- otherwise on whichever row's
  terminal `Accepted`/`Failed`/`Unsupported` landed within the last 1.5 s
  (the latest one), or is dropped: a refusal's reason follows its
  `FAILED`, a warning follows its `ACCEPTED`. A text in MAVLink 2's
  chunked form (nonzero `id`, `chunk_seq` from 0, the last chunk
  NUL-terminated) is reassembled before it lands, so a row gets the whole
  text; a missing chunk drops it.
- **State.** `state()` exposes `linked`, `alive_ms`, `table_mismatch`,
  `list`, `pending_row` (the row a `set()` is in flight for, or -1), and
  one row per table parameter holding `have_value`/`value`/`st`/`status`/
  `result_ms` from the list/read/set traffic above -- a row always shows
  the value the drone last reported for it (a `PARAM_EXT_VALUE` or an
  ack), never text an operator typed and the drone hasn't confirmed. `seq()` bumps once per call that changed
  anything in it, so a UI can poll it cheaply instead of diffing the
  struct; a resend that changes nothing observable does not bump it.

### Trust boundary

Over the air the tunnel is authenticated by mabur's SipHash tag, bound to
the session nonces (`docs/link-pairing.md`): only a GS holding the link
key, in the current session, can make the drone act. The UDP endpoint
sits inside that boundary, like the calibration control socket: **whoever
can reach the socket is trusted** and can rewrite the drone's config. The
default bind is loopback; binding the GS's LAN address is a deliberate
choice that lets anything on that LAN in.

MAVLink 2 message signing stays off in both directions. It needs a wall
clock the drone does not have, and a second key next to the link key; the
SipHash tag is the authentication. Signing between a ground app and a
flight controller would pass through the tunnel untouched: mabur never
rewrites packets.

## Bench tool

`tools/bench/mavparam.py` runs on the host with pymavlink (NixOS:
`nix-shell -p python3Packages.pymavlink`) and talks UDP to the endpoint.
`GS=` (the GS address) and `PORT=` come from the environment, with no
default:

```
GS=<gs-ip> PORT=14550 nix-shell -p python3Packages.pymavlink --run "python3 tools/bench/mavparam.py list"
GS=<gs-ip> PORT=14550 nix-shell -p python3Packages.pymavlink --run "python3 tools/bench/mavparam.py get VENC_MODE"
GS=<gs-ip> PORT=14550 nix-shell -p python3Packages.pymavlink --run "python3 tools/bench/mavparam.py set REC_ENABLE 0"
```

It sends one heartbeat so the endpoint learns the peer, waits up to 5 s
for the drone's own `HEARTBEAT` (system 1, component 191), then runs the
one subcommand:

- `list` requests the whole table and prints one `id  type  value` row
  per parameter, in `param_index` order, retrying missing indices
  individually; a parameter still missing after the retries is a
  non-zero exit.
- `get ID` reads one parameter by id (1 s timeout, 3 retries) and prints
  its `id  type  value` row.
- `set ID VALUE` looks `ID` up in its own copy of the 12-row table (ids
  and types mirrored from `common/include/mabur/config_params.h`) to
  encode `VALUE` for the wire and sends the set (2 s timeout, 3 retries).
  A set the drone applies (a real change past a loader check) answers
  `IN_PROGRESS` first, holding the current value; the tool prints
  `IN_PROGRESS  <value>` and waits up to 15 s from that ack for the
  final one before giving up — long enough for a rebuild's ~0.5 s or,
  on the ladder's worst case (two failed restarts plus an ~8.5 s cold
  one), for that to finish too. It then prints the final
  ack result (`ACCEPTED`/`FAILED`/`VALUE_UNSUPPORTED`) and value, and
  any `STATUSTEXT` that follows within 1 s. A set refused immediately
  (unknown id, a type mismatch, a loader refusal, "apply in progress",
  "calibration in progress") never sees `IN_PROGRESS` at all.

Exit 0 on an ACCEPTED set or a successful list/get; 1 on a runtime
failure — no heartbeat, a request that times out after retries, an
incomplete list, an ack other than ACCEPTED, or a set whose final ack
never arrived within 15 s of `IN_PROGRESS`; 2 on a usage error (missing
`GS`/`PORT`, a bad command line, an unknown parameter id, or a value
that doesn't parse for its type).

The endpoint binds loopback by default, so reaching it from the host means
setting `[mavlink] bind` to the GS's LAN address in the GS's live config:
back up `/etc/maburgs.toml`, edit it, restart `S96maburgs`
(`docs/deploy.md`).

## Operator UIs

The web GS's VTX tab (`docs/web-gs.md` "VTX tab") renders the parameter
table and drives it live; a `maburplay` screen onto the same table is not
built.

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
