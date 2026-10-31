# Link pairing: the authenticated control plane

Spec: `docs/superpowers/specs/2026-10-01-link-pairing-design.md`
(gitignored; this page is the committed, as-built record). Hardware gate:
`docs/findings/2026-10-01-link-pairing-bench.md`.

The drone applies control only from a ground end that holds the shared
link key, and a recorded session cannot be replayed to it. Video, the MSP
OSD stream and drone telemetry stay in the clear and unchanged, so the
passive web spotter keeps working with no key at all.

## The key

Sixteen random bytes, kept in a **key file**, never in the TOML. One file
is the pairing: the same bytes on the drone, the GS, and (loaded into
local storage) the web page.

**File format** (`/etc/mabur.key`, same name on both ends):

```
# mabur link key, generated 2026-10-01
3f9a1c77e04b5d2290ab6ef1c8d34e5a
```

Exactly one token of 32 hex characters (case-insensitive); blank lines,
surrounding whitespace and lines starting with `#` are ignored. Anything
else (two tokens, wrong length, non-hex) is a boot failure naming the
file — never a silent fallback. One parser in `common/`
(`mabur::load_key_file`, `mabur::parse_key_text`, `mabur::parse_key_hex`
in `common/src/link_key.cpp`) serves the drone, the GS and the web core
alike.

**Config:**

```toml
[link]
key_file = "/etc/mabur.key"   # the pairing key; same file on the ground station
```

on both the drone (`bundle/mabur.default.toml`) and the GS
(`gs/bundle/maburgs.default.toml`). The GS alone also accepts an inline `[link] key = "<32 hex>"` — the web page's
overlay path (`web/ui/src/lib/config.js` builds it, `web/src/web_main.cpp`
`load_cfg` merges it into `link.key`). That parser is `gs/src/config.cpp`,
shared by the web core and `maburgs`, so `maburgs` itself also accepts
`[link] key` in its TOML (it overrides `key_file`); the daemons are meant
to use `key_file` and keep the key out of the config. `maburd` accepts
only `key_file`.

**Default key.** `mabur::kDefaultLinkKey` — ASCII `mabur-default-00` — is
compiled into all three builds. A daemon whose key file does **not
exist** (`ENOENT` only — an empty file fails the format above, and an
unreadable path, e.g. no permission or a directory, is a boot failure
`<path>: <strerror>`) uses it and logs one boot line; a
web page with no key loaded does the same. A fresh or wiped install links
out of the box; pairing is strictly opt-in. The default offers no security: every default install
pairs with every other.

**Fingerprint.** `mabur::key_fingerprint(key)`: `"default"` for the
compiled-in key, else the first two bytes of `SipHash(key, "mabur.key")`
as 4 lowercase hex characters. Never the key itself. Shown:

- in each daemon's boot log: `link: key <fp> (<source>)`, `<source>` the
  file path, `"default"`, or `"link.key"` (the GS's inline overlay);
- on the sideport as `link.key_fp`;
- in `tools/maburtop.py`'s link panel (`key <fp>` / `KEY MISMATCH`);
- beside the web page's Load/Clear buttons, with a `Key mismatch` status
  tag and `Drone rejects our link key (<fp>) — load the same mabur.key on
  both ends` when the drone is rejecting it.

Not on the OSD (no room, and the fingerprint alone doesn't help a pilot
mid-flight — see "Known edges").

## The wire

No frame carries a VTX id: the key is the identity. Every GS-to-drone
control frame carries an 8-byte SipHash-2-4 tag (`kTagLen`) right before
its CRC; the drone-to-GS frames stay untagged:

- **DISC** (GS to drone): tagged over the frame bytes alone (no session exists yet — tagged with
  the key only).
- **DISC_ACK** (drone to GS): carries u32 `vtx_nonce` and u8 `flags` (bit0 =
  `kAckKeyMismatch`, `0x01`). Unauthenticated — the drone has nothing of
  the GS's to prove yet, and a forged ack can only confuse a display, not
  move the aircraft (see "Not defended"). `chip_caps` carries `CAP_MAVLINK`
  (`0x0008`) when the drone understands T_MAV. It is display-only: the GS
  logs `maburgs: mavlink: drone lacks CAP_MAVLINK` once per session when the
  bit is missing and gates nothing on it.
- **RCF** (GS to drone): tagged over the frame bytes
  plus `vrx_nonce ‖ vtx_nonce ‖ seq32` (`TagCtx`) — session-bound, not
  just key-bound.
- **NACK** (GS to drone, `docs/fec-nack.md`): same session-bound tag, with
  the request's own `counter` as `seq32`. The drone accepts only a counter
  greater than the last one it accepted under the current pair
  (`RcAgent::check_nack` → `verify_session_tagged` +
  `accept_nack_counter`, on the RX thread); a tag or counter failure sets
  `auth_reject`, a parse failure does not.
- **MAV** (both directions, carries one or more opaque MAVLink packets): GS to drone
  is tagged, same session-bound `TagCtx` as NACK, with the request's own
  `counter` as `seq32` under the same accept-only-greater rule, on its own
  counter state (`RcAgent::check_mav` → `verify_session_tagged` +
  `accept_mav_counter`, on the MAV thread); a tag or counter failure sets
  `auth_reject`, a parse failure does not. Drone to GS is untagged, like
  Telem. Calibration rides MAV too (`docs/calibration.md`): a frame made
  entirely of calibration messages counts in a counter domain of its own
  (`cal_last_`) and may verify under the calibration run's pair after the
  session lapses ("Session and the nonces").
- **Telem** (drone to GS): flags bit1 is `kTelemAuthReject`
  (`0x02`) — at least one control frame failed verification this
  telemetry period. (Bit1 was `radio_rx_ok` before the 2026-09-30 telem
  diet; a recording from before that date reads bit1 as the old meaning —
  `docs/data-provenance.md`.)

`rc::verify_control(buf, len, key, ctx)` is the one place a tag is
checked — on the drone, for DISC/RCF on the agent thread, for NACK on
the RX thread and for MAV on the MAV thread. It finds the tag at the frame's
**structural** offset, derived from the type (`DISC_LEN`, `RCF_HEAD_LEN`,
`kNackFixedLen + n * kNackEntryLen`, or `kMavFixedLen + len16`), never at
`len - 10`: on hardware the drone's RC body still carries devourer's
trailing 4-byte 802.11 FCS
(`Packet.Data`), and bytes past the CRC are ignored exactly as `parse_*`
ignores them. `common/include/mabur/siphash.h` + `common/src/siphash.cpp`
are the primitive — SipHash-2-4, 128-bit key, 64-bit output, no library,
builds unchanged for armv7/aarch64/WASM.

## Session and the nonces

A session is the pair (`vrx_nonce`, `vtx_nonce`):

- `vrx_nonce` — the GS's own nonce, random per process start
  (`std::random_device` in `VrxRendezvous`'s constructor). Fixed for the
  life of the process; a restart is a new `vrx_nonce`, deliberately, so a
  restarted GS never inherits a stale session's seq position.
- `vtx_nonce` — picked by the drone, fresh per **new** `vrx_nonce` it
  hears (`RcAgent::fresh_vtx_nonce_`). A repeated DISC with the same
  `vrx_nonce` gets back the same `vtx_nonce` (keep-alive / lost-ack
  retry, harmless either way).

`seq32` is the RCF's 16-bit wire `seq` extended with a wrap count each
end keeps locally (the GS beside its own counter, the drone per session,
`RcAgent::Session::last_seq32`). A frame verifies only if its `seq32` is
strictly greater than the session's last accepted — the only surviving
replay is an RCF the GS sent moments ago that the drone missed on air and
an attacker resends before the next one lands.

**Session rules on the drone:**

- **Every exit from LINKED clears both sessions** (`current_` and
  `pending_`) — failsafe entry and the move-unconfirmed fallback alike
  (`RcAgent::clear_sessions_()`). A kept pair would leave every RCF the
  GS sent during an uplink fade replayable for the life of the process.
  Recovery is the GS's next keep-alive DISC: same `vrx_nonce`, so the
  drone issues a fresh `vtx_nonce`; the GS adopts it (newest wins) and
  re-tags. Until then its RCFs set `auth_reject` — at most one
  `beacon_keepalive_ms` (1 s); the loopback test measured 190 ms.
- **A calibration run keeps its pair.** Every calibration sweep outlasts
  `failsafe_ms`, so the drone clears its sessions inside every run. While
  a run is open, `RcAgent::check_mav` re-verifies a MAV frame that fails
  the published session under the **run pair** — the pair of the last
  accepted calibration-only frame — with a counter above that pair's
  calibration high-water mark. Only frames made entirely of calibration
  messages take that path; any other frame (a parameter set) is rejected
  whole and sets `auth_reject`, so the latch cannot change config outside
  a run. A run opens only from a SWEEP verified under the published
  session; a SWEEP that verifies only under the run pair and names another
  run is dropped. The GS tags a run's frames under the pair frozen when
  the run first commanded the drone. Detail: `docs/mavlink.md`
  "Two counter domains and the run-pair latch".

## Rendezvous, as built

A DISC only ever elicits an ack — **no op, no retune, no LINKED entry**.
The first RCF that verifies under the acked pair is what actually links
the drone (`RcAgent::on_rc_frame` T_RCF: verify against `current_`, then
`pending_`; a `pending_` pass promotes it to `current_`, resets its
`seq32` tracker, calls `take_session_promoted()` true once).

Cold start:

1. GS beacons a tagged DISC every `beacon_period_ms` (20 ms) until the
   first accepted ack; in SESSION it sends one keep-alive DISC every
   `beacon_keepalive_ms` (1000 ms) in place of an RCF.
2. Drone verifies the tag (`rc::verify_control(..., cfg_.link.key,
   TagCtx{})`):
   - pass: pick or reuse a `vtx_nonce` for this `vrx_nonce`, store it as
     `pending_`, ack unflagged (`kAckKeyMismatch` clear);
   - fail: ack with `kAckKeyMismatch` set and `vtx_nonce` zero, set
     `auth_reject`. Nothing else changes.
3. GS (`VrxRendezvous::feed_disc_ack`): `vrx_nonce` must match ours.
   - unflagged: adopt the `vtx_nonce` (**newest unflagged ack for our
     `vrx_nonce` wins**), enter `SESSION`, start tagging and sending
     RCFs;
   - flagged: the stranger rule below.
4. Drone's first RCF under `pending_` verifies: promotes to `current_`,
   enters `LINKED`. If the DISC proposed a channel move, one immediate
   Telem goes out on the OLD channel first (the "I heard you, moving"
   signal — `RcAgent::take_session_promoted()` / `deferred_move_ch_`,
   executed in `tick()` so main has a chance to send that Telem before
   the retune), then the drone retunes.
5. GS: `VrxController::note_drone_state(2)` on that LINKED Telem — or,
   if it's lost, `kMoveAfterRcfs = 5` RCFs sent under the new session —
   arms `take_move_edge()`; GS main follows to the agreed channel on that
   edge. If the GS never follows, the drone hears no accepted RCF on the
   new channel within `link.move_confirm_ms` and returns to the channel
   it came from (`docs/channel-select.md`).

Keep-alive DISC while LINKED: verified like any DISC, acked with the
*same* `vtx_nonce` already issued for that `vrx_nonce` — nothing else
changes. A *different* `vrx_nonce` while the drone is LINKED means a
restarted GS: a fresh `pending_` pair, promoted on its first verified
RCF without touching the current op (the existing op-thrash rule — a GS
crash mid-flight must not drop the drone to the bottom rung).

### The stranger rule, as implemented

The GS enters `VrxState::KEY_MISMATCH` when a flagged ack
arrives and the current flagged-only run (no unflagged ack since its first
flagged ack) satisfies BOTH:

- it is at least `VrxRendezvous::kKeyMismatchMs = 1000` old, and
- at least `VrxRendezvous::kKeyMismatchBeacons = 3` DISCs have gone out
  since its first flagged ack (`beacons_since_flagged_`, counted in
  `beacon()`).

In BEACONING (20 ms DISCs) the time term dominates: ~1 s. In SESSION the
DISCs are the 1 s keep-alives, so the beacon term does: ~3 s, the ack to
the third keep-alive after the first flagged one. That is what keeps a
foreign-key drone in range (it answers every DISC flagged, since with no
VTX id in the frames every drone answers) from tripping the state when one or two of
our own drone's acks are lost. Any unflagged ack resets both terms and,
if already in `KEY_MISMATCH`, leaves it immediately. In `KEY_MISMATCH` the
GS sends no RCFs (no cal either) and keeps beaconing, so a corrected
config on either end recovers without a restart on the other.

The held `vtx_nonce_` is **kept across `KEY_MISMATCH`**, not dropped.
Clearing it would
make our own drone's next same-`vrx_nonce` ack read as a brand-new
session, resetting the GS's `seq32` counter — and the still-LINKED drone
(`seq32` strictly increasing) would then reject every RCF under the
"old" `seq32` until failsafe tore the session down and rebuilt it. That
is worse than the stranger's few flagged acks `KEY_MISMATCH` exists to
flag in the first place.

### Restart cases (all RAM, nothing persisted)

- **Drone restart, GS running** (battery swap). Drone boots with no
  session; the GS's RCFs fail verification (`auth_reject` for about one
  Telem period). The keep-alive DISC (same `vrx_nonce`) arrives within
  `beacon_keepalive_ms`; the drone issues a fresh `vtx_nonce` and acks;
  the GS adopts it (newest wins) and re-tags; the next RCF verifies and
  links. ~1.1 s worst case — measured 990 ms in the loopback test below.
- **GS restart, drone running.** New process, new `vrx_nonce`, beacons;
  the LINKED drone acks with a new `vtx_nonce` as `pending_`; the new
  GS's first RCF promotes it, op untouched, video never pauses. Control
  resumes about a second after the GS restart, because the new GS sends
  no RCFs until it holds a `vtx_nonce`. The old session's tags are dead.
- **Both restart.** Cold start.
- **maburgs to phone takeover.** The second controller's first verified
  RCF steals the session. Running both controllers at once makes the
  drone flip between sessions and ladders and is unsupported (two GSs
  with different seq positions fight over the op point). Signature:
  sustained `auth_reject`.

## Not defended

Out of scope by design (the spec's threat model): jamming/DoS; a captured drone or a stolen
GS config leaking the key (change it); the browser's key storage, whose
security is the phone's lock screen; spoofed telemetry/DISC_ACK toward
the GS (unauthenticated by design — can confuse a display, never move
the aircraft); confidentiality of anything (video/OSD/telemetry stay
clear). **Same-key strangers**: with no VTX id in the frames, a nearby
default-key GS beaconing on the
same channel is answered exactly like our own GS — it only fails to win
a session because our GS's unflagged acks keep winning "newest wins",
not because the drone refuses it. It can thrash the drone's single
`pending_` slot (wasted work, not a session loss as long as our own
keep-alives keep landing). **A spoofed unflagged DISC_ACK** makes the GS
adopt a bogus `vtx_nonce`; the drone rejects the RCFs tagged under it
until `failsafe_ms` (the drone's `[link] failsafe_ms`, 3000 ms in
`bundle/mabur.default.toml`) drops it out of LINKED, and the next
keep-alive DISC re-pairs — DoS, out of scope.

## Known edges

All self-healing.

- **Stranger + three lost acks.** A foreign-key drone in range plus
  three consecutive lost acks from our own drone (~3 s of keep-alives in
  SESSION) still makes the GS's flagged-only run look like a real
  mismatch and enter `KEY_MISMATCH` mid-session; it leaves on the next
  unflagged ack, same as any entry. One or two lost acks do not.
- **Promote Telem drains counters out of cadence.** The immediate Telem
  a session promotion sends is extra and off the regular period — it
  drains the drone's per-period counters early, shortening the next
  regular Telem's window. One short sideport sample per link-up, not a
  sustained effect.

## Observability

- Sideport: `link.key_fp` (`"default"` or 4 hex), `link.state` =
  `key_mismatch` (else `session` / `beaconing`), `drone.auth_reject` (bool, this period). Three causes
  set `auth_reject`: a drone/GS restart or failsafe re-pair (one period,
  expected); two controllers fighting over the session (sustained); and a
  foreign-key GS in range — its DISCs fail the tag and set the bit every
  period while it beacons on our channel, even though our own link is
  fine.
- `tools/maburtop.py`: `key <fp>` in the link panel; `KEY MISMATCH` in
  place of the link state when flagged; `AUTH!` beside the drone panel
  on `drone.auth_reject`.
- `tools/flightreport.py`: `auth reject: N periods` — one period around
  a drone/GS restart is the expected transient; sustained means a bug,
  two controllers fighting over the session, or a foreign-key GS in range.
- Player OSD (`gs/player/src/gs_compact.cpp`): `KEY?` in the `ch:` slot
  that otherwise shows the stale/searching channel state — the one
  failure that would otherwise look exactly like the stale-caps restart
  deadlock and send the operator to `restart maburd`.
- `maburgs` stderr: one line on the `KEY_MISMATCH` edge (entry and
  exit) — `link KEY MISMATCH -- the drone rejects our tag; both ends
  need the same /etc/mabur.key (our key <fp>)` / `link key accepted
  (our key <fp>)`. `ctl.log` does not record it: KEY_MISMATCH is a
  rendezvous state, not a ladder-controller event.
- Web page: `Key mismatch` status tag, `Drone rejects our link key
  (<fp>) — load the same mabur.key on both ends` text. Spotter mode
  hides the key row and writes no overlay.
- Boot log, both daemons: `link: key <fp> (<source>)`, plus a
  `link: DEFAULT key in use ...; see docs/deploy.md 'Pairing'` line
  when `key_is_default`.

## The replay harness's session

`maburd --dry-run` (the host e2e scripts, `tests/integration/*.sh`) calls
`RcAgent::install_session_for_replay(1, 1)` so a file of RCFs built by
`tests/integration/mabur_rc.py`'s `pack_rcf(..., vrx=1, vtx=1)` verifies
without running a DISC exchange first. `mabur_rc.py` reads `RC_VERSION`
out of `common/include/mabur/rc_proto.h` itself, so a wire bump can't
half-land against a stale copy, and ships the same SipHash-2-4
implementation pinned against the reference test vectors
(`python3 tests/integration/mabur_rc.py` prints `ok`).

## In-process pair test

`tests/test_link_auth_e2e.cpp` is the only place both ends of the
rendezvous — a real `RcAgent` and a real `VrxController` — run against
each other on the host, stepped in 10 ms increments with a perfect air
link (no loss, no reorder, except where a case drops frames on purpose).
Five cases:

1. **Matching keys link within a second.**
2. **Mismatched keys** end in `KEY_MISMATCH`, zero RCFs, the drone never
   reaches `LINKED`, no operating point beyond the automatic BOOT
   default is ever applied, no retune, and `auth_reject` is set.
3. **A foreign-key stranger drone beside ours** never trips
   `KEY_MISMATCH`, not even for one ack — run with the stranger's ack
   first and with ours first, each with one of our drone's keep-alive
   acks dropped. Our drone stays linked; the stranger never links.
4. **A drone restart relinks through the GS's keep-alive DISC** — a
   fresh `RcAgent` against the same, still-running `VrxController`
   reaches `LINKED` within one keep-alive interval plus one RCF
   (measured 990 ms).
5. **An uplink fade past failsafe relinks through the keep-alive DISC**
   — GS-to-drone frames dropped for `failsafe_ms` + 300 ms while video
   keeps the GS in SESSION; an RCF captured during the fade and replayed
   after failsafe is rejected (`auth_reject`, still FAILSAFE); once the
   uplink returns the next keep-alive re-pairs under a new `vtx_nonce`
   (measured 190 ms after restore; bound one keep-alive + one RCF).
