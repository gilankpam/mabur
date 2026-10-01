# Link pairing: the authenticated control plane

Spec: `docs/superpowers/specs/2026-10-01-link-pairing-design.md`
(gitignored; this page is the committed, as-built record). RC_VERSION 14.

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
(`gs/bundle/maburgs.default.toml`) — the slot `link.vtx_id` used to
occupy; an old config carrying `vtx_id` fails boot (strict keys). The GS
alone also accepts an inline `[link] key = "<32 hex>"` — the web page's
overlay path (`web/ui/src/lib/config.js` builds it, `web/src/web_main.cpp`
`load_cfg` merges it into `link.key`); `maburd` and `maburgs` reading a
real TOML file accept only `key_file`, never the inline form.

**Default key.** `mabur::kDefaultLinkKey` — ASCII `mabur-default-00` — is
compiled into all three builds. A daemon whose key file is **missing**
(not merely empty — see format above) uses it and logs one boot line; a
web page with no key loaded does the same. A fresh or wiped install links
out of the box exactly as `vtx_id 1` did before this feature; pairing is
strictly opt-in. The default offers no security: every default install
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

## The wire (RC_VERSION 14)

`link.vtx_id` (u32) is gone from every frame and both configs — it
defaulted to 1 everywhere and never separated two mabur setups at one
field; the key is the identity now. In its place:

- **DISC** (GS to drone): + an 8-byte SipHash-2-4 tag before the existing
  CRC, over the frame bytes alone (no session exists yet — tagged with
  the key only).
- **DISC_ACK** (drone to GS): + u32 `vtx_nonce`, + u8 `flags` (bit0 =
  `kAckKeyMismatch`, `0x01`). Unauthenticated — the drone has nothing of
  the GS's to prove yet, and a forged ack can only confuse a display, not
  move the aircraft (see "Not defended").
- **RCF** (GS to drone): + the 8-byte tag, computed over the frame bytes
  plus `vrx_nonce ‖ vtx_nonce ‖ seq32` (`TagCtx`) — session-bound, not
  just key-bound.
- **CAL_CMD / CAL_RESULT**: same tag, same `TagCtx` inputs (minus
  `seq32`: cal freshness is the cal nonce, see "Session and the nonces").
- **Telem** (drone to GS): flags bit1 is `kTelemAuthReject`
  (`0x02`) — at least one control frame failed verification this
  telemetry period. (Bit1 was `radio_rx_ok` before the 2026-09-30 telem
  diet; a recording from before that date reads bit1 as the old meaning —
  `docs/data-provenance.md`.)

`rc::verify_control(buf, len, key, ctx)` is the one place a tag is
checked, on both the drone and the GS cal path. `common/mabur/siphash.h`
is the primitive — SipHash-2-4, 128-bit key, 64-bit output, no library,
~60 lines, builds unchanged for armv7/aarch64/WASM.

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
- **Cal nonces are single-use per session.** Cal frames carry
  `seq32 = 0`, so freshness is the cal nonce: `RcAgent::verify_cal_frame`
  refuses a CAL_CMD whose nonce it already accepted in this session,
  other than the running sweep's own (a retransmission or its next phase,
  which `CalSweep` dedupes). The ring is `RcAgent::kCalNonceRing` (8)
  deep, belongs to the published session it was filled under, and is
  forgotten when that session is cleared or promoted. A refusal sets
  `auth_reject`. CAL_RESULT needs no ring — `CalSweep::on_result` takes
  only the running nonce.

## Rendezvous, as built

A DISC only ever elicits an ack — **no op, no retune, no LINKED entry**.
The first RCF that verifies under the acked pair is what actually links
the drone (`RcAgent::on_rc_frame` T_RCF: verify against `current_`, then
`pending_`; a `pending_` pass promotes it to `current_`, resets its
`seq32` tracker, calls `take_session_promoted()` true once).

Cold start:

1. GS beacons a tagged DISC at `beacon_period_ms` (fast
   `unacked_keepalive_ms` — 250 ms — until the first accepted ack, then
   the steady-state `beacon_keepalive_ms`, 1000 ms).
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
   edge. The existing split-and-reunite beaconing
   (`radio.scan.split_after_ms`) is the fallback if both are lost.

Keep-alive DISC while LINKED: verified like any DISC, acked with the
*same* `vtx_nonce` already issued for that `vrx_nonce` — nothing else
changes. A *different* `vrx_nonce` while the drone is LINKED means a
restarted GS: a fresh `pending_` pair, promoted on its first verified
RCF without touching the current op (the existing op-thrash rule — a GS
crash mid-flight must not drop the drone to the bottom rung).

### The stranger rule, as implemented

**As built:** the GS enters `VrxState::KEY_MISMATCH` when a flagged ack
arrives and the current flagged-only run (no unflagged ack since its first
flagged ack) satisfies BOTH:

- it is at least `VrxRendezvous::kKeyMismatchMs = 1000` old, and
- at least `VrxRendezvous::kKeyMismatchBeacons = 3` DISCs have gone out
  since its first flagged ack (`beacons_since_flagged_`, counted in
  `beacon()`).

In BEACONING (20 ms DISCs) the time term dominates: ~1 s. In SESSION the
DISCs are the 1 s keep-alives, so the beacon term does: ~3 s, the ack to
the third keep-alive after the first flagged one. That is what keeps a
foreign-key drone in range (it answers every DISC flagged, since without
`vtx_id` every drone answers) from tripping the state when one or two of
our own drone's acks are lost. Any unflagged ack resets both terms and,
if already in `KEY_MISMATCH`, leaves it immediately. In `KEY_MISMATCH` the
GS sends no RCFs (no cal either) and keeps beaconing, so a corrected
config on either end recovers without a restart on the other.

The held `vtx_nonce_` is **kept across `KEY_MISMATCH`**, not dropped —
a controller ruling made during implementation, not what a literal
reading of the spec's rendezvous text might suggest. Clearing it would
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
  drone flip between sessions and ladders and is unsupported (same as
  before this feature: two GSs with different seq positions fight over
  the op point). Signature: sustained `auth_reject`.

## Not defended

Carried straight from the spec's threat model (section 1) — nothing here
changed during implementation: jamming/DoS; a captured drone or a stolen
GS config leaking the key (change it); the browser's key storage, whose
security is the phone's lock screen; spoofed telemetry/DISC_ACK toward
the GS (unauthenticated by design — can confuse a display, never move
the aircraft); confidentiality of anything (video/OSD/telemetry stay
clear). **Same-key strangers**, not in the spec's list: without
`vtx_id`'s accidental filtering, a nearby default-key GS beaconing on the
same channel is answered exactly like our own GS — it only fails to win
a session because our GS's unflagged acks keep winning "newest wins",
not because the drone refuses it. It can thrash the drone's single
`pending_` slot (wasted work, not a session loss as long as our own
keep-alives keep landing).

## Known edges

All self-healing; found during the Task 9 reviews, not hardware bugs.

- **Stranger + three lost acks.** A foreign-key drone in range plus
  three consecutive lost acks from our own drone (~3 s of keep-alives in
  SESSION) still makes the GS's flagged-only run look like a real
  mismatch and enter `KEY_MISMATCH` mid-session; it leaves on the next
  unflagged ack, same as any entry. One or two lost acks no longer do
  (the time-only rule tripped on those).
- **Failsafe recovery via an old-session RCF.** If the GS's first
  recovering RCF lands on the old (stale) session before a fresh
  keep-alive DISC does, that DISC can hand the GS the old `vtx_nonce`
  back; RCFs against the stale `current_` session are rejected until the
  next DISC/RCF round re-pairs — about 1-2 s, bounded by the keep-alive
  cadence.
- **Promote Telem drains counters out of cadence.** The immediate Telem
  a session promotion sends is extra and off the regular period — it
  drains the drone's per-period counters early, shortening the next
  regular Telem's window. One short sideport sample per link-up, not a
  sustained effect.

## Observability

- Sideport: `link.key_fp` (`"default"` or 4 hex), `link.state` gains
  `key_mismatch`, `drone.auth_reject` (bool, this period).
- `tools/maburtop.py`: `key <fp>` in the link panel; `KEY MISMATCH` in
  place of the link state when flagged; `AUTH!` beside the drone panel
  on `drone.auth_reject`.
- `tools/flightreport.py`: `auth reject: N periods` — one period around
  a drone/GS restart is the expected transient; sustained means a bug or
  two controllers fighting over the session.
- Player OSD (`gs/player/src/gs_compact.cpp`): `KEY?` in the `ch:` slot
  that otherwise shows the stale/searching channel state — the one
  failure that would otherwise look exactly like the stale-caps restart
  deadlock and send the operator to `restart maburd`.
- `maburgs` stderr: one line on the `KEY_MISMATCH` edge (entry and
  exit) — `link KEY MISMATCH -- the drone rejects our tag; both ends
  need the same /etc/mabur.key (our key <fp>)` / `link key accepted
  (our key <fp>)`. `ctl.log` is untouched (`ctllog 12` unchanged):
  KEY_MISMATCH is a rendezvous state, not a ladder-controller event.
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
link (no loss, no reorder). Four cases:

1. **Matching keys link within a second.**
2. **Mismatched keys** end in `KEY_MISMATCH`, zero RCFs, the drone never
   reaches `LINKED`, no operating point beyond the automatic BOOT
   default is ever applied, no retune, and `auth_reject` is set.
3. **A same-key stranger drone beside ours** never trips `KEY_MISMATCH`
   on our own session — our drone still links, the stranger drone (wrong
   key relative to our GS) never does.
4. **A drone restart relinks through the GS's keep-alive DISC** — a
   fresh `RcAgent` against the same, still-running `VrxController`
   reaches `LINKED` within one keep-alive interval plus one RCF.

## Bench results

Filled in by the operator during the Task 9 Step 6 hardware gate — the
four timed recoveries, mirroring the spec's "Restart cases" above
(cold start / drone restart / GS restart / takeover), stopwatched from
`maburtop`.

| case | expected | measured |
|---|---|---|
| cold rendezvous, default keys on both ends | ~1 s |  |
| drone power cycle with GS running | <= 1.2 s |  |
| `restart maburgs` with drone running | <= 1.2 s |  |
| maburgs stopped, phone page connected with the key loaded | <= 1.2 s |  |
