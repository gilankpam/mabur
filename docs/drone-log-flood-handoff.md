# maburd flooded /tmp with devourer `tx.agg` events

**Status:** fixed and verified on the rig 2026-07-25 (found the same day while
regression-testing the old-path deletion).

**One-line summary:** maburd tried to suppress devourer's per-URB JSON event
stream with `logger->set_level(Logger::Level::Warn)`, but that knob does not
gate events at all, so `/tmp/mabur.log` grew ~1.5 MB/min and filled the drone's
45 MB tmpfs in about half an hour — after which every maburd stderr write
failed and the log you would debug from was frozen.

## Symptom as seen on the rig

```
$ ssh root@192.168.10.152 'df -h | grep tmp'
tmpfs                    45.3M     45.3M         0 100% /tmp
$ ls -l /tmp/mabur.log
-rw-r--r--    1 root     root      47497216 Jul 25 09:26 /tmp/mabur.log   # frozen at 09:26
```

The process had been up 2.5 h; the log stopped at 09:26 because the filesystem
was full, not because logging stopped. Everything after that point — stats
lines, the frame_ring counters, any `FATAL`/`WARNING` — was silently lost. The
daemon itself kept running and video stayed clean.

Content is essentially all this, one line per aggregated USB URB:

```json
{"ev":"tx.agg","frames":3,"bytes":4367,"shim":false,"ok":true}
```

## Measurement (live, mcs5/ov0.25, ~8.2 Mbps video)

```
$ ssh root@192.168.10.152 'a=$(stat -c %s /tmp/mabur.log); sleep 20;
                           b=$(stat -c %s /tmp/mabur.log);
                           echo $((b-a)); grep -c tx.agg /tmp/mabur.log'
508548        # bytes in 20 s  => ~25 KB/s, ~1.49 MB/min
107557        # tx.agg lines already in the file
```

45 MB tmpfs / 1.49 MB per min ≈ **30 minutes to fill /tmp from empty**. The
init script (`bundle/S96mabur`) truncates the log on every respawn, so the
clock resets only when maburd restarts — a long clean run always ends up here.

## Root cause: the suppression was aimed at the wrong knob

`drone/src/main.cpp` already tried to fix exactly this:

```cpp
auto logger = std::make_shared<Logger>();
// Info-level events include one tx.agg line per aggregated URB (~600/s at
// video rate) — that floods the RAM-backed /tmp/mabur.log. Warnings only.
logger->set_level(Logger::Level::Warn);
```

The diagnosis in that comment was right and the fix did not work, because
devourer has **two independent output channels**:

| Channel | API | Gated by |
|---|---|---|
| Human diagnostics | `logger->trace/debug/info/warn/error(...)` | `set_level()` |
| Machine events (JSON Lines) | `devourer::Ev(logger->events(), "name")` | `EventSink::enabled()` only |

`Ev`'s constructor checks `_sink.enabled()` and nothing else
(`../devourer/src/Event.h:86-92`), and `EventSink` defaults to
**`_out = stdout`, `_enabled = true`, `FlushPolicy::EveryLine`**
(`Event.h:66-68`). `set_level()` never touches it. The emitter is
unconditional at `../devourer/src/jaguar3/RtlJaguar3Device.cpp:1791` (same in
jaguar1/jaguar2), inside the bulk-send loop:

```cpp
const int rc = _device.bulk_send_sync_ep(...);
devourer::Ev(_logger->events(), "tx.agg")
    .f("frames", ...).f("bytes", ...).f("shim", ...).f("ok", rc >= 0);
```

Events land in `/tmp/mabur.log` because they default to **stdout** and
`S96mabur` redirects `> /tmp/mabur.log 2>&1`.

Secondary cost: `FlushPolicy::EveryLine` makes this an `fwrite` + `fflush` per
URB (~600/s) on the TX path — and once the filesystem is full, a failing write
syscall per URB forever.

## The fix

Three call sites construct a `Logger` and hand it to devourer's TX path; all
three used the same wrong idiom, and all three now disable the event sink next
to the (still-correct, still-needed) `set_level` call:

- `drone/src/main.cpp` — maburd. Nothing operational is lost: the only event
  worth having on the TX path was `tx.fail`, and maburd's own stats line
  already carries `tx_failed=` from its `UsbTxPool::send_fail()` counter.
- `bench/linkbench/tx_main.cpp` and `bench/txagcbench/tx_main.cpp` — the
  latent-bug siblings flagged during the original triage, now confirmed: both
  are TX-side, both hit the same jaguar3 emitter, and nothing in `tools/` or
  `bench/` parses devourer events (the benches report on stderr), so this is
  pure removal of an `fwrite`+`fflush` per URB from a measurement path.

```cpp
logger->set_level(Logger::Level::Warn);
logger->events().disable();          // the API devourer's own examples use
```

`maburgs` is untouched and did not need it: measured 5925 bytes in 15 s with
**zero** `"ev"` lines (RX side never reaches the TX aggregation path) into a
990 MB tmpfs.

## Defence in depth: `bundle/S96mabur` caps the log

A daemon should not be able to blind its own log because a library it links
decided to be chatty. The init script now runs a `logwatch` sidecar that
truncates `/tmp/mabur.log` past 4 MB (checked every 60 s), tracked by its own
`/var/run/mabur-logwatch.pid` so `stop` reaps it.

The non-obvious part is the redirect: it changed from `>` to an explicit
truncate plus `>>`. Both start a run with an empty file, but **only O_APPEND
lets a third party truncate underneath a running daemon**. With a plain `>` the
daemon's file offset survives the truncate, so the next write recreates a
sparse file of the old apparent size and the cap never holds. Measured both
ways on the host (20 lines × 41 B, truncated mid-run): plain `>` ends at 820 B
apparent with a hole, `>>` restarts at 0 and ends at 574 B. The sidecar was
then self-tested on the drone itself under busybox with a 100 KB cap and a
synthetic writer — three truncations, final size 70 KB, 144 blocks actually
used, no sparse re-growth.

## Decision: fix mabur-side, do NOT flip devourer's EventSink default

The open question was whether the real fix belongs upstream — should
`EventSink` default to disabled, so no future consumer inherits this trap? The
answer is **no, not that way**, on evidence:

- Every env-driven demo would be fine: `apply_logging_env`
  (`examples/common/env_config.cpp:228-239`) *always* calls
  `events().configure(...)`, defaulting to stdout, so `DEVOURER_EVENTS`
  consumers never depend on the member default.
- But three examples never call `apply_logging_env` — `doctor`,
  `kestrelprobe`, `pcieprobe` — and every one of them exists to emit events
  (`doctor.verdict`, `adapter.caps`, `kestrel.id/power/fw/trx/phy/trig/twt`,
  `pcie.id/power/fw`). Their harnesses parse that stdout:
  `tests/adapter_doctor_cold.sh`, `tests/kestrelprobe_id.sh`,
  `tests/pcie_vfio_bind.sh`, `tests/ul_trigger_airs.sh`,
  `tests/twt_fw_discovery.sh`, `tests/chanscout_stress.sh`. A default flip
  breaks all of them **silently** — a disabled sink emits nothing, it does not
  error.

The better upstream change is narrower: **gate `tx.agg` itself**, because it
is the outlier, not the sink. Every other per-operation diagnostic event in
devourer already sits behind a `DeviceConfig::Debug` bool defaulting to
false — `debug.log_writes` for `debug.wreg` (per register write,
`src/RtlAdapter.h:174-183`), `debug.hop_prof` for `hop.prof`
(`src/HopProf.h:22-25`), plus `log_txpwr` / `dump_canary` / `bb_dump` /
`efuse_dump` / `gaintab_dbg`. `tx.agg` is the only event emitted at URB rate
with no flag at all, in all three generations (`jaguar1/RtlJaguarDevice.cpp:878`,
`jaguar2/RtlJaguar2Device.cpp:1241`, `jaguar3/RtlJaguar3Device.cpp:1791`). A
`debug.log_tx_agg` flag would match the existing convention, need no consumer
sweep, and fix the class of failure (a library flooding its host's log from a
hot path) without touching the sink contract.

That is a devourer-repo change with its own review, deliberately not bundled
here. Until it lands, the mabur-side `events().disable()` is the fix, and it
would remain correct afterwards anyway.

## Verification (rig, 2026-07-25)

Per the `rtp-regression-test` procedure — the tap is passive, so build and
deploy first, then tap.

- Host gate: 59/59 ctest pass.
- Cross-builds: `out/arm/maburd` md5 `ed700397c6dc5e6a66fe70a8a04d7b70`,
  `out/arm64/maburgs` md5 `e18bf80c4955e91044916467e345ce0d`.
- Attribution: rebuilding HEAD *without* this change reproduced the deployed
  binary's md5 `2dab28c2b63d2cef5725378c2b585e7c` byte for byte, so the drone
  was running exactly HEAD and this change is the only delta. The freshly
  built `maburgs` was already byte-identical to the deployed one, so the GS
  needed no redeploy.
- Deployed `maburd` + `S96mabur` to the drone (staged as `.new`, `sh -n`
  checked with the device's own shell, then `mv` + restart). `/tmp` went
  45.3M/100% → 40K/0%.
- **`grep -c tx.agg /tmp/mabur.log` = 0** after 2 min, and `grep -c '"ev":'`
  = 0. Growth **8.8–9.0 KB/min** over two independent windows (18290 B/125 s
  and 8978 B/60 s), down from 1.49 MB/min — a ~170× drop.
- The remaining traffic is exactly the lines we wanted to keep: 69 lines/min =
  60 `stats:` + 12 `maburd frame_ring:`, plus 6 one-shot startup lines.
- That rate puts the 4 MB logwatch cap ~7.5 h out and the 45 MB tmpfs ~85 h,
  so the cap always intervenes first and `/tmp` can no longer fill. `df /tmp`
  sampled every 10 min stayed at 0% for as long as it was watched; the
  arithmetic, not the sampling, is what makes a refill impossible.
- 30 s `rtpsniff.py lo 5600` on the GS: `pkts=22729 (8.21 Mbps)`, `gaps=0`,
  `out_of_order=0`, `end_lost_hard=0`, `frames=1785 ok=1784 bad=1 (0% bad)`,
  `ok_fps=59.5`. maburgs counters over the same window: `udp_fail=0 q_drop=0`,
  `bc=0 sbf=0 fl=0` on both streams, `ord[gap=0]`, SNR 58–65 dB both cards.

## Adjacent state (as of 2026-07-25)

- Drone disks were tight and are now not: removing three stale
  `/usr/bin/maburd.pre-*` backups took `/overlay` from **98% → 51%** (2.8 MB
  free). A 1.9 MB install fits; two do not.
- **There is no on-device rollback binary on either device** (deliberate — the
  backups are what filled `/overlay`). Recovery is a rebuild from git. For
  this deploy the outgoing binary was instead pulled to the host with
  `ssh root@… 'cat /usr/bin/maburd' > maburd.pre-evdisable`, which costs the
  drone nothing. Note the drone has no `sftp-server`, so plain `scp` fails —
  use `cat` over ssh (or `scp -O`).
- Anything lost while `/tmp` was full is unrecoverable: maburgs counters were
  unaffected and video was clean throughout the window, so probably nothing —
  but any drone-side incident before 2026-07-25 09:26 has no log tail.
