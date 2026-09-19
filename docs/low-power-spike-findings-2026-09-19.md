# Low-power (pre-arm) mode spike — 2026-09-19

Question: can the drone run cool while still streaming video, and return to
full power on arm? Bench only, **fan blowing on the drone the whole time**,
so every delta below is compressed; relative ordering is what this page
establishes. Drone .152 (SSC338Q, Runcam WiFiLink 2 / 8812EU), GS
10.18.0.1, channel 136, ladder free (rung 5 on the bench). SoC temperature
is `cpufreq/temp_out`; radio is devourer's `thermal_delta` (chip thermal
meter minus efuse reference); video gate is `ausniff.py --seconds 90`.
Each arm ran ~4 min after the change; the SoC settles within ~1 min with
the fan.

## Arms

| arm | SoC °C | radio Δ | CPU busy (2 cores) | ausniff |
|---|---|---|---|---|
| baseline: 800 MHz, 1080p60, 16 Mb/s | 61 | 6–7 | ~70 % | 60.3 fps, 0 gaps |
| CPU 600 MHz (min=max) | 59–61 | 6 | ~95 % | 60.1 fps, 0 gaps |
| **`encoder.bitrate_max_kbps = 2000`**, 60 fps | **48–50** | **1–2** | ~20 % | 60.2 fps, 0 gaps |
| + `venc.fps = 30` | 46–48 | 1–2 | ~15 % | 30.2 fps, 0 gaps |
| + CPU 400 MHz | 46–47 | 1 | ~35 % | 30.2 fps, 0 gaps |
| + cpu1 offline (1 core @ 400) | 46–47 | 1 | ~75 % (1 core) | 30.2 fps, 0 gaps |
| TX index override 4 (anchor 39), full 16 Mb/s 60 fps | 57 (from cold boot) | 5–6 | ~60 % | 60.1 fps, 0 gaps; GS RSSI −65/−66 vs −54/−56 |
| back to stock binary + config | 61 within 2 min | 6–7 | ~65 % | 60.2 fps, 0 gaps |
| maburd dead (reference floor) | 51 after 30 s | 2 | idle | — |

## What it says

1. **Bitrate is the lever, for both chips.** Capping the encoder at 2 Mb/s
   takes the SoC from 61 to 48–50 °C and the radio delta from 7 to 1–2 —
   most of the way to the maburd-dead floor (51 °C at 30 s, still falling).
   The radio cools because TX duty cycle drops ~8x; the SoC because VENC,
   FEC and the USB TX path scale with bytes, not frames.
2. **fps is a second-order lever**: 30 fps on top buys ~2 °C.
3. **CPU frequency and core count are NOT levers.** The vendor cpufreq
   driver (`ms_cpufreq_init` pins 800 MHz) exposes 400–1200 MHz but no
   voltage scaling (no regulator sysfs), so at a fixed workload a lower
   clock only raises busy % and the energy per frame is unchanged: 600 MHz
   at full load and 400 MHz / one core at low load all read the same as
   the arm before them. Also: `cpuinfo_cur_freq` reports the PLL, not the
   core (600 MHz shows 1199570, 400 shows 796917); a shell loop confirms
   600 and 400 are real. `userspace` governor's `scaling_setspeed` is not
   honoured. cpu1 can be hot-unplugged and maburd survives (its producer
   pin on cpu1 is silently lost).
4. **TX power is a weak lever at full duty.** Flat TXAGC index 4 (anchor
   mcs7 index 39, ~12 dB less at the GS) read one step lower on the radio
   delta and 2–4 °C on the SoC, both confounded by post-boot warm-up. At
   2 Mb/s the radio delta is already 1–2, so there is little left for
   power to take. Not worth the calibration-table interaction.
5. **Full power comes back cleanly**: stock config + binary relinked and
   returned to baseline temperature within 2 min, RSSI restored.

## Fan OFF, low-power mode (2 Mb/s + 30 fps) — the number that matters

Config applied by swap + restart (`/etc/mabur.toml.pre-lp` is the stock
copy), fan switched off at drone uptime 1355 s with the SoC at 57 °C.

| min after fan off | SoC °C | radio Δ | note |
|---|---|---|---|
| 0 | 57 | 3 | |
| 1 | 59 | 4–5 | |
| 2 | 63 | 5–6 | still +3 °C/min |
| ~4 | 63 | 6 | **radio dropped off USB, maburd aborted** (below) |
| 7 (after restart) | 66 | 6 | |
| 8 | 70 | 7–8 | |
| 10 | 76 | 9 | still +2 °C/min |
| 12 | 79–80 | 9–10 | |
| 15 | 84 | 10–11 | slope ~1 °C/min, flattening but not flat; no second USB drop |

So with no airflow the "cool" mode does not hold the SoC anywhere near the
fan-on 46–48 °C: the board climbs through 76 °C in 10 min at 2.4 Mb/s and
15 % CPU. The fan was doing ~25 °C of work. At 15 min it is 84 °C and still
creeping, i.e. above flight 21's 81 °C reading, in the *low-power* mode. (For scale: flight 21's incident
was 81 °C at full rate in the air; long bench sessions without a fan sat
at 70–73.)

### The USB drop

~4 min after the fan went off (SoC 63 °C, radio Δ 6) the kernel logged
`usb 1-1: USB disconnect, device number 2` and re-enumerated the same
card as device 3. devourer's every `bulk_send EP 5` then failed with
`rc=-4` (LIBUSB_ERROR_NO_DEVICE), a register read threw
`rtw_read: iostream error`, and maburd aborted. Not attributable to heat
from one event — the dongle has prior "comes up deaf"/"wedges after
linkbench" history — but it is the first drop seen while the temperature
was being watched, and it happened well below the SoC's flight reading.
If it repeats under heat, the disarmed-idle assumption (radio stays up)
is the thing to fix, not the mode.

### Harness gotcha: the wrapper's respawn loop dies on SIGPIPE

`S00mabur`'s `loop()` writes "maburd exited (N) - respawn in 2s" to
stderr. Started from an ssh session (as in this spike), that stderr is
the session's pipe; once the session is gone the write raises SIGPIPE
and kills the loop subshell, so maburd never respawns — only `logwatch`
survives (visible as a lone `{S00mabur}` in `ps`, sleeping). At boot
stderr is the console, so the field is unaffected. Start it detached
when testing: `nohup /etc/init.d/S00mabur start </dev/null >/dev/null 2>&1 &`.

## What the arm-wiring will need (not built)

- The two winning knobs are `encoder.bitrate_max_kbps` (and optionally
  `venc.fps`) — today they are config keys read at boot. RcAgent already
  changes bitrate at runtime via `SetChnAttr` (one IDR per write, see
  `venc-attr-change-idr`), so a runtime "preview cap" on the bitrate policy
  is an agent-side clamp, no restart. A runtime fps change is a venc
  attribute change; untested whether the SDK takes it live.
- A restart is NOT an acceptable transition: `S00mabur stop` leaves
  `/var/log/devourer-usb-1-1.lock` behind (respawn loop "USB adapter in
  use" until it is removed) and a warm restart hit the stale-MI-worker
  crash once in four tries (reboot to clear).
- The trigger (armed/disarmed) is not on this page; it will arrive over
  MSP from the FC, which maburd already reads.
- The fan-off run above says the mode alone does not keep the board cool
  on the ground with no airflow; on the airframe the disarmed drone has
  no prop wash either. Expect the mode to slow the climb, not stop it —
  measure the full-rate fan-off slope next to know what it buys.

## Can maburd read the FC's arm state? Yes — MSP_STATUS on the OSD UART

Today maburd only *listens* on `/dev/ttyS2` (`MspSerial` is read-only;
`MspSource` parses the DisplayPort stream and forwards it as stream 4).
The FC pushes nothing but DisplayPort (cmd 182) on its own, so the arm
state has to be *asked for*. Probed with maburd stopped (exclusive UART),
raw 115200 8N1, 10 requests each of `MSP_STATUS` (101) and
`MSP_STATUS_EX` (150) at 2 Hz:

```
frames (dir,cmd,crc_ok): {('>',182,True): 1050, ('>',101,True): 10, ('>',150,True): 10}
cmd 101 len 27: cycle=123 sensors=0x002b flightModeFlags=0x00000002 ARM_bit0=0 profile=0
cmd 150 len 27: cycle=124 sensors=0x002b flightModeFlags=0x00000002 ARM_bit0=0 profile=0 cpu_load=42
```

- Every request was answered, checksum-clean, and the DisplayPort stream
  kept running at 150 frames/s alongside — the port is bidirectional MSP,
  exactly what msposd relies on for its record-on-arm.
- `flightModeFlags` bit 0 is BOXARM (id 0) in both Betaflight and iNav;
  it read 0 on the disarmed bench FC, bit 1 (ANGLE) set. The flip to 1
  on arming is not yet observed here — arm the bench FC (props off) once
  the request path is in maburd.
- Sharing the UART with a second reader (a `cat` next to maburd) splits
  bytes between readers and breaks checksums — the probe MUST be done
  from inside maburd's own reader, not a side process.
- Non-182 messages already fall through `MspScreen::apply` untouched, and
  `msp_append_message` builds v1 requests, so the drone side needs only:
  a `write()` on `MspSerial`, a 2 Hz `MSP_STATUS` request from the msp
  thread, and `cmd == 101 → armed = payload[6..9] & 1` handed to RcAgent
  as the bitrate-clamp input. Nothing on the wire to the GS changes.
