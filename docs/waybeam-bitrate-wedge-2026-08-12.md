# waybeam bitrate wedge — encoder left at the 1400 kbps boot floor

Bench, 2026-08-12. Found while triaging a "bitrate collapsed below 1 Mbps"
report that initially looked like a regression from the vanish-detection
deploy (`65c94fd`). A/B (old vs new maburd, fresh restarts each) exonerated
the binary: both produced ~11 Mbps. Distinct bug, recorded separately.

## Symptom

- 60 fps of tiny frames: **~0.85 Mbps static, ~4 Mbps under motion**,
  indefinitely.
- Meanwhile every instrument reads healthy: `cmd_kbps=16000`,
  `waybeam_failures=0`, `/etc/waybeam.json` says `bitrate: 16000`, wire
  clean, fps normal.
- GS-log knee (1 Hz stats, s1 packet rate): ~580 pkts/s → 34 pkts/s within
  seconds of a maburd link-up.

## Mechanism (experiment-backed)

maburd's boot sequence commands `set_bitrate(1400)` (BOOT/max-range floor)
and then `set_bitrate(16000)` at LINKED seconds later. Occasionally the
second apply does not take effect in the encoder even though both HTTP
calls return ok and the config file records 16000.

Live probe on a healthy encoder reproduced the signature exactly:

- `set?video0.bitrate=1400` → **1.28 Mbps** static (matches the wedge)
- `set?video0.bitrate=16000` → **15.9 Mbps** immediately

So the wedged state is the encoder running at (or near) the boot-floor CBR
budget while the committed config says 16000. Static scenes undershoot the
tiny budget further (hence 0.85), motion overshoots it transiently (hence
~4).

Suspected locus: waybeam's apply path (`star6e_controls.c apply_bitrate` is
a Get/SetChnAttr read-modify-write that also fires its own rate-limited
IDR) racing the rest of the link-up window (ROI toggle low→normal, IDR
grants, config saves). Not proven: API sets are serialized on the httpd
thread, and the race would not reproduce on demand — 5 rapid ForceIdrs
fine, back-to-back bitrate sets fine, ~1-in-5 maburd restarts wedged that
day.

## Recovery

Re-apply the bitrate — no restart needed:

```
curl 'http://127.0.0.1/api/v1/set?video0.bitrate=16000'   # on the drone
```

(Any maburd restart also cures it, because link-up re-commands the
bitrate.)

## Fix directions (unbuilt)

1. **maburd watchdog**: enc byte-rate ≪ `cmd_kbps` for N s → re-apply
   `set_bitrate` + a telemetry counter. Heals any cause. Do NOT re-apply
   periodically instead: every `/api/v1/set` rewrites `/etc/waybeam.json`
   (77 flash writes in one bench day already).
2. **waybeam verify-after-apply**: read back `ChnAttr` after `SetChnAttr`,
   retry on mismatch — fix at the source; fold into the venc-ring 8→32
   rebuild trip.

## Verification gotcha (how it was missed at deploy time)

The first post-deploy ausniff had the collapse in plain sight — `bytes:
4508548` over 30 s = 1.2 Mbps — but the check read only fps/gaps/incomplete.
**ausniff verdicts must include the bytes-derived bitrate.**
