# Drone boot time — baseline, the `quiet` A/B, and the U-Boot attribution, 2026-09-07

Question that started it: the drone boots visibly slower than a DJI O4 or a
Walksnail Avatar air unit, and `maburd` no longer depends on a second daemon
— is full Linux still the right substrate, or should the SoC run something
smaller?

Short answer: **keep Linux.** The time is not spent being an OS. It is spent
in U-Boot and in generic OpenIPC init that runs before `maburd` gets a turn.
Both are ours to cut. Numbers below.

## Why "not Linux" is not on the table

Not a preference — three hard dependencies:

- `maburd` is not dependency-free. `drone/venc/star6e_*.c` and
  `isp_runtime.c` `dlopen` the SigmaStar MI libraries at runtime
  (`libmi_sys`, `_vif`, `_vpe`, `_venc`, `_sensor`, `_isp`, `_ipu`,
  `libcam_os_wrapper`), and those sit on 14 vendor kernel modules that
  `load_sigmastar` insmods. Closed, Linux-only, prebuilt against kernel
  4.9.84. SigmaStar's fast-boot story for this family is a dual-OS
  arrangement under NDA that OpenIPC does not have.
- The radio is a USB dongle driven by devourer through libusb. An RTOS
  would need its own USB host stack plus a port of the 8812eu driver.
- The glibc-dynamic build is load-bearing, not incidental — see the header
  comment in `tools/build-arm.sh`. A static binary has no loader to
  `dlopen` the MI blobs with.

DJI and Walksnail air units run Linux too. Their advantage is a boot path
built for one job.

## Rig

Two rigs, in this order. **Everything down to the end of the `quiet` A/B is
the network rig described here; from "Serial console" onward it is the
serial one**, on a different board, and that section states its own caveats.

No serial console — the UART0 TX solder pad on the bench board is destroyed
(see "What is still blocked"). Everything in the next three sections is
measured over the network, from three clocks, reconciled:

- **Host**, `date +%s.%N` around each step, plus a 20 Hz ping loop to catch
  the moment the drone drops off the net (the closest proxy for power-on
  available without a console).
- **Drone kernel clock**, `/proc/uptime`, and — the precise part — process
  start times from field 22 of `/proc/<pid>/stat`, which is ticks since boot
  at `CONFIG_HZ=100`. This dates every rcS entry after the fact on one clock,
  with no polling skew.
- **GS**, `wseq` on the maburgs AU ring at `/dev/shm/mabur-au`, polled at
  200 Hz through `ausniff.open_ring`, stamping the stall and the resume.
  The GS clock is unset (2017), so a sandwiched `host → gs → host` read gives
  the offset.

Kernel t=0 in host clock comes from the first successful ssh:
`kernel_zero = host_time − uptime_read_in_that_session`. Biased late by the
ssh round-trip tail (~0.1 s), common-mode across runs, so A/B deltas survive
it; absolute pre-kernel numbers carry that error.

Two gotchas worth writing down:

- The in-drone poller must be piped over ssh stdin (`ssh host 'sh -s' <
  poll.sh`). Anything staged in `/tmp` beforehand is gone — `/tmp` is tmpfs.
- ssh does not answer until ~7 s of uptime, so nothing that happens before
  that can be observed live. It is recovered afterwards from process start
  times and from the venc frame counter, never from poll timestamps.

## Baseline

Two reboots for the timeline, then A1/A2 below as the A/B control. Stage
costs, seconds:

| stage | window | cost |
|---|---|---|
| reset → kernel t=0 (**wrong — see below**) | host clock | **6.16** |
| kernel init → first rcS entry (`S01syslogd`) | 0 → 2.84 | 2.84 |
| `S01syslogd` → `S50dropbear` | 2.84 → 3.65 | 0.81 |
| `S60crond` + `S70vendor` (`load_sigmastar`) | 3.65 → 5.11 | 1.46 |
| `S96mabur` → first encoded frame | 5.11 → 6.56 | 1.45 |
| first frame → radio RX loop | 6.56 → 9.20 | 2.64 |

End to end, net-drop → first AU in the GS ring: **15.0 / 15.1 s** across the
two runs. Issuing `reboot` adds a further, variable 1.5–2.1 s of shutdown
before the net drops, which is why the wall-clock figure from a `reboot`
command ranges 16.6–17.1 s and the net-drop number is the one to quote.

**Correction, from the serial console (see "Pre-kernel, actually
attributed"): the 6.16 s row is not pre-kernel.** The net drops ~3.4 s
*before* the CPU resets, so that row is shutdown tail plus a real pre-kernel
cost of ~2.8 s, and every "reset →" figure in this table is inflated by the
same ~3.4 s. Reset → video is ~11.6 s, not 15.0. The A/B deltas below are
unaffected: the bias is common-mode.

The radio RX-loop log line lands ~0.1 s *after* video is already flowing —
`device_ready.store(true)` opens the TX gate before the `fprintf` in
`drone/src/main.cpp:1854` — so "rxloop" is used below as the end marker in
place of the GS ring when the GS is unreachable.

Notes on the individual stages:

- `load_sigmastar` re-detects the sensor on every boot (insmod
  `sensor_config`, probe `/dev/srcfg`, rmmod, `ipcinfo -s`, then insmod the
  real driver) even though rcS has already exported `SENSOR` from the U-Boot
  env, where it is pinned to `imx415`. It also loads `mi_ai`, `mi_ao`,
  `mi_mipitx`, `mi_ldc`, `mi_divp`, `mi_rgn` and `mi_shadow`, none of which
  `maburd` opens.
- The 2.64 s from first frame to radio-ready contains devourer's `InitWrite`
  — power-on, 8812eu firmware download, TX enable — but its exact share is
  **unmeasured**, because `maburd`'s log lines carry no timestamps of their
  own and the poller cannot attach before ~7.3 s.
- Nothing before 5.11 s is video work. The overlay mount, four generic
  daemons and the sensor autodetect sit in front of `maburd`.

## The `quiet` A/B

Hypothesis: the console at 115200 is synchronous, ~434 lines land in the
kernel ring buffer per boot, so the UART transmit itself is costing real
time — and it is being paid whether or not anything is attached to the pad.

Method: append `quiet loglevel=1` to `bootargs` via `fw_setenv`, leaving
every other token byte-identical. Four runs interleaved A/B/A/B so drift
cannot fake the result.

| marker (s since kernel t=0) | A1 | A2 | B1 | B2 | Δ |
|---|---|---|---|---|---|
| pre-kernel (host clock) | 6.16 | 6.16 | 6.09 | 6.02 | −0.10 |
| `S01syslogd` | 2.86 | 2.81 | 2.21 | 2.11 | **−0.68** |
| `S50dropbear` | 3.67 | 3.63 | 2.99 | 2.94 | −0.68 |
| `S96mabur` | 5.13 | 5.09 | 4.43 | 4.41 | −0.69 |
| first encoded frame | 6.57 | 6.55 | 5.98 | 5.86 | −0.64 |
| radio RX loop | 9.19 | 9.21 | 8.55 | 8.45 | −0.70 |
| net-drop → RX loop | 15.35 | 15.37 | 14.64 | 14.47 | −0.80 |

Runs within an arm agree to ±0.1 s.

**Result: 0.68 s.** The pre-measurement estimate in this session was ~2 s and
was 3x too high. 0.68 s at 115200 is ~7.8 kB actually transmitted, about a
third of the ring buffer's bytes — most lines were already below the default
console loglevel and never reached the wire.

Three things the shape of the table says:

1. **The whole saving is banked before rcS starts** and is then carried
   forward unchanged. Console printk cost lives entirely in kernel boot,
   0–2.8 s.
2. **Module load and `maburd`'s MI chatter are free on the console.** The
   `syslogd → maburd` window is 2.27 s baseline vs 2.22 s quiet. So the
   remaining stages are real work, not logging — worth knowing before
   optimising the wrong one.
3. **`dmesg` still holds all 434 lines** with quiet on. The ring buffer is
   untouched; only console transmission is suppressed. No loss of post-boot
   diagnosability.

The −0.10 s on the pre-kernel row is not claimed: a kernel cmdline parameter
cannot affect U-Boot, and it is within that measurement's ssh-tail noise.

**Reverted.** The device is back on the original `bootargs`, verified
byte-identical to the pre-experiment backup. Two reasons not to leave it on:
it is a persistent change that was not asked for, and `loglevel=1` would
blank the very console currently being brought up. Re-apply when the console
work is done:

```sh
fw_setenv bootargs 'console=ttyS0,115200 quiet loglevel=1 panic=20 \
  root=/dev/mtdblock3 init=/init \
  mtdparts=NOR_FLASH:256k(boot),64k(env),2048k(kernel),${rootmtd}(rootfs),-(rootfs_data) \
  LX_MEM=${memlx} mma_heap=mma_heap_name0,miu=0,sz=${memsz}'
```

Single quotes are required — `${rootmtd}`, `${memlx}` and `${memsz}` must
reach the env as literals for U-Boot to expand at boot.

## Serial console, 2026-09-07 (later) — the rig that unblocked all of this

A CP2102 USB-UART is now attached to a **spare** SSC338Q/infinity6e board at
`192.168.10.95`, `/dev/ttyUSB0` at 115200 8N1. Both directions work: the
board's getty answers a bare CR, and a `\r` typed during autoboot lands at
the `OpenIPC #` U-Boot prompt.

Read the board caveat before trusting any number below. `.95` is **not the
mabur drone**. It is an older, pre-mabur OpenIPC image (2.6.01.02,
`master+4c53cf5`), runs `majestic` + `wifibroadcast` in userspace, and its
camera may be dead. What it shares with the drone is everything that matters
for the block that was blocked:

| | mabur drone | `.95` |
|---|---|---|
| SoC | SSC338Q, infinity6e | same |
| U-Boot | 2015.01 (Nov 01 2023), `I6E#g4728ac3` | same |
| NOR layout | 256k boot / 64k env / 2048k kernel / rootfs | same |
| `bootargs`, `bootcmd` shape | `console=ttyS0,115200 … mtdparts=NOR_FLASH:…` | byte-identical shape |
| kernel | 4.9.84-ssc338q | same version |
| userspace | `maburd` | `majestic`/`wifibroadcast` — **does not transfer** |

So: **U-Boot and kernel-init numbers transfer, userspace numbers do not.**
Every figure in the three sections that follow is U-Boot or kernel.

Two things worth knowing about driving this U-Boot:

- **A bare LF re-runs the previous command.** Sending `\r\n` executes
  everything twice. Send `\r` only. This cost an experiment before it was
  spotted — the doubled `sf read` was mistaken for a slow read.
- `bootdelay` was raised 0 → 2 to get a recovery window. On the **stock**
  U-Boot it costs **nothing measurable**: with no key pressed the delay loop
  is skipped entirely (`sstar_emac` → `sf read` is 0.393 s at `bootdelay=0`
  and 0.401 s at 2). **This stops being true on the rebuilt U-Boot** — see
  "Rebuilding U-Boot" below, where `bootdelay=2` costs 1.195 s and the right
  setting is 0. The recovery window survives either way: a CR spammed during
  boot reaches the prompt even at `bootdelay=0`.
- Experiments are run **non-persistently**: at the prompt, `setenv bootcmd
  …` then `run bootcmd`. A reset throws it all away. Nothing needs to be
  saved until it is proven.

## Pre-kernel, actually attributed

Reset to `Starting kernel ...`, cable connected, stock env, two runs. This
replaces the single 6.16 s "reset → kernel t=0" row in the baseline table.

| stage | run 1 | run 2 |
|---|---|---|
| IPL: DRAM/MIU PLL/BIST → `JMP+++` | 0.200 | 0.200 |
| IPL_CUST: XZ-decompress U-Boot (0x55CA8) | 0.092 | 0.098 |
| U-Boot init → `Net:` (I2C, DRAM, MMC, 2× flash detect, env) | 0.157 | 0.151 |
| **Ethernet auto-negotiation** | **1.392** | **1.192** |
| `sf read` 2 MiB kernel from NOR | 0.396 | 0.404 |
| `bootm` CRC verify | 0.250 | 0.250 |
| `bootm` copy image to load address | 0.349 | 0.349 |
| **reset → `Starting kernel ...`** | **2.910** | **2.709** |

Stage costs repeat to a few ms; only auto-negotiation moves (±0.2 s).

**The 6.16 s in the baseline table was not pre-kernel.** It was measured
net-drop → kernel, and on the serial console the network stops **3.45 s
before the CPU actually resets**:

```
  0.301  Stopping network...
  1.652  The system is going down NOW!
  2.651  Sent SIGKILL to all processes
  3.750  reboot: Restarting system      <- the real t=0
```

3.45 s of shutdown + 2.7 s of pre-kernel ≈ 6.2 s, which is the 6.16 s that
was booked as pre-kernel. So roughly **half of that block was the shutdown
tail of the `reboot` command** and does not exist on a power-on. Every
"reset →" figure in the baseline table is inflated by ~3.4 s, and the 15.0 s
end-to-end is really ~11.6 s from reset. The A/B deltas are unaffected —
the bias is common-mode, exactly as that section claimed.

## The 4-second finding: auto-negotiation with no cable

U-Boot brings the MAC up and waits for link on every boot, although
`bootcmd` is only `sf probe; sf read; bootm` and never touches the network.
With a cable that costs 1.19–1.39 s. **In the configuration the drone
actually flies in — no Ethernet — it does not fast-fail, it times out:**

```
  0.451  Auto-Negotiation...
  4.496  AN failLink Status Speed:10 Full-duplex:0
  4.499  Status Error!
```

**4.045 s.** +2.65 s over the cable-connected case, and it is pure waste in
both. The kernel and userspace timelines are unchanged with the cable out
(every marker within 0.05 s of the connected boot), so the entire penalty
sits in U-Boot.

This is the single largest item in the whole boot, and **every measurement
taken before the serial console was blind to it** — the old rig needed the
network, so it could only ever measure the 1.3 s case. Flight-configuration
pre-kernel is ~5.5 s on stock env, not 2.8 s.

Caveat: **n = 1.** The mechanism is explicit and the delta is 2.65 s against
±0.2 s run-to-run noise, but it has been measured once. Repeats are cheap
the next time the board is parked at the U-Boot prompt with the cable out —
`reset` at the prompt needs no network.

No env knob was found for it; `Auto-Negotiation...` is printed
unconditionally from the `sstar_emac` driver's init. Killing it means
rebuilding U-Boot, and reflashing U-Boot is the one operation with no
recovery path short of an SPI programmer.

## Two env-only savings, validated

Both proven non-persistently at the prompt first, then persisted with
`fw_setenv` and re-measured end-to-end.

1. **`verify=no`** — skips the `bootm` CRC over the 2 MB image; the
   `Verifying Checksum ...` line disappears entirely. **−0.250 s.**
2. **`baseaddr=0x20007FC0`** — the image copy exists only because the kernel
   is read to 0x21000000 and then memmoved to its 0x20008000 load address.
   Read it 64 bytes (one uImage header) below the load address instead and
   U-Boot prints `XIP Kernel Image ... OK` and skips the copy: 0.349 →
   0.049 s. **−0.300 s.** One variable does both, because `bootcmd` is
   `sf read ${baseaddr} …; bootm ${baseaddr}`.

Quote those two from the **autoboot** path, which is what ships. Measured at
the U-Boot prompt instead, the copy is identical (0.349 s) but the CRC reads
0.349 s rather than 0.250 s, and the whole `bootm` block runs 0.06 s slower.
Unexplained, and the reason to trust the autoboot column:

| `bootm` → `Starting kernel` | autoboot | at the prompt |
|---|---|---|
| stock | 0.662 / 0.654 | 0.721 / 0.721 |
| `verify=no` | — | 0.372 |
| `verify=no` + XIP | 0.103 | 0.072 |

End-to-end, matched against the baseline run with the same 1.392 s
auto-negotiation: **pre-kernel 2.910 → 2.360 s, −0.550 s**, everything after
`Starting kernel` unchanged (kernel entry → console 1.073 vs 1.076 s; rootfs
mount +2.440 s in both). Board boots clean to userspace and ssh.

Both are live on `.95` now. To apply on the drone:

```sh
fw_setenv verify no
fw_setenv baseaddr 0x20007FC0
fw_setenv bootdelay 2      # recovery window; costs nothing
```

Do the `bootdelay` one **first**, and only with a serial console attached —
`baseaddr` is also the scratch address for `uknor`/`urnor`/`ubnor`, so a
typo there is a firmware-update path you cannot reach without a prompt.
Why this is safe: 0x20007FC0 + 16 MiB is well inside `LX_MEM=0xFFE0000`,
and U-Boot has already relocated itself to the top of RAM.

## Kernel init, with printk timestamps

`printk.time=1` appended to `bootargs` in RAM at the prompt — no persistent
change, exactly the trick the old blocked-work section proposed, and it does
work with `CONFIG_PRINTK_TIME` unset.

One trap: **`sched_clock` does not start until the arch timer registers**,
so every message before `Switching to timer-based delay loop` stamps
`0.000`. Timestamps are only meaningful after that point; the wall-clock
serial stamps carry the rest.

| window | cost |
|---|---|
| `Starting kernel ...` → console registered | 1.07 |
| …of which is before `sched_clock` starts (unattributable) | ~0.76 |
| console registered → rootfs mounted | 1.27 |
| rootfs mounted → first rcS entry | 1.01 |

Nothing in the driver window is individually large. The one clearly wasted
item: the MDIO bus rejects the device tree's PHY node —

```
mdio_bus mdio-bus@emac0: /soc/emac0/mdio-bus/ethernet-phy@0 has invalid PHY address
mdio_bus mdio-bus@emac0: scan phy ethernet-phy at address 0 … 31
```

— then scans all 32 addresses and finds it at address 0 anyway. **0.185 s**,
identical with and without a cable, fixable in the DT. Small, but it is a
bug, not work.

## Rebuilding U-Boot: the auto-negotiation fix, measured

Done and flashed on `.95` on 2026-09-07. **Pre-kernel 2.907 s → 0.904 s with
a cable, and 4.911 s → 0.854 s without one.**

| config | pre-kernel, cable | pre-kernel, no cable |
|---|---|---|
| stock U-Boot, stock env | 2.907 | ~5.5 (derived) |
| stock U-Boot, `verify=no` + XIP | 2.356 | 4.911 |
| **rebuilt U-Boot, + `bootdelay=0`** | **0.904** | **0.854** |

Cable and no-cable are now the same number, which is the whole point: the
flight configuration no longer pays a penalty for having no Ethernet.

### Source and build

Upstream is `OpenIPC/u-boot-sigmastar` ("U-Boot for Infinity6xx"). The two
changes below live on our fork, **`gilankpam/u-boot-sigmastar`, branch
`mabur-fastboot`** (`f8a00c4`, on top of upstream master `bf77aff`).

`openipc-builder` (branch `feat/mabur`) builds that fork as part of a normal
device build: `build_uboot()` runs after `make BOARD=`, clones the fork and
compiles it with the Buildroot toolchain the device build has just produced,
so no second cross compiler is involved. It drops three files into
`output/images` and the timestamped archive:

| file | what it is |
|---|---|
| `u-boot-<soc>-nor.bin` | raw `BOOT.bin` |
| `u-boot-<soc>-universal.bin` | same, under the name `autoup_rootfs` expects |
| `u-boot-<soc>-nor-padded.bin` | padded to the 256k boot partition with 0xFF, ready for `flashcp` |

`SKIP_UBOOT=1` turns the step off; `UBOOT_REPO`, `UBOOT_REF`, `UBOOT_DIR`
and `UBOOT_PART_SIZE` are overridable. An unknown SoC or a missing toolchain
skips the step rather than failing the build. Worth knowing why this was
needed at all: for ssc338q the builder previously produced **no U-Boot**,
because the prebuilt download only feeds `autoup_rootfs` and that runs for
`hi3518ev200_lite` alone.

To build it by hand instead:

```sh
export PATH=$HOME/Projects/drone/openipc-builder/openipc/output/host/bin:$PATH
export ARCH=arm CROSS_COMPILE=arm-openipc-linux-gnueabihf-
make distclean && make infinity6e_defconfig
make -j8 KCFLAGS=-DPRODUCT_SOC=ssc338q
nix-shell -p bc --run "sh make_boot_spinor.sh infinity6e"   # -> BOOT.bin
```

The mabur/OpenIPC glibc toolchain (gcc 13.3, hardfloat) builds this 2015-era
U-Boot cleanly — upstream's `arm-linux-gnueabi` gcc 11.4 is not needed. `bc`
is the only missing host tool and only for packaging.

**Build at master, not at the commit the banner names.** The device reports
`I6E#g4728ac3`, and that commit does reproduce the same uImage name — but
its prebuilt `ipl/infinity6e/*.bin` blobs do **not** match the device's.
Master's match byte-for-byte against a `/dev/mtd0` dump over `0..0x20000`.
Those blobs are the unrecoverable early stages, so master is the *safer*
base, not the riskier one. Verified on both builds.

### The three changes

1. **`CONFIG_ETHERNET_FIXLINK=y`** in `configs/infinity6e_defconfig`
   (was `# ... is not set`). Config-only, no patch. It compiles out the
   `MHal_EMAC_NegotiationPHY()` call at `drivers/mstar/emac/mdrv_emac.c:541`
   and hardwires the MAC to 100/full. The stall itself is
   `drivers/mstar/emac/infinity6e/mhal_emac.c:1028` — `counter > 20` ×
   `mdelay(200)`, i.e. exactly the 4.045 s measured. **Measured: `Net:` →
   `sstar_emac` 1.444 → 0.003 s, and no `AN fail` with the cable out.**
   Cost: U-Boot's own `tftpboot` (used by `uknor`/`urnor`) now only works on
   a 100full link. Linux is unaffected — it re-probes the PHY itself.
2. **Patch out the SD boot-script probe** in `common/autoboot.c`. Master
   added two unconditional `mmc_get_dev()` calls just before `bootcmd` runs;
   `mmc_get_dev(0)` initialises an empty SD slot and `mmc_get_dev(1)` prints
   `MMC Device 1 not found`. **−0.10 s.** They are guarded behind
   `CONFIG_SSTAR_SD_BOOTSCRIPT` (never defined) rather than deleted. SD
   support is kept, so `fatload mmc 0` and the SD update path still work.
   Note an upstream bug found on the way: those calls are not guarded by
   `CONFIG_GENERIC_MMC`, so setting `CONFIG_MS_SDMMC=n` fails to link with
   `undefined reference to mmc_get_dev`.
3. **`bootdelay=0`** — and this one is a **correction to the "costs nothing"
   claim in the section above**. That was measured on the *stock* U-Boot,
   where it was true. On master's autoboot it is not: `bootdelay=2` costs
   **1.195 s**. The recovery window is still free, because a CR spammed
   during boot reaches the `OpenIPC #` prompt at `bootdelay=0` too —
   verified after flashing. So: keep `bootdelay=0` and interrupt with CR.

### Flashing

`flashcp BOOT.bin /dev/mtd0` from Linux on the device, having padded the
image to the full 262144 with 0xFF. This preserves the env in mtd1.

**Do not use the `ubnor` env command:** it is `sf erase 0x0 0x50000`, which
wipes mtd0 *and* the env at 0x40000, losing `ethaddr` and every `fw_setenv`
tweak. Save `fw_printenv` first regardless — and dump `/dev/mtd0` to a file
first, which is a byte-exact rollback as long as the new U-Boot still boots
Linux. If it does not, there is no recovery short of an SPI programmer; the
IPL will still run and print, but it cannot load an alternative payload.

Every flash here was verified by reading `/dev/mtd0` back and comparing md5
against the file that was written.

**The builder-produced image is the one now running on `.95`** (2026-09-08):
a full `builder.sh ssc338q_fpv_openipc-urllc-aio` run, then
`flashcp u-boot-ssc338q-nor-padded.bin /dev/mtd0`. It reports
`Version: I6E#f8a00c4#`, boots to userspace and ssh, and measures pre-kernel
0.856 s against the hand-built image's 0.904 s — the 0.05 s is run variance
in the `bootm` block, not a difference between the two builds. So the whole
path from `builder.sh` to a booting board is verified, not just the
hand-assembled one.

## Ranked next steps

Re-sized against a **flight-configuration** boot (no Ethernet), which is
~2.7 s longer than anything measured before the serial console existed.
Items 1-3 are measured; the rest are estimates.

1. ~~**Kill U-Boot's Ethernet auto-negotiation**~~ — **DONE and measured**,
   see the section above. Pre-kernel 2.907 → 0.904 s with a cable, 4.911 →
   0.854 s without. Live on `.95`; not yet on the drone.
2. **`verify=no` + `baseaddr=0x20007FC0`** — **0.55 s**, measured, env-only,
   reversible, already live on `.95`. Included in the number above.
3. **`quiet loglevel=1`** — 0.68 s, measured earlier this session,
   config-only. Re-apply once the console work is done; it blanks the
   console it is being measured on.
4. **Overlap devourer `InitWrite` with the MI bring-up** in `maburd` —
   estimated ~1.5 s. Today they are strictly serial and the encoder spends
   the gap producing frames into the void (`drops=309`, `sent=0`).
   Code-only, no deploy-order hazard. Now cheap to verify: timestamp
   `maburd`'s log lines and read them off the console.
5. **A mabur-specific device profile and a custom rcS** — estimated
   1.5-2 s. Pin the sensor from the U-Boot env instead of autodetecting,
   insmod only the MI modules `maburd` opens, start `maburd` before
   syslogd/ntpd/dropbear/crond. Note the current
   `ssc338q_fpv_openipc-urllc-aio` profile still sets
   `BR2_PACKAGE_WAYBEAM_VENC=y`, so a fresh image would ship and start
   waybeam again at S95; and `package/mabur/mabur.mk` pins commit
   `423d286`, 542 commits behind HEAD.
6. **Fix the emac PHY node in the device tree** — 0.185 s, measured. The
   MDIO bus scans all 32 addresses because `ethernet-phy@0` has an
   "invalid PHY address", then finds the PHY at 0.
7. **LZO or LZ4 instead of XZ** for kernel and squashfs, funded by dropping
   majestic, waybeam, vtund, curl, mbedtls, opus, ntpd, crond, wireguard,
   wpa_supplicant and exfat from the profile — estimated 1-2 s, unverified.
   Note the XZ cost now has a measured reference point: decompressing
   U-Boot's own 351 kB image takes 0.092-0.098 s.
8. **Shrink or relocate the jffs2 overlay** — up to ~1.5 s, but it holds the
   config and the imx415 ISP tuning bin, so those need a home first.

Still worth doing before 4-8: **timestamp `maburd`'s own log lines** with a
monotonic prefix, so the 2.64 s radio window is directly measurable.

Realistic floor with this SoC, the vendor ISP blobs and a USB dongle: 5-6 s
from power to video. Items 1-3 alone are 5.2 s of measured, mostly cheap
savings against that.

## What is still blocked

Much less than before. The serial console resolved the pre-kernel
attribution outright.

- **Netconsole is not an option and no longer needs to be.** Confirmed dead:
  the U-Boot image was pulled from `/dev/mtd0`, XZ-decompressed (the 351400
  bytes match the `decomp_size=0x00055ca8` it prints), and contains no
  `netconsole`/`ncip` strings.
- **The mabur drone still has no console.** `.95` is a stand-in, and its
  UART0 pad works. The drone's UART0 TX pad is still destroyed, so
  everything mabur-specific — `load_sigmastar`, the 2.64 s radio window,
  `maburd` itself — is still only reachable over the network or by
  repairing the pad. `console=ttyS2,115200` on the FC pads (needs
  `msp.enabled = false`) remains the cheapest way to get the drone's own
  kernel log, and `printk.time=1` is now confirmed to work there.
- **The no-cable auto-negotiation number is n = 1.** See above.

The confirmed tty mapping is unchanged, from `infinity6e.dtsi:72`
(`console = &uart0`) and `bundle/mabur.default.toml`:

| tty | node | address | use |
|---|---|---|---|
| ttyS0 | `uart0` | 0x1F221000 | console — U-Boot + kernel + getty |
| ttyS1 | `uart1` | 0x1F221200 | unused (`PAD_GPIO0`) |
| ttyS2 | `fuart` | 0x1F220400 | FC / MSP (`msp.serial`) |
| ttyS3 | `pm_uart` | 0x1F006A00 | unused |

## Reproducing

**Serial rig (current).** Two committed scripts, both needing `pyserial`
(NixOS: `nix-shell -p python3Packages.pyserial --run ...`):

- `tools/bench/bootcap.py <seconds> <out.log> ["<trigger cmd>"]` — captures
  the console with a per-line host timestamp taken from the arrival of the
  line's **first** byte, which is what makes stage boundaries honest at
  115200. The optional trigger (e.g. `ssh root@… reboot`) fires just after
  the port opens.
- `tools/bench/ubcmd.py <out.log> <bootm-wait-s> "<cmd>" …` — resets the
  board (`reset` at the prompt if it is already there, otherwise ssh
  `reboot`), interrupts autoboot, runs commands at the `OpenIPC #` prompt,
  and timestamps the result the same way. Use it for non-persistent A/Bs:
  `setenv bootcmd …` then `run bootcmd`.

Analysis is `grep`/`awk` over the resulting logs; the marker strings used
for the tables above are in the "Pre-kernel" section.

**Network rig (used for the baseline and the `quiet` A/B).** Lived in a
session scratchpad and is not committed; it is four short pieces, and the
fiddly parts are the two gotchas under "Rig" above.

- Host ping loop at 20 Hz to `192.168.10.152`, stamping UP/DOWN, for the
  net-drop mark.
- GS: `sys.path.insert(0, "/root")`, `import ausniff`, `open_ring
  ("/dev/shm/mabur-au")`, poll `struct.unpack_from("<Q", mm, 16)` (`wseq`) at
  200 Hz, print stall and resume with `time.time()`.
- Drone, piped over ssh stdin: loop on `/proc/uptime`, tail
  `/tmp/mabur.log`, poll `curl 127.0.0.1:8301/venc` for the frame counter,
  and on exit dump field 22 of every `/proc/<pid>/stat` sorted ascending.
- Analysis: first encoded frame is recovered as
  `poll_start_uptime − frames/60`, since the counter is already non-zero by
  the time ssh answers.

The serial scripts superseded the plan to promote this to
`tools/bench/boottime.sh`: they measure the same stages without needing the
drone to be on the network, and without a GS.
