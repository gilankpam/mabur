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
  — power-on, 8812eu firmware download, TX enable. Its share was
  unmeasurable from this rig; it is now measured at **1.73 s** on the
  stamped build — see "maburd's own startup, stamped" below.
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

### Flashing — the runbook

Every step below was run in this order on `.95`. It is written to be
device-agnostic: set `HOST` and go. **Do not skip the readback in step 5** —
it is the last point at which a bad write is still recoverable.

Before starting: a **serial console must be attached**. If the new U-Boot
does not boot there is no recovery short of an SPI programmer, and without a
console you will not even see how it failed.

```sh
HOST=root@192.168.10.95           # the drone is root@192.168.10.152
SOC=ssc338q
IMG=$(ls -t ~/Projects/drone/openipc-builder/archive/*/*/u-boot-${SOC}-nor-padded.bin | head -1)
```

**0 — Build the image.** A normal device build produces it; see "Source and
build" above.

```sh
cd ~/Projects/drone/openipc-builder
printf './builder.sh ssc338q_fpv_openipc-urllc-aio\n' | nix-shell
```

`nix-shell --run` does **not** work here: the `buildFHSEnv` shell's
`runScript = "bash"` overrides it, so `--run` starts bash, finds no tty and
exits 0 having built nothing. Pipe the command in instead.

**1 — Back up mtd0 and the environment.** The mtd0 dump is a byte-exact
rollback, but only as long as the new U-Boot still boots Linux.

```sh
ssh $HOST 'dd if=/dev/mtd0 bs=64k 2>/dev/null' > mtd0-backup-$(date +%Y%m%d%H%M).bin
ssh $HOST 'fw_printenv' > env-backup-$(date +%Y%m%d%H%M).txt
ssh $HOST 'dd if=/dev/mtd0 bs=64k 2>/dev/null | md5sum'
```

**2 — Transfer.**

```sh
scp -O "$IMG" $HOST:/tmp/
```

**3 — Verify the transfer.** Must equal `md5sum "$IMG"` on the host.

```sh
ssh $HOST "md5sum /tmp/$(basename $IMG)"
```

**4 — Flash.** The irreversible step. Keep it a command of its own.

```sh
ssh $HOST "flashcp /tmp/$(basename $IMG) /dev/mtd0"
```

**5 — Verify the readback BEFORE rebooting.** Must equal the step-3 md5.
**If it does not match, re-flash — do not reboot.**

```sh
ssh $HOST 'dd if=/dev/mtd0 bs=64k 2>/dev/null | md5sum'
```

**6 — Reboot with the console watching**, so a failure is visible rather
than just an unreachable board.

```sh
cd ~/Projects/drone/mabur
nix-shell -p python3Packages.pyserial --run \
  "python3 tools/bench/bootcap.py 35 boot-new-uboot.log 'ssh $HOST reboot'"
```

**7 — Confirm and measure.** Expect the new banner, then `Starting kernel`,
then a login prompt; `IPL` → `Starting kernel` should be ~0.9 s.

```sh
grep -E 'Version:|Starting kernel|Mounted root' boot-new-uboot.log
ssh $HOST 'uptime'
```

**8 — Set the environment.** **Normally nothing to do:** the image ships
these in `/usr/share/openipc/customizer.sh`, which `S30customizer` runs once
per overlay wipe. They take effect on the *next* boot, so the first boot
after a flash is still the slow one. By hand, if the overlay was not wiped
or you are on an older rootfs:

```sh
ssh $HOST "fw_setenv bootdelay 0; fw_setenv verify no; fw_setenv baseaddr 0x20007FC0"
ssh $HOST "fw_setenv bootargs 'console=ttyS0,115200 quiet loglevel=1 panic=20 \
  root=/dev/mtdblock3 init=/init \
  mtdparts=NOR_FLASH:256k(boot),64k(env),2048k(kernel),\${rootmtd}(rootfs),-(rootfs_data) \
  LX_MEM=\${memlx} mma_heap=mma_heap_name0,miu=0,sz=\${memsz}'"
```

All four are safe on the stock U-Boot as well as the rebuilt one, so the
rootfs does not depend on the bootloader having been flashed first. Baking
them into the U-Boot compiled-in defaults instead would **not** help an
existing device — a stored environment overrides the compiled ones, so that
only reaches a board whose env is blank or erased.

`bootdelay=0` matters **only on the rebuilt U-Boot**, where a non-zero
bootdelay costs 1.195 s; on the stock one it was free. It does not cost the
recovery window: a CR spammed during boot still reaches the `OpenIPC #`
prompt at `bootdelay=0`, which is what `tools/bench/ubcmd.py` relies on.

#### Rollback

Name the backup explicitly — a glob would match every backup you have ever
taken, and `flashcp` would take the wrong one.

```sh
BAK=mtd0-backup-202609080434.bin          # the one from step 1
scp -O "$BAK" $HOST:/tmp/
ssh $HOST "flashcp /tmp/$BAK /dev/mtd0"
```

Then re-verify with step 5. If the board does **not** boot Linux, this path
is gone with it — that is the whole reason for the serial console and for
doing this on the spare board first.

#### Two things that do not work

- **`sysupgrade` cannot flash U-Boot.** Its only options are `--kernel` and
  `--rootfs` (plus `-k`/`-r`/`--url`/`--archive`, which feed the same two),
  and internally it only ever `flashcp`s the kernel and rootfs partitions
  and erases `rootfs_data`. Its single mention of `autoupdate-uboot.img` is
  a `check_sdcard()` interlock that *aborts* the upgrade if it finds that
  file on a mounted SD card. It is still the right tool for the kernel and
  rootfs the same build produces:

  ```sh
  scp -O $A/uImage.ssc338q $A/rootfs.squashfs.ssc338q $HOST:/tmp/
  ssh $HOST 'sysupgrade --kernel=/tmp/uImage.ssc338q \
                        --rootfs=/tmp/rootfs.squashfs.ssc338q -n'
  ```

  **Never pass `-x` (no reboot).** `sysupgrade` overwrites the squashfs the
  running system is executing from, so between the write and the reboot the
  machine is incoherent — the console fills with respawning getty and the
  `sysupgrade` script itself wedges, because its own remaining pages are
  read back from flash that has changed under it. It reboots unconditionally
  by default for exactly this reason. Learned the hard way on 2026-09-08:
  with `-x` the board hung for ~1 minute and then the hardware watchdog
  reset it. No damage — it came up on the new image and both partitions
  verified — but the reboot is not optional, and `-n` (wipe overlay) is what
  you want going from a stock image to a mabur one.
- **The `ubnor` env command**, which is `sf erase 0x0 0x50000` — that spans
  mtd0 *and* the environment at 0x40000, so it takes `ethaddr` and every
  `fw_setenv` tweak with it. `flashcp` to `/dev/mtd0` touches only mtd0.

Every flash recorded here was verified by reading `/dev/mtd0` back and
comparing md5 against the file written.

**The builder-produced image is the one now running on `.95`** (2026-09-08):
a full `builder.sh` run, then this runbook. It reports
`Version: I6E#f8a00c4#`, boots to userspace and ssh, and measures pre-kernel
0.856 s against the hand-built image's 0.904 s — the 0.05 s is run variance
in the `bootm` block, not a difference between the two builds. So the whole
path from `builder.sh` to a booting board is verified, not just the
hand-assembled one.

### Whole-stack boot on the new image

`.95` was moved onto the build's own kernel and rootfs with `sysupgrade` on
2026-09-08 (both partitions md5-verified against the files after flashing),
which makes it the first board on the serial rig running a **mabur**
userspace rather than stock majestic/wifibroadcast. From the first IPL byte:

| marker | s from IPL | Δ |
|---|---|---|
| `Starting kernel` | 0.903 | 0.903 |
| console registered | 2.229 | 1.326 |
| rootfs mounted | 3.344 | 1.115 |
| `Starting syslogd` (first rcS) | 4.259 | 0.915 |
| `Starting network` | 5.060 | 0.801 |
| `Starting dropbear` | 10.168 | **5.108** |
| sensor assigned (`S70vendor`) | 11.668 | 1.500 |
| login prompt | 11.871 | 0.203 |

Two things to read carefully before optimising against this:

- **The 5.1 s at `Starting network` is a `.95` artifact, not a drone cost.**
  Its `eth0` is `inet dhcp` and it waits for a lease. The drone is static
  (`192.168.10.152`), which is why the earlier drone baseline shows only
  0.81 s for the whole `syslogd → dropbear` window. Do not chase it.
- **`maburd` starts and stays up** (no respawn over 72 s) but sits at
  `waiting for encoder data... fill=0%`. That is this board's dead camera,
  not a software fault — `.95` has no working sensor. So everything from the
  MI bring-up onward still has to be measured on the drone.

## The drone, flashed — 2026-09-08

`.95` had proved the pieces; this is all of them on the drone at
`192.168.10.152`, in one session. **Net-drop → first AU in the GS ring:
13.23/13.33 s → 8.66/8.52 s, a 4.7 s cut (−35 %).** The three changes were
applied and measured one at a time, so the split below is measured, not
apportioned:

| step | net-drop → first AU | Δ |
|---|---|---|
| baseline (this morning's build) | 13.23 / 13.33 | — |
| + rebuilt U-Boot on `/dev/mtd0` | 12.15 | **−1.1** |
| + `verify=no`, `baseaddr` XIP, `quiet loglevel=1` | 10.68 | **−1.47** |
| + new kernel & rootfs (`sysupgrade -n`) | 8.66 / 8.52 | **−2.1** |

Note the baseline is 13.2 s, not the 15.0/15.1 s at the top of this
document: that table predates the `InitWrite` overlap, which was deployed on
2026-09-08 before any of this. Same rig, same day, so the deltas are clean.

Where the last −2.1 s comes from, on the drone's own clock (field 22 of
`/proc/<pid>/stat`, ticks since boot at `CONFIG_HZ=100`):

| stage, s since kernel t=0 | before | after |
|---|---|---|
| first rcS entry (`syslogd`) | 2.87 | **0.99** |
| MI stack up (`SensorIfThreadW`) | 4.22 | **1.75** |
| `maburd` start | 5.15 | **2.18** |
| venc threads | 6.95 | **3.52** |

That is squashfs LZO plus the rcS reorder plus the trimmed module list,
which are not separable from each other in this run — but the shape is
unmistakable: **`maburd` now starts at 2.18 s instead of 5.15 s**, because
`S39mabur` runs ahead of `S40network`, `S49ntpd`, `S50dropbear` and
`S60crond` instead of behind all four.

The clearest sign the reorder is doing what it was meant to: **ping now comes
back at 10.6 s, two seconds *after* video is already flowing at 8.5 s.** The
network is no longer on the critical path to a picture. It also means the
network rig's "ping back" column stops being a useful proxy for anything —
use the AU-ring resume.

### What went onto the board

- **U-Boot**: `u-boot-ssc338q-nor-padded.bin` from the builder, `flashcp` to
  `/dev/mtd0`, md5 readback verified against the file before rebooting
  (`779387cb…`). The runbook above, followed step for step. `mtd0` and
  `fw_printenv` were backed up first and the backup verified byte-exact
  against a device readback.
- **Kernel + rootfs**: `sysupgrade --kernel= --rootfs= -n`. The `-n` matters
  here — the drone's overlay held 3.7 MB of hand-deployed history (four
  `maburd` binaries, `S96mabur`, `waybeam`, nine `mabur.json` backups), all
  of which would have **shadowed** the image's own copies through the
  overlayfs. After the wipe the overlay is 272 KB and every file in the
  video path comes from the image.
- **Config**: `/etc/mabur.toml` now comes from the image. Since mabur
  `5f3077a` the bundle default is a verbatim copy of the drone's flight
  config rather than a neutral seed, so this was a no-op on behaviour —
  confirmed live afterwards by `curl 127.0.0.1:8301/venc` reporting
  `req_bitrate_kbps: 16000`, the flight ceiling, where the old seed capped
  at 10000.

### Two things to know before repeating this

- **`baseaddr=0x20007FC0` is only correct while the uImage loads at
  `0x20008000`**, and it is unrecoverable on a board with no console if it
  is wrong. Check it, do not assume it: the load address is bytes 16-19 of
  the uImage header, big-endian, and `baseaddr` is that minus the 0x40-byte
  header. Both the image being flashed *and* the kernel already on the board
  were checked here (`dd if=/dev/mtd2 bs=64 count=1 | od -An -tx1`), because
  the env change takes effect one boot before the new kernel does.
- **`sysupgrade -n` takes ssh with it.** The image ships root with no
  password and no `authorized_keys`, so `/usr/sbin/openipc-claim` — which is
  root's login shell until `/etc/shadow` has a hash — refuses every
  non-interactive `ssh host 'cmd'` with "This camera has not been set up
  yet". One interactive login to set a password releases it (the script
  rewrites root's shell back to `/bin/sh` itself). Budget for that step, or
  the board is up and encoding while being unreachable to every script you
  own.

## Ranked next steps

Re-sized against a **flight-configuration** boot (no Ethernet), which is
~2.7 s longer than anything measured before the serial console existed.
Items 1-3 are measured; the rest are estimates.

1. ~~**Kill U-Boot's Ethernet auto-negotiation**~~ — **DONE and measured**,
   see the section above. Pre-kernel 2.907 → 0.904 s with a cable, 4.911 →
   0.854 s without. **Live on the drone since 2026-09-08** — worth 1.1 s of
   the 4.7 s measured there; see "The drone, flashed".
2. **`verify=no` + `baseaddr=0x20007FC0`** — **0.55 s**, measured, env-only,
   reversible. **Live on the drone since 2026-09-08.** Check the uImage load
   address before setting `baseaddr` on any other board.
3. ~~**`quiet loglevel=1`**~~ — **DONE and re-measured: 0.84 s**, see
   below. Config-only (`bootargs`). **Live on the drone since 2026-09-08**;
   with item 2 it accounts for the 1.47 s env step there.
4. ~~**Overlap devourer `InitWrite` with the MI bring-up** in `maburd`~~ —
   **DONE 2026-09-08 and deployed on the drone: −0.87 s cold boot
   (venc fully hidden; TX gate 2.92 s after start), −1.73 s warm
   restart (n=5)**; see
   "InitWrite overlapped" below. The sizing that follows is what it was
   built against — **now measured, not estimated**: `InitWrite` is 1.73 s and the USB
   claim + port reset ahead of it another 0.80 s, both strictly serial
   with a venc bring-up of ~1.5 s (cold) — see "maburd's own startup,
   stamped" below. Today the encoder spends the gap producing frames into
   the void (`drops=309`, `sent=0`). Code-only, no deploy-order hazard.
5. ~~**A mabur-specific device profile and a custom rcS**~~ — **DONE**,
   and **superseded on 2026-09-09** by "rcS, stamped" below: maburd is
   now the first rcS entry and loads the MI stack itself; the
   `S38vendor < S39mabur < S40network` order described here no longer
   exists.
   both halves, see below. `load_sigmastar` returned −0.07 s (far less than
   estimated: squashfs LZO had already taken most of that window). The rcS
   reorder moves `maburd`'s start from 9.6 s to 3.6 s on `.95`, most of
   which is DHCP relocation, so expect nearer 1–1.5 s on the static-IP
   drone — plus determinism, which does transfer. It turned out to be
   builder-only after all: `mabur.mk` chooses the installed name, so the
   mabur repo is untouched.

   Two warnings that used to sit here are **no longer true**, verified
   against a full build on 2026-09-08: the `ssc338q_fpv_openipc-urllc-aio`
   profile no longer sets `BR2_PACKAGE_WAYBEAM_VENC` at all (the built
   rootfs contains no waybeam and no S95 entry), and `package/mabur/mabur.mk`
   no longer pins a stale SHA — it resolves `gilankpam/mabur` master at build
   time via `git ls-remote`. A fresh image ships current `maburd`,
   `S96mabur` and `/etc/mabur.toml`, with majestic explicitly off.
6. **Fix the emac PHY node in the device tree** — 0.185 s, measured. The
   MDIO bus scans all 32 addresses because `ethernet-phy@0` has an
   "invalid PHY address", then finds the PHY at 0.
7. **LZO instead of XZ.** Split in two, because the halves behave nothing
   alike:
   - **squashfs — DONE and measured, 0.58 s.** See "squashfs LZO" below.
     `BR2_TARGET_ROOTFS_SQUASHFS4_LZO=y`, live on `.95` and, since
     2026-09-08, on the drone.
   - **kernel — do this last, not first.** The doc used to say this would be
     "funded by dropping majestic, waybeam, vtund, curl, mbedtls, …" from the
     profile. That funding no longer exists: the profile is already minimal
     and curl/mbedtls are kept deliberately for `sysupgrade`. And those are
     rootfs packages, while the binding constraint is the **kernel
     partition**: `Image` is 4202496, the XZ `zImage` 2038312, and the
     partition is 2 MiB — **58 KB spare**. LZO would take the zImage to
     roughly 2.9 MB and LZ4 to ~3.1 MB; neither fits. It needs a slimmer
     kernel config first, or a repartition (`mtdparts` plus `kernaddr` /
     `kernsize` / `rootaddr`), which is a flag day. It also partly
     self-defeats: +1 MB of NOR read at the measured 5.2 MB/s is ~+0.2 s
     back. The upside is real though — `CONFIG_KERNEL_XZ=y`, and U-Boot
     reports "uncompressed" only because it jumps straight to the
     self-decompressing zImage, so 4 MB of XZ decompression happens inside
     the silent 1.07 s window before the console comes up. **Trimming
     `infinity6e-ssc012b.config` is the pure win** — a smaller `Image`
     decompresses faster *and* shortens the `sf read`, with no partition
     change.
8. **Shrink or relocate the jffs2 overlay** — up to ~1.5 s, but it holds the
   config and the imx415 ISP tuning bin, so those need a home first.
9. **Skip the USB port reset on a cold boot** — 0.85 s, measured;
   **PARKED 2026-09-08**, see "The USB port reset — parked" below for
   the mechanism, the change, and the soak it needs before it is believed.
10. **The GS player's ~1.6 s from first AU to first picture** — measured,
    unattributed, GS side; not touched by anything in this document.

~~Still worth doing before 4-8: **timestamp `maburd`'s own log lines**~~ —
**DONE 2026-09-08**, `drone/src/boot_trace.h`; the section below is what
it found.

Realistic floor with this SoC, the vendor ISP blobs and a USB dongle: 5-6 s
from power to video. Items 1-3 alone are 5.2 s of measured, mostly cheap
savings against that. Inside `maburd` — the only part deployed on the
drone as of 2026-09-08 — item 4 plus the core-1 pin and the link-up IDR
took `maburd` start → TX gate from ~3.8 s to ~2.4 s, measured and
deployed (the link-up IDR request was tried and reverted). Of the 2.5 s to LINKED, 2.4 is the radio: 0.85
USB port reset (item 9, parked) + 1.53 `InitWrite` (devourer).

### squashfs LZO — measured

`BR2_TARGET_ROOTFS_SQUASHFS4_XZ` → `BR2_TARGET_ROOTFS_SQUASHFS4_LZO` in the
device defconfig, built and flashed to `.95` on 2026-09-08. squashfs
decompresses lazily on every file read, so the compressor is paid right
through userspace startup rather than once.

| marker (s from IPL) | XZ | LZO #1 | LZO #2 |
|---|---|---|---|
| `Starting kernel` | 0.903 | 0.855 | 0.904 |
| console registered | 2.229 | 2.218 | 2.230 |
| `Mounted root` | 3.344 | 3.310 | 3.345 |
| `Starting syslogd` | 4.259 | **3.858** | **3.858** |
| `Starting network` | 5.060 | **4.462** | **4.462** |
| `Starting dropbear` | 10.168 | 10.626 | 10.224 |
| login prompt | 11.871 | 11.527 | 11.125 |

**Quote the DHCP-free window**, `Mounted root` → `Starting network`:
**1.716 → 1.152 / 1.117 s, a 0.58 s saving**, with the two LZO runs agreeing
to 35 ms. Everything after `Starting network` is contaminated by `.95`'s
DHCP wait, which varies 5.1–6.2 s run to run and swamps the totals; the
drone is static and never pays it, so there the 0.58 s shows through
directly. Pre-kernel and the mount itself are unchanged, which is the
expected shape — mounting reads only the superblock.

Cost: 4.26 MB → 5.42 MB of flash. Two things that cost an hour to work out:

- **U-Boot sizes the rootfs partition from the image.** `common/cmd_sf.c`
  reads the squashfs superblock at `0x250000` on every `sf probe`:
  `bytes + 0x1000 < 0x500000` → `rootmtd=5120k`, else `8192k`. The 4.26 MB XZ
  image had already shrunk mtd3 to **5 MB**, so "the partition is 8 MiB,
  plenty of headroom" was wrong. A fresh flash sizes itself correctly; an
  incremental `sysupgrade` cannot bootstrap it, because the partition is
  still 5 MB at the moment `flashcp` runs. Break the cycle by pinning
  `mtdparts` to `8192k` literally for one boot, flashing, then restoring
  `bootargs` to the `${rootmtd}` form — U-Boot then derives `8192k` itself.
- **`sysupgrade` reports success over a failed flash.** It printed
  `RootFS updated to …` immediately after `flashcp: /tmp/rootfs-lzo.squashfs
  bigger than /dev/mtd3`, because it never checks flashcp's exit status.
  Only the superblock told the truth (compression byte `04` = XZ vs `03` =
  LZO). **Verify a rootfs flash by reading the partition back**, exactly as
  the U-Boot runbook does — do not trust the tool's own summary. Related:
  `sysupgrade` also skips silently when the version string matches
  (`Same version, nothing to update`), and changing only the compressor does
  not change the version, so iterating needs `-f`.

LZ4 is not available without a kernel change: `CONFIG_SQUASHFS_LZO=y` is set
but `CONFIG_SQUASHFS_LZ4` is not, and the failure mode is an unmountable
rootfs.

### load_sigmastar — measured, and a boot-failure mode found

Builder-side, 2026-09-08. `load_sigmastar` loaded all 14 vendor modules and
probed for the sensor over I2C. Both were cut back; the result is **−0.07 s**
and, more usefully, one fewer way for the drone to boot without video.

| | 14 modules, `ipcinfo` probe | 9 modules, env-pinned |
|---|---|---|
| `insert_ko` | 0.625 | **0.544** |
| `detect_sensor` | 0.074 | 0.087 |
| `Loading vendor modules` → login | 0.852 | **0.784** |
| sensor detected | 4/4 | **8/8** |

Dropped — `mi_ai`, `mi_ao`, `mi_divp`, `mi_shadow`, `mi_mipitx`, 466 KB.
`maburd` dlopens only `libcam_os_wrapper`, `libmi_ipu`, `libmi_isp`,
`libmi_sensor`, `libmi_sys`, `libmi_venc`, `libmi_vif`, `libmi_vpe`; ISP and
IPU are served through `mhal`/`mi_sys` and have no `.ko`.

**`mi_ldc` and `mi_rgn` must stay** even though `maburd` never calls them:
`mi_vpe` links against both, 27 and 2 symbols. Removing them gives
`insmod: can't insert mi_vpe.ko: unknown symbol in module`, then
`maburd exited (139)` respawning every 2 s. Check a candidate before
dropping it:

```sh
nm -u <kept>.ko | awk '{print $2}' | sort -u > keep.txt
nm -g --defined-only <candidate>.ko | awk '{print $3}' | sort -u \
    | comm -12 - keep.txt        # must be empty
```

**The interesting part: the trim exposed a latent race.** With ~0.1 s less
`insmod` ahead of it, `ipcinfo -s` — which reads the sensor ID over I2C —
returned **empty on 5 of 8 boots**. An empty `SENSOR` makes the script print
`Sensor parameter MISSING` and exit before `set_sensor`, so no sensor driver
is loaded at all and `maburd` exits(3) in a respawn loop. It never failed
once across 4 boots with the full module list, so this was latent, not new:
the extra module-loading time was hiding it.

`SENSOR` now comes from the U-Boot environment (already pinned to `imx415`),
with the probe kept as a fallback. The `sensor_config`/`srcfg` sequence is
deliberately untouched — it powers and clocks the sensor for the driver
loaded later, and removing it was never the goal. Eight consecutive boots
detect the sensor, with `maburd`'s MI bring-up identical to baseline
(sensor pad, star6e pipeline, VPE scaling, jpeg init) and zero respawns.

Two things worth carrying forward:

- **A faster boot is not automatically a working boot.** This is the second
  time today that removing time from the boot path changed behaviour rather
  than just timing — the first being `sysupgrade -x`. Any change here wants
  a repeat-boot loop, not a single sample.
- **The same race may exist on the drone.** It has never been observed
  there, but the drone has never been measured across repeated boots either,
  and its timing differs. The env pin removes the question rather than
  answering it.

### `quiet loglevel=1` — re-measured on the serial rig, 0.84 s

Applied to `bootargs` on `.95` on 2026-09-08 and measured over 8 boots per
arm, against the LZO + trimmed-module + env-pinned build.

| marker (s from IPL) | baseline (n=8) | quiet (n=8) | Δ |
|---|---|---|---|
| `Starting kernel` | 0.886 ±0.024 | 0.880 ±0.025 | −0.006 |
| `Starting syslogd` (first rcS) | 3.835 ±0.025 | **2.997 ±0.025** | **−0.839** |
| `Starting network` | 4.456 ±0.046 | 3.612 ±0.002 | −0.844 |
| `Loading vendor modules` | 10.509 ±0.430 | 9.584 ±0.673 | −0.925 |
| login prompt | 11.293 ±0.455 | 10.381 ±0.673 | −0.912 |

**0.84 s**, against the 0.68 s measured over the network rig earlier — the
same effect, larger here because this image logs more. `Starting kernel` is
unchanged, which is the sanity check: a kernel cmdline parameter cannot
affect U-Boot, and it doesn't. The saving is banked before rcS and carried
forward unchanged, exactly as the original A/B found. `syslogd` and
`network` are the numbers to quote (±0.03); everything past `network` is
inside `.95`'s DHCP wait and carries its ±0.5 s.

No regressions: 8/8 sensor detections, zero `maburd` respawns, MI bring-up
unchanged.

**What this does to the measurement rig.** `loglevel=1` silences kernel
printk on the console but *not* userspace writes to `/dev/console`, so with
quiet on:

- **Still visible** — every IPL and U-Boot line, `Starting kernel ...`, and
  all rcS output (`Starting syslogd`, `Starting network`, `Loading vendor
  modules`, `Sensor assigned`, the login prompt). The console log drops from
  ~305 lines to ~92.
- **Gone** — `Booting Linux on physical CPU`, `console [ttyS0] enabled`,
  `Mounted root (squashfs …)`, and the whole kernel-init window. Anything
  measuring those markers reads zero.

That last point is a trap: a script that greps for a kernel marker cannot
distinguish "quiet is on" from "the board failed". To measure the kernel
phase again, drop `quiet loglevel=1` from `bootargs` for the session.
`dmesg` is unaffected either way — still 702 lines with quiet on, so nothing
is lost post-boot.

### Custom rcS — the video path first

Builder-side, 2026-09-08. `busybox` rcS runs `/etc/init.d/S*` strictly
serially in lexical order, and the video path was at the end of it:
`S70vendor` (`load_sigmastar`) and `S96mabur` ran *after* `S40network`,
`S49ntpd`, `S50dropbear` and `S60crond`. New order:

```
S38mdev  <  S38vendor  <  S39mabur  <  S40network  <  S49ntpd  <  S50dropbear
```

`mdev` stays first because `maburd` needs the device nodes, and `S38vendor`
precedes `maburd` because it insmods the MI modules `maburd` dlopens
against. 8 boots each side:

| marker (s from IPL) | before | after |
|---|---|---|
| `Starting syslogd` | 2.997 | 2.998 |
| `Loading vendor modules` | 9.584 (8.63–9.98) | **3.556 (3.51–3.61)** |
| `Sensor assigned` | 10.216 (9.33–10.63) | **4.188 (4.16–4.22)** |
| login prompt | 10.381 | 9.594 |

8/8 sensor detections, no `maburd` respawns, and `eth0` still gets its lease
even though the network now starts after `maburd`.

**Read the 6 s carefully.** Most of it is `.95`'s DHCP wait being moved
*behind* video rather than in front of it. A static-IP drone never paid that,
so its gain is nearer the estimated 1–1.5 s — what used to run between
`mdev` and the MI bring-up is `S40network` (cheap when static), `S49ntpd`,
`S50dropbear` and `S60crond`. The markers before the change are unchanged,
which is the shape that says the measurement is honest.

The other half of the win does transfer, though: **`maburd`'s start time
becomes deterministic**, 3.51–3.61 s against 8.63–9.98 s before, because it
is no longer downstream of DHCP.

Three parts, all builder-side — note the second, which is not obvious:

- `S38vendor` shipped in the device overlay, a copy of `S70vendor`.
- `/etc/init.d/S70vendor` added to the excludes list. **An overlay can
  overwrite a file but not delete one**, and leaving both would run
  `load_sigmastar` twice; `general/scripts/rootfs_script.sh` `rm -f`s every
  path in `scripts/excludes/<soc>_<variant>.list` at image assembly, and
  reports entries that matched nothing.
- `package/mabur/mabur.mk` installs the init script as `S39mabur`. Only the
  destination name changes — the source is still `bundle/S96mabur` from the
  mabur repo, so this is builder-only despite appearances.

Two traps when rebuilding this incrementally rather than through
`builder.sh`:

- `builder.sh` copies **both** `devices/<dev>/*` and `package/*` into the
  firmware tree (`copy_extra_packages`). Copying only the device overlay
  builds against the old `mabur.mk` and the rename silently does nothing.
- Buildroot's **per-package directory** keeps its own target tree. Deleting
  the stale `output/target/etc/init.d/S96mabur` is not enough — it is
  re-populated from `output/per-package/mabur/target/`, and the image ends
  up with both `S39mabur` and `S96mabur`, starting `maburd` twice. A clean
  `builder.sh` run has neither problem.


## maburd's own startup, stamped — 2026-09-08

For this investigation `maburd` printed `[boot +S.mmm] …` lines
(`drone/src/boot_trace.h`, monotonic since the first statement of
`main()`) at every stage boundary of its own bring-up, including inside
the venc code, written to a private dup of stderr so the venc's
`sdk_quiet` redirects could not swallow them. They landed in
`/tmp/mabur.log`, which was the point: the drone has no console, but the
log survives and is readable over ssh after the fact.

**The stamps were removed again on 2026-09-08** once the measurements
below were taken (operator's call: no debug trace in the shipped code).
To re-measure, cherry-pick `7eaab01` (stage lines), `4e51c7d` (venc
internals + the `sdk_quiet`-proof writer) and the rendezvous stamps from
`299323f` onto a scratch branch, build, and run from `/tmp` as under
"Reproducing" below; nothing else in those commits is needed. The
affinity change (`d18938d`) and the `InitWrite` overlap (`1a78ccf`) are
code, not trace, and stay.

Measured on the drone (`192.168.10.152`) by running the stamped binary
from `/tmp` with the wrapper stopped — i.e. a **warm restart**, the thing
every deploy and every wedge recovery does. Five runs, all within ±40 ms:

| stage | cost (s) |
|---|---|
| config load, USB open | 0.004 |
| USB `claim_interface_then_reset` (port reset + re-enumeration) | **0.804** |
| `CreateRtlDevice` | 0.001 |
| **venc bring-up** (`venc_core_start`) | **10.64** |
| debug HTTP + spawning the four worker threads | 0.010 |
| **radio `InitWrite`** (power-on, firmware, TX enable) | **1.73** |
| TX power table + A-MPDU | 0.024 |
| **total → TX gate open** | **13.2** |

The first frame comes out of the venc ring 0.14 s after `InitWrite`
starts — the encoder is already producing while the radio is still coming
up, which is the overlap item 4 targets, seen from the other side.

### Where the 10.6 s goes, and why it is not a boot cost

Stamping down through `star6e_runtime_init` and `star6e_pipeline_start`
attributes 10.4 of the 10.64 s to **five calls at 2.04–2.08 s each**, and
every one of them is the *first call into a different MI module*:

| call | module | cost (s) |
|---|---|---|
| `MI_SYS_Init` | sys | 2.04 |
| `MI_VENC_StopRecvPic(0)` (pre-init teardown) | venc | 2.08 |
| `MI_VPE_GetChannelAttr(0)` (pre-init teardown) | vpe | 2.08 |
| `MI_VIF_DisableChnPort(0,0)` (pre-init teardown) | vif | 2.08 |
| `MI_SNR_QueryResCount(0)` (sensor select) | sensor | 2.08 |

The second call into the same module is free; everything else in the
pipeline — VIF/VPE/VENC create, bind, ISP bin load, `MI_SNR_Enable`
(0.17 s) — totals under 0.3 s.

Three things pin it to the vendor kernel driver rather than to anything
in `drone/venc`:

- With `printk.time` on, dmesg shows `client [pid] connected, module:sys`
  … `venc` … `vpe` … `vif` … `sensor` at exactly 2.08 s intervals, and
  `/proc/<pid>/wchan` through the whole window is **`MI_DEVICE_Open`** —
  the process is asleep inside `mi_sys.ko`'s device open, silently.
- A bare `exec 3<>/dev/mi_sys` from a shell, no `maburd` running, pays
  the same 2.04 s; `/dev/mi_venc` 2.08 s.
- The previous client disconnects cleanly (`client [pid] disconnected` for
  all five modules within 1 s of the kill) and `/proc/mi_modules/mi_sys/
  mi_sys0` shows nothing left over; a 1–2 minute gap before the restart
  does not help. It is not an expiring timer and not stale state we can
  scrub — it is what the driver does on the second-ever open of a module.

**On a cold boot it does not exist.** The kernel log buffer stores
timestamps whether or not `printk.time` displays them, so switching it
on after the fact reads back the boot: on the first `maburd` after power-up
all five `client connected` lines land between **6.812 and 6.832 s** —
20 ms for all of them. So the cold-boot venc bring-up really is ~1.5 s
(6.81 → ~7.3 s on that boot: MI connects, sensor query at 6.84/7.02), the
old 1.45 s reconstruction stands, and the 10.6 s is a **restart-only
penalty**: `S96mabur restart` costs ~10.4 s more than a boot does before
video returns. Nothing in mabur can shorten it; measure boot-path work on
a cold boot, or subtract 5 × 2.08 s from a warm one.

The cold-boot kernel log also dates the rest of `maburd`'s start on that
boot: process start 5.20 s, USB port reset 6.11 s, MI connects 6.81 s —
consistent with the warm-restart table above minus the penalty.

### `InitWrite` overlapped — measured

`drone/src/main.cpp`: `InitWrite` now runs on its own thread, spawned the
moment `CreateRtlDevice` returns, and is joined where the call used to be
— just before the TX-power and A-MPDU writes and the `device_ready`
store, which keep their order on the main thread. An `InitWrite` throw is
captured and rethrown after the join, so a USB/firmware failure still
escapes `main()` for the wrapper exactly as before; the `venc_core_start`
failure path joins before releasing the USB handle.

Five warm restarts of the overlap build against the five before it:

| marker | serial (s) | overlapped (s) |
|---|---|---|
| `InitWrite` start → done | 11.45 → 13.18 (1.73) | 0.82 → 2.14 (**1.33**, hidden inside venc) |
| venc bring-up done | 11.44 | 11.45 |
| **TX gate open** | **13.20** | **11.47** (−1.73, ±0.03 over n=5) |
| boot-window `drops=` in the first stats line | 231–256 | **0** |

`InitWrite` itself got faster (1.73 → 1.33 s) because it now runs while
the encoder thread is asleep in the kernel rather than alongside a live
hot/agent/tx thread set. The drops row is the side effect worth having:
the TX gate opens 20 ms after venc completes, before the first frame
leaves the ring (+0.12 s), so nothing is encoded into the void any more.
5/5 runs `state=2`, sending, no faults, no respawns.

**Cold boot, measured** (binary installed as `/usr/bin/maburd` by
rotation — rollback `maburd.pre-bootoverlap` — and the drone rebooted;
first process after power-up, `S96mabur` starts it at 5.26 s of uptime):

| marker | s after `maburd` start |
|---|---|
| USB claim + port reset done | 0.844 |
| venc bring-up start / radio `InitWrite` start | 0.862 / 0.864 |
| MI libraries dlopened | 1.303 (0.42 s — the biggest venc item cold) |
| `MI_SYS_Init` … all five MI client connects | 1.306 → 1.336 (**30 ms**) |
| `MI_SNR_Enable` done | 1.524 |
| **venc bring-up done** | **1.748** (0.89 s) |
| first encoded frame out of the ring | 1.881 |
| **radio `InitWrite` done** | **2.906** (2.04 s) |
| **TX gate open** | **2.921** → 8.18 s of uptime |

The venc bring-up is 0.89 s cold and is hidden in full: it ends at +1.75
while `InitWrite` runs to +2.91, so the radio is the entire critical path
(0.81 + 2.04 of the 2.92 s). Summing the same stamps serially gives
3.79 s — **−0.87 s on a cold boot**, i.e. the whole venc window, as the
`min(venc, InitWrite)` model predicts. `InitWrite` is 2.04 s cold against
1.33 s warm: warm it runs while the encoder thread sleeps in
`MI_DEVICE_Open`; cold it shares the two cores with a real bring-up.
`state=2`, sending, zero respawns. For scale against the network-rig
baseline table above: `S96mabur` → RX loop was 4.09 s there, on the
serial build.

One clock caveat when lining the stamps up with dmesg on the same boot:
the kernel log stamps (`client connected` at 7.05 s) sit ~0.5 s later than
`/proc/<pid>/stat` start tick + `[boot +…]` (6.57 s) predicts, because
printk stamps run on `sched_clock`, which on this SoC starts late (the
"Kernel init" section above measured ~0.76 s before it exists). Use
process-relative stamps plus the start tick for absolutes; use dmesg only
for ordering and for deltas within dmesg.

### IDR on DISC-driven LINKED entry — tried, glitched, reverted

The GS player cannot start on a P-frame (parameter sets ride in-band on
IDRs only), and the first LINKED after boot is DISC-driven, a path that
never calls `request_idr()` — only the RCF-driven re-entry does
(`rc_agent.cpp:449`). 3 of 5 measured resumes started on an IDR anyway,
by accident: the forced bitrate write at LINKED entry drags one out of
`SetChnAttr` when the rate changes. The other 2 waited 1.0 s for the GOP.

Tried on 2026-09-08 (commit `6c85d3e`): the DISC path requested an IDR
through the same `idr_due` pacer as the RCF path. On the GS the first AU
after each of three resumes was an IDR within 0.11 s — **and the pilot
saw a visible glitch on the first link-up.** Reverted the same day, code
and drone. Not attributed: the likely shape is two IDRs within a few
frames (the explicit one and the `SetChnAttr` side-effect one, which the
pacer's 100 ms floor does not always separate), or the explicit IDR
landing at the MAX_RANGE bitrate before the rung's rate applies. Whoever
retries this should watch the first second of video on the GS, not the
`au.log`, and consider ordering the request after `run_bitrate_policy`.


`claim_interface_then_reset(…, do_reset=true)` costs 0.85 s on the
drone's cold path, and after the core-1 pin it is 35 % of `maburd` →
LINKED. Two questions were asked of it and both were answered on
2026-09-08; the change itself is **parked**, not built.

**Overlapping it with the venc bring-up does not pay.** The radio path is
0.85 + 1.53 = 2.4 s and the venc bring-up 0.7 s, so the radio is already
the critical path: moving the reset onto the thread gives `max(2.4,
venc)` where today is `0.85 + max(venc, 1.53)` — the same number — and
would move every USB open/claim/create failure to after the encoder has
started, a new failure shape for a `return 1`.

**Skipping it on a cold boot would.** Read from devourer's
`UsbOpen.cpp`: the function takes an advisory lock, detaches the kernel
driver, sets configuration 1 and claims interface 0 — microseconds —
then calls `libusb_reset_device()` and re-claims. There are no sleeps in
devourer; the 0.8 s is the reset itself: the kernel drops the port, the
dongle re-enumerates (the `Plug in USB Port1` / `reset high-speed USB
device` pair in dmesg at every `maburd` start), descriptors are re-read
and libusb re-attaches. Devourer's own comment calls it "re-runs the
chip's own boot". It exists so a *restarted* `maburd` never inherits a
chip the previous process left half-configured — the deaf-radio-after-
restart history from July — and devourer already skips it for the
Kestrel (11ax) family, where the reset *causes* a bad state, so "always
reset" is a per-chip judgment, not a law. On a battery plug the kernel
enumerated the dongle ~4 s before `maburd` opens it and nothing has
touched it; the reset re-runs a chip boot that just happened, and
`InitWrite` does the full power-on sequence afterwards regardless.

**The change, when it is taken up:** reset only when this is not the
first `maburd` since boot — a marker in `/tmp` (tmpfs, gone on reboot)
written by every instance, and `do_reset = marker_existed`. Cold boot
skips it (−0.8 s), every restart keeps it. About ten lines in
`run_real_mode`.

**Why it is parked:** it is the one candidate that changes what the
radio path *does* rather than when, and the failure it guards against —
a radio that comes up deaf — is exactly the kind a single clean boot
cannot rule out. It needs ≥10 cold power-cycles with LINKED and video
confirmed on each (the boot stamps for LINKED, `tools/bench/ausniff.py`
on the GS for video), scripted `reboot`s being an acceptable stand-in.
Not one sample, the way the affinity pin was accepted.

### Where battery-to-picture actually goes — why −0.87 s is invisible on a stopwatch

Hand-timed battery-plug → first picture did not visibly change after the
overlap deployed. It shouldn't have: the drone got exactly one change
today, and it is 0.87 s of a ~15 s chain. The rest of the chain,
measured on the drone and the GS on 2026-09-08 (the drone still runs the
**stock** U-Boot; everything in items 1-3, 5 and 7 is only on `.95`):

| leg | s | measured how |
|---|---|---|
| U-Boot, no Ethernet (flight config) | ~5.5 | serial rig, stock U-Boot (`.95`); ~2.8 with a cable |
| kernel → `S96mabur` starts `maburd` | 5.13–5.26 | `/proc/<pid>/stat` start tick |
| `maburd` start → TX gate open | 2.43 (was 3.79 serial, 2.92–3.01 overlapped, before the core-1 pin) | boot stamps |
| TX gate → first packet from GS → LINKED | **0.003 → 0.08** | boot stamps, cold and warm |
| first AU on GS → first IDR AU | 0 (3 of 5 resumes) / 1.0 (2 of 5) | `au.log` `nal0`=32 |
| first AU on GS → first displayed frame | **~1.6** | `lat.log` first window |
| **battery → picture** | **~15 (no cable) / ~12.5 (cable)** — was ~16/15 before 2026-09-08 | sum; matches the stopwatch |

Three things this settles:

- **The rendezvous is instant**, cold and warm: first packet 3 ms after
  `StartRxLoop`, LINKED 80 ms after (stamps `first packet received`,
  `first RC frame`, `link established`, added for this). An earlier read
  of the drone's `stats:` line as "deaf for 2 s after the gate" was wrong
  — the hot loop spins at 200 Hz while the venc ring is empty, so
  `hot_beat=203` is ~1 s into the process, not 5.
- **The player, not the link, owns the last 1.6 s.** In the three most
  recent resumes the very first AU was already an IDR, and the first
  displayed frame still came 1.6 s later. Two earlier resumes additionally
  waited 1.0 s for the next GOP IDR because DISC-driven LINKED entry
  never requests one. An explicit request was tried the same day and
  **reverted — it glitched the first link-up** (see "IDR on DISC-driven
  LINKED entry" above). The player's 1.6 s is GS-side and untouched
  (ranked item 10).
- **The biggest leg is still U-Boot's auto-negotiation on the drone**,
  because the rebuilt U-Boot has only been flashed on `.95`. In the field
  there is no cable, so the drone pays the 4.0 s timeout on every battery
  plug. Flashing it needs a console for recovery, and the drone's UART0
  pad is destroyed — that is the real blocker on the number the pilot
  sees, and it has been since the first day of this document.

On the numbers a stopwatch can resolve, end of 2026-09-08: the drone's
share went from ~3.8 s to ~2.4 s (`maburd` start → TX gate), so battery →
picture is ~15 s without a cable against ~16 before; the 0–2 s of GOP
luck at link-up stays (the IDR request was reverted). 14 → ~9 s is what
items 1-3 would do on the drone once the console problem is solved; the
parked USB reset (item 9) and the player (item 10) are another ~2.4 s
after that.

### Two hazards found on the way

- **Never bare-open `/dev/mi_*` from a shell.** `/dev/mi_sys` and
  `/dev/mi_venc` opened and closed (2 s each); `exec 3<>/dev/mi_vpe` **took
  the drone down** — the shell hung and ~2 minutes later the board was back
  up from a fresh boot (`panic=20` in `bootargs`; whether it was the panic
  or the hardware watchdog is unknowable from the new dmesg). A non-MI
  client evidently trips the `/dev/mi_*` close deadlock waybeam documented
  (`../waybeam_venc/documentation/STAR6E_SINGLE_PID_REINIT_FINDINGS.md`).
  The timing question it was answering is settled above; do not repeat it.
- **`sdk_quiet` eats stderr.** The venc brackets every vendor call in
  `sdk_quiet_begin/end`, which `dup2()`s `/dev/null` over fds 1 and 2.
  Anything that logs by writing to fd 2 inside those windows vanishes —
  the first attribution run lost all ten stamps of the pre-init teardown
  to it. `bootlog` writes to its own dup of stderr for exactly this
  reason, and `tests/test_boot_trace.cpp` pins it.

### Reproducing

Stamped binary from `tools/build-arm.sh`, staged in tmpfs so the rootfs
(3 × `maburd` already, 2.5 MB free) is untouched:

```sh
scp -O out/arm/maburd root@192.168.10.152:/tmp/maburd.bt
ssh root@192.168.10.152 '/etc/init.d/S96mabur stop'         # own invocation
ssh root@192.168.10.152 'ps | grep "[m]aburd"'               # must be empty
ssh root@192.168.10.152 'setsid sh -c "/tmp/maburd.bt -c /etc/mabur.toml \
  > /tmp/bt.log 2>&1" </dev/null >/dev/null 2>&1 &'
sleep 20; ssh root@192.168.10.152 'grep "\[boot" /tmp/bt.log'
```

Kernel-side view of the same run: `echo Y >
/sys/module/printk/parameters/time; dmesg | grep "client \["`. Restore with
`setsid /etc/init.d/S96mabur start </dev/null >/dev/null 2>&1 &`.

## rcS, stamped — 2026-09-08/09: the video path goes first, maburd loads the MI stack

The custom-rcS work above was measured on `.95`; this is the same window on
the drone itself, with a rig that finally dates every rcS entry on one
clock and without a console: `/etc/init.d/rcS` writes
`rcS: <script>` to `/dev/kmsg` before each entry, which lands in the kernel
ring buffer with a kernel timestamp regardless of `quiet loglevel=1` (only
the console is silenced), and `dmesg` reads it back over ssh after the
boot. The end marker is the first AU in the GS ring (`wseq` at
`/dev/shm/mabur-au`, 200 Hz), converted to drone uptime through a
host→GS clock sandwich and `kernel t=0 = host_time − /proc/uptime` from the
first ssh. `/proc/uptime` and the dmesg clock agree to ~10 ms here, so the
stamps and the AU time are directly comparable. Two boots per arm; the
pairs agree to ±0.05 s except where noted.

**Result: first AU on the GS 5.40 → 4.02 / 4.09 s of uptime (−1.3 s,
−24 %) in the layout that shipped**,
from three changes plus a cold-boot cpufreq boost, all builder-side except
one small `maburd` change. Nothing on the wire moved.

### Where the 5.4 s went (baseline, seconds of uptime)

| marker | t | Δ |
|---|---|---|
| root mounted (squashfs) | 0.51 | kernel |
| `inittab: begin` | 1.09 | **0.58 — `/init`: the jffs2 overlay mount** |
| `rcS: begin` | 1.11 | 0.05 inittab |
| `S38vendor` (MI insmods) starts | 1.73 | **0.63 of generic init** ahead of it |
| `S39mabur` starts | 2.46 | 0.71 MI chain |
| maburd's USB port reset starts | 2.96 | 0.47 exec + page-in, cold |
| MI client connects (venc bring-up) | 3.65 | 0.69 USB reset |
| first AU in the GS ring | 5.40 | InitWrite ~2.0, venc hidden under it |

Of the 0.63 s of generic init: `S02fakehwclock` 0.32, `S30customizer` 0.09,
`S38mdev` 0.07, the two `fw_printenv` exports in rcS 0.04, `S02sysctl`
0.04, everything else ≤ 0.02 each. `fake-hwclock load` is the surprise: it
takes the newest mtime under `/etc` with `find -exec date -r {} \;`, one
`date` fork per file, 75 files. On a warm system that call alone measures
0.9 s.

### The arms, in the order they were tried

| arm | change on top of the previous | first AU (s) |
|---|---|---|
| baseline | image as flashed 2026-09-08 | 5.40 |
| A | rcS runs `S38vendor` + `S39mabur` before every generic script | 5.23 / 4.65 |
| A+C | + `fake-hwclock` uses one `stat` over `/etc` instead of a fork per file | 4.74 / 4.71 |
| B | + `S38vendor` backgrounds `load_sigmastar`, maburd waits for the modules | 4.58 / 4.91 |
| D | A+C + kernel uevent helper (`/sbin/mdev`) disabled during rcS | 4.80 / 4.84 |
| E | A+C + cpufreq 1.2 GHz during rcS (vendor driver pins 800 MHz) | 4.48 / 4.48 |
| **Y** | A+C, **maburd first, maburd runs `load_sigmastar` itself after its USB reset** | **4.39 / 4.37** |
| **Y+E** | Y + the boot-only 1.2 GHz | **4.23 / 4.25** |
| **final** | the shipped layout: stock rcS loop + stamps, `S00cpuboost`, `S00mabur`, `S99cpurestore` | **4.09 / 4.02** |

What each one taught:

- **A, the reorder, is worth 0.67 s at maburd's start** (2.46 → 1.79) but
  only 0.2–0.7 s at the AU, because the generic scripts now run *during*
  maburd's exec and stretch it from 0.47 to 0.6–1.0 s. They fight for the
  same NOR flash and two cores.
- **C recovers most of that**: with the fork storm gone the exec window is
  back to ~0.6 s and the AU time is reproducible.
- **B is a wash, and the reason matters.** Starting the insmod chain in
  parallel with maburd's exec gains nothing: both are NOR reads + LZO
  decompression + CPU, the chain slows from 0.67 to 1.44 s and the exec
  from 0.6 to 1.3 s. The only *idle* window on this path is the USB port
  reset (0.69 s of waiting for re-enumeration) followed by devourer's
  `InitWrite` (~2 s, USB-bound). Overlap has to land there or it is not
  overlap. (The readiness wait built for B stayed; see Y.)
- **D is a dead end**: the kernel forks `/sbin/mdev` for every uevent
  (`CONFIG_UEVENT_HELPER_PATH`), but only 48 uevents fall between rcS
  start and maburd's start. `mdev -s` itself is 0.07 s and devtmpfs makes
  every node maburd and the MI stack use (`/dev/mi_poll` is `mknod`ed by
  `load_sigmastar`), so `S38mdev` is not on the video path either way.
- **E: the SoC runs at 800 MHz because the vendor driver says so.**
  `drivers/sstar/cpufreq/infinity6e/cpufreq.c` `ms_cpufreq_init` sets
  `policy->min = policy->max = 800000` although the OPP table reaches
  1.2 GHz. Raising the cap for the rcS window is −0.25 s on top of A+C
  (the insmod chain 0.67 → 0.54 s, the exec 0.62 → 0.57 s); most of the
  boot is I/O- or USB-bound, which is why it is not more. `S00cpuboost`
  raises it, `S99cpurestore` puts the 800 MHz cap back at the end of rcS,
  so the in-flight envelope is untouched. Verified live first: a shell
  loop ran 2.3x faster at 1.2 GHz and the board stayed up.
- **Y is the one that pays.** maburd is the *first* rcS entry
  (`S00mabur`), there is no vendor init script at all, and maburd runs
  `load_sigmastar -i` itself (new config key `venc.module_loader`) right
  after `claim_interface_then_reset` and the `InitWrite` thread spawn,
  when `/sys/module` shows no live `mi_venc` + `sensor_*_mipi`
  (`drone/src/mi_ready.h`). Timeline on the drone: maburd starts at 1.21,
  exec 0.47 (uncontended again), USB reset 1.68 → 2.37, insmod chain
  2.4 → 3.06, MI client connects 3.13, venc up ~4.0, TX gate ~4.4, first
  AU 4.37. The chain (0.7) plus the venc bring-up (0.9) fit under
  `InitWrite` (2.0) with ~0.4 s to spare, and the radio is the entire
  critical path: `rcS begin 1.17 + exec 0.47 + reset 0.69 + InitWrite
  2.04 ≈ 4.4`. Warm restarts find the modules live and skip the loader
  (one sysfs scan). A loader failure is now *retried* by the wrapper's
  respawn, which `S38vendor` never did.

Side effect worth knowing: the first `stats:` line shows `drops=34–47`
again (frames encoded before the TX gate opened), where the InitWrite
overlap had brought it to 0. That is the venc finishing ~0.4 s before the
radio, not a fault; the frames cost nothing.

### What shipped

- mabur branch `boot-rcs`: `drone/src/mi_ready.h` (+ `tests/test_mi_ready`),
  `venc.module_loader` (`config.h`/`.cpp`, `bundle/mabur.default.toml`,
  default `/usr/bin/load_sigmastar -i`, `""` = only wait), the loader call
  and wait in `main.cpp` before `venc_core_start`.
- `openipc-builder` `feat/mabur`: `mabur.mk` installs the wrapper as
  `S00mabur`; device overlay drops `S38vendor`, adds `S00cpuboost`,
  `S99cpurestore`, a stamped `etc/init.d/rcS` (the `rcS:` kmsg lines now
  ship, so every boot leaves its timeline in `dmesg`) and the fixed
  `usr/sbin/fake-hwclock`; `S70vendor` stays in the excludes list.
- The drone runs exactly this layout through its overlay (rollback binary
  `/usr/bin/maburd.pre-miwait`); the next image flash brings the same
  from the squashfs.

**Deploy order for the config key**: binary before config — an old
`maburd` exits on the unknown `module_loader` key. The bundle default
equals the compiled default, so the config needs no edit at all.

### What is left on this path, sized

| item | s | note |
|---|---|---|
| U-Boot + kernel to root mount | ~1.35 | items 6-7 above |
| `/init`: jffs2 overlay mount | 0.58 | scan of a 5.8 MB partition at ~4.5 MB/s NOR; item 8. `CONFIG_JFFS2_SUMMARY` is off; empty blocks are already fast-pathed, so shrinking the partition is the lever |
| maburd exec + page-in | 0.47 | 1 MB binary + 1.4 MB libstdc++ from LZO squashfs |
| USB port reset | 0.69 | item 9, parked; the chain + venc (1.6 s) still fit under InitWrite alone, so skipping it would be the full 0.69 |
| devourer `InitWrite` | ~~2.04~~ **0.65** | 14k USB control transfers, now pipelined — see "InitWrite, attributed and pipelined" |

The rig: `tools/bench/cycle.sh`-style scripts lived in the session
scratchpad; the parts that matter are the `rcS:` stamps (now permanent),
the GS `wseq` poller from "Reproducing" below, and `dmesg` after the boot.

## InitWrite, attributed and pipelined — 2026-09-09

After "rcS, stamped" the radio's `InitWrite` was the largest leg left on
the cold-boot path: 2.04 s cold, 1.33 s warm. Two questions: what is it
made of, and what is the floor.

### What it is made of

devourer's `InitTimer` (`src/InitTimer.h`, already used by the Jaguar1 HAL)
now brackets every stage of the Jaguar3 `rtw_hal_init` and `InitWrite`
(`j3hal.*`, `j3init.*` events), and reports per stage the number of USB
vendor control transfers it spent (`UsbXferCount.h`, bumped by
`UsbTransport`). maburd keeps devourer's event stream off (it fills `/tmp`
at video rate); `MABUR_DEVOURER_EVENTS=1` turns it on for a run from
`/tmp`, routed to a private dup of stdout because the venc's `sdk_quiet`
brackets dup2 `/dev/null` over fds 1 and 2 while the radio thread is
still bringing up. Warm restart, before any change:

| stage | ms | transfers | µs/transfer |
|---|---|---|---|
| IQK | 267 | 2918 | 92 |
| RF radioA+B tables (read-modify-write per entry) | 263 | 3094 | 85 |
| RFK `cal_init` table | 213 | 2619 | 81 |
| efuse/OTP walk | 140 | 1149 | 122 |
| firmware download (49 × 4 KB + polls) | 122 | 1023 | 119 |
| BB `phy_reg` + AGC tables | 157 | 1935 | 81 |
| DACK | 59 | 696 | 85 |
| power-on, MAC/USB cfg, coex, FW H2Cs, TX power, … | ~77 | ~830 | |
| **total** | **1298** | **14265** | **91** |

**14,265 synchronous EP0 round trips at ~91 µs each, and nothing else.**
There is no sleep worth naming (the largest are 50 ms pseudo-entries in
the tables, hit a handful of times). The bring-up is paid in USB control
transfers, so the only levers are fewer transfers or cheaper ones.

### The floor of a control transfer on this SoC

A libusb microbench against the dongle from the drone (`usbctlbench`, in
the session scratchpad; 1000 × 1-byte reads/writes of `0x0640`):

| method | µs/transfer |
|---|---|
| synchronous `libusb_control_transfer` | 76–80 |
| async, 2 in flight | 43–45 |
| async, 4 in flight | 30–31 |
| async, 8 in flight | **27–28** |
| async, 16–32 in flight | 26–27 |

The bus cost is ~27 µs; the other ~50 µs is the host side of one URB
round trip (submit, interrupt, wake, return). EP0 completes URBs in
submission order, so a queue of writes followed by a read behaves exactly
like the synchronous sequence — the read still sees every write — while
the host only pays the round trip once per read.

### Pipelined register writes (devourer)

`IRtlTransport::write_batch_begin/end` + `flush_writes` (no-ops on PCIe).
Inside a batch, `UsbTransport` submits register writes as async control
URBs from a pool of 8 and does not wait; a read is submitted behind them
and waited for on its own completion only, so a read-modify-write pair
costs one wakeup instead of two; bulk transfers and `write_bytes` drain
the queue first. The Jaguar3 `InitWrite` opens a batch for the whole
bring-up and closes it before the coex thread starts (RAII guard, so a
throw never leaves the transport in batch mode). The millisecond-scale
delays in the table loaders, `Halrf8822e::delay_ms` and the efuse
power-cut flush before sleeping; sub-millisecond ones are noise next to
the ~0.2 ms a full queue holds. Single-threaded by contract.

Second change: the RF table load is write-only. The vendor's
`config_phydm_direct_write_rf_reg` does a read-modify-write under
`MASK20BITS`, preserving bits [31:20] of the direct-window dword. Those
bits read back **0 for every one of the 1540 entries**, cold boot and
warm restart alike (histogrammed on the 8812EU), so the read was a
synchronous round trip preserving nothing.

| warm InitWrite | ms | transfers |
|---|---|---|
| before | 1298 | 14265 |
| + async writes | 991 | 14262 |
| + async reads (one wait per RMW pair) | 741 | 14330 |
| + RF table write-only | **646** | **12789** |

**Cold boot: 2.04 → 0.74 s** measured with the events on through the
wrapper (before the RF change; ~0.65 expected after). The cold number
used to be 0.7 s worse than warm because every synchronous transfer
paid a scheduling wakeup under a loaded CPU; a queue that keeps the host
controller busy while the thread waits to run does not care.

### On the boot path

With the radio 1.4 s faster the venc side became the critical path
(first `stats:` line back to `drops=0` — the TX gate opened before the
first frame). So the module loader moved to a thread spawned at the top
of `run_real_mode`, under the 0.69 s USB port reset (an idle wait), and
the same thread then dlopens the MI libraries (`venc_core_preload`,
idempotent; `venc_core_start` still does it if nobody did).

| cold boot, first AU on the GS ring (s of uptime) | |
|---|---|
| "rcS, stamped" final layout | 4.02 / 4.09 |
| + pipelined InitWrite | 3.84 / 3.89 |
| + loader thread under the USB reset | 3.56 / 3.69 |
| + RF write-only + MI-library preload | **3.40 / 3.68** |

ausniff on the GS after the last one: 60.3 fps, 0 fid gaps, 0 resyncs;
`state=2`, `tx_failed=0`, one maburd, no respawns; run-to-run noise on
the AU time is ±0.15 s. Timeline of the last boot: maburd 1.30, USB reset
1.89 → 2.58, insmod chain done 2.45, MI clients 2.55, first AU ~3.5. The
critical path is now `maburd start → insmod chain (1.15 s, slowed by
maburd's own page-in and the reset's re-enumeration) → venc bring-up
(0.9)`; the radio (reset 0.69 + InitWrite 0.65) has ~0.3 s of slack.

What is left in InitWrite itself, and why it stops here: IQK 167 ms and
DACK 40 ms are the calibration algorithms' own poll loops (serial by
nature); the efuse walk (107 ms, 1149 transfers for ~380 OTP bytes, each
a trigger write + a ready poll) could be read ahead in blocks or cached
per unit, worth ~80 ms; the firmware download (~100 ms) is 49 chunks with
~20 polls each. None of it is on the critical path any more.

Deploy notes: devourer branch `jgr3-init-timing` (based on
`jgr3-keep-corrupted`, which the `../devourer` checkout must stay on for
GS builds — the new branch is a superset). **The GS RX bring-up (`Init`)
deliberately opens no batch** until it is measured on a ground-station
card; the write-only RF load applies to both paths, so the next maburgs
rebuild needs its ausniff gate. mabur: `MABUR_DEVOURER_EVENTS=1` env
knob, the loader thread + preload in `main.cpp`, `venc_core_preload`.
Drone runs it (rollback `/usr/bin/maburd.pre-usbpipe`).

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
  repairing the pad. This did not block the 2026-09-08 flash (see "The
  drone, flashed"), but it is why every irreversible write there was
  md5-verified by readback before the reboot, and why `baseaddr` was
  checked against the uImage header on both the old and the new kernel. `console=ttyS2,115200` on the FC pads (needs
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
