# EMAX Wyvern Link Alpha — first flash

Board `ssc338q_fpv_emax-wyvern-link-alpha`: the URLLC AIO image
(SSC338Q / IMX415 / NOR 16M, everything in `devices/_mabur-common`) with
maburd built for the **RTL8812CU** (devourer rtl8822c) instead of the
8812EU. The ground station is unchanged — RK3566 + 8812EU cards running
current mabur `maburgs`/`maburplay`.

**Status: untested on hardware.** Nobody with this board has flown it yet.

## 0 — Before flashing: confirm the card

On the board's stock firmware:

```sh
lsusb                      # must show 0bda:c812 (8812CU) or 0bda:c82c (8822CU)
fw_printenv | grep -E '^(sensor|soc|mtdparts|bootargs)='
cat /proc/mtd
```

`0bda:a81a` means an **8812EU** — flash the URLLC AIO image instead; this
one fails at radio bring-up (`CreateRtlDevice failed (unsupported chip)`).

## 1 — Back up

```sh
HOST=root@<wyvern-ip>
ssh $HOST 'fw_printenv' > env-backup-$(date +%Y%m%d%H%M).txt
for n in 0 1 2 3; do ssh $HOST "dd if=/dev/mtd$n bs=64k 2>/dev/null" > mtd$n-backup.bin; done
```

## 2 — Pin the sensor

`load_sigmastar` takes the sensor from the U-Boot env and only falls back to
an I2C probe when it is missing (a failed probe = no video, maburd in a 2 s
respawn loop). Pin it:

```sh
ssh $HOST 'fw_setenv sensor imx415'
```

## 3 — Flash kernel + rootfs (NOT U-Boot)

Do **not** flash `u-boot-ssc338q-nor-padded.bin`: the stock U-Boot boots
this image, and a bad U-Boot on a board without a serial console needs an
SPI programmer (`docs/flash-uboot.md`).

The rootfs is LZO squashfs, ~5.7 MB. U-Boot sizes the rootfs partition from
the squashfs **already on flash** (`< 5 MB` → `rootmtd=5120k`), so on a
board coming from a stock XZ image the partition is still 5 MB when
`sysupgrade` runs, `flashcp` refuses ("bigger than /dev/mtd3") — and
**sysupgrade reports success anyway**. Break the cycle by pinning the
partition to 8192k for one boot:

```sh
ssh $HOST 'fw_printenv bootargs'          # note it: step 4 restores it
# Same bootargs with ${rootmtd} replaced by the literal 8192k:
ssh $HOST "fw_setenv bootargs '<bootargs with 8192k(rootfs)>'" && ssh $HOST reboot
ssh $HOST 'cat /proc/mtd'                 # mtd3 "rootfs" must be 00800000
```

Then flash from this fork's release (the board's `upgrade` URL is set to it
by `customizer.sh` after the first boot, not before):

```sh
scp -O ssc338q_fpv_emax-wyvern-link-alpha-nor.tgz $HOST:/tmp/
ssh $HOST 'cd /tmp && tar xzf ssc338q_fpv_emax-wyvern-link-alpha-nor.tgz && \
  sysupgrade -f --kernel=/tmp/uImage.ssc338q --rootfs=/tmp/rootfs.squashfs.ssc338q -n'
```

**Verify by reading back, not by sysupgrade's output**: the squashfs
superblock compression byte at mtd3 offset 20 must be `03` (LZO), not `04`
(XZ):

```sh
ssh $HOST 'dd if=/dev/mtd3 bs=1 skip=20 count=1 2>/dev/null | hexdump -C'
```

## 4 — First boot

`customizer.sh` runs once (overlay wiped by `-n`) and rewrites `bootargs`
back to the `${rootmtd}` form — U-Boot then derives 8192k from the LZO
image by itself. It takes effect on the **next** boot, so reboot once more.

Check:

```sh
ssh $HOST 'grep -E "opened device|CreateRtlDevice|error" /tmp/mabur.log | head'
# expect: opened device 0bda:c812
```

Then on the GS: video on the glass, `maburtop` link panel live,
`tools/bench/ausniff.py` clean.

## 5 — Config notes (`/etc/mabur.toml`)

- `radio.power_mode = "none"` as shipped. The 8812CU then flies devourer's
  flat TX-power reference (index 40, every channel, uncalibrated). Leave it
  there until `maburcal start` (run on the GS) has measured *this* unit's
  walls — the shipped `rate_walls_rel` are another board's 8812EU numbers.
  Calibrate on the channel flown most: on the CU the walls are exact there
  and only approximate on other channels (mabur `docs/deploy.md`,
  "Supported drone cards").
- `msp.serial = "/dev/ttyS2"` is the URLLC AIO's FC UART. Unverified on the
  Wyvern: if the flight controller is wired to another UART, MSP OSD and arm
  state stay silent (the link still flies, but low-power disarmed mode never
  sees "armed"). The stock firmware's telemetry config names the right port.
