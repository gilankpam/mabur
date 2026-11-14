# SD-card recovery

A board that no longer boots Linux can be re-flashed from an SD card, with
no serial console and no SPI programmer. U-Boot looks for `boot.scr` on the
card on **every** boot, before it runs `bootcmd`, so the card gets control
however broken the kernel or rootfs is. The script below writes the kernel
and rootfs from the card, then the board resets and boots.

This covers the board `ssc338q_fpv_openipc-urllc-aio` (SSC338Q, NOR 16M) and
`ssc338q_fpv_emax-wyvern-link-alpha`, which has the same flash layout.

## When it works, and when it cannot

| Board state | SD recovery |
|---|---|
| Kernel or rootfs corrupt (interrupted `sysupgrade`, bad image, boot loop) | Works |
| U-Boot env lost (bad CRC) | Works: U-Boot falls back to its built-in env, whose `baseaddr` is the default |
| `baseaddr` is not `0x21000000` (see below) | Card is **silently ignored** |
| U-Boot from this repo at `f8a00c4` | Card is ignored: that build has no SD probe at all (restored in `496b2e2`) |
| U-Boot itself (`mtd0`) corrupt | Does not work: needs an SPI programmer |

### Check `baseaddr` while the board still boots

U-Boot loads `boot.scr` to `${baseaddr}` and then runs a bare `source`,
which reads the fixed `0x21000000`. With any other `baseaddr` the script is
loaded to one address and run from another, and nothing happens.

Images built before the fix set `baseaddr=0x20007FC0`. Check it, and fix it
now, while you still have ssh:

```sh
fw_printenv baseaddr                 # must be 0x21000000
fw_setenv baseaddr 0x21000000
fw_setenv bootcmd 'sf probe 0; setenv setargs setenv bootargs ${bootargs}; run setargs; sf read 0x20007FC0 ${kernaddr} ${kernsize}; bootm 0x20007FC0'
```

The `bootcmd` line keeps the faster XIP kernel boot that the old `baseaddr`
gave. Current images set both lines in `/usr/share/openipc/customizer.sh`,
but only when the overlay is fresh: a plain `sysupgrade` keeps
`/etc/custom.ok`, so the customizer does not run again. Use `sysupgrade -n`,
or the two `fw_setenv` lines above.

A board that is already dead **with the old `baseaddr`** needs the
double-copy card described at the end of this page.

## 1 — Make the card

Use a FAT32-formatted card and put the files in the root of the first
partition. Other files on the card are left alone: U-Boot only reads from
it.

Take the images from the `-nor.tgz` of the release you want on the board:

```sh
tar xzf ssc338q_fpv_openipc-urllc-aio-nor.tgz
md5sum -c uImage.ssc338q.md5sum rootfs.squashfs.ssc338q.md5sum
```

Write `boot.cmd`. Set `STAMP` to anything that names this image; the
script saves it in the env once the flash is complete, which is how it
avoids re-flashing on every boot while the card stays in.

```sh
STAMP=mabur-4c94bc23
cat > boot.cmd <<EOF
setenv sddev 0
if fatsize mmc 1 boot.scr; then setenv sddev 1; fi
if fatsize mmc 0 boot.scr; then setenv sddev 0; fi
if test "\${mabur_sd}" = "$STAMP"; then
echo "mabur SD: $STAMP already flashed, booting"
run bootcmd
fi
echo "mabur SD: flashing $STAMP"
setenv mabur_sd_step 0
if sf probe 0; then
setenv mabur_sd_step 1
if fatload mmc \${sddev} 0x22100000 uImage.ssc338q; then
setenv mabur_sd_step 2
if sf erase 0x50000 0x200000 && sf write 0x22100000 0x50000 \${filesize}; then
setenv mabur_sd_step 3
if fatload mmc \${sddev} 0x22100000 rootfs.squashfs.ssc338q; then
setenv mabur_sd_step 4
if sf erase 0x250000 0x800000 && sf write 0x22100000 0x250000 \${filesize}; then
setenv mabur_sd_step 5
setenv mabur_sd $STAMP
saveenv
reset
fi
fi
fi
fi
fi
echo "mabur SD: FLASH FAILED at step \${mabur_sd_step}"
saveenv
run bootcmd
EOF
mkimage -A arm -O linux -T script -C none -n mabur-sd -d boot.cmd boot.scr
```

`mkimage` comes with the `u-boot-tools` package (`nix-shell -p ubootTools`
on NixOS).

Copy `boot.scr`, `uImage.ssc338q` and `rootfs.squashfs.ssc338q` to the card.
Run `sync` before you pull it.

Keep the script as written. Old U-Boot hush is not trusted with comments
inside a script or with `&&` across lines. Each step is its own `if`, so a
failed read stops before anything is written with a stale `${filesize}`.
The stamp is saved last, so a flash that stops part-way retries on the next
power-up.

### Optional: wipe the overlay too

The script keeps the overlay (`rootfs_data`), and with it `/etc`, the root
password, `mabur.key` and anything else written since the last
`sysupgrade -n`. If the overlay itself is what breaks the boot, erase it as
one more step. In `boot.cmd`, replace

```
setenv mabur_sd_step 5
setenv mabur_sd $STAMP
saveenv
reset
fi
```

with

```
setenv mabur_sd_step 5
if sf erase 0xa50000 0x5b0000; then
setenv mabur_sd_step 6
setenv mabur_sd $STAMP
saveenv
reset
fi
fi
```

A successful run then reads step 6, not 5. After a wipe, the board comes up
as a fresh flash: set a root password, put `mabur.key` back and restart
maburd. The customizer runs again on that first boot. The wipe step has not
been tested on hardware.

## 2 — Run it

1. Power the board off and insert the card.
2. Power it on. Expect about a minute: the erase and write add roughly 45 s
   to a normal boot. The LEDs look the same as on any boot.
3. The board resets by itself after the flash and boots the new image. That
   boot finds the stamp and goes straight to `bootcmd`.
4. Remove the card, or delete `boot.scr` from it. Left in, it costs only the
   stamp check on each boot.

To confirm it from Linux afterwards:

```sh
fw_printenv mabur_sd mabur_sd_step   # your STAMP, and 5
```

If it does not come back:

- **Power-cycle again with the card in.** A flash that stopped part-way has
  saved no stamp and will run again.
- **If the board boots without flashing**, U-Boot did not run the script.
  Check `baseaddr` (above), and that the card is FAT32 with the files in the
  first partition.
- **If the board boots but `mabur_sd_step` is below 5**, the script ran and
  stopped at that step. 1 = `sf probe` OK but the kernel file was not read,
  2 = kernel erase/write failed, 3 = kernel written but the rootfs file
  was not read, 4 = rootfs erase/write failed.

## Interrupted `sysupgrade`

`sysupgrade` erases and writes `mtd2` (kernel), then `mtd3` (rootfs). A
dropped ssh session or Ctrl-C does not stop it. Only a power cut or a hang
does, and it never writes U-Boot. Whichever partition it stopped in, the
board either sits at the U-Boot prompt, hangs in a half-written kernel, or
boot-loops on a kernel panic every 20 s. Each of those passes through
U-Boot's SD probe, so insert a card made from the image you were upgrading
to and power-cycle.

## Old `baseaddr` and the board will not boot: the double-copy card

If the board is dead and still has `baseaddr=0x20007FC0`, `fatload` puts
`boot.scr` at `0x20007FC0` while `source` reads `0x21000000`. That is
`0xFF8040` bytes into the file. Put the script image at both places and it
runs either way:

```sh
mkimage -A arm -O linux -T script -C none -n mabur-sd -d boot.cmd one.scr
python3 - <<'PY'
one = open('one.scr', 'rb').read()
off = 0x21000000 - 0x20007FC0
buf = bytearray(b'\xff' * off)
buf[:len(one)] = one
open('boot.scr', 'wb').write(bytes(buf) + one)
PY
```

The resulting `boot.scr` is about 16 MiB. The flash itself then runs as
above.

**Status:** not yet tested on hardware. The single-copy card above was
tested on a WiFiLink 2, as a full kernel and rootfs re-flash.
