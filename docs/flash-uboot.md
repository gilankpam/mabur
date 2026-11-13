## Flashing U-Boot

`u-boot-<soc>-nor-padded.bin` is the entire 256k `boot` partition, padded
with `0xFF` (the erased state of NOR), so writing it leaves no stale tail
behind. It is built from the fork this repo pins, not taken from an OpenIPC
release.

**Attach a serial console first.** If the new U-Boot does not boot, there is
no recovery short of an SPI programmer, and with no console you will not
even see how it failed. On a board where that is impossible, the md5
readback in step 4 is the only safety net you have — do not skip it.

```sh
HOST=root@192.168.1.10          # your camera
IMG=u-boot-<soc>-nor-padded.bin # downloaded from this release
```

**1 — Back up mtd0 and the environment.** This is the only rollback, and it
is good only for as long as the new U-Boot still boots Linux.

```sh
ssh $HOST 'dd if=/dev/mtd0 bs=64k 2>/dev/null' > mtd0-backup-$(date +%Y%m%d%H%M).bin
ssh $HOST 'fw_printenv' > env-backup-$(date +%Y%m%d%H%M).txt
```

**2 — Transfer, and verify the transfer.** The result must equal the md5
published above.

```sh
scp -O "$IMG" $HOST:/tmp/
ssh $HOST "md5sum /tmp/$(basename $IMG)"
```

**3 — Flash.** The irreversible step; keep it a command of its own.

```sh
ssh $HOST "flashcp /tmp/$(basename $IMG) /dev/mtd0"
```

**4 — Read back and compare BEFORE rebooting.** Must equal the same md5.
**If it does not match, re-flash — do not reboot.**

```sh
ssh $HOST 'dd if=/dev/mtd0 bs=64k 2>/dev/null | md5sum'
```

**5 — Reboot**, with the console watching if you have one.

```sh
ssh $HOST reboot
```

### Rollback

Name the backup explicitly — a glob would match every backup you have ever
taken, and `flashcp` would take the wrong one. Verify by readback as in
step 4.

```sh
scp -O mtd0-backup-<stamp>.bin $HOST:/tmp/
ssh $HOST "flashcp /tmp/mtd0-backup-<stamp>.bin /dev/mtd0"
```

### Two things that do not work

- **`sysupgrade` cannot flash U-Boot.** Its only targets are `--kernel` and
  `--rootfs`; it never touches `/dev/mtd0`. It is still the right tool for
  the kernel and rootfs from this same build, and it reboots unconditionally
  for a reason — do not pass `-x`.
- **The `ubnor` U-Boot command is `sf erase 0x0 0x50000`**, which spans mtd0
  *and* the environment at `0x40000`. It takes `ethaddr` and every
  `fw_setenv` setting with it. `flashcp` to `/dev/mtd0` touches only mtd0.
