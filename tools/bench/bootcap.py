#!/usr/bin/env python3
"""Timestamped serial boot capture.

  bootcap.py <seconds> <out.log> ["<trigger>"]

Each output line is "<t_rel>  <text>", where t_rel is seconds since the
first byte of the capture.  A line is stamped with the arrival time of its
FIRST byte, not its last: at 115200 a long line takes ~4 ms to transmit, and
stage boundaries in a boot log are worth more than that.

<trigger> fires just after the port opens.  Two forms:
  "reset"          - send "reset" to a U-Boot prompt over the serial line
                     (works with no network at all)
  anything else    - run it as a shell command, e.g. "ssh root@HOST reboot"

Env: MABUR_SERIAL (default /dev/ttyUSB0), MABUR_SERIAL_BAUD (115200).
NixOS: nix-shell -p python3Packages.pyserial --run "python3 ...".
"""
import serial, sys, time, subprocess, threading

import os
port   = os.environ.get("MABUR_SERIAL", "/dev/ttyUSB0")
baud   = int(os.environ.get("MABUR_SERIAL_BAUD", "115200"))
dur    = float(sys.argv[1]) if len(sys.argv) > 1 else 40.0
outf   = sys.argv[2] if len(sys.argv) > 2 else "boot.log"
trigger= sys.argv[3] if len(sys.argv) > 3 else ""   # shell cmd to fire after open

s = serial.Serial(port, baud, timeout=0.05)
s.reset_input_buffer()

fired = {"t": None}
def fire():
    time.sleep(0.3)
    fired["t"] = time.time()
    if trigger == "reset":
        # Board is sitting at the U-Boot prompt.  Send CR alone: a bare LF
        # makes U-Boot re-run the previous command.
        s.write(b"reset\r"); s.flush()
    else:
        subprocess.run(trigger, shell=True, stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL)
if trigger:
    threading.Thread(target=fire, daemon=True).start()

t_start = time.time()
recs = []            # (host_time, bytes)
while time.time() - t_start < dur:
    b = s.read(4096)
    if b:
        recs.append((time.time(), b))
s.close()

# Re-assemble into lines, stamping each line with the arrival time of its
# FIRST byte (the byte-level stamp is what makes stage boundaries honest).
lines = []
cur = bytearray(); cur_t = None
for ts, chunk in recs:
    # linear interpolation across the chunk at 115200 8N1 = 11520 B/s
    n = len(chunk)
    for i, c in enumerate(chunk):
        bt = ts - (n - 1 - i) / 11520.0
        if cur_t is None:
            cur_t = bt
        if c in (10, 13):
            if cur:
                lines.append((cur_t, bytes(cur)))
            cur = bytearray(); cur_t = None
        else:
            cur.append(c)
if cur:
    lines.append((cur_t, bytes(cur)))

t0 = lines[0][0] if lines else t_start
with open(outf, "w") as f:
    f.write("# capture_start_host=%.6f trigger_host=%s first_byte_host=%.6f\n"
            % (t_start, ("%.6f" % fired["t"]) if fired["t"] else "none", t0))
    for t, ln in lines:
        f.write("%8.3f  %s\n" % (t - t0, ln.decode("utf-8", "replace")))
print("wrote %s: %d lines, span %.3f s" % (outf, len(lines),
      (lines[-1][0]-t0) if lines else 0.0))
if fired["t"]:
    print("trigger fired at %+.3f s relative to first byte" % (fired["t"]-t0))
