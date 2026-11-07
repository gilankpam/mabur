#!/usr/bin/env python3
"""tools/bench/fecab/sched.py STEP -- runs on the GS, one arm's whole loss
schedule, mono-stamped so it survives the host ssh session dropping
mid-run (the GS is reached over its Wi-Fi hotspot). Drives maburgs'
--loss-sim control socket (127.0.0.1:8303, tools/bench/losssim.py's
protocol) through three steps: `clean` (loss off), `l2b1` (eff=2% burst=1
on both video streams) and `l5b3` (eff=5% burst=3). run.sh starts this
detached (setsid ... &) 25 s after restarting maburd, waits for SCHED_DONE
in its redirected stdout, then cats that output into the local .run file
-- tools/bench/fecab/analyze.py reads the `--- step` and `mono_us=` lines
from there to find each step's analysis window.
"""
import socket
import sys
import time

STEP = int(sys.argv[1]) if len(sys.argv) > 1 else 60
PORT = 8303

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.settimeout(2)


def send(cmd):
    s.sendto(cmd.encode(), ("127.0.0.1", PORT))
    try:
        reply = s.recv(2048).decode().strip()
    except socket.timeout:
        reply = "(no reply)"
    print(f"mono_us={int(time.monotonic() * 1e6)} cmd={cmd!r} reply={reply}", flush=True)


STEPS = (
    ("clean", []),
    ("l2b1", ["s0 eff=2 burst=1", "s1 eff=2 burst=1"]),
    ("l5b3", ["s0 eff=5 burst=3", "s1 eff=5 burst=3"]),
)

for name, cmds in STEPS:
    print(f"--- step {name}", flush=True)
    send("off")
    for c in cmds:
        send(c)
    time.sleep(STEP)
    send("status")
send("off")
print("SCHED_DONE", flush=True)
