#!/usr/bin/env python3
"""Run commands at the SigmaStar U-Boot prompt, with timestamped output.

  ubcmd.py <out.log> <boot-wait-s> "<cmd>" ["<cmd>" ...]

Resets the board (over serial if it is already at the prompt, otherwise by
ssh'ing "reboot" to it), interrupts autoboot, then issues each command and
captures the console.  Used for NON-PERSISTENT A/Bs -- a reset discards
anything set here, so nothing needs saving until it is proven:

  ubcmd.py a.log 12 'setenv setargs setenv bootargs ${bootargs}' \
      'run setargs' 'setenv verify no' \
      'setenv bootcmd sf probe 0; sf read 0x20007fc0 0x50000 0x200000; bootm 0x20007fc0' \
      'run bootcmd'

Gotcha: commands are terminated with CR alone.  U-Boot re-runs the previous
command on a bare LF, so sending CRLF executes everything twice -- which
looks exactly like the hardware being slow.

Env: MABUR_SERIAL (default /dev/ttyUSB0), MABUR_DRONE (192.168.10.95).
"""
import serial, sys, time, subprocess, threading

out  = sys.argv[1]
dur  = float(sys.argv[2])
cmds = sys.argv[3:]

import os
HOST = os.environ.get("MABUR_DRONE", "192.168.10.95")
s = serial.Serial(os.environ.get("MABUR_SERIAL", "/dev/ttyUSB0"),
                  int(os.environ.get("MABUR_SERIAL_BAUD", "115200")), timeout=0.05)
s.reset_input_buffer()
recs=[]; buf=b""
def rd(t):
    global buf
    end=time.time()+t
    while time.time()<end:
        b=s.read(4096)
        if b: recs.append((time.time(),b)); buf+=b

def rd0(t):
    global buf
    end=time.time()+t
    while time.time()<end:
        b=s.read(4096)
        if b: recs.append((time.time(),b)); buf+=b

# Is the board already sitting at a U-Boot prompt?  If so reset over serial;
# otherwise it is running Linux and ssh reboot is the trigger.
s.write(b"\r"); s.flush(); rd0(0.6)
if b"OpenIPC # " in buf[-60:]:
    s.write(b"reset\r"); s.flush()
else:
    def trig():
        # Linux may still be coming up from a previous experiment.
        for _ in range(40):
            r = subprocess.run(["ssh","-o","ConnectTimeout=3","-o",
                                "StrictHostKeyChecking=no",
                                "root@"+HOST,"reboot"],
                               stdout=subprocess.DEVNULL,
                               stderr=subprocess.DEVNULL)
            if r.returncode == 0:
                return
            time.sleep(2)
    threading.Thread(target=trig, daemon=True).start()
buf=b""; recs=[]

end=time.time()+120; prompt=False
while time.time()<end:
    b=s.read(256)
    if b: recs.append((time.time(),b)); buf+=b
    if b"U-Boot 2015.01" in buf: s.write(b"\r"); s.flush()
    if b"OpenIPC # " in buf[-60:]: prompt=True; break
if not prompt:
    print("NO PROMPT"); sys.exit(1)
rd(0.4)
mark = len(buf)
for c in cmds:
    s.write((c+"\r").encode()); s.flush()
    rd(dur if c in ("run bootcmd",) or c.split()[0]=="bootm" else 1.2)
s.close()

# line assembly with per-line first-byte stamps
lines=[]; cur=bytearray(); cur_t=None
for ts,chunk in recs:
    n=len(chunk)
    for i,c in enumerate(chunk):
        bt=ts-(n-1-i)/11520.0
        if cur_t is None: cur_t=bt
        if c in (10,13):
            if cur: lines.append((cur_t,bytes(cur)))
            cur=bytearray(); cur_t=None
        else: cur.append(c)
if cur: lines.append((cur_t,bytes(cur)))
t0=lines[0][0]
with open(out,"w") as f:
    for t,ln in lines:
        f.write("%8.3f  %s\n"%(t-t0, ln.decode("utf-8","replace")))
# print from the first issued command onward
started=False
for t,ln in lines:
    txt=ln.decode("utf-8","replace")
    if not started and cmds and cmds[0] in txt: started=True
    if started: print("%8.3f  %s"%(t-t0,txt))
