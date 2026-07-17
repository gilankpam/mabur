#!/usr/bin/env python3
"""fu_probe.py — reliable FU-truncation probe for the maburgs->PixelPilot
RTP stream. The trustworthy successor to rtpsniff.py for loss/truncation
NUMBERS (rtpsniff keeps the richer per-gap log, but its default-size
AF_PACKET buffer drops captures under load and massively over-counts loss
— reconfirmed 2026-07-17: it reported 4.7% missing / 62% bad frames on a
stream this probe measured at 0.02% missing / 0 truncations).

What it fixes over rtpsniff:
  - 8 MB SO_RCVBUF (the whole difference between 62% and 0.4% "bad");
  - dedup by RTP seq BEFORE analysis (AF_PACKET on lo captures every
    packet twice: once TX, once RX);
  - seq-space unwrap (a capture window crossing 65535->0 otherwise
    scrambles chain order and fabricates ~46k "missing" seqs);
  - marker-classified chain ends: a chain ending marker-but-no-E is a
    known-harmless encoder idiosyncrasy; no-marker-no-E ("hard") is real
    mid-frame truncation — with zero transport loss around it, that means
    the slice tail was overwritten in waybeam's venc ring BEFORE maburd
    read it (drone drain ceiling exceeded; see
    docs/source-truncation-drain-ceiling.md).

Run ON THE GS (root, python3):  python3 fu_probe.py [iface] [port] [seconds]
Healthy stream @60fps: fps ~59.5+, hard=0, missing ~0.
"""
import socket
import sys
import time

iface = sys.argv[1] if len(sys.argv) > 1 else "lo"
port = int(sys.argv[2]) if len(sys.argv) > 2 else 5600
dur = float(sys.argv[3]) if len(sys.argv) > 3 else 20.0

s = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(3))
s.bind((iface, 0))
s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 * 1024 * 1024)
s.settimeout(1.0)

pkts = {}
caps = 0
t0 = time.time()
while time.time() < t0 + dur:
    try:
        pkt = s.recv(65535)
    except socket.timeout:
        continue
    if len(pkt) < 44 or pkt[12:14] != b"\x08\x00":
        continue
    ihl = (pkt[14] & 0x0F) * 4
    if pkt[23] != 17:
        continue
    udp = 14 + ihl
    if ((pkt[udp + 2] << 8) | pkt[udp + 3]) != port:
        continue
    rtp = udp + 8
    seq = (pkt[rtp + 2] << 8) | pkt[rtp + 3]
    caps += 1
    if seq not in pkts:
        pkts[seq] = pkt[rtp:]

u = sorted(pkts)
if u and u[-1] - u[0] > 32768:  # window crossed the 16-bit wrap
    u = sorted((q + 65536 if q < 32768 else q) for q in u)


def get(q):
    return pkts[q % 65536]


fu_active = False
prev_marker = False
closed_e = end_marker = end_hard = lost_start = 0
frames = holes = hole_events = 0
for i, q in enumerate(u):
    p = get(q)
    if i and q - u[i - 1] > 1:
        holes += q - u[i - 1] - 1
        hole_events += 1
    marker = (p[1] >> 7) & 1
    nal = (p[12] >> 1) & 0x3F
    if nal == 49 and len(p) > 14:  # HEVC FU
        fh = p[14]
        s_bit = fh >> 7
        e_bit = (fh >> 6) & 1
        if s_bit and fu_active:
            if prev_marker:
                end_marker += 1  # harmless encoder idiosyncrasy
            else:
                end_hard += 1  # REAL mid-frame truncation
        elif not s_bit and not fu_active:
            lost_start += 1
        if s_bit:
            fu_active = True
        if e_bit:
            fu_active = False
            closed_e += 1
    if marker:
        frames += 1
    prev_marker = marker

span = (u[-1] - u[0] + 1) if u else 0
print(f"captures={caps} unique={len(u)} (dedup x{caps / max(1, len(u)):.2f})")
print(f"seq: missing={holes} ({100.0 * holes / max(1, span):.2f}%) "
      f"in {hole_events} gap(s) over span {span}")
print(f"frames={frames} fps={frames / dur:.1f}")
print(f"fu chains: closed_E={closed_e} end_via_marker={end_marker} "
      f"end_HARD={end_hard} lost_start={lost_start}")
if closed_e + end_hard:
    pct = 100.0 * end_hard / (closed_e + end_hard)
    print(f"hard truncation: {pct:.1f}% "
          f"{'** SOURCE OVERLOAD — see docs/source-truncation-drain-ceiling.md **' if pct > 2 else '(healthy)'}")
