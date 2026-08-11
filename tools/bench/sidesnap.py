#!/usr/bin/env python3
"""Grab sideport datagrams passively, without binding the port.

The sideport is a UDP push to a fixed port, so only ONE process can bind it
-- normally maburtop or a socat recorder. When that seat is taken, a second
consumer cannot `socat udp-recv` the same port (bind gives EADDRINUSE); the
documented alternative (CLAUDE.md) is to sniff loopback instead. This is
that alternative, packaged.

Reads whole IPv4/UDP datagrams off `lo` via AF_PACKET and writes the
payloads to stdout, one JSON object per line. Never binds the port, so it
composes with a running maburtop rather than displacing it.

  python3 sidesnap.py                 # one datagram from :8300, then exit
  python3 sidesnap.py --count 20      # 20 of them, as JSONL
  python3 sidesnap.py --port 8302 --seconds 5

Needs CAP_NET_RAW (run as root on the GS). Loopback MTU is 65536, so a
sideport datagram is never fragmented and one packet is always one whole
datagram -- this would need reassembly on a real NIC, but not here.
"""
import argparse
import socket
import struct
import sys
import time

ap = argparse.ArgumentParser()
ap.add_argument("--port", type=int, default=8300, help="destination UDP port")
ap.add_argument("--iface", default="lo")
ap.add_argument("--count", type=int, default=1, help="datagrams to capture")
ap.add_argument("--seconds", type=float, default=10.0, help="give-up timeout")
a = ap.parse_args()

ETH_P_IP = 0x0800
try:
    s = socket.socket(socket.AF_PACKET, socket.SOCK_DGRAM, socket.htons(ETH_P_IP))
except PermissionError:
    sys.exit("sidesnap: needs CAP_NET_RAW (run as root)")
s.bind((a.iface, 0))
s.settimeout(0.5)

got = 0
deadline = time.monotonic() + a.seconds
while got < a.count and time.monotonic() < deadline:
    try:
        pkt = s.recv(65535)
    except socket.timeout:
        continue
    if len(pkt) < 20 or (pkt[0] >> 4) != 4:
        continue
    ihl = (pkt[0] & 0x0F) * 4
    if pkt[9] != socket.IPPROTO_UDP or len(pkt) < ihl + 8:
        continue
    _sport, dport, ulen = struct.unpack("!HHH", pkt[ihl:ihl + 6])
    if dport != a.port:
        continue
    # ulen counts the 8-byte UDP header; trust it over len(pkt) so trailing
    # padding never rides along into the JSON.
    payload = pkt[ihl + 8:ihl + ulen]
    sys.stdout.write(payload.decode("utf-8", errors="replace") + "\n")
    sys.stdout.flush()
    got += 1

if got < a.count:
    sys.exit(f"sidesnap: captured {got}/{a.count} on {a.iface}:{a.port} "
             f"in {a.seconds}s -- is the sender running?")
