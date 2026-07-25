# Raw RTP sequence dump: logs EVERY sniffed RTP packet on iface:port to a
# file, one line per packet, plus the kernel AF_PACKET drop counter at exit.
# Companion to rtpsniff.py, but does no online accounting — the point is an
# offline, set-membership gap analysis (seqdump_analyze.py) that is immune
# to loopback TX/RX duplicate interleaving and can prove whether a "gap" is
# real loss or a sniffer artifact (tp_drops > 0 => capture is suspect).
#   python3 seqdump.py [iface] [port] [seconds] [outfile]
# Line: <us_since_start> <seq> <rtp_ts> <marker> <nal_type> <fu_flags> <paylen> <fu_rt>
#   fu_flags: bit0=FU start, bit1=FU end (0 if not FU type 49)
#   fu_rt: FU inner (real) NAL type, -1 if not FU — same NAL rule the drone
#   routes frames by (crit 16-23/32-34 -> stream 0)
import socket, struct, sys, time

iface = sys.argv[1] if len(sys.argv) > 1 else "lo"
port = int(sys.argv[2]) if len(sys.argv) > 2 else 5600
dur = float(sys.argv[3]) if len(sys.argv) > 3 else 60.0
out = sys.argv[4] if len(sys.argv) > 4 else "/tmp/seqdump.txt"

ETH_P_ALL = 3
SOL_PACKET = 263
PACKET_STATISTICS = 6
s = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(ETH_P_ALL))
s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 << 20)
s.bind((iface, 0))
s.settimeout(1.0)
# drain the stats counter so the exit read covers only our window
s.getsockopt(SOL_PACKET, PACKET_STATISTICS, 8)

f = open(out, "w", buffering=1 << 20)
n = 0
t0 = time.monotonic()
end = t0 + dur
while time.monotonic() < end:
    try:
        pkt = s.recv(65535)
    except socket.timeout:
        continue
    now = time.monotonic()
    if len(pkt) < 42 or pkt[12:14] != b"\x08\x00": continue
    ihl = (pkt[14] & 0x0F) * 4
    if pkt[23] != 17: continue
    udp = 14 + ihl
    dport = (pkt[udp + 2] << 8) | pkt[udp + 3]
    if dport != port: continue
    d = pkt[udp + 8:]
    if len(d) < 14: continue
    if d[0] >> 6 != 2: continue
    n += 1
    seq = (d[2] << 8) | d[3]
    ts = struct.unpack(">I", d[4:8])[0]
    marker = 1 if d[1] & 0x80 else 0
    pay = d[12:]
    nal = (pay[0] >> 1) & 0x3F
    fu = 0
    fu_rt = -1
    if nal == 49 and len(pay) >= 3:
        if pay[2] & 0x80: fu |= 1
        if pay[2] & 0x40: fu |= 2
        fu_rt = pay[2] & 0x3F
    f.write(f"{int((now - t0) * 1e6)} {seq} {ts} {marker} {nal} {fu} {len(pay)} {fu_rt}\n")
f.close()
st = s.getsockopt(SOL_PACKET, PACKET_STATISTICS, 8)
tp_packets, tp_drops = struct.unpack("II", st)
print(f"captured={n} kernel_seen={tp_packets} kernel_drops={tp_drops} out={out}")
if tp_drops:
    print("WARNING: kernel dropped packets — gap analysis is NOT trustworthy")
