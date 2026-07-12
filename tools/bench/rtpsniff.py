# Passive RTP tap: sniffs UDP packets to a port on an interface via
# AF_PACKET, so the live maburgs->PixelPilot session is measured without
# restarting anything (rtptap.py's port-swap procedure kills the session
# under test and rerolls the RX bring-up lottery). Root only.
#   python3 rtpsniff.py [iface] [port] [seconds]
# Reports the same seq/FU/frame accounting as rtptap.py, and classifies
# FU chains that end without the E bit by whether the chain's last packet
# carried the RTP marker (encoder idiosyncrasy, harmless to marker-aware
# depayloaders) or not (real mid-frame truncation).
import socket, struct, sys, time

iface = sys.argv[1] if len(sys.argv) > 1 else "lo"
port = int(sys.argv[2]) if len(sys.argv) > 2 else 5600
dur = float(sys.argv[3]) if len(sys.argv) > 3 else 30.0

ETH_P_ALL = 3
s = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(ETH_P_ALL))
s.bind((iface, 0))
s.settimeout(1.0)

n = 0; last_seq = None; gaps = 0; gap_seqs = 0; back = 0; dups = 0
fu_active = False; fu_last_had_marker = False
lost_start = 0; lost_end_marker = 0; lost_end_hard = 0
frames = 0; frame_bad = 0; cur_ts = None; frame_has_gap = False
bad_frames = 0; ok_frames = 0; nal_hist = {}
bytes_total = 0
gap_log = []
t0 = time.time(); end = t0 + dur
while time.time() < end:
    try:
        pkt = s.recv(65535)
    except socket.timeout:
        continue
    # ethernet(14) + ip + udp; assume IPv4 no options unless IHL says more
    if len(pkt) < 42 or pkt[12:14] != b"\x08\x00": continue
    ihl = (pkt[14] & 0x0F) * 4
    if pkt[23] != 17: continue  # not UDP
    udp = 14 + ihl
    dport = (pkt[udp + 2] << 8) | pkt[udp + 3]
    if dport != port: continue
    d = pkt[udp + 8:]
    if len(d) < 14: continue
    if d[0] >> 6 != 2: continue
    n += 1; bytes_total += len(d)
    seq = (d[2] << 8) | d[3]
    ts = struct.unpack(">I", d[4:8])[0]
    marker = bool(d[1] & 0x80)
    pay = d[12:]
    if last_seq is not None:
        delta = (seq - last_seq) & 0xFFFF
        if delta == 1: pass
        elif delta == 0: dups += 1
        elif delta <= 0x7FFF:
            gaps += 1; gap_seqs += delta - 1; frame_has_gap = True
            if len(gap_log) < 12: gap_log.append((round(time.time()-t0,1), delta-1))
        else: back += 1
    last_seq = seq
    if cur_ts is not None and ts != cur_ts:
        frames += 1
        if frame_has_gap or frame_bad: bad_frames += 1
        else: ok_frames += 1
        frame_has_gap = False; frame_bad = 0
    cur_ts = ts
    t = (pay[0] >> 1) & 0x3F
    nal_hist[t] = nal_hist.get(t, 0) + 1
    if t == 49 and len(pay) >= 3:
        fuh = pay[2]; start = fuh & 0x80; endb = fuh & 0x40
        if start:
            if fu_active:
                if fu_last_had_marker: lost_end_marker += 1
                else: lost_end_hard += 1; frame_bad += 1
            fu_active = True
        else:
            if not fu_active: lost_start += 1; frame_bad += 1
        if endb: fu_active = False
        fu_last_had_marker = marker
    if marker:
        frames += 1
        if frame_has_gap or frame_bad: bad_frames += 1
        else: ok_frames += 1
        frame_has_gap = False; frame_bad = 0
        cur_ts = None

el = dur
print(f"pkts={n} ({bytes_total*8/el/1e6:.2f} Mbps) dups={dups}")
print(f"seq: gaps={gaps} (missing {gap_seqs} seqs, {gap_seqs*100/max(1,n+gap_seqs):.2f}%) out_of_order={back}")
print(f"fu: lost_start={lost_start} end_via_marker={lost_end_marker} end_lost_hard={lost_end_hard}")
print(f"frames={frames} ok={ok_frames} bad={bad_frames} ({bad_frames*100/max(1,frames):.0f}% bad) -> ok_fps={ok_frames/el:.1f}")
print("nal types:", dict(sorted(nal_hist.items(), key=lambda x: -x[1])[:8]))
print("gap log (t_offset_s, seqs):", gap_log)
