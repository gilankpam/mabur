import socket, struct, sys, time

port = int(sys.argv[1]) if len(sys.argv) > 1 else 5601
dur = float(sys.argv[2]) if len(sys.argv) > 2 else 30.0
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(("0.0.0.0", port))
s.settimeout(1.0)

n = 0; t0 = None; last_seq = None; gaps = 0; gap_seqs = 0; back = 0; dups = 0
fu_active = False; fu_lost_start = 0; fu_lost_end = 0
frames = 0; frame_pkts = 0; frame_bad = 0; cur_ts = None; frame_has_gap = False
bad_frames = 0; ok_frames = 0; nal_hist = {}
bytes_total = 0; malformed = 0
end = time.time() + dur
while time.time() < end:
    try:
        d, _ = s.recvfrom(4096)
    except socket.timeout:
        continue
    if t0 is None: t0 = time.time()
    n += 1; bytes_total += len(d)
    if len(d) < 14: malformed += 1; continue
    v = d[0] >> 6
    if v != 2: malformed += 1; continue
    seq = (d[2] << 8) | d[3]
    ts = struct.unpack(">I", d[4:8])[0]
    marker = bool(d[1] & 0x80)
    pay = d[12:]
    # seq accounting
    if last_seq is not None:
        delta = (seq - last_seq) & 0xFFFF
        if delta == 1: pass
        elif delta == 0: dups += 1
        elif delta <= 0x7FFF:
            gaps += 1; gap_seqs += delta - 1; frame_has_gap = True
        else: back += 1
    last_seq = seq
    # frame boundary by ts change
    if cur_ts is not None and ts != cur_ts:
        frames += 1
        if frame_has_gap or frame_bad: bad_frames += 1
        else: ok_frames += 1
        frame_has_gap = False; frame_bad = 0; frame_pkts = 0
    cur_ts = ts
    frame_pkts += 1
    # NAL structure
    t = (pay[0] >> 1) & 0x3F
    nal_hist[t] = nal_hist.get(t, 0) + 1
    if t == 49:
        if len(pay) < 3: malformed += 1; continue
        fuh = pay[2]; start = fuh & 0x80; endb = fuh & 0x40
        if start:
            if fu_active: fu_lost_end += 1; frame_bad += 1
            fu_active = True
        else:
            if not fu_active: fu_lost_start += 1; frame_bad += 1
        if endb: fu_active = False
    if marker:
        frames += 1
        if frame_has_gap or frame_bad: bad_frames += 1
        else: ok_frames += 1
        frame_has_gap = False; frame_bad = 0; frame_pkts = 0
        cur_ts = None

el = dur
print(f"pkts={n} ({bytes_total*8/el/1e6:.2f} Mbps) malformed={malformed} dups={dups}")
print(f"seq: gaps={gaps} (missing {gap_seqs} seqs, {gap_seqs*100/max(1,n+gap_seqs):.1f}%) out_of_order={back}")
print(f"fu: lost_start={fu_lost_start} lost_end={fu_lost_end}")
print(f"frames={frames} ok={ok_frames} bad={bad_frames} ({bad_frames*100/max(1,frames):.0f}% bad) -> ok_fps={ok_frames/el:.1f}")
print("nal types:", dict(sorted(nal_hist.items(), key=lambda x:-x[1])[:8]))
