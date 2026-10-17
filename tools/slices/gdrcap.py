#!/usr/bin/env python3
"""Capture the GS AU ring for N seconds with per-AU meta kept.

`ausniff --dump-annexb` concatenates payloads and keeps nothing per AU;
the encoder spikes (gdrheal.py) need the AU boundaries, the stream id
(base/enhance) and the ring's slice count. Writes OUT.bin (every complete
AU's payload, concatenated, in ring order) and OUT.idx, one line per AU:

    rec fid pts sid len flags nslices t_complete off dq_ms enc_us air_ms

`off` is the payload's byte offset in OUT.bin, -1 for an AU that was not
complete (not written); dq_ms / enc_us / air_ms are the drone's stamps
carried in the AU meta (encode time in enc_us). Runs on the GS (BusyBox python3, stdlib only);
ring layout = tools/bench/ausniff.py (AU ring v4). Starts at the ring head,
so a live capture never includes retained history.

    scp -O tools/slices/gdrcap.py $GS:/tmp/
    ssh $GS python3 /tmp/gdrcap.py /dev/shm/mabur-au 6 /tmp/cap
    scp -O "$GS:/tmp/cap.bin" "$GS:/tmp/cap.idx" .
"""
import mmap
import struct
import sys
import time

HDR, SLOT_HDR, MAGIC, VERSION = 4096, 64, 0x4D425541, 4
FLAG_COMPLETE = 0x80
STATE_CLOSED, STATE_OPEN, STATE_ABORTED = 0, 1, 2


def read_slot(mm, base, slot_bytes):
    """One seqlock-validated copy; None = torn/mid-write, retry."""
    l1 = struct.unpack_from("<I", mm, base)[0]
    if l1 & 1:
        return None
    ln, rec, fid, pts, sid, flags = struct.unpack_from("<IQQIBB", mm, base + 4)
    t_first, t_complete = struct.unpack_from("<QQ", mm, base + 32)
    dq_ms, enc_us, air_ms = struct.unpack_from("<HHH", mm, base + 48)
    state, nslices = struct.unpack_from("<BB", mm, base + 54)
    if ln > slot_bytes:
        return None
    payload = bytes(mm[base + SLOT_HDR:base + SLOT_HDR + ln])
    if struct.unpack_from("<I", mm, base)[0] != l1:
        return None
    if state == STATE_ABORTED:
        return {"aborted": True}
    if state != STATE_CLOSED:
        return None
    return dict(rec=rec, len=ln, fid=fid, pts=pts, sid=sid, flags=flags,
                nslices=nslices, payload=payload, t_complete=t_complete,
                dq_ms=dq_ms, enc_us=enc_us, air_ms=air_ms)


def main():
    if len(sys.argv) != 4:
        sys.exit("usage: gdrcap.py RING SECONDS OUT")
    ring, seconds, out = sys.argv[1], float(sys.argv[2]), sys.argv[3]
    f = open(ring, "rb")
    mm = mmap.mmap(f.fileno(), 0, prot=mmap.PROT_READ)
    magic, ver, slot_bytes, slot_count = struct.unpack_from("<IIII", mm, 0)
    if magic != MAGIC or ver != VERSION:
        sys.exit("%s: bad magic/version %#x/%d" % (ring, magic, ver))

    def wseq():
        return struct.unpack_from("<Q", mm, 16)[0]

    cursor = wseq()
    binf = open(out + ".bin", "wb")
    idx = open(out + ".idx", "w")
    off = 0
    n = 0
    deadline = time.time() + seconds
    while time.time() < deadline:
        w = wseq()
        if cursor >= w:
            time.sleep(0.002)
            continue
        base = HDR + (cursor % slot_count) * (SLOT_HDR + slot_bytes)
        m = read_slot(mm, base, slot_bytes)
        if m is None:
            time.sleep(0.001)
            continue
        if m.get("aborted") or m["rec"] < cursor:
            cursor += 1
            continue
        cursor = m["rec"] + 1
        o = -1
        if m["flags"] & FLAG_COMPLETE:
            o = off
            binf.write(m["payload"])
            off += m["len"]
        idx.write("%d %d %d %d %d %d %d %d %d %d %d %d\n" % (
            m["rec"], m["fid"], m["pts"], m["sid"], m["len"], m["flags"],
            m["nslices"], m["t_complete"], o, m["dq_ms"], m["enc_us"], m["air_ms"]))
        n += 1
    binf.close()
    idx.close()
    print("aus=%d bytes=%d" % (n, off))


if __name__ == "__main__":
    main()
