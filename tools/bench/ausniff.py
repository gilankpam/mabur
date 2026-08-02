#!/usr/bin/env python3
# External gate for the maburgs AU ring (successor to rtpsniff.py): mmaps
# the ring READ-ONLY from outside the daemon and validates what maburgs
# claims to publish. Layout mirrors gs/src/au_ring.h byte-for-byte — change
# both together. Seqlock read: copy, re-check lock word, accept only stable
# even generations; newer-lap records are accepted and counted as resyncs.
#   python3 ausniff.py --ring /dev/shm/mabur-au --seconds 30
#   python3 ausniff.py --ring RING --oneshot --dump-annexb out.bin --json
import argparse, json, mmap, struct, sys, time

HDR = 4096
SLOT_HDR = 64
MAGIC = 0x4D425541
VERSION = 1
FLAG_IDR, FLAG_DISCONT, FLAG_COMPLETE = 0x01, 0x02, 0x04


def read_slot(mm, base, slot_bytes):
    """One seqlock-validated copy attempt. Returns meta dict or None."""
    l1 = struct.unpack_from("<I", mm, base)[0]
    if l1 & 1:
        return None
    ln, rec, fid, pts, sid, flags, codec = struct.unpack_from(
        "<IQQIBBB", mm, base + 4)
    if ln > slot_bytes:
        return None
    payload = bytes(mm[base + SLOT_HDR:base + SLOT_HDR + ln])
    l2 = struct.unpack_from("<I", mm, base)[0]
    if l1 != l2:
        return None
    return {"rec": rec, "len": ln, "fid": fid, "pts": pts, "sid": sid,
            "flags": flags, "codec": codec, "payload": payload}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ring", required=True)
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--oneshot", action="store_true",
                    help="read everything retained right now, then exit")
    ap.add_argument("--dump-annexb")
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args()

    try:
        f = open(a.ring, "rb")
        mm = mmap.mmap(f.fileno(), 0, prot=mmap.PROT_READ)
    except (FileNotFoundError, OSError, IOError) as e:
        sys.exit(f"{a.ring}: {e}")
    magic, ver, slot_bytes, slot_count = struct.unpack_from("<IIII", mm, 0)
    if magic != MAGIC or ver != VERSION:
        sys.exit(f"{a.ring}: bad magic/version {magic:#x}/{ver}")
    if slot_bytes % 64 != 0 or slot_bytes == 0 or slot_count == 0:
        sys.exit(f"{a.ring}: invalid slot_bytes/slot_count {slot_bytes}/{slot_count}")

    def wseq():
        return struct.unpack_from("<Q", mm, 16)[0]

    def slot_base(n):
        return HDR + (n % slot_count) * (SLOT_HDR + slot_bytes)

    w = wseq()
    cursor = w - slot_count if w > slot_count else 0
    aus = 0
    complete = {}
    incomplete = {}
    gaps = 0
    resyncs = 0
    total_bytes = 0
    last_fid = None
    dump = open(a.dump_annexb, "wb") if a.dump_annexb else None
    t0 = time.time()
    deadline = t0 + a.seconds
    first_t = last_t = None

    while True:
        w = wseq()
        if cursor >= w:
            if a.oneshot:
                break
            if time.time() >= deadline:
                break
            time.sleep(0.002)
            continue
        m = read_slot(mm, slot_base(cursor), slot_bytes)
        if m is None:
            time.sleep(0.001)  # writer mid-slot; transient by construction
            continue
        if m["rec"] < cursor:
            time.sleep(0.001)
            continue
        if m["rec"] > cursor:  # overrun: records [cursor, rec) are gone
            resyncs += 1
        cursor = m["rec"] + 1
        aus += 1
        total_bytes += m["len"]
        now = time.time()
        first_t = first_t if first_t is not None else now
        last_t = now
        key = str(m["sid"])
        if m["flags"] & FLAG_COMPLETE:
            complete[key] = complete.get(key, 0) + 1
            if dump:
                dump.write(m["payload"])
        else:
            incomplete[key] = incomplete.get(key, 0) + 1
        if last_fid is not None and m["fid"] > last_fid + 1:
            gaps += m["fid"] - last_fid - 1
        last_fid = m["fid"]
        if not a.oneshot and time.time() >= deadline:
            break

    if dump:
        dump.close()
    dropped = struct.unpack_from("<Q", mm, 24)[0]
    dur = (last_t - first_t) if (first_t is not None and last_t > first_t) else 0.0
    out = {"aus": aus, "complete": complete, "incomplete": incomplete,
           "frame_id_gaps": gaps, "resyncs": resyncs, "bytes": total_bytes,
           "dropped_oversize": dropped,
           "fps": round(aus / dur, 1) if dur > 0 else None}
    if a.json:
        print(json.dumps(out))
    else:
        print(f"aus={aus} complete={complete} incomplete={incomplete} "
              f"fid_gaps={gaps} resyncs={resyncs} bytes={total_bytes} "
              f"dropped_oversize={dropped} fps={out['fps']}")
    sys.exit(0 if aus > 0 else 1)


if __name__ == "__main__":
    main()
