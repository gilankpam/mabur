#!/usr/bin/env python3
# External gate for the maburgs AU ring (successor to rtpsniff.py): mmaps
# the ring READ-ONLY from outside the daemon and validates what maburgs
# claims to publish. Layout mirrors gs/src/au_ring.h byte-for-byte — change
# both together (RingHdr offset 32 = u64 epoch, writer boot stamp, nonzero;
# 0 means a pre-epoch ring). Seqlock read: copy, re-check lock word, accept
# only stable even generations; newer-lap records are accepted and counted
# as resyncs.
# Live mode is best-effort on aarch64 (Python cannot issue memory fences;
# the seqlock re-check is advisory); oneshot/post-hoc reads of a quiescent
# ring are exact.
#   python3 ausniff.py --ring /dev/shm/mabur-au --seconds 30
#   python3 ausniff.py --ring RING --oneshot --dump-annexb out.bin --json
import argparse, json, mmap, struct, sys, time

HDR = 4096
SLOT_HDR = 64
MAGIC = 0x4D425541
# SlotHdr v2 (kAuRingVersion 2, 2026-08-30 latency-accounting task 6): the
# reader's version gate just needs to accept the new ring so gs_au_e2e keeps
# passing; the v2 fields themselves (t_first_us/t_complete_us/drone_q_ms/
# enc_us at slot offsets 32/40/48/50) aren't parsed here yet -- that's a
# later task's full mirror update.
VERSION = 2
FLAG_IDR, FLAG_DISCONT, FLAG_COMPLETE = 0x01, 0x02, 0x80


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


def open_ring(path, exit_on_fail=True):
    """Map path read-only and parse/validate the header.

    On failure: exits with an error if exit_on_fail (the default — used for
    the initial open and any oneshot reopen); otherwise closes what it
    opened and returns None so a live-mode caller can retry. A ring
    mid-recreate (writer: ftruncate -> memset -> geometry -> epoch -> magic
    last/release, non-atomic) is routinely unreadable/torn for a live
    reader that happens to poll during that window — that is not fatal.
    """
    def fail(msg, f=None, mm=None):
        if mm is not None:
            mm.close()
        if f is not None:
            f.close()
        if exit_on_fail:
            sys.exit(msg)
        return None

    try:
        f = open(path, "rb")
        mm = mmap.mmap(f.fileno(), 0, prot=mmap.PROT_READ)
    except (FileNotFoundError, OSError, IOError, ValueError) as e:
        return fail(f"{path}: {e}")
    magic, ver, slot_bytes, slot_count = struct.unpack_from("<IIII", mm, 0)
    if magic != MAGIC or ver != VERSION:
        return fail(f"{path}: bad magic/version {magic:#x}/{ver}", f, mm)
    if slot_bytes % 64 != 0 or slot_bytes == 0 or slot_count == 0:
        return fail(f"{path}: invalid slot_bytes/slot_count {slot_bytes}/{slot_count}", f, mm)
    need = HDR + slot_count * (SLOT_HDR + slot_bytes)
    if mm.size() < need:
        return fail(f"{path}: file too short for declared geometry ({mm.size()} < {need})", f, mm)
    epoch = struct.unpack_from("<Q", mm, 32)[0]  # 0 = pre-epoch ring
    return f, mm, slot_bytes, slot_count, epoch


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ring", required=True)
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--oneshot", action="store_true",
                    help="read everything retained right now, then exit")
    ap.add_argument("--dump-annexb")
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args()

    f, mm, slot_bytes, slot_count, epoch = open_ring(a.ring)

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
    stall = 0

    while True:
        if not a.oneshot and time.time() >= deadline:
            break
        cur_epoch = struct.unpack_from("<Q", mm, 32)[0]
        if cur_epoch != epoch:
            # Writer re-created the ring (restart). Mirrors the C++ reader's
            # full reopen: cheapest safe path, re-latches epoch, geometry,
            # and cursor together whether or not geometry actually changed.
            print(f"{a.ring}: writer epoch changed ({epoch:#x} -> "
                  f"{cur_epoch:#x}), resyncing", file=sys.stderr)
            reopened = open_ring(a.ring, exit_on_fail=a.oneshot)
            if reopened is None:
                # Live mode only (oneshot exits inside open_ring above): the
                # header is mid-recreate (unreadable/torn) — routine, not
                # fatal. Retry on the next iteration within --seconds.
                time.sleep(0.01)
                continue
            new_f, new_mm, new_slot_bytes, new_slot_count, new_epoch = reopened
            if a.oneshot and (new_slot_bytes != slot_bytes or new_slot_count != slot_count):
                sys.exit(f"{a.ring}: geometry changed mid oneshot read "
                         f"({slot_bytes}/{slot_count} -> "
                         f"{new_slot_bytes}/{new_slot_count})")
            mm.close()
            f.close()
            f, mm, slot_bytes, slot_count, epoch = (
                new_f, new_mm, new_slot_bytes, new_slot_count, new_epoch)
            w = wseq()
            cursor = w - slot_count if w > slot_count else 0
            resyncs += 1
            last_fid = None  # new writer session: don't diff fid across it
            stall = 0
            continue
        w = wseq()
        if cursor >= w:
            if a.oneshot:
                break
            time.sleep(0.002)
            continue
        m = read_slot(mm, slot_base(cursor), slot_bytes)
        if m is None or m["rec"] < cursor:
            stall += 1
            if stall > 500:
                if a.oneshot:
                    sys.exit(f"{a.ring}: unreadable slot at rec {cursor} "
                             "(torn or stale snapshot)")
                # live ring: mirror the C++ reader — count a resync and
                # skip to the retained tail rather than spinning
                resyncs += 1
                # never rewind past what we already emitted
                cursor = max(cursor, w - slot_count if w > slot_count else 0)
                stall = 0
                continue
            time.sleep(0.001)
            continue
        stall = 0
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
