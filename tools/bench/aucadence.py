#!/usr/bin/env python3
# Base-vs-enh AU completion-offset gate for the maburgs AU ring (metric
# defined in docs/airtime-balance-spike-findings-2026-08-29.md; the spike's
# throwaway aucadence.py, mainlined 2026-08-30 after the airtime-balance-uep
# flag day made this a standing acceptance number).
#
# Per-AU completion delay = t_complete_us − pts (SlotHdr v2's writer-stamped
# finish() time; 2026-08-31, was arrival_mono_us from a ~0.5 ms poll of the
# ring writer index before that — see docs/data-provenance.md). Only relative
# delays are meaningful (drone pts and GS monotonic clocks share no epoch),
# so the reported OFFSET is p50(base delays) − p50(enh delays): the clock
# offset cancels in the difference, leaving the systematic, class-dependent
# part of delivery delay. Because base/enh strictly alternate at frame
# cadence, a nonzero offset is a long/short sawtooth in AU inter-arrival —
# the CAUSE of which the sideport jitter_ms EMA is the SYMPTOM. Judge
# balance/transport work on this offset, not the EMA (the EMA also swallows
# encoder frame-size variance; measured repeatability: offset ±0.5 ms, EMA
# ±1.4 ms — pre-t_complete numbers, not yet re-baselined).
#
# IDR-flagged AUs are excluded: they are 2–10x size outliers that belong to
# the IDR pacer / venc size caps, not to the per-class balance (same ruling
# as the drone-side AirBalancer's EWMAs).
#
# A slot with t_complete_us == 0 (pre-epoch writer, or a caller that never
# passed AuLatMeta) falls back to a ~0.5 ms poll of the ring writer index,
# same as before; fallback_rows in the JSON counts how many delays used it,
# and "clock" records which basis produced the reported offset.
#
# Ring layout mirrors gs/src/au_ring.h byte-for-byte, same contract as
# tools/bench/ausniff.py — change all three together. Live-mode caveat also
# as ausniff: Python cannot fence, the seqlock re-check is advisory.
#
#   python3 aucadence.py --ring /dev/shm/mabur-au --seconds 25 --json
#   python3 aucadence.py --ring /dev/shm/mabur-au --seconds 25 --gate-ms 4.0
#
# --gate-ms exits 1 when |offset| exceeds it (scripted acceptance). The
# baselines that make a sensible gate value live in docs/observability.md.
import argparse, json, mmap, statistics, struct, sys, time

HDR = 4096
SLOT_HDR = 64
MAGIC = 0x4D425541
# SlotHdr v2 (kAuRingVersion 2, 2026-08-30 latency-accounting task 6): meta()
# below reads t_complete_us (offset 40) as the completion clock (task 13).
VERSION = 2
FLAG_IDR = 0x01


def open_ring(path):
    f = open(path, "rb")
    mm = mmap.mmap(f.fileno(), 0, prot=mmap.PROT_READ)
    magic, ver, slot_bytes, slot_count = struct.unpack_from("<IIII", mm, 0)
    if magic != MAGIC or ver != VERSION:
        sys.exit(f"{path}: bad magic/version {magic:#x}/{ver}")
    if slot_bytes % 64 != 0 or slot_bytes == 0 or slot_count == 0:
        sys.exit(f"{path}: invalid slot_bytes/slot_count {slot_bytes}/{slot_count}")
    return mm, slot_bytes, slot_count


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ring", required=True)
    ap.add_argument("--seconds", type=float, default=25.0)
    ap.add_argument("--json", action="store_true")
    ap.add_argument("--gate-ms", type=float, default=None,
                    help="exit 1 when |offset_ms| exceeds this")
    ap.add_argument("--min-samples", type=int, default=100,
                    help="per-class floor below which the offset is refused")
    a = ap.parse_args()

    mm, slot_bytes, slot_count = open_ring(a.ring)

    def wseq():
        return struct.unpack_from("<Q", mm, 16)[0]

    def slot_base(n):
        return HDR + (n % slot_count) * (SLOT_HDR + slot_bytes)

    def meta(base):
        l1 = struct.unpack_from("<I", mm, base)[0]
        if l1 & 1:
            return None
        ln, rec, fid, pts, sid, flags, codec = struct.unpack_from(
            "<IQQIBBB", mm, base + 4)
        t_complete = struct.unpack_from("<Q", mm, base + 40)[0]
        l2 = struct.unpack_from("<I", mm, base)[0]
        if l1 != l2:
            return None
        return ln, pts, sid, flags, t_complete

    cursor = wseq()
    delays = {0: [], 1: []}
    lens = {0: [], 1: []}
    idr_excluded = 0
    resyncs = 0
    fallback_rows = 0
    deadline = time.time() + a.seconds
    while time.time() < deadline:
        w = wseq()
        if w == cursor:
            time.sleep(0.0005)
            continue
        now_us = time.monotonic_ns() // 1000
        if w - cursor > slot_count:
            cursor = w - 1
            resyncs += 1
        for rec in range(cursor, w):
            m = meta(slot_base(rec))
            if m is None:
                continue
            ln, pts, sid, flags, t_complete = m
            if sid not in (0, 1):
                continue
            if flags & FLAG_IDR:
                idr_excluded += 1
                continue
            if t_complete:
                delays[sid].append(t_complete - pts)
            else:
                delays[sid].append(now_us - pts)
                fallback_rows += 1
            lens[sid].append(ln)
        cursor = w

    out = {"n_base": len(delays[0]), "n_enh": len(delays[1]),
           "idr_excluded": idr_excluded, "resyncs": resyncs,
           "fallback_rows": fallback_rows, "clock": "t_complete",
           "offset_ms": None,
           "len_p50": {s: statistics.median(lens[s])
                       for s in (0, 1) if lens[s]}}
    ok_samples = all(len(delays[s]) >= a.min_samples for s in (0, 1))
    if ok_samples:
        p50 = {s: statistics.median(delays[s]) for s in (0, 1)}
        out["offset_ms"] = round((p50[0] - p50[1]) / 1000.0, 3)

    if a.json:
        print(json.dumps(out))
    else:
        if out["offset_ms"] is None:
            print(f"aucadence: too few samples "
                  f"(base {out['n_base']}, enh {out['n_enh']}, "
                  f"need {a.min_samples}/class) — no offset")
        else:
            print(f"aucadence: offset {out['offset_ms']:+.3f} ms "
                  f"(p50 base − p50 enh; − = enh late) over "
                  f"{out['n_base']}+{out['n_enh']} AUs, "
                  f"{idr_excluded} IDR excluded")

    if out["offset_ms"] is None:
        sys.exit(1 if a.gate_ms is not None else 0)
    if a.gate_ms is not None and abs(out["offset_ms"]) > a.gate_ms:
        sys.exit(1)


if __name__ == "__main__":
    main()
