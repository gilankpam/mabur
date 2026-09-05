#!/usr/bin/env python3
"""Test suite for the GS flight instrument recorder (tools/gs/flightrec.py).

Builds synthetic AU rings byte-for-byte in the gs/src/au_ring.h layout
(mirrored from tools/bench/ausniff.py: 4096 B header, 64 B slot headers,
seqlock word per slot) and drives flightrec's reader against them.
"""
import os
import struct
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools" / "gs"))
import flightrec

HDR = 4096
SLOT_HDR = 64
MAGIC = 0x4D425541


def make_ring(slot_bytes=256, slot_count=8, epoch=0xABCD, version=3):
    buf = bytearray(HDR + slot_count * (SLOT_HDR + slot_bytes))
    struct.pack_into("<IIII", buf, 0, MAGIC, version, slot_bytes, slot_count)
    struct.pack_into("<Q", buf, 32, epoch)
    return buf


def put_slot(buf, slot_bytes, slot_count, rec, payload, fid=None, pts=1000,
             sid=0, flags=0x80, codec=1, lock=None,
             t_first=0, t_complete=0, dq_ms=0, enc_us=0, air_ms=0):
    base = HDR + (rec % slot_count) * (SLOT_HDR + slot_bytes)
    struct.pack_into("<I", buf, base, (rec * 2) if lock is None else lock)
    struct.pack_into("<IQQIBBB", buf, base + 4, len(payload), rec,
                     fid if fid is not None else rec, pts, sid, flags, codec)
    struct.pack_into("<QQ", buf, base + 32, t_first, t_complete)
    struct.pack_into("<HHH", buf, base + 48, dq_ms, enc_us, air_ms)
    buf[base + SLOT_HDR:base + SLOT_HDR + len(payload)] = payload


def set_wseq(buf, w):
    struct.pack_into("<Q", buf, 16, w)


def write_ring(path, buf):
    with open(path, "wb") as f:
        f.write(buf)


H265_VPS = b"\x00\x00\x00\x01\x40\x01"      # nal type 32
H265_IDR = b"\x00\x00\x00\x01\x26\x01"      # nal type 19 (IDR_W_RADL)
H265_TRAIL = b"\x00\x00\x01\x02\x01"        # 3-byte start code, nal type 1


def test_nal0_parsing():
    assert flightrec.nal0_of(H265_VPS) == 32
    assert flightrec.nal0_of(H265_IDR) == 19
    assert flightrec.nal0_of(H265_TRAIL) == 1
    assert flightrec.nal0_of(b"\xff\xff") == -1
    assert flightrec.nal0_of(b"") == -1


def test_reads_metas_from_ring():
    with tempfile.TemporaryDirectory() as d:
        ring = os.path.join(d, "ring")
        buf = make_ring()
        put_slot(buf, 256, 8, 0, H265_VPS, pts=111, sid=0)
        put_slot(buf, 256, 8, 1, H265_TRAIL + b"x" * 40, pts=222, sid=1,
                 t_first=5000, t_complete=5030, dq_ms=4, enc_us=1800,
                 air_ms=17)
        put_slot(buf, 256, 8, 2, H265_IDR + b"y" * 10, pts=333, sid=0)
        set_wseq(buf, 3)
        write_ring(ring, buf)

        r = flightrec.RingReader(ring)
        metas = r.poll()
        assert [m["rec"] for m in metas] == [0, 1, 2]
        assert [m["sid"] for m in metas] == [0, 1, 0]
        assert [m["pts"] for m in metas] == [111, 222, 333]
        assert [m["nal0"] for m in metas] == [32, 1, 19]
        assert metas[1]["len"] == len(H265_TRAIL) + 40
        # SlotHdr v2 latency fields (Task 6/13): unset slots read back zero,
        # a set one round-trips exactly.
        assert (metas[0]["t_first"], metas[0]["t_complete"],
               metas[0]["dq_ms"], metas[0]["enc_us"]) == (0, 0, 0, 0)
        assert (metas[1]["t_first"], metas[1]["t_complete"],
               metas[1]["dq_ms"], metas[1]["enc_us"]) == (5000, 5030, 4, 1800)
        # SlotHdr v3 (Task 9): drone_air_ms round-trips.
        assert metas[1]["air_ms"] == 17
        # nothing new -> empty poll
        assert r.poll() == []
        r.close()


def test_torn_slot_returns_none():
    buf = make_ring()
    put_slot(buf, 256, 8, 0, H265_VPS, lock=1)  # odd lock word = mid-write
    with tempfile.TemporaryDirectory() as d:
        ring = os.path.join(d, "ring")
        set_wseq(buf, 1)
        write_ring(ring, buf)
        r = flightrec.RingReader(ring)
        # torn slot is not delivered and does not crash or spin forever
        assert r.poll() == []
        r.close()


def test_epoch_resync():
    with tempfile.TemporaryDirectory() as d:
        ring = os.path.join(d, "ring")
        buf = make_ring(epoch=0x1)
        put_slot(buf, 256, 8, 0, H265_VPS)
        set_wseq(buf, 1)
        write_ring(ring, buf)
        r = flightrec.RingReader(ring)
        assert len(r.poll()) == 1
        assert r.resyncs == 0
        # writer restarted: new epoch, fresh wseq
        buf2 = make_ring(epoch=0x2)
        put_slot(buf2, 256, 8, 0, H265_IDR, pts=999)
        set_wseq(buf2, 1)
        write_ring(ring, buf2)
        metas = r.poll()
        assert r.resyncs == 1
        assert len(metas) == 1 and metas[0]["pts"] == 999
        r.close()


def test_start_at_head_skips_backlog():
    """The recorder must not emit pre-attach history: those AUs would all be
    stamped with the attach time, fabricating a burst. start_at_head=True
    begins at the write head; only records published after attach flow."""
    with tempfile.TemporaryDirectory() as d:
        ring = os.path.join(d, "ring")
        buf = make_ring()
        for rec in range(3):
            put_slot(buf, 256, 8, rec, H265_VPS)
        set_wseq(buf, 3)
        write_ring(ring, buf)
        r = flightrec.RingReader(ring, start_at_head=True)
        assert r.poll() == []  # backlog skipped
        put_slot(buf, 256, 8, 3, H265_IDR, pts=444)
        set_wseq(buf, 4)
        write_ring(ring, buf)
        metas = r.poll()
        assert len(metas) == 1 and metas[0]["pts"] == 444
        r.close()


def test_pick_index():
    with tempfile.TemporaryDirectory() as d:
        assert flightrec.pick_index(d) == 0
        Path(d, "flight-0003.jsonl").touch()
        assert flightrec.pick_index(d) == 4
        Path(d, "au-0007.log").touch()
        assert flightrec.pick_index(d) == 8
        Path(d, "au-junk.log").touch()  # non-numeric ignored
        assert flightrec.pick_index(d) == 8


def test_row_format():
    """v3 row (Task 9): the 5 SlotHdr columns ride behind nal0, in
    t_first/t_complete/enc_us/dq_ms/air_ms order — flightjitter's
    marker-keyed parser depends on this exact order."""
    m = {"pts": 42, "sid": 1, "fid": 7, "len": 1234, "flags": 0x80,
         "nal0": 19, "t_first": 900000, "t_complete": 900450,
         "enc_us": 2200, "dq_ms": 6, "air_ms": 31}
    row = flightrec.format_row(1755000000123456, m)
    (t, pts, sid, fid, ln, flags, nal0, t_first, t_complete, enc_us, dq_ms,
     air_ms) = row.split()
    assert (int(t), int(pts), int(sid), int(fid), int(ln)) == (
        1755000000123456, 42, 1, 7, 1234)
    assert int(flags, 0) == 0x80 and int(nal0) == 19
    assert (int(t_first), int(t_complete), int(enc_us), int(dq_ms)) == (
        900000, 900450, 2200, 6)
    assert int(air_ms) == 31


def test_version_mismatch_refused():
    """open_ring (via RingReader) refuses a v1 or v2 ring header, matching
    the C++ readers' version gate — the tool must not silently misparse an
    old-layout ring as v3."""
    for version in (1, 2):
        with tempfile.TemporaryDirectory() as d:
            ring = os.path.join(d, "ring")
            buf = make_ring(version=version)
            put_slot(buf, 256, 8, 0, H265_VPS)
            set_wseq(buf, 1)
            write_ring(ring, buf)
            try:
                flightrec.RingReader(ring)
                assert False, "expected FileNotFoundError on version mismatch"
            except FileNotFoundError:
                pass


def test_au_log_marker_written():
    """AuLog.__init__ writes '# aulog 3' as the session's first line so
    flightjitter can key its row-format parsing on it."""
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "au-0000.log")
        log = flightrec.AuLog(path)
        log.write_line(flightrec.format_row(1000, {
            "pts": 1, "sid": 0, "fid": 0, "len": 10, "flags": 0x80,
            "nal0": 1, "t_first": 0, "t_complete": 0, "enc_us": 0,
            "dq_ms": 0, "air_ms": 0}))
        log.flush()
        with open(path) as f:
            lines = f.read().splitlines()
        assert lines[0] == "# aulog 3"


def test_extract_t_ms():
    dg = b'{"v":1,"t_ms":123456,"link":{"video":{"jitter_ms":5.0}}}'
    assert flightrec.extract_t_ms(dg) == 123456
    assert flightrec.extract_t_ms(b'{"v":1}') is None
    assert flightrec.extract_t_ms(b"not json at all") is None
    # ONLY the top-level t_ms counts: the real datagram serializes nested,
    # frozen timestamps (ctl.last_event.t_ms) BEFORE the
    # top-level key — a byte-order regex returns the frozen one (live bug,
    # session 0024: every sync line carried last_event's 2722491).
    dg = (b'{"link":{"ctl":{"last_event":{"t_ms":2722491.0}}},'
          b' "t_ms": 3014113}')
    assert flightrec.extract_t_ms(dg) == 3014113
    # non-integer top-level t_ms is truncated, not rejected
    assert flightrec.extract_t_ms(b'{"t_ms": 99.7}') == 99


def main():
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in fns:
        fn()
        print(f"PASS {fn.__name__}")
    print(f"{len(fns)} tests passed")


if __name__ == "__main__":
    main()
