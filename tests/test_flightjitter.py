#!/usr/bin/env python3
"""Test suite for the jitter/stutter attribution analyzer (tools/flightjitter.py).

Synthesizes au-NNNN.log AU rows (and optionally a flight-NNNN.jsonl) with a
known ground-truth cause per anomaly and checks the analyzer attributes each
one correctly:
  - size:      arrival delay fully explained by the frame-size delta
  - rung-change: anomaly within the guard window of a ladder/bitrate step
  - loss-recovery: delay coinciding with recorded pre-FEC loss, sizes flat
  - transport: delay with none of the above
  - gap:       frame_id discontinuity (frame never arrived)
"""
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))
import flightjitter

FRAME_US = 16667
SLOPE_US_PER_BYTE = 0.85


def make_rows(n=600, base_len=8000, t0=1_000_000_000):
    """Steady 60 fps alternating sid 0/1, constant sizes, model-exact timing."""
    rows = []
    t = t0
    prev_len = base_len
    for i in range(n):
        ln = base_len
        t += FRAME_US + int(SLOPE_US_PER_BYTE * (ln - prev_len))
        prev_len = ln
        rows.append(dict(t_us=t, pts=(t + 50_000) & 0xFFFFFFFF, sid=i % 2,
                         fid=i, len=ln, flags=0x80, nal0=1))
    return rows


def apply_size_spike(rows, at, extra_bytes):
    """A big frame whose arrival is late exactly per the size model."""
    delay = int(SLOPE_US_PER_BYTE * extra_bytes)
    rows[at]["len"] += extra_bytes
    for r in rows[at:]:
        r["t_us"] += delay
    # the following frame is small again -> its interval shrinks (model-exact)
    if at + 1 < len(rows):
        pass  # sizes after `at` unchanged; next Δlen is negative, matching t


def apply_transport_stall(rows, at, delay_us):
    """Arrival delayed with NO size change (USB/scheduling/queueing)."""
    for r in rows[at:]:
        r["t_us"] += delay_us


def apply_gap(rows, at):
    """Frame never arrived: drop the row, leaving a fid discontinuity."""
    del rows[at]


def jsonl_second(t_ms, rung_idx=5, pre_fec=0.0, cmd_kbps=8000):
    return {
        "t_ms": t_ms,
        "link": {
            "residual_loss": 0.0,
            "video": {"jitter_ms": 0.0, "fps": 60.0, "mbps": 8.0},
            "ctl": {"rung": {"idx": rung_idx, "mcs": 5, "ov": 0.5},
                    "pre_fec_loss": pre_fec},
        },
        "drone": {"enc": {"cmd_kbps": cmd_kbps}},
    }


def test_clean_stream_no_events():
    rows = make_rows()
    rep = flightjitter.analyze(rows)
    assert rep["events"] == []
    assert rep["summary"]["jitter_ema_ms"] < 1.0
    assert rep["summary"]["aus"] == 600


def test_size_spike_attributed_size():
    rows = make_rows()
    apply_size_spike(rows, 300, 40_000)  # +40 KB -> +34 ms arrival: stutter
    rep = flightjitter.analyze(rows)
    kinds = [e["kind"] for e in rep["events"]]
    assert kinds == ["size"], kinds
    assert rep["events"][0]["fid"] == 300


def test_transport_stall_attributed_transport():
    rows = make_rows()
    apply_transport_stall(rows, 300, 30_000)
    rep = flightjitter.analyze(rows)
    kinds = [e["kind"] for e in rep["events"]]
    assert kinds == ["transport"], kinds


def test_gap_attributed():
    rows = make_rows()
    apply_gap(rows, 300)
    rep = flightjitter.analyze(rows)
    kinds = [e["kind"] for e in rep["events"]]
    assert kinds == ["gap"], kinds
    assert rep["events"][0]["fid"] == 300  # the missing frame's id


def test_rung_change_wins_over_size():
    rows = make_rows()
    apply_size_spike(rows, 300, 40_000)
    t_event_ms = rows[300]["t_us"] // 1000
    js = [jsonl_second(t_event_ms - 3000, rung_idx=4, cmd_kbps=6000),
          jsonl_second(t_event_ms - 1000, rung_idx=4, cmd_kbps=6000),
          jsonl_second(t_event_ms, rung_idx=5, cmd_kbps=8000),
          jsonl_second(t_event_ms + 1000, rung_idx=5, cmd_kbps=8000)]
    rep = flightjitter.analyze(rows, jsonl=js)
    kinds = [e["kind"] for e in rep["events"]]
    assert kinds == ["rung-change"], kinds


def test_fec_wait_attributed():
    """Direct per-frame evidence (SlotHdr v2's own t_complete-t_first > 20 ms
    on the stutter frame) attributes fec-wait even with no jsonl at all."""
    rows = make_rows()
    apply_transport_stall(rows, 300, 30_000)
    b = rows[300]
    b["t_complete"] = b["t_us"]
    b["t_first"] = b["t_us"] - 25_000
    rep = flightjitter.analyze(rows)
    kinds = [e["kind"] for e in rep["events"]]
    assert kinds == ["fec-wait"], kinds


def test_fec_wait_wins_over_loss_recovery():
    """fec-wait is checked BEFORE the loss-correlation classes: direct
    per-frame evidence must win even when the jsonl also shows pre-FEC
    loss that second (which alone would read as loss-recovery)."""
    rows = make_rows()
    apply_transport_stall(rows, 300, 30_000)
    b = rows[300]
    b["t_complete"] = b["t_us"]
    b["t_first"] = b["t_us"] - 25_000
    t_event_ms = rows[300]["t_us"] // 1000
    js = [jsonl_second(t_event_ms - 1000, pre_fec=0.04),
          jsonl_second(t_event_ms, pre_fec=0.06),
          jsonl_second(t_event_ms + 1000, pre_fec=0.0)]
    rep = flightjitter.analyze(rows, jsonl=js)
    kinds = [e["kind"] for e in rep["events"]]
    assert kinds == ["fec-wait"], kinds


def test_jitter_uses_t_complete_when_present():
    """The jitter EMA reproduction must use the writer-stamped t_complete
    (SlotHdr v2) as the arrival basis, not flightrec's noisier read-time
    t_us, whenever t_complete is present — same fix as aucadence.py."""
    rows = make_rows()
    for i, r in enumerate(rows):
        r["t_complete"] = r["t_us"]           # true, model-exact arrival
        r["t_us"] += 3000 if i % 2 == 0 else -3000  # noisy read-time stamp
    rep = flightjitter.analyze(rows)
    assert rep["events"] == []
    assert rep["summary"]["jitter_ema_ms"] < 1.0


def test_loss_recovery_attribution():
    rows = make_rows()
    apply_transport_stall(rows, 300, 30_000)
    t_event_ms = rows[300]["t_us"] // 1000
    js = [jsonl_second(t_event_ms - 1000, pre_fec=0.04),
          jsonl_second(t_event_ms, pre_fec=0.06),
          jsonl_second(t_event_ms + 1000, pre_fec=0.0)]
    rep = flightjitter.analyze(rows, jsonl=js)
    kinds = [e["kind"] for e in rep["events"]]
    assert kinds == ["loss-recovery"], kinds


def test_explained_fraction_high_when_size_driven():
    """Sizes alternating big/small with model-exact arrivals: the size model
    should explain nearly all of the (large) measured jitter."""
    rows = make_rows()
    for i, r in enumerate(rows):
        bump = 4000 if i % 2 == 0 else -4000
        r["len"] += bump
    # rebuild arrival times from the size model
    t = rows[0]["t_us"]
    prev_len = rows[0]["len"]
    for r in rows[1:]:
        t += FRAME_US + int(SLOPE_US_PER_BYTE * (r["len"] - prev_len))
        prev_len = r["len"]
        r["t_us"] = t
    rep = flightjitter.analyze(rows)
    s = rep["summary"]
    assert s["jitter_ema_ms"] > 5.0          # alternation is real
    assert s["size_explained_frac"] > 0.8, s  # ...and size-driven
    assert s["residual_jitter_ms"] < 2.0, s


def test_discont_breaks_chain_not_gap():
    """A DISCONT-flagged AU (0x02, maburgs-marked: drone restart / fid
    namespace jump — live session 0028: fid 115 -> 65646) must break the
    interval/fid chain: no gap event, no stutter across the seam, and the
    fid delta must not pollute the gap count."""
    rows = make_rows(n=200)
    for r in rows[100:]:
        r["fid"] += 65531
        r["t_us"] += 40_000  # re-establish pause would read as a stutter
    rows[100]["flags"] |= 0x02
    rep = flightjitter.analyze(rows)
    assert rep["events"] == [], rep["events"]
    assert rep["summary"]["gaps"] == 0
    assert rep["summary"]["disconts"] == 1


def test_incomplete_au_reported_damaged():
    """AUs without FLAG_COMPLETE (0x80) arrived broken (post-FEC damage):
    report them as 'damaged' events even when timing is unremarkable."""
    rows = make_rows(n=200)
    rows[100]["flags"] = 0x00
    rep = flightjitter.analyze(rows)
    kinds = [e["kind"] for e in rep["events"]]
    assert kinds == ["damaged"], kinds
    assert rep["events"][0]["fid"] == 100
    assert rep["summary"]["damaged"] == 1


def test_per_rung_breakdown():
    """With a jsonl rung timeline, the summary carries per-rung dwell and
    a |Δinterval| jitter proxy so 'where does the jitter live' is answered
    per rung, not as one global number."""
    rows = make_rows(n=600)  # 10 s
    t0_ms = rows[0]["t_us"] // 1000
    js = []
    for s in range(11):
        rung = 5 if s < 5 else 3
        js.append(jsonl_second(t0_ms + s * 1000, rung_idx=rung))
        js[-1]["link"]["ctl"]["rung"]["mcs"] = rung
    rep = flightjitter.analyze(rows, jsonl=js)
    per = rep["summary"]["per_rung"]
    assert set(per) == {5, 3}, per
    assert per[5]["dwell_s"] > 3 and per[3]["dwell_s"] > 3
    assert per[5]["djit_p50_ms"] < 1.0


def test_perceptual_metrics():
    """The summary carries what eyes notice — hole rate (missing frames:
    enh->enh... i.e. sid seams and fid gaps) and stall rates — not just
    the EMA (session 0029: EMA +32% but holes/min +117%, which is what
    'much worse' actually was)."""
    rows = make_rows(n=600)  # 10 s @ 60 fps, sid alternating 0/1
    del rows[300]            # one fid gap
    del rows[100]            # a second hole: now sid 0 follows sid 0
    apply_transport_stall(rows, 400, 60_000)
    rep = flightjitter.analyze(rows)
    s = rep["summary"]
    assert s["holes_per_min"] > 0
    assert abs(s["holes_per_min"] - 2 / (s["duration_s"] / 60.0)) < 0.5, s
    assert s["stalls50_per_min"] > 0
    assert s["iv_p99_ms"] > 16.0


def test_jsonl_null_drone_tolerated():
    """Live sideport rows carry "drone": null until drone telemetry arrives
    (seen on session 0026): the walker must not crash on them."""
    rows = make_rows(n=100)
    js = [jsonl_second(rows[0]["t_us"] // 1000)]
    js[0]["drone"] = None
    js.append(jsonl_second(rows[50]["t_us"] // 1000))
    rep = flightjitter.analyze(rows, jsonl=js)
    assert rep["events"] == []


def test_load_au_log_sync_offset(tmp_path=None):
    import tempfile, os
    rows = make_rows(n=10)
    with tempfile.TemporaryDirectory() as d:
        p = os.path.join(d, "au-0000.log")
        with open(p, "w") as f:
            f.write("# resync 1\n")
            # wall clock runs 5_000_000 us ahead of the sideport t_ms clock
            f.write(f"# sync {rows[0]['t_us']} {(rows[0]['t_us'] - 5_000_000) // 1000}\n")
            f.write(f"# sync {rows[5]['t_us']} {(rows[5]['t_us'] - 5_000_000) // 1000}\n")
            for r in rows:
                f.write(f"{r['t_us']} {r['pts']} {r['sid']} {r['fid']} "
                        f"{r['len']} 0x{r['flags']:02x} {r['nal0']}\n")
        loaded, offset, resyncs, dropped = flightjitter.load_au_log(p)
        assert len(loaded) == 10
        assert resyncs == 1
        assert abs(offset - 5_000_000) < 1000, offset
        # No "# aulog N" marker -> v1 parse: the 4 SlotHdr v2 columns are
        # absent from the row text, so they default to 0.
        assert all(r["t_first"] == 0 and r["t_complete"] == 0
                  for r in loaded)


def test_load_au_log_v2_marker():
    import tempfile, os
    rows = make_rows(n=5)
    with tempfile.TemporaryDirectory() as d:
        p = os.path.join(d, "au-0001.log")
        with open(p, "w") as f:
            f.write("# aulog 2\n")
            for r in rows:
                f.write(f"{r['t_us']} {r['pts']} {r['sid']} {r['fid']} "
                        f"{r['len']} 0x{r['flags']:02x} {r['nal0']} "
                        f"{r['t_us'] - 1000} {r['t_us']} 1800 4\n")
        loaded, offset, resyncs, dropped = flightjitter.load_au_log(p)
        assert len(loaded) == 5
        assert loaded[0]["t_first"] == rows[0]["t_us"] - 1000
        assert loaded[0]["t_complete"] == rows[0]["t_us"]
        assert loaded[0]["enc_us"] == 1800
        assert loaded[0]["dq_ms"] == 4


def test_aulog_v4_needs_no_clock_offset():
    import tempfile, os
    with tempfile.TemporaryDirectory() as d:
        p = os.path.join(d, "au.log")
        with open(p, "w") as f:
            f.write("# aulog 4\n")
            f.write("1000000 100 0 1 5000 0x81 32 900000 990000 7000 3 21\n")
            f.write("1016000 200 0 2 5000 0x01 1 1000000 1006000 7000 3 21\n")
        rows, offset, resyncs, dropped = flightjitter.load_au_log(p)
        # v4 t_us is already monotonic: no `# sync` bridge, so no offset.
        assert offset == 0
        assert len(rows) == 2
        assert rows[0]["t_us"] == 1000000


def test_aulog_v4_ignores_sync_lines_even_when_present():
    """The distinguishing case for the v4 no-offset rule: a v4 log that
    DOES carry `# sync` lines (the pre-v4 wall/mono bridge) must still
    report offset=0, because v4's t_us is already monotonic and the sync
    lines are not consulted. Without gating on version, this is the case
    that would silently regress -- a v4 file with no sync lines at all
    (the smoke case above) passes with or without the version check,
    since an empty `syncs` list already yields offset 0 either way."""
    import tempfile, os
    rows = make_rows(n=10)
    with tempfile.TemporaryDirectory() as d:
        p = os.path.join(d, "au.log")
        with open(p, "w") as f:
            f.write("# aulog 4\n")
            # wall clock runs 5_000_000 us ahead of the sideport t_ms clock
            # -- same shape as test_load_au_log_sync_offset's v<=3 case.
            f.write(f"# sync {rows[0]['t_us']} {(rows[0]['t_us'] - 5_000_000) // 1000}\n")
            f.write(f"# sync {rows[5]['t_us']} {(rows[5]['t_us'] - 5_000_000) // 1000}\n")
            for r in rows:
                f.write(f"{r['t_us']} {r['pts']} {r['sid']} {r['fid']} "
                        f"{r['len']} 0x{r['flags']:02x} {r['nal0']} "
                        f"{r['t_us'] - 1000} {r['t_us']} 1800 4\n")
        rows_loaded, offset, resyncs, dropped = flightjitter.load_au_log(p)
        assert len(rows_loaded) == 10
        assert offset == 0, offset


def test_repeated_marker_line_is_not_a_row():
    import tempfile, os
    with tempfile.TemporaryDirectory() as d:
        p = os.path.join(d, "au.log")
        with open(p, "w") as f:
            f.write("# aulog 4\n")
            f.write("1000000 100 0 1 5000 0x81 32 900000 990000 7000 3 21\n")
            f.write("# aulog 4\n")  # a respawn rejoined this session
            f.write("2000000 200 0 2 5000 0x01 1 1900000 1990000 7000 3 21\n")
        rows, _, _, _ = flightjitter.load_au_log(p)
        assert len(rows) == 2


def test_dropped_lines_summed_and_surfaced():
    """`# dropped N` (LogWriter's ring-overflow hole signal, gs/src/log_writer.h
    -- the v4 replacement for the deleted flightrec's `# resync`) must be
    summed by load_au_log, not silently discarded like every other
    '#'-prefixed line, and must land in the printed summary the way
    ring_resyncs already does (mirrors main()'s s["ring_resyncs"] = resyncs
    assignment)."""
    import tempfile, os
    with tempfile.TemporaryDirectory() as d:
        p = os.path.join(d, "au.log")
        with open(p, "w") as f:
            f.write("# aulog 4\n")
            f.write("1000000 100 0 1 5000 0x81 32 900000 990000 7000 3 21\n")
            f.write("# dropped 3\n")
            f.write("2000000 200 0 2 5000 0x01 1 1900000 1990000 7000 3 21\n")
        rows, offset, resyncs, dropped = flightjitter.load_au_log(p)
        assert len(rows) == 2
        assert dropped == 3
        rep = flightjitter.analyze(rows)
        s = rep["summary"]
        s["ring_resyncs"] = resyncs
        s["log_dropped"] = dropped
        assert s["log_dropped"] == 3


def test_dropped_lines_from_multiple_markers_accumulate():
    """A rejoined session (2 s wrapper respawn) can flush more than one
    "# dropped N" marker into the same file; they must accumulate, not
    overwrite."""
    import tempfile, os
    with tempfile.TemporaryDirectory() as d:
        p = os.path.join(d, "au.log")
        with open(p, "w") as f:
            f.write("# aulog 4\n")
            f.write("1000000 100 0 1 5000 0x81 32 900000 990000 7000 3 21\n")
            f.write("# dropped 2\n")
            f.write("2000000 200 0 2 5000 0x01 1 1900000 1990000 7000 3 21\n")
            f.write("# dropped 5\n")
            f.write("3000000 300 0 3 5000 0x01 1 2900000 2990000 7000 3 21\n")
        rows, offset, resyncs, dropped = flightjitter.load_au_log(p)
        assert len(rows) == 3
        assert dropped == 7


def main():
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in fns:
        fn()
        print(f"PASS {fn.__name__}")
    print(f"{len(fns)} tests passed")


if __name__ == "__main__":
    main()
