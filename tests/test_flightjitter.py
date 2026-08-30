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
        loaded, offset, resyncs = flightjitter.load_au_log(p)
        assert len(loaded) == 10
        assert resyncs == 1
        assert abs(offset - 5_000_000) < 1000, offset


def main():
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in fns:
        fn()
        print(f"PASS {fn.__name__}")
    print(f"{len(fns)} tests passed")


if __name__ == "__main__":
    main()
