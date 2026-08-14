#!/usr/bin/env python3
"""Test suite for flightreport.py post-flight analysis tool."""
import contextlib
import io
import json
import math
import os
import re
import subprocess
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))
import flightreport


def synthesize_flight_jsonl():
    """Generate a regression test fixture at 500ms cadence (2 Hz sideport).

    Regression cases:
    1. Two-burst case: burst1 at t=0/500, gap=1500ms, burst2 at t=2000/2500
       - Old anchor logic (2500ms from episode-start): wrongly merges → 1 episode
       - New anchor logic (750ms from previous-sample): correctly splits → 2 episodes

    2. Continuous residual run: 11 samples (5.5s continuous, 500ms apart)
       - Old anchor logic: splits at t=2500 and t=5000 → 2-3 episodes
       - New anchor logic: no clean samples, so 1 episode
    """
    rows = []

    def make_datagram(t, rung_idx, util, residual_loss=0.0, event=None, drone_state="linked"):
        dg = {
            "v": 1,
            "t_ms": t,
            "link": {
                "state": drone_state,
                "residual_loss": residual_loss if residual_loss > 0 else None,
                "ctl": {
                    "rung": {"idx": rung_idx, "mcs": 5 - rung_idx, "ov": 0.25},
                    "util": util,
                    "pre_fec_loss": 0.01,
                    "budget": 0.5,
                    "probation_ms_left": 0,
                    "penalized": [],
                    "counters": {
                        "demotes_residual": 0,
                        "demotes_util": 0,
                        "promotes": 0,
                        "probation_fails": 0,
                        "starved_drops": 0,
                        "timeout_drops": 0
                    },
                    "last_event": event or {
                        "t_ms": 0,
                        "from": 0,
                        "to": 0,
                        "reason": "none",
                        "u": 0.0
                    }
                }
            },
            "cards": [
                {
                    "frames": 1000,
                    "classes": {
                        "s1": {
                            "rssi": -50.0,
                            "snr": 27.0,
                            "pps": 900
                        },
                        "ctrl": {
                            "rssi": -47.2,
                            "snr": 25.0
                        }
                    }
                }
            ],
            "drone": {
                "state": drone_state,
                "tlm_age_ms": 100,
                "enc": {"fps": 59.9},
                "uplink": {"rssi_b": -57.95}
            }
        }
        return json.dumps(dg)

    t_ms = 0

    # Phase 1: Steady rung 5 (30s at 500ms = 60 samples)
    for i in range(60):
        rows.append(make_datagram(t_ms, rung_idx=5, util=0.08 + 0.02 * (i % 2)))
        t_ms += 500

    # Phase 2: Util demote (rung 5 -> 4)
    event = {
        "t_ms": t_ms,
        "from": 5,
        "to": 4,
        "reason": "util",
        "u": 0.63
    }
    rows.append(make_datagram(t_ms, rung_idx=4, util=0.63, event=event))
    t_ms += 500

    # Phase 3: Steady rung 4 (5s at 500ms = 10 samples)
    for i in range(10):
        rows.append(make_datagram(t_ms, rung_idx=4, util=0.15 + 0.05 * (i % 2)))
        t_ms += 500

    # ===== REGRESSION TEST CASE 1: Two-burst case =====
    # Burst 1 at t=0/500 (relative to start of bursts)
    burst1_start = t_ms
    rows.append(make_datagram(t_ms, rung_idx=4, util=0.20, residual_loss=0.02))
    t_ms += 500
    rows.append(make_datagram(t_ms, rung_idx=4, util=0.20, residual_loss=0.02))
    t_ms += 500

    # Clean gap: 3 clean samples at 500ms each = 1500ms gap
    # (>750ms threshold for new logic, but <2500ms for old logic)
    for _ in range(3):
        rows.append(make_datagram(t_ms, rung_idx=4, util=0.12))
        t_ms += 500

    # Burst 2 at ~2000ms from burst1_start
    # Old anchor logic: 2000ms - burst1_start < 2500ms → merged into 1 episode
    # New anchor logic: 2000ms - last_sample (500+1500=2000ms ago) = 500ms < 750ms → continue
    # Wait, that's still continuous. Let me recalculate...

    # Actually, if burst1 is at t_rel 0 and 500, and gap is 3 samples (1500ms),
    # then burst2 starts at t_rel 2000.
    # For new logic: gap between last sample of burst1 (t_rel 500) and first of burst2 (t_rel 2000)
    # is 1500ms > 750ms → NEW EPISODE
    # For old logic: gap between burst1_start (t_rel 0) and first of burst2 (t_rel 2000)
    # is 2000ms < 2500ms → SAME EPISODE

    rows.append(make_datagram(t_ms, rung_idx=4, util=0.22, residual_loss=0.03))
    t_ms += 500
    rows.append(make_datagram(t_ms, rung_idx=4, util=0.22, residual_loss=0.03))
    t_ms += 500

    # Clean gap before continuous case: 2 clean samples (1000ms)
    for _ in range(2):
        rows.append(make_datagram(t_ms, rung_idx=4, util=0.12))
        t_ms += 500

    # ===== REGRESSION TEST CASE 2: Continuous residual run =====
    # 11 consecutive samples with residual, no clean samples between
    # Old logic: splits at t=2500 and t=5000 from start → 2-3 episodes
    # New logic: no gaps, all consecutive → 1 episode
    continuous_start = t_ms
    for i in range(11):
        rows.append(make_datagram(t_ms, rung_idx=4, util=0.25, residual_loss=0.04))
        t_ms += 500

    # Phase 7: Demote event (optional, after residual cases)
    event = {
        "t_ms": t_ms,
        "from": 4,
        "to": 3,
        "reason": "residual",
        "u": 0.30
    }
    rows.append(make_datagram(t_ms, rung_idx=3, util=0.30, event=event))
    t_ms += 500

    # Phase 8: Promote event (optional)
    event = {
        "t_ms": t_ms,
        "from": 3,
        "to": 4,
        "reason": "promote",
        "u": 0.25
    }
    rows.append(make_datagram(t_ms, rung_idx=4, util=0.25, event=event))
    t_ms += 500

    # Phase 9: Final samples at rung 4
    for i in range(3):
        rows.append(make_datagram(t_ms, rung_idx=4, util=0.12))
        t_ms += 500

    return "\n".join(rows) + "\n"


def test_flightreport_structure():
    """Run flightreport.py and verify output structure."""
    # Create fixture directory
    fixture_dir = Path("tests/fixtures")
    fixture_dir.mkdir(exist_ok=True)

    # Write synthetic flight.jsonl
    flight_jsonl = fixture_dir / "flight-ladder-fixture.jsonl"
    flight_jsonl.write_text(synthesize_flight_jsonl())

    # Run flightreport.py
    result = subprocess.run(
        [sys.executable, "tools/flightreport.py", str(flight_jsonl)],
        capture_output=True,
        text=True
    )

    if result.returncode != 0:
        print("STDERR:", result.stderr)
        print("STDOUT:", result.stdout)
        raise AssertionError(f"flightreport.py exited with code {result.returncode}")

    output = result.stdout
    print("\n=== flightreport.py output ===\n", output)

    # Verify structure: TRANSITIONS section
    assert "TRANSITIONS" in output, "Missing TRANSITIONS section"
    trans_section = output[output.find("TRANSITIONS"):output.find("TIME IN RUNG")]
    assert "reason=util u=0.63" in trans_section, "Missing util demote transition"
    assert "reason=residual" in trans_section, "Missing residual demote transition"
    assert "reason=promote" in trans_section, "Missing promote transition"

    # Verify TIME IN RUNG section
    assert "TIME IN RUNG" in output, "Missing TIME IN RUNG section"
    time_section = output[output.find("TIME IN RUNG"):output.find("U PER RUNG")]
    assert "rung 5:" in time_section, "Missing rung 5 in TIME IN RUNG"
    assert "rung 4:" in time_section, "Missing rung 4 in TIME IN RUNG"
    assert "rung 3:" in time_section, "Missing rung 3 in TIME IN RUNG"

    # Verify U PER RUNG section with p50/p95
    assert "U PER RUNG" in output, "Missing U PER RUNG section"
    u_section = output[output.find("U PER RUNG"):output.find("RESIDUAL")]
    assert "rung 5:" in u_section, "Missing rung 5 stats"
    assert "rung 4:" in u_section, "Missing rung 4 stats"
    # Should have p50/p95/max pattern: "rung X: 0.XX/0.XX/0.XX  n=N"
    assert "/" in u_section, "Missing p50/p95/max format"

    # Verify RESIDUAL EPISODES section: 3 episodes (regression test cases)
    # Case 1: burst1 (0.02) + gap + burst2 (0.03) → 2 separate episodes (gap > 750ms from last sample)
    # Case 2: continuous run (0.04) → 1 episode (all consecutive, no gap)
    assert "RESIDUAL EPISODES: 3" in output, "Should have exactly 3 residual episodes (2 separated bursts + 1 continuous)"

    # Verify each episode has expected residual values
    residual_section = output[output.find("RESIDUAL EPISODES"):]
    assert "residual=0.0200" in residual_section, "Missing burst 1 (0.02)"
    assert "residual=0.0300" in residual_section, "Missing burst 2 (0.03)"
    assert "residual=0.0400" in residual_section, "Missing continuous run (0.04)"

    # Verify drone state context join
    assert "drone_state=linked" in output, "Missing drone state context in output"

    print("\n✓ All assertions passed!")


def test_old_scale_snr_warns_on_stderr():
    """A recording with SNR > 60 predates the 2026-08-04 half-dB fix and
    must be flagged, not silently misread as if it were already dB."""
    fixture_dir = Path("tests/fixtures")
    fixture_dir.mkdir(exist_ok=True)
    flight_jsonl = fixture_dir / "flight-old-snr-scale-fixture.jsonl"
    row = {
        "v": 1, "t_ms": 0,
        "link": {
            "state": "linked", "residual_loss": None,
            "ctl": {
                "rung": {"idx": 0, "mcs": 5, "ov": 0.25}, "util": 0.1,
                "pre_fec_loss": 0.01, "budget": 0.5, "probation_ms_left": 0,
                "penalized": [],
                "counters": {"demotes_residual": 0, "demotes_util": 0, "promotes": 0,
                             "probation_fails": 0, "starved_drops": 0, "timeout_drops": 0},
                "last_event": {"t_ms": 0, "from": 0, "to": 0, "reason": "none", "u": 0.0},
            },
        },
        "cards": [{"frames": 1000, "classes": {"s1": {"rssi": -50.0, "snr": 70.0, "pps": 900}}}],
    }
    flight_jsonl.write_text(json.dumps(row) + "\n")

    result = subprocess.run(
        [sys.executable, "tools/flightreport.py", str(flight_jsonl)],
        capture_output=True, text=True,
    )
    assert result.returncode == 0, f"flightreport.py exited {result.returncode}: {result.stderr}"
    assert "predates the 2026-08-04 half-dB fix" in result.stderr, (
        f"Expected old-scale SNR warning on stderr, got: {result.stderr!r}"
    )

    # A same-shaped recording already on the dB scale must NOT warn.
    row["cards"][0]["classes"]["s1"]["snr"] = 27.0
    flight_jsonl.write_text(json.dumps(row) + "\n")
    result = subprocess.run(
        [sys.executable, "tools/flightreport.py", str(flight_jsonl)],
        capture_output=True, text=True,
    )
    assert result.returncode == 0
    assert "predates the 2026-08-04 half-dB fix" not in result.stderr, (
        f"Unexpected old-scale warning for a dB-scale recording: {result.stderr!r}"
    )
    # ...and a recording with no drone telemetry must not claim anything
    # about the uplink path either.
    assert "drone.uplink" not in result.stderr, (
        f"Unexpected uplink warning with no drone block: {result.stderr!r}"
    )

    # drone.uplink.snr_a/snr_b were fixed the same day and at the same place
    # as cards[].classes[].snr, so they are now the same backstop with the
    # same >60 threshold. An ordinary post-fix uplink reading 27 dB is a
    # legitimate value and must NOT warn.
    row["drone"] = {"state": "flying",
                    "uplink": {"rssi_a": -55.0, "rssi_b": -58.0,
                               "snr_a": 27.0, "snr_b": 24.0}}
    flight_jsonl.write_text(json.dumps(row) + "\n")
    result = subprocess.run(
        [sys.executable, "tools/flightreport.py", str(flight_jsonl)],
        capture_output=True, text=True,
    )
    assert result.returncode == 0
    assert "drone.uplink" not in result.stderr, (
        f"A post-fix 27 dB uplink must not warn, got: {result.stderr!r}"
    )

    # ...but an old-scale uplink (76 = 38 dB) still trips the backstop, and
    # says so as its own line so a file that straddles only one of the two
    # senders stays diagnosable.
    row["drone"]["uplink"] = {"rssi_a": -55.0, "rssi_b": -58.0,
                              "snr_a": 76.0, "snr_b": 74.0}
    flight_jsonl.write_text(json.dumps(row) + "\n")
    result = subprocess.run(
        [sys.executable, "tools/flightreport.py", str(flight_jsonl)],
        capture_output=True, text=True,
    )
    assert result.returncode == 0
    assert "drone.uplink.snr_a/snr_b exceeds 60" in result.stderr, (
        f"Expected uplink old-scale warning, got: {result.stderr!r}"
    )
    assert "38.0 dB" in result.stderr, (
        f"Expected the converted figure in the warning, got: {result.stderr!r}"
    )
    # cards[] is on the dB scale in this row, so only the uplink line fires.
    assert "cards[].classes[].snr exceeds 60" not in result.stderr, (
        f"cards[] must not warn on a dB-scale row: {result.stderr!r}"
    )

    # A null uplink (deaf radio / pre-DISC: the exporter writes nulls) is not
    # a reading, so it must not warn.
    row["drone"]["uplink"] = {"rssi_a": None, "rssi_b": None,
                              "snr_a": None, "snr_b": None}
    flight_jsonl.write_text(json.dumps(row) + "\n")
    result = subprocess.run(
        [sys.executable, "tools/flightreport.py", str(flight_jsonl)],
        capture_output=True, text=True,
    )
    assert result.returncode == 0
    assert "drone.uplink" not in result.stderr, (
        f"Unexpected uplink warning for a null uplink: {result.stderr!r}"
    )

    print("\n✓ Old-scale SNR warning test passed!")


CTL_LOG = """ctllog 1 ladder=0/100,2/50,4/25,5/25,6/25,7/10 down_util=0.35 up_util=0.15
S 1000 3 0.0100 31.5 0.0000 0.0200 0.0000
S 2000 3 0.0100 31.4 0.0000 0.0200 0.0000 -24.5
P 2000 4 pass 30.0 0.0500 2000
P 9000 4 fail 24.5 0.9000 600 -21.0
N 9000 4 1 19000
P 20000 4 fail 23.0 0.8000 550
E 30000 3 2 s3_util 0.4000 26.0 -20.5
P 40000 4 fail 41.0 0.9500 500 -23.0
P 50000 4 pass 29.5 0.0400 2000
"""


def test_ctllog_evm_optional_trailing_token():
    """Test that load_ctllog parses optional EVM trailing token and defaults
    to nan for pre-EVM logs (backward compatibility)."""
    with tempfile.TemporaryDirectory() as tmp_dir:
        p = Path(tmp_dir) / "ctl-0001_x.log"
        p.write_text(CTL_LOG)
        log = flightreport.load_ctllog(str(p))
    s_old, s_new = log["S"][0], log["S"][1]
    assert math.isnan(s_old["evm_db"])          # pre-EVM line -> nan, not KeyError
    assert s_new["evm_db"] == -24.5
    assert log["E"][0]["evm_db"] == -20.5
    evms = [p["evm_db"] for p in log["P"]]
    assert -21.0 in evms and sum(1 for v in evms if math.isnan(v)) == 3


def test_s_line_resid_cur_parsed_and_absent_is_nan():
    """resid_cur (ctllog 2, 2026-08-14 attribution) parses on a v2-shaped S
    line and defaults to nan on a v1-shaped one (backward compatibility)."""
    text = (
        "ctllog 2 ladder=5/25 down_util=0.60 up_util=0.15\n"
        "S 1000 3 0.0123 31.5 0.0456 0.0000 0.0000 -21.0 0.0011\n"
        "S 2000 3 0.0123 31.5 0.0456 0.0000 0.0000 -21.0\n"  # v1-shaped line
    )
    with tempfile.TemporaryDirectory() as tmp_dir:
        p = Path(tmp_dir) / "ctl-0001_20260814.log"
        p.write_text(text)
        log = flightreport.load_ctllog(str(p))
    assert log["S"][0]["resid_cur"] == 0.0011
    assert math.isnan(log["S"][1]["resid_cur"])


def test_s_line_fade_deltas_parsed_and_absent_nan():
    """drssi/dsnr (ctllog 3, 2026-08-14 fade-trigger deltas) parse on a
    v3-shaped S line and default to nan on a v2-shaped one (backward
    compatibility)."""
    text = (
        "ctllog 3 ladder=5/25 down_util=0.60 up_util=0.15\n"
        "S 1000 3 0.0123 31.5 0.0456 0.0000 0.0000 -21.0 0.0011 9.5 4.2\n"
        "S 2000 3 0.0123 31.5 0.0456 0.0000 0.0000 -21.0 0.0011\n"  # v2 line
    )
    with tempfile.TemporaryDirectory() as tmp_dir:
        p = Path(tmp_dir) / "ctl-0001_20260814.log"
        p.write_text(text)
        log = flightreport.load_ctllog(str(p))
    assert log["S"][0]["drssi"] == 9.5
    assert log["S"][0]["dsnr"] == 4.2
    assert math.isnan(log["S"][1]["drssi"])
    assert math.isnan(log["S"][1]["dsnr"])


def test_wall_report():
    """Direct-invocation pattern (matches the siblings above): write the
    CTL_LOG fixture to a temp file, capture flightreport's stdout, and check
    it against a ctl-log rather than a jsonl."""
    with tempfile.TemporaryDirectory() as tmp_dir:
        p = Path(tmp_dir) / "ctl-0001_x.log"
        p.write_text(CTL_LOG)
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            flightreport.main(str(p))
        out = buf.getvalue()

    assert "rung 4" in out
    assert "pass" in out and "fail" in out
    # fail cluster 23.0-24.5, passes 29.5-30.0 -> suggested wall between them
    m = re.search(r"suggested wall[^0-9]*([0-9.]+)", out)
    assert m and 24.5 < float(m.group(1)) < 29.5
    # the 41.0 dB fail is an outlier (mcs6-hole signature), flagged not pooled:
    assert "outlier" in out
    assert "s3_util" in out          # event summary includes new reasons
    assert "evm" in out              # DWELL and EVENTS now report EVM

    print("\n✓ Wall report test passed!")


def test_ctllog_r_lines_and_inversion():
    text = (
        "ctllog 1 ladder=0/100 down_util=0.35 up_util=0.15\n"
        "S 1000 0 0.0100 30.0 0.0000 0.0000 0.0000 -20.0\n"
        "R 10000 0 0.0100 0.0000 0.0200 0.0000 -20.0 0.50 400 0.0 0.0000 0\n"
        "R 10000 1 0.0500 0.0500 0.0400 0.0000 -24.0 0.80 350 5.0 0.1000 3\n"
        "R 20000 0 0.0110 0.0000 0.0200 0.0000 -20.1 0.50 600 0.0 0.0000 0\n"
    )
    with tempfile.NamedTemporaryFile("w", suffix=".log", delete=False) as f:
        f.write(text)
        path = f.name
    try:
        log = flightreport.load_ctllog(path)
        assert len(log["R"]) == 3
        assert log["R"][1]["rung"] == 1
        assert log["R"][1]["n"] == 350
        assert log["R"][1]["evm_sd_db"] == 0.80
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            flightreport.print_wall_report(log)
        s = out.getvalue()
        assert "RUNG STORE" in s
        # Final snapshots: rung0 resid 0.0 (n=600), rung1 resid 0.05 (n=350)
        # -> 0.05 >= max(2*0.0, 0.0+0.02): resid inversion, both n >= 300.
        assert "INVERSION" in s and "rung 1" in s
    finally:
        os.unlink(path)


def _mk_e(t, frm, to, reason):
    return {"t_ms": float(t), "from": frm, "to": to, "reason": reason,
            "u": 0.0, "snr_db": 30.0, "evm_db": float("nan")}


def test_find_episodes_clusters_and_first_reason():
    E = [
        _mk_e(1000, 5, 4, "fade"),
        _mk_e(1200, 4, 3, "util"),
        _mk_e(1400, 3, 2, "residual"),
        _mk_e(9000, 2, 3, "promote"),
        _mk_e(20000, 3, 2, "residual"),
    ]
    eps = flightreport.find_episodes(E)
    assert len(eps) == 2
    assert eps[0]["first_reason"] == "fade"
    assert eps[0]["steps"] == 3
    assert eps[0]["path"] == (5, 2)
    assert eps[0]["fade_lead_ms"] == 200.0  # 1200 - 1000
    assert eps[1]["first_reason"] == "residual"
    assert eps[1]["fade_lead_ms"] is None


def test_false_fade_and_attribution_miss():
    E = [
        _mk_e(1000, 5, 4, "fade"),          # false fade (episode is fade-only)
        _mk_e(8000, 4, 5, "promote"),
        _mk_e(20000, 5, 4, "util"),
        _mk_e(20150, 4, 3, "residual"),      # within 200 ms of previous E -> canary
        _mk_e(30000, 3, 2, "residual"),      # isolated -> not a canary hit
    ]
    eps = flightreport.find_episodes(E)
    false_fades = [e for e in eps if e["false_fade"]]
    assert len(false_fades) == 1
    assert false_fades[0]["repromote_ms"] == 7000.0
    misses = flightreport.attribution_misses(E)
    assert len(misses) == 1
    assert misses[0]["t_ms"] == 20150.0


def test_find_episodes_gap_boundary_closes_run():
    """The episode definition is 'consecutive demotes <= gap_ms apart';
    a gap of exactly gap_ms (default 3000) must NOT close the episode, and
    gap_ms + 1 must. This pins the half of find_episodes's contract that
    test_find_episodes_clusters_and_first_reason (closes via a promote) and
    test_false_fade_and_attribution_miss (its 9850ms gap is never asserted
    on) leave uncovered -- a > gap_ms mutation must fail this test."""
    E_at = [
        _mk_e(0, 5, 4, "util"),
        _mk_e(3000, 4, 3, "residual"),  # exactly gap_ms after the previous demote
    ]
    eps_at = flightreport.find_episodes(E_at)
    assert len(eps_at) == 1
    assert eps_at[0]["path"] == (5, 3)
    assert eps_at[0]["steps"] == 2

    E_over = [
        _mk_e(0, 5, 4, "util"),
        _mk_e(3001, 4, 3, "residual"),  # gap_ms + 1: must split
    ]
    eps_over = flightreport.find_episodes(E_over)
    assert len(eps_over) == 2
    assert eps_over[0]["path"] == (5, 4)
    assert eps_over[0]["steps"] == 1
    assert eps_over[1]["path"] == (4, 3)
    assert eps_over[1]["steps"] == 1


if __name__ == "__main__":
    test_flightreport_structure()
    test_old_scale_snr_warns_on_stderr()
    test_ctllog_evm_optional_trailing_token()
    test_wall_report()
    test_ctllog_r_lines_and_inversion()
    test_find_episodes_clusters_and_first_reason()
    test_false_fade_and_attribution_miss()
    test_find_episodes_gap_boundary_closes_run()
