#!/usr/bin/env python3
"""Test suite for flightreport.py post-flight analysis tool."""
import json
import os
import subprocess
import sys
import tempfile
from pathlib import Path


def make_datagram(t, rung_idx, util, residual_loss=0.0, event=None, drone_state="linked", offset_qdb=0):
    """Helper for tests: create a sideport datagram. Shared by
    synthesize_flight_jsonl() and the --calib tests."""
    dg = {
        "v": 1,
        "t_ms": t,
        "link": {
            "state": drone_state,
            "residual_loss": residual_loss if residual_loss > 0 else None,
            "op": {"offset_qdb": offset_qdb},
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
    return dg


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

    return "\n".join(json.dumps(r) for r in rows) + "\n"


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


def test_calib_mode():
    """Test flightreport.py --calib mode for threshold calibration."""
    # Run A (clean baseline): 20 samples at u=0.01, rung 5, no residual, offset 0.
    # Run B (slow ramp): u climbs 0.05..0.50 over 10 samples (500 ms apart,
    # offset ramping negative), then a residual sample at the end.
    rows_a = [make_datagram(t * 500, 5, 0.01) for t in range(20)]
    rows_b = []
    for i, u in enumerate([0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45, 0.50]):
        rows_b.append(make_datagram(i * 500, 4, u, offset_qdb=-2 * i))
    rows_b.append(make_datagram(5000, 4, 0.55, residual_loss=0.01, offset_qdb=-20))

    with tempfile.TemporaryDirectory() as td:
        pa, pb = os.path.join(td, "a.jsonl"), os.path.join(td, "b.jsonl")
        with open(pa, "w") as f:
            f.write("\n".join(json.dumps(r) for r in rows_a))
        with open(pb, "w") as f:
            f.write("\n".join(json.dumps(r) for r in rows_b))
        out = subprocess.run(
            ["python3", os.path.join(os.path.dirname(__file__), "..", "tools", "flightreport.py"),
             "--calib", pa, pb],
            capture_output=True, text=True, check=True).stdout

    assert "CLEAN U PER RUNG" in out
    assert "rung 5" in out                    # baseline percentiles present
    assert "CANDIDATE down_util SWEEP" in out
    # u=0.20 (t=1500) is not > 0.20, so the first sample to actually cross
    # candidate 0.20 is u=0.25 at t=2000; residual hits at t=5000: candidate
    # 0.20 must be caught with a 3.0 s lead; candidate 0.60 must catch nothing.
    assert "0.2" in out and "1/1" in out
    assert "0/1" in out                       # 0.60 (and others above 0.55) miss
    assert "EPISODES" in out


def test_calib_no_phantom_catch_after_recovery():
    """Regression: a residual preceded by a FULL recovery must not inherit a
    phantom "catch" from an earlier, already-recovered crossing (the util
    path lost the race; a crossing that recovered before the episode is a
    loss, not a catch). Reviewer repro: u=0.4 for a couple of samples
    (crosses candidate 0.20), then u=0.0 for 20s, then a residual sample —
    candidate 0.20 must NOT catch this episode."""
    rows = []
    t_ms = 0
    # Brief spike above 0.20 that fully recovers before the episode.
    for _ in range(2):
        rows.append(make_datagram(t_ms, 4, 0.4))
        t_ms += 500
    # 20s of quiet (u=0.0): the spike is long gone by the time the residual
    # below fires, so no candidate should be scored as having caught it.
    quiet_until = t_ms + 20000
    while t_ms < quiet_until:
        rows.append(make_datagram(t_ms, 4, 0.0))
        t_ms += 500
    rows.append(make_datagram(t_ms, 4, 0.0, residual_loss=0.01))

    with tempfile.TemporaryDirectory() as td:
        p = os.path.join(td, "recovered.jsonl")
        with open(p, "w") as f:
            f.write("\n".join(json.dumps(r) for r in rows))
        out = subprocess.run(
            ["python3", os.path.join(os.path.dirname(__file__), "..", "tools", "flightreport.py"),
             "--calib", p],
            capture_output=True, text=True, check=True).stdout

    assert "CANDIDATE down_util SWEEP (episodes=1" in out
    sweep_section = out[out.find("CANDIDATE down_util SWEEP"):out.find("EPISODES")]
    for line in sweep_section.splitlines():
        if line.strip().startswith("0.20:"):
            assert "caught 0/1" in line, f"phantom catch survived: {line!r}"
            break
    else:
        raise AssertionError("candidate 0.20 missing from sweep output")
    # The episode line itself should show it uncaught at every candidate.
    episodes_section = out[out.find("EPISODES"):]
    assert "uncaught at all candidates" in episodes_section, episodes_section

    # Same fixture also carries the pre-existing ramp case (test_calib_mode)
    # unaffected: verified separately there, still caught.


if __name__ == "__main__":
    test_flightreport_structure()
    test_calib_mode()
    test_calib_no_phantom_catch_after_recovery()
