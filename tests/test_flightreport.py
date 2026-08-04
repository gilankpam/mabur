#!/usr/bin/env python3
"""Test suite for flightreport.py post-flight analysis tool."""
import json
import os
import subprocess
import sys
import tempfile
from pathlib import Path


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


if __name__ == "__main__":
    test_flightreport_structure()
    test_old_scale_snr_warns_on_stderr()
