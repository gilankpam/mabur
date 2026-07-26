#!/usr/bin/env python3
"""Test suite for flightreport.py post-flight analysis tool."""
import json
import os
import subprocess
import sys
import tempfile
from pathlib import Path


def synthesize_flight_jsonl():
    """Generate a synthetic 200-line flight.jsonl fixture.

    Scenario:
    - 60 s steady at rung 5, u≈0.1
    - util demote (reason=util, u=0.63, from rung 5 to 4)
    - 10 s at rung 4
    - residual episode (2 samples with residual_loss 0.02)
    - residual demote (reason=residual, from rung 4 to 3)
    - recovery promote (from rung 3 to 4)
    """
    rows = []
    t_ms = 0

    # Helper to create a datagram
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

    # Phase 1: 60 s at rung 5, u≈0.1 (60 samples at 1 Hz)
    for i in range(60):
        t = i * 1000
        rows.append(make_datagram(t, rung_idx=5, util=0.08 + 0.02 * (i % 2)))

    # Phase 2: util demote event at t=60s (rung 5 -> 4, reason=util, u=0.63)
    event_time = 60000
    event = {
        "t_ms": event_time,
        "from": 5,
        "to": 4,
        "reason": "util",
        "u": 0.63
    }
    rows.append(make_datagram(event_time, rung_idx=4, util=0.63, event=event))

    # Phase 3: 10 s at rung 4 (10 samples at 1 Hz)
    for i in range(1, 11):
        t = 60000 + i * 1000
        rows.append(make_datagram(t, rung_idx=4, util=0.15 + 0.05 * (i % 2)))

    # Phase 4: residual episode (2 consecutive samples with residual_loss 0.02)
    # These are NOT demote events yet, just elevated residual_loss
    for i in range(2):
        t = 70000 + i * 1000
        rows.append(make_datagram(t, rung_idx=4, util=0.20, residual_loss=0.02))

    # Phase 5: residual demote event (rung 4 -> 3, reason=residual)
    event_time = 72000
    event = {
        "t_ms": event_time,
        "from": 4,
        "to": 3,
        "reason": "residual",
        "u": 0.30
    }
    rows.append(make_datagram(event_time, rung_idx=3, util=0.30, event=event, drone_state="linked"))

    # Phase 6: recovery promote (rung 3 -> 4, reason=promote)
    event_time = 75000
    event = {
        "t_ms": event_time,
        "from": 3,
        "to": 4,
        "reason": "promote",
        "u": 0.25
    }
    rows.append(make_datagram(event_time, rung_idx=4, util=0.25, event=event, drone_state="linked"))

    # Phase 7: a few more samples at rung 4 to complete the fixture
    for i in range(3):
        t = 75000 + (i + 1) * 1000
        rows.append(make_datagram(t, rung_idx=4, util=0.12))

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

    # Verify RESIDUAL EPISODES section
    assert "RESIDUAL EPISODES: 1" in output, "Should have exactly 1 residual episode (merged from 2 consecutive samples)"

    # Verify drone state context join
    # The output should reference drone states somewhere in the transition context
    # At minimum, we should see drone state info
    assert "drone" in output.lower() or "state" in output.lower(), "Missing drone state context"

    print("\n✓ All assertions passed!")


if __name__ == "__main__":
    test_flightreport_structure()
