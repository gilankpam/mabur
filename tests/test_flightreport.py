#!/usr/bin/env python3
"""Test suite for flightreport.py post-flight analysis tool."""
import json
import os
import subprocess
import sys
import tempfile
from pathlib import Path


def synthesize_flight_jsonl():
    """Generate a synthetic flight.jsonl fixture at real 500ms cadence (2 Hz sideport).

    Scenario:
    - 30s steady at rung 5, u≈0.1 (60 samples at 500ms)
    - util demote (reason=util, u=0.63, from rung 5 to 4)
    - 5s at rung 4 (10 samples)
    - residual burst 1: 2 consecutive samples (residual_loss 0.02, no gap between)
    - clean recovery: 4 clean samples (no residual_loss) — 2s of gap
    - residual burst 2: 2 consecutive samples (residual_loss 0.03, no gap between)
    - residual demote (reason=residual)
    - recovery promote (from rung 3 to 4)
    """
    rows = []

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

    t_ms = 0

    # Phase 1: 30s at rung 5, u≈0.1 (60 samples at 500ms cadence)
    for i in range(60):
        rows.append(make_datagram(t_ms, rung_idx=5, util=0.08 + 0.02 * (i % 2)))
        t_ms += 500

    # Phase 2: util demote event (rung 5 -> 4, reason=util, u=0.63)
    event = {
        "t_ms": t_ms,
        "from": 5,
        "to": 4,
        "reason": "util",
        "u": 0.63
    }
    rows.append(make_datagram(t_ms, rung_idx=4, util=0.63, event=event))
    t_ms += 500

    # Phase 3: 5s at rung 4 (10 samples at 500ms)
    for i in range(10):
        rows.append(make_datagram(t_ms, rung_idx=4, util=0.15 + 0.05 * (i % 2)))
        t_ms += 500

    # Phase 4: residual burst 1 (2 consecutive samples with residual_loss 0.02)
    for i in range(2):
        rows.append(make_datagram(t_ms, rung_idx=4, util=0.20, residual_loss=0.02))
        t_ms += 500

    # Phase 5: clean recovery (4 clean samples = 2s gap > 750ms threshold)
    for i in range(4):
        rows.append(make_datagram(t_ms, rung_idx=4, util=0.12))
        t_ms += 500

    # Phase 6: residual burst 2 (2 consecutive samples with residual_loss 0.03)
    for i in range(2):
        rows.append(make_datagram(t_ms, rung_idx=4, util=0.22, residual_loss=0.03))
        t_ms += 500

    # Phase 7: residual demote event (rung 4 -> 3, reason=residual)
    event = {
        "t_ms": t_ms,
        "from": 4,
        "to": 3,
        "reason": "residual",
        "u": 0.30
    }
    rows.append(make_datagram(t_ms, rung_idx=3, util=0.30, event=event, drone_state="linked"))
    t_ms += 500

    # Phase 8: recovery promote (rung 3 -> 4, reason=promote)
    event = {
        "t_ms": t_ms,
        "from": 3,
        "to": 4,
        "reason": "promote",
        "u": 0.25
    }
    rows.append(make_datagram(t_ms, rung_idx=4, util=0.25, event=event, drone_state="linked"))
    t_ms += 500

    # Phase 9: final samples at rung 4
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

    # Verify RESIDUAL EPISODES section: 2 bursts separated by clean samples
    assert "RESIDUAL EPISODES: 2" in output, "Should have exactly 2 residual episodes (separated by 4 clean samples > 750ms threshold)"

    # Verify each episode has expected residual values
    residual_section = output[output.find("RESIDUAL EPISODES"):]
    assert "residual=0.0200" in residual_section, "Missing first residual burst (0.02)"
    assert "residual=0.0300" in residual_section, "Missing second residual burst (0.03)"

    # Verify drone state context join
    # The output should reference drone states somewhere in the transition context
    # At minimum, we should see drone state info
    assert "drone_state=linked" in output, "Missing drone state context in output"

    print("\n✓ All assertions passed!")


if __name__ == "__main__":
    test_flightreport_structure()
