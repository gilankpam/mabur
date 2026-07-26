#!/usr/bin/env python3
"""Golden vectors for the GS score window, generated from devourer's Python
score reference (tools/precoder/score.py). Deterministic: no randomness, no
time. Re-run + git diff must be clean.

optable.json, controller_replay.json, and the "rung" section of score.json
were removed 2026-07-27 (SDD ladder-controller Task 5): the model-driven
link table, its controller, and the per-rung delivery window are all dead
code, superseded by the measured-loss ladder controller
(gs/src/ladder_controller.h). ScoreWindow (this script's only remaining
consumer) is unaffected — it never depended on the deleted models."""
import json, os, sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
PRECODER = os.path.abspath(os.path.join(ROOT, "..", "devourer", "tools", "precoder"))
sys.path.insert(0, PRECODER)

VEC = os.path.join(ROOT, "tests", "vectors")
os.makedirs(VEC, exist_ok=True)

def dump(name, obj):
    with open(os.path.join(VEC, name), "w") as f:
        json.dump(obj, f, indent=1, sort_keys=True)
    print("wrote", name)

# --- score -------------------------------------------------------------------
from score import ScoreWindow

def run_trace(frames, residual=None):
    w = ScoreWindow()
    for rssi, snr, crc, seq, t in frames:
        w.add_frame(rssi, snr, crc, seq, t)
    return {"frames": frames, "n": w.n(),
            "snr_est": w.snr_estimate(), "rssi_est": w.rssi_estimate(),
            "fcs_loss": w.fcs_loss(), "seq_gap_loss": w.seq_gap_loss(),
            "ack_seq": w.ack_seq(),
            "score_none": w.score(), "score_residual": w.score(residual),
            "residual": residual}

score_cases = [
    run_trace([(-55.0, 22.0, False, 10, 0.00), (-56.0, 21.0, False, 11, 0.05),
               (-54.0, 23.0, True, 13, 0.10)], residual=0.02),
    # 12-bit wrap inside the window: must NOT read as a giant gap
    run_trace([(-60.0, 15.0, False, 4093, 0.00), (-60.0, 15.0, False, 4095, 0.02),
               (-60.0, 15.0, False, 1, 0.04), (-60.0, 15.0, False, 2, 0.06)],
              residual=0.0),
    # window pruning: first frame falls out of the 0.5 s window
    run_trace([(-50.0, 30.0, False, 1, 0.00), (-80.0, 5.0, False, 2, 0.60),
               (-80.0, 5.0, False, 3, 0.65)], residual=None),
    run_trace([], residual=None),
]
dump("score.json", {"score": score_cases})
