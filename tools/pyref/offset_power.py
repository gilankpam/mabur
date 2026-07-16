"""Mabur-owned TX-power model (offset semantics), 2026-07-17.
DIVERGES from devourer tools/precoder/energy_model.py (frozen prototype):
power is a signed qdB offset from the calibrated baseline, gain is linear
0.25 dB/qdB (bench-validated slope, docs/txagc-calibration.md), and PA draw
maps offset -> effective index over the prototype's kPaW curve."""

def gain_db(offset_qdb):
    return 0.25 * offset_qdb

def min_offset_qdb_for_gain(need_db):
    import math
    return int(math.ceil(need_db * 4.0))

def pa_index(offset_qdb, base_ref_idx=53):
    return max(0, min(63, base_ref_idx + offset_qdb))
