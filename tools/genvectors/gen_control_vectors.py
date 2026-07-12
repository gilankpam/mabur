#!/usr/bin/env python3
"""Golden vectors for GS energy control, generated from devourer's Python
energy_model. Deterministic: no randomness, no time. Re-run + git diff must be
clean."""
import json, os, sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
PRECODER = os.path.abspath(os.path.join(ROOT, "..", "devourer", "tools", "precoder"))
sys.path.insert(0, PRECODER)

import energy_model as em  # noqa: E402

VEC = os.path.join(ROOT, "tests", "vectors")
os.makedirs(VEC, exist_ok=True)

def dump(name, obj):
    with open(os.path.join(VEC, name), "w") as f:
        json.dump(obj, f, indent=1, sort_keys=True)
    print("wrote", name)

# --- energy ---------------------------------------------------------------
cal = em.load_calibration()
e_cases = []
for mode, mcs, bw, sgi, txagc, src, ov, payload, pdel in [
    ("ht", 0, 20, False, 63, 1.4e6, 1.00, 1024, 1.0),
    ("ht", 2, 20, False, 32, 4e6, 0.25, 1024, 0.99),
    ("ht", 7, 40, True, 10, 8e6, 0.10, 1024, 0.95),
    ("vht", 8, 80, False, 0, 20e6, 0.10, 1024, 1.0),
    ("ht", 0, 20, False, 63, 50e6, 1.00, 1024, 1.0),   # infeasible: airtime > 1
    ("ht", 4, 20, False, 32, 4e6, 0.25, 1024, 0.0),    # nothing delivered
]:
    pt = em.TxPoint(mode, mcs, bw, sgi, txagc)
    af = em.airtime_fraction(pt, src, ov, payload, cal)
    eb = em.energy_per_delivered_bit(pt, src, ov, payload, pdel, cal)
    e_cases.append({"vht": mode == "vht", "mcs": mcs, "bw": bw, "sgi": sgi,
                    "txagc": txagc, "src": src, "ov": ov, "payload": payload,
                    "p_deliver": pdel,
                    "eff_bps": em.phy_rate_eff_bps(pt, payload, cal),
                    "airtime": af,
                    "e_bit": None if eb == float("inf") else eb})
g_cases = [{"need_db": d,
            "idx": (lambda r: -1 if r is None else r)(cal.min_txagc_for_gain(d))}
           for d in [0.0, 0.001, 5.0, 24.9, 25.0, 30.0]]
dump("energy.json", {"cases": e_cases, "gain": g_cases,
                     "bw_noise": [{"bw": b, "db": em.bw_noise_db(b)} for b in (20, 40, 80)]})

# --- optable ------------------------------------------------------------------
import link_model as lm
import op_table

link = lm.LinkModel()
snr_req = [{"mcs": m, "ov": ov, "target": t,
            "req": link.snr_required(m, ov, t)}
           for m in range(8) for ov in (0.10, 0.25, 0.50, 0.75, 1.00)
           for t in (0.90, 0.99, 0.999)]
rows = op_table.build_link_rows(link, 0.99, range(8),
                                (0.10, 0.25, 0.50, 0.75, 1.00), 20)
res_cases = []
for pl in (-10.0, 0.0, 5.5, 12.0, 30.0):
    for r in rows[::7]:                       # sample every 7th row
        op = op_table.resolve(r, pl, cal, link, 1024, 4e6, 2.0)
        res_cases.append({
            "row": {"vht": r.mode == "vht", "mcs": r.mcs, "bw": r.bw,
                    "sgi": r.sgi, "ov": r.overhead, "snr_req": r.snr_req},
            "pl": pl,
            "op": None if op is None else {
                "txagc": op.txagc, "e_bit": None if op.e_bit == float("inf") else op.e_bit,
                "p_deliver": op.p_deliver}})
# Edge cases: sentinel and grid clamping (parity test hazards, never exercised above)
# Sentinel: impossible target (2.0 > max p_deliver 1.0) must return hi+step = 40.5
edges_snr_req = [
    {"mcs": 0, "ov": 0.10, "target": 2.0, "snr_req": link.snr_required(0, 0.10, 2.0)},
    {"mcs": 7, "ov": 1.00, "target": 2.0, "snr_req": link.snr_required(7, 1.00, 2.0)},
]
# Grid clamp: SNR outside [-20, 60] bucket range must clamp to edge bucket
# snr=-50 clamps to bucket -20; snr=100 clamps to bucket 60
edges_pdeliver = [
    {"mcs": 3, "ov": 0.50, "snr": -50.0, "p_deliver": link.p_deliver(-50.0, 3, 0.50, sbi=True)},
    {"mcs": 5, "ov": 0.75, "snr": 100.0, "p_deliver": link.p_deliver(100.0, 5, 0.75, sbi=True)},
]

dump("optable.json", {"edges_pdeliver": edges_pdeliver,
                      "edges_snr_req": edges_snr_req,
                      "resolve": res_cases,
                      "rows": [{"vht": r.mode == "vht", "mcs": r.mcs, "bw": r.bw,
                                "sgi": r.sgi, "ov": r.overhead, "snr_req": r.snr_req}
                               for r in rows],
                      "snr_req": snr_req})

# --- score -------------------------------------------------------------------
from score import ScoreWindow, RungWindow

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
rw = RungWindow((20, 40))
rung_seqs = [s for s in range(64) if s % 5 != 0]      # drop every 5th seq
for s in rung_seqs:
    rw.add_seq(s)
rung_stats = {str(bw): [d, n] for bw, (d, n) in rw.stats().items()}
dump("score.json", {"score": score_cases,
                    "rung": {"bw_set": [20, 40], "seqs": rung_seqs,
                             "stats": rung_stats}})
