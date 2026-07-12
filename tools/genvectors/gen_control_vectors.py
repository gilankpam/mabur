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
dump("optable.json", {"snr_req": snr_req,
                      "rows": [{"vht": r.mode == "vht", "mcs": r.mcs, "bw": r.bw,
                                "sgi": r.sgi, "ov": r.overhead, "snr_req": r.snr_req}
                               for r in rows],
                      "resolve": res_cases})
