#!/usr/bin/env python3
"""analyze_sweep.py — judge a txagcbench sweep against kTxagcGainDb.

Input: JSONL from txagcbench-rx (one object per CRC-clean sweep frame:
idx, pass, seq, rssi_a, rssi_b, snr_a, snr_b; rssi_* raw pwdb,
dBm ~= raw - 110; chain A is off-scale on the 8822E so chain B is the
default).

Only the SHAPE of the curve is judged: measured medians and the reference
table are both re-anchored to the lowest measured index before comparison
(a constant offset cancels between the controller's update() normalization
and its resolve() inversion).

Verdict: PASS iff RMS <= 1.0 dB and max-abs <= 2.0 dB over measured
indices. Exit codes: 0 PASS, 1 FAIL, 2 usage/input error.

--emit-table prints a drop-in kTxagcGainDb[] replacement: measured shape,
re-anchored so entry 0 = 0.0 (table convention), interior gaps linearly
interpolated, head/tail gaps extended flat (warned).

--measured-only skips the reference-table load and the table-comparison
columns/verdict entirely: prints per-idx medians/n/drift and the health
warnings only, exit 0 always (mabur's power model moved to a linear
offset-qdB gain — gs/src/gen/gen_tables.h no longer carries kTxagcGainDb,
see tools/pyref/offset_power.py — so this is the mode to use post-move).

--selftest needs no hardware and does NOT read kTxagcGainDb (or any file):
it synthesizes two small built-in reference curves — a saturating "ref"
curve and a distinct linear 0.5 dB/step curve — and proves the analyzer's
PASS/FAIL shape discrimination: a sweep synthesized from the ref curve
(+0.3 dB noise) must PASS against it, and a sweep synthesized from the
linear curve must FAIL against it. This only tests analyze()'s judging
logic, not any real gain table.

Spec: docs/superpowers/specs/2026-07-16-txagcbench-design.md.
"""
import argparse
import json
import math
import random
import re
import statistics
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
DEFAULT_TABLE = REPO / "gs" / "src" / "gen" / "gen_tables.h"

RMS_LIMIT_DB = 1.0
MAX_LIMIT_DB = 2.0
SAT_WARN_DBM = -45.0
FLOOR_WARN_DBM = -85.0
MONO_SLACK_DB = 0.5


def die(msg):
    print(f"error: {msg}", file=sys.stderr)
    sys.exit(2)


def parse_gain_table(path):
    try:
        text = Path(path).read_text()
    except OSError as e:
        die(f"cannot read {path}: {e}")
    m = re.search(r"kTxagcGainDb\[\]\s*=\s*\{([^}]*)\}", text)
    if not m:
        die(f"kTxagcGainDb not found in {path} — mabur's power model moved "
            f"to a linear offset-qdB gain (gs/src/energy.h, "
            f"tools/pyref/offset_power.py) and the table was deleted; use "
            f"--measured-only for shape-only analysis with no reference "
            f"table, or pass --table <path> pointing at a file that still "
            f"has kTxagcGainDb[]")
    vals = [float(v) for v in re.findall(r"-?\d+(?:\.\d+)?", m.group(1))]
    if len(vals) != 64:
        die(f"expected 64 kTxagcGainDb entries, got {len(vals)}")
    return vals


def load_rows(path, chain):
    """-> list of (idx, pass, dbm)."""
    key = "rssi_b" if chain == "b" else "rssi_a"
    rows = []
    try:
        text = Path(path).read_text()
    except OSError as e:
        die(f"cannot read {path}: {e}")
    for ln, line in enumerate(text.splitlines(), 1):
        line = line.strip()
        if not line:
            continue
        try:
            o = json.loads(line)
            idx = int(o["idx"])
            if not 0 <= idx <= 127:
                die(f"{path}:{ln}: idx {idx} out of range 0..127")
            rows.append((idx, int(o["pass"]), float(o[key]) - 110.0))
        except (ValueError, KeyError) as e:
            die(f"{path}:{ln}: bad record ({e})")
    if not rows:
        die(f"{path}: no records")
    return rows


def analyze(rows, table, min_samples, out=print):
    """-> (passed: bool, med: {idx: median_dbm} or None)."""
    by_idx, by_idx_pass = {}, {}
    for idx, pss, dbm in rows:
        by_idx.setdefault(idx, []).append(dbm)
        by_idx_pass.setdefault((idx, pss), []).append(dbm)

    med, skipped = {}, []
    for idx, vals in sorted(by_idx.items()):
        if len(vals) < min_samples:
            skipped.append((idx, len(vals)))
            continue
        med[idx] = statistics.median(vals)
    if len(med) < 2:
        out("FAIL: fewer than 2 indices with enough samples")
        return False, None

    # The reference table only covers idx 0..63; higher indices (7-bit
    # Jaguar3 TXAGC) are measured-only: shown in the curve, excluded from
    # the verdict.
    cmp_idx = [i for i in med if i < len(table)]
    if len(cmp_idx) < 2:
        out("FAIL: fewer than 2 table-range indices with enough samples")
        return False, None
    anchor = min(cmp_idx)
    meas_rel = {i: med[i] - med[anchor] for i in med}
    tab_rel = {i: table[i] - table[anchor] for i in cmp_idx}
    errs = {i: meas_rel[i] - tab_rel[i] for i in cmp_idx}

    drift = {}
    for idx in med:
        p1 = by_idx_pass.get((idx, 1), [])
        p2 = by_idx_pass.get((idx, 2), [])
        if p1 and p2:
            drift[idx] = statistics.median(p2) - statistics.median(p1)

    out(f"anchor: idx {anchor} = 0 dB (shape-only; offsets cancel)")
    out(f"{'idx':>3} {'n':>5} {'dBm':>7} {'meas_rel':>9} {'tab_rel':>8} "
        f"{'err':>6} {'drift':>6}")
    for idx in sorted(med):
        d = f"{drift[idx]:+.2f}" if idx in drift else "n/a"
        if idx in errs:
            tcol = f"{tab_rel[idx]:>8.2f} {errs[idx]:>+6.2f}"
        else:
            tcol = f"{'n/a':>8} {'n/a':>6}"
        out(f"{idx:>3} {len(by_idx[idx]):>5} {med[idx]:>7.1f} "
            f"{meas_rel[idx]:>9.2f} {tcol} {d:>6}")

    e = list(errs.values())
    rms = math.sqrt(sum(x * x for x in e) / len(e))
    worst = max(errs, key=lambda i: abs(errs[i]))
    srt = sorted(med)
    mono_bad = [b for a_, b in zip(srt, srt[1:])
                if med[b] < med[a_] - MONO_SLACK_DB]

    out(f"\nmeasured {len(med)} indices ({len(cmp_idx)} in table range 0.."
        f"{len(table) - 1}) | RMS {rms:.2f} dB | "
        f"max |err| {abs(errs[worst]):.2f} dB @ idx {worst}"
        + (" | verdict covers table range only" if len(cmp_idx) < len(med)
           else ""))
    if drift:
        dworst = max(drift, key=lambda i: abs(drift[i]))
        out(f"up/down drift: max {drift[dworst]:+.2f} dB @ idx {dworst} "
            f"(large = thermal drift during the run)")
    if skipped:
        out(f"UNMEASURED (n < {min_samples}): "
            + ", ".join(f"idx {i} (n={n})" for i, n in skipped))
    if mono_bad:
        out(f"WARNING: non-monotonic at idx {mono_bad} "
            f"(> {MONO_SLACK_DB} dB reversal)")
    if max(med.values()) > SAT_WARN_DBM:
        out(f"WARNING: max median {max(med.values()):.1f} dBm > "
            f"{SAT_WARN_DBM} — RX may be saturating; add attenuation or "
            f"distance and re-run")
    if min(med.values()) < FLOOR_WARN_DBM:
        out(f"WARNING: min median {min(med.values()):.1f} dBm < "
            f"{FLOOR_WARN_DBM} — bottom of sweep near sensitivity floor")

    passed = rms <= RMS_LIMIT_DB and abs(errs[worst]) <= MAX_LIMIT_DB
    return passed, med


def analyze_measured_only(rows, min_samples, out=print):
    """Shape-only report with no reference table: medians/n/drift + the same
    health warnings as analyze(), minus the table columns and verdict.
    -> med: {idx: median_dbm} or None."""
    by_idx, by_idx_pass = {}, {}
    for idx, pss, dbm in rows:
        by_idx.setdefault(idx, []).append(dbm)
        by_idx_pass.setdefault((idx, pss), []).append(dbm)

    med, skipped = {}, []
    for idx, vals in sorted(by_idx.items()):
        if len(vals) < min_samples:
            skipped.append((idx, len(vals)))
            continue
        med[idx] = statistics.median(vals)
    if len(med) < 2:
        out("FAIL: fewer than 2 indices with enough samples")
        return None

    anchor = min(med)
    meas_rel = {i: med[i] - med[anchor] for i in med}

    drift = {}
    for idx in med:
        p1 = by_idx_pass.get((idx, 1), [])
        p2 = by_idx_pass.get((idx, 2), [])
        if p1 and p2:
            drift[idx] = statistics.median(p2) - statistics.median(p1)

    out(f"anchor: idx {anchor} = 0 dB (shape-only; measured-only mode, no "
        f"reference table)")
    out(f"{'idx':>3} {'n':>5} {'dBm':>7} {'meas_rel':>9} {'drift':>6}")
    for idx in sorted(med):
        d = f"{drift[idx]:+.2f}" if idx in drift else "n/a"
        out(f"{idx:>3} {len(by_idx[idx]):>5} {med[idx]:>7.1f} "
            f"{meas_rel[idx]:>9.2f} {d:>6}")

    srt = sorted(med)
    mono_bad = [b for a_, b in zip(srt, srt[1:])
                if med[b] < med[a_] - MONO_SLACK_DB]

    out(f"\nmeasured {len(med)} indices (measured-only, no verdict)")
    if drift:
        dworst = max(drift, key=lambda i: abs(drift[i]))
        out(f"up/down drift: max {drift[dworst]:+.2f} dB @ idx {dworst} "
            f"(large = thermal drift during the run)")
    if skipped:
        out(f"UNMEASURED (n < {min_samples}): "
            + ", ".join(f"idx {i} (n={n})" for i, n in skipped))
    if mono_bad:
        out(f"WARNING: non-monotonic at idx {mono_bad} "
            f"(> {MONO_SLACK_DB} dB reversal)")
    if max(med.values()) > SAT_WARN_DBM:
        out(f"WARNING: max median {max(med.values()):.1f} dBm > "
            f"{SAT_WARN_DBM} — RX may be saturating; add attenuation or "
            f"distance and re-run")
    if min(med.values()) < FLOOR_WARN_DBM:
        out(f"WARNING: min median {min(med.values()):.1f} dBm < "
            f"{FLOOR_WARN_DBM} — bottom of sweep near sensitivity floor")

    return med


def emit_table(med, out=print):
    # Drop-in stays 64 entries: mabur's power axis is idx 0..63 today.
    # Measured indices above 63 are reported in the curve but not emitted.
    idxs = sorted(i for i in med if i <= 63)
    g = [None] * 64
    for i in idxs:
        g[i] = med[i] - med[idxs[0]]
    for i in range(idxs[0]):            # head: flat
        g[i] = g[idxs[0]]
    for i in range(idxs[-1] + 1, 64):   # tail: flat
        g[i] = g[idxs[-1]]
    prev = idxs[0]
    for i in range(idxs[0] + 1, idxs[-1]):   # interior gaps: linear
        if g[i] is not None:
            prev = i
            continue
        nxt = next(j for j in range(i + 1, idxs[-1] + 1) if g[j] is not None)
        g[i] = g[prev] + (g[nxt] - g[prev]) * (i - prev) / (nxt - prev)
    z = g[0]                            # table convention: entry 0 = 0.0
    g = [v - z for v in g]
    if idxs[0] > 0 or idxs[-1] < 63:
        out(f"// WARNING: idx <{idxs[0]} and >{idxs[-1]} unmeasured — "
            f"extended flat")
    out("inline constexpr double kTxagcGainDb[] = {")
    for row in range(0, 64, 6):
        vals = ", ".join(f"{v:.3f}" for v in g[row:row + 6])
        out(f"    {vals},")
    out("};")


def selftest():
    """Self-contained: proves analyze()'s PASS/FAIL shape discrimination
    using two small built-in synthetic curves. Reads no file (not even
    kTxagcGainDb) — this only exercises the judging logic, not any real
    hardware table."""
    random.seed(1234)

    # A saturating reference curve (distinct shape from the linear FAIL
    # case below) stands in for a real gain table.
    ref = [8.0 * math.log2(1 + i) for i in range(64)]
    linear = [0.5 * i for i in range(64)]

    def synth(curve):
        rows = []
        for idx in range(64):
            for pss in (1, 2):
                for _ in range(50):
                    rows.append((idx, pss,
                                 -80.0 + curve[idx] + random.gauss(0, 0.3)))
        return rows

    silent = lambda *a, **k: None
    ok1, _ = analyze(synth(ref), ref, 20, out=silent)
    ok2, _ = analyze(synth(linear), ref, 20, out=silent)
    print(f"selftest: ref-shaped input -> {'PASS' if ok1 else 'FAIL'} "
          f"(expect PASS)")
    print(f"selftest: linear 0.5 dB/step input -> "
          f"{'PASS' if ok2 else 'FAIL'} (expect FAIL)")
    good = ok1 and not ok2
    print(f"selftest: {'PASS' if good else 'FAIL'}")
    return 0 if good else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("jsonl", nargs="?", help="txagcbench-rx output")
    ap.add_argument("--table", default=str(DEFAULT_TABLE))
    ap.add_argument("--chain", choices=["a", "b"], default="b",
                    help="RSSI chain (A is off-scale on 8822E; default b)")
    ap.add_argument("--min-samples", type=int, default=20)
    ap.add_argument("--emit-table", action="store_true",
                    help="print a drop-in kTxagcGainDb[] from the measurement")
    ap.add_argument("--measured-only", action="store_true",
                    help="skip the reference-table load/compare/verdict; "
                         "print medians/drift/health only, exit 0 always")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()

    if args.selftest:
        sys.exit(selftest())
    if not args.jsonl:
        ap.error("jsonl input required (or --selftest)")

    rows = load_rows(args.jsonl, args.chain)

    if args.measured_only:
        med = analyze_measured_only(rows, args.min_samples)
        if args.emit_table and med:
            print()
            emit_table(med)
        sys.exit(0)

    table = parse_gain_table(args.table)
    passed, med = analyze(rows, table, args.min_samples)
    if args.emit_table and med:
        print()
        emit_table(med)
    print(f"\nVERDICT: {'PASS' if passed else 'FAIL'} "
          f"(limits: RMS <= {RMS_LIMIT_DB} dB, max <= {MAX_LIMIT_DB} dB)")
    sys.exit(0 if passed else 1)


if __name__ == "__main__":
    main()
