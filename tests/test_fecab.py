#!/usr/bin/env python3
"""Test for tools/bench/fecab/analyze.py (the bench A/B rig for FEC
placement, tools/bench/fecab/).

Builds a synthetic session: a au.log (`# aulog 7`, 20 columns) and a
frame.log (`# framelog 3`, 22 columns) under a fake SESSROOT/<sess>/, a
.run file (step + mono_us command stamps, like run.sh's local log and the
GS's sched.log catted into it) under a fake OUTDIR, and a pair of drone CPU
snapshot files. Then checks analyze.py's per-arm-x-step row against the
known ground truth:

  - a step's trunc% counts only flag-less-0x80 (incomplete, see
    gs/src/au_ring.h kRecFlagComplete) AUs INSIDE its window -- truncated
    rows just outside the window must not move it;
  - one AU completing 15 ms after its neighbours (vs. the rolling 2 s
    per-sid median of t_complete - pts) shows up in >10ms% but not >20ms%;
  - nodisp% counts a pts missing from frame.log;
  - per-thread CPU of a thread with +50 ticks over a 1 s uptime delta is
    50.0 (the (utime+stime) delta / (uptime delta x 100) x 100 formula).
"""
import os
import sys
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                 '..', 'tools', 'bench', 'fecab'))
import analyze  # noqa: E402

PTS_OFFSET = 50_000_000
STEP_US = 500_000
T0 = 103_000_000  # earliest AU row (2 s before the window opens)
T1 = 129_500_000  # latest AU row (after the window closes)

# The scheduler's own stamps for one step: t0 = first non-status command,
# t1 = the 'status' command. Window = [t0+5e6, t1-1e6) per the brief.
STEP_T0 = 100_000_000
STEP_T1 = 130_000_000
WIN_A = STEP_T0 + 5_000_000
WIN_B = STEP_T1 - 1_000_000

BASELINE_DIFF_US = 30_000   # t_complete - pts, steady state
EXCESS_T = 110_000_000      # +15 ms vs the rolling median, inside the window
EXCESS_DIFF_US = BASELINE_DIFF_US + 15_000
MISSING_FRAME_T = 115_000_000  # pts left out of frame.log, inside the window
TRUNC_IN_WINDOW_TS = {106_000_000, 107_000_000, 108_000_000}
TRUNC_OUT_WINDOW_TS = {103_000_000, 104_000_000}


def write_session(sess_dir):
    os.makedirs(sess_dir, exist_ok=True)
    times = list(range(T0, T1 + 1, STEP_US))
    au_lines = ['# aulog 7']
    frame_lines = ['# framelog 3']
    fid = 0
    rows = []
    for t in times:
        pts = t - PTS_OFFSET
        diff = EXCESS_DIFF_US if t == EXCESS_T else BASELINE_DIFF_US
        tc = pts + diff
        tf = tc - 5000
        truncated = t in TRUNC_IN_WINDOW_TS or t in TRUNC_OUT_WINDOW_TS
        flags = 0x00 if truncated else 0x80
        ln = 6640
        au_lines.append(
            f"{t} {pts} 0 {fid} {ln} {hex(flags)} 1 {tf} {tc} 0 0 0 0 0 0 0 {tc} 0 0 0")
        rows.append((t, pts, fid, truncated))
        if t != MISSING_FRAME_T:
            e2e_us = 20_000
            frame_lines.append(
                f"0 {pts} {fid} 0 {ln} 0x0 0 0 0 0 0 0 0 0 0 0 0 0 {e2e_us} 1 0 0x0")
        fid += 1
    with open(os.path.join(sess_dir, 'au.log'), 'w') as f:
        f.write('\n'.join(au_lines) + '\n')
    with open(os.path.join(sess_dir, 'frame.log'), 'w') as f:
        f.write('\n'.join(frame_lines) + '\n')
    return rows


def write_run_file(path):
    with open(path, 'w') as f:
        f.write("=== A binary=/x config=/y 00:00:00\n")
        f.write("--- step l2b1\n")
        f.write(f"mono_us={STEP_T0} cmd='off' reply=ok\n")
        f.write(f"mono_us={STEP_T0 + 50} cmd='s0 eff=2 burst=1' reply=ok\n")
        f.write(f"mono_us={STEP_T0 + 100} cmd='s1 eff=2 burst=1' reply=ok\n")
        f.write(f"mono_us={STEP_T1} cmd='status' reply=ok\n")
        f.write("session /media/dvr/log/0100\n")


def write_snap(path, uptime, cpu_line, threads):
    with open(path, 'w') as f:
        f.write(f"SNAP {uptime:.6f} pid=1234\n")
        f.write(f"cpu  {cpu_line}\n")
        for tid, comm, utime, stime in threads:
            f.write(f"T {tid} {comm} {utime} {stime}\n")


def main():
    outdir = tempfile.mkdtemp()
    sessroot = tempfile.mkdtemp()
    sess_dir = os.path.join(sessroot, '0100')
    rows = write_session(sess_dir)
    write_run_file(os.path.join(outdir, 'A.run'))
    write_snap(os.path.join(outdir, 'A.l2b1.snap0'), 1000.0,
               "1000 0 1000 8000 0 0 0 0 0 0",
               [('100', 'mbr-hot', 1000, 0)])
    write_snap(os.path.join(outdir, 'A.l2b1.snap1'), 1001.0,
               "1010 0 1010 8070 10 0 0 0 0 0",
               [('100', 'mbr-hot', 1025, 25)])

    built = analyze.build_rows(outdir, sessroot)
    assert len(built) == 1, built
    row = built[0]
    assert row['arm'] == 'A' and row['step'] == 'l2b1' and row['sess'] == '0100', row

    in_window = [r for r in rows if WIN_A <= r[0] < WIN_B]
    out_window_trunc = [r for r in rows if not (WIN_A <= r[0] < WIN_B) and r[3]]
    assert out_window_trunc, "fixture must have a truncated AU outside the window"
    in_window_trunc = [r for r in in_window if r[3]]
    assert in_window_trunc, "fixture must have a truncated AU inside the window"

    # trunc% counts only the in-window flag-less-0x80 rows.
    expected_trunc_pct = 100.0 * len(in_window_trunc) / len(in_window)
    assert abs(row['trunc'] - expected_trunc_pct) < 1e-6, (row['trunc'], expected_trunc_pct)
    assert row['n'] == len(in_window), (row['n'], len(in_window))

    # The +15 ms AU trips >10ms% but not >20ms%.
    assert row['gt10'] > 0.0, row['gt10']
    assert row['gt20'] == 0.0, row['gt20']
    expected_gt10 = 100.0 / len(in_window)
    assert abs(row['gt10'] - expected_gt10) < 1e-6, (row['gt10'], expected_gt10)

    # nodisp% counts the one in-window pts missing from frame.log (gap=0
    # here: the window's fids are a contiguous run).
    assert row['gap'] == 0, row['gap']
    expected_nodisp = 100.0 * (1.0 - (len(in_window) - 1) / len(in_window))
    assert abs(row['nodisp'] - expected_nodisp) < 1e-6, (row['nodisp'], expected_nodisp)
    assert row['nodisp'] > 0.0, row['nodisp']

    # Per-thread CPU: +50 ticks over a 1 s uptime delta is 50.0.
    assert row['cpu'].get('hot') == 50.0, row['cpu']

    print('✓ fecab test passed!')


if __name__ == '__main__':
    main()
