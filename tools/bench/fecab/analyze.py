#!/usr/bin/env python3
"""tools/bench/fecab/analyze.py -- per arm x step metrics for the FEC
placement A/B rig (tools/bench/fecab/run.sh). usage: analyze.py OUTDIR
SESSROOT

OUTDIR holds run.sh's own output: one <name>.run file per arm (the step
boundaries, as mono_us command stamps catted in from the GS's sched.log)
and a pair of drone CPU snapshot files per arm x step
(<name>.<step>.snap0/.snap1). SESSROOT holds the GS session directories
(au.log, frame.log) that run.sh's arms wrote into, named as the basename
of the "session <dir>" line the GS prints.

A step's analysis window is [first command's mono_us + 5 s, 'status'
mono_us - 1 s] on au.log's t_us, so it covers only the loss level that was
actually in force (sched.py sleeps STEP seconds between the last loss
command and 'status').
"""
import glob
import os
import re
import statistics as st
import sys
from collections import defaultdict, deque

SKIP_US = 5_000_000   # window opens 5 s after the step's first command
TAIL_US = 1_000_000   # window closes 1 s before 'status'


def pct(values, p):
    """Nearest-rank p-th percentile of values (0 <= p <= 1), nan if empty."""
    if not values:
        return float('nan')
    v = sorted(values)
    return v[min(len(v) - 1, int(p * len(v)))]


class BadRunLog(RuntimeError):
    """Raised by _parse_run_file when a .run file's step/mono_us stamps
    are corrupt: a repeated `--- step` name, or a mono_us stamp that goes
    backwards. The usual cause is an ssh-retried final `cat
    /tmp/sched.log` (run.sh) duplicating the whole schedule into the log
    -- the second copy's first stamp lands before the first copy's last
    one, and every step name repeats. find_runs() catches this per file
    and reports the arm as failed instead of windowing the doubled-up
    data."""


def _parse_run_file(path):
    """Parse one OUTDIR/*.run file: `tag` is its basename minus .run (the
    arm name); `sess` is the basename of the GS's "session <dir>" line;
    `steps` is a list of {"name", "t0", "t1"} -- t0 the mono_us of the
    step's first scheduler command, t1 the mono_us of its 'status'
    command. Raises BadRunLog on a repeated step name or a non-monotonic
    mono_us stamp (see BadRunLog's docstring)."""
    tag = os.path.basename(path)[:-len(".run")]
    sess, steps, cur = None, [], None
    seen_names, prev_mono = set(), None
    with open(path) as f:
        for line in f:
            m = re.match(r"session (\S+)", line)
            if m and sess is None:
                sess = os.path.basename(m.group(1).rstrip('/'))
                continue
            m = re.match(r"--- step (\S+)", line)
            if m:
                name = m.group(1)
                if name in seen_names:
                    raise BadRunLog(f"{path}: repeated step name {name!r}")
                seen_names.add(name)
                cur = {"name": name, "t0": None, "t1": None}
                steps.append(cur)
                continue
            m = re.match(r"mono_us=(\d+) cmd='(.*?)'", line)
            if m and cur is not None:
                t = int(m.group(1))
                if prev_mono is not None and t < prev_mono:
                    raise BadRunLog(f"{path}: mono_us went backwards ({t} < {prev_mono})")
                prev_mono = t
                if m.group(2) == "status":
                    cur["t1"] = t
                elif cur["t0"] is None:
                    cur["t0"] = t
    return tag, sess, steps


def find_runs(outdir):
    """Yield (tag, session, steps) for every OUTDIR/*.run file that parses
    cleanly (see _parse_run_file); a file that fails BadRunLog's checks is
    reported to stderr and skipped -- that arm is left out of build_rows'
    output entirely rather than windowed on doubled-up data."""
    for path in sorted(glob.glob(os.path.join(outdir, "*.run"))):
        try:
            yield _parse_run_file(path)
        except BadRunLog as e:
            print(f"FAILED: {e}", file=sys.stderr)
            continue


def load_au(session_dir):
    """au.log rows (`# aulog 7`, 20 columns, docs/observability.md):

        t_us pts sid fid len flags nal0 t_first t_complete enc dq air
        slices kept filled after_hole t_last t_drain t_push vcl

    Adds `ex`: (t_complete - pts) minus the rolling 2 s median of the same
    quantity on the same sid, in ms -- the per-frame completion excess
    over the recent steady state."""
    rows = []
    with open(os.path.join(session_dir, "au.log")) as f:
        for line in f:
            if line.startswith("#"):
                continue
            c = line.split()
            if len(c) < 20:
                continue
            rows.append(dict(
                t=int(c[0]), pts=int(c[1]), sid=int(c[2]), fid=int(c[3]),
                ln=int(c[4]), fl=int(c[5], 16),
                t_first=int(c[7]), t_complete=int(c[8]),
            ))
    for sid in (0, 1):
        same = [x for x in rows if x["sid"] == sid]
        q = deque()
        for x in same:
            d = x["t_complete"] - x["pts"]
            q.append((x["t"], d))
            while q[0][0] < x["t"] - 2_000_000:
                q.popleft()
            x["ex"] = (d - st.median(v for _, v in q)) / 1000.0
    return rows


def load_frames(session_dir):
    """pts -> e2e ms, from frame.log (`# framelog 3`, 22 columns,
    docs/observability.md); column 19 (1-indexed) is e2e, in us. None if
    frame.log itself is missing -- distinct from an empty-but-present {},
    which step_row's caller must NOT read as "every pts in the window was
    dropped"."""
    path = os.path.join(session_dir, "frame.log")
    if not os.path.exists(path):
        return None
    out = {}
    with open(path) as f:
        for line in f:
            if line.startswith("#"):
                continue
            c = line.split()
            if len(c) < 19:
                continue
            out[int(c[1])] = int(c[18]) / 1000.0
    return out


def tr(window, sid):
    """Truncation %% (flags lacking kRecFlagComplete, gs/src/au_ring.h
    0x80) restricted to one sid, nan if that sid has no rows in window."""
    v = [x for x in window if x["sid"] == sid]
    if not v:
        return float('nan')
    return 100.0 * sum(1 for x in v if not x["fl"] & 0x80) / len(v)


def step_row(window, frame_map, span_us):
    """Metrics for one arm x step's AU window (non-empty)."""
    n = len(window)
    trunc = sum(1 for x in window if not x["fl"] & 0x80)
    # fid is a GLOBAL counter across all UEP layers, one id per AU and
    # never shared between a frame's base and enh AU
    # (common/include/mabur/frame_wire.h), so counting ids missing from
    # the window across BOTH layers combined is exact, not an
    # approximation.
    fids = sorted(x["fid"] for x in window)
    gap = sum(max(0, b - a - 1) for a, b in zip(fids, fids[1:]))
    ex = [x["ex"] for x in window]
    mbps = sum(x["ln"] for x in window) * 8 / (span_us / 1e6) / 1e6
    # frame_map is None when frame.log itself is missing -- report e2e/
    # nodisp as n/a rather than reading a missing file as "every pts
    # dropped" (100 % nodisp).
    if frame_map is None:
        e2e = []
        e2e50 = e2e99 = nodisp = float('nan')
    else:
        e2e = [frame_map[x["pts"]] for x in window if x["pts"] in frame_map]
        nodisp = 100.0 * (1.0 - len(e2e) / (n + gap)) if (n + gap) else float('nan')
        e2e50, e2e99 = pct(e2e, .5), pct(e2e, .99)
    return dict(
        n=n, mbps=mbps, trunc=100.0 * trunc / n, gap=gap,
        ex50=pct(ex, .5), ex99=pct(ex, .99), ex999=pct(ex, .999),
        gt10=100.0 * sum(1 for e in ex if e > 10) / n,
        gt20=100.0 * sum(1 for e in ex if e > 20) / n,
        tr0=tr(window, 0), tr1=tr(window, 1),
        symf=st.mean(x["ln"] for x in window) / 332,
        e2e50=e2e50, e2e99=e2e99, nodisp=nodisp,
    )


def read_snap(path):
    """One drone CPU snapshot (snap.sh's output): uptime (s), maburd's
    pid at snap time (snap.sh's `SNAP <uptime> pid=<pid>`, None if that
    field is missing), the /proc/stat `cpu ` line's 10 counters, and
    {tid: (comm, utime+stime)} per task."""
    uptime, pid, sysline, threads = None, None, None, {}
    with open(path) as f:
        for line in f:
            if line.startswith("SNAP"):
                parts = line.split()
                uptime = float(parts[1])
                pid = next((p.split("=", 1)[1] for p in parts
                            if p.startswith("pid=")), None)
            elif line.startswith("cpu "):
                sysline = [int(x) for x in line.split()[1:]]
            elif line.startswith("T "):
                _, tid, name, u, s = line.split()
                threads[tid] = (name, int(u) + int(s))
    return uptime, pid, sysline, threads


_CPU_NA = dict(hot=float('nan'), fecw=float('nan'), venc=float('nan'),
               txw=float('nan'), maburd=float('nan'), sys=float('nan'))


def cpu(prefix):
    """Per-thread CPU%% between prefix.snap0 and prefix.snap1: per-thread
    (utime+stime) delta / (uptime delta x 100 ticks) x 100, summed per
    comm, named for the four worker threads (mbr-hot, mbr-fecw, venc-enc,
    mbr-txw), `maburd` = every thread's share, and `sys` from the
    /proc/stat idle+iowait share. None if either snapshot is missing; all
    columns n/a (not a misleadingly low number) if maburd's pid differs
    between the two snapshots -- a restart mid-step resets the tids the
    diff is keyed on, so the delta would mostly read as near-zero CPU
    rather than "unknown"."""
    try:
        u0, pid0, s0, t0 = read_snap(prefix + ".snap0")
        u1, pid1, s1, t1 = read_snap(prefix + ".snap1")
    except (FileNotFoundError, OSError):
        return None
    if pid0 != pid1:
        return dict(_CPU_NA)
    dt = (u1 - u0) * 100  # ticks elapsed, assuming the standard 100 Hz clock tick
    by = defaultdict(float)
    for tid, (name, v1) in t1.items():
        if tid in t0:
            by[name] += (v1 - t0[tid][1]) / dt * 100
    maburd = sum(by.values())
    ds = [b - a for a, b in zip(s0, s1)]
    sys_pct = 100.0 * (1 - (ds[3] + ds[4]) / sum(ds))
    return dict(hot=by.get("mbr-hot", 0.0), fecw=by.get("mbr-fecw", 0.0),
                venc=by.get("venc-enc", 0.0), txw=by.get("mbr-txw", 0.0),
                maburd=maburd, sys=sys_pct)


def build_rows(outdir, sessroot):
    """One row per arm x step: the window's AU/frame metrics plus that
    window's CPU snapshot pair, if any."""
    rows = []
    cache = {}
    for tag, sess, steps in find_runs(outdir):
        if not sess:
            continue
        d = os.path.join(sessroot, sess)
        if not os.path.exists(os.path.join(d, "au.log")):
            continue
        if sess not in cache:
            cache[sess] = (load_au(d), load_frames(d))
        au, fr = cache[sess]
        for s in steps:
            if s["t0"] is None or s["t1"] is None:
                continue
            a, b = s["t0"] + SKIP_US, s["t1"] - TAIL_US
            window = [x for x in au if a <= x["t"] < b]
            if not window:
                continue
            row = step_row(window, fr, b - a)
            row["arm"] = tag
            row["step"] = s["name"]
            row["sess"] = sess
            row["cpu"] = cpu(os.path.join(outdir, f"{tag}.{s['name']}")) or {}
            rows.append(row)
    return rows


_DLOG_RE = re.compile(
    r"maburd fec_worker sid=\d+: .*backlog_drops=(\d+) frame_end=(\d+) "
    r"frame_end_us/rep=(\d+)")


def parse_dlog(path):
    """One arm's drone log (run.sh's OUTDIR/<name>.dlog, /tmp/mabur.log
    read at the end of the arm): sums `backlog_drops=` across every
    `fec_worker sid=` line (both sids, every 5 s cadence tick) and the
    frame_end-weighted mean of `frame_end_us/rep=` -- each line's own
    mean is already an average over that line's `frame_end=` repairs
    (docs/observability.md "fec_worker"), so the overall mean weights by
    that count rather than treating every line as equal-sized.
    (0, nan) if the log has no matching line."""
    backlog_drops = 0
    weighted_us, total_frame_end = 0, 0
    with open(path) as f:
        for line in f:
            m = _DLOG_RE.search(line)
            if not m:
                continue
            backlog_drops += int(m.group(1))
            frame_end = int(m.group(2))
            us_per_rep = int(m.group(3))
            weighted_us += frame_end * us_per_rep
            total_frame_end += frame_end
    mean_us = weighted_us / total_frame_end if total_frame_end else float('nan')
    return backlog_drops, mean_us


def dlog_summary(outdir):
    """One (tag, backlog_drops, frame_end_us_per_rep) row per
    OUTDIR/*.dlog file, tag = the arm name (the file's basename minus
    .dlog)."""
    rows = []
    for path in sorted(glob.glob(os.path.join(outdir, "*.dlog"))):
        tag = os.path.basename(path)[:-len(".dlog")]
        drops, us = parse_dlog(path)
        rows.append((tag, drops, us))
    return rows


HEADER = (
    f"{'arm':4s} {'step':6s} {'sess':5s} {'n':>5s} {'Mbps':>5s} {'trunc%':>6s} {'gap':>4s} "
    f"{'ex50':>5s} {'ex99':>5s} {'ex999':>6s} {'>10ms%':>6s} {'>20ms%':>6s} "
    f"{'tr0%':>5s} {'tr1%':>5s} {'sym/f':>5s} {'e2e50':>5s} {'e2e99':>5s} {'nodisp%':>7s} | "
    f"{'hot%':>5s} {'fecw%':>5s} {'venc%':>5s} {'txw%':>5s} {'maburd%':>7s} {'sys%':>5s}"
)


def format_row(r):
    c = r["cpu"]
    cpu_str = " ".join(
        f"{c.get(k, float('nan')):{w}.1f}"
        for k, w in (("hot", 5), ("fecw", 5), ("venc", 5), ("txw", 5),
                     ("maburd", 7), ("sys", 5)))
    return (
        f"{r['arm']:4s} {r['step']:6s} {r['sess']:5s} {r['n']:5d} {r['mbps']:5.1f} "
        f"{r['trunc']:6.2f} {r['gap']:4d} {r['ex50']:5.1f} {r['ex99']:5.1f} {r['ex999']:6.1f} "
        f"{r['gt10']:6.2f} {r['gt20']:6.2f} {r['tr0']:5.2f} {r['tr1']:5.2f} {r['symf']:5.0f} "
        f"{r['e2e50']:5.1f} {r['e2e99']:5.1f} {r['nodisp']:7.2f} | {cpu_str}"
    )


def main(argv):
    if len(argv) != 3:
        print(f"usage: {argv[0]} OUTDIR SESSROOT", file=sys.stderr)
        return 2
    outdir, sessroot = argv[1], argv[2]
    print(HEADER)
    for row in build_rows(outdir, sessroot):
        print(format_row(row))
    dlogs = dlog_summary(outdir)
    if dlogs:
        print()
        print(f"{'arm':4s} {'backlog_drops':>13s} {'frame_end_us/rep':>16s}")
        for tag, drops, us in dlogs:
            us_str = f"{us:16.0f}" if us == us else f"{'n/a':>16s}"
            print(f"{tag:4s} {drops:13d} {us_str}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
