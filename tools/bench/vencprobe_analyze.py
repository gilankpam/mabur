#!/usr/bin/env python3
"""Analyze a tools/bench/vencprobe capture: how long after a bitrate command
does the encoder's output actually change?

Per commanded step it reports:
  first    — the first frame whose CAPTURE instant (pts) is after the
             command, i.e. the earliest frame the command could possibly
             affect, and how much its size moved. If this frame has already
             moved, the encoder applied the command within one frame
             interval and the lag is a capture-phase lag, not encoder work.
  onset    — first frame past the midpoint between the pre-step and
             post-step size levels, in ms after the command returned.
  settle   — first time the 8-frame moving mean enters (and stays within)
             +-15% of the window's steady tail level.

Windows are truncated at the first /venc poll showing a req_bitrate the
probe did not command: that is RcAgent's 5 s re-assert stepping on the
override, and everything after it describes RcAgent, not the encoder.

Base and enhance frames are sized differently (SVC-T), so all size
comparisons are made within a class (flags & 0x04). IDR frames (flags &
0x01) are excluded — a GOP boundary is a size outlier unrelated to the step.

  vencprobe_analyze.py vp.csv
"""
import sys
from statistics import mean, median

FLAG_IDR = 0x01
FLAG_ENH = 0x04


def load(path):
    frames, cmds, polls = [], [], []
    for line in open(path):
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        p = line.split(',')
        if p[0] == 'f':
            frames.append(dict(t=int(p[1]), idx=int(p[2]), len=int(p[3]),
                               pts=int(p[4]), flags=int(p[5]), enc=int(p[6])))
        elif p[0] == 'c':
            cmds.append(dict(tb=int(p[1]), ta=int(p[2]), kbps=int(p[3]),
                             ok=int(p[4])))
        elif p[0] == 's':
            # 3rd column (encoder qp) existed for a few hours on 2026-09-03
            # and was always 0 (the SDK never fills startQual); tolerated.
            polls.append(dict(t=int(p[1]), kbps=int(p[2]),
                              qp=int(p[3]) if len(p) > 3 else None))
    return frames, cmds, polls


def klass(f):
    return 'enh' if f['flags'] & FLAG_ENH else 'base'


def main():
    frames, cmds, polls = load(sys.argv[1])
    usable = [f for f in frames if not (f['flags'] & FLAG_IDR)]
    print(f"{len(frames)} frames ({len(frames)-len(usable)} IDR excluded), "
          f"{len(cmds)} commands, {len(polls)} polls\n")

    onsets, firsts, settles = [], [], []
    for i, c in enumerate(cmds):
        t = c['ta']
        end = cmds[i + 1]['tb'] if i + 1 < len(cmds) else frames[-1]['t']
        # Truncate at RcAgent's re-assert.
        clipped = None
        for p in polls:
            if t + 50_000 < p['t'] < end and p['kbps'] != c['kbps'] and p['kbps'] > 0:
                clipped, end = p['kbps'], p['t']
                break

        pre = {k: [f['len'] for f in usable
                   if t - 600_000 <= f['t'] < t and klass(f) == k]
               for k in ('base', 'enh')}
        win = [f for f in usable if t <= f['t'] < end]
        tail0 = t + int(0.6 * (end - t))
        post = {k: [f['len'] for f in win if f['t'] >= tail0 and klass(f) == k]
                for k in ('base', 'enh')}
        note = f"  [clipped by RcAgent -> {clipped} kbps at "\
               f"+{(end-t)/1000:.0f} ms]" if clipped else ""
        print(f"step {i}: -> {c['kbps']:5d} kbps  "
              f"(http {(c['ta']-c['tb'])/1000:.1f} ms){note}")
        if not all(pre[k] and len(post[k]) >= 3 for k in ('base', 'enh')):
            print("   window too short after clipping — skipped\n")
            continue

        mid = {k: (mean(pre[k]) + mean(post[k])) / 2 for k in ('base', 'enh')}
        down = mean(post['base']) < mean(pre['base'])
        moved = lambda f: (f['len'] < mid[klass(f)]) if down else \
                          (f['len'] > mid[klass(f)])

        print(f"   pre  base {mean(pre['base'])/1000:6.1f} kB  "
              f"enh {mean(pre['enh'])/1000:6.1f} kB    "
              f"post base {mean(post['base'])/1000:6.1f} kB  "
              f"enh {mean(post['enh'])/1000:6.1f} kB")

        # The first frame that could possibly carry the new rate.
        cand = [f for f in win if f['pts'] > t]
        if cand:
            f = cand[0]
            delta = f['len'] / mean(pre[klass(f)]) - 1.0
            firsts.append(((f['t'] - t) / 1000, moved(f)))
            print(f"   first  captured +{(f['pts']-t)/1000:5.1f} ms, committed "
                  f"+{(f['t']-t)/1000:5.1f} ms, {delta*100:+6.1f}% vs pre "
                  f"{klass(f)}  -> {'MOVED' if moved(f) else 'not yet'}")

        onset = next((f for f in win if moved(f)), None)
        if onset:
            n = win.index(onset)
            onsets.append((onset['t'] - t) / 1000)
            print(f"   onset  +{(onset['t']-t)/1000:5.1f} ms committed "
                  f"({n} frames into the window)")
        else:
            print("   onset  never crossed the midpoint")

        steady = mean([f['len'] for f in win if f['t'] >= tail0])
        settle = None
        for n in range(len(win) - 8):
            if all(abs(mean([g['len'] for g in win[m:m + 8]]) - steady)
                   <= 0.15 * steady for m in range(n, len(win) - 8)):
                settle = win[n + 7]
                break
        if settle:
            settles.append((settle['t'] - t) / 1000)
            print(f"   settle +{(settle['t']-t)/1000:5.1f} ms")
        else:
            print("   settle not reached inside the window")
        print()

    if onsets:
        already = sum(1 for _, m in firsts if m)
        print(f"SUMMARY over {len(onsets)} measured steps")
        print(f"  first frame captured after the command already moved: "
              f"{already}/{len(firsts)}")
        print(f"  onset  median {median(onsets):.1f} ms  "
              f"(min {min(onsets):.1f}, max {max(onsets):.1f})")
        if settles:
            print(f"  settle median {median(settles):.0f} ms  "
                  f"(min {min(settles):.0f}, max {max(settles):.0f}, "
                  f"n={len(settles)})")


if __name__ == '__main__':
    main()
