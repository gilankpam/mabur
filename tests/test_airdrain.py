#!/usr/bin/env python3
"""Test for tools/bench/airdrain.py (per-frame air-excess replay around rung
transitions, docs/probe-stream-flight-findings-2026-09-05.md section 9).

Synthesizes one boot's ctl-NNNN log + au-NNNN log with a known ground truth:
a cold climb, a 2-step cascade at t=60 s whose air excess ramps to 120 ms at
+0.8 s and drains by +2.0 s (with two IDRs), a single demote at t=90 s with
a 40 ms one-frame bump, a promote at t=110 s with none, a drone restart
(pts discontinuity) at t=130 s that must NOT read as a 100 s spike, and a
starved end. Checks the cascade metrics, the pre-demote excess, the
discontinuity reset and the steady-state rung table.
"""
import os, sys, tempfile
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'tools', 'bench'))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'tools'))
import airdrain

PERIOD_MS = 1000.0 / 60.0
PTS_OFF_MS = 14900.0   # GS mono - drone pts, like flights 20/21
FLOOR_MS = 3.0         # first-body transit floor


def excess_at(t_ms):
    """Ground-truth per-frame air excess (ms) at GS time t_ms."""
    dt = t_ms - 60000.0
    if 0.0 <= dt <= 800.0: return 120.0 * dt / 800.0
    if 800.0 < dt <= 2000.0: return 120.0 * (2000.0 - dt) / 1200.0
    if 90000.0 <= t_ms < 90000.0 + PERIOD_MS: return 40.0
    return 0.0


def write_logs(d, model=True):
    ctl = os.path.join(d, 'ctl-0007_20260905.log')
    au = os.path.join(d, 'log', 'au-0003.log')
    os.makedirs(os.path.dirname(au), exist_ok=True)
    E = [(10000, 0, 1, 'promote_probed'), (13000, 1, 2, 'promote_probed'),
         (16000, 2, 3, 'promote_probed'), (19000, 3, 4, 'promote_probed'),
         (22000, 4, 5, 'promote_probed'),
         (60000, 5, 4, 's3_residual'), (60300, 4, 3, 's3_residual'),   # the cascade
         (90000, 3, 2, 'residual'),                                    # single
         (110000, 2, 3, 'promote_probed'),
         (150000, 3, 0, 'starved'),                                    # link loss mid-flight...
         (155000, 0, 1, 'promote_probed')]                             # ...and the re-climb after it
    with open(ctl, 'w') as f:
        f.write('ctllog 10 ladder=0/100:50,1/100:50,2/100:50,3/100:50,4/100:50,5/100:50 '
                'down_util=0.35 up_util=0.15 probe_offset=1\n')
        ei = 0; rung = 0
        for t in range(5000, 162000, 1000):
            while ei < len(E) and E[ei][0] <= t:
                f.write(f'E {E[ei][0]} {E[ei][1]} {E[ei][2]} {E[ei][3]} 0.0000 30.0 -20.0\n')
                rung = E[ei][2]; ei += 1
            f.write(f'S {t} {rung} 0.0000 30.0 0.0000 0.0000 0.0000 -20.0 0.0000 0.0 0.0 -50.0 -1 nan 0\n')
    with open(au, 'w') as f:
        f.write('# aulog 3\n' if model else '# aulog 2\n')
        fid = 0; t = 5000.0
        while t < 162000.0:
            sid = fid & 1
            pts = t - PTS_OFF_MS + (100000.0 if t >= 130000.0 else 0.0)  # restart: pts jumps +100 s
            enc = 7000; q = 1
            idr = t in (60110.0,) or (60100.0 <= t < 60100.0 + PERIOD_MS) or (60400.0 <= t < 60400.0 + PERIOD_MS)
            length = 80000 if idr else (30000 if sid == 0 else 20000)
            tf = t + FLOOR_MS + enc / 1000.0 + q + excess_at(t)
            tc = tf + 12.0
            line = (f'{int(1.5e15 + t * 1000)} {int(pts * 1000)} {sid} {fid} {length} '
                    f'{"0x81" if idr else "0x80"} 1 {int(tf * 1000)} {int(tc * 1000)} {enc} {q}')
            if model:
                line += f' {int(0.8 * excess_at(t))}'
            f.write(line + '\n')
            fid += 1; t += PERIOD_MS
    return ctl, au


def approx(a, b, tol):
    return abs(a - b) <= tol


def main():
    d = tempfile.mkdtemp()
    ctl, au = write_logs(d)
    r = airdrain.analyze(ctl, au)
    # The window runs to the LAST S line: a starve is link loss, not the end
    # of the flight (flight 0031 read as 72 s of 566 when it stopped at the
    # first starved E line, 2026-09-06).
    assert r['t0'] == 10000 and r['t1'] == 161000, (r['t0'], r['t1'])

    # The cascade: one episode, 5->3 in 2 steps, peak ~120 ms at ~+800 ms,
    # drained by ~+2000 ms, nothing before it, two IDRs in the bins.
    assert len(r['cascades']) == 1, r['cascades']
    c = r['cascades'][0]
    assert (c['from'], c['to'], c['steps'], c['first_reason']) == (5, 3, 2, 's3_residual'), c
    assert approx(c['peak_ms'], 120.0, 3.0), c['peak_ms']
    # Times are ARRIVAL-referenced (t_first), so the peak frame lands its own
    # 120 ms excess + floor + enc + q after its capture-time peak at +800.
    arrival_shift = 120.0 + FLOOR_MS + 7.0 + 1.0
    assert approx(c['time_to_peak_ms'], 800.0 + arrival_shift, PERIOD_MS + 1), c['time_to_peak_ms']
    assert 1850.0 <= c['settle_ms'] <= 2150.0, c['settle_ms']   # < 15 ms from +1850 ms (capture) on
    assert c['pre_ms'] < 1.0, c['pre_ms']
    idrs = [i for _, i in c['bins'] if i]
    assert len(idrs) == 2 and all(i == [80] for i in idrs), idrs
    # First 500 ms after the demote at 60 fps of 30/20 kB frames, ov 1.0/0.5,
    # against rung 3 = mcs3 nominal 26 Mb/s: (15*30*2 + 15*20*1.5 + IDR extra)
    # kB vs 1625 kB -- well under 100 %, and finite.
    assert 60.0 <= c['onair_pct'] <= 100.0, c['onair_pct']

    # Single demote: one 40 ms frame, nothing before. Promote: nothing at all.
    assert len(r['singles']) == 1 and approx(r['singles'][0]['peak_ms'], 40.0, 1.0), r['singles']
    assert r['singles'][0]['pre_ms'] < 1.0
    assert len(r['promotes']) == 7, len(r['promotes'])           # 5 climb + 110 s + the post-starve 155 s
    late = [p for p in r['promotes'] if p['t0'] == 110000]
    assert late and late[0]['peak_ms'] < 1.0, late

    # The drone restart at 130 s (pts +100 s) must re-seed the floor, not
    # read as a 100 s excess: the whole-flight max is the cascade's peak.
    assert max(r['excess']) < 125.0, max(r['excess'])

    # Steady state: rung 5 (22..60 s) and rung 3 (from 113 s on) flat at ~0.
    assert approx(airdrain.pct(r['steady'][5], .5), 0.0, 0.5), r['steady'][5][:5]
    assert approx(airdrain.pct(r['steady'][3], .99), 0.0, 0.5)
    # Spike seconds: the drain covers t=60..62 -> 2 seconds with median >= 15,
    # none of them standalone (a transition is within 3 s).
    assert r['spike_secs'] == 2, r['spike_secs']
    assert r['standalone'] == [], r['standalone']

    # print_report must not throw on this input (smoke).
    import contextlib, io
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        airdrain.print_report(r, ctl, au, profiles=True)
    out = buf.getvalue()
    assert 'cascade summary: peak p50=120' in out, out

    # --model: aulog-3 air_ms column vs measured air + q. The fixture's model
    # is 0.8x the truth, so the through-origin slope (measured/model) reads
    # 1/0.8 plus the q=1 ms offset spread over big frames.
    m = r['model']
    assert m is not None and m['n'] == r['n_au'], m
    assert 1.20 <= m['slope'] <= 1.32, m['slope']
    assert m['quiet_model_p99'] == 0, m['quiet_model_p99']          # no phantom backlog
    assert len(m['cascades']) == 1
    t0, pk_model, pk_meas = m['cascades'][0]
    assert approx(pk_model, 96.0, 2.0) and approx(pk_meas, 121.0, 3.0), m['cascades']

    # aulog-2 (no air_ms column): model comparison is absent, report doesn't throw.
    d2 = tempfile.mkdtemp()
    ctl2, au2 = write_logs(d2, model=False)
    r2 = airdrain.analyze(ctl2, au2)
    assert r2['model'] is None, r2['model']
    buf2 = io.StringIO()
    with contextlib.redirect_stdout(buf2):
        airdrain.print_report(r2, ctl2, au2, model_flag=True)

    print('✓ airdrain test passed!')


if __name__ == '__main__':
    main()
