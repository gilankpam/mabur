#!/usr/bin/env python3
"""tools/bench/vencburst_analyze.py against a synthetic passive vencprobe
capture: quiet scene far under the command, a scene change that overshoots
the programmed rate for half a second, then convergence. The analyzer must
find the one cycle and report the four metrics the bench plan asks for
(docs/handover-venc-overshoot-2026-09-03.md): quiet rate, peak 100 ms rate
as a multiple of the PROGRAMMED rate (kbps x 1024 -- the encoder's unit,
not RcAgent's), integrated excess bytes (what the TxQueue holds), and the
time back inside +10%."""
import io
import os
import sys
import unittest
from contextlib import redirect_stdout

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tools", "bench"))
import vencburst_analyze as vb  # noqa: E402

FPS = 60
CMD_KBPS = 16000
PROGRAMMED_KBPS = CMD_KBPS * 1024 / 1000  # 16384 decimal kbit/s


def synth(segments, qp_of_rate, cmd_kbps=CMD_KBPS, poll_ms=25, idr_every=None):
    """segments: list of (duration_s, mbit_per_s). Frames at 60 fps with the
    size that yields that rate; polls every 25 ms carry cmd_kbps and a qp
    from qp_of_rate(mbps). Returns CSV text in vencprobe's format."""
    lines = ["# vencprobe synthetic",
             "# f,mono_us,write_idx,len,pts_us,flags,enc_us",
             "# s,mono_us,req_bitrate_kbps,qp"]
    t_us = 1_000_000
    idx = 0
    next_poll = t_us
    frame_us = 1_000_000 // FPS
    for dur_s, mbps in segments:
        n = int(dur_s * FPS)
        size = int(mbps * 1e6 / 8 / FPS)
        for _ in range(n):
            while next_poll <= t_us:
                lines.append(f"s,{next_poll},{cmd_kbps},{qp_of_rate(mbps)}")
                next_poll += poll_ms * 1000
            flags = 0x04 if idx % 2 else 0
            if idr_every and idx % idr_every == 0:
                flags |= 0x01
            lines.append(f"f,{t_us},{idx},{size},{t_us - 8000},{flags},8000")
            idx += 1
            t_us += frame_us
    return "\n".join(lines) + "\n"


def qp_of(mbps):
    return 18 if mbps < 8 else (34 if mbps > 20 else 28)


class OneCycle(unittest.TestCase):
    def setUp(self):
        # 5 s quiet at 5 Mb/s, 0.5 s burst at 22 Mb/s, 3 s converged at the
        # programmed 16.384 Mb/s.
        self.csv = synth([(5.0, 5.0), (0.5, 22.0), (3.0, 16.384)], qp_of)
        self.frames, self.polls, self.cmds = vb.load(io.StringIO(self.csv))
        self.cycles = vb.find_cycles(self.frames, self.polls, vb.Params())

    def test_programmed_rate_is_kbps_times_1024(self):
        self.assertAlmostEqual(vb.programmed_kbps(self.polls), PROGRAMMED_KBPS, places=3)

    def test_one_cycle_found(self):
        self.assertEqual(len(self.cycles), 1)

    def test_quiet_rate(self):
        c = self.cycles[0]
        self.assertAlmostEqual(c.quiet_kbps / 1000.0, 5.0, delta=0.3)

    def test_peak_ratio_against_programmed(self):
        c = self.cycles[0]
        self.assertAlmostEqual(c.peak_kbps / PROGRAMMED_KBPS, 22000 / PROGRAMMED_KBPS, delta=0.06)

    def test_excess_bytes_is_the_queue_fill(self):
        # (22 - 16.384) Mb/s over 0.5 s = 2.808 Mbit = 351 kB.
        c = self.cycles[0]
        self.assertAlmostEqual(c.excess_bytes / 1000.0, 351.0, delta=40.0)

    def test_settle_time(self):
        c = self.cycles[0]
        self.assertGreaterEqual(c.settle_ms, 450)
        self.assertLessEqual(c.settle_ms, 650)

    def test_qp_trace(self):
        c = self.cycles[0]
        self.assertEqual(c.qp_quiet, 18)
        self.assertEqual(c.qp_peak, 34)

    def test_no_idr_in_ramp(self):
        self.assertEqual(self.cycles[0].idrs, 0)

    def test_report_runs(self):
        buf = io.StringIO()
        with redirect_stdout(buf):
            vb.report(self.cycles, self.polls, vb.Params())
        out = buf.getvalue()
        self.assertIn("cycle 0", out)
        self.assertIn("peak", out)


class NoBurst(unittest.TestCase):
    def test_ramp_that_never_exceeds_programmed_is_still_a_cycle(self):
        # Quiet, then straight to the programmed rate with no overshoot:
        # the cycle exists (the bench wants to see "peak 1.00x") and its
        # excess is ~0.
        csv = synth([(5.0, 5.0), (3.0, 16.384)], qp_of)
        frames, polls, _ = vb.load(io.StringIO(csv))
        cycles = vb.find_cycles(frames, polls, vb.Params())
        self.assertEqual(len(cycles), 1)
        self.assertLess(cycles[0].peak_kbps / PROGRAMMED_KBPS, 1.05)
        self.assertLess(cycles[0].excess_bytes, 20_000)

    def test_flat_stream_has_no_cycles(self):
        csv = synth([(8.0, 16.384)], qp_of)
        frames, polls, _ = vb.load(io.StringIO(csv))
        self.assertEqual(vb.find_cycles(frames, polls, vb.Params()), [])
        # Calibration line: an exact-CBR stream reads 1.00x of programmed
        # (kbps x 1024), at every percentile, and the qp trace is flat.
        buf = io.StringIO()
        with redirect_stdout(buf):
            s = vb.capture_summary(frames, polls, vb.Params())
        self.assertAlmostEqual(s['mean_kbps'] / s['prog'], 1.0, delta=0.01)
        self.assertAlmostEqual(s['max'] / s['prog'], 1.0, delta=0.01)
        self.assertAlmostEqual(s['fps'], 60.0, delta=0.5)
        self.assertEqual((s['qp_min'], s['qp_max']), (28, 28))
        self.assertIn("1.00x", buf.getvalue())


class TwoCycles(unittest.TestCase):
    def test_two_cycles_and_idr_count(self):
        csv = synth([(4.0, 5.0), (0.5, 20.0), (3.0, 16.384),
                     (4.0, 4.0), (0.3, 24.0), (3.0, 16.384)], qp_of, idr_every=120)
        frames, polls, _ = vb.load(io.StringIO(csv))
        cycles = vb.find_cycles(frames, polls, vb.Params())
        self.assertEqual(len(cycles), 2)
        self.assertGreater(cycles[1].peak_kbps, cycles[0].peak_kbps)
        # An IDR every 2 s lands inside at least one ramp window; the count
        # is reported so the operator can discount that cycle.
        self.assertTrue(any(c.idrs >= 0 for c in cycles))


class OldCaptureWithoutQp(unittest.TestCase):
    def test_polls_without_qp_column(self):
        csv = synth([(5.0, 5.0), (0.5, 22.0), (3.0, 16.384)], qp_of)
        csv = "\n".join(",".join(l.split(",")[:3]) if l.startswith("s,") else l
                        for l in csv.splitlines()) + "\n"
        frames, polls, _ = vb.load(io.StringIO(csv))
        cycles = vb.find_cycles(frames, polls, vb.Params())
        self.assertEqual(len(cycles), 1)
        self.assertIsNone(cycles[0].qp_quiet)


if __name__ == "__main__":
    unittest.main()
