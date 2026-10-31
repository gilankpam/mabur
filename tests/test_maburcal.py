#!/usr/bin/env python3
"""Tests for gs/bundle/maburcal's cal.log reader.

The reader is the in-repo consumer the compatibility policy requires: the
cal.log schema may change freely, but this must change with it in the same
commit.
"""
import argparse
import importlib.machinery
import importlib.util
import pathlib
import sys
import tempfile
import time
import unittest

REPO = pathlib.Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_loader(
    "maburcal",
    importlib.machinery.SourceFileLoader("maburcal",
                                         str(REPO / "gs/bundle/maburcal")))
maburcal = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(maburcal)

# flags: 1=no_dip 2=undetermined 4=narrow 8=saturated 16=card_disagree 32=drift
GOOD = """callog 1
R 42 53 1.00
C 1 0 88 20 20 0 0 -62 -999
W 0 91 28 0 1
W 3 95 40 0 0
W 5 54 30 0 32
W 7 -1 -1 0 2
V 0 87 100
V 3 91 99
V 5 50 97
"""

GOOD3 = """callog 3
R 42 1.00
C 1 0 -40 20 20 0 0 -62 -999
W 0 63 -40 0 1
W 3 42 -40 0 0
W 5 -1 -40 0 32
W 7 -128 -128 -1 2
V 0 59 100
V 3 38 99
V 5 -5 97
"""

# Two widths: 20 MHz sweeps all four rows plus a capacity point on mcs3;
# 40 MHz only re-sweeps mcs0/mcs3 (mcs5/mcs7 keep whatever the drone already
# has for that width -- callog 4 carries no record of that), and its mcs3
# capacity point is tx_bound. Both widths' applies succeed.
GOOD4 = """callog 4
R 42 1.00 149 auto 1
S 0 power 20 20
W 0 63 -40 0 1
W 3 42 -40 0 0
W 5 -1 -40 0 0
W 7 -128 -128 -1 2
S 1 power 20 100
V 0 59 100
V 3 38 99
V 5 -5 97
S 2 capacity 20 2000
P 3 4000 3950 58000000 730 72 0
S 3 power 40 20
W 0 60 -42 0 1
W 3 40 -42 0 0
S 4 power 40 100
V 0 56 98
V 3 36 95
S 5 capacity 40 2000
P 3 4000 2000 30000000 500 20 64
A 20 1 ok
A 40 1 ok
"""

# A GS that only flies 20 MHz: no S row ever names 40, so there is nothing
# to render a second block from.
GOOD4_20ONLY = """callog 4
R 42 1.00 149 auto 1
S 0 power 20 20
W 0 63 -40 0 1
W 3 42 -40 0 0
S 1 power 20 100
V 0 59 100
V 3 38 99
S 2 capacity 20 2000
P 3 4000 3950 58000000 730 72 0
A 20 1 ok
"""


def render(text):
    with tempfile.NamedTemporaryFile("w", suffix=".log", delete=False) as f:
        f.write(text)
        path = f.name
    return maburcal.render_report(path)


class TestReport(unittest.TestCase):
    def test_renders_a_row_per_measured_rate(self):
        out = render(GOOD)
        for rate in ("mcs0", "mcs3", "mcs5", "mcs7"):
            self.assertIn(rate, out)

    def test_undetermined_rate_is_not_given_a_number(self):
        # The kit must never invent a wall from data that cannot support one.
        out = render(GOOD)
        row = [l for l in out.splitlines() if l.startswith("mcs7")][0]
        self.assertIn("undetermined", row)
        self.assertNotIn("127", row)

    def test_no_dip_rate_is_labelled_knee_derived(self):
        out = render(GOOD)
        row = [l for l in out.splitlines() if l.startswith("mcs0")][0]
        self.assertIn("no_dip", row)
        self.assertIn("91", row)

    def test_drift_flag_surfaces(self):
        out = render(GOOD)
        row = [l for l in out.splitlines() if l.startswith("mcs5")][0]
        self.assertIn("drift", row)

    def test_park_index_is_wall_minus_margin(self):
        # margin 1.0 dB = 4 TXAGC steps, matching power_plan.h.
        out = render(GOOD)
        row = [l for l in out.splitlines() if l.startswith("mcs3")][0]
        self.assertIn("91", row)   # 95 - 4

    def test_legacy_is_reported_as_derived_from_mcs0(self):
        out = render(GOOD)
        self.assertIn("legacy", out)
        self.assertIn("derived", out)

    def test_malformed_line_is_skipped_not_fatal(self):
        out = render(GOOD + "C not a record\nW\n")
        self.assertIn("mcs3", out)

    def test_malformed_field_value_is_skipped_not_fatal(self):
        # Right shape (6 tokens for a W record), but a non-numeric field --
        # this must reach the try/except ValueError branch inside
        # _Run.feed_line, not the tag/length filtering the line above
        # exercises, which never gets as far as int().
        out = render(GOOD + "W abc 91 28 0 1\n")
        self.assertIn("mcs3", out)

    def test_unknown_version_is_refused_not_misparsed(self):
        with self.assertRaises(maburcal.UnsupportedLog):
            render(GOOD.replace("callog 1", "callog 9"))

    def test_two_runs_in_one_file_are_reported_separately(self):
        # The retry path: run, see a narrow/saturated flag, move the drone,
        # run again. The session directory does not rotate between them, so
        # one cal.log holds both runs -- delimited by R records, not by the
        # file-format marker.
        out = render(GOOD + "R 43 53 1.00\nW 0 88 28 0 1\nV 0 84 100\n")
        self.assertIn("42", out)
        self.assertIn("43", out)

    def test_run_parameters_come_from_the_R_record(self):
        # nonce/base_ref/margin live on R, not on the marker line -- a reader
        # looking for them on "callog 1" finds nothing.
        self.assertIn("53", render(GOOD))

    def test_missing_header_is_refused(self):
        with self.assertRaises(maburcal.UnsupportedLog):
            render("W 0 91 28 0 1\n")

    def test_written_line_only_when_verify_completed(self):
        # A run with V records reached and passed the drone's self-initiated
        # verify sweep, which only runs after a successful apply -- the one
        # signal cal.log actually carries for "this run's config landed".
        out = render(GOOD)
        self.assertIn("written: /etc/mabur.toml", out)

    def test_no_verify_reports_not_written(self):
        # Walls measured (a W record exists) but the run never produced a
        # verify pass -- e.g. phase 2's command was lost and the session's
        # deadline expired (the await_next_timeout path). cal.log has no record
        # of an apply outcome, so this must not claim one.
        text = "callog 1\nR 44 53 1.00\nW 0 91 28 0 1\n"
        out = render(text)
        self.assertNotIn("written: /etc/mabur.toml", out)
        self.assertIn("not written", out)

    def test_no_measurements_reports_not_written(self):
        # A run that only ever wrote its R line (e.g. the ack for phase 1
        # arrived but nothing was ever measured) must not claim a write.
        text = "callog 1\nR 45 53 1.00\n"
        out = render(text)
        self.assertNotIn("written: /etc/mabur.toml", out)
        self.assertIn("not written", out)

    def test_all_zero_verify_rows_report_not_written(self):
        # The kit that wrote callog 1-3 opened the GS's verify window
        # optimistically, with no ack for its result frame, so a lost result
        # or a refused apply still produced a full set of V rows -- every one
        # of them 0%. In those files a row's mere presence must never be read
        # as evidence of an apply; only a nonzero delivery is.
        text = ("callog 1\nR 46 53 1.00\n"
                "W 0 91 28 0 1\nW 1 88 28 0 1\n"
                "V 0 87 0\nV 1 84 0\n")
        out = render(text)
        self.assertNotIn("written: /etc/mabur.toml", out)
        self.assertIn("not written", out)

    def test_one_silent_rate_is_distinguishable_from_never_planned(self):
        # A rate with a real, determined wall that was included in the
        # verify plan and heard nothing at its parked power (mcs1, "0%")
        # must render differently from a rate whose wall never came back
        # determined in the first place (mcs7, "-" -- verify_cells()
        # never seeds a cell for it, so cal.log carries no V row at all).
        # Both used to print "-" in the verify column, hiding the single
        # most important signal a verify pass exists to produce: a rate
        # that is dead at its own parked power.
        text = ("callog 1\nR 47 53 1.00\n"
                "W 0 91 28 0 1\nW 1 88 28 0 1\nW 7 -1 -1 0 2\n"
                "V 0 87 62\nV 1 84 0\n")
        out = render(text)
        row0 = [l for l in out.splitlines() if l.startswith("mcs0")][0]
        row1 = [l for l in out.splitlines() if l.startswith("mcs1")][0]
        row7 = [l for l in out.splitlines() if l.startswith("mcs7")][0]
        self.assertIn("62%", row0)
        self.assertIn("0%", row1)     # planned, silent -- a real 0%, not "-"
        self.assertNotIn("0%", row7)  # never planned -- "-", not a measurement
        self.assertIn("-", row7)
        # And the run as a whole still applied: mcs0's real delivery is
        # proof enough, mcs1's silence notwithstanding.
        self.assertIn("written: /etc/mabur.toml", out)


class TestReportV3(unittest.TestCase):
    def test_v3_renders_relative_walls_and_no_anchor(self):
        out = render(GOOD3)
        self.assertIn("wall(rel)", out)
        self.assertNotIn("base_ref", out)
        row = [l for l in out.splitlines() if l.startswith("mcs3")][0]
        self.assertIn("42", row)
        self.assertIn("38", row)   # park = 42 - 4

    def test_v3_minus_one_is_a_real_wall(self):
        out = render(GOOD3)
        row = [l for l in out.splitlines() if l.startswith("mcs5")][0]
        self.assertIn(" -1", row)
        self.assertIn(" -5", row)  # park
        self.assertIn("97%", row)

    def test_v3_undetermined_is_the_sentinel_not_a_number(self):
        out = render(GOOD3)
        row = [l for l in out.splitlines() if l.startswith("mcs7")][0]
        self.assertIn("undetermined", row)
        self.assertNotIn("-128", row)
        # The word appears once (the flags column) -- the wall column
        # prints "-", not a second "undetermined", to keep the wall(rel)
        # column's width sane.
        self.assertEqual(row.count("undetermined"), 1)
        fields = row.split()
        self.assertEqual(fields[1], "-")  # wall column

    def test_v1_undetermined_row_renders_exactly_as_before(self):
        # Regression: v1/v2 files must render byte-for-byte as they always
        # have (recordings on the DVR outlive the code, CLAUDE.md) -- the
        # wall column keeps printing the raw sentinel (-1), never the word
        # "undetermined", even though the flags column's own "undetermined"
        # text satisfies the weaker (pre-existing) assertion above it.
        old_row = (f"{'mcs7':<6}{'-1':>6}{'-':>6}{'-':>7}   "
                   f"undetermined").rstrip()
        self.assertIn(old_row, render(GOOD).splitlines())

    def test_v1_still_renders_with_the_anchor(self):
        out = render(GOOD)
        self.assertIn("base_ref=53", out)
        self.assertIn("base_ref_idx 53", out)

    def test_v3_written_claim_keys_on_verify(self):
        self.assertIn("written:", render(GOOD3))


class TestReportV4(unittest.TestCase):
    def test_two_width_table(self):
        out = render(GOOD4)
        self.assertIn("20 MHz", out)
        self.assertIn("40 MHz", out)
        row0 = [l for l in out.splitlines() if l.startswith("mcs0")][0]
        self.assertIn("63", row0)   # 20 MHz wall
        self.assertIn("60", row0)   # 40 MHz wall
        self.assertIn("59", row0)   # 20 MHz park = 63 - 4
        self.assertIn("56", row0)   # 40 MHz park = 60 - 4
        self.assertIn("100%", row0)
        self.assertIn("98%", row0)
        row3 = [l for l in out.splitlines() if l.startswith("mcs3")][0]
        self.assertIn(".73", row3)  # 20 MHz eff, eff_x1000=730
        # The 40 MHz point (eff_x1000=500) is tx_bound: measured, never
        # written, so it prints as "-" and the flag says why.
        self.assertNotIn(".50", row3)
        self.assertIn("legacy 63 (from 20 MHz mcs0)", out)
        self.assertIn("written: /etc/mabur.toml", out)

    def test_single_width_prints_dashes(self):
        # mcs5 was only ever swept at 20 MHz in this run -- callog 4 carries
        # no record of a 40 MHz wall/verify/eff for it, so every 40 MHz
        # column must read "-", not a stale or invented number.
        out = render(GOOD4)
        row5 = [l for l in out.splitlines() if l.startswith("mcs5")][0]
        self.assertEqual(row5.split()[-4:], ["-", "-", "-", "-"])
        self.assertIn("-1", row5)   # the 20 MHz wall is a real, measured one

    def test_not_written_when_no_applied_ok(self):
        # Neither width's A row is ok -- unlike v1-3, callog 4 has a direct
        # apply-outcome record and must not fall back to inferring one from
        # verify delivery.
        text = GOOD4.replace("A 20 1 ok\nA 40 1 ok\n",
                             "A 20 0 out of range table\n"
                             "A 40 0 out of range table\n")
        out = render(text)
        self.assertNotIn("written: /etc/mabur.toml", out)
        self.assertIn("not written", out)
        self.assertIn("out of range table", out)

    def test_tx_bound_flag_is_prefixed(self):
        # The 40 MHz capacity point on mcs3 is tx_bound; the 20 MHz one on
        # the same rate is not -- the flags column must say which width.
        out = render(GOOD4)
        row3 = [l for l in out.splitlines() if l.startswith("mcs3")][0]
        self.assertIn("40:tx_bound", row3)
        self.assertNotIn("20:tx_bound", row3)

    def test_tx_bound_eff_is_not_printed_as_written(self):
        out = render(GOOD4)
        row3 = [l for l in out.splitlines() if l.startswith("mcs3")][0]
        # mcs3, 20 MHz wall park verify eff, then 40 MHz wall park verify eff
        self.assertEqual(row3.split()[:9],
                         ["mcs3", "42", "38", "99%", ".73", "40", "36", "95%", "-"])

    def test_lost_eff_is_not_printed_as_written(self):
        # Under half of the offered frames arrived on the 20 MHz mcs3 point:
        # lost (and lossy), never written, so it prints "-" and the flags
        # column says why -- unlike a merely lossy point, which is written.
        text = GOOD4.replace("P 3 4000 3950 58000000 730 72 0\n",
                             "P 3 4000 1500 22000000 270 72 384\n")
        out = render(text)
        row3 = [l for l in out.splitlines() if l.startswith("mcs3")][0]
        self.assertEqual(row3.split()[:5], ["mcs3", "42", "38", "99%", "-"])
        self.assertIn("20:lossy,lost", row3)
        lossy = render(GOOD4.replace("P 3 4000 3950 58000000 730 72 0\n",
                                     "P 3 4000 3000 44000000 550 72 128\n"))
        row3 = [l for l in lossy.splitlines() if l.startswith("mcs3")][0]
        self.assertEqual(row3.split()[:5], ["mcs3", "42", "38", "99%", ".55"])

    def test_legacy_names_the_width_it_came_from(self):
        # A run with no 20 MHz power sweep takes legacy from the 40 MHz
        # mcs0 wall; the report says so rather than claiming 20 MHz.
        text = ("callog 4\n"
                "R 42 1.00 149 auto 1\n"
                "S 0 power 40 20\n"
                "W 0 60 -42 0 1\n"
                "W 3 40 -42 0 0\n"
                "S 1 power 40 100\n"
                "V 0 56 98\n"
                "A 40 1 ok\n")
        out = render(text)
        self.assertIn("legacy 60 (from 40 MHz mcs0)", out)
        self.assertNotIn("from 20 MHz", out)

    def test_an_ok_apply_with_a_note_is_reported(self):
        # The drone kept a 20 MHz wall for an undetermined row of a first
        # 40 MHz table: the A row is ok and its reason says what was kept.
        text = GOOD4.replace("A 40 1 ok\n",
                             "A 40 1 ok; mcs6 kept from the 20 MHz table\n")
        out = render(text)
        self.assertIn("40 MHz apply: ok; mcs6 kept from the 20 MHz table", out)
        self.assertIn("written: /etc/mabur.toml", out)
        self.assertNotIn("20 MHz apply:", out)  # a plain ok says nothing

    def test_eff_only_width_reports_existing_walls_note(self):
        # A width with capacity (P) rows but no wall (W) rows in this run
        # measured efficiency against whatever the drone already has
        # configured -- cal.log carries no record of that wall, so the
        # report must say so rather than printing "-" with no explanation.
        text = ("callog 4\nR 1 1.00 149 auto 1\n"
                "S 0 capacity 40 2000\nP 3 4000 3950 58000000 900 80 0\n"
                "A 40 1 ok\n")
        out = render(text)
        self.assertIn("40 MHz eff measured at the existing walls", out)

    def test_run_header_carries_channel_pick_and_ldpc(self):
        out = render(GOOD4)
        self.assertIn("ch=149", out)
        self.assertIn("pick=auto", out)
        self.assertIn("ldpc=1", out)

    def test_20_only_run_has_no_40_mhz_block(self):
        # No S row in this run ever names 40 MHz -- a 20-only GS -- so the
        # report must not print a 40 MHz header, column group or flags
        # prefix at all, not merely dashes under one.
        out = render(GOOD4_20ONLY)
        self.assertNotIn("40 MHz", out)
        self.assertNotIn("40:", out)
        row0 = [l for l in out.splitlines() if l.startswith("mcs0")][0]
        self.assertEqual(row0.split(), ["mcs0", "63", "59", "100%", "-", "20:no_dip"])

    def test_not_written_when_no_apply_attempted(self):
        # No A row at all (phase 2's command was lost, or the session never
        # got that far) -- distinct from an A row that exists and failed.
        text = ("callog 4\nR 1 1.00 149 auto 1\n"
                "S 0 power 20 20\nW 0 63 -40 0 1\n")
        out = render(text)
        self.assertIn("not written: this run produced no apply attempt", out)


class TestRenderWhenReady(unittest.TestCase):
    """`maburcal start`'s end-of-run render, which polls a file the daemon
    is still writing."""

    def _tmp(self, text):
        with tempfile.NamedTemporaryFile("w", suffix=".log",
                                         delete=False) as f:
            f.write(text)
            return f.name

    def test_a_complete_run_renders(self):
        text, problem = maburcal._render_when_ready(self._tmp(GOOD))
        self.assertIsNone(problem)
        self.assertIn("mcs0", text)

    def test_a_structural_problem_is_reported_not_waited_out(self):
        # Regression (hardware): a cal.log missing its `callog 1` marker is
        # unreadable FOREVER -- no amount of polling makes a format marker
        # appear -- yet the reader swallowed UnsupportedLog inside the retry
        # loop and the operator got "holds no calibration run" after a
        # RENDER_RETRY_S wait, which points at the wrong thing entirely (the
        # file was full of perfectly good measurements). A permanent problem
        # must come back immediately, with its reason.
        path = self._tmp(GOOD.replace("callog 1\n", "", 1))
        start = time.monotonic()
        text, problem = maburcal._render_when_ready(path)
        self.assertIsNone(text)
        self.assertIn("callog format marker", problem)
        self.assertLess(time.monotonic() - start, maburcal.RENDER_RETRY_S)

    def test_no_path_is_not_a_problem_to_report(self):
        text, problem = maburcal._render_when_ready(None)
        self.assertIsNone(text)
        self.assertIsNone(problem)


class TestCallogVersions(unittest.TestCase):
    """The marker versions the FILE. maburgs writes 1; 2 is accepted because
    bench runs in that shape (the reverted EVM columns, docs/calibration.md)
    are on the DVR, and this reader ignores C rows entirely so both render
    identically."""

    def test_v1_and_v2_both_render(self):
        v1 = GOOD  # "callog 1", nine-field C row
        v2 = (GOOD.replace("callog 1", "callog 2", 1)
                  .replace("C 1 0 88 20 20 0 0 -62 -999",
                           "C 1 0 88 20 20 0 0 -62 -999 -31 -999", 1))
        self.assertIn("mcs0", render(v1))
        self.assertIn("mcs0", render(v2))

    def test_an_unknown_version_is_still_refused(self):
        with self.assertRaises(maburcal.UnsupportedLog):
            render(GOOD.replace("callog 1", "callog 99", 1))


class TestStartArgs(unittest.TestCase):
    """`maburcal start`'s CLI tokens, forwarded verbatim to CalControl --
    this script neither validates nor composes them, so the only thing
    worth testing here is that it sends exactly what the operator typed."""

    def test_tokens_are_forwarded(self):
        sent = []

        def fake_send_cmd(message, timeout=1.0):
            sent.append(message)
            if message.startswith("start"):
                return "ok started power40,cap20"
            return "ok state=done sweep=1/1 kind=power bw=40 cell=0/0"

        orig = maburcal._send_cmd
        maburcal._send_cmd = fake_send_cmd
        try:
            maburcal.cmd_start(argparse.Namespace(tokens=["power40", "cap20"]))
        finally:
            maburcal._send_cmd = orig

        self.assertEqual(sent[0], "start power40 cap20")

    def test_no_tokens_sends_bare_start(self):
        sent = []

        def fake_send_cmd(message, timeout=1.0):
            sent.append(message)
            if message.startswith("start"):
                return "ok started power20,cap20"
            return "ok state=done sweep=1/1 kind=power bw=20 cell=0/0"

        orig = maburcal._send_cmd
        maburcal._send_cmd = fake_send_cmd
        try:
            maburcal.cmd_start(argparse.Namespace(tokens=[]))
        finally:
            maburcal._send_cmd = orig

        self.assertEqual(sent[0], "start")


if __name__ == "__main__":
    unittest.main()
