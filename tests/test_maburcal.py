#!/usr/bin/env python3
"""Tests for gs/bundle/maburcal's cal.log reader.

The reader is the in-repo consumer the compatibility policy requires: the
cal.log schema may change freely, but this must change with it in the same
commit.
"""
import importlib.machinery
import importlib.util
import pathlib
import sys
import tempfile
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
        # deadline expired (Task 10's await_next_timeout path). cal.log has
        # no record of an apply outcome, so this must not claim one.
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


if __name__ == "__main__":
    unittest.main()
