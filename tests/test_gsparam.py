#!/usr/bin/env python3
"""Test for tools/bench/gsparam.py's pure functions: the wire-line builder
(build_line), the single-line reply parser (parse_reply), and the --all
away/back table (expected_pairs) -- the parts that need no network or ssh.
The remote exchange (send_recv/remote_shell) and the --all driver are
exercised only by running the script against a real GS (not here; see
CLAUDE.md's "Do not deploy to ... any device").
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                 '..', 'tools', 'bench'))
import gsparam  # noqa: E402


class TestBuildLine(unittest.TestCase):
    def test_set(self):
        self.assertEqual(gsparam.build_line("set", "link.static_mcs", "3"),
                          "set link.static_mcs = 3")

    def test_get_with_key(self):
        self.assertEqual(gsparam.build_line("get", "link.static_mcs"),
                          "get link.static_mcs")

    def test_get_no_key(self):
        self.assertEqual(gsparam.build_line("get"), "get")


class TestParseReply(unittest.TestCase):
    def test_ok_with_value(self):
        self.assertEqual(gsparam.parse_reply("ok -1"), ("ok", "-1"))

    def test_err_busy(self):
        self.assertEqual(gsparam.parse_reply("err busy"), ("err", "busy"))

    def test_err_not_live(self):
        self.assertEqual(gsparam.parse_reply("err not live: link.ladder"),
                          ("err", "not live: link.ladder"))

    def test_ok_no_value(self):
        self.assertEqual(gsparam.parse_reply("ok"), ("ok", ""))


class TestExpectedPairs(unittest.TestCase):
    def test_gs_has_nine_keys(self):
        pairs = gsparam.expected_pairs("gs")
        self.assertEqual(len(pairs), 9)

    def test_gs_keys_match_live_table_order(self):
        pairs = gsparam.expected_pairs("gs")
        self.assertEqual([k for k, _, _ in pairs], [
            "radio.channel",
            "link.static_mcs",
            "link.static_bw",
            "link.static_overhead_base",
            "link.static_overhead_enh",
            "link.ladder",
            "link.nack.enable",
            "debug_log.enable",
            "msp.enable",
        ])

    def test_gs_away_and_back_differ(self):
        for key, away, back in gsparam.expected_pairs("gs"):
            self.assertNotEqual(away, back, key)

    def test_player_has_nine_keys(self):
        pairs = gsparam.expected_pairs("player")
        self.assertEqual(len(pairs), 9)

    def test_player_keys_match_live_table_order(self):
        pairs = gsparam.expected_pairs("player")
        self.assertEqual([k for k, _, _ in pairs], [
            "screen_mode",
            "colortrans.enable",
            "osd.enable",
            "osd.gs.enable",
            "dvr.target",
            "dvr.mode",
            "dvr.burned.bitrate_kbps",
            "dvr.burned.every",
            "dvr.burned.size",
        ])

    def test_player_screen_mode_away_from_alt_mode(self):
        pairs = gsparam.expected_pairs("player", alt_mode="1280x720@120")
        away = dict((k, a) for k, a, _ in pairs)["screen_mode"]
        self.assertEqual(away, '"1280x720@120"')


if __name__ == "__main__":
    unittest.main()
