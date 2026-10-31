#!/usr/bin/env python3
"""Test for tools/bench/gsparam.py's pure functions: the wire-line builder
(build_line), the single-line reply parser (parse_reply), the --all
away/back table (expected_pairs), the ladder renderer (render_ladder), and
the section-scoped TOML value check (value_in_section and its
parse_toml_sections/section_of/leaf_of helpers), and the ladder's block
count (ladder_block_count) -- the parts that need no
network or ssh. The remote exchange (send_recv/remote_shell) and the --all
driver that calls them are a later bench gate's job to run against a real
GS, not this host test's.
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


class TestRenderLadder(unittest.TestCase):
    def test_bundle_ladder_text(self):
        self.assertEqual(
            gsparam.render_ladder(gsparam.BUNDLE_LADDER),
            "[{mcs = 0, bw = 40, overhead_base = 0.5, overhead_enh = 0.25}, "
            "{mcs = 1, bw = 40, overhead_base = 0.5, overhead_enh = 0.25}, "
            "{mcs = 2, bw = 40, overhead_base = 0.5, overhead_enh = 0.25}, "
            "{mcs = 3, bw = 40, overhead_base = 0.5, overhead_enh = 0.25}, "
            "{mcs = 4, bw = 40, overhead_base = 0.5, overhead_enh = 0.25}]")


class TestSectionOfAndLeafOf(unittest.TestCase):
    def test_section_of_nested(self):
        self.assertEqual(gsparam.section_of("link.nack.enable"), "link.nack")

    def test_section_of_single_dot(self):
        self.assertEqual(gsparam.section_of("msp.enable"), "msp")

    def test_section_of_top_level(self):
        self.assertEqual(gsparam.section_of("screen_mode"), "")

    def test_leaf_of(self):
        self.assertEqual(gsparam.leaf_of("link.nack.enable"), "enable")
        self.assertEqual(gsparam.leaf_of("screen_mode"), "screen_mode")


# A fragment with the GS's three `enable` sections at different values,
# the shape that makes a file-wide "does <leaf> = <value> appear anywhere"
# check pass vacuously: every "enable" leaf but the target one sits at some
# other value, so a check that isn't scoped to the target's own section
# would see the right text off the wrong section.
THREE_ENABLE_SECTIONS = (
    "[msp]\n"
    "enable      = true\n"
    "symbol_size = 1312\n"
    "\n"
    "[link.nack]\n"
    "enable    = false\n"
    "lookback  = 256\n"
    "\n"
    "[debug_log]\n"
    "enable        = false\n"
    "dir           = \"/media/dvr/log\"\n"
)


class TestParseTomlSections(unittest.TestCase):
    def test_splits_by_header(self):
        sections = gsparam.parse_toml_sections(THREE_ENABLE_SECTIONS)
        self.assertIn("enable      = true", sections["msp"])
        self.assertIn("enable    = false", sections["link.nack"])
        self.assertIn("enable        = false", sections["debug_log"])

    def test_top_level_before_any_header(self):
        text = 'screen_mode = "1920x1080@60"\n[osd]\nenable = true\n'
        sections = gsparam.parse_toml_sections(text)
        self.assertIn('screen_mode = "1920x1080@60"', sections[""])
        self.assertNotIn('screen_mode = "1920x1080@60"', sections.get("osd", []))


class TestValueInSection(unittest.TestCase):
    def test_matches_in_its_own_section(self):
        self.assertTrue(gsparam.value_in_section(
            THREE_ENABLE_SECTIONS, "msp.enable", "true"))
        self.assertTrue(gsparam.value_in_section(
            THREE_ENABLE_SECTIONS, "link.nack.enable", "false"))
        self.assertTrue(gsparam.value_in_section(
            THREE_ENABLE_SECTIONS, "debug_log.enable", "false"))

    def test_does_not_match_off_another_section(self):
        # [msp] enable is true, but link.nack.enable is false here -- a
        # file-wide search for "enable = true" would wrongly pass this.
        self.assertFalse(gsparam.value_in_section(
            THREE_ENABLE_SECTIONS, "link.nack.enable", "true"))
        # debug_log.enable is false, never true in this fragment.
        self.assertFalse(gsparam.value_in_section(
            THREE_ENABLE_SECTIONS, "debug_log.enable", "true"))

    def test_top_level_key(self):
        text = 'screen_mode = "1920x1080@60"\n[osd]\nenable = true\n'
        self.assertTrue(gsparam.value_in_section(
            text, "screen_mode", '"1920x1080@60"'))
        self.assertFalse(gsparam.value_in_section(
            text, "screen_mode", '"1280x720@120"'))


BUNDLE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                      '..', 'gs', 'bundle', 'maburgs.default.toml')


class TestLadderBlockCount(unittest.TestCase):
    def test_counts_the_bundles_rung_headers_not_its_comments(self):
        # The bundle names `[[link.ladder]]` in a comment too; only a line
        # that is the header itself is a rung.
        with open(BUNDLE) as f:
            text = f.read()
        self.assertGreater(text.count("[[link.ladder]]"), 5)
        self.assertEqual(gsparam.ladder_block_count(text), 5)

    def test_indented_header_counts(self):
        self.assertEqual(gsparam.ladder_block_count(
            "  [[link.ladder]]  \nmcs = 0\n# [[link.ladder]] in a comment\n"), 1)


if __name__ == "__main__":
    unittest.main()
