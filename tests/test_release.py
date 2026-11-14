#!/usr/bin/env python3
"""tools/release.py: the drone<->GS contract, the minor/patch rule, notes."""
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools"))
import release  # noqa: E402

RC_PROTO = """
constexpr uint8_t RC_VERSION = 18;
// a comment naming CAP_NOT_A_CONSTANT
constexpr uint16_t CAP_FRAME_WIRE = 0x0001;
constexpr uint16_t CAP_TELEMETRY = 0x0002;
"""
PARAMS = """
static const ParamRow kTable[] = {
    {"RADIO_LDPC", ParamType::kBool, "radio", "ldpc"},
    {"VENC_MODE", ParamType::kStr, "venc", "mode"},
};
"""


def tree(rc=RC_PROTO, params=PARAMS):
    root = tempfile.mkdtemp()
    inc = os.path.join(root, "common", "include", "mabur")
    os.makedirs(inc)
    with open(os.path.join(inc, "rc_proto.h"), "w") as f:
        f.write(rc)
    with open(os.path.join(inc, "config_params.h"), "w") as f:
        f.write(params)
    return root


class Contract(unittest.TestCase):
    def test_contract_reads_rc_version_caps_and_params(self):
        self.assertEqual(release.contract(tree()), {
            "rc_version": 18,
            "caps": {"CAP_FRAME_WIRE": 1, "CAP_TELEMETRY": 2},
            "params": ["RADIO_LDPC", "VENC_MODE"],
        })

    def test_diff_reports_each_change(self):
        prev = release.contract(tree())
        cur = release.contract(tree(
            rc=RC_PROTO.replace("= 18", "= 19") + "constexpr uint16_t CAP_CAL_WIDE = 0x0010;\n",
            params=PARAMS.replace("};", '    {"VENC_GOP_S", ParamType::kInt, "venc", "gop_s"},\n};')))
        changes = release.diff(prev, cur)
        self.assertEqual(len(changes), 3, changes)
        self.assertIn("RC_VERSION 18 -> 19", changes)
        self.assertTrue(any("CAP_CAL_WIDE" in c for c in changes))
        self.assertTrue(any("param rows 2 -> 3" in c for c in changes))
        self.assertEqual(release.diff(prev, prev), [])

    def test_reordered_params_are_a_change(self):
        prev = release.contract(tree())
        cur = release.contract(tree(params=PARAMS.replace("RADIO_LDPC", "TMP").replace(
            "VENC_MODE", "RADIO_LDPC").replace("TMP", "VENC_MODE")))
        self.assertEqual(len(release.diff(prev, cur)), 1)


class CheckTag(unittest.TestCase):
    def test_patch_bump_over_contract_change_is_refused(self):
        err = release.check_tag("v0.1.1", "v0.1.0", ["RC_VERSION 18 -> 19"])
        self.assertIsNotNone(err)
        self.assertIn("v0.2.0", err)

    def test_minor_bump_over_contract_change_passes(self):
        self.assertIsNone(release.check_tag("v0.2.0", "v0.1.0", ["RC_VERSION 18 -> 19"]))

    def test_patch_bump_without_change_passes(self):
        self.assertIsNone(release.check_tag("v0.1.1", "v0.1.0", []))

    def test_first_release_has_no_previous(self):
        self.assertIsNone(release.check_tag("v0.1.0", None, []))

    def test_malformed_tag_is_refused(self):
        self.assertIsNotNone(release.check_tag("v1.2", "v0.1.0", []))
        self.assertIsNotNone(release.check_tag("0.1.2", "v0.1.0", []))


class Notes(unittest.TestCase):
    INFO = {"board": "x", "openipc": "o" * 40, "kernel": "l" * 40, "devourer": "d" * 40, "uboot": "u" * 40,
            "kernel_sha256": "k" * 64, "toolchain_sha256": "t" * 64,
            "ipctool_sha256": "i" * 64}
    FILES = {"mabur-v0.2.0-a-nor.tgz": "aa" * 16, "mabur-v0.2.0-u-boot.bin": "bb" * 16}

    def test_first_release_notes_say_so(self):
        text = release.notes("v0.1.0", "c" * 40, self.INFO,
                             release.contract(tree()), None, [], self.FILES)
        self.assertIn("first release", text)

    def test_notes_lists_contract_and_files(self):
        changes = ["RC_VERSION 18 -> 19"]
        text = release.notes("v0.2.0", "c" * 40, self.INFO,
                             release.contract(tree()), "v0.1.0", changes, self.FILES)
        self.assertIn("RC_VERSION 18", text)
        self.assertIn("changed since v0.1.0", text)
        self.assertIn("RC_VERSION 18 -> 19", text)
        for name, md5 in self.FILES.items():
            self.assertIn(name, text)
            self.assertIn(md5, text)
        for key in ("openipc", "kernel", "devourer", "uboot", "kernel_sha256",
                    "toolchain_sha256", "ipctool_sha256"):
            self.assertIn(self.INFO[key], text)


if __name__ == "__main__":
    unittest.main()
