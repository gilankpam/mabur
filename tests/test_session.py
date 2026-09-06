import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tools"))
import session  # noqa: E402


def make_session(root, idx, names):
    d = os.path.join(root, f"{idx:04d}")
    os.makedirs(d, exist_ok=True)
    for n in names:
        open(os.path.join(d, n), "w").close()
    return d


class TestResolve(unittest.TestCase):
    def test_session_dir_resolves_every_file(self):
        with tempfile.TemporaryDirectory() as root:
            d = make_session(root, 42, ["ctl.log", "probe.log", "au.log",
                                        "flight.jsonl", "lat.log"])
            s = session.resolve(d)
            self.assertEqual(s.dir, d)
            self.assertEqual(s.ctl, os.path.join(d, "ctl.log"))
            self.assertEqual(s.au, os.path.join(d, "au.log"))
            self.assertEqual(s.flight, os.path.join(d, "flight.jsonl"))
            self.assertEqual(s.lat, os.path.join(d, "lat.log"))

    def test_missing_files_are_none_not_errors(self):
        with tempfile.TemporaryDirectory() as root:
            d = make_session(root, 1, ["ctl.log"])
            s = session.resolve(d)
            self.assertIsNotNone(s.ctl)
            self.assertIsNone(s.probe)
            self.assertIsNone(s.lat)

    def test_legacy_file_path_passes_through(self):
        with tempfile.TemporaryDirectory() as root:
            p = os.path.join(root, "ctl-0007_20260906.log")
            with open(p, "w") as f:
                f.write("ctllog 11 x\n")
            s = session.resolve(p)
            self.assertEqual(s.ctl, p)
            self.assertIsNone(s.dir)
            self.assertIsNone(s.au)

    def test_legacy_file_classified_by_marker_not_by_name(self):
        with tempfile.TemporaryDirectory() as root:
            p = os.path.join(root, "whatever.log")
            with open(p, "w") as f:
                f.write("probelog 2 bpb=4\n")
            self.assertEqual(session.resolve(p).probe, p)

    def test_latest_picks_highest_index_not_newest_mtime(self):
        with tempfile.TemporaryDirectory() as root:
            old = make_session(root, 41, ["ctl.log"])
            new = make_session(root, 7, ["ctl.log"])
            os.utime(new, (10**9, 10**9))  # far newer mtime, lower index
            self.assertEqual(session.latest(root), old)

    def test_no_arg_uses_latest(self):
        with tempfile.TemporaryDirectory() as root:
            d = make_session(root, 3, ["au.log"])
            s = session.resolve(None, root=root)
            self.assertEqual(s.dir, d)
            self.assertEqual(s.au, os.path.join(d, "au.log"))


if __name__ == "__main__":
    unittest.main()
