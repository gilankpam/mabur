import unittest

from maburtop import Model, render_rows, GRID_WIDTH

DGRAM = {
    "v": 1, "session": 0xDEADBEEF, "seq": 7, "t_ms": 567890,
    "link": {"state": "session", "tx_card": 0,
             "op": {"mcs": 5, "bw": 20, "sgi": False, "vht": False,
                    "overhead": 0.25, "offset_qdb": 0, "snr_req": 18.5},
             "deadline_ms": 60, "residual_loss": 0.012,
             "layer_delivery_pct": [100, 100, 97, 91]},
    "cards": [{"id": 0, "up": True, "frames": 123456, "crc_fail": 12,
               "loss_pct": 0.4, "rssi": -50.1, "rssi_a": -50.9, "rssi_b": -52.3,
               "snr": 27.1, "snr_a": 26.0, "snr_b": 24.5,
               "rx_mbps": 11.2, "pps": 980, "last_frame_age_ms": 4}],
    "fec": [{"stream": 0, "recovered_s": 12.0, "abandoned_s": 0.0,
             "syms_in_s": 4210.0, "recovered": 4021, "abandoned": 3,
             "stale": 0, "bad_cfg": 0, "sub_fail": 1, "in_flight": 2}],
    "video": {"fps": 59.9, "mbps": 9.31, "jitter_ms": 1.8,
              "clean": 21500, "truncated": 3, "dropped": 0, "stall_resets": 0,
              "rtp": {"ok": 812345, "gap": 2, "gap_seqs": 9, "back": 0},
              "udp": {"sent": 812347, "failed": 0, "bytes": 123456789},
              "q_drop": 0},
}


class RenderTest(unittest.TestCase):
    def fresh(self, dgram=None, wall=100.0):
        m = Model()
        m.update(dgram or DGRAM, wall)
        return m

    def test_header_and_card_row_content(self):
        rows = render_rows(self.fresh(), wall=100.2, width=GRID_WIDTH)
        self.assertIn("SESSION", rows[0])
        self.assertIn("MCS 5/20", rows[0])
        card = next(r for r in rows if r.lstrip().startswith("c0"))
        for cell in ("UP", "980", "11.2", "-50.1", "-50.9", "-52.3", "27.1"):
            self.assertIn(cell, card)

    def test_fixed_width_rows_never_shift(self):
        rows_a = render_rows(self.fresh(), wall=100.2, width=GRID_WIDTH)
        big = dict(DGRAM)
        big["cards"] = [dict(DGRAM["cards"][0],
                             frames=999999999, pps=99999, rx_mbps=999.9,
                             last_frame_age_ms=15000)]
        rows_b = render_rows(self.fresh(big), wall=100.2, width=GRID_WIDTH)
        self.assertEqual([len(r) for r in rows_a], [len(r) for r in rows_b])

    def test_fixed_width_survives_huge_last_frame_age(self):
        # A card silent for minutes (age >= 10s) must not widen the CARD row
        # (regression: naive "{ms}ms".rjust(6) overflowed past 10000 ms).
        rows_a = render_rows(self.fresh(), wall=100.2, width=GRID_WIDTH)
        huge = dict(DGRAM)
        huge["cards"] = [dict(DGRAM["cards"][0], last_frame_age_ms=123456789)]
        rows_b = render_rows(self.fresh(huge), wall=100.2, width=GRID_WIDTH)
        self.assertEqual([len(r) for r in rows_a], [len(r) for r in rows_b])

    def test_null_renders_dashes(self):
        d = dict(DGRAM)
        d["cards"] = [dict(DGRAM["cards"][0], rssi=None, loss_pct=None)]
        card = next(r for r in render_rows(self.fresh(d), 100.2, GRID_WIDTH)
                    if r.lstrip().startswith("c0"))
        self.assertIn("--", card)

    def test_stale_banner_after_2s(self):
        rows = render_rows(self.fresh(wall=100.0), wall=103.5, width=GRID_WIDTH)
        self.assertIn("STALE", rows[0])
        self.assertIn("3.5", rows[0])
        # values survive the staleness
        self.assertTrue(any("980" in r for r in rows))

    def test_session_change_counts_restart(self):
        m = self.fresh(wall=100.0)
        m.update(dict(DGRAM, session=0x1234, seq=0), 101.0)
        rows = render_rows(m, wall=101.1, width=GRID_WIDTH)
        self.assertTrue(any("restarts 1" in r for r in rows))

    def test_fec_rows_sticky(self):
        m = self.fresh()
        no_fec = dict(DGRAM, fec=[])
        m.update(no_fec, 101.0)
        rows = render_rows(m, wall=101.1, width=GRID_WIDTH)
        self.assertTrue(any(r.lstrip().startswith("s0") for r in rows))

    def test_unsupported_version_banner(self):
        m = Model()
        m.update(dict(DGRAM, v=2), 100.0)
        rows = render_rows(m, wall=100.1, width=GRID_WIDTH)
        self.assertTrue(any("unsupported schema" in r for r in rows))

    def test_narrow_terminal_single_line(self):
        rows = render_rows(self.fresh(), wall=100.2, width=40)
        self.assertEqual(len(rows), 1)
        for cell in ("session", "mcs5", "59.9", "9.31"):
            self.assertIn(cell, rows[0])

    def test_update_ignores_non_dict_input(self):
        # A UDP datagram that is valid JSON but not an object (null, a bare
        # number, a list, ...) must not crash Model.update or disturb the
        # last good state.
        m = self.fresh(wall=100.0)
        before = (m.d, m.session, m.last_rx_wall, m.restarts,
                  dict(m.fec_rows), m.bad_version)
        for bad in (None, 42, [1, 2, 3], "oops", 3.14):
            m.update(bad, 999.0)
        after = (m.d, m.session, m.last_rx_wall, m.restarts,
                 dict(m.fec_rows), m.bad_version)
        self.assertEqual(before, after)


if __name__ == "__main__":
    unittest.main()
