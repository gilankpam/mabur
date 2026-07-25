import unittest

from maburtop import CARD_COLS, SIG_COLS, STRM_COLS, Model, render_rows, GRID_WIDTH

DGRAM = {
    "v": 1, "session": 0xDEADBEEF, "seq": 7, "t_ms": 567890,
    "link": {
        "vtx_id": 1, "state": "session", "tx_card": 0,
        "op": {"mcs": 5, "bw": 20, "sgi": False, "vht": False,
               "overhead": 0.25, "offset_qdb": 0, "snr_req": 18.5},
        "deadline_ms": 60, "residual_loss": 0.012,
        "layer_delivery_pct": [100, 100, 97, 91],
        "streams": [
            {"stream": 0, "ov": 1.0, "recovered_s": 12.0, "abandoned_s": 0.0,
             "syms_in_s": 4210.0, "recovered": 4021, "abandoned": 3,
             "stale": 0, "bad_cfg": 0, "sub_fail": 1, "in_flight": 2},
            {"stream": 1, "ov": 0.25, "recovered_s": 50.0, "abandoned_s": 0.0,
             "syms_in_s": 10876.0, "recovered": 9000, "abandoned": 0,
             "stale": 0, "bad_cfg": 0, "sub_fail": 0, "in_flight": 0},
        ],
        "video": {"fps": 59.9, "mbps": 9.31, "jitter_ms": 1.8,
                  "clean": 21500, "truncated": 3, "dropped": 0, "stall_resets": 0,
                  "rtp": {"ok": 812345, "gap": 2, "gap_seqs": 9, "back": 0},
                  "udp": {"sent": 812347, "failed": 0, "bytes": 123456789},
                  "q_drop": 0},
    },
    "cards": [
        {"id": 0, "up": True, "frames": 123456, "crc_fail": 0,
         "loss_pct": 0.0, "rx_mbps": 15.6, "pps": 1450,
         "last_frame_age_ms": 4, "foreign_pps": 3.2, "self_pps": 20.1,
         "classes": {
             "s1": {"pps": 890.0, "rssi": -50.1, "rssi_a": -50.9, "rssi_b": -52.3,
                    "snr": 27.1, "snr_a": 26.0, "snr_b": 24.5},
             "ctrl": {"pps": 1.0, "rssi": -47.2, "rssi_a": -47.9, "rssi_b": -49.0,
                      "snr": 25.0, "snr_a": 24.1, "snr_b": 23.6},
         }},
        {"id": 1, "up": True, "frames": 120000, "crc_fail": 0,
         "loss_pct": 0.0, "rx_mbps": 15.6, "pps": 1438,
         "last_frame_age_ms": 5, "foreign_pps": 1.0, "self_pps": 19.8,
         "classes": {
             "s1": {"pps": 885.0, "rssi": -53.2, "rssi_a": -53.8, "rssi_b": -55.1,
                    "snr": 25.2, "snr_a": 24.1, "snr_b": 22.9},
             "ctrl": {"pps": 1.0, "rssi": -49.9, "rssi_a": -50.2, "rssi_b": -51.7,
                      "snr": 24.4, "snr_a": 23.8, "snr_b": 22.5},
         }},
    ],
}


class RenderTest(unittest.TestCase):
    def fresh(self, dgram=None, wall=100.0):
        m = Model()
        m.update(dgram or DGRAM, wall)
        return m

    def test_header_and_card_row_content(self):
        rows = render_rows(self.fresh(), wall=100.2, width=GRID_WIDTH)
        self.assertIn("SESSION", rows[0])
        self.assertIn("vtx 1", rows[0])
        self.assertIn("MCS 5/20", rows[0])
        card = next(r for r in rows if r.lstrip().startswith("c0"))
        for cell in ("UP", "1450", "15.6", "20.1", "3.2"):
            self.assertIn(cell, card)

    def test_sig_row_content(self):
        rows = render_rows(self.fresh(), wall=100.2, width=GRID_WIDTH)
        sig_header_idx = next(i for i, r in enumerate(rows) if r.startswith("SIG"))
        sig_rows = rows[sig_header_idx + 1:]
        c0_s1 = next(r for r in sig_rows if r.lstrip().startswith("c0")
                     and "s1" in r)
        for cell in ("890", "-50.1", "-50.9", "-52.3", "27.1", "26.0", "24.5"):
            self.assertIn(cell, c0_s1)
        # "ctrl" class renders as "ctl" (4-wide cls column)
        c0_ctrl = next(r for r in sig_rows if r.lstrip().startswith("c0")
                       and "ctl" in r)
        self.assertIn("-47.2", c0_ctrl)

    def test_strm_row_content(self):
        rows = render_rows(self.fresh(), wall=100.2, width=GRID_WIDTH)
        s0 = next(r for r in rows if r.lstrip().startswith("s0"))
        for cell in ("1.00", "12.0", "4210"):
            self.assertIn(cell, s0)
        s1 = next(r for r in rows if r.lstrip().startswith("s1"))
        self.assertIn("0.25", s1)

    def test_fixed_width_rows_never_shift(self):
        rows_a = render_rows(self.fresh(), wall=100.2, width=GRID_WIDTH)
        big = dict(DGRAM)
        big["cards"] = [dict(DGRAM["cards"][0],
                             frames=999999999, pps=99999, rx_mbps=999.9,
                             last_frame_age_ms=15000),
                        DGRAM["cards"][1]]
        rows_b = render_rows(self.fresh(big), wall=100.2, width=GRID_WIDTH)
        self.assertEqual([len(r) for r in rows_a], [len(r) for r in rows_b])

    def test_fixed_width_survives_huge_last_frame_age(self):
        # A card silent for minutes (age >= 10s) must not widen the CARD row
        # (regression: naive "{ms}ms".rjust(6) overflowed past 10000 ms).
        rows_a = render_rows(self.fresh(), wall=100.2, width=GRID_WIDTH)
        huge = dict(DGRAM)
        huge["cards"] = [dict(DGRAM["cards"][0], last_frame_age_ms=123456789),
                         DGRAM["cards"][1]]
        rows_b = render_rows(self.fresh(huge), wall=100.2, width=GRID_WIDTH)
        self.assertEqual([len(r) for r in rows_a], [len(r) for r in rows_b])

    def test_null_renders_dashes(self):
        d = dict(DGRAM)
        d["cards"] = [dict(DGRAM["cards"][0], loss_pct=None, foreign_pps=None)]
        card = next(r for r in render_rows(self.fresh(d), 100.2, GRID_WIDTH)
                    if r.lstrip().startswith("c0"))
        self.assertIn("--", card)

    def test_stale_banner_after_2s(self):
        rows = render_rows(self.fresh(wall=100.0), wall=103.5, width=GRID_WIDTH)
        self.assertIn("STALE", rows[0])
        self.assertIn("3.5", rows[0])
        # values survive the staleness
        self.assertTrue(any("1450" in r for r in rows))

    def test_session_change_counts_restart(self):
        m = self.fresh(wall=100.0)
        m.update(dict(DGRAM, session=0x1234, seq=0), 101.0)
        rows = render_rows(m, wall=101.1, width=GRID_WIDTH)
        self.assertTrue(any("restarts 1" in r for r in rows))

    def test_strm_rows_sticky(self):
        m = self.fresh()
        no_streams = dict(DGRAM)
        no_streams["link"] = dict(DGRAM["link"], streams=[])
        m.update(no_streams, 101.0)
        rows = render_rows(m, wall=101.1, width=GRID_WIDTH)
        self.assertTrue(any(r.lstrip().startswith("s0") for r in rows))
        self.assertTrue(any(r.lstrip().startswith("s1") for r in rows))

    def test_sig_rows_sticky(self):
        # A class vanishing from a later datagram must not drop its row —
        # the frozen EMAs are exactly what you want to read on a dead class.
        m = self.fresh()
        no_classes = dict(DGRAM)
        no_classes["cards"] = [dict(c, classes={}) for c in DGRAM["cards"]]
        m.update(no_classes, 101.0)
        rows = render_rows(m, wall=101.1, width=GRID_WIDTH)
        sig_header_idx = next(i for i, r in enumerate(rows) if r.startswith("SIG"))
        sig_rows = rows[sig_header_idx + 1:]
        self.assertTrue(any(r.lstrip().startswith("c0") and "s1" in r
                            for r in sig_rows))
        self.assertTrue(any(r.lstrip().startswith("c1") and "ctl" in r
                            for r in sig_rows))

    def test_unsupported_version_banner(self):
        m = Model()
        m.update(dict(DGRAM, v=2), 100.0)
        rows = render_rows(m, wall=100.1, width=GRID_WIDTH)
        self.assertTrue(any("unsupported schema" in r for r in rows))

    def test_header_and_data_columns_align(self):
        # Every column is right-aligned, so a title's right edge must be the
        # right edge of the value below it — for all three grids.
        rows = render_rows(self.fresh(), wall=100.2, width=GRID_WIDTH)
        for header_key, data_key, cols in (
            ("CARD", "c0", CARD_COLS),
            ("SIG", "c0", SIG_COLS),
            ("STRM", "s0", STRM_COLS),
        ):
            h_idx = next(i for i, r in enumerate(rows) if r.startswith(header_key))
            header = rows[h_idx]
            data = next(r for r in rows[h_idx + 1:]
                       if r.lstrip().startswith(data_key))
            for title, _ in cols:
                end = header.index(title) + len(title)
                self.assertNotEqual(
                    data[end - 1], " ",
                    f"{header_key}/{title}: no value ending at col {end}\n"
                    f"{header!r}\n{data!r}")
                self.assertTrue(
                    end == len(data) or data[end] == " ",
                    f"{header_key}/{title}: value overruns col {end}\n"
                    f"{header!r}\n{data!r}")

    def test_narrow_terminal_single_line(self):
        rows = render_rows(self.fresh(), wall=100.2, width=40)
        self.assertEqual(len(rows), 1)
        for cell in ("session", "mcs5", "59.9", "9.31"):
            self.assertIn(cell, rows[0])

    def test_narrow_fallback_stale_prefix(self):
        # Narrow fallback must prefix with "STALE " when feed is >2s stale
        m = self.fresh(wall=100.0)
        # Not stale yet
        rows = render_rows(m, wall=101.5, width=40)
        self.assertEqual(len(rows), 1)
        self.assertNotIn("STALE", rows[0])
        # Now stale (>2s)
        rows = render_rows(m, wall=103.5, width=40)
        self.assertEqual(len(rows), 1)
        self.assertIn("STALE", rows[0])

    def test_card_missing_id_does_not_poison_sig_rows(self):
        # A card dict missing "id" must not enter sig_rows with a None key —
        # a mix of int and None keys makes sorted() in render_rows raise
        # TypeError on every redraw (regression: permanently blank screen).
        d = dict(DGRAM)
        d["cards"] = list(DGRAM["cards"]) + [
            {"up": True, "frames": 1, "crc_fail": 0, "loss_pct": 0.0,
             "rx_mbps": 0.0, "pps": 0,
             "classes": {"s0": {"pps": 1.0, "rssi": -40.0}}},
        ]
        m = self.fresh(d)
        self.assertNotIn(None, [cid for cid, _cls in m.sig_rows])
        rows = render_rows(m, wall=100.2, width=GRID_WIDTH)
        self.assertTrue(len(rows) > 0)

    def test_update_ignores_non_dict_input(self):
        # A UDP datagram that is valid JSON but not an object (null, a bare
        # number, a list, ...) must not crash Model.update or disturb the
        # last good state.
        m = self.fresh(wall=100.0)
        before = (m.d, m.session, m.last_rx_wall, m.restarts,
                  dict(m.strm_rows), dict(m.sig_rows), m.bad_version)
        for bad in (None, 42, [1, 2, 3], "oops", 3.14):
            m.update(bad, 999.0)
        after = (m.d, m.session, m.last_rx_wall, m.restarts,
                 dict(m.strm_rows), dict(m.sig_rows), m.bad_version)
        self.assertEqual(before, after)


if __name__ == "__main__":
    unittest.main()
