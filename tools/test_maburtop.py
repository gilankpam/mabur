import unittest

from maburtop import (
    Model, render_screen, render_rows_compact, hstack,
    panel_topbar, panel_drone, panel_video, panel_links, panel_gs_radios,
    panel_ladder,
    CARD_COLS, RADIO_COLS, GRID_WIDTH,
)

DGRAM = {
    "v": 1, "session": 0xDEADBEEF, "seq": 7, "t_ms": 567890,
    "link": {
        "vtx_id": 1, "state": "session", "tx_card": 0,
        "op": {"mcs": 5, "bw": 20, "sgi": False, "vht": False,
               "overhead_base": 0.25, "overhead_enh": 0.25, "snr_req": 18.5},
        "deadline_ms": 60, "residual_loss": 0.012,
        "layer_delivery_pct": [100, 100],
        "streams": [
            {"stream": 0, "ov": 1.0, "rung_mcs": 5, "rung_ldpc": True,
             "rung_stbc": True, "phy_mbps": 52.0, "inj_kbps": 650.0,
             "recovered_s": 12.0, "abandoned_s": 0.0,
             "syms_in_s": 4210.0, "recovered": 4021, "abandoned": 3,
             "stale": 0, "bad_cfg": 0, "sub_fail": 1, "in_flight": 2},
            {"stream": 1, "ov": 0.25, "rung_mcs": 5, "rung_ldpc": True,
             "rung_stbc": True, "phy_mbps": 52.0, "inj_kbps": 15600.0,
             "recovered_s": 50.0, "abandoned_s": 0.0,
             "syms_in_s": 10876.0, "recovered": 9000, "abandoned": 0,
             "stale": 0, "bad_cfg": 0, "sub_fail": 0, "in_flight": 0},
        ],
        "air_pct": 31.3,
        "video": {"fps": 59.9, "mbps": 9.31, "jitter_ms": 1.8,
                  "clean": 21500, "truncated": 3, "dropped": 0, "stall_resets": 0,
                  "ring": {"published": 812345, "dropped_oversize": 0,
                           "bytes": 123456789},
                  "q_drop": 0},
    },
    "drone": {
        "tlm_age_ms": 800, "tlm_seq": 4211, "state": "linked",
        "gen": 7, "failsafe_shed": False, "radio_rx_ok": True,
        "congestion_shed": True,
        "applied": {"mcs": 5, "bw": 20, "vht": False,
                    "overhead_base": 0.25, "overhead_enh": 0.25},
        "rcf": {"age_ms": 45, "rx_pps": 19.4},
        "enc": {"fps": 59.9, "mbps": 9.21, "cmd_kbps": 9000, "qp": 31,
                "roi_qp": -24, "ring_drops": 0},
        "txq": {"depth": 3, "cap": 64, "drop_pps": 0.0, "drops": 0},
        "radio": {"sent_pps": 1461.0, "drops": 0, "usb_fail": 0},
        "uplink": {"rssi_a": -58.9, "rssi_b": -58.0, "snr_a": 21.0, "snr_b": 22.0},
        "sys": {"soc_temp_c": 61, "thermal_delta": 3, "load": 0.72},
    },
    "cards": [
        {"id": 0, "up": True, "frames": 123456, "crc_fail": 0,
         "loss_pct": 0.0, "rx_mbps": 15.6, "pps": 1450,
         "last_frame_age_ms": 4, "foreign_pps": 3.2, "self_pps": 20.1,
         "inj_pps": 1455.0, "tx_pps": 0.0, "tx_fail": 0,
         "classes": {
             "s1": {"pps": 890.0, "mbps": 14.2, "rssi": -50.1, "rssi_a": -50.9, "rssi_b": -52.3,
                    "snr": 27.1, "snr_a": 26.0, "snr_b": 24.5},
             "msp": {"pps": 8.0, "mbps": 0.084, "rssi": -49.0, "rssi_a": -50.0,
                     "rssi_b": -49.0, "snr": 67.0, "snr_a": 66.9, "snr_b": 66.1},
             "ctrl": {"pps": 1.0, "mbps": 0.004, "rssi": -47.2, "rssi_a": -47.9, "rssi_b": -49.0,
                      "snr": 25.0, "snr_a": 24.1, "snr_b": 23.6},
         }},
        {"id": 1, "up": True, "frames": 120000, "crc_fail": 0,
         "loss_pct": 0.0, "rx_mbps": 15.6, "pps": 1438,
         "last_frame_age_ms": 5, "foreign_pps": 1.0, "self_pps": 19.8,
         "inj_pps": 1455.0, "tx_pps": 20.0, "tx_fail": 1,
         "classes": {
             "s1": {"pps": 885.0, "mbps": 14.1, "rssi": -53.2, "rssi_a": -53.8, "rssi_b": -55.1,
                    "snr": 25.2, "snr_a": 24.1, "snr_b": 22.9},
             "ctrl": {"pps": 1.0, "mbps": 0.003, "rssi": -49.9, "rssi_a": -50.2, "rssi_b": -51.7,
                      "snr": 24.4, "snr_a": 23.8, "snr_b": 22.5},
         }},
    ],
}


def texts(rows):
    return [t for t, _ in rows]


def _fresh(dgram=None, wall=100.0):
    m = Model()
    m.update(dgram or DGRAM, wall)
    return m


class TopBarTest(unittest.TestCase):
    def test_content(self):
        text, spans = panel_topbar(_fresh(), 100.2)[0]
        self.assertIn("SESSION", text)
        self.assertIn("vtx 1", text)
        self.assertIn("MCS 5/20", text)
        self.assertIn("restarts 0", text)
        self.assertTrue(any(style == "good" for _, _, style in spans))

    def test_stale_takeover(self):
        m = _fresh(wall=100.0)
        text, spans = panel_topbar(m, 103.5)[0]
        self.assertIn("STALE", text)
        self.assertIn("3.5", text)
        self.assertTrue(any(style == "rev" for _, _, style in spans))
        self.assertTrue(any(style == "dim" for _, _, style in spans))
        # frozen bar content (session id etc) still present under the takeover
        self.assertIn("session 0xdeadbeef", text)

    def test_beaconing_dot_warn_span(self):
        d = dict(DGRAM, link=dict(DGRAM["link"], state="beaconing"))
        text, spans = panel_topbar(_fresh(d), 100.2)[0]
        self.assertIn("BEACONING", text)
        self.assertTrue(any(style == "warn" for _, _, style in spans))
        self.assertFalse(any(style == "good" for _, _, style in spans))

    def test_unsupported_version_banner(self):
        m = Model()
        m.update(dict(DGRAM, v=2), 100.0)
        rows = panel_topbar(m, 100.1)
        self.assertTrue(any("unsupported schema v=2" in t for t, _ in rows))

    def test_session_change_counts_restart(self):
        m = _fresh(wall=100.0)
        m.update(dict(DGRAM, session=0x1234, seq=0), 101.0)
        text, _ = panel_topbar(m, 101.1)[0]
        self.assertIn("restarts 1", text)


class DronePanelTest(unittest.TestCase):
    def test_content(self):
        rows = panel_drone(_fresh(), 100.2)
        self.assertTrue(rows[0][0].startswith("──"))
        joined = "\n".join(texts(rows))
        for cell in ("LINKED", "gen", "7", "mcs5/20", "ov b0.25/e0.25", "800ms",
                     "59.9 fps", "9.21 Mbps", "9000k", "qp 31", "roi -24",
                     "shed CONG", "1461",
                     "-58.9", "-58.0", "19.4", " 61", "0.72"):
            self.assertIn(cell, joined)

    def test_shed_cell_states(self):
        # failsafe wins the label (rung-0 forced shed), then congestion,
        # then a plain "-" when neither; an old recording without the
        # congestion key renders "--" not a crash.
        d = dict(DGRAM)
        d["drone"] = dict(DGRAM["drone"], failsafe_shed=True, congestion_shed=True)
        self.assertIn("shed FS", "\n".join(texts(panel_drone(_fresh(d), 100.2))))
        d["drone"] = dict(DGRAM["drone"], failsafe_shed=False, congestion_shed=False)
        self.assertIn("shed off", "\n".join(texts(panel_drone(_fresh(d), 100.2))))
        d["drone"] = dict(DGRAM["drone"])
        del d["drone"]["congestion_shed"]
        self.assertIn("shed --", "\n".join(texts(panel_drone(_fresh(d), 100.2))))

    def test_null_telemetry_single_line(self):
        d = dict(DGRAM, drone=None)
        rows = panel_drone(_fresh(d), 100.2)
        body = rows[1:]
        self.assertEqual(len(body), 1)
        text, spans = body[0]
        self.assertIn("no telemetry", text)
        self.assertEqual(spans, [(0, len(text), "dim")])

    def test_applied_mismatch_bad_span(self):
        d = dict(DGRAM)
        d["drone"] = dict(DGRAM["drone"],
                           applied=dict(DGRAM["drone"]["applied"], mcs=6))
        rows = panel_drone(_fresh(d), 100.2)
        text, spans = next((t, s) for t, s in rows if t.startswith("applied"))
        bad = [sp for sp in spans if sp[2] == "bad"]
        self.assertTrue(bad)
        st, ln, _ = bad[0]
        self.assertIn("mcs6/20", text[st:st + ln])

    def test_ov_split_renders_base_and_enh_no_mismatch_span(self):
        # applied.overhead_{base,enh} is not diffed against op's pair the
        # way mcs/bw are: the runtime balancer that used to explain a split
        # is deleted (2026-08-30 same-rate-fixed-pairs), so applied ≡
        # commanded except under an armed :8301 override — no bad span
        # here either way, this panel just doesn't render that comparison.
        d = dict(DGRAM)
        d["drone"] = dict(DGRAM["drone"],
                           applied=dict(DGRAM["drone"]["applied"],
                                        overhead_base=0.5, overhead_enh=1.0))
        rows = panel_drone(_fresh(d), 100.2)
        text, spans = next((t, s) for t, s in rows if t.startswith("applied"))
        self.assertIn("ov b0.50/e1.00", text)
        self.assertFalse(any(style == "bad" for _, _, style in spans))

    def test_deaf_cell_bad_span(self):
        d = dict(DGRAM)
        d["drone"] = dict(DGRAM["drone"], radio_rx_ok=False)
        rows = panel_drone(_fresh(d), 100.2)
        text, spans = next((t, s) for t, s in rows if t.startswith("system"))
        self.assertIn("DEAF", text)
        self.assertTrue(any(text[st:st + ln] == "DEAF" and style == "bad"
                             for st, ln, style in spans))

    def test_radio_drops_increased_bad_span(self):
        m = _fresh()
        d2 = dict(DGRAM)
        d2["drone"] = dict(DGRAM["drone"],
                            radio=dict(DGRAM["drone"]["radio"], drops=1))
        m.update(d2, 101.0)
        rows = panel_drone(m, 101.1)
        text, spans = next((t, s) for t, s in rows if t.startswith("radio"))
        self.assertTrue(any(style == "bad" for _, _, style in spans))

    def test_radio_drops_not_flagged_on_first_sample(self):
        # No previous datagram yet: an increase can't be judged, so no bad
        # span even though drops is nonzero.
        d = dict(DGRAM)
        d["drone"] = dict(DGRAM["drone"],
                           radio=dict(DGRAM["drone"]["radio"], drops=5))
        rows = panel_drone(_fresh(d), 100.2)
        text, spans = next((t, s) for t, s in rows if t.startswith("radio"))
        self.assertFalse(any(style == "bad" for _, _, style in spans))


class VideoPanelTest(unittest.TestCase):
    def test_content(self):
        rows = panel_video(_fresh(), 100.2)
        joined = "\n".join(texts(rows))
        for cell in ("59.9 fps", "9.31 Mbps", "jitter", "1.8 ms", "clean",
                     "21500", "trunc", "drop", "812345", "ring",
                     "pub", "q_drop", "residual"):
            self.assertIn(cell, joined)

    def test_cross_check_present_with_telemetry(self):
        rows = panel_video(_fresh(), 100.2)
        joined = "\n".join(texts(rows))
        self.assertIn("──►", joined)
        self.assertIn("encoder", joined)
        self.assertIn("out", joined)
        self.assertIn("sent", joined)
        self.assertIn("inj", joined)

    def test_cross_check_absent_without_telemetry(self):
        d = dict(DGRAM, drone=None)
        rows = panel_video(_fresh(d), 100.2)
        joined = "\n".join(texts(rows))
        self.assertNotIn("──►", joined)

    def test_cross_check_bad_span_on_fps_mismatch(self):
        d = dict(DGRAM)
        d["drone"] = dict(DGRAM["drone"],
                           enc=dict(DGRAM["drone"]["enc"], fps=10.0))
        rows = panel_video(_fresh(d), 100.2)
        text, spans = next((t, s) for t, s in rows if "──►" in t)
        self.assertTrue(any(style == "bad" for _, _, style in spans))

    def test_cross_check_good_span_within_tolerance(self):
        rows = panel_video(_fresh(), 100.2)
        text, spans = next((t, s) for t, s in rows if "──►" in t)
        self.assertTrue(any(style == "good" for _, _, style in spans))

    def test_increased_truncated_bad_span(self):
        m = _fresh()
        d2 = dict(DGRAM)
        d2["link"] = dict(DGRAM["link"],
                           video=dict(DGRAM["link"]["video"], truncated=4))
        m.update(d2, 101.0)
        rows = panel_video(m, 101.1)
        text, spans = next((t, s) for t, s in rows if t.startswith("frames"))
        bad = [sp for sp in spans if sp[2] == "bad"]
        self.assertTrue(bad)
        st, ln, _ = bad[0]
        self.assertIn("trunc", text[st:st + ln])

    def test_jitter_warn_span(self):
        d = dict(DGRAM)
        d["link"] = dict(DGRAM["link"],
                          video=dict(DGRAM["link"]["video"], jitter_ms=25.0))
        rows = panel_video(_fresh(d), 100.2)
        text, spans = next((t, s) for t, s in rows if t.startswith("out"))
        self.assertTrue(any(style == "warn" for _, _, style in spans))


class LinksPanelTest(unittest.TestCase):
    def _block(self, rows, prefix):
        out = []
        started = False
        for t, s in rows:
            if not started:
                if t.strip().startswith(prefix):
                    started = True
                else:
                    continue
            elif t == "":
                break
            out.append((t, s))
        return out

    def test_block_order_fixed(self):
        rows = panel_links(_fresh(), 100.2)
        txt = texts(rows)
        idx = {}
        for label in ("s0 ·", "s1 ·", "msp ·", "ctl ·"):
            idx[label] = next(i for i, t in enumerate(txt) if t.strip().startswith(label))
        self.assertLess(idx["s0 ·"], idx["s1 ·"])
        self.assertLess(idx["s1 ·"], idx["msp ·"])
        self.assertLess(idx["msp ·"], idx["ctl ·"])

    def test_tx_decode_radio_row_content(self):
        rows = panel_links(_fresh(), 100.2)
        block = self._block(rows, "s1 ·")
        joined = "\n".join(t for t, _ in block)
        for cell in ("mcs5+LS", "52.0M", "ov 0.25", "dlv 100%", "inj ~15.6M",
                     "rec/s   50.0", "in/s  10876", "sfail  0", "flt  0",
                     "890", "14200", "-50.1", "-50.9", "-52.3", "27.1"):
            self.assertIn(cell, joined)

    def test_msp_ctl_annotation_and_header_repeat(self):
        rows = panel_links(_fresh(), 100.2)
        msp_block = self._block(rows, "msp ·")
        self.assertIn("no fec decode", msp_block[0][0])
        self.assertTrue(any(t.strip().startswith("radio") for t, _ in msp_block))
        ctl_block = self._block(rows, "ctl ·")
        self.assertIn("rendezvous", ctl_block[0][0])
        self.assertTrue(any(t.strip().startswith("radio") for t, _ in ctl_block))

    def test_sticky_block_survives_class_disappearance_and_dims(self):
        # Dormancy has a CLASS_DORMANT_S memory: a class that was active
        # moments ago must NOT dim just because this 500 ms rate window
        # read 0 pps (a healthy 1 Hz ctl/telemetry beat alternates windows
        # — the flicker this rule exists to prevent). Only sustained
        # silence dims the block.
        m = _fresh()
        no_classes = dict(DGRAM)
        no_classes["cards"] = [dict(c, classes={}) for c in DGRAM["cards"]]
        m.update(no_classes, 101.0)
        rows = panel_links(m, 101.1)          # 1.1 s after last activity
        block = self._block(rows, "s1 ·")
        joined = "\n".join(t for t, _ in block)
        self.assertIn("c0", joined)  # frozen radio row survives
        self.assertFalse(any(style == "dim"
                             for _, spans in block for _, _, style in spans),
                         "recently-active block must not flicker dim")
        rows = panel_links(m, 104.5)          # > CLASS_DORMANT_S of silence
        block = self._block(rows, "s1 ·")
        for t, spans in block:
            if t:
                self.assertTrue(any(style == "dim" for _, _, style in spans),
                                 f"expected dim span on dormant block row: {t!r}")

    def test_strm_rows_sticky(self):
        m = _fresh()
        no_streams = dict(DGRAM)
        no_streams["link"] = dict(DGRAM["link"], streams=[])
        m.update(no_streams, 101.0)
        rows = panel_links(m, 101.1)
        joined = "\n".join(t for t, _ in rows)
        self.assertIn("s0 ·", joined)
        self.assertIn("s1 ·", joined)

    def test_dlv_bad_and_warn_thresholds(self):
        d = dict(DGRAM)
        d["link"] = dict(DGRAM["link"], layer_delivery_pct=[100, 85])
        rows = panel_links(_fresh(d), 100.2)
        block = self._block(rows, "s1 ·")
        text, spans = block[0]
        self.assertTrue(any(style == "bad" for _, _, style in spans))

    def test_abandoned_and_subfail_bad_spans(self):
        d = dict(DGRAM)
        streams = [dict(s) for s in DGRAM["link"]["streams"]]
        streams[1]["abandoned_s"] = 2.0
        streams[1]["sub_fail"] = 3
        d["link"] = dict(DGRAM["link"], streams=streams)
        rows = panel_links(_fresh(d), 100.2)
        block = self._block(rows, "s1 ·")
        decode_text, decode_spans = block[1]
        styles = [sp[2] for sp in decode_spans]
        self.assertIn("bad", styles)
        self.assertEqual(len(decode_spans), 2)

    def test_radio_table_alignment(self):
        rows = panel_links(_fresh(), 100.2)
        block = self._block(rows, "s1 ·")
        header = next(t for t, _ in block if t.strip().startswith("radio"))
        data = next(t for t, _ in block if t.lstrip().startswith("c0"))
        for title, _w in RADIO_COLS:
            end = header.index(title) + len(title)
            self.assertNotEqual(data[end - 1], " ",
                                 f"{title}: no value ending at col {end}\n{header!r}\n{data!r}")
            self.assertTrue(end == len(data) or data[end] == " ",
                             f"{title}: value overruns col {end}\n{header!r}\n{data!r}")

    def test_no_link_data(self):
        d = dict(DGRAM, link={}, cards=[])
        rows = panel_links(_fresh(d), 100.2)
        self.assertTrue(any("no link data" in t for t, _ in rows))

    def test_ctl_row_absent_when_no_ctl(self):
        rows = panel_links(_fresh(), 100.2)  # DGRAM carries no link.ctl
        joined = "\n".join(t for t, _ in rows)
        self.assertNotIn("of budget", joined)

    def test_ctl_row_content_and_good_span(self):
        d = dict(DGRAM)
        d["link"] = dict(DGRAM["link"], ctl={
            "rung": {"idx": 3, "mcs": 5, "ov_base": 0.25, "ov_enh": 0.10},
            "util": 0.08, "pre_fec_loss": 0.035, "budget": 0.43,
            "probation_ms_left": 0, "penalized": [],
            "counters": {"demotes_residual": 0, "demotes_util": 3, "promotes": 4,
                         "probation_fails": 1, "starved_drops": 0, "timeout_drops": 1},
            "last_event": {"t_ms": 39243748, "from": 4, "to": 3,
                           "reason": "util", "u": 0.65},
        })
        rows = panel_links(_fresh(d), 100.2)
        text, spans = next((t, s) for t, s in rows if "of budget" in t)
        self.assertIn("rung 3 (mcs5 ov b0.25/e0.10)", text)
        self.assertIn("u=0.08", text)
        self.assertIn("of budget 43%", text)
        self.assertIn("[util@0.65]", text)
        self.assertTrue(any(style == "good" for _, _, style in spans))

    def test_ctl_row_warn_and_bad_util_thresholds(self):
        base_ctl = {
            "rung": {"idx": 3, "mcs": 5, "ov_base": 0.25, "ov_enh": 0.10},
            "pre_fec_loss": 0.1, "budget": 0.43,
            "probation_ms_left": 0, "penalized": [],
            "counters": {"demotes_residual": 0, "demotes_util": 0, "promotes": 0,
                         "probation_fails": 0, "starved_drops": 0, "timeout_drops": 0},
            "last_event": {"t_ms": 0, "from": 0, "to": 0, "reason": "none", "u": 0.0},
        }
        d_warn = dict(DGRAM)
        d_warn["link"] = dict(DGRAM["link"], ctl=dict(base_ctl, util=0.45))
        rows = panel_links(_fresh(d_warn), 100.2)
        _text, spans = next((t, s) for t, s in rows if "of budget" in t)
        self.assertTrue(any(style == "warn" for _, _, style in spans))

        d_bad = dict(DGRAM)
        d_bad["link"] = dict(DGRAM["link"], ctl=dict(base_ctl, util=0.75))
        rows = panel_links(_fresh(d_bad), 100.2)
        _text, spans = next((t, s) for t, s in rows if "of budget" in t)
        self.assertTrue(any(style == "bad" for _, _, style in spans))

    def test_ladder_rung_rows_render_overhead_pair(self):
        # Per-rung overhead is a base/enh pair (same-rate-fixed-pairs); the
        # removed scalar 'ov' key must not resurface as '--'.
        d = dict(DGRAM)
        d["link"] = dict(DGRAM["link"], ctl={
            "rung": {"idx": 1, "mcs": 3, "ov_base": 0.5, "ov_enh": 0.25},
            "util": 0.1, "down_util": 0.05, "up_util": 0.5,
            "pre_fec_loss": 0.01, "budget": 0.4,
            "probation_ms_left": 0, "penalized": [],
            "ladder": [
                {"mcs": 1, "ov_base": 1.0, "ov_enh": 1.0},
                {"mcs": 3, "ov_base": 0.5, "ov_enh": 0.25},
            ],
            "counters": {"demotes_residual": 0, "demotes_util": 0, "promotes": 0,
                         "probation_fails": 0, "starved_drops": 0, "timeout_drops": 0},
            "last_event": {"t_ms": 0, "from": 0, "to": 0, "reason": "none", "u": 0.0},
        })
        rows = panel_ladder(_fresh(d), 100.2)
        joined = "\n".join(t for t, _ in rows)
        self.assertIn("1 mcs3/ov0.50:0.25", joined)
        self.assertIn("0 mcs1/ov1.00:1.00", joined)
        self.assertNotIn("--", joined.split("\n")[1])  # current rung row


class GsRadiosPanelTest(unittest.TestCase):
    def test_content(self):
        rows = panel_gs_radios(_fresh(), 100.2)
        joined = "\n".join(texts(rows))
        for cell in ("GS RADIOS", "UP", "1450", "1455", "15.6", "20.1", "3.2"):
            self.assertIn(cell, joined)

    def test_alignment(self):
        rows = panel_gs_radios(_fresh(), 100.2)
        header = rows[1][0]
        data = next(t for t, _ in rows if t.lstrip().startswith("c0"))
        for title, _w in CARD_COLS:
            end = header.index(title) + len(title)
            self.assertNotEqual(data[end - 1], " ",
                                 f"{title}: no value ending at col {end}\n{header!r}\n{data!r}")
            self.assertTrue(end == len(data) or data[end] == " ",
                             f"{title}: value overruns col {end}\n{header!r}\n{data!r}")

    def test_loss_threshold_spans(self):
        d = dict(DGRAM)
        d["cards"] = [dict(DGRAM["cards"][0], loss_pct=10.0), DGRAM["cards"][1]]
        rows = panel_gs_radios(_fresh(d), 100.2)
        _text, spans = next((t, s) for t, s in rows if t.lstrip().startswith("c0"))
        self.assertTrue(any(style == "bad" for _, _, style in spans))

        d2 = dict(DGRAM)
        d2["cards"] = [dict(DGRAM["cards"][0], loss_pct=1.0), DGRAM["cards"][1]]
        rows2 = panel_gs_radios(_fresh(d2), 100.2)
        _text2, spans2 = next((t, s) for t, s in rows2 if t.lstrip().startswith("c0"))
        self.assertTrue(any(style == "warn" for _, _, style in spans2))

    def test_crc_increased_bad_span(self):
        m = _fresh()
        d2 = dict(DGRAM)
        d2["cards"] = [dict(DGRAM["cards"][0], crc_fail=1), DGRAM["cards"][1]]
        m.update(d2, 101.0)
        rows = panel_gs_radios(m, 101.1)
        _text, spans = next((t, s) for t, s in rows if t.lstrip().startswith("c0"))
        self.assertTrue(any(style == "bad" for _, _, style in spans))

    def test_down_bad_and_txf_bad(self):
        d = dict(DGRAM)
        d["cards"] = [dict(DGRAM["cards"][0], up=False),
                      dict(DGRAM["cards"][1], tx_fail=2)]
        rows = panel_gs_radios(_fresh(d), 100.2)
        _t0, s0 = next((t, s) for t, s in rows if t.lstrip().startswith("c0"))
        self.assertTrue(any(style == "bad" for _, _, style in s0))
        _t1, s1 = next((t, s) for t, s in rows if t.lstrip().startswith("c1"))
        self.assertTrue(any(style == "bad" for _, _, style in s1))

    def test_no_cards(self):
        d = dict(DGRAM, cards=[])
        rows = panel_gs_radios(_fresh(d), 100.2)
        self.assertTrue(any("no cards" in t for t, _ in rows))


class LayoutTest(unittest.TestCase):
    def test_wide_hstack_gutter_present(self):
        rows = render_screen(_fresh(), 100.2, 160, 60)
        gutter_row = next(t for t, _ in rows if "DRONE" in t and "VIDEO OUT" in t)
        self.assertIn(" │  ", gutter_row)

    def test_stacked_sequential_at_120(self):
        rows = render_screen(_fresh(), 100.2, 120, 60)
        txt = texts(rows)
        self.assertFalse(any("DRONE" in t and "VIDEO OUT" in t for t in txt))
        idx_drone = next(i for i, t in enumerate(txt) if "DRONE" in t)
        idx_video = next(i for i, t in enumerate(txt) if "VIDEO OUT" in t)
        idx_links = next(i for i, t in enumerate(txt) if "LINKS" in t)
        idx_gs = next(i for i, t in enumerate(txt) if "GS RADIOS" in t)
        self.assertTrue(idx_drone < idx_video < idx_links < idx_gs)

    def test_compact_renderer_at_w90(self):
        rows = render_screen(_fresh(), 100.2, 90, 60)
        txt = texts(rows)
        self.assertTrue(any("mcs5" in t and "fps" in t for t in txt))
        # compact rows carry no styling spans
        self.assertTrue(all(s == [] for _, s in rows))

    def test_one_line_fallback_at_w50(self):
        m = _fresh(wall=100.0)
        rows = render_screen(m, 101.5, 50, 60)
        self.assertEqual(len(rows), 1)
        self.assertNotIn("STALE", rows[0][0])
        rows = render_screen(m, 103.5, 50, 60)
        self.assertEqual(len(rows), 1)
        self.assertIn("STALE", rows[0][0])


class RenderRowsCompactTest(unittest.TestCase):
    def test_narrow_terminal_single_line(self):
        rows = render_rows_compact(_fresh(), wall=100.2, width=40)
        self.assertEqual(len(rows), 1)
        for cell in ("session", "mcs5", "59.9", "9.31"):
            self.assertIn(cell, rows[0])

    def test_w90_below_grid_width_uses_single_line(self):
        # Documents why render_screen's w=90 and w=50 cases exercise the
        # same code path: both are below GRID_WIDTH.
        self.assertLess(90, GRID_WIDTH)
        rows = render_rows_compact(_fresh(), wall=100.2, width=90)
        self.assertEqual(len(rows), 1)


class HstackTest(unittest.TestCase):
    def test_pads_and_offsets_right_spans(self):
        left = [("abc", [(0, 1, "good")]), ("de", [])]
        right = [("XY", [(1, 1, "bad")])]
        out = hstack(left, right, gutter=" | ")
        self.assertEqual(len(out), 2)
        text0, spans0 = out[0]
        self.assertEqual(text0, "abc | XY")
        self.assertIn((0, 1, "good"), spans0)
        # right span offset by len("abc") + len(" | ") = 7
        self.assertIn((7, 1, "bad"), spans0)
        text1, spans1 = out[1]
        # left row padded to width 3, right side padded to width 2 (blank)
        self.assertEqual(text1, "de  |   ")
        self.assertEqual(spans1, [])


class FixedWidthTest(unittest.TestCase):
    def test_wide_panels_never_shift_on_extreme_values(self):
        rows_a = render_screen(_fresh(), 100.2, 160, 60)
        extreme = dict(DGRAM)
        extreme["drone"] = dict(
            DGRAM["drone"],
            gen=4294967295,
            tlm_age_ms=987654321,
            applied=dict(DGRAM["drone"]["applied"], mcs=999999999, bw=888888888),
            enc=dict(DGRAM["drone"]["enc"], fps=999999999.99, mbps=999999999.99),
            txq=dict(DGRAM["drone"]["txq"], depth=999999999, cap=888888888,
                     drops=999999999),
            radio=dict(DGRAM["drone"]["radio"], sent_pps=999999999.0,
                       drops=999999999, usb_fail=99999),
            rcf={"age_ms": 123456789, "rx_pps": 99999.9},
            sys={"soc_temp_c": -128, "thermal_delta": 99, "load": 9999.99},
        )
        extreme["cards"] = [
            dict(DGRAM["cards"][0], frames=999999999, pps=99999, rx_mbps=999.9,
                 last_frame_age_ms=123456789, crc_fail=999999, tx_fail=999999),
            DGRAM["cards"][1],
        ]
        rows_b = render_screen(_fresh(extreme), 100.2, 160, 60)
        self.assertEqual([len(t) for t, _ in rows_a], [len(t) for t, _ in rows_b])

    def test_gs_radios_survives_huge_last_frame_age(self):
        rows_a = panel_gs_radios(_fresh(), 100.2)
        huge = dict(DGRAM)
        huge["cards"] = [dict(DGRAM["cards"][0], last_frame_age_ms=123456789),
                         DGRAM["cards"][1]]
        rows_b = panel_gs_radios(_fresh(huge), 100.2)
        self.assertEqual([len(t) for t, _ in rows_a], [len(t) for t, _ in rows_b])


class NullRenderTest(unittest.TestCase):
    def test_gs_radios_null_renders_dashes(self):
        d = dict(DGRAM)
        d["cards"] = [dict(DGRAM["cards"][0], loss_pct=None, foreign_pps=None)]
        rows = panel_gs_radios(_fresh(d), 100.2)
        text = next(t for t, _ in rows if t.lstrip().startswith("c0"))
        self.assertIn("--", text)

    def test_drone_null_rates_render_dashes(self):
        d = dict(DGRAM)
        d["drone"] = dict(DGRAM["drone"],
                           enc={"fps": None, "mbps": None, "cmd_kbps": 9000,
                                "qp": 31, "roi_qp": -24, "ring_drops": 0},
                           radio={"sent_pps": None, "drops": 0, "usb_fail": 0},
                           rcf={"age_ms": 45, "rx_pps": None})
        rows = panel_drone(_fresh(d), 100.2)
        joined = "\n".join(texts(rows))
        self.assertIn("--", joined)


class ModelInvariantsTest(unittest.TestCase):
    def test_update_ignores_non_dict_input(self):
        m = _fresh(wall=100.0)
        before = (m.d, m.session, m.last_rx_wall, m.restarts,
                  dict(m.strm_rows), dict(m.sig_rows), m.bad_version)
        for bad in (None, 42, [1, 2, 3], "oops", 3.14):
            m.update(bad, 999.0)
        after = (m.d, m.session, m.last_rx_wall, m.restarts,
                 dict(m.strm_rows), dict(m.sig_rows), m.bad_version)
        self.assertEqual(before, after)

    def test_card_missing_id_does_not_poison_sig_rows(self):
        d = dict(DGRAM)
        d["cards"] = list(DGRAM["cards"]) + [
            {"up": True, "frames": 1, "crc_fail": 0, "loss_pct": 0.0,
             "rx_mbps": 0.0, "pps": 0,
             "classes": {"s0": {"pps": 1.0, "rssi": -40.0}}},
        ]
        m = _fresh(d)
        self.assertNotIn(None, [cid for cid, _cls in m.sig_rows])
        rows = render_screen(m, 100.2, 160, 60)
        self.assertTrue(len(rows) > 0)



class VanishDisplayTest(unittest.TestCase):
    # venc-ring vanish counters (docs/venc-ring-vanish-findings-2026-08-12.md):
    # drone.enc.{vanished_base,vanished_enh,self_idr_refused}, zeroed at first
    # link-establish — nonzero base = silent decoder corruption, must be visible.
    def test_panel_drone_shows_vanish_counters(self):
        import copy
        d = copy.deepcopy(DGRAM)
        d["drone"]["enc"]["vanished_base"] = 2
        d["drone"]["enc"]["vanished_enh"] = 5
        d["drone"]["enc"]["self_idr_refused"] = 1
        m = _fresh(d)
        text = "\n".join(r[0] if isinstance(r, tuple) else r
                          for r in panel_drone(m, 100.2))
        self.assertRegex(text, r"van\s+2/\s*5")
        self.assertRegex(text, r"ref\s+1")

    def test_compact_enc_row_shows_vanish_counters(self):
        import copy
        d = copy.deepcopy(DGRAM)
        d["drone"]["enc"]["vanished_base"] = 3
        d["drone"]["enc"]["vanished_enh"] = 7
        m = _fresh(d)
        text = "\n".join(r[0] if isinstance(r, tuple) else r
                          for r in render_rows_compact(m, 100.2, 200))
        self.assertRegex(text, r"van\s+3/\s*7")

    def test_panel_and_compact_show_venc_ring_stats(self):
        # Producer-side venc ring (spec 2026-08-28 venc-foldin): fill % and
        # lifetime full-drops, the only view of an encoder outrunning maburd.
        import copy
        d = copy.deepcopy(DGRAM)
        d["drone"]["enc"]["venc_ring_fill_pct"] = 62
        d["drone"]["enc"]["venc_full_drops"] = 4
        m = _fresh(d)
        panel = "\n".join(r[0] if isinstance(r, tuple) else r
                           for r in panel_drone(m, 100.2))
        self.assertRegex(panel, r"vring\s+62%")
        self.assertRegex(panel, r"drop\s+4")
        compact = "\n".join(r[0] if isinstance(r, tuple) else r
                             for r in render_rows_compact(m, 100.2, 240))
        self.assertRegex(compact, r"vring\s+62%")

    def test_vanish_keys_absent_renders_dashes(self):
        # Old-daemon datagram (pre-detection): keys missing entirely — panel
        # must render placeholders, never crash.
        m = _fresh(DGRAM)  # fixture has no vanished_* keys
        text = "\n".join(r[0] if isinstance(r, tuple) else r
                          for r in panel_drone(m, 100.2))
        self.assertRegex(text, r"van\s+--/\s*--")



if __name__ == "__main__":
    unittest.main()
