"""Mabur-owned TX-power model (offset semantics), 2026-07-17.
DIVERGES from devourer tools/precoder/energy_model.py (frozen prototype):
power is a signed qdB offset from the calibrated baseline, gain is linear
0.25 dB/qdB (bench-validated slope, docs/txagc-calibration.md), and PA draw
maps offset -> effective index over the prototype's kPaW curve.

resolve()/path_loss()/Controller below mirror devourer's op_table.py /
controller.py structure exactly, swapping ONLY the power axis (TXAGC index
-> signed qdB offset, rail [min_offset_qdb, max_offset_qdb] with max ZERO by
construction) so the golden vectors stay apples-to-apples with gs/src's C++
port. Non-power dimensions (snr_req, p_deliver, rows, airtime, e_bit) still
ride the frozen devourer link_model/op_table/energy_model imports."""

import math


def gain_db(offset_qdb):
    return 0.25 * offset_qdb


def min_offset_qdb_for_gain(need_db):
    return int(math.ceil(need_db * 4.0))


def pa_index(offset_qdb, base_ref_idx=53):
    return max(0, min(63, base_ref_idx + offset_qdb))


# Mabur-owned DEFAULT_PROFILE_TABLE pwr_offset_qdb column — DIVERGES from
# devourer rc_proto.DEFAULT_PROFILE_TABLE's pwr_idx (TXAGC 63/48/32/20/8)
# since 2026-07-17: bench-tunable qdB offsets, 0 = full legal power (max
# legal offset is ZERO; wall-equalized diffs already park every rate at
# wall - margin at offset 0). Mirrors common/src/profile.cpp verbatim.
PROFILE_TABLE_PWR_OFFSET_QDB = [0, -4, -8, -12, -16]


def resolve(row, path_loss_db, link, payload_bytes, src_bitrate_bps,
           margin_db, min_offset_qdb, max_offset_qdb, base_ref_idx,
           energy_model):
    """Mirrors gs/src/op_table.cpp resolve(): pick the smallest qdB offset
    that supplies row.snr_req + margin at this path loss, clamped to
    [min_offset_qdb, max_offset_qdb]; infeasible (None) when the required
    offset exceeds max_offset_qdb."""
    need_gain = (row.snr_req + margin_db) - path_loss_db
    off = min_offset_qdb_for_gain(need_gain)
    if off > max_offset_qdb:
        return None
    off = max(off, min_offset_qdb)
    recv = path_loss_db + gain_db(off)
    pdel = link.p_deliver(recv - energy_model.bw_noise_db(row.bw), row.mcs,
                          row.overhead, sbi=True)
    pt = energy_model.TxPoint(row.mode, row.mcs, row.bw, row.sgi,
                              pa_index(off, base_ref_idx))
    cal = energy_model.load_calibration()
    eb = energy_model.energy_per_delivered_bit(pt, src_bitrate_bps,
                                               row.overhead, payload_bytes,
                                               pdel, cal)
    import op_table as _op_table
    return _op_table.OpPoint(row.mode, row.mcs, row.bw, row.sgi, off,
                             row.overhead, row.snr_req, eb, pdel)


def max_range(max_offset_qdb):
    import op_table as _op_table
    return _op_table.OpPoint("ht", 0, 20, False, max_offset_qdb, 1.00, 0.0,
                             float("inf"), 0.0)


class ControllerConfig:
    def __init__(self, target=0.99, payload_bytes=1024, src_bitrate_bps=4e6,
                mcs_set=tuple(range(8)),
                overhead_set=(0.10, 0.25, 0.50, 0.75, 1.00), bw=20,
                bw_set=None, mode="ht", ema_alpha=0.3, ema_alpha_down=0.8,
                margin_db=2.0, min_between_changes_ms=150,
                hold_after_downgrade_ms=4000, improve_frac=0.03,
                feedback_timeout_ms=1000, allow_shed=False,
                rung_block_delta=0.15, rung_block_hold_ms=5000,
                rung_min_samples=8, min_offset_qdb=-40, max_offset_qdb=0,
                base_ref_idx=53):
        self.target = target
        self.payload_bytes = payload_bytes
        self.src_bitrate_bps = src_bitrate_bps
        self.mcs_set = mcs_set
        self.overhead_set = overhead_set
        self.bw = bw
        self.bw_set = bw_set
        self.mode = mode
        self.ema_alpha = ema_alpha
        self.ema_alpha_down = ema_alpha_down
        self.margin_db = margin_db
        self.min_between_changes_ms = min_between_changes_ms
        self.hold_after_downgrade_ms = hold_after_downgrade_ms
        self.improve_frac = improve_frac
        self.feedback_timeout_ms = feedback_timeout_ms
        self.allow_shed = allow_shed
        self.rung_block_delta = rung_block_delta
        self.rung_block_hold_ms = rung_block_hold_ms
        self.rung_min_samples = rung_min_samples
        self.min_offset_qdb = min_offset_qdb
        self.max_offset_qdb = max_offset_qdb
        self.base_ref_idx = base_ref_idx


class Controller:
    """Mirrors devourer controller.py's Controller, swapping the power axis
    (TXAGC index -> qdB offset via resolve()/path_loss() above) — same
    hysteresis/shed/failsafe state machine, unmodified."""

    def __init__(self, link, energy_model, cfg=None):
        import op_table as _op_table
        self.link = link
        self.energy_model = energy_model
        self.cfg = cfg or ControllerConfig()
        self.rows = _op_table.build_link_rows(
            link, self.cfg.target, self.cfg.mcs_set, self.cfg.overhead_set,
            self.cfg.bw, sbi=True, bw_set=self.cfg.bw_set, mode=self.cfg.mode)
        self.snr_ema = None
        self.cur = None
        self.shed = False
        self._last_change_ms = -1 << 60
        self._last_downgrade_ms = -1 << 60
        self._last_feedback_ms = -1 << 60
        self._rung_block = {}
        self._now_ms = -1 << 60
        self.primary_dirty = False

    def _path_loss(self, reported_snr, reported_offset_qdb):
        return reported_snr - gain_db(reported_offset_qdb)

    def _rung_blocked(self, bw):
        until = self._rung_block.get(bw)
        return until is not None and self._now_ms < until

    def report_rung_delivery(self, stats, now_ms):
        usable = {bw: d for bw, (d, n) in stats.items()
                 if n >= self.cfg.rung_min_samples}
        if not usable:
            return
        best = max(usable.values())
        narrowest = min(usable)
        for bw, d in usable.items():
            if bw != narrowest and d < best - self.cfg.rung_block_delta:
                self._rung_block[bw] = now_ms + self.cfg.rung_block_hold_ms
        self.primary_dirty = (len(usable) >= 2 and
                              all(d < self.cfg.target - self.cfg.rung_block_delta
                                  for d in usable.values()))

    def _best(self, path_loss, margin):
        best = None
        for r in self.rows:
            if self._rung_blocked(r.bw):
                continue
            op = resolve(r, path_loss, self.link, self.cfg.payload_bytes,
                        self.cfg.src_bitrate_bps, margin,
                        self.cfg.min_offset_qdb, self.cfg.max_offset_qdb,
                        self.cfg.base_ref_idx, self.energy_model)
            if op is None or op.p_deliver < self.cfg.target or op.e_bit == float("inf"):
                continue
            if best is None or op.e_bit < best.e_bit:
                best = op
        return best

    def update(self, reported_snr, reported_offset_qdb, now_ms):
        self._last_feedback_ms = now_ms
        self._now_ms = now_ms
        pl_inst = self._path_loss(reported_snr, reported_offset_qdb)
        if self.snr_ema is None:
            self.snr_ema = pl_inst
        else:
            a = (self.cfg.ema_alpha_down if pl_inst < self.snr_ema
                else self.cfg.ema_alpha)
            self.snr_ema = (1 - a) * self.snr_ema + a * pl_inst
        return self._decide(self.snr_ema, now_ms)

    def _decide(self, path_loss, now_ms):
        import op_table as _op_table
        cur_ok = False
        if self.cur is not None and not self.shed:
            cur_now = resolve(
                _op_table.LinkRow(self.cur.mode, self.cur.mcs, self.cur.bw,
                                  self.cur.sgi, self.cur.overhead,
                                  self.cur.snr_req),
                path_loss, self.link, self.cfg.payload_bytes,
                self.cfg.src_bitrate_bps, 0.0, self.cfg.min_offset_qdb,
                self.cfg.max_offset_qdb, self.cfg.base_ref_idx,
                self.energy_model)
            cur_ok = (cur_now is not None and cur_now.p_deliver >= self.cfg.target
                     and not self._rung_blocked(self.cur.bw))
            if cur_ok:
                self.cur = cur_now

        cand = self._best(path_loss, self.cfg.margin_db)

        if cand is None:
            if cur_ok:
                return self.cur
            if self.cfg.allow_shed:
                self.shed = True
                self.cur = None
                return None
            self.cur = max_range(self.cfg.max_offset_qdb)
            return self.cur

        self.shed = False
        if self.cur is None or self.shed:
            return self._commit(cand, now_ms)

        if (now_ms - self._last_change_ms < self.cfg.min_between_changes_ms
                and cur_ok):
            return self.cur
        is_upgrade = cand.e_bit < self.cur.e_bit
        if is_upgrade:
            if cur_ok and cand.e_bit > self.cur.e_bit * (1 - self.cfg.improve_frac):
                return self.cur
            if (cur_ok and now_ms - self._last_downgrade_ms <
                    self.cfg.hold_after_downgrade_ms):
                return self.cur
            return self._commit(cand, now_ms)
        if not cur_ok:
            self._last_downgrade_ms = now_ms
            return self._commit(cand, now_ms)
        return self.cur

    def on_tick(self, now_ms):
        if now_ms - self._last_feedback_ms > self.cfg.feedback_timeout_ms:
            self.shed = False
            self.cur = max_range(self.cfg.max_offset_qdb)
        return self.cur

    def _commit(self, op, now_ms):
        self.cur = op
        self._last_change_ms = now_ms
        return op
