#!/usr/bin/env python3
"""maburtop — fullscreen console for the maburgs stats sideport.

Binds the sideport UDP port and renders the JSON feed as a fixed grid that
refreshes in place (spec: docs/superpowers/specs/2026-07-25-gs-stats-sideport-design.md).
Usage: maburtop.py [--port 8300] [--bind 0.0.0.0] [--interval 1.0]
"""
import argparse
import curses
import json
import socket
import time

STALE_S = 2.0

# One spec per grid: (title, width). Header titles and data cells are both
# rendered from these, right-aligned into the same slots — they cannot
# misalign. Label column (CARD / "  c0") is LABEL_W wide, cells are joined
# with a single space.
LABEL_W = 6
CARD_COLS = [("st", 4), ("pps", 5), ("inj", 5), ("Mbps", 5), ("loss%", 5),
             ("crc", 5), ("age", 6), ("forgn", 6), ("self", 6), ("tx", 4),
             ("txf", 4)]
# LNK blocks: one block per link type (class), a decode line for the FEC
# streams, then per-card signal rows sharing these columns across all blocks
# (their titles live on the LNK rule line).
LNKSIG_COLS = [("card", 4), ("pps", 5), ("kbps", 6), ("rssi", 6), ("rssiA", 6),
               ("rssiB", 6), ("snr", 5), ("snrA", 5), ("snrB", 5)]

# Sticky class rows render in this fixed order regardless of dict/arrival
# order; "ctrl" gets the short display label "ctl" (cls column is 4 wide).
CLASS_ORDER = ["s0", "s1", "s2", "s3", "msp", "ctrl"]
CLASS_LABELS = {"ctrl": "ctl"}


def _grid_row(label, cells):
    """label padded/truncated to LABEL_W, then one space before each
    fixed-width cell. Both header and data rows come through here."""
    return label[:LABEL_W].ljust(LABEL_W) + "".join(" " + c for c in cells)


def _grid_width(cols):
    return LABEL_W + sum(w + 1 for _, w in cols)


def _rung_cell(strm, w=7):
    """TX rung the drone airs this stream at, e.g. 'mcs5+LS' (L=LDPC,
    S=STBC). Derived GS-side from the commanded op; '--' pre-schema."""
    m = strm.get("rung_mcs")
    if m is None:
        return "--".ljust(w)
    flags = ("L" if strm.get("rung_ldpc") else "") + \
            ("S" if strm.get("rung_stbc") else "")
    s = f"mcs{m}" + (f"+{flags}" if flags else "")
    return s[:w].ljust(w) if len(s) > w else s.ljust(w)


def _dec_line(label, strm, dlv):
    """Per-stream decode line: TX config (rung, PHY rate, injection
    estimate) then RX decode health. Inline-labeled, fixed cell widths so
    the s0..s3 lines align vertically. strm = the sticky link.streams row."""
    inj_kbps = strm.get("inj_kbps")
    inj_m = None if inj_kbps is None else inj_kbps / 1000.0
    return (
        f"{label:<{LABEL_W}} {_rung_cell(strm)}"
        f" {_f(strm.get('phy_mbps'), 5, 1)}M"
        f" inj {_f(inj_m, 5, 1)}M"
        f" ov {_f(strm.get('ov'), 4, 2)}"
        f" dlv {_f(dlv, 3, 0)}%"
        f" rec/s {_f(strm.get('recovered_s'), 6, 1)}"
        f" abn/s {_f(strm.get('abandoned_s'), 5, 1)}"
        f" in/s {_f(strm.get('syms_in_s'), 6, 0)}"
        f" sf {_f(strm.get('sub_fail'), 2)}"
        f" fl {_f(strm.get('in_flight'), 2)}"
    )


def _f(v, w, prec=1):
    """Right-align value in w chars; None -> '--'. Never widens the cell."""
    if v is None:
        return "--".rjust(w)
    if isinstance(v, float):
        s = f"{v:.{prec}f}"
    else:
        s = str(v)
    return s[:w].rjust(w) if len(s) > w else s.rjust(w)


def _age_cell(age_ms, w=6):
    """Fixed-width 'time since last frame' cell: None -> '--'. Auto-scales
    ms -> s -> m so a card silent for minutes still reads as a sane number
    instead of overflowing the column (and, as a last resort, truncates —
    same never-widen contract as _f())."""
    if age_ms is None:
        return "--".rjust(w)
    if age_ms < 10_000:
        s = f"{age_ms}ms"
    elif age_ms < 1_000_000:
        s = f"{age_ms // 1000}s"
    else:
        s = f"{age_ms // 60_000}m"
    return s[:w].rjust(w) if len(s) > w else s.rjust(w)


GRID_WIDTH = max(_grid_width(CARD_COLS), _grid_width(LNKSIG_COLS),
                 len(_dec_line("s0", {}, None)))  # widest grid row


def _applied_mcsbw_cell(mcs, bw, w=7):
    """'mcs5/20'-style composite cell, fixed-width like _rung_cell: compose
    then truncate/pad so an untrusted/absurd mcs or bw off the wire can't
    widen the DRONE row. w=7 fits today's real values (1-digit mcs,
    2-digit bw) exactly, matching the mockup with no extra padding."""
    mcs_s = "--" if mcs is None else str(mcs)
    bw_s = "--" if bw is None else str(bw)
    s = f"mcs{mcs_s}/{bw_s}"
    return s[:w].ljust(w) if len(s) > w else s.ljust(w)


def _drone_row(drone):
    """DRONE row: link state, generation, applied op (vs the header's
    commanded op two lines up — a mismatch should be visually obvious),
    RCF freshness, telemetry age. Inline-labeled like the decode lines;
    every variable-length field goes through _f/_age_cell/
    _applied_mcsbw_cell so extreme values (u32 generation, saturating
    ages, an absurd mcs/bw off the wire) truncate instead of shifting the
    row."""
    state = drone.get("state")
    state_s = state.upper() if isinstance(state, str) else None
    applied = drone.get("applied") or {}
    rcf = drone.get("rcf") or {}
    return (
        f"DRONE   {_f(state_s, 8)}  gen {_f(drone.get('gen'), 6)}   "
        f"applied {_applied_mcsbw_cell(applied.get('mcs'), applied.get('bw'))}"
        f" ov {_f(applied.get('overhead'), 4, 2)}"
        f" off {_f(applied.get('offset_qdb'), 3)}"
        f" der {_f(applied.get('derate_qdb'), 3)}  "
        f"rcf age {_age_cell(rcf.get('age_ms'))}  "
        f"tlm {_age_cell(drone.get('tlm_age_ms'))}"
    )


def _enc_row(enc):
    """ENC row: encoder-side counters/rates (fps, mbps) that the GS can
    cross-check against Σ stream inj_kbps on the s0..s3 decode lines to
    localize pipeline loss (waybeam ring-full aborts etc)."""
    return (
        f"ENC     {_f(enc.get('fps'), 5, 1)} fps   "
        f"{_f(enc.get('mbps'), 5, 2)} Mbps   "
        f"cmd {_f(enc.get('cmd_kbps'), 5)}k   "
        f"qp {_f(enc.get('qp'), 2)}   "
        f"ring {_f(enc.get('ring_drops'), 5)}"
    )


def _txq_row(txq, radio):
    """TXQ row: on-drone queue depth/drops plus RadioTx counters (sent_pps
    cross-checks against CARD inj_pps for injection-estimator calibration).
    usb_fail/drops are cumulative wire counters, not rates."""
    return (
        f"TXQ     depth {_f(txq.get('depth'), 3)}/{_f(txq.get('cap'), 3)}   "
        f"sent {_f(radio.get('sent_pps'), 6, 0)} pps   "
        f"drop {_f(txq.get('drops'), 5)}   "
        f"usb fail {_f(radio.get('usb_fail'), 5)}"
    )


def _uplink_row(uplink, rcf):
    """UPLNK row: the only place the drone's view of the GS is visible —
    downlink RF is on every LNK row, this is the uplink margin."""
    return (
        f"UPLNK   rssi {_f(uplink.get('rssi_a'), 6, 1)} {_f(uplink.get('rssi_b'), 6, 1)}   "
        f"snr {_f(uplink.get('snr_a'), 5, 1)} {_f(uplink.get('snr_b'), 5, 1)}   "
        f"rcf rx {_f(rcf.get('rx_pps'), 5, 1)}/s"
    )


def _sys_row(sys_d, radio_rx_ok):
    """SYS row: SoC/radio thermal state, loadavg, and the radio-RX wedge
    flag (the 'comes up deaf after restart' condition made visible).
    soc_temp_c == -128 is the wire sentinel for 'unavailable'."""
    soc = sys_d.get("soc_temp_c")
    if soc is not None and soc <= -128:
        soc = None
    if radio_rx_ok is None:
        rx_s = None
    else:
        rx_s = "ok" if radio_rx_ok else "DEAF"
    return (
        f"SYS     soc {_f(soc, 3)}C   "
        f"rf delta {_f(sys_d.get('thermal_delta'), 3)}   "
        f"load {_f(sys_d.get('load'), 4, 2)}   "
        f"radio rx {_f(rx_s, 4)}"
    )


class Model:
    """Latest datagram + feed bookkeeping. update() is pure bookkeeping;
    all layout lives in render_rows()."""

    def __init__(self):
        self.d = None            # last datagram (dict)
        self.last_rx_wall = None
        self.session = None
        self.restarts = 0
        self.rx_times = []       # wall clocks of last ~10 datagrams -> rx Hz
        self.sig_rows = {}       # (card_id, class_key) -> last class dict (sticky)
        self.strm_rows = {}      # stream id -> last row dict (sticky)
        self.bad_version = None

    def update(self, dgram, wall):
        if not isinstance(dgram, dict):
            return  # malformed/non-object datagram: keep last good state
        v = dgram.get("v")
        if v != 1:
            self.bad_version = v
            self.last_rx_wall = wall
            return
        self.bad_version = None
        if self.session is not None and dgram.get("session") != self.session:
            self.restarts += 1
        self.session = dgram.get("session")
        self.d = dgram
        self.last_rx_wall = wall
        self.rx_times = (self.rx_times + [wall])[-10:]
        for card in dgram.get("cards", []):
            cid = card.get("id")
            if cid is None:
                continue  # unindexable card: don't poison sig_rows' sort key
            for cls_key, cls_val in (card.get("classes") or {}).items():
                self.sig_rows[(cid, cls_key)] = cls_val
        link = dgram.get("link") or {}
        for row in link.get("streams", []):
            self.strm_rows[row["stream"]] = row


def _s(v, prec=None):
    """Loose, non-fixed-width scalar formatter for prose (header/fallback)
    cells: None -> '--'."""
    if v is None:
        return "--"
    if isinstance(v, float) and prec is not None:
        return f"{v:.{prec}f}"
    return str(v)


def render_rows(model, wall, width):
    d = model.d or {}
    link = d.get("link") or {}
    op = link.get("op") or {}
    cards = d.get("cards") or []
    video = link.get("video") or {}
    rtp = video.get("rtp") or {}
    udpstats = video.get("udp") or {}

    state = link.get("state")
    mcs = op.get("mcs")
    fps = video.get("fps")
    mbps = video.get("mbps")
    loss = cards[0].get("loss_pct") if cards else None

    if width < GRID_WIDTH:
        stale = model.last_rx_wall is not None and (wall - model.last_rx_wall) > STALE_S
        prefix = "STALE " if stale else ""
        line = (
            f"{prefix}{_s(state)} mcs{_s(mcs)} {_s(fps, 1)} fps "
            f"{_s(mbps, 2)} Mbps loss {_s(loss, 1)}%"
        )
        return [line]

    rows = []

    # --- header / STALE banner ---
    stale = model.last_rx_wall is not None and (wall - model.last_rx_wall) > STALE_S
    if stale:
        age = wall - model.last_rx_wall
        header = f"STALE — last seen {age:.1f} s ago".ljust(width)
    else:
        tx_card = link.get("tx_card")
        vtx_id = link.get("vtx_id")
        bw = op.get("bw")
        overhead = op.get("overhead")
        offset_qdb = op.get("offset_qdb")
        deadline_ms = link.get("deadline_ms")
        state_s = state.upper() if isinstance(state, str) else "--"
        header = (
            f"maburgs   {state_s}   vtx {_s(vtx_id)}   tx c{_s(tx_card)}   "
            f"MCS {_s(mcs)}/{_s(bw)}   ov {_s(overhead, 2)}   "
            f"off {_s(offset_qdb)} qdB   deadline {_s(deadline_ms)} ms"
        ).ljust(width)
    rows.append(header)

    if model.bad_version is not None:
        rows.append(f"unsupported schema v={model.bad_version}".ljust(width))
    else:
        rows.append(("─" * width)[:width])

    # --- CARD --- physical radio totals (canonical traffic only)
    rows.append(_grid_row("CARD", [t.rjust(w) for t, w in CARD_COLS]))
    if not cards:
        rows.append(_grid_row("  --", ["no cards".ljust(GRID_WIDTH - LABEL_W - 1)]))
    else:
        for c in cards:
            up = c.get("up")
            st_s = "UP" if up else ("DOWN" if up is not None else None)
            cells = [
                _f(st_s, CARD_COLS[0][1]),
                _f(c.get("pps"), CARD_COLS[1][1], 0),
                _f(c.get("inj_pps"), CARD_COLS[2][1], 0),
                _f(c.get("rx_mbps"), CARD_COLS[3][1], 1),
                _f(c.get("loss_pct"), CARD_COLS[4][1], 1),
                _f(c.get("crc_fail"), CARD_COLS[5][1]),
                _age_cell(c.get("last_frame_age_ms"), CARD_COLS[6][1]),
                _f(c.get("foreign_pps"), CARD_COLS[7][1], 1),
                _f(c.get("self_pps"), CARD_COLS[8][1], 1),
                _f(c.get("tx_pps"), CARD_COLS[9][1], 0),
                _f(c.get("tx_fail"), CARD_COLS[10][1]),
            ]
            rows.append(_grid_row(f"  c{_s(c.get('id'))}", cells))

    # --- LNK blocks: one per link type, decode line + per-card signal rows.
    # Signal columns are shared across every block; their titles ride the
    # LNK rule line so the alignment contract holds grid-wide.
    hdr = _grid_row("LNK ──", [t.rjust(w) for t, w in LNKSIG_COLS])
    rows.append(hdr + " " + ("─" * max(0, width - len(hdr) - 1)))
    seen_classes = {cls for _cid, cls in model.sig_rows}
    seen_classes |= {f"s{sid}" for sid in model.strm_rows}
    if not seen_classes:
        rows.append(_grid_row("  --", ["no link data".ljust(GRID_WIDTH - LABEL_W - 1)]))
    layers = link.get("layer_delivery_pct") or []
    for cls in CLASS_ORDER:
        if cls not in seen_classes:
            continue
        label = CLASS_LABELS.get(cls, cls)
        sid = int(cls[1]) if cls.startswith("s") and cls[1:].isdigit() else None
        if sid is not None and sid in model.strm_rows:
            dlv = layers[sid] if sid < len(layers) else None
            rows.append(_dec_line(label, model.strm_rows[sid], dlv))
        elif cls == "msp":
            rows.append(f"{label:<{LABEL_W}} (osd side-channel — no fec decode)")
        elif cls == "ctrl":
            rows.append(f"{label:<{LABEL_W}} (control — tx at rendezvous only)")
        else:
            rows.append(f"{label:<{LABEL_W}}")
        for cid in sorted({c for c, k in model.sig_rows if k == cls}):
            s = model.sig_rows[(cid, cls)]
            mbps_c = s.get("mbps")
            kbps = None if mbps_c is None else mbps_c * 1000.0
            cells = [
                _f(f"c{_s(cid)}", LNKSIG_COLS[0][1]),
                _f(s.get("pps"), LNKSIG_COLS[1][1], 0),
                _f(kbps, LNKSIG_COLS[2][1], 0),
                _f(s.get("rssi"), LNKSIG_COLS[3][1], 1),
                _f(s.get("rssi_a"), LNKSIG_COLS[4][1], 1),
                _f(s.get("rssi_b"), LNKSIG_COLS[5][1], 1),
                _f(s.get("snr"), LNKSIG_COLS[6][1], 1),
                _f(s.get("snr_a"), LNKSIG_COLS[7][1], 1),
                _f(s.get("snr_b"), LNKSIG_COLS[8][1], 1),
            ]
            rows.append(_grid_row("", cells))

    # --- DRONE telemetry region: absent-safe (null until the first T_TELEM
    # frame of the session / old maburd / old peer_caps).
    drone = d.get("drone")
    if drone is None:
        rows.append("DRONE   no telemetry (old maburd / peer caps)")
    else:
        rows.append(_drone_row(drone))
        rows.append(_enc_row(drone.get("enc") or {}))
        rows.append(_txq_row(drone.get("txq") or {}, drone.get("radio") or {}))
        rows.append(_uplink_row(drone.get("uplink") or {}, drone.get("rcf") or {}))
        rows.append(_sys_row(drone.get("sys") or {}, drone.get("radio_rx_ok")))

    # --- link-wide residual (per-stream delivery now lives on the dec lines)
    residual = link.get("residual_loss")
    residual_pct = None if residual is None else residual * 100.0
    air = link.get("air_pct")
    rows.append(f"LINK    residual {_f(residual_pct, 5, 1)} %"
                f"   air ~{_f(air, 4, 1)}% of ladder")

    # --- VIDEO ---
    rows.append(
        f"VIDEO  {_f(fps, 6, 1)} fps  {_f(mbps, 6, 2)} Mbps   "
        f"jit {_f(video.get('jitter_ms'), 5, 1)} ms   "
        f"clean {_f(video.get('clean'), 7)}   "
        f"trunc {_f(video.get('truncated'), 4)}   "
        f"drop {_f(video.get('dropped'), 4)}"
    )

    # --- RTP ---
    rows.append(
        f"RTP     ok {_f(rtp.get('ok'), 8)}   "
        f"gap {_f(rtp.get('gap'), 3)} (+{_f(rtp.get('gap_seqs'), 2)})   "
        f"back {_f(rtp.get('back'), 3)}   "
        f"udp fail {_f(udpstats.get('failed'), 3)}   "
        f"q_drop {_f(video.get('q_drop'), 3)}"
    )

    # --- footer (rx rate / staleness / session / restarts) ---
    rx_times = model.rx_times
    if len(rx_times) >= 2:
        dt = rx_times[-1] - rx_times[0]
        hz = (len(rx_times) - 1) / dt if dt > 0 else 0.0
    else:
        hz = 0.0
    last_age = 0.0 if model.last_rx_wall is None else max(0.0, wall - model.last_rx_wall)
    session = model.session
    session_s = "--" if session is None else f"0x{session:08x}"
    rows.append(
        f"        rx {hz:.1f} Hz   last {last_age:.1f} s   "
        f"session {session_s}   maburgs restarts {model.restarts}"
    )

    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8300)
    ap.add_argument("--bind", default="0.0.0.0")
    ap.add_argument("--interval", type=float, default=1.0)
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.bind, args.port))
    sock.settimeout(0.25)
    model = Model()

    def loop(scr):
        curses.curs_set(0)
        scr.nodelay(True)
        last_draw = 0.0
        while True:
            try:
                data, _ = sock.recvfrom(65535)
                try:
                    model.update(json.loads(data.decode()), time.time())
                except (ValueError, KeyError, TypeError, AttributeError):
                    pass  # malformed datagram: keep last good state
            except socket.timeout:
                pass
            if scr.getch() in (ord("q"), 3):
                return
            now = time.time()
            if now - last_draw >= args.interval:
                last_draw = now
                h, w = scr.getmaxyx()
                scr.erase()
                try:
                    for y, row in enumerate(render_rows(model, now, w - 1)):
                        if y >= h:
                            break
                        attr = curses.A_REVERSE if ("STALE" in row and y == 0) else 0
                        scr.addnstr(y, 0, row, w - 1, attr)
                except Exception:
                    pass
                scr.refresh()

    try:
        curses.wrapper(loop)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
