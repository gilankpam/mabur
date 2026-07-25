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

GRID_WIDTH = 86
STALE_S = 2.0


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


class Model:
    """Latest datagram + feed bookkeeping. update() is pure bookkeeping;
    all layout lives in render_rows()."""

    def __init__(self):
        self.d = None            # last datagram (dict)
        self.last_rx_wall = None
        self.session = None
        self.restarts = 0
        self.rx_times = []       # wall clocks of last ~10 datagrams -> rx Hz
        self.fec_rows = {}       # stream id -> last row dict (sticky)
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
        for row in dgram.get("fec", []):
            self.fec_rows[row["stream"]] = row


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
    video = d.get("video") or {}
    rtp = video.get("rtp") or {}
    udpstats = video.get("udp") or {}

    state = link.get("state")
    mcs = op.get("mcs")
    fps = video.get("fps")
    mbps = video.get("mbps")
    loss = cards[0].get("loss_pct") if cards else None

    if width < GRID_WIDTH:
        line = (
            f"{_s(state)} mcs{_s(mcs)} {_s(fps, 1)} fps "
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
        bw = op.get("bw")
        overhead = op.get("overhead")
        offset_qdb = op.get("offset_qdb")
        deadline_ms = link.get("deadline_ms")
        state_s = state.upper() if isinstance(state, str) else "--"
        header = (
            f"maburgs   {state_s}   tx c{_s(tx_card)}   "
            f"MCS {_s(mcs)}/{_s(bw)}   ov {_s(overhead, 2)}   "
            f"off {_s(offset_qdb)} qdB   deadline {_s(deadline_ms)} ms"
        ).ljust(width)
    rows.append(header)

    if model.bad_version is not None:
        rows.append(f"unsupported schema v={model.bad_version}".ljust(width))
    else:
        rows.append(("─" * width)[:width])

    # --- LINK ---
    residual = link.get("residual_loss")
    residual_pct = None if residual is None else residual * 100.0
    layers = link.get("layer_delivery_pct")
    layers_s = " ".join(_f(v, 3) for v in layers) if layers else "--"
    rows.append(
        f"LINK    residual {_f(residual_pct, 5, 1)} %     layers {layers_s} %"
    )

    # --- CARD ---
    rows.append(
        "CARD    st   pps   Mbps  loss%   rssi  rssiA  rssiB    snr   snrA   snrB   crc   age"
    )
    if not cards:
        rows.append("  --    no cards")
    else:
        for c in cards:
            cid = c.get("id")
            up = c.get("up")
            st_s = "UP" if up else ("DOWN" if up is not None else None)
            age_s = _age_cell(c.get("last_frame_age_ms"), 6)
            row = (
                f"  c{_s(cid)}"
                f"  {_f(st_s, 5)}"
                f" {_f(c.get('pps'), 6)}"
                f" {_f(c.get('rx_mbps'), 7, 1)}"
                f" {_f(c.get('loss_pct'), 7, 1)}"
                f" {_f(c.get('rssi'), 7, 1)}"
                f" {_f(c.get('rssi_a'), 7, 1)}"
                f" {_f(c.get('rssi_b'), 7, 1)}"
                f" {_f(c.get('snr'), 7, 1)}"
                f" {_f(c.get('snr_a'), 7, 1)}"
                f" {_f(c.get('snr_b'), 7, 1)}"
                f" {_f(c.get('crc_fail'), 6)}"
                f" {age_s}"
            )
            rows.append(row)

    # --- FEC (sticky) ---
    rows.append("FEC     str  rec/s  abn/s   in/s  sfail  flight")
    if not model.fec_rows:
        rows.append("  --    no fec streams")
    else:
        for sid in sorted(model.fec_rows):
            f = model.fec_rows[sid]
            row = (
                f"  s{_s(sid)}"
                f" {_f(f.get('recovered_s'), 9, 1)}"
                f" {_f(f.get('abandoned_s'), 7, 1)}"
                f" {_f(f.get('syms_in_s'), 7, 0)}"
                f" {_f(f.get('sub_fail'), 7)}"
                f" {_f(f.get('in_flight'), 8)}"
            )
            rows.append(row)

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
                for y, row in enumerate(render_rows(model, now, w - 1)):
                    if y >= h:
                        break
                    attr = curses.A_REVERSE if ("STALE" in row and y == 0) else 0
                    scr.addnstr(y, 0, row, w - 1, attr)
                scr.refresh()

    curses.wrapper(loop)


if __name__ == "__main__":
    main()
