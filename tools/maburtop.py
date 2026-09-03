#!/usr/bin/env python3
"""maburtop — fullscreen console for the maburgs stats sideport.

Binds the sideport UDP port and renders the JSON feed as a full-screen,
panel-based, color-aware layout grouped by link (spec:
docs/superpowers/specs/2026-07-25-gs-stats-sideport-design.md +
docs/superpowers/specs/2026-07-26-drone-telemetry-design.md).
Usage: maburtop.py [--port 8300] [--bind 0.0.0.0] [--interval 1.0]

Render architecture: every render function returns styled rows —
list[tuple[str, list[tuple[int, int, str]]]] — (text, spans), where a span
is (start, length, style_name). STYLES maps style names to curses attr
lambdas resolved at runtime; the curses loop paints text plain, then paints
each span's slice with its resolved attribute. Panels are pure functions
panel_xxx(model, wall) -> styled rows; render_screen() composes them by
terminal width. Narrow terminals (< 100 cols) fall back to the plain-text
compact renderer (render_rows_compact, ex-render_rows).
"""
import argparse
import curses
import json
import socket
import time

STALE_S = 2.0

# One spec per grid: (title, width). Header titles and data cells are both
# rendered from these, right-aligned into the same slots — they cannot
# misalign. Label column is label_w wide (default LABEL_W), cells are
# joined with a single space.
LABEL_W = 6
CARD_COLS = [("st", 4), ("pps", 5), ("inj", 5), ("Mbps", 5), ("loss%", 5),
             ("crc", 5), ("age", 6), ("forgn", 6), ("self", 6), ("tx", 4),
             ("txf", 4)]
# LNK blocks (compact renderer only): one block per link type (class), a
# decode line for the FEC streams, then per-card signal rows sharing these
# columns across all blocks (their titles live on the LNK rule line).
# evm: combined best-chain RX EVM, dB (negative = clean; devourer's provisional dirty/clean marks are -23.5/-24.5 dB). Compact mode shows only the combined value; per-chain rides the wide radio panel + sideport.
LNKSIG_COLS = [("card", 4), ("pps", 5), ("kbps", 6), ("rssi", 6), ("rssiA", 6),
               ("rssiB", 6), ("snr", 5), ("snrA", 5), ("snrB", 5), ("evm", 6)]

# Per-card radio table inside a wide-mode LINKS block: same signal columns
# as LNKSIG_COLS but without a "card" column — the card id is the row
# label instead (see mockup in the brief).
RADIO_LABEL_W = 10
RADIO_COLS = [("pps", 5), ("kbps", 6), ("rssi", 6), ("rssiA", 6), ("rssiB", 6),
              ("snr", 5), ("snrA", 5), ("snrB", 5), ("evm", 6), ("evmA", 6),
              ("evmB", 6)]

# Sticky class rows render in this fixed order regardless of dict/arrival
# order; "ctrl" gets the short display label "ctl" (cls column is 4 wide,
# compact renderer only).
# 2-stream video (2026-08-29 airtime-balance-uep): s0 = BASE (mirrors the
# ladder rung's mcs-1), s1 = ENH (canary attribution). s2/s3 are gone from
# the wire — class_seen_ never lights them up GS-side — but the string
# space stays sparse-safe: an unrecognized class key from an old recording
# would just never match here, not crash.
CLASS_ORDER = ["s0", "s1", "msp", "ctrl"]
CLASS_LABELS = {"ctrl": "ctl"}

# Wide-mode LINKS panel block labels.
LINK_CLASS_LABELS = {
    "s0": "s0 · video BASE",
    "s1": "s1 · video ENH",
    "msp": "msp · osd",
    "ctrl": "ctl · control",
}

# Style names available to spans: title bold good warn bad dim rev. Color
# pairs 1=green 2=yellow 3=red are initialized (when curses.has_colors())
# by _init_colors() at loop startup; resolution is deferred to paint time
# so importing/using this module without a curses screen (tests) is safe.
STYLES = {
    "title": lambda: curses.A_BOLD,
    "bold": lambda: curses.A_BOLD,
    "good": lambda: curses.A_BOLD | (curses.color_pair(1) if curses.has_colors() else 0),
    "warn": lambda: curses.A_BOLD | (curses.color_pair(2) if curses.has_colors() else 0),
    "bad": lambda: curses.A_BOLD | (curses.color_pair(3) if curses.has_colors() else 0),
    "dim": lambda: curses.A_DIM,
    "rev": lambda: curses.A_REVERSE,
}


def _grid_row(label, cells, label_w=LABEL_W):
    """label padded/truncated to label_w, then one space before each
    fixed-width cell. Both header and data rows come through here."""
    return label[:label_w].ljust(label_w) + "".join(" " + c for c in cells)


def _grid_width(cols):
    return LABEL_W + sum(w + 1 for _, w in cols)


def _cell_offsets(widths, label_w=LABEL_W):
    """Start column of each cell built by _grid_row(label, cells, label_w)
    — used to place styling spans without re-deriving _grid_row's layout."""
    off = label_w
    offsets = []
    for w in widths:
        off += 1
        offsets.append(off)
        off += w
    return offsets


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
    """Per-stream decode line (compact renderer): TX config (rung, PHY
    rate, injection estimate) then RX decode health. Inline-labeled, fixed
    cell widths so the s0/s1 lines align vertically. strm = the sticky
    link.streams row."""
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
                 len(_dec_line("s0", {}, None)))  # widest compact grid row


def _applied_mcsbw_cell(mcs, bw, w=7):
    """'mcs5/20'-style composite cell, fixed-width like _rung_cell: compose
    then truncate/pad so an untrusted/absurd mcs or bw off the wire can't
    widen a row. w=7 fits today's real values (1-digit mcs, 2-digit bw)
    exactly, matching the mockup with no extra padding."""
    mcs_s = "--" if mcs is None else str(mcs)
    bw_s = "--" if bw is None else str(bw)
    s = f"mcs{mcs_s}/{bw_s}"
    return s[:w].ljust(w) if len(s) > w else s.ljust(w)


def _ov_cmd_cell(cmd_base, cmd_enh, ov_base, ov_enh):
    """Prose-style (top bar / compact header) overhead cell: cmd_base/
    cmd_enh are the GS-commanded pair the ladder currently sends
    (link.op.overhead_base/overhead_enh — same-rate-fixed-pairs, Task 5);
    ov_base/ov_enh are the drone's actual applied per-stream pair from
    telemetry — '--' before the first T_TELEM snapshot. The runtime
    AirBalancer solver that used to explain a commanded-vs-applied split is
    deleted (2026-08-30 same-rate-fixed-pairs); applied now equals commanded
    except while a bench :8301 ov_base_pct/ov_enh_pct override is armed, so
    a divergence here means an armed override or a stale/old-daemon
    snapshot, not a balancer doing its job."""
    return (f"ov cmd b{_s(cmd_base, 2)}/e{_s(cmd_enh, 2)} "
            f"(b {_s(ov_base, 2)}/e {_s(ov_enh, 2)})")


def _ov_applied_cell(ov_base, ov_enh, w=4):
    """Fixed-width grid cell for the drone's actual applied per-stream
    overhead (compact DRONE row / wide 'applied' line) — no comparison
    against the commanded op scalar; divergence means an armed :8301
    override or staleness, not a balancer (see _ov_cmd_cell; the solver is
    deleted as of 2026-08-30 same-rate-fixed-pairs)."""
    return f"ov b{_f(ov_base, w, 2)}/e{_f(ov_enh, w, 2)}"


def _increased(cur, prev):
    """True when both sides of a cumulative counter are known and the
    counter grew — the "something bad just happened" signal for wire
    counters (drops, crc_fail, trunc, ...); None on either side means no
    verdict (first sample / absent field), never a false positive."""
    return cur is not None and prev is not None and cur > prev


def _s(v, prec=None):
    """Loose, non-fixed-width scalar formatter for prose (top bar/compact
    fallback) cells: None -> '--'."""
    if v is None:
        return "--"
    if isinstance(v, float) and prec is not None:
        return f"{v:.{prec}f}"
    return str(v)


class Model:
    """Latest datagram + feed bookkeeping. update() is pure bookkeeping;
    all layout lives in the panel_xxx()/render_screen()/render_rows_compact()
    functions."""

    def __init__(self):
        self.d = None            # last datagram (dict)
        self.last_rx_wall = None
        self.session = None
        self.restarts = 0
        self.rx_times = []       # wall clocks of last ~10 datagrams -> rx Hz
        self.sig_rows = {}       # (card_id, class_key) -> last class dict (sticky)
        self.class_active = {}   # class_key -> wall clock of last datagram with pps > 0
        self.strm_rows = {}      # stream id -> last row dict (sticky)
        self.bad_version = None
        # Previous datagram's counters, for increased-vs-previous styling.
        self.prev_drone = None
        self.prev_video = None
        self.prev_cards = {}     # card id -> last card dict

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
        old = self.d or {}
        self.prev_drone = old.get("drone")
        self.prev_video = (old.get("link") or {}).get("video")
        self.prev_cards = {c.get("id"): c for c in (old.get("cards") or [])
                            if c.get("id") is not None}
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
                if cls_val.get("pps"):
                    self.class_active[cls_key] = wall
        link = dgram.get("link") or {}
        for row in link.get("streams", []):
            self.strm_rows[row["stream"]] = row


# --------------------------------------------------------------------------
# Compact (narrow-terminal) renderer — unchanged plain-text behavior.
# --------------------------------------------------------------------------

def render_rows_compact(model, wall, width):
    d = model.d or {}
    link = d.get("link") or {}
    op = link.get("op") or {}
    cards = d.get("cards") or []
    video = link.get("video") or {}
    ring = video.get("ring") or {}

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
        cmd_ov_base = op.get("overhead_base")
        cmd_ov_enh = op.get("overhead_enh")
        drone_applied = (d.get("drone") or {}).get("applied") or {}
        state_s = state.upper() if isinstance(state, str) else "--"
        header = (
            f"maburgs   {state_s}   vtx {_s(vtx_id)}   tx c{_s(tx_card)}   "
            f"MCS {_s(mcs)}/{_s(bw)}   "
            f"{_ov_cmd_cell(cmd_ov_base, cmd_ov_enh, drone_applied.get('overhead_base'), drone_applied.get('overhead_enh'))}"
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
            rows.append(f"{label:<{LABEL_W}} (control — rendezvous + 1 Hz telemetry)")
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
                _f(s.get("evm"), LNKSIG_COLS[9][1], 1),
            ]
            rows.append(_grid_row("", cells))

    # --- DRONE telemetry region: absent-safe (null until the first T_TELEM
    # frame of the session / old maburd / old peer_caps).
    drone = d.get("drone")
    if drone is None:
        rows.append("DRONE   no telemetry (old maburd / peer caps)")
    else:
        applied = drone.get("applied") or {}
        rcf = drone.get("rcf") or {}
        enc = drone.get("enc") or {}
        txq = drone.get("txq") or {}
        radio = drone.get("radio") or {}
        uplink = drone.get("uplink") or {}
        sys_d = drone.get("sys") or {}
        state_d = drone.get("state")
        state_ds = state_d.upper() if isinstance(state_d, str) else None
        rows.append(
            f"DRONE   {_f(state_ds, 8)}  gen {_f(drone.get('gen'), 6)}   "
            f"applied {_applied_mcsbw_cell(applied.get('mcs'), applied.get('bw'))}"
            f" {_ov_applied_cell(applied.get('overhead_base'), applied.get('overhead_enh'))}  "
            f"rcf age {_age_cell(rcf.get('age_ms'))}  "
            f"tlm {_age_cell(drone.get('tlm_age_ms'))}"
        )
        rows.append(
            f"ENC     {_f(enc.get('fps'), 5, 1)} fps   "
            f"{_f(enc.get('mbps'), 5, 2)} Mbps   "
            f"cmd {_f(enc.get('cmd_kbps'), 5)}k   "
            f"qp {_f(enc.get('qp'), 2)} roi {_f(enc.get('roi_qp'), 3)}   "
            f"ring {_f(enc.get('ring_drops'), 5)}   "
            f"dis {_f(enc.get('idr_disagree'), 3)}/{_f(enc.get('enhance_disagree'), 3)}   "
            f"van {_f(enc.get('vanished_base'), 3)}/{_f(enc.get('vanished_enh'), 3)}   "
            f"vring {_f(enc.get('venc_ring_fill_pct'), 3)}% "
            f"drop {_f(enc.get('venc_full_drops'), 4)}"
        )
        rows.append(
            f"TXQ     depth {_f(txq.get('depth'), 3)}/{_f(txq.get('cap'), 3)}   "
            f"sent {_f(radio.get('sent_pps'), 6, 0)} pps   "
            f"drop {_f(txq.get('drops'), 5)}   "
            f"usb fail {_f(radio.get('usb_fail'), 5)}"
        )
        rows.append(
            f"UPLNK   rssi {_f(uplink.get('rssi_a'), 6, 1)} {_f(uplink.get('rssi_b'), 6, 1)}   "
            f"snr {_f(uplink.get('snr_a'), 5, 1)} {_f(uplink.get('snr_b'), 5, 1)}   "
            f"rcf rx {_f(rcf.get('rx_pps'), 5, 1)}/s"
        )
        soc = sys_d.get("soc_temp_c")
        if soc is not None and soc <= -128:
            soc = None
        radio_rx_ok = drone.get("radio_rx_ok")
        rx_s = None if radio_rx_ok is None else ("ok" if radio_rx_ok else "DEAF")
        rows.append(
            f"SYS     soc {_f(soc, 3)}C   "
            f"rf delta {_f(sys_d.get('thermal_delta'), 3)}   "
            f"load {_f(sys_d.get('load'), 4, 2)}   "
            f"radio rx {_f(rx_s, 4)}   "
            f"shed {(_shed_cell(drone) or '--').ljust(4)}"
        )

    # --- link-wide residual (per-stream delivery now lives on the dec lines)
    residual = link.get("residual_loss")
    residual_pct = None if residual is None else residual * 100.0
    air = link.get("air_pct")
    # link.rtt (link-rtt 2026-09-02): control-path RTT — telem queues behind
    # video on the drone TX, so this reads high under saturation. flr is the
    # absolute network floor (anchor + pts offset).
    rtt = link.get("rtt") or {}
    rows.append(f"LINK    residual {_f(residual_pct, 5, 1)} %"
                f"   air ~{_f(air, 4, 1)}% of ladder"
                f"   rtt {_f(rtt.get('ms'), 4, 1)} ms"
                f" (min {_f(rtt.get('min_ms'), 4, 1)})"
                f"   flr {_f(rtt.get('floor_ms'), 4, 1)} ms")

    # --- VIDEO ---
    rows.append(
        f"VIDEO  {_f(fps, 6, 1)} fps  {_f(mbps, 6, 2)} Mbps   "
        f"jit {_f(video.get('jitter_ms'), 5, 1)} ms   "
        f"clean {_f(video.get('clean'), 7)}   "
        f"trunc {_f(video.get('truncated'), 4)}   "
        f"drop {_f(video.get('dropped'), 4)}"
    )

    # --- AU ring (PR C: replaced the RTP row -- video leaves maburgs via
    # the shm ring; published/dropped come from the writer's own counters) ---
    rows.append(
        f"RING    pub {_f(ring.get('published'), 8)}   "
        f"drop {_f(ring.get('dropped_oversize'), 3)}   "
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


# --------------------------------------------------------------------------
# Compose layer
# --------------------------------------------------------------------------

def hstack(left_rows, right_rows, gutter=" │  "):
    """Glue two styled-row lists side by side: pad both to their own max
    width, join row-wise with gutter, offsetting the right side's spans."""
    lw = max((len(t) for t, _ in left_rows), default=0)
    rw = max((len(t) for t, _ in right_rows), default=0)
    n = max(len(left_rows), len(right_rows))
    offset = lw + len(gutter)
    out = []
    for i in range(n):
        lt, ls = left_rows[i] if i < len(left_rows) else ("", [])
        rt, rs = right_rows[i] if i < len(right_rows) else ("", [])
        text = lt.ljust(lw) + gutter + rt.ljust(rw)
        spans = list(ls) + [(start + offset, length, style)
                             for start, length, style in rs]
        out.append((text, spans))
    return out


def _title_row(label, width):
    core = f"── {label} ──"
    pad = max(0, width - len(core))
    text = core + "─" * pad
    return (text, [(0, len(text), "title")])


def _panel(label, body_rows, min_width=20):
    """Prepend a self-sized "── LABEL ──────" title row, its rule stretched
    to the panel's own natural content width (panels don't know the
    terminal width — hstack/render_screen handle cross-panel alignment)."""
    width = max([len(t) for t, _ in body_rows] + [min_width])
    return [_title_row(label, width)] + body_rows


# --------------------------------------------------------------------------
# Panels (wide-mode)
# --------------------------------------------------------------------------

def panel_topbar(model, wall):
    d = model.d or {}
    link = d.get("link") or {}
    op = link.get("op") or {}
    state = link.get("state")
    state_s = state.upper() if isinstance(state, str) else "--"
    vtx_id = link.get("vtx_id")
    mcs, bw = op.get("mcs"), op.get("bw")
    cmd_ov_base = op.get("overhead_base")
    cmd_ov_enh = op.get("overhead_enh")
    drone_applied = (d.get("drone") or {}).get("applied") or {}
    air = link.get("air_pct")
    session = model.session
    session_s = "--" if session is None else f"0x{session:08x}"

    rx_times = model.rx_times
    if len(rx_times) >= 2:
        dt = rx_times[-1] - rx_times[0]
        hz = (len(rx_times) - 1) / dt if dt > 0 else 0.0
    else:
        hz = 0.0

    dot = "●"
    text = (
        f" maburgs  {dot} {state_s}   vtx {_s(vtx_id)}   "
        f"cmd MCS {_s(mcs)}/{_s(bw)}  "
        f"{_ov_cmd_cell(cmd_ov_base, cmd_ov_enh, drone_applied.get('overhead_base'), drone_applied.get('overhead_enh'))}   "
        f"air ~{_s(air, 0)}%      session {session_s}   "
        f"restarts {model.restarts}   rx {hz:.1f} Hz"
    )
    spans = []
    dot_start = text.index(dot)
    if state == "beaconing":
        spans.append((dot_start, len(dot), "warn"))
    elif state == "session":
        spans.append((dot_start, len(dot), "good"))

    stale = model.last_rx_wall is not None and (wall - model.last_rx_wall) > STALE_S
    if stale:
        age = wall - model.last_rx_wall
        prefix = f"STALE — last seen {age:.1f} s ago  "
        full_text = prefix + text
        bar_row = (full_text, [(0, len(prefix), "rev"),
                                (len(prefix), len(text), "dim")])
    else:
        bar_row = (text, spans)

    rows = [bar_row, ("═" * len(bar_row[0]), [])]
    if model.bad_version is not None:
        rows.append((f"unsupported schema v={model.bad_version}", []))
    return rows


def _shed_cell(drone):
    """Which shed holds the enh layer: FS (failsafe, rung 0 / lost link)
    wins over CONG (drone-local TxQueue-pressure or USB-failure shed,
    sideport drone.congestion_shed, 2026-09-03), else "off". None when the
    recording predates the congestion key, so it renders as dashes."""
    if drone.get("failsafe_shed"):
        return "FS"
    cong = drone.get("congestion_shed")
    if cong is None:
        return None
    return "CONG" if cong else "off"


def panel_drone(model, wall):
    d = model.d or {}
    link = d.get("link") or {}
    op = link.get("op") or {}
    drone = d.get("drone")

    if drone is None:
        text = "no telemetry (old maburd / peer caps)"
        return _panel("DRONE", [(text, [(0, len(text), "dim")])])

    prev_drone = model.prev_drone or {}
    body = []

    # state / gen / tlm age
    state = drone.get("state")
    state_s = state.upper() if isinstance(state, str) else None
    tlm_age = _age_cell(drone.get("tlm_age_ms"))
    line = (f"state     {_f(state_s, 10)}   gen {_f(drone.get('gen'), 10)}   "
            f"tlm {tlm_age}")
    spans = []
    style_map = {"LINKED": "good", "FAILSAFE": "bad", "BOOT": "warn",
                 "RENDEZVOUS": "warn"}
    if state_s in style_map:
        idx = line.index(state_s)
        spans.append((idx, len(state_s), style_map[state_s]))
    tlm_idx = line.rindex("tlm " + tlm_age) + len("tlm ")
    spans.append((tlm_idx, len(tlm_age), "dim"))
    body.append((line, spans))

    # applied vs commanded op. Overhead is NOT compared against
    # op.overhead_base/overhead_enh here: the runtime AirBalancer solver
    # that used to explain a commanded-vs-applied split is deleted
    # (2026-08-30 same-rate-fixed-pairs) — applied now equals commanded
    # except while a bench :8301 override is armed, so a numeric diff here
    # means an armed override or staleness, not the expected behavior an
    # mcs/bw mismatch would flag.
    applied = drone.get("applied") or {}
    mcsbw = _applied_mcsbw_cell(applied.get("mcs"), applied.get("bw"))
    ov_cell = _ov_applied_cell(applied.get("overhead_base"), applied.get("overhead_enh"))
    line2 = f"applied   {mcsbw}   {ov_cell}"
    spans2 = []
    a_mcs, a_bw = applied.get("mcs"), applied.get("bw")
    if a_mcs != op.get("mcs") or a_bw != op.get("bw"):
        idx = line2.index(mcsbw)
        spans2.append((idx, len(mcsbw), "bad"))
    body.append((line2, spans2))

    # encoder
    enc = drone.get("enc") or {}
    line3 = (f"encoder   {_f(enc.get('fps'), 5, 1)} fps    "
             f"{_f(enc.get('mbps'), 5, 2)} Mbps    "
             f"cmd {_f(enc.get('cmd_kbps'), 5)}k   qp {_f(enc.get('qp'), 2)}"
             f" roi {_f(enc.get('roi_qp'), 3)}"
             f"   ring {_f(enc.get('ring_drops'), 5)}"
             f"   dis {_f(enc.get('idr_disagree'), 3)}/{_f(enc.get('enhance_disagree'), 3)}"
             f"   van {_f(enc.get('vanished_base'), 3)}/{_f(enc.get('vanished_enh'), 3)}"
             f" ref {_f(enc.get('self_idr_refused'), 2)}"
             f"   vring {_f(enc.get('venc_ring_fill_pct'), 3)}%"
             f" drop {_f(enc.get('venc_full_drops'), 4)}")
    ring = enc.get("ring_drops")
    spans3 = []
    dis_idx = line3.rindex("   dis ")  # anchor to cap ring span
    if isinstance(ring, (int, float)) and ring > 0:
        idx = line3.rindex("ring ") + 5
        spans3.append((idx, dis_idx - idx, "bad"))
    idr_dis = enc.get("idr_disagree")
    enh_dis = enc.get("enhance_disagree")
    if (isinstance(idr_dis, (int, float)) and idr_dis > 0) or (isinstance(enh_dis, (int, float)) and enh_dis > 0):
        idx = line3.rindex("dis ") + 4
        spans3.append((idx, len(line3) - idx, "bad"))
    body.append((line3, spans3))

    # queue (txq)
    txq = drone.get("txq") or {}
    depth, cap = txq.get("depth"), txq.get("cap")
    depth_s, cap_s = _f(depth, 3), _f(cap, 3)
    drops_s = _f(txq.get("drops"), 5)
    wait_s = _f(drone.get("txq_wait_ms"), 5)
    line4 = f"queue     {depth_s} / {cap_s}   txw {wait_s} ms   drops {drops_s}"
    spans4 = []
    if depth is not None and cap is not None and depth > cap / 2:
        idx = line4.index(depth_s)
        spans4.append((idx, len(depth_s), "warn"))
    if _increased(txq.get("drops"), (prev_drone.get("txq") or {}).get("drops")):
        idx = line4.rindex(drops_s)
        spans4.append((idx, len(drops_s), "bad"))
    body.append((line4, spans4))

    # radio (RadioTx)
    radio = drone.get("radio") or {}
    prev_radio = prev_drone.get("radio") or {}
    sent_s = _f(radio.get("sent_pps"), 6, 0)
    rdrops_s = _f(radio.get("drops"), 5)
    usbf_s = _f(radio.get("usb_fail"), 5)
    line5 = f"radio     sent {sent_s}/s   drops {rdrops_s}    usb fail {usbf_s}"
    spans5 = []
    if _increased(radio.get("drops"), prev_radio.get("drops")):
        idx = line5.index(rdrops_s)
        spans5.append((idx, len(rdrops_s), "bad"))
    if _increased(radio.get("usb_fail"), prev_radio.get("usb_fail")):
        idx = line5.rindex(usbf_s)
        spans5.append((idx, len(usbf_s), "bad"))
    body.append((line5, spans5))

    # uplink
    uplink = drone.get("uplink") or {}
    line6 = (f"uplink    rssi {_f(uplink.get('rssi_a'), 6, 1)} / "
             f"{_f(uplink.get('rssi_b'), 6, 1)}    snr "
             f"{_f(uplink.get('snr_a'), 4, 1)} / {_f(uplink.get('snr_b'), 4, 1)}")
    body.append((line6, []))

    # rcf ("of ~20" cell deliberately dropped — the GS cannot derive
    # feedback_ms from this datagram; see brief correction)
    rcf = drone.get("rcf") or {}
    line7 = (f"rcf       rx {_f(rcf.get('rx_pps'), 5, 1)}/s   "
             f"age {_age_cell(rcf.get('age_ms'))}")
    body.append((line7, []))

    # system
    sys_d = drone.get("sys") or {}
    soc = sys_d.get("soc_temp_c")
    if soc is not None and soc <= -128:
        soc = None
    soc_s = _f(soc, 3)
    line8 = (f"system    soc {soc_s}°C    "
             f"rf Δ{_f(sys_d.get('thermal_delta'), 2)}    "
             f"load {_f(sys_d.get('load'), 5, 2)}")
    spans8 = []
    if soc is not None:
        style = "bad" if soc > 85 else ("warn" if soc > 75 else None)
        if style:
            idx = line8.index(soc_s)
            spans8.append((idx, len(soc_s), style))
    if drone.get("radio_rx_ok") is False:
        line8 += "    radio rx DEAF"
        idx = line8.rindex("DEAF")
        spans8.append((idx, len("DEAF"), "bad"))
    shed = _shed_cell(drone)
    shed_s = (shed if shed is not None else "--").ljust(4)
    line8 += f"    shed {shed_s}"
    if shed in ("FS", "CONG"):
        idx = line8.rindex(shed_s)
        spans8.append((idx, len(shed_s), "warn"))
    body.append((line8, spans8))

    return _panel("DRONE", body)


def panel_video(model, wall):
    d = model.d or {}
    link = d.get("link") or {}
    video = link.get("video") or {}
    ring = video.get("ring") or {}
    prev_video = model.prev_video or {}
    prev_ring = prev_video.get("ring") or {}

    body = []

    jitter = video.get("jitter_ms")
    jitter_s = _f(jitter, 5, 1)
    line1 = (f"out       {_f(video.get('fps'), 5, 1)} fps    "
             f"{_f(video.get('mbps'), 5, 2)} Mbps     jitter {jitter_s} ms")
    spans1 = []
    if jitter is not None and jitter > 20:
        idx = line1.index(jitter_s)
        spans1.append((idx, len(jitter_s), "warn"))
    body.append((line1, spans1))

    trunc_s, drop_s = _f(video.get("truncated"), 4), _f(video.get("dropped"), 4)
    line2 = f"frames    clean {_f(video.get('clean'), 7)}    trunc {trunc_s}     drop {drop_s}"
    spans2 = []
    if _increased(video.get("truncated"), prev_video.get("truncated")):
        idx = line2.index(f"trunc {trunc_s}")
        spans2.append((idx, len(f"trunc {trunc_s}"), "bad"))
    if _increased(video.get("dropped"), prev_video.get("dropped")):
        idx = line2.index(f"drop {drop_s}")
        spans2.append((idx, len(f"drop {drop_s}"), "bad"))
    body.append((line2, spans2))

    drop_ring_s, qdrop_s = _f(ring.get("dropped_oversize"), 3), _f(video.get("q_drop"), 3)
    line3 = (f"ring      pub {_f(ring.get('published'), 8)}    drop {drop_ring_s}"
             f"       q_drop {qdrop_s}")
    spans3 = []
    if _increased(ring.get("dropped_oversize"), prev_ring.get("dropped_oversize")):
        idx = line3.index(f"drop {drop_ring_s}")
        spans3.append((idx, len(f"drop {drop_ring_s}"), "bad"))
    if _increased(video.get("q_drop"), prev_video.get("q_drop")):
        idx = line3.index(f"q_drop {qdrop_s}")
        spans3.append((idx, len(f"q_drop {qdrop_s}"), "bad"))
    body.append((line3, spans3))

    residual = link.get("residual_loss")
    residual_pct = None if residual is None else residual * 100.0
    rtt = link.get("rtt") or {}
    body.append((f"fec       residual {_f(residual_pct, 4, 1)} %"
                 f"   rtt {_f(rtt.get('ms'), 4, 1)} ms"
                 f" (min {_f(rtt.get('min_ms'), 4, 1)}"
                 f", n {_s(rtt.get('n'))})"
                 f"   flr {_f(rtt.get('floor_ms'), 4, 1)} ms", []))

    # lat: head-segment latency aggregates (link.video.lat), omitted
    # entirely upstream while the pts anchor isn't usable or the window is
    # empty -- absent key here reads the same as "--" everywhere below.
    lat = video.get("lat")
    if lat is not None:
        def _lat(seg):
            v = lat.get(seg)
            if not v:
                return "--/--"
            return f"{v[0] / 1000.0:.0f}/{v[1] / 1000.0:.0f}"

        line_lat = (f"lat ms p50/p99  enc {_lat('enc'):>7}  dq {_lat('dq'):>7}  "
                    f"air+ {_lat('air'):>7}  fec {_lat('fec'):>7}")
        body.append((line_lat, []))

    drone = d.get("drone")
    if drone is not None:
        enc = drone.get("enc") or {}
        enc_fps, out_fps = enc.get("fps"), video.get("fps")
        radio = drone.get("radio") or {}
        sent_pps = radio.get("sent_pps")
        cards = d.get("cards") or []
        inj_vals = [c.get("inj_pps") for c in cards if c.get("inj_pps") is not None]
        inj_pps = max(inj_vals) if inj_vals else None

        parts, cross_spans, cursor = [], [], 0
        if enc_fps is not None and out_fps is not None:
            seg = f"encoder {_f(enc_fps, 5, 1)} fps ──► out {_f(out_fps, 5, 1)} fps"
            ok = enc_fps != 0 and abs(enc_fps - out_fps) <= 0.05 * abs(enc_fps)
            cross_spans.append((cursor, len(seg), "good" if ok else "bad"))
            parts.append(seg)
            cursor += len(seg) + 6
        if sent_pps is not None and inj_pps is not None:
            seg2 = f"sent {_f(sent_pps, 5, 0)}/s ──► inj {_f(inj_pps, 5, 0)}/s"
            ok2 = sent_pps != 0 and abs(sent_pps - inj_pps) <= 0.05 * abs(sent_pps)
            cross_spans.append((cursor, len(seg2), "good" if ok2 else "bad"))
            parts.append(seg2)
        if parts:
            body.append(("", []))
            body.append(("      ".join(parts), cross_spans))

    return _panel("VIDEO OUT", body)


CLASS_DORMANT_S = 3.0


def _block_dormant(model, wall, cls):
    """A block is dormant when its class has shown zero pps for
    CLASS_DORMANT_S — long enough that a healthy 1 Hz stream (ctl carries
    the telemetry beat) never flickers dim between 500 ms rate windows,
    short enough that a genuinely dead class dims within a few redraws."""
    last = model.class_active.get(cls)
    return last is None or (wall - last) > CLASS_DORMANT_S


def _tx_line(label, strm, dlv):
    inj_kbps = strm.get("inj_kbps")
    inj_m = None if inj_kbps is None else inj_kbps / 1000.0
    dlv_s = _f(dlv, 3, 0)
    return (
        f"  {label:<20}tx  {_rung_cell(strm)} @ {_f(strm.get('phy_mbps'), 5, 1)}M"
        f"    ov {_f(strm.get('ov'), 4, 2)}    dlv {dlv_s}%"
        f"    inj ~{_f(inj_m, 4, 1)}M"
    )


def _decode_line(strm):
    return (
        f"     decode          rec/s {_f(strm.get('recovered_s'), 6, 1)}"
        f"   abn/s {_f(strm.get('abandoned_s'), 5, 1)}"
        f"   in/s {_f(strm.get('syms_in_s'), 6, 0)}"
        f"   sfail {_f(strm.get('sub_fail'), 2)}"
        f"   flt {_f(strm.get('in_flight'), 2)}"
    )


def _annotation_line(label, text):
    return f"  {label:<20}{text}"


def _radio_header():
    return _grid_row("     radio", [t.rjust(w) for t, w in RADIO_COLS],
                      label_w=RADIO_LABEL_W)


def _radio_row(cid, sig):
    mbps_c = sig.get("mbps")
    kbps = None if mbps_c is None else mbps_c * 1000.0
    cells = [
        _f(sig.get("pps"), RADIO_COLS[0][1], 0),
        _f(kbps, RADIO_COLS[1][1], 0),
        _f(sig.get("rssi"), RADIO_COLS[2][1], 1),
        _f(sig.get("rssi_a"), RADIO_COLS[3][1], 1),
        _f(sig.get("rssi_b"), RADIO_COLS[4][1], 1),
        _f(sig.get("snr"), RADIO_COLS[5][1], 1),
        _f(sig.get("snr_a"), RADIO_COLS[6][1], 1),
        _f(sig.get("snr_b"), RADIO_COLS[7][1], 1),
        _f(sig.get("evm"), RADIO_COLS[8][1], 1),
        _f(sig.get("evm_a"), RADIO_COLS[9][1], 1),
        _f(sig.get("evm_b"), RADIO_COLS[10][1], 1),
    ]
    return _grid_row(f"       c{_s(cid)}", cells, label_w=RADIO_LABEL_W)


def _build_block(model, wall, d, link, cls):
    label = LINK_CLASS_LABELS.get(cls, cls)
    sid = int(cls[1]) if cls.startswith("s") and cls[1:].isdigit() else None
    rows = []

    if sid is not None and sid in model.strm_rows:
        strm = model.strm_rows[sid]
        layers = link.get("layer_delivery_pct") or []
        dlv = layers[sid] if sid < len(layers) else None
        tx_text = _tx_line(label, strm, dlv)
        tx_spans = []
        if dlv is not None:
            dlv_cell = f"dlv {_f(dlv, 3, 0)}%"
            idx = tx_text.find(dlv_cell)
            if idx >= 0:
                if dlv < 90:
                    tx_spans.append((idx, len(dlv_cell), "bad"))
                elif dlv < 100:
                    tx_spans.append((idx, len(dlv_cell), "warn"))
        rows.append((tx_text, tx_spans))

        dec_text = _decode_line(strm)
        dec_spans = []
        abn = strm.get("abandoned_s")
        if abn is not None and abn > 0:
            cell = f"abn/s {_f(abn, 5, 1)}"
            dec_spans.append((dec_text.index(cell), len(cell), "bad"))
        sfail = strm.get("sub_fail")
        if sfail is not None and sfail > 0:
            cell = f"sfail {_f(sfail, 2)}"
            dec_spans.append((dec_text.index(cell), len(cell), "bad"))
        rows.append((dec_text, dec_spans))
    elif cls == "msp":
        rows.append((_annotation_line(label, "(no fec decode — repairs in MspSink)"), []))
    elif cls == "ctrl":
        rows.append((_annotation_line(label, "(control — rendezvous + 1 Hz telemetry)"), []))
    else:
        rows.append((f"  {label}", []))

    card_ids = sorted({cid for cid, k in model.sig_rows if k == cls})
    if card_ids:
        rows.append((_radio_header(), []))
        for cid in card_ids:
            rows.append((_radio_row(cid, model.sig_rows[(cid, cls)]), []))

    if _block_dormant(model, wall, cls):
        rows = [(t, [(0, len(t), "dim")]) for t, _ in rows]
    return rows


def _ctl_row(ctl):
    """Ladder-controller summary row: current rung, this window's
    loss-pressure u against budget, and the most recent transition (or
    'none@0.00' before the first one ever fires). Rung overhead is a
    base/enh pair now (same-rate-fixed-pairs) — rendered in the
    _ov_applied_cell house style, not the removed scalar 'ov' key."""
    rung = ctl.get("rung") or {}
    util = ctl.get("util")
    budget = ctl.get("budget")
    last_event = ctl.get("last_event") or {}
    reason = last_event.get("reason", "none")
    u_s = _s(last_event.get("u"), 2)
    util_cell = f"u={_s(util, 2)}"
    budget_s = "--" if budget is None else f"{budget:.0%}"
    text = (
        f"  rung {_s(rung.get('idx'))} (mcs{_s(rung.get('mcs'))}"
        f" ov b{_s(rung.get('ov_base'), 2)}/e{_s(rung.get('ov_enh'), 2)})"
        f"  {util_cell} of budget {budget_s}"
        f"  [{reason}@{u_s}]"
    )
    spans = []
    if isinstance(util, (int, float)):
        style = "bad" if util >= 0.6 else ("warn" if util >= 0.4 else "good")
        idx = text.index(util_cell)
        spans.append((idx, len(util_cell), style))
    return (text, spans)


def _ladder_rung_rows(ctl):
    """One row per rung, top = highest index. Current rung gets the marker +
    good style; annotations: PROB countdown (current rung on probation),
    pen countdown (penalized), failsafe (rung 0). Per-rung overhead is a
    base/enh pair now (same-rate-fixed-pairs); the removed scalar 'ov' key
    is gone from the wire, so this renders the compact 'ov base:enh' form
    to keep the row narrow."""
    ladder = ctl.get("ladder") or []
    cur = (ctl.get("rung") or {}).get("idx")
    prob_ms = ctl.get("probation_ms_left")
    prob_ms = prob_ms if isinstance(prob_ms, (int, float)) else 0
    pen = {p.get("rung"): p.get("ms_left")
           for p in (ctl.get("penalized") or [])
           if isinstance(p, dict) and p.get("rung") is not None and isinstance(p.get("ms_left"), (int, float))}
    rows = []
    for idx in range(len(ladder) - 1, -1, -1):
        r = ladder[idx] if isinstance(ladder[idx], dict) else {}
        cell = (f"{idx} mcs{_s(r.get('mcs'))}"
                f"/ov{_s(r.get('ov_base'), 2)}:{_s(r.get('ov_enh'), 2)}")
        marker = "▶" if idx == cur else " "
        note, note_style = "", None
        if idx == cur and prob_ms > 0:
            note, note_style = f"PROB {prob_ms / 1000:.1f}s", "warn"
        elif pen.get(idx) is not None:
            note, note_style = f"pen {max(pen[idx], 0) // 1000}s", "dim"
        elif idx == 0:
            note, note_style = "failsafe", "dim"
        text = f" {marker}{cell:<20} {note}".rstrip()
        spans = []
        if idx == cur:
            spans.append((1, 1 + len(cell), "good"))
        if note:
            spans.append((text.index(note), len(note), note_style))
        rows.append((text, spans))
    return rows


def _ladder_footer_rows(ctl, t_ms):
    """Pressure line (u vs the real thresholds + raw pre-FEC loss + budget),
    last-event line with client-computed age (datagram t_ms − event t_ms,
    same daemon clock), and the lifetime transition counters."""
    util, down, up = ctl.get("util"), ctl.get("down_util"), ctl.get("up_util")
    loss, budget = ctl.get("pre_fec_loss"), ctl.get("budget")
    u_cell = f"u={_s(util, 2)}/{_s(down, 2)}"
    loss_s = "--" if not isinstance(loss, (int, float)) else f"{loss * 100:.1f}%"
    bud_s = "--" if not isinstance(budget, (int, float)) else f"{budget:.0%}"
    line1 = f" {u_cell}  loss {loss_s}  bud {bud_s}"
    spans1 = []
    if isinstance(util, (int, float)):
        if isinstance(down, (int, float)) and util >= down:
            style = "bad"
        elif isinstance(up, (int, float)) and util >= up:
            style = "warn"
        else:
            style = "good"
        spans1.append((1, len(u_cell), style))

    ev = ctl.get("last_event") or {}
    reason = ev.get("reason", "none")
    if reason == "none":
        line2, spans2 = " last: none", [(1, len("last: none"), "dim")]
    else:
        age = ""
        ev_t = ev.get("t_ms")
        if isinstance(ev_t, (int, float)) and isinstance(t_ms, (int, float)):
            age = f"  {_age_cell(int(max(t_ms - ev_t, 0))).strip()} ago"
        line2 = (f" last: {reason} {_s(ev.get('from'))}→{_s(ev.get('to'))}"
                 f" @{_s(ev.get('u'), 2)}{age}")
        spans2 = []

    c = ctl.get("counters") or {}
    line3 = (f" ↑{_s(c.get('promotes'))} ↓res{_s(c.get('demotes_residual'))}"
             f" ↓util{_s(c.get('demotes_util'))}"
             f" ↓fade{_s(c.get('demotes_fade'))}"
             f" prob✗{_s(c.get('probation_fails'))}"
             f" stv{_s(c.get('starved_drops'))} to{_s(c.get('timeout_drops'))}")
    return [(line1, spans1), (line2, spans2), (line3, [])]


def panel_ladder(model, wall):
    """Ladder-controller panel (wide mode). Empty when the feed has no ctl
    block (static pin) or predates the ladder field (old daemon) — callers
    then fall back to the one-line ctl row inside LINKS."""
    d = model.d or {}
    ctl = (d.get("link") or {}).get("ctl")
    if not ctl or not ctl.get("ladder"):
        return []
    body = _ladder_rung_rows(ctl)
    body.append(("", []))
    body.extend(_ladder_footer_rows(ctl, d.get("t_ms")))
    at = (d.get("link") or {}).get("attrib")
    if at:
        # `suppressed` was deleted 2026-09-02 with the packet-level delivery
        # window it was defined against; residual_cur is the current-rung
        # (attributed) post-FEC loss the ladder actually demotes on.
        rc = at.get("residual_cur")
        rc_pct = None if rc is None else rc * 100.0
        body.append((f" attrib resid_cur{_f(rc_pct, 5, 1)}%"
                     f" close{_s(at.get('close_ms'), 0)}ms", []))
    fade = (ctl or {}).get("fade") or {}
    if fade.get("active"):
        body.append((" fade:ACTIVE"
                     f" drssi{_s(fade.get('drssi'), 1)} dsnr{_s(fade.get('dsnr'), 1)}", []))
    return _panel("LADDER", body, min_width=34)


def panel_links(model, wall, ctl_row=True):
    d = model.d or {}
    link = d.get("link") or {}
    seen_classes = {cls for _cid, cls in model.sig_rows}
    seen_classes |= {f"s{sid}" for sid in model.strm_rows}
    blocks = [cls for cls in CLASS_ORDER if cls in seen_classes]

    body = []
    ctl = link.get("ctl")
    if ctl and ctl_row:
        body.append(_ctl_row(ctl))
        body.append(("", []))

    if not blocks:
        text = "  no link data"
        body.append((text, [(0, len(text), "dim")]))
    else:
        for i, cls in enumerate(blocks):
            body.extend(_build_block(model, wall, d, link, cls))
            if i != len(blocks) - 1:
                body.append(("", []))

    return _panel("LINKS", body, min_width=40)


def panel_gs_radios(model, wall):
    d = model.d or {}
    cards = d.get("cards") or []
    body = [(_grid_row("", [t.rjust(w) for t, w in CARD_COLS]), [])]

    if not cards:
        empty = _grid_row("  --", ["no cards".ljust(_grid_width(CARD_COLS) - LABEL_W - 1)])
        body.append((empty, [(0, len(empty), "dim")]))
    else:
        offsets = _cell_offsets([w for _, w in CARD_COLS])
        for c in cards:
            up = c.get("up")
            st_s = "UP" if up else ("DOWN" if up is not None else None)
            loss, crc, txf, inj = (c.get("loss_pct"), c.get("crc_fail"),
                                    c.get("tx_fail"), c.get("inj_pps"))
            cells = [
                _f(st_s, CARD_COLS[0][1]),
                _f(c.get("pps"), CARD_COLS[1][1], 0),
                _f(inj, CARD_COLS[2][1], 0),
                _f(c.get("rx_mbps"), CARD_COLS[3][1], 1),
                _f(loss, CARD_COLS[4][1], 1),
                _f(crc, CARD_COLS[5][1]),
                _age_cell(c.get("last_frame_age_ms"), CARD_COLS[6][1]),
                _f(c.get("foreign_pps"), CARD_COLS[7][1], 1),
                _f(c.get("self_pps"), CARD_COLS[8][1], 1),
                _f(c.get("tx_pps"), CARD_COLS[9][1], 0),
                _f(txf, CARD_COLS[10][1]),
            ]
            text = _grid_row(f"  c{_s(c.get('id'))}", cells)
            spans = []
            if st_s == "UP":
                spans.append((offsets[0], CARD_COLS[0][1], "good"))
            elif st_s == "DOWN":
                spans.append((offsets[0], CARD_COLS[0][1], "bad"))
            spans.append((offsets[2], CARD_COLS[2][1], "dim"))  # inj: estimate
            if loss is not None:
                if loss > 5:
                    spans.append((offsets[4], CARD_COLS[4][1], "bad"))
                elif loss > 0.5:
                    spans.append((offsets[4], CARD_COLS[4][1], "warn"))
            prev = model.prev_cards.get(c.get("id")) or {}
            if _increased(crc, prev.get("crc_fail")):
                spans.append((offsets[5], CARD_COLS[5][1], "bad"))
            if txf is not None and txf > 0:
                spans.append((offsets[10], CARD_COLS[10][1], "bad"))
            body.append((text, spans))

    return _panel("GS RADIOS (physical)", body)


def render_screen(model, wall, w, h):
    """Compose the full-screen layout for terminal size (w, h). w < 100
    falls back to the plain-text compact renderer, auto-wrapped to styled
    rows with no spans."""
    if w < 100:
        return [(t, []) for t in render_rows_compact(model, wall, w)]

    rows = list(panel_topbar(model, wall))
    drone_rows = panel_drone(model, wall)
    video_rows = panel_video(model, wall)
    ladder_rows = panel_ladder(model, wall)
    links_rows = panel_links(model, wall, ctl_row=not ladder_rows)
    gs_rows = panel_gs_radios(model, wall)

    rows.append(("", []))
    if w >= 150:
        rows.extend(hstack(drone_rows, video_rows))
    else:
        rows.extend(drone_rows)
        rows.append(("", []))
        rows.extend(video_rows)
    rows.append(("", []))
    if ladder_rows:
        lw = max(len(t) for t, _ in links_rows)
        aw = max(len(t) for t, _ in ladder_rows)
        if lw + 4 + aw <= w:
            rows.extend(hstack(links_rows, ladder_rows))
        else:
            rows.extend(ladder_rows)
            rows.append(("", []))
            rows.extend(links_rows)
    else:
        rows.extend(links_rows)
    rows.append(("", []))
    rows.extend(gs_rows)
    return rows


# --------------------------------------------------------------------------
# curses loop
# --------------------------------------------------------------------------

def _resolve_style(name):
    fn = STYLES.get(name)
    return fn() if fn else 0


def _init_colors():
    if not curses.has_colors():
        return
    curses.start_color()
    try:
        curses.use_default_colors()
        bg = -1
    except curses.error:
        bg = curses.COLOR_BLACK
    curses.init_pair(1, curses.COLOR_GREEN, bg)
    curses.init_pair(2, curses.COLOR_YELLOW, bg)
    curses.init_pair(3, curses.COLOR_RED, bg)


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
        _init_colors()
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
                    for y, (text, spans) in enumerate(render_screen(model, now, w - 1, h)):
                        if y >= h:
                            break
                        scr.addnstr(y, 0, text, w - 1)
                        for start, length, style in spans:
                            if start >= w - 1 or start < 0:
                                continue
                            seg = text[start:start + length]
                            remaining = (w - 1) - start
                            scr.addnstr(y, start, seg, remaining, _resolve_style(style))
                except Exception:
                    pass
                scr.refresh()

    try:
        curses.wrapper(loop)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
