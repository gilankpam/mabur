#!/usr/bin/env python3
# GS flight instrument: always-on recorder for post-flight jitter/stutter
# attribution (tools/flightjitter.py is the analyzer).
#
# Two capture threads, one session index NNNN (max existing + 1):
#   - AU meta rows from the maburgs AU ring -> <logdir>/au-NNNN.log
#     ("t_us pts sid fid len flags nal0", one line per AU, flushed 1/s).
#     Ring layout mirrors gs/src/au_ring.h byte-for-byte, reader logic
#     mirrors tools/bench/ausniff.py (seqlock copy, epoch resync). Only
#     the first payload bytes are copied (NAL type), never whole AUs.
#   - Sideport 1 Hz JSON datagrams on UDP :8300 -> <logdir>/flight-NNNN.jsonl
#     (absorbs the old /root/rec8300.py so the stats recorder can never
#     again be left unwired while this daemon runs).
#
# Deliberately dependency-free single file: it is scp'd to the GS as-is
# and started by tools/gs/S95flightrec.
import argparse
import json
import mmap
import os
import re
import socket
import struct
import sys
import threading
import time

HDR = 4096
SLOT_HDR = 64
MAGIC = 0x4D425541
# SlotHdr v2 (kAuRingVersion 2, 2026-08-30 latency-accounting task 6): the
# latency fields (t_first_us/t_complete_us/drone_q_ms/enc_us at slot offsets
# 32/40/48/50) are parsed by read_slot_meta below (task 13).
VERSION = 2
STALL_LIMIT = 500
MAX_AU_LOG_BYTES = 1 << 30  # 1 GiB/session cap; ~17 MB/h means weeks of margin


def nal0_of(buf):
    """First NAL unit type (H.265) after a 3- or 4-byte start code, else -1."""
    if len(buf) >= 5 and buf[0] == 0 and buf[1] == 0:
        if buf[2] == 1:
            return (buf[3] >> 1) & 0x3F
        if len(buf) >= 6 and buf[2] == 0 and buf[3] == 1:
            return (buf[4] >> 1) & 0x3F
    return -1


def read_slot_meta(mm, base, slot_bytes):
    """One seqlock-validated meta copy (no payload). None if torn/mid-write."""
    l1 = struct.unpack_from("<I", mm, base)[0]
    if l1 & 1:
        return None
    ln, rec, fid, pts, sid, flags, codec = struct.unpack_from(
        "<IQQIBBB", mm, base + 4)
    t_first, t_complete = struct.unpack_from("<QQ", mm, base + 32)
    dq_ms, enc_us = struct.unpack_from("<HH", mm, base + 48)
    if ln > slot_bytes:
        return None
    head = bytes(mm[base + SLOT_HDR:base + SLOT_HDR + min(ln, 6)])
    l2 = struct.unpack_from("<I", mm, base)[0]
    if l1 != l2:
        return None
    return {"rec": rec, "len": ln, "fid": fid, "pts": pts, "sid": sid,
            "flags": flags, "codec": codec, "nal0": nal0_of(head),
            "t_first": t_first, "t_complete": t_complete,
            "dq_ms": dq_ms, "enc_us": enc_us}


def open_ring(path):
    """Map + validate the ring header. Returns (f, mm, slot_bytes, slot_count,
    epoch) or None (missing/torn/mid-recreate — caller retries)."""
    try:
        f = open(path, "rb")
    except OSError:
        return None
    try:
        mm = mmap.mmap(f.fileno(), 0, prot=mmap.PROT_READ)
    except (OSError, ValueError):
        f.close()
        return None
    magic, ver, slot_bytes, slot_count = struct.unpack_from("<IIII", mm, 0)
    if (magic != MAGIC or ver != VERSION or slot_bytes == 0 or
            slot_bytes % 64 != 0 or slot_count == 0 or
            mm.size() < HDR + slot_count * (SLOT_HDR + slot_bytes)):
        mm.close()
        f.close()
        return None
    epoch = struct.unpack_from("<Q", mm, 32)[0]
    return f, mm, slot_bytes, slot_count, epoch


class RingReader:
    """Cursor over the live AU ring. poll() returns new AU metas (possibly
    []), transparently resyncing on writer restarts (epoch change) and
    overruns; `resyncs` counts both, mirroring ausniff."""

    def __init__(self, path, start_at_head=False):
        self.path = path
        self.start_at_head = start_at_head
        self.resyncs = 0
        self._stall = 0
        opened = open_ring(path)
        if opened is None:
            raise FileNotFoundError(path)
        self._adopt(opened)

    def _adopt(self, opened):
        self.f, self.mm, self.slot_bytes, self.slot_count, self.epoch = opened
        w = self._wseq()
        if self.start_at_head:
            # Recorder semantics: pre-attach history would all get stamped
            # with the attach time, fabricating a burst — skip it.
            self.cursor = w
        else:
            self.cursor = w - self.slot_count if w > self.slot_count else 0

    def _wseq(self):
        return struct.unpack_from("<Q", self.mm, 16)[0]

    def _slot_base(self, n):
        return HDR + (n % self.slot_count) * (SLOT_HDR + self.slot_bytes)

    def poll(self, max_n=256):
        cur_epoch = struct.unpack_from("<Q", self.mm, 32)[0]
        if cur_epoch != self.epoch:
            reopened = open_ring(self.path)
            if reopened is None:
                return []  # mid-recreate; retry next poll
            self.mm.close()
            self.f.close()
            self._adopt(reopened)
            self.resyncs += 1
            self._stall = 0
        out = []
        while len(out) < max_n:
            w = self._wseq()
            if self.cursor >= w:
                break
            m = read_slot_meta(self.mm, self._slot_base(self.cursor),
                               self.slot_bytes)
            if m is None or m["rec"] < self.cursor:
                self._stall += 1
                if self._stall > STALL_LIMIT:
                    self.resyncs += 1
                    tail = w - self.slot_count if w > self.slot_count else 0
                    self.cursor = max(self.cursor, tail)
                    self._stall = 0
                break  # torn or stale: let the writer make progress
            self._stall = 0
            if m["rec"] > self.cursor:  # overrun: records in between are gone
                self.resyncs += 1
            self.cursor = m["rec"] + 1
            out.append(m)
        return out

    def close(self):
        self.mm.close()
        self.f.close()


def extract_t_ms(datagram):
    """Sideport datagram's TOP-LEVEL t_ms (the live maburgs clock), or None.
    Must be a real JSON parse: the datagram also serializes nested, frozen
    timestamps (ctl.last_event.t_ms, last_probe.t_ms) before the top-level
    key, so any first-match-in-bytes shortcut returns a stale clock. Called
    at most every sync_period_s — parse cost is irrelevant."""
    try:
        t = json.loads(datagram).get("t_ms")
    except (json.JSONDecodeError, UnicodeDecodeError, AttributeError):
        return None
    return int(t) if isinstance(t, (int, float)) else None


def pick_index(logdir):
    """Next session index: 1 + max NNNN over flight-NNNN.jsonl / au-NNNN.log."""
    top = -1
    pat = re.compile(r"^(?:flight|au)-(\d+)\.(?:jsonl|log)$")
    for name in os.listdir(logdir):
        mo = pat.match(name)
        if mo:
            top = max(top, int(mo.group(1)))
    return top + 1


def format_row(t_us, m):
    return (f"{t_us} {m['pts']} {m['sid']} {m['fid']} {m['len']} "
            f"0x{m['flags']:02x} {m['nal0']} {m['t_first']} "
            f"{m['t_complete']} {m['enc_us']} {m['dq_ms']}")


class AuLog:
    """The au-NNNN.log writer, shared by both threads: the ring thread
    appends AU rows, the UDP thread appends '# sync <t_us> <t_ms>' clock
    anchors (flightjitter.load_au_log derives the jsonl<->au clock offset
    from them). One lock; the ring thread owns flushing."""

    def __init__(self, path):
        self.path = path
        self.f = open(path, "a", buffering=1 << 16)
        self.lock = threading.Lock()
        self.written = 0
        # Every session starts a fresh au-NNNN.log (pick_index), so this is
        # always the first line: flightjitter.load_au_log keys its v1-vs-v2
        # row parsing on it (absent marker = pre-2026-08-31 v1 rows).
        self.write_line("# aulog 2")

    def write_line(self, line):
        with self.lock:
            self.f.write(line + "\n")
            self.written += len(line) + 1

    def flush(self):
        with self.lock:
            self.f.flush()


def ring_loop(ring_path, log):
    reader = None
    last_flush = time.monotonic()
    last_resyncs = 0
    while True:
        if reader is None:
            try:
                reader = RingReader(ring_path, start_at_head=True)
                last_resyncs = reader.resyncs
            except FileNotFoundError:
                time.sleep(1.0)
                continue
        metas = reader.poll()
        if reader.resyncs != last_resyncs:
            log.write_line(f"# resync {reader.resyncs}")
            last_resyncs = reader.resyncs
        t_us = time.time_ns() // 1000
        for m in metas:
            log.write_line(format_row(t_us, m))
        if log.written > MAX_AU_LOG_BYTES:
            log.write_line("# capped")
            log.flush()
            print(f"flightrec: {log.path} hit size cap, au capture stopped",
                  file=sys.stderr)
            return
        now = time.monotonic()
        if now - last_flush >= 1.0:
            log.flush()
            last_flush = now
        if not metas:
            time.sleep(0.002)


def udp_loop(port, out_path, log=None, sync_period_s=10.0):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("127.0.0.1", port))
    last_sync = 0.0
    with open(out_path, "ab", buffering=0) as f:
        while True:
            d = s.recv(65535)
            f.write(d.rstrip(b"\n") + b"\n")
            now = time.monotonic()
            if log is not None and now - last_sync >= sync_period_s:
                t_ms = extract_t_ms(d)
                if t_ms is not None:
                    log.write_line(f"# sync {time.time_ns() // 1000} {t_ms}")
                    last_sync = now


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--logdir", default="/media/dvr/log")
    ap.add_argument("--ring", default="/dev/shm/mabur-au")
    ap.add_argument("--port", type=int, default=8300)
    a = ap.parse_args()
    os.makedirs(a.logdir, exist_ok=True)
    idx = pick_index(a.logdir)
    au_path = os.path.join(a.logdir, f"au-{idx:04d}.log")
    jsonl_path = os.path.join(a.logdir, f"flight-{idx:04d}.jsonl")
    print(f"flightrec: session {idx:04d} -> {au_path} + {jsonl_path}",
          flush=True)
    log = AuLog(au_path)
    t = threading.Thread(target=udp_loop, args=(a.port, jsonl_path, log),
                         daemon=True)
    t.start()
    ring_loop(a.ring, log)  # returns only on size cap
    t.join()


if __name__ == "__main__":
    main()
