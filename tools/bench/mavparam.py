#!/usr/bin/env python3
"""mavparam.py -- list/get/set the drone's config parameters over maburgs's
MAVLink UDP endpoint (docs/vtx-config-mavlink.md).

Usage:
  GS=<gs-ip> PORT=14550 python3 tools/bench/mavparam.py list
  GS=<gs-ip> PORT=14550 python3 tools/bench/mavparam.py get ID
  GS=<gs-ip> PORT=14550 python3 tools/bench/mavparam.py set ID VALUE

GS (the GS's MAVLink endpoint address) and PORT come from the
environment, with no default (maburgs's `[mavlink] bind`/`port`). Talks
to the drone's parameter component (system 1, component 191) through
maburgs's last-peer UDP endpoint, using MAVLink's extended-parameter
protocol: PARAM_EXT_REQUEST_LIST / PARAM_EXT_REQUEST_READ / PARAM_EXT_SET
answered with PARAM_EXT_VALUE / PARAM_EXT_ACK, plus a STATUSTEXT after a
refused set.

A `set` that the drone applies answers PARAM_EXT_ACK IN_PROGRESS first
(holding the current value) and then a final ACCEPTED or FAILED (also
followed by a STATUSTEXT reason on FAILED); this tool prints the
IN_PROGRESS line and waits up to 15 s from it for the final ack before
giving up.

Exit: 0 on an ACCEPTED set or a successful list/get; 1 on a runtime
failure (no heartbeat from the drone, a request that timed out after
retries, an incomplete list, a set whose final ack never arrived within
15 s of an IN_PROGRESS, or an ack other than ACCEPTED); 2 on a usage
error (missing GS/PORT, a bad command line, an unknown parameter id, or
a value that doesn't parse for its type).
"""
import os
import struct
import sys
import time

# pymavlink is imported lazily, inside main() after the usage checks below:
# it is only on sys.path inside the project's nix-shell, and a bare
# `python3 tools/bench/mavparam.py` (no GS set) must still print usage and
# exit 2 rather than fail on the import. main() binds this name at module
# scope (`global mavutil`) before any other function needs it.
mavutil = None

# The four MAV_PARAM_ACK_* result codes (common.xml), mirroring
# mavutil.mavlink.PARAM_ACK_*. Hardcoded rather than read off an imported
# pymavlink: pymavlink picks its dialect (MAVLink 1.0 vs 2.0) at first
# import and caches it, so importing it here -- before main() sets
# MAVLINK20=1 below -- would permanently pin the process to MAVLink 1.0,
# where the extended-parameter *_ext_*_send methods this file needs don't
# exist. wait_final_ack() and its test (which has no pymavlink installed)
# use these directly instead.
PARAM_ACK_ACCEPTED = 0
PARAM_ACK_VALUE_UNSUPPORTED = 1
PARAM_ACK_FAILED = 2
PARAM_ACK_IN_PROGRESS = 3

RESULT_NAMES = {
    PARAM_ACK_ACCEPTED: "ACCEPTED",
    PARAM_ACK_IN_PROGRESS: "IN_PROGRESS",
    PARAM_ACK_FAILED: "FAILED",
    PARAM_ACK_VALUE_UNSUPPORTED: "VALUE_UNSUPPORTED",
}

DRONE_SYSTEM = 1
DRONE_COMPONENT = 191  # MAV_COMP_ID_ONBOARD_COMPUTER

# The 14-row parameter table, id -> wire type ('b' Bool, 'i' Int32,
# 'f' Real32, 's' Str/CUSTOM). common/include/mabur/config_params.h is the
# source of truth for ids and types; this copy is kept in sync by hand.
TABLE = [
    ("RADIO_WALL_MARGN", "f"),
    ("RADIO_LDPC", "b"),
    ("VENC_MODE", "s"),
    ("VENC_SLICES", "i"),
    ("ENC_BITRATE_MAX", "i"),
    ("ENC_AIRTIME", "f"),
    ("REC_ENABLE", "b"),
    ("REC_BITRATE", "i"),
    ("REC_FPS", "i"),
    ("REC_SIZE", "s"),
    ("LOWPWR_ENABLE", "b"),
    ("MSP_ENABLE", "b"),
    ("VENC_MIRROR", "b"),
    ("VENC_FLIP", "b"),
]
TYPE_BY_ID = dict(TABLE)

# MAV_PARAM_EXT_TYPE values (common.xml), matching config_params.h's
# mav_type(): Bool -> UINT8, Int32 -> INT32, Real32 -> REAL32, Str -> CUSTOM.
# Used for ENCODING a `set` value (the local table says what type to send).
# DECODING never uses this by id -- see decode_value()'s note.
MAV_TYPE = {"b": 1, "i": 6, "f": 9, "s": 11}
TYPE_NAME = {"b": "Bool", "i": "Int32", "f": "Real32", "s": "Str"}
CODE_BY_MAV_TYPE = {v: k for k, v in MAV_TYPE.items()}


def encode_value(wtype, text):
    """CLI value text -> the 128-byte wire value for `wtype`."""
    if wtype == "b":
        low = text.lower()
        if low in ("1", "true"):
            buf = bytes([1])
        elif low in ("0", "false"):
            buf = bytes([0])
        else:
            raise ValueError("not 0/1/true/false: %r" % text)
    elif wtype == "i":
        buf = struct.pack("<i", int(text))
    elif wtype == "f":
        buf = struct.pack("<f", float(text))
    else:  # 's'
        b = text.encode("ascii")
        if len(b) > 127:
            raise ValueError("value too long (max 127 characters)")
        buf = b + b"\x00"
    return buf.ljust(128, b"\x00")


def decode_value(msg):
    """A (type_name, value_str) pair decoded from a PARAM_EXT_VALUE or
    PARAM_EXT_ACK message, using the message's OWN `param_type` field --
    never the local table. The printed TYPE column already comes off the
    wire; decoding VALUE by the wire's type too means the two can never
    disagree, even if the local table and config_params.h ever drift on
    a row's type (the local table is for ENCODING a `set` value only).

    pymavlink's generated `MAVLink.decode()` rstrips trailing NUL bytes
    from every fixed 'char' array field before constructing the message,
    which can truncate the 128-byte value down to just its leading
    non-zero bytes (e.g. int32 90 = 0x5a 0x00 0x00 0x00 arrives as a
    single byte). `_param_value_raw` -- set unconditionally by every
    PARAM_EXT_VALUE/PARAM_EXT_ACK message's `__init__`, including the one
    `decode()` calls -- is the exact (already-truncated) bytes; padding
    it back out on the right before unpacking recovers the original
    value, since the stripped bytes were genuinely zero.
    """
    code = CODE_BY_MAV_TYPE.get(msg.param_type)
    type_name = TYPE_NAME.get(code, "?")
    raw = msg._param_value_raw
    if isinstance(raw, str):
        raw = raw.encode("latin1", errors="replace")
    else:
        raw = bytes(raw)
    if code == "b":
        value = str(raw[0] if raw else 0)
    elif code == "i":
        value = str(struct.unpack("<i", raw[:4].ljust(4, b"\x00"))[0])
    elif code == "f":
        value = repr(struct.unpack("<f", raw[:4].ljust(4, b"\x00"))[0])
    elif code == "s":
        value = raw.split(b"\x00", 1)[0].decode("ascii", errors="replace")
    else:
        value = raw.hex()  # unrecognized MAV_PARAM_EXT_TYPE
    return type_name, value


def recv_one(conn, msgtype, timeout):
    """The next `msgtype` message from the drone component (1, 191) within
    `timeout` seconds, or None."""
    deadline = time.time() + timeout
    while True:
        remaining = deadline - time.time()
        if remaining <= 0:
            return None
        msg = conn.recv_match(type=msgtype, blocking=True, timeout=remaining)
        if msg is None:
            return None
        if msg.get_srcSystem() == DRONE_SYSTEM and msg.get_srcComponent() == DRONE_COMPONENT:
            return msg


def wait_for_drone(conn, timeout=5.0):
    return recv_one(conn, "HEARTBEAT", timeout) is not None


def collect_values(conn, results, idle_s):
    """Read PARAM_EXT_VALUE messages into results[param_index] until every
    index 0..param_count-1 has arrived, or `idle_s` passes with no new one
    landing. Returns the param_count the drone reported, or None if
    nothing arrived."""
    count = None
    last = time.time()
    while True:
        remaining = idle_s - (time.time() - last)
        if remaining <= 0:
            break
        msg = recv_one(conn, "PARAM_EXT_VALUE", remaining)
        if msg is None:
            break
        last = time.time()
        count = msg.param_count
        type_name, value = decode_value(msg)
        results[msg.param_index] = (msg.param_id, type_name, value)
        if count is not None and all(i in results for i in range(count)):
            break
    return count


def cmd_list(conn):
    conn.mav.param_ext_request_list_send(DRONE_SYSTEM, DRONE_COMPONENT)
    results = {}
    count = collect_values(conn, results, idle_s=3.0)
    if count is None:
        print("no PARAM_EXT_VALUE received", file=sys.stderr)
        return 1
    for _ in range(3):
        missing = [i for i in range(count) if i not in results]
        if not missing:
            break
        for idx in missing:
            conn.mav.param_ext_request_read_send(DRONE_SYSTEM, DRONE_COMPONENT, b"", idx)
        collect_values(conn, results, idle_s=1.0)
    missing = [i for i in range(count) if i not in results]
    for i in sorted(results):
        pid, wtype_name, value = results[i]
        print("%-16s  %-6s  %s" % (pid, wtype_name, value))
    if missing:
        print("missing %d/%d parameters after retries: %s" % (len(missing), count, missing), file=sys.stderr)
        return 1
    return 0


def cmd_get(conn, pid):
    pid_b = pid.encode("ascii")
    msg = None
    for _ in range(4):  # 1 try + 3 retries
        conn.mav.param_ext_request_read_send(DRONE_SYSTEM, DRONE_COMPONENT, pid_b, -1)
        msg = recv_one(conn, "PARAM_EXT_VALUE", 1.0)
        if msg is not None:
            break
    if msg is None:
        print("no PARAM_EXT_VALUE for %s after 3 retries" % pid, file=sys.stderr)
        return 1
    type_name, value = decode_value(msg)
    print("%-16s  %-6s  %s" % (msg.param_id, type_name, value))
    return 0


def wait_final_ack(conn, pid, first_timeout=2.0, final_timeout=15.0, recv=None):
    """Wait for the drone's response to a PARAM_EXT_SET for `pid`.

    A set the drone applies answers IN_PROGRESS first (holding the
    current value) and then a final ACCEPTED or FAILED (also current
    value; FAILED is followed by a STATUSTEXT reason, read separately by
    the caller) -- normally within ~1 s, up to ~10 s on the fallback
    path. A set the drone refuses outright (unknown id, bad value)
    answers VALUE_UNSUPPORTED with no IN_PROGRESS at all.

    Waits up to `first_timeout` for the first ack for `pid`. If that ack
    is IN_PROGRESS, prints "IN_PROGRESS  <value>" and keeps waiting --
    still only for `pid`, ignoring acks for any other parameter, and
    tolerating further IN_PROGRESS acks on the fallback path -- for up
    to `final_timeout` measured from the IN_PROGRESS (not from this
    call) for a terminal one.

    Returns (result_name, final_msg): result_name is one of "ACCEPTED",
    "FAILED", "VALUE_UNSUPPORTED" or "TIMEOUT"; final_msg is the ack
    that produced it, or None on TIMEOUT.

    `recv` is a `timeout_s -> PARAM_EXT_ACK-like or None` callable; it
    defaults to a thin wrapper over recv_one(conn, "PARAM_EXT_ACK", t)
    so a test can inject a fake connection with no pymavlink installed.
    """
    if recv is None:
        recv = lambda timeout: recv_one(conn, "PARAM_EXT_ACK", timeout)

    def recv_for_pid(deadline):
        while True:
            remaining = deadline - time.time()
            if remaining <= 0:
                return None
            ack = recv(remaining)
            if ack is None:
                return None
            if ack.param_id != pid:
                continue  # a stray ack for some other parameter
            return ack

    ack = recv_for_pid(time.time() + first_timeout)
    if ack is None:
        return "TIMEOUT", None
    if ack.param_result != PARAM_ACK_IN_PROGRESS:
        return RESULT_NAMES.get(ack.param_result, "FAILED"), ack

    _, value = decode_value(ack)
    print("IN_PROGRESS  %s" % value)

    deadline = time.time() + final_timeout
    while True:
        ack = recv_for_pid(deadline)
        if ack is None:
            return "TIMEOUT", None
        if ack.param_result == PARAM_ACK_IN_PROGRESS:
            continue  # another placeholder on the fallback path; the
                      # final deadline does not restart
        return RESULT_NAMES.get(ack.param_result, "FAILED"), ack


def cmd_set(conn, pid, wtype, raw):
    """`wtype`/`raw` are already validated and encoded by main() before any
    network I/O (an unknown id or a value that fails to encode is a usage
    error, reported before a connection is even opened).

    Resends the PARAM_EXT_SET up to 3 times (1 try + 3 retries) while no
    ack for `pid` has arrived at all. The moment any ack for `pid`
    arrives -- including an IN_PROGRESS placeholder -- resending stops:
    the drone refuses a PARAM_EXT_SET for a parameter it is still
    applying ("apply in progress"), so wait_final_ack is left to wait
    out the final ack on its own instead of cmd_set resending into it.
    """
    pid_b = pid.encode("ascii")
    got_ack = [False]

    def recv(timeout):
        msg = recv_one(conn, "PARAM_EXT_ACK", timeout)
        if msg is not None and msg.param_id == pid:
            got_ack[0] = True
        return msg

    result, final = "TIMEOUT", None
    for _ in range(4):  # 1 try + 3 retries
        conn.mav.param_ext_set_send(DRONE_SYSTEM, DRONE_COMPONENT, pid_b, raw, MAV_TYPE[wtype])
        result, final = wait_final_ack(conn, pid, recv=recv)
        if result != "TIMEOUT" or got_ack[0]:
            break

    if result == "TIMEOUT":
        if got_ack[0]:
            print("no final PARAM_EXT_ACK for %s within 15 s of IN_PROGRESS" % pid, file=sys.stderr)
        else:
            print("no PARAM_EXT_ACK for %s after 3 retries" % pid, file=sys.stderr)
        return 1

    _, value = decode_value(final)
    print("%s  %s" % (result, value))
    status = recv_one(conn, "STATUSTEXT", 1.0)
    if status is not None:
        print(status.text)
    return 0 if result == "ACCEPTED" else 1


def main(argv):
    gs = os.environ.get("GS")
    port = os.environ.get("PORT")
    if not gs or not port:
        print(__doc__, file=sys.stderr)
        return 2
    try:
        port = int(port)
    except ValueError:
        print(__doc__, file=sys.stderr)
        return 2

    if len(argv) == 1 and argv[0] == "list":
        cmd, args = "list", []
    elif len(argv) == 2 and argv[0] == "get":
        cmd, args = "get", [argv[1]]
    elif len(argv) == 3 and argv[0] == "set":
        cmd, args = "set", [argv[1], argv[2]]
    else:
        print(__doc__, file=sys.stderr)
        return 2

    # `set` is validated and encoded here, before any network I/O: an
    # unknown id or a value that doesn't fit its type is a usage error
    # (int()/float() raise ValueError on unparseable text; struct.pack
    # raises struct.error or OverflowError on text that parses but
    # doesn't fit the wire type, e.g. an int32 out of range).
    set_wtype = set_raw = None
    if cmd == "set":
        pid, value = args
        set_wtype = TYPE_BY_ID.get(pid)
        if set_wtype is None:
            print("unknown parameter id: %s" % pid, file=sys.stderr)
            return 2
        try:
            set_raw = encode_value(set_wtype, value)
        except (ValueError, OverflowError, struct.error) as exc:
            print("bad value for %s: %s" % (pid, exc), file=sys.stderr)
            return 2

    # Deferred import (see the `mavutil = None` note above): the extended-
    # parameter messages are MAVLink 2 only, and pymavlink defaults to v1
    # unless told otherwise before the dialect is used.
    global mavutil
    os.environ["MAVLINK20"] = "1"
    from pymavlink import mavutil  # noqa: F811 (rebinds the module global)

    conn = mavutil.mavlink_connection("udpout:%s:%d" % (gs, port), source_system=255, source_component=190)
    conn.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0,
        0,
        mavutil.mavlink.MAV_STATE_ACTIVE,
    )
    if not wait_for_drone(conn):
        print("no HEARTBEAT from the drone (system 1, component 191) within 5 s", file=sys.stderr)
        return 1

    if cmd == "list":
        return cmd_list(conn)
    if cmd == "get":
        return cmd_get(conn, args[0])
    return cmd_set(conn, args[0], set_wtype, set_raw)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
