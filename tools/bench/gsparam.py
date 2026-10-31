#!/usr/bin/env python3
"""gsparam.py -- get/set the GS's live-config controls: maburgs's loopback
listener on 127.0.0.1:8402 and maburplay's on 127.0.0.1:8403. One UDP
datagram in, one reply out: `set <key> = <toml value>` answers `ok` |
`err <reason>` | `err busy` | `err not live: <key>`; `get <key>` answers
`ok <value>`; `get` with no key answers `ok` then one `<key> = <value>`
line per live key, in table order (gs/src/gs_config_apply.cpp's
kGsLiveKeys, gs/player/src/player_config_apply.cpp's kPlayerLiveKeys).

Usage:
  GS=<gs-ip> python3 tools/bench/gsparam.py gs     get [KEY]
  GS=<gs-ip> python3 tools/bench/gsparam.py gs     set KEY VALUE
  GS=<gs-ip> python3 tools/bench/gsparam.py player get [KEY]
  GS=<gs-ip> python3 tools/bench/gsparam.py player set KEY VALUE
  GS=<gs-ip> python3 tools/bench/gsparam.py --all gs
  GS=<gs-ip> python3 tools/bench/gsparam.py --all player --alt-mode MODE [--opi]

GS (the ground station's ssh address) comes from the environment, with no
default. Every exchange runs over `ssh root@$GS`, which executes a short
`python3 -c` UDP client against the loopback listener (busybox + python3 on
the GS): sends one datagram, waits up to 5 s for the one-datagram reply.

`--all` drives every live key of the named daemon away from its bundle
default and back, checking along the way: the `set` reply is `ok`, a
follow-up `get` reads the value back, and the daemon's TOML on the GS
(/etc/maburgs.toml; /etc/maburplay.toml, a symlink onto vfat /config --
grep follows it) shows the written value. `link.ladder`'s file shape is a
run of `[[link.ladder]]` blocks rather than one line, so it is checked by
block count instead. `screen_mode` has no single alternate value -- which
modes a connector offers depends on the display -- so `--all player` needs
`--alt-mode MODE` naming the one to swap to and back from. `--opi` marks
the Orange Pi build, which has no MPP encoder: `dvr.mode`'s restore to
"burned" is expected to come back `err` (unsupported_on_build) rather than
`ok`, and is not checked against the file.

Exit: 0 all requested checks (or the one get/set) passed; 1 a runtime
failure -- no reply within 5 s, a `set`/`get` mismatch, or an `--all`
check that didn't hold; 2 a usage error.
"""
import os
import re
import shlex
import subprocess
import sys

ROLE_PORT = {"gs": 8402, "player": 8403}
ROLE_FILE = {"gs": "/etc/maburgs.toml", "player": "/etc/maburplay.toml"}

# The UDP client run on the GS over ssh: one datagram out, one datagram (up
# to 64 KiB, enough for a `get` with no key) back within 5 s.
REMOTE_PY = (
    "import socket,sys\n"
    "s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)\n"
    "s.settimeout(5)\n"
    "s.sendto(sys.argv[1].encode(),('127.0.0.1',int(sys.argv[2])))\n"
    "try:\n"
    "    data,_=s.recvfrom(65536)\n"
    "except socket.timeout:\n"
    "    sys.exit(1)\n"
    "sys.stdout.write(data.decode())\n"
)

# The bundle ladder (gs/bundle/maburgs.default.toml's [[link.ladder]] rungs)
# and channel set (its [radio] channels), kept here as the "back" values
# for --all gs; the loader's own render_live_value() is the source of truth
# this is checked against (docs/data-provenance.md's duty to track defaults
# does not cover this file, so it is kept in sync by hand).
BUNDLE_LADDER = [
    (0, 40, 0.5, 0.25),
    (1, 40, 0.5, 0.25),
    (2, 40, 0.5, 0.25),
    (3, 40, 0.5, 0.25),
    (4, 40, 0.5, 0.25),
]
BUNDLE_CHANNELS = [40, 64, 112, 144]


def build_line(cmd, key=None, value=None):
    """The wire line for a `get [KEY]` or `set KEY = VALUE`."""
    if cmd == "get":
        return "get %s" % key if key else "get"
    if cmd == "set":
        return "set %s = %s" % (key, value)
    raise ValueError("unknown command: %r" % cmd)


def parse_reply(line):
    """A single reply line -> (status, rest): "ok" -> ("ok", ""), "ok -1"
    -> ("ok", "-1"), "err busy" -> ("err", "busy"), "err not live: KEY" ->
    ("err", "not live: KEY") -- only the first space is a separator, so a
    multi-word reason stays intact in `rest`."""
    line = line.rstrip("\n")
    parts = line.split(" ", 1)
    return parts[0], (parts[1] if len(parts) > 1 else "")


def render_real(v):
    """The shortest decimal text for `v` that still reads as a float."""
    s = "%g" % v
    if "." not in s and "e" not in s and "n" not in s:
        s += ".0"
    return s


def render_ladder(rungs):
    """(mcs, bw, overhead_base, overhead_enh) tuples -> the inline-table
    array text gs_config_apply.cpp's render_live_value() sends back for
    `get link.ladder`, so a value sent on a `set` round-trips byte-for-byte
    through a `get`."""
    parts = ["{mcs = %d, bw = %d, overhead_base = %s, overhead_enh = %s}" %
             (mcs, bw, render_real(ob), render_real(oe))
             for mcs, bw, ob, oe in rungs]
    return "[" + ", ".join(parts) + "]"


def expected_pairs(role, alt_mode=None):
    """The --all table for `role`: (key, away_value, back_value) in the
    daemon's live-key table order, both values as the exact TOML-value text
    a `set` sends on the wire. `away` is the bundle default moved somewhere
    else; `back` is the bundle default itself, restoring it.

    `alt_mode` is screen_mode's away value's bare mode text (player only);
    with no `alt_mode`, screen_mode's entry carries `away=None` -- the
    caller must supply one (via --alt-mode) before running --all player.
    """
    if role == "gs":
        full_ladder = render_ladder(BUNDLE_LADDER)
        minus_last = render_ladder(BUNDLE_LADDER[:-1])
        return [
            ("radio.channel", str(BUNDLE_CHANNELS[1]), '"auto"'),
            ("link.static_mcs", "3", "-1"),
            ("link.static_bw", "40", "20"),
            ("link.static_overhead_base", "0.6", "0.5"),
            ("link.static_overhead_enh", "0.25", "0.5"),
            ("link.ladder", minus_last, full_ladder),
            ("link.nack.enable", "false", "true"),
            ("debug_log.enable", "true", "false"),
            ("msp.enable", "false", "true"),
        ]
    if role == "player":
        away_mode = ('"%s"' % alt_mode) if alt_mode else None
        return [
            ("screen_mode", away_mode, '"1920x1080@60"'),
            ("colortrans.enable", "false", "true"),
            ("osd.enable", "false", "true"),
            ("osd.gs.enable", "false", "true"),
            ("dvr.target", '"both"', '"gs"'),
            ("dvr.mode", '"raw"', '"burned"'),
            ("dvr.burned.bitrate_kbps", "6000", "8000"),
            ("dvr.burned.every", "2", "1"),
            ("dvr.burned.size", '"1280x720"', '""'),
        ]
    raise ValueError("unknown role: %r" % role)


def send_recv(gs, port, line, timeout=10.0):
    """Runs the UDP exchange on the GS over ssh; returns the reply text, or
    None on a timeout, a dropped connection, or no reply within 5 s."""
    remote = "python3 -c %s %s %s" % (shlex.quote(REMOTE_PY), shlex.quote(line), port)
    try:
        r = subprocess.run(["ssh", "-o", "ConnectTimeout=5", "root@%s" % gs, remote],
                            capture_output=True, timeout=timeout, text=True)
    except subprocess.TimeoutExpired:
        return None
    if r.returncode != 0:
        return None
    return r.stdout


def remote_shell(gs, cmd, timeout=10.0):
    """An arbitrary read-only shell command on the GS over ssh; returns its
    stdout, or None if ssh itself failed (a command that legitimately found
    nothing, like `grep` with no match, still returns its, empty, stdout)."""
    try:
        r = subprocess.run(["ssh", "-o", "ConnectTimeout=5", "root@%s" % gs, cmd],
                            capture_output=True, timeout=timeout, text=True)
    except subprocess.TimeoutExpired:
        return None
    if r.returncode not in (0, 1):  # grep: 0 = match, 1 = no match, 2+ = real error
        return None
    return r.stdout


def first_line(text):
    lines = text.splitlines() if text else []
    return lines[0] if lines else ""


def cmd_get(gs, role, key):
    reply = send_recv(gs, ROLE_PORT[role], build_line("get", key))
    if reply is None:
        print("no reply from %s on %d within 5 s" % (gs, ROLE_PORT[role]), file=sys.stderr)
        return 1
    status, _ = parse_reply(first_line(reply))
    sys.stdout.write(reply if reply.endswith("\n") else reply + "\n")
    return 0 if status == "ok" else 1


def cmd_set(gs, role, key, value):
    reply = send_recv(gs, ROLE_PORT[role], build_line("set", key, value))
    if reply is None:
        print("no reply from %s on %d within 5 s" % (gs, ROLE_PORT[role]), file=sys.stderr)
        return 1
    status, rest = parse_reply(first_line(reply))
    print("%s  %s" % (status, rest))
    return 0 if status == "ok" else 1


def file_matches(gs, role, key, value):
    """Whether `path` on the GS (grepped over ssh) shows `key` holding
    `value`. link.ladder has no single line to grep -- it is a run of
    `[[link.ladder]]` blocks -- so it is checked by block count (one
    "{" in `value` per rung) instead."""
    path = ROLE_FILE[role]
    if key == "link.ladder":
        want = value.count("{")
        out = remote_shell(gs, "grep -cF '[[link.ladder]]' %s" % path)
        if out is None:
            return False
        try:
            return int(out.strip()) == want
        except ValueError:
            return False
    leaf = key.rsplit(".", 1)[-1]
    out = remote_shell(gs, "grep -n '^%s' %s" % (leaf, path))
    if out is None:
        return False
    pattern = re.compile(r"^\d+:%s\s*=\s*%s\s*(#.*)?$" % (re.escape(leaf), re.escape(value)))
    return any(pattern.match(line) for line in out.splitlines())


def step(gs, role, key, value, expect_status):
    """One `set` of `--all`'s away/back cycle: send it, check the reply
    status, and -- only when that status is "ok" -- read the value back
    with `get` and grep it off the file. Returns whether every check that
    applied held."""
    port = ROLE_PORT[role]
    line = build_line("set", key, value)
    reply = send_recv(gs, port, line)
    if reply is None:
        print("MISMATCH %s: no reply within 5 s" % line)
        return False
    status, rest = parse_reply(first_line(reply))
    print("%s -> %s %s" % (line, status, rest))
    if status != expect_status:
        print("MISMATCH %s: got %r, want %r" % (line, status, expect_status))
        return False
    if status != "ok":
        return True  # an expected refusal leaves the file untouched; nothing else to check

    get_reply = send_recv(gs, port, build_line("get", key))
    if get_reply is None:
        print("MISMATCH get %s: no reply within 5 s" % key)
        return False
    gstatus, gvalue = parse_reply(first_line(get_reply))
    if gstatus != "ok" or gvalue != value:
        print("MISMATCH get %s -> %r, want %r" % (key, gvalue, value))
        return False

    if not file_matches(gs, role, key, value):
        print("MISMATCH file check for %s = %s" % (key, value))
        return False
    return True


def cmd_all(gs, role, alt_mode, opi):
    ok = True
    for key, away, back in expected_pairs(role, alt_mode):
        expect_back = "err" if (key == "dvr.mode" and opi) else "ok"
        if not step(gs, role, key, away, "ok"):
            ok = False
        if not step(gs, role, key, back, expect_back):
            ok = False
    print("ALL OK" if ok else "ALL FAILED")
    return 0 if ok else 1


def main(argv):
    gs = os.environ.get("GS")
    if not gs:
        print(__doc__, file=sys.stderr)
        return 2

    if argv[:1] == ["--all"]:
        if len(argv) < 2 or argv[1] not in ("gs", "player"):
            print(__doc__, file=sys.stderr)
            return 2
        role = argv[1]
        alt_mode, opi = None, False
        i = 2
        while i < len(argv):
            if argv[i] == "--alt-mode" and i + 1 < len(argv):
                alt_mode = argv[i + 1]
                i += 2
            elif argv[i] == "--opi":
                opi = True
                i += 1
            else:
                print(__doc__, file=sys.stderr)
                return 2
        if role == "player" and alt_mode is None:
            print("--all player needs --alt-mode MODE: screen_mode has no "
                  "single alternate value to swap to", file=sys.stderr)
            return 2
        return cmd_all(gs, role, alt_mode, opi)

    if len(argv) >= 2 and argv[0] in ("gs", "player"):
        role, cmd, rest = argv[0], argv[1], argv[2:]
        if cmd == "get" and len(rest) <= 1:
            return cmd_get(gs, role, rest[0] if rest else None)
        if cmd == "set" and len(rest) == 2:
            return cmd_set(gs, role, rest[0], rest[1])

    print(__doc__, file=sys.stderr)
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
