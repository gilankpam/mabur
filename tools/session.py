#!/usr/bin/env python3
"""Resolve a debug-log session to the files inside it.

One session is one directory: /media/dvr/log/NNNN/{ctl,probe,au,lat}.log plus
flight.jsonl, all on one CLOCK_MONOTONIC clock (docs/observability.md). This
module is the single place that knows that layout, so flightreport.py,
flightjitter.py, airdrain.py and probesend.py do not each carry their own
path logic.

Accepts, in order of preference:
  * a session directory          -> every file inside it
  * a single file path (legacy)  -> that one file, classified by its FORMAT
                                    MARKER, not its name: recordings from
                                    before the consolidation are named
                                    ctl-NNNN_<date>.log / au-NNNN.log and must
                                    keep working (CLAUDE.md: recordings
                                    outlive the code that wrote them)
  * nothing                      -> the highest-numbered session under `root`
"""
import collections
import os
import re

DEFAULT_ROOT = "/media/dvr/log"

Session = collections.namedtuple("Session", "dir ctl probe au flight lat")

_FILES = {"ctl": "ctl.log", "probe": "probe.log", "au": "au.log",
          "flight": "flight.jsonl", "lat": "lat.log"}
_SESSION_DIR = re.compile(r"^\d{4}$")


def latest(root=DEFAULT_ROOT):
    """Highest-numbered session directory under `root`, or None.

    Highest INDEX, never newest mtime: a restored or copied DVR carries
    mtimes that lie, which has already cost one analysis pass.
    """
    try:
        names = os.listdir(root)
    except OSError:
        return None
    idx = [n for n in names
           if _SESSION_DIR.match(n) and os.path.isdir(os.path.join(root, n))]
    if not idx:
        return None
    return os.path.join(root, max(idx))


def _classify(path):
    """Which slot a legacy single file belongs in, by its marker line."""
    try:
        with open(path, "rb") as f:
            first = f.readline().decode("utf-8", "replace")
    except OSError:
        return None
    if first.startswith("ctllog "):
        return "ctl"
    if first.startswith("probelog "):
        return "probe"
    if first.startswith("# aulog ") or first.startswith("# latlog "):
        return "au" if "aulog" in first else "lat"
    if first.lstrip().startswith("{"):
        return "flight"
    # A v1 au log has no marker at all; fall back to the name.
    base = os.path.basename(path)
    for key, prefix in (("au", "au-"), ("lat", "lat-"), ("ctl", "ctl-"),
                        ("probe", "probe-"), ("flight", "flight-")):
        if base.startswith(prefix):
            return key
    return None


def resolve(arg=None, root=DEFAULT_ROOT):
    """Session for `arg` (a directory, a file, or None -> latest)."""
    if arg is None:
        arg = latest(root)
        if arg is None:
            return Session(None, None, None, None, None, None)
    if os.path.isdir(arg):
        found = {}
        for key, name in _FILES.items():
            p = os.path.join(arg, name)
            found[key] = p if os.path.exists(p) else None
        return Session(arg, found["ctl"], found["probe"], found["au"],
                       found["flight"], found["lat"])
    slot = _classify(arg)
    fields = {k: None for k in _FILES}
    if slot:
        fields[slot] = arg
    return Session(None, fields["ctl"], fields["probe"], fields["au"],
                   fields["flight"], fields["lat"])
