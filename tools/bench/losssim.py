#!/usr/bin/env python3
"""Bench driver for maburgs' injected per-stream loss (--loss-sim).

Mainlined (2026-08-29) behind the CMake option MABUR_LOSS_SIM (default OFF).
Requires maburgs built with -DMABUR_LOSS_SIM=ON; the loopback control socket
does not exist otherwise.

Run on the GS, where maburgs' control socket is bound to loopback.

  losssim.py status
  losssim.py s3 eff=8 burst=3      # 8% union loss at the decoder
  losssim.py s3 loss=8 burst=3     # 8% PER CARD (~0.6% union on 2 cards)
  losssim.py off
  losssim.py --sweep 0,2,5,10,20,35 --dwell 25 --burst 3 --stream 3

TWO RATES. maburgs injects loss independently per receive card, because real
fading is per-card and the multi-card union is what the diversity exists to
exploit. A body only reaches the FEC decoder as lost when EVERY card dropped
it, so with N cards the loss under test is roughly percard**N: on the 2-card
bench GS, dialling 20% per card delivers ~4%. `--sweep` steps are therefore
EFFECTIVE (union) percentages by default and are sent as `eff=`; pass
--percard to dial the per-card rate directly instead. Every reply states both
numbers (`percard=` and `eff=`).

`eff` is NOMINAL: it assumes every card heard every body, which only holds on
a clean link. On a link already losing bodies the true injected loss is
HIGHER. So the dial is for dialling — the loss written into the findings
document must be read from the stats sideport's per-stream s3 counters.

The sweep prints one line per step, stamped with BOTH a human-readable clock
time and a monotonic millisecond counter (`mono=`). The monotonic one is the
important one: the recorded telemetry JSONL carries only `t_ms`, which is a
steady_clock reading, and Python's time.monotonic() and libstdc++'s
steady_clock are both CLOCK_MONOTONIC on Linux — so `mono=` is directly
comparable to `t_ms` in the recording, with no boot-offset arithmetic against
a ground station whose RTC is wrong at boot. Keep this output: it is the ONLY
record of which loss level was in force when.

The sweep's first log line is the reply to a `status` sent before any step, so
the starting state is recorded rather than assumed — a leftover `s1 loss=10`
from a contrast case would otherwise silently contaminate the baseline.

A step whose reply does not start with "ok" (a rejected command, a no-reply
timeout, or a socket error) is a FAILED step, not a successful one at some
other loss level. Such a step is printed with a "FAILED " marker at the START
of the line, and the sweep ABORTS immediately rather than continuing — a
shortened sweep the operator can rerun is strictly better than a full-length
recording with one mislabelled stretch. The link is still restored to zero and
a final status printed either way, but the process exits non-zero so a
short/aborted sweep is never mistaken for a clean, complete one.
"""
import argparse
import socket
import sys
import time

DEFAULT_PORT = 8302


def is_ok(reply):
    """True iff `reply` is a genuine "ok ..." response from maburgs, as
    opposed to an "err ...", a no-reply timeout, or a socket-error string —
    all of which mean the requested loss level was NOT actually applied."""
    return isinstance(reply, str) and reply.startswith("ok")


def send(sock, addr, cmd, timeout=1.0):
    try:
        sock.sendto(cmd.encode(), addr)
        sock.settimeout(timeout)
        return sock.recv(1024).decode().strip()
    except socket.timeout:
        return "<no reply — is maburgs running with --loss-sim?>"
    except OSError as e:
        # e.g. ConnectionRefusedError from a prior ICMP port-unreachable when
        # maburgs has died or restarted mid-sweep. Never let this escape as a
        # traceback — return a string that is obviously not "ok ..." so the
        # sweep loop's is_ok() classifies it as a failed step.
        return f"<send failed: {e} — is maburgs running with --loss-sim?>"


def stamp():
    """Clock time for a human, monotonic ms for the analysis.

    time.monotonic() is CLOCK_MONOTONIC, the same clock libstdc++'s
    steady_clock reads, so `mono=` lines up directly with the `t_ms` field in
    a recorded stats-sideport JSONL. The wall clock is kept only because an
    operator watching the run needs something they recognise; it must never be
    what a step is aligned by (the GS RTC is wrong at boot)."""
    return f"mono={int(time.monotonic() * 1000)} {time.strftime('%H:%M:%S')}"


def main():
    ap = argparse.ArgumentParser(
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description=__doc__)
    ap.add_argument("cmd", nargs="*", help="command to send, e.g. s3 eff=8 burst=3")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--sweep",
                    help="comma-separated loss percentages; EFFECTIVE (union) "
                         "loss by default, per-card with --percard")
    ap.add_argument("--percard", action="store_true",
                    help="dial --sweep steps as PER-CARD loss instead of "
                         "effective union loss. On N cards the decoder then "
                         "sees roughly step**N, which is NOT the number to "
                         "write down as the loss under test.")
    ap.add_argument("--dwell", type=float, default=25.0, help="seconds per step")
    ap.add_argument("--burst", type=float, default=1.0, help="mean burst length")
    ap.add_argument("--stream", type=int, default=3,
                    help="stream id 0..5 (1 = enh, 5 = probe)")
    a = ap.parse_args()

    addr = ("127.0.0.1", a.port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    if a.sweep:
        try:
            steps = [float(x) for x in a.sweep.split(",")]
        except ValueError:
            print("error: --sweep wants comma-separated numbers", file=sys.stderr)
            return 2
        # Which number the steps ARE has to be stated where it cannot be
        # missed: the same "20" means ~4% at the decoder in one mode and 20%
        # in the other, and the log is the only surviving record.
        key = "loss" if a.percard else "eff"
        mode = ("mode=PER-CARD key=loss -- each card injects the step; the "
                "decoder sees roughly step**ncards, which is NOT the loss "
                "under test" if a.percard else
                "mode=EFFECTIVE key=eff -- the step is the union loss at the "
                "decoder (NOMINAL; record real loss from the stats sideport, "
                "not from this dial)")
        print(f"# sweep s{a.stream} {mode}", flush=True)
        print(f"# sweep s{a.stream} burst={a.burst} dwell={a.dwell}s "
              f"steps={steps}", flush=True)
        print(f"# mono= is CLOCK_MONOTONIC ms, directly comparable to t_ms "
              f"in the recorded stats JSONL", flush=True)
        sweep_failed = False
        restore_ok = True
        try:
            # FINDING 3: a sweep must record the state it started from. `sN
            # off` only zeroes the swept stream, so a leftover setting on
            # another stream would otherwise ride through the whole run and
            # quietly contaminate the baseline everything is measured against.
            print(f"{stamp()} start  -> {send(sock, addr, 'status')}",
                  flush=True)
            for pct in steps:
                cmd = (f"s{a.stream} off" if pct <= 0
                       else f"s{a.stream} {key}={pct} burst={a.burst}")
                reply = send(sock, addr, cmd)
                ok = is_ok(reply)
                # Marker goes at the START of the line: a failed step must be
                # unmistakable at a glance and grep -v FAILED-able, never
                # confusable with a normal successful step.
                marker = "" if ok else "FAILED "
                print(f"{marker}{stamp()} "
                      f"{key}={pct:>5.1f}% -> {reply}", flush=True)
                if not ok:
                    print(f"!!! step failed at {key}={pct:.1f}% — aborting "
                          f"sweep now. This stretch of any recording must "
                          f"NOT be attributed to {key}={pct:.1f}%.", flush=True)
                    sweep_failed = True
                    break
                time.sleep(a.dwell)
        except KeyboardInterrupt:
            print("\n# interrupted", flush=True)
        finally:
            # Never leave the link degraded because the operator hit Ctrl-C
            # (or a failed step aborted the sweep) — and never let restoring
            # itself raise, e.g. on a second Ctrl-C or a daemon that died
            # mid-sweep (ConnectionRefusedError). Guard each call so both
            # lines below always print.
            try:
                off_reply = send(sock, addr, "off")
            except BaseException as e:
                off_reply = f"<restore exception: {e!r}>"
            print(f"{stamp()} restore -> {off_reply}", flush=True)
            if not is_ok(off_reply):
                restore_ok = False

            try:
                final_status = send(sock, addr, "status")
            except BaseException as e:
                final_status = f"<status exception: {e!r}>"
            print(f"{stamp()} final  -> {final_status}", flush=True)
            if not is_ok(final_status):
                restore_ok = False

            if not restore_ok:
                print("!!! WARNING: restore could NOT be confirmed — the "
                      "link may still be degraded. Verify manually with "
                      "`losssim.py status` and re-run `losssim.py off` if "
                      "it is not clean.", flush=True)
        return 1 if (sweep_failed or not restore_ok) else 0

    if not a.cmd:
        print(send(sock, addr, "status"), flush=True)
        return 0
    print(send(sock, addr, " ".join(a.cmd)), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
