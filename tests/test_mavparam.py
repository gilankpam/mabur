#!/usr/bin/env python3
"""Test for tools/bench/mavparam.py's PARAM_EXT_ACK wait: a set the drone
applies answers IN_PROGRESS first (current value), then a final ACCEPTED or
FAILED (docs/vtx-config-mavlink.md). Covers wait_final_ack()'s own two-phase receive
(IN_PROGRESS -> final, acks for another param_id ignored, a first-phase
timeout) and cmd_set()'s resend policy (retries only while no ack has
arrived at all; never resends once IN_PROGRESS is seen).

The fakes below need no pymavlink: an ack stand-in is a plain object with
param_id/param_result/param_type/_param_value_raw, and FakeConn/FakeMav
imitate just the pieces of a pymavlink connection mavparam.py touches
(recv_match, conn.mav.param_ext_set_send).
"""
import os
import struct
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                 '..', 'tools', 'bench'))
import mavparam  # noqa: E402

ACCEPTED = mavparam.PARAM_ACK_ACCEPTED
VALUE_UNSUPPORTED = mavparam.PARAM_ACK_VALUE_UNSUPPORTED
FAILED = mavparam.PARAM_ACK_FAILED
IN_PROGRESS = mavparam.PARAM_ACK_IN_PROGRESS


class FakeAck:
    """An IN_PROGRESS/ACCEPTED/FAILED/VALUE_UNSUPPORTED PARAM_EXT_ACK stand-
    in, decodable by mavparam.decode_value() (Int32) and pre-filtered by
    recv_one() as if it came from the drone's component (1, 191)."""

    def __init__(self, param_id, result, value=0):
        self.param_id = param_id
        self.param_result = result
        self.param_type = mavparam.MAV_TYPE["i"]
        self._param_value_raw = struct.pack("<i", value)

    def get_srcSystem(self):
        return mavparam.DRONE_SYSTEM

    def get_srcComponent(self):
        return mavparam.DRONE_COMPONENT


def fake_ack(param_id, result, value=0):
    return FakeAck(param_id, result, value)


class FakeStatus:
    """A STATUSTEXT stand-in: just the `.text` field cmd_set() prints, with
    the same (1, 191) source stand-in recv_one() filters on."""

    def __init__(self, text):
        self.text = text

    def get_srcSystem(self):
        return mavparam.DRONE_SYSTEM

    def get_srcComponent(self):
        return mavparam.DRONE_COMPONENT


class FakeMav:
    def __init__(self):
        self.sets = []

    def param_ext_set_send(self, system, component, param_id, raw, mtype):
        self.sets.append(param_id)


class FakeConn:
    """A PARAM_EXT_ACK queue behind recv_match(), and (for cmd_set tests) a
    FakeMav counting PARAM_EXT_SET sends. `then_none=True` means the queue
    answers None forever once drained (simulating a real timeout); the
    default raises if drained unexpectedly, so a test that over-consumes
    acks fails loudly instead of silently passing."""

    def __init__(self, acks, then_none=False, statuses=None):
        self.mav = FakeMav()
        self._acks = list(acks)
        self._then_none = then_none
        self._statuses = list(statuses or [])

    def recv_match(self, type=None, blocking=True, timeout=None):
        if type == "STATUSTEXT":
            return self._statuses.pop(0) if self._statuses else None
        assert type == "PARAM_EXT_ACK", type
        if self._acks:
            return self._acks.pop(0)
        if self._then_none:
            return None
        raise AssertionError("FakeConn's ack queue ran out unexpectedly")


class WaitFinalAckTest(unittest.TestCase):
    def test_in_progress_then_accepted(self):
        acks = [fake_ack("REC_FPS", IN_PROGRESS), fake_ack("REC_FPS", ACCEPTED)]
        result, final = mavparam.wait_final_ack(FakeConn(acks), "REC_FPS")
        self.assertEqual(result, "ACCEPTED")
        self.assertIs(final, acks[1])

    def test_in_progress_then_timeout_is_failure(self):
        acks = [fake_ack("REC_FPS", IN_PROGRESS)]
        conn = FakeConn(acks, then_none=True)
        result, final = mavparam.wait_final_ack(conn, "REC_FPS", final_timeout=0.01)
        self.assertEqual(result, "TIMEOUT")
        self.assertIsNone(final)

    def test_in_progress_then_failed_keeps_statustext(self):
        # wait_final_ack only ever receives PARAM_EXT_ACK: a STATUSTEXT in
        # the queue is untouched for cmd_set to read afterwards.
        acks = [fake_ack("REC_FPS", IN_PROGRESS), fake_ack("REC_FPS", FAILED)]
        status = FakeStatus("bad value")
        conn = FakeConn(acks, statuses=[status])
        result, final = mavparam.wait_final_ack(conn, "REC_FPS")
        self.assertEqual(result, "FAILED")
        self.assertIs(final, acks[1])
        self.assertEqual(conn._statuses, [status])

    def test_immediate_value_unsupported_has_no_in_progress(self):
        acks = [fake_ack("VENC_MODE", VALUE_UNSUPPORTED)]
        result, final = mavparam.wait_final_ack(FakeConn(acks), "VENC_MODE")
        self.assertEqual(result, "VALUE_UNSUPPORTED")
        self.assertIs(final, acks[0])

    def test_first_ack_timeout_before_anything_arrives(self):
        conn = FakeConn([], then_none=True)
        result, final = mavparam.wait_final_ack(conn, "REC_FPS", first_timeout=0.01)
        self.assertEqual(result, "TIMEOUT")
        self.assertIsNone(final)

    def test_ignores_ack_for_other_param_id(self):
        acks = [fake_ack("OTHER_ID", ACCEPTED), fake_ack("REC_FPS", ACCEPTED)]
        result, final = mavparam.wait_final_ack(FakeConn(acks), "REC_FPS")
        self.assertEqual(result, "ACCEPTED")
        self.assertIs(final, acks[1])

    def test_ignores_stray_id_between_in_progress_and_final(self):
        acks = [fake_ack("REC_FPS", IN_PROGRESS), fake_ack("OTHER_ID", ACCEPTED),
                fake_ack("REC_FPS", ACCEPTED)]
        result, final = mavparam.wait_final_ack(FakeConn(acks), "REC_FPS")
        self.assertEqual(result, "ACCEPTED")
        self.assertIs(final, acks[2])

    def test_second_in_progress_does_not_restart_as_final(self):
        acks = [fake_ack("REC_FPS", IN_PROGRESS), fake_ack("REC_FPS", IN_PROGRESS),
                fake_ack("REC_FPS", ACCEPTED)]
        result, final = mavparam.wait_final_ack(FakeConn(acks), "REC_FPS")
        self.assertEqual(result, "ACCEPTED")
        self.assertIs(final, acks[2])


class CmdSetTest(unittest.TestCase):
    def _raw(self):
        return mavparam.encode_value("i", "5")

    def test_in_progress_then_accepted_sends_once_and_exits_zero(self):
        acks = [fake_ack("REC_FPS", IN_PROGRESS), fake_ack("REC_FPS", ACCEPTED)]
        conn = FakeConn(acks)
        rc = mavparam.cmd_set(conn, "REC_FPS", "i", self._raw())
        self.assertEqual(rc, 0)
        self.assertEqual(len(conn.mav.sets), 1)

    def test_in_progress_then_failed_exits_one_without_resend(self):
        acks = [fake_ack("REC_FPS", IN_PROGRESS), fake_ack("REC_FPS", FAILED)]
        conn = FakeConn(acks, statuses=[FakeStatus("reason: out of range")])
        rc = mavparam.cmd_set(conn, "REC_FPS", "i", self._raw())
        self.assertEqual(rc, 1)
        self.assertEqual(len(conn.mav.sets), 1)

    def test_no_ack_at_all_retries_three_times(self):
        conn = FakeConn([], then_none=True)
        rc = mavparam.cmd_set(conn, "REC_FPS", "i", self._raw())
        self.assertEqual(rc, 1)
        self.assertEqual(len(conn.mav.sets), 4)  # 1 try + 3 retries

    def test_in_progress_then_no_final_ack_does_not_resend(self):
        # The fallback path's final ack never lands (the FakeConn queue
        # answers None after the single IN_PROGRESS): cmd_set must stop
        # waiting and report failure WITHOUT sending another
        # PARAM_EXT_SET, since the drone would refuse one mid-apply.
        acks = [fake_ack("REC_FPS", IN_PROGRESS)]
        conn = FakeConn(acks, then_none=True)
        rc = mavparam.cmd_set(conn, "REC_FPS", "i", self._raw())
        self.assertEqual(rc, 1)
        self.assertEqual(len(conn.mav.sets), 1)


if __name__ == "__main__":
    unittest.main()
