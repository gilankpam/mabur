#!/usr/bin/env python3
"""webgs live --relay against a fake mabur-relay v3 (UDP). argv[1] = webgs binary."""
import socket, struct, subprocess, sys, threading, time, unittest, json

WEBGS = sys.argv.pop(1) if len(sys.argv) > 1 else 'build/web/webgs'
FRAME, HELLO, TUNE, STATUS, TX = 1, 2, 3, 4, 5

def hdr(t): return struct.pack('<HBB', 0x524D, 3, t)
def status(state, ch, sec, you_own, tune_id=0):
    return hdr(STATUS) + struct.pack('<HBBBBB', tune_id, state, ch, sec, 1, you_own) + b'\0' * 36
def qos_frame(seq, dot_seq):
    d = bytes([0x88, 0, 0, 0]) + b'\xff' * 6 + bytes([0x57, 0x42, 0x75, 0x05, 0xd6, 0x00]) * 2
    d += struct.pack('<H', dot_seq << 4) + b'\0\0' + bytes(range(40))
    return hdr(FRAME) + struct.pack('<IBBBBbbbbI', seq, 136, 2, 0x04, 4, -40, -42, -95, -95, seq) + d

class FakeRelay:
    def __init__(self, own=True, answer=True, tune_fail=False):
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s.bind(('127.0.0.1', 0)); self.port = self.s.getsockname()[1]
        self.own, self.answer, self.stop = own, answer, False
        self.tune_fail = tune_fail
        self.got, self.peer, self.tuned = [], None, None
        threading.Thread(target=self.run, daemon=True).start()
    def run(self):
        self.s.settimeout(0.05); seq = 0
        while not self.stop:
            try:
                b, a = self.s.recvfrom(4096); self.peer = a; self.got.append(b)
                t = b[3]
                if t == TUNE: self.tuned = (b[6], b[7])
                if self.answer and t in (HELLO, TUNE):
                    if self.tune_fail:
                        # State 2 (mid-retune/refused) on a channel other
                        # than what was requested: we own the relay but it
                        # never reaches our channel/sec.
                        self.s.sendto(status(2, 100, 0, 1), a)
                    else:
                        ch, sec = self.tuned or (0, 0)
                        self.s.sendto(status(0 if self.own else 3, ch, sec, 1 if self.own else 0), a)
            except socket.timeout: pass
            if self.peer and self.answer and self.own:
                for _ in range(20):
                    self.s.sendto(qos_frame(seq, seq & 0xFFF), self.peer); seq += 1
    def types(self): return [b[3] for b in self.got]

def run_webgs(port, mode, secs):
    return subprocess.run([WEBGS, 'live', '--relay', f'127.0.0.1:{port}', '--mode', mode,
                           '--ch', '136', '--w', '40', '--secs', str(secs)],
                          capture_output=True, text=True, timeout=30)

def stats(out):
    return [json.loads(l[6:]) for l in out.splitlines() if l.startswith('STATS ')]

class RelayE2E(unittest.TestCase):
    def test_gs_owned_tunes_receives_and_transmits(self):
        r = FakeRelay()
        p = run_webgs(r.port, 'gs', 3); r.stop = True
        self.assertEqual(p.returncode, 0, p.stdout + p.stderr)
        self.assertEqual(r.tuned, (136, 2))                      # 136 HT40-: sec 2
        self.assertIn(HELLO, r.types()); self.assertIn(TX, r.types())
        tx = [b for b in r.got if b[3] == TX][0]
        self.assertEqual((tx[4], tx[5]), (0, 0x03))              # MCS0, LDPC|STBC
        self.assertEqual(tx[6], 0x40)                            # probe-req control frame
        self.assertEqual(tx[16:22], bytes([0x57, 0x42, 0x75, 0x05, 0xd6, 0x00]))
        st = stats(p.stdout)[-1]
        self.assertEqual(st['radio'], 'relay'); self.assertEqual(st['relay_owned'], 1)
        self.assertGreater(st['bodies'], 100); self.assertEqual(st['relay_gaps'], 0)

    def test_gs_refused_when_not_owner(self):
        r = FakeRelay(own=False)
        t0 = time.time(); p = run_webgs(r.port, 'gs', 10); r.stop = True
        self.assertNotEqual(p.returncode, 0)
        self.assertIn('ERROR relay owned by another client', p.stdout)
        self.assertLess(time.time() - t0, 6)
        self.assertNotIn(TX, r.types())

    def test_spotter_proceeds_when_not_owner(self):
        r = FakeRelay(own=False)
        p = run_webgs(r.port, 'spotter', 2); r.stop = True
        self.assertEqual(p.returncode, 0, p.stdout)
        self.assertNotIn(TX, r.types())

    def test_silent_relay_is_unreachable(self):
        r = FakeRelay(answer=False)
        t0 = time.time(); p = run_webgs(r.port, 'gs', 10); r.stop = True
        self.assertIn('ERROR relay unreachable', p.stdout)
        self.assertLess(time.time() - t0, 6)

    def test_gs_tune_failed(self):
        r = FakeRelay(tune_fail=True)
        t0 = time.time(); p = run_webgs(r.port, 'gs', 10); r.stop = True
        self.assertIn('ERROR relay cannot tune', p.stdout)
        self.assertLess(time.time() - t0, 6)
        self.assertNotIn(TX, r.types())

    def test_status_stops_is_lost(self):
        r = FakeRelay()
        def mute(): time.sleep(1.5); r.answer = False
        threading.Thread(target=mute, daemon=True).start()
        p = run_webgs(r.port, 'gs', 10); r.stop = True
        self.assertIn('ERROR relay lost', p.stdout)

if __name__ == '__main__':
    unittest.main()
