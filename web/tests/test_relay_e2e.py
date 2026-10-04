#!/usr/bin/env python3
"""webgs live --relay against a fake mabur-relay v4 (UDP). argv[1] = webgs binary."""
import socket, struct, subprocess, sys, threading, time, unittest, json
import tempfile, os, atexit

WEBGS = sys.argv.pop(1) if len(sys.argv) > 1 else 'build/web/webgs'
FRAME, HELLO, TUNE, STATUS, TX = 1, 2, 3, 4, 5

def overlay(text):
    f = tempfile.NamedTemporaryFile('w', suffix='.toml', delete=False)
    f.write(text); f.close()
    atexit.register(os.unlink, f.name)
    return f.name

# The bundle's set does not hold 136: a one-member set so --ch 136 is a
# member. channel = "auto" (not a pin), so a relay-only GS roster takes the
# core's relay search-only path (no boot scan) rather than the pin's.
OVERLAY = overlay('[radio]\nchannels = [136]\nchannel = "auto"\n')
# What the page sends at 20 MHz for a set with no 40 MHz pair: [radio] width
# 20 (the loader checks the set at 20, not the bundle's 40) and a ladder at 20.
LINK20 = ('\n[link]\nstatic_mcs = -1\nstatic_bw = 20\nmax_mcs = 7\n'
          '\n[[link.ladder]]\nmcs = 0\nbw = 20\noverhead_base = 0.5\noverhead_enh = 0.25\n')
OVERLAY_GS20 = overlay('[radio]\nchannels = [165]\nchannel = 165\nwidth = 20\n' + LINK20)
OVERLAY_SP20 = overlay('[radio]\nchannels = [165]\nchannel = "auto"\nwidth = 20\n' + LINK20)

def hdr(t): return struct.pack('<HBB', 0x524D, 4, t)
def status(state, ch, sec, you_own, tune_id=0):
    # v4 STATUS is 51 bytes: the v3 47-byte layout (header + tune_id/state/
    # channel/sec/owner/you_own + 9 zeroed u32 counters) plus tx_scan_drop
    # (u32) appended at offset 47. No SURVEY feed needed for this test.
    return hdr(STATUS) + struct.pack('<HBBBBB', tune_id, state, ch, sec, 1, you_own) + b'\0' * 40
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
        self.stolen = False   # another client took ownership mid-session
        self.got, self.peer, self.tuned = [], None, None
        threading.Thread(target=self.run, daemon=True).start()
    def steal(self):
        # Another UDP subscriber takes ownership: the relay stays tuned on
        # our channel (state 0), but you_own flips to 0 for us.
        self.own = False
        self.stolen = True
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
                    elif self.stolen:
                        ch, sec = self.tuned or (0, 0)
                        self.s.sendto(status(0, ch, sec, 0), a)
                    else:
                        ch, sec = self.tuned or (0, 0)
                        self.s.sendto(status(0 if self.own else 3, ch, sec, 1 if self.own else 0), a)
            except socket.timeout: pass
            if self.peer and self.answer and self.own:
                for _ in range(20):
                    self.s.sendto(qos_frame(seq, seq & 0xFFF), self.peer); seq += 1
    def types(self): return [b[3] for b in self.got]

def run_webgs(port, mode, secs, ch=136, w=40, ov=OVERLAY):
    return subprocess.run([WEBGS, 'live', '--relay', f'127.0.0.1:{port}', '--mode', mode,
                           '--ch', str(ch), '--w', str(w), '--secs', str(secs), '--overlay', ov],
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
        self.assertEqual(st['channel'], 136)
        self.assertEqual(st['scan_state'], 'off')                # auto, relay-only roster: search-only
        self.assertIsNone(st['follow_state'])

    def test_gs_20mhz_set_passes_the_loader(self):
        # 165 has no 40 MHz pair: valid at 20 only. The overlay's [radio]
        # width = 20 is what lets the loader accept it (Important 1).
        r = FakeRelay()
        p = run_webgs(r.port, 'gs', 3, ch=165, w=20, ov=OVERLAY_GS20); r.stop = True
        self.assertEqual(p.returncode, 0, p.stdout + p.stderr)
        self.assertNotIn('ERROR', p.stdout)
        self.assertEqual(r.tuned, (165, 0))
        st = stats(p.stdout)[-1]
        self.assertEqual(st['channel'], 165); self.assertEqual(st['bw'], 20)

    def test_spotter_20mhz_set_passes_the_loader(self):
        r = FakeRelay()
        p = run_webgs(r.port, 'spotter', 3, ch=165, w=20, ov=OVERLAY_SP20); r.stop = True
        self.assertEqual(p.returncode, 0, p.stdout + p.stderr)
        self.assertNotIn('ERROR', p.stdout)

    def test_gs_refused_when_not_owner(self):
        r = FakeRelay(own=False)
        t0 = time.time(); p = run_webgs(r.port, 'gs', 10); r.stop = True
        self.assertNotEqual(p.returncode, 0)
        self.assertIn('ERROR relay owned by another client', p.stdout)
        self.assertLess(time.time() - t0, 6)
        self.assertNotIn(TX, r.types())

    def test_spotter_is_refused_when_not_owner(self):
        # One relay, one client (spec 2026-10-04 §1): the page is the relay's
        # tune owner in BOTH modes; a refusal is an operator error, not a listen-only path.
        r = FakeRelay(own=False)
        t0 = time.time(); p = run_webgs(r.port, 'spotter', 10); r.stop = True
        self.assertNotEqual(p.returncode, 0)
        self.assertIn('ERROR relay owned by another client', p.stdout)
        self.assertLess(time.time() - t0, 6)
        self.assertIn(TUNE, r.types())           # it did ask
        self.assertNotIn(TX, r.types())

    def test_spotter_owned_receives_without_tx(self):
        r = FakeRelay()
        p = run_webgs(r.port, 'spotter', 3); r.stop = True
        self.assertEqual(p.returncode, 0, p.stdout + p.stderr)
        self.assertEqual(r.tuned, (136, 2))
        self.assertNotIn(TX, r.types())
        st = stats(p.stdout)[-1]
        self.assertEqual(st['channel'], 136); self.assertIsNone(st['scan_state'])
        self.assertEqual(st['follow_state'], 'locked')   # the fake relay's frames arrive on 136 (FRAME header channel)
        self.assertEqual(st['follows'], 0)
        self.assertGreater(st['bodies'], 100)

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

    def test_gs_ownership_taken_mid_session_reports_and_exits(self):
        r = FakeRelay()
        def steal(): time.sleep(3); r.steal()
        threading.Thread(target=steal, daemon=True).start()
        t0 = time.time(); p = run_webgs(r.port, 'gs', 10); r.stop = True
        self.assertNotEqual(p.returncode, 0)
        self.assertIn('ERROR relay taken by another client', p.stdout)
        self.assertLess(time.time() - t0, 6)

    def test_gs_ownership_taken_late_reports_and_exits(self):
        # Theft past RemoteCard's 5 s refused-restart point: the page's card
        # never restarts, so the refusal is reported, not reset away.
        r = FakeRelay()
        def steal(): time.sleep(7); r.steal()
        threading.Thread(target=steal, daemon=True).start()
        t0 = time.time(); p = run_webgs(r.port, 'gs', 15); r.stop = True
        self.assertNotEqual(p.returncode, 0)
        self.assertIn('ERROR relay taken by another client', p.stdout)
        self.assertLess(time.time() - t0, 12)

    def test_spotter_ownership_taken_mid_session_reports_and_exits(self):
        # One relay, one client: a spotter owns the relay too, so losing it
        # mid-session is the same operator error as for a GS.
        r = FakeRelay()
        def steal(): time.sleep(1); r.steal()
        threading.Thread(target=steal, daemon=True).start()
        t0 = time.time(); p = run_webgs(r.port, 'spotter', 10); r.stop = True
        self.assertNotEqual(p.returncode, 0)
        self.assertIn('ERROR relay taken by another client', p.stdout)
        self.assertLess(time.time() - t0, 6)

if __name__ == '__main__':
    unittest.main()
