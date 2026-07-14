"""Pure-Python reference for mabur's sliding-window FEC (sw_wire.h /
sw_encoder.cpp / sw_decoder.cpp). This is the authority for the frozen wire
contract: gen_vectors.py derives tests/vectors/sw.json from it, and
tools/bench/decode_bodies.py decodes maburd output with it. Deterministic —
no randomness, no time."""
import struct

M64 = (1 << 64) - 1
SW_MAGIC = 0xF541
SW_HDR = struct.Struct("<HBHIBI")  # magic, flags, symbol_size, seq, window_len, repair_key
SW_HDR_LEN = 14
FLAG_REPAIR = 0x01

# --- GF(2^8), primitive polynomial 0x11D (matches mabur::gf) -------------
_EXP = [0] * 512
_LOG = [0] * 256
_x = 1
for _i in range(255):
    _EXP[_i] = _x
    _LOG[_x] = _i
    _x <<= 1
    if _x & 0x100:
        _x ^= 0x11D
for _i in range(255, 512):
    _EXP[_i] = _EXP[_i - 255]


def gf_mul(a, b):
    if a == 0 or b == 0:
        return 0
    return _EXP[_LOG[a] + _LOG[b]]


def gf_inv(a):
    return _EXP[255 - _LOG[a]] if a else 0


def _lincomb(acc, sym, coeff):
    """acc[i] ^= coeff * sym[i] — mirrors mabur::gf::lincomb."""
    for i in range(len(acc)):
        acc[i] ^= gf_mul(coeff, sym[i])


def _splitmix64(state):
    state = (state + 0x9E3779B97F4A7C15) & M64
    z = state
    z = ((z ^ (z >> 30)) * 0xBF58476D1CE4E5B9) & M64
    z = ((z ^ (z >> 27)) * 0x94D049BB133111EB) & M64
    return state, z ^ (z >> 31)


def repair_coeffs(key, n):
    s = key
    out = []
    for _ in range(n):
        s, z = _splitmix64(s)
        out.append((z & 0xFF) % 255 + 1)
    return out


class SwEncoder:
    def __init__(self, symbol_size=64, window=128, overhead=0.25):
        self.ss, self.window, self.overhead = symbol_size, window, overhead
        self.ring = []          # last <=window sealed payloads
        self.cur = bytearray()
        self.next_seq = 0
        self.repair_key = 0
        self.credit = 0.0
        self.tail_pending = False

    def _seal(self, out):
        if not self.cur:
            return
        self.cur += bytes(self.ss - len(self.cur))
        out.append(SW_HDR.pack(SW_MAGIC, 0, self.ss, self.next_seq, 0, 0) + bytes(self.cur))
        self.ring.append(bytes(self.cur))
        self.cur = bytearray()
        if len(self.ring) > self.window:
            self.ring.pop(0)
        self.next_seq += 1
        self.tail_pending = True
        self.credit += self.overhead
        while self.credit >= 1.0:
            out.append(self._repair())
            self.credit -= 1.0

    def _repair(self):
        wl = len(self.ring)
        ws = self.next_seq - wl
        key = self.repair_key
        self.repair_key += 1
        acc = bytearray(self.ss)
        for c, sym in zip(repair_coeffs(key, wl), self.ring):
            _lincomb(acc, sym, c)
        self.tail_pending = False
        return SW_HDR.pack(SW_MAGIC, FLAG_REPAIR, self.ss, ws, wl, key) + bytes(acc)

    def add_packet(self, pkt):
        out = []
        if len(pkt) > self.ss - 2:
            return out
        if 2 + len(pkt) > self.ss - len(self.cur):
            self._seal(out)
        self.cur += struct.pack("<H", len(pkt)) + pkt
        return out

    def flush(self):
        out = []
        self._seal(out)
        if self.tail_pending and self.ring:
            out.append(self._repair())
        return out


class SwDecoder:
    """In-order-feed GE decoder (enough for vectors and decode_bodies; the
    C++ decoder additionally handles wrap/reset/expiry)."""

    def __init__(self, symbol_size=64, horizon=0):
        self.ss = symbol_size
        self.horizon = horizon or 10 ** 9
        self.known = {}    # seq -> payload
        self.rows = {}     # pivot seq -> (coeffs dict, payload bytearray)
        self.newest = -1

    def _unpack(self, sym):
        out, pos = [], 0
        while pos + 2 <= self.ss:
            (ln,) = struct.unpack_from("<H", sym, pos)
            if ln == 0 or pos + 2 + ln > self.ss:
                break
            out.append(bytes(sym[pos + 2:pos + 2 + ln]))
            pos += 2 + ln
        return out

    def _insert(self, coeffs, payload, solved):
        while True:
            if not coeffs:
                return
            pivot = min(coeffs)
            if pivot not in self.rows:
                lead = coeffs[pivot]
                if lead != 1:
                    il = gf_inv(lead)
                    coeffs = {s: gf_mul(c, il) for s, c in coeffs.items()}
                    scaled = bytearray(self.ss)
                    _lincomb(scaled, payload, il)
                    payload = scaled
                if len(coeffs) == 1:
                    solved.append((pivot, payload))
                    return
                self.rows[pivot] = (coeffs, payload)
                return
            ec, ep = self.rows[pivot]
            f = coeffs[pivot]
            for s, c in ec.items():
                nv = coeffs.get(s, 0) ^ gf_mul(f, c)
                if nv:
                    coeffs[s] = nv
                else:
                    coeffs.pop(s, None)
            _lincomb(payload, ep, f)

    def _ingest(self, seq, payload, out):
        queue = [(seq, payload)]
        while queue:
            s, p = queue.pop()
            if s in self.known:
                continue
            out += self._unpack(p)
            self.known[s] = bytes(p)
            affected = [(k, v) for k, v in self.rows.items() if s in v[0]]
            for k, (coeffs, rp) in affected:
                del self.rows[k]
                _lincomb(rp, self.known[s], coeffs.pop(s))
                solved = []
                self._insert(coeffs, rp, solved)
                queue += solved

    def add_symbol(self, env):
        out = []
        if len(env) != SW_HDR_LEN + self.ss:
            return out
        magic, flags, ss, seq, wl, key = SW_HDR.unpack_from(env)
        if magic != SW_MAGIC or ss != self.ss:
            return out
        payload = bytearray(env[SW_HDR_LEN:])
        if not (flags & FLAG_REPAIR):
            if seq in self.known:
                return out
            self.newest = max(self.newest, seq)
            self._ingest(seq, payload, out)
            return out
        self.newest = max(self.newest, seq + wl - 1)
        coeffs = {}
        for i, c in enumerate(repair_coeffs(key, wl)):
            s = seq + i
            if s in self.known:
                _lincomb(payload, self.known[s], c)
            else:
                coeffs[s] = c
        solved = []
        self._insert(coeffs, payload, solved)
        for sv, sym in solved:
            self._ingest(sv, sym, out)
        return out
