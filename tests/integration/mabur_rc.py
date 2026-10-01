"""RC-frame packer for the host e2e scripts: the RC_VERSION 14 RCF with its
SipHash-2-4 tag (spec 2026-10-01 link-pairing). maburd --dry-run installs the
replay session (vrx 1, vtx 1), so frames tagged under it verify. Reads
RC_VERSION from rc_proto.h so a bump can't half-land (run_gs_e2e.sh history)."""
import os, re, struct

RC_MAGIC = 0x5243
T_RCF = 1
DEFAULT_KEY = b"mabur-default-00"
_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.abspath(os.path.join(_HERE, "..", ".."))
with open(os.path.join(_ROOT, "common/include/mabur/rc_proto.h")) as _f:
    RC_VERSION = int(re.search(r"RC_VERSION\s*=\s*(\d+)", _f.read()).group(1))

def crc16_ccitt(data: bytes, init: int = 0xFFFF) -> int:
    crc = init
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc

M64 = (1 << 64) - 1
def _rotl(x, b): return ((x << b) | (x >> (64 - b))) & M64
def siphash24(key: bytes, msg: bytes) -> int:
    k0, k1 = struct.unpack("<QQ", key)
    v0 = 0x736f6d6570736575 ^ k0; v1 = 0x646f72616e646f6d ^ k1
    v2 = 0x6c7967656e657261 ^ k0; v3 = 0x7465646279746573 ^ k1
    def rnd():
        nonlocal v0, v1, v2, v3
        v0 = (v0 + v1) & M64; v1 = _rotl(v1, 13); v1 ^= v0; v0 = _rotl(v0, 32)
        v2 = (v2 + v3) & M64; v3 = _rotl(v3, 16); v3 ^= v2
        v0 = (v0 + v3) & M64; v3 = _rotl(v3, 21); v3 ^= v0
        v2 = (v2 + v1) & M64; v1 = _rotl(v1, 17); v1 ^= v2; v2 = _rotl(v2, 32)
    n = len(msg); full = n - (n % 8)
    for i in range(0, full, 8):
        m = struct.unpack_from("<Q", msg, i)[0]
        v3 ^= m; rnd(); rnd(); v0 ^= m
    b = n << 56
    for j, byte in enumerate(msg[full:]): b |= byte << (8 * j)
    v3 ^= b; rnd(); rnd(); v0 ^= b
    v2 ^= 0xFF
    for _ in range(4): rnd()
    return (v0 ^ v1 ^ v2 ^ v3) & M64

def encode_profile(mcs: int, bw: int) -> int:
    # HT only here; mirrors common/src/profile.cpp encode_profile(HT, mcs, bw):
    # (mcs & 0x0F) | (bw_code << 4), bw_code 20->0, 40->1, 80->2 (VHT adds 0x40).
    return (mcs & 0x0F) | ({20: 0, 40: 1, 80: 2}[bw] << 4)

def tag(key: bytes, body: bytes, vrx: int, vtx: int, seq32: int) -> bytes:
    return struct.pack("<Q", siphash24(key, body + struct.pack("<III", vrx, vtx, seq32)))

def pack_rcf(seq, profile, ovb_x100, ove_x100, probe=0xFF, hop_ch=0, hop_epoch=0, rec=0, idr_epoch=0,
             *, vrx=1, vtx=1, seq32=None, key=DEFAULT_KEY) -> bytes:
    body = struct.pack("<HBBBHBBBBBBBB", RC_MAGIC, RC_VERSION, T_RCF, 0, seq & 0xFFFF, profile,
                       ovb_x100, ove_x100, probe, hop_ch, hop_epoch, rec, idr_epoch)
    body += tag(key, body, vrx, vtx, seq if seq32 is None else seq32)
    return body + struct.pack("<H", crc16_ccitt(body))

if __name__ == "__main__":
    k = bytes(range(16))
    assert siphash24(k, b"") == 0x726fdb47dd0e0e31
    assert siphash24(k, bytes(range(15))) == 0xa129ca6149be45e5
    assert encode_profile(4, 20) == 0x04 and encode_profile(3, 40) == 0x13
    print("ok")
