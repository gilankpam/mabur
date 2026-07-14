#!/usr/bin/env python3
"""Golden vectors for mabur, generated from devourer's Python references.
Deterministic (no randomness, no time). Re-run + git diff must be clean."""
import json, os, struct, sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
PRECODER = os.path.abspath(os.path.join(ROOT, "..", "devourer", "tools", "precoder"))
sys.path.insert(0, PRECODER)

import fec_subblock, rc_proto, svc_uep_fec  # noqa: E402
import adaptive_link, energy_model  # noqa: E402

VEC = os.path.join(ROOT, "tests", "vectors")
FIX = os.path.join(ROOT, "tests", "fixtures")
os.makedirs(VEC, exist_ok=True); os.makedirs(FIX, exist_ok=True)

def pat(n, seed):
    return bytes(((i * 31 + seed * 17 + 7) & 0xFF) for i in range(n))

def hx(b): return bytes(b).hex()

def dump(name, obj):
    with open(os.path.join(VEC, name), "w") as f:
        json.dump(obj, f, indent=1, sort_keys=True)
    print("wrote", name)

# --- crc16 -------------------------------------------------------------
dump("crc16.json", {"cases": [
    {"in": hx(pat(n, s)), "crc": fec_subblock.crc16_ccitt(pat(n, s))}
    for n, s in [(0, 0), (1, 1), (9, 2), (32, 3), (75, 4), (1400, 5)]]})

# --- sliding-window fec (mabur-native; reference = tools/pyref/sw_fec.py) --
sys.path.insert(0, os.path.join(ROOT, "tools", "pyref"))
import sw_fec  # noqa: E402

SW_PKT_SIZES = [10, 50, 62, 1, 30, 62, 44, 62, 20, 62, 62, 5, 61, 33, 62, 62]
sw_cases = []
for window, ov in ((8, 1.0), (16, 0.5), (128, 0.25)):
    enc = sw_fec.SwEncoder(symbol_size=64, window=window, overhead=ov)
    stream, pkts = [], []
    for i, n in enumerate(SW_PKT_SIZES):
        p = pat(n, i)
        pkts.append(hx(p))
        stream += [hx(e) for e in enc.add_packet(p)]
    flush = [hx(e) for e in enc.flush()]
    envs = [bytes.fromhex(h) for h in stream + flush]
    src_idx = [i for i, e in enumerate(envs) if not (e[2] & 1)]
    # Decode scenarios: clean, drop one source, drop two consecutive sources.
    scen = [[], [src_idx[2]], src_idx[3:5]]
    decode = []
    for drop in scen:
        dec = sw_fec.SwDecoder(symbol_size=64)
        got = []
        for i, e in enumerate(envs):
            if i in drop:
                continue
            got += dec.add_symbol(e)
        decode.append({"drop": sorted(drop),
                       "recovered_sorted": sorted(hx(g) for g in got)})
    sw_cases.append({"symbol_size": 64, "window": window, "overhead": ov,
                     "packets": pkts, "stream": stream, "flush": flush,
                     "decode": decode})
dump("sw.json", {"cases": sw_cases})

# --- sbi ---------------------------------------------------------------
pk = fec_subblock.SubBlockPacker(75, 4, stream_id=2)
sbi_stream, envs = [], [pat(75, i + 40) for i in range(9)]
for e in envs:
    sbi_stream += [hx(b) for b in pk.add(e)]
sbi_flush = [hx(b) for b in pk.flush()]
dump("sbi.json", {"block_payload": 75, "blocks_per_body": 4, "stream_id": 2,
                  "envelopes": [hx(e) for e in envs],
                  "stream": sbi_stream, "flush": sbi_flush})

# --- frag --------------------------------------------------------------
class _P:  # minimal shim exposing what _frag_packets reads
    def __init__(self): self._seq = {0: 0}
frag_cases, shim = [], svc_uep_fec.SvcUepEncoder(svc_uep_fec.default_uep_policy(), fragment=True)
for n in (5, 58, 59, 200, 1400):
    p = pat(n, n & 7)
    frags = shim._frag_packets(0, p)  # usable = 64-2-4 = 58 for symbol_size 64
    frag_cases.append({"stream_id": 0, "usable": 58, "in": hx(p),
                       "out": [hx(f) for f in frags]})
dump("frag.json", {"cases": frag_cases})

# --- nal (HEVC NAL header classification, devourer's parser) -----------
def nal_bytes(t, tid, size=8):
    return bytes([(t << 1) & 0xFF, (tid + 1) & 0x07]) + pat(size - 2, t)
nal_cases = []
for t, tid in [(32, 0), (33, 0), (34, 0), (19, 0), (20, 0), (21, 0), (1, 0),
               (1, 1), (1, 2), (0, 0), (16, 1), (23, 2), (63, 0)]:
    nb = nal_bytes(t, tid)
    info = svc_uep_fec.parse_hevc_nal(nb)
    nal_cases.append({"in": hx(nb), "type": info.type, "tid": info.tid,
                      "critical": info.critical})
dump("nal.json", {"cases": nal_cases})

# --- RTP fixture + uep -------------------------------------------------
def rtp(seq, ts, payload, marker=0):
    return struct.pack(">BBHII", 0x80, (marker << 7) | 97, seq & 0xFFFF,
                       ts & 0xFFFFFFFF, 0x11223344) + payload

def hevc_hdr(t, tid): return bytes([(t << 1) & 0xFF, (tid + 1) & 0x07])

def fu(t, tid, chunk, start, end):
    fh = (0x80 if start else 0) | (0x40 if end else 0) | (t & 0x3F)
    return hevc_hdr(49, tid) + bytes([fh]) + chunk

rtp_packets, rtp_seq = [], 0
def emit(payload, marker=0):
    global rtp_seq
    rtp_packets.append(rtp(rtp_seq, 90000 + 3000 * rtp_seq, payload, marker))
    rtp_seq += 1

emit(hevc_hdr(32, 0) + pat(20, 1))          # VPS
emit(hevc_hdr(33, 0) + pat(40, 2))          # SPS
emit(hevc_hdr(34, 0) + pat(12, 3))          # PPS
idr = pat(3000, 4)                            # IDR sliced into 1200 B FUs
for i in range(0, len(idr), 1200):
    c = idr[i:i + 1200]
    emit(fu(19, 0, c, i == 0, i + 1200 >= len(idr)), marker=(i + 1200 >= len(idr)))
for f in range(12):                           # P frames on tids 0,1,2 round-robin
    tid = f % 3
    emit(hevc_hdr(1, tid) + pat(900 + 37 * f, 5 + f), marker=1)

with open(os.path.join(FIX, "rtp_stream.bin"), "wb") as f:
    for p in rtp_packets:
        f.write(struct.pack("<H", len(p))); f.write(p)
print("wrote rtp_stream.bin,", len(rtp_packets), "packets")

def classify(pkt):  # mirror of mabur classify_rtp (authoritative NAL rule above)
    if len(pkt) < 14 or (pkt[0] >> 6) != 2: return 0
    off = 12 + 4 * (pkt[0] & 0x0F); p = pkt[off:]
    t = (p[0] >> 1) & 0x3F
    if t == 49:
        real, tid = p[2] & 0x3F, (p[1] & 7) - 1
    elif t == 48:
        return 0
    else:
        real, tid = t, (p[1] & 7) - 1
    tid = max(tid, 0)
    crit = (16 <= real <= 23) or (32 <= real <= 34)
    return 0 if crit else 1 + min(tid, 2)

REF_OV = [1.00, 0.75, 0.50, 0.25]
encs = [sw_fec.SwEncoder(symbol_size=64, window=128, overhead=o) for o in REF_OV]
UEP_BLOCK_PAYLOAD = sw_fec.SW_HDR_LEN + 64  # 14 + symbol_size, matches Layer::packer in uep_encoder.h
pks = [fec_subblock.SubBlockPacker(UEP_BLOCK_PAYLOAD, 4, stream_id=s) for s in range(4)]
fseq = [0, 0, 0, 0]
uep_stream, uep_sids = [], []
def frag4(sid, pkt, usable=58):
    global fseq
    chunks = [pkt[i:i + usable] for i in range(0, max(len(pkt), 1), usable)]
    s = fseq[sid]; fseq[sid] = (s + 1) & 0xFFFF
    return [struct.pack("<HBB", s, i, len(chunks)) + c
            for i, c in enumerate(chunks)]
for pkt in rtp_packets:
    sid = classify(pkt)
    uep_sids.append(sid)
    for fp in frag4(sid, pkt):
        for env in encs[sid].add_packet(fp):
            for body in pks[sid].add(env):
                uep_stream.append({"sid": sid, "body": hx(body)})
uep_flush = []
for sid in range(4):
    for env in encs[sid].flush():
        for body in pks[sid].add(env):
            uep_flush.append({"sid": sid, "body": hx(body)})
    for body in pks[sid].flush():
        uep_flush.append({"sid": sid, "body": hx(body)})
# test_uep.cpp only reads symbol_size/blocks_per_body/overheads/classify (it
# round-trips body content live through UepEncoder/UepDecoder rather than
# pinning uep_stream/uep_flush bytes -- see the comment above that test).
# stream/flush are kept for debugging visibility; the RS-era "k" field is
# gone since nothing reads it under the sliding-window scheme.
dump("uep.json", {"symbol_size": 64, "blocks_per_body": 4,
                  "overheads": REF_OV, "classify": uep_sids,
                  "stream": uep_stream, "flush": uep_flush})

# --- rc ----------------------------------------------------------------
rcfs = [rc_proto.Rcf(vtx_id=0xDEADBEEF, seq=7, ack_seq=3800, profile=0x24,
                     score=1543, pwr_idx=40, fec_overhead_16ths=8, flags=0,
                     layer_delivery=(100, 98, 80, 10)),
        rc_proto.Rcf(vtx_id=1, seq=65535, ack_seq=0, profile=0x00, score=1000,
                     pwr_idx=rc_proto.PWR_NO_CHANGE, fec_overhead_16ths=16,
                     flags=rc_proto.F_FAILSAFE, layer_delivery=())]
discs = [rc_proto.Disc(vtx_id=1, vrx_nonce=0xCAFE0001, op_channel=149,
                       op_width=20, init_profile=0, seq=2)]
acks = [rc_proto.DiscAck(vtx_id=1, vrx_nonce=0xCAFE0001, chip_caps=0x0003,
                         agreed_channel=149, agreed_width=20, seq=1)]
dump("rc.json", {
  "rcf": [{"fields": {"vtx_id": r.vtx_id, "seq": r.seq, "ack_seq": r.ack_seq,
                      "profile": r.profile, "score": r.score, "pwr_idx": r.pwr_idx,
                      "fec_overhead_16ths": r.fec_overhead_16ths, "flags": r.flags,
                      "layer_delivery": list(r.layer_delivery)},
           "wire": hx(rc_proto.pack_rcf(r))} for r in rcfs],
  "disc": [{"fields": {"vtx_id": d.vtx_id, "vrx_nonce": d.vrx_nonce,
                       "op_channel": d.op_channel, "op_width": d.op_width,
                       "table_ver": d.table_ver, "init_profile": d.init_profile,
                       "cap_bits": d.cap_bits, "seq": d.seq},
            "wire": hx(rc_proto.pack_disc(d))} for d in discs],
  "disc_ack": [{"fields": {"vtx_id": a.vtx_id, "vrx_nonce": a.vrx_nonce,
                           "chip_caps": a.chip_caps,
                           "agreed_channel": a.agreed_channel,
                           "agreed_width": a.agreed_width, "seq": a.seq},
                "wire": hx(rc_proto.pack_disc_ack(a))} for a in acks]})

# --- profile / ladder / probe / phy rate --------------------------------
prof_cases = [{"mode": m, "mcs": mc, "bw": bw,
               "byte": rc_proto.encode_profile(m, mc, bw),
               "ladder": adaptive_link.ladder_spec(m, mc, bw)}
              for m in ("ht", "vht") for mc in range(0, 9)
              for bw in (20, 40, 80) if not (m == "ht" and (bw == 80 or mc > 7))]
probe_cases = [{"bw_set": list(bs),
                "probe": [rc_proto.probe_bw(s, bs) for s in range(64)]}
               for bs in ((20, 40), (20, 40, 80))]
rate_cases = [{"mode": m, "mcs": mc, "bw": bw, "sgi": sgi,
               "mbps": energy_model.phy_rate_mbps(m, mc, bw, sgi)}
              for m, mc, bw, sgi in [("ht", 0, 20, False), ("ht", 4, 20, False),
                                     ("ht", 7, 40, True), ("vht", 8, 80, False),
                                     ("vht", 4, 40, True)]]
dump("profile.json", {"profiles": prof_cases, "probe": probe_cases,
                      "rates": rate_cases,
                      "table": [{"ladder": p.svc_ladder, "pwr": p.pwr_idx,
                                 "ov": p.fec_overhead, "bw": p.bw}
                                for p in rc_proto.DEFAULT_PROFILE_TABLE]})

# --- sbi unpack ----------------------------------------------------------
su_cases = []
for bh in sbi_stream + sbi_flush:
    body = bytes.fromhex(bh)
    variants = [("clean", body)]
    c1 = bytearray(body); c1[fec_subblock.SBI_HDR_LEN + 2 + 3] ^= 0xFF  # 1st sub-blk payload
    variants.append(("one_subblock_corrupt", bytes(c1)))
    c2 = bytearray(body); c2[0] ^= 0xFF                                  # header magic
    variants.append(("header_corrupt", bytes(c2)))
    for name, b in variants:
        r = fec_subblock.unpack(b, 75)
        su_cases.append({"name": name, "body": hx(b), "block_payload": 75,
                         "survivors": [hx(s) for s in r.survivors],
                         "n_blocks": r.n_blocks, "n_failed": r.n_failed,
                         "header_ok": r.header_ok, "stream_id": r.stream_id})
su_cases.append({"name": "short", "body": hx(b"\xb0\xf5\x00"), "block_payload": 75,
                 "survivors": [], "n_blocks": 0, "n_failed": 0,
                 "header_ok": False, "stream_id": 0})
dump("sbi_unpack.json", {"cases": su_cases})

# --- node (RxBody / CardStatus, NEW mabur wire format v0) -----------------
def pack_rx_body(card_id, mono_us, rssi, snr, crc_ok, mac_seq, body):
    hdr = struct.pack("<HBBQBBbbBHH", 0xF5A0, 0, card_id, mono_us,
                      rssi[0], rssi[1], snr[0], snr[1],
                      1 if crc_ok else 0, mac_seq, len(body))
    buf = hdr + body
    return buf + struct.pack("<H", fec_subblock.crc16_ccitt(buf))

def pack_card_status(card_id, mono_us, frames_seen, crc_fail, uptime_s):
    buf = struct.pack("<HBBQIII", 0xF5A5, 0, card_id, mono_us,
                      frames_seen, crc_fail, uptime_s)
    return buf + struct.pack("<H", fec_subblock.crc16_ccitt(buf))

nb_cases = [
    {"card_id": 0, "mono_us": 0, "rssi": [0, 0], "snr": [0, 0], "crc_ok": True,
     "mac_seq": 0, "body": hx(b"")},
    {"card_id": 3, "mono_us": 1720000123456, "rssi": [42, 55], "snr": [25, -3],
     "crc_ok": False, "mac_seq": 4095, "body": hx(pat(75, 9))},
    {"card_id": 1, "mono_us": 2**63, "rssi": [200, 131], "snr": [127, -128],
     "crc_ok": True, "mac_seq": 2048, "body": hx(pat(1239, 2))},
]
for c in nb_cases:
    c["wire"] = hx(pack_rx_body(c["card_id"], c["mono_us"], c["rssi"], c["snr"],
                                c["crc_ok"], c["mac_seq"], bytes.fromhex(c["body"])))
cs_cases = [{"card_id": 2, "mono_us": 5_000_000, "frames_seen": 123456,
             "crc_fail": 78, "uptime_s": 3600}]
for c in cs_cases:
    c["wire"] = hx(pack_card_status(c["card_id"], c["mono_us"], c["frames_seen"],
                                    c["crc_fail"], c["uptime_s"]))
dump("node.json", {"rx_body": nb_cases, "card_status": cs_cases})

print("done")
