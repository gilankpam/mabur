#!/usr/bin/env python3
"""Golden vectors for mabur, generated from devourer's Python references.
Deterministic (no randomness, no time). Re-run + git diff must be clean.

energy.json's power-model cases (gain/pa-index) diverge from the prototype
since 2026-07-17. The airtime/rate/baseline-power dimensions are unchanged
and still ride devourer's frozen energy_model.py."""
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
import sbi  # noqa: E402

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

# energy.json (gs/src's TX-energy golden vectors) was removed 2026-07-27
# (SDD ladder-controller Task 5): the gs/src energy model and its only
# consumers (the model-driven controller and link table) were deleted,
# superseded by the measured-loss ladder controller.

# --- sbi (ver 1 with q_ms, enc_us latency fields; reference = tools/pyref/sbi.py) --
pk = sbi.SubBlockPacker(75, 4, stream_id=2)
sbi_stream, envs = [], [pat(75, i + 40) for i in range(9)]
for e in envs:
    sbi_stream += [hx(b) for b in pk.add(e)]
sbi_flush = [hx(b) for b in pk.flush()]
dump("sbi.json", {"block_payload": 75, "blocks_per_body": 4, "stream_id": 2,
                  "envelopes": [hx(e) for e in envs],
                  "stream": sbi_stream, "flush": sbi_flush})

# No frag vectors: the 4-byte FRAG format devourer's svc_uep_fec mirrors was
# deleted with the pre-frame-shm video path. The 6-byte wide format that
# replaced it is mabur-native (no Python reference); tests/test_frag.cpp pins
# it directly.

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

# --- frame fixture + uep -----------------------------------------------
# One record per encoded frame, byte-identical to what waybeam's frame-shm
# ring hands maburd (drone/vendor/venc_frame_ring.h):
#
#   u32-LE record length | VencFrameMeta | Annex-B frame
#   VencFrameMeta = pts u32-LE (µs) | codec u8 | flags u8 | reserved u16
#
# Access units follow the standard grouping rule the encoder emits: the
# parameter sets belong to the IDR that follows them, and each later VCL NAL
# is its own AU. Same NAL payloads (pat() seeds) as the pre-frame-shm
# rtp_stream.bin fixture this replaced, so the video content is unchanged —
# only the framing is.
VENC_FRAME_CODEC_H265 = 0x01
VENC_FRAME_FLAG_IDR = 0x01
VENC_FRAME_FLAG_ENHANCE = 0x04  # SVC-T enhance layer (droppable); must AGREE
                                 # with the payload's TRAIL_N-ness (frame_pipeline.cpp
                                 # protect-up demotion) or the frame lands on BASE.
START_CODE = b"\x00\x00\x00\x01"

def hevc_hdr(t, tid): return bytes([(t << 1) & 0xFF, (tid + 1) & 0x07])

def annexb(*nals): return b"".join(START_CODE + n for n in nals)

frames = [(VENC_FRAME_FLAG_IDR,
           annexb(hevc_hdr(32, 0) + pat(20, 1),      # VPS
                  hevc_hdr(33, 0) + pat(40, 2),      # SPS
                  hevc_hdr(34, 0) + pat(12, 3),      # PPS
                  hevc_hdr(19, 0) + pat(3000, 4)))]  # IDR_W_RADL slice
# P frames, tids 0,1,2, alternating BASE (TRAIL_R, type 1) and ENH (TRAIL_N,
# type 0) so the standing e2e fixture exercises both UEP streams (spec
# 2026-08-29-airtime-balance-uep; classify_frame routes TRAIL_N -> sid 1).
# Odd f -> enhance: the VENC_FRAME_FLAG_ENHANCE producer flag must be set
# alongside the TRAIL_N payload, or maburd's scan/flag agreement check
# demotes the frame back to sid 0.
for f in range(12):
    enhance = f % 2 == 1
    nal_type = 0 if enhance else 1
    flags = VENC_FRAME_FLAG_ENHANCE if enhance else 0
    frames.append((flags, annexb(hevc_hdr(nal_type, f % 3) + pat(900 + 37 * f, 5 + f))))

with open(os.path.join(FIX, "frame_stream.bin"), "wb") as f:
    for i, (flags, data) in enumerate(frames):
        meta = struct.pack("<IBBH", (i * 16667) & 0xFFFFFFFF,  # 60 fps pts
                           VENC_FRAME_CODEC_H265, flags, 0)
        f.write(struct.pack("<I", len(meta) + len(data)))
        f.write(meta + data)
print("wrote frame_stream.bin,", len(frames), "frames")

def classify_frame(data):  # mirror of mabur classify_frame (nal.cpp), 2-stream
    if len(data) < 5: return 0
    sid, i = None, 0
    while i + 4 < len(data):
        if data[i:i + 3] != b"\x00\x00\x01":
            i += 1
            continue
        info = svc_uep_fec.parse_hevc_nal(data[i + 3:])
        if info.critical: return 0
        if sid is None and info.type < 16: sid = 1 if info.type == 0 else 0
        i += 3
    return 0 if sid is None else sid

# test_uep.cpp reads symbol_size/blocks_per_body/overheads/classify: body
# content is round-tripped live through UepEncoder/UepDecoder rather than
# pinned byte-wise (sliding-window envelopes have no Python reference), while
# classify stays pinned against this mirror of the C++ classifier. 2-stream
# since the airtime-balance-uep fold-in: sid 0 = BASE (critical NALs + any
# VCL NAL type != 0), sid 1 = ENH (VCL NAL type 0, i.e. TRAIL_N).
dump("uep.json", {"symbol_size": 64, "blocks_per_body": 4,
                  "overheads": [1.00, 0.50],
                  "classify": [classify_frame(d) for _, d in frames]})

# --- rc ----------------------------------------------------------------
# Plain dicts, not rc_proto.Rcf: the frozen Python dataclass still carries the
# ack_seq/score/layer_delivery/flags fields mabur deleted from the wire in
# RC_VERSION 3, and constructing it here would only invite them back. RCF
# v5 (2026-08-30, same-rate-fixed-pairs) split the single literal-overhead
# byte into a per-stream pair (fec_overhead_base/fec_overhead_enh); the
# first two cases are equal pairs (prod's default shape, one value
# duplicated into both fields) and the third is a genuine asym pair
# (base 1.0 / enh 0.5) so classify/pack/parse actually exercise ENH riding
# a different overhead than BASE, not just a duplicated scalar.
# RC_VERSION 6: fixed probe byte; cases 1-2 leave it at the 0xFF default.
rcfs = [{"vtx_id": 0xDEADBEEF, "seq": 7, "profile": 0x24,
         "fec_overhead_base": 0.5, "fec_overhead_enh": 0.5},
        {"vtx_id": 1, "seq": 65535, "profile": 0x00,
         "fec_overhead_base": 1.0, "fec_overhead_enh": 1.0},
        {"vtx_id": 0x11223344, "seq": 42, "profile": 0x08,
         "fec_overhead_base": 1.0, "fec_overhead_enh": 0.5,
         "probe_profile": 0x06}]
discs = [rc_proto.Disc(vtx_id=1, vrx_nonce=0xCAFE0001, op_channel=149,
                       op_width=20, init_profile=0, seq=2)]
acks = [rc_proto.DiscAck(vtx_id=1, vrx_nonce=0xCAFE0001, chip_caps=0x0003,
                         agreed_channel=149, agreed_width=20, seq=1)]
# mabur owns its RC wire bytes as of 2026-08-12. devourer's frozen
# tools/precoder/rc_proto.py is pinned at RC_VERSION 1 and still packs a
# pwr_idx byte, so it can no longer serve as a wire oracle across mabur's
# RC_VERSION 2 change (nor the RC_VERSION 3 RCF shrink after it). The field
# cases below stay Python-generated; the expected bytes are hardcoded goldens
# in tests/test_rc.cpp.
dump("rc.json", {
  "rcf": [{"fields": r} for r in rcfs],
  "disc": [{"fields": {"vtx_id": d.vtx_id, "vrx_nonce": d.vrx_nonce,
                       "op_channel": d.op_channel, "op_width": d.op_width,
                       "table_ver": d.table_ver, "init_profile": d.init_profile,
                       "cap_bits": d.cap_bits, "seq": d.seq}} for d in discs],
  "disc_ack": [{"fields": {"vtx_id": a.vtx_id, "vrx_nonce": a.vrx_nonce,
                           "chip_caps": a.chip_caps,
                           "agreed_channel": a.agreed_channel,
                           "agreed_width": a.agreed_width, "seq": a.seq}} for a in acks]})

# --- profile / ladder / phy rate -----------------------------------------
# The per-seq bandwidth-probe-schedule vectors were removed 2026-07-27 (SDD
# ladder-controller Task 5): the ladder controller never varies bw
# independently of the commanded rung, so mabur's probe-schedule port and
# its golden vectors are dead. devourer's own reference is untouched — this
# just stops mirroring it into mabur's vectors.
prof_cases = [{"mode": m, "mcs": mc, "bw": bw,
               "byte": rc_proto.encode_profile(m, mc, bw),
               "ladder": adaptive_link.ladder_spec(m, mc, bw)}
              for m in ("ht", "vht") for mc in range(0, 9)
              for bw in (20, 40, 80) if not (m == "ht" and (bw == 80 or mc > 7))]
rate_cases = [{"mode": m, "mcs": mc, "bw": bw, "sgi": sgi,
               "mbps": energy_model.phy_rate_mbps(m, mc, bw, sgi)}
              for m, mc, bw, sgi in [("ht", 0, 20, False), ("ht", 4, 20, False),
                                     ("ht", 7, 40, True), ("vht", 8, 80, False),
                                     ("vht", 4, 40, True)]]
# NOTE: committed profile.json's profiles[].ladder values are mabur's own
# flat ladder (common/src/profile.cpp ladder_spec_str(), flat by the
# 2026-07-26 hw ruling documented there), NOT a fresh mirror of devourer's
# adaptive_link.ladder_spec() below, which now emits escalating T1/T2 MCS.
# Re-running this generator therefore reproduces a profiles[].ladder diff
# that is PRE-EXISTING drift, not a regression you introduced — devourer is
# frozen/off-limits, so reconciling it is a separate decision, not a side
# effect of regenerating vectors.
dump("profile.json", {"profiles": prof_cases,
                      "rates": rate_cases,
                      "table": [{"ladder": p.svc_ladder,
                                 "ov": p.fec_overhead, "bw": p.bw}
                                for p in rc_proto.DEFAULT_PROFILE_TABLE]})

# --- sbi unpack ----------------------------------------------------------
su_cases = []
for bh in sbi_stream + sbi_flush:
    body = bytes.fromhex(bh)
    variants = [("clean", body)]
    c1 = bytearray(body); c1[sbi.SBI_HDR_LEN + 2 + 3] ^= 0xFF  # 1st sub-blk payload
    variants.append(("one_subblock_corrupt", bytes(c1)))
    c2 = bytearray(body); c2[0] ^= 0xFF                        # header magic
    variants.append(("header_corrupt", bytes(c2)))
    for name, b in variants:
        r = sbi.unpack(b, 75)
        su_cases.append({"name": name, "body": hx(b), "block_payload": 75,
                         "survivors": [hx(s) for s in r["survivors"]],
                         "n_blocks": r["n_blocks"], "n_failed": r["n_failed"],
                         "header_ok": r["header_ok"], "stream_id": r["stream_id"]})
su_cases.append({"name": "short", "body": hx(b"\xb0\xf5\x00"), "block_payload": 75,
                 "survivors": [], "n_blocks": 0, "n_failed": 0,
                 "header_ok": False, "stream_id": 0})
dump("sbi_unpack.json", {"cases": su_cases})

# --- node (RxBody / CardStatus, NEW mabur wire format v0) -----------------
def pack_rx_body(card_id, mono_us, rssi, snr, crc_ok, mac_seq, body, phy_valid=True):
    flags = (1 if crc_ok else 0) | (2 if phy_valid else 0)
    hdr = struct.pack("<HBBQBBbbBHH", 0xF5A0, 0, card_id, mono_us,
                      rssi[0], rssi[1], snr[0], snr[1],
                      flags, mac_seq, len(body))
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
