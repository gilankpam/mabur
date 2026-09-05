#include "mabur/probe_wire.h"
#include "mabur/crc16.h"
#include "mabur/sbi.h"

namespace mabur::probe {
namespace {
uint64_t splitmix64(uint64_t& x) {
  uint64_t z = (x += 0x9E3779B97F4A7C15ull);
  z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
  z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
  return z ^ (z >> 31);
}
}  // namespace

void pack_hdr(uint8_t out[kProbeHdrLen], const ProbeHdr& h) {
  out[0] = kProbeMagic & 0xFF; out[1] = kProbeMagic >> 8;
  out[2] = h.seq & 0xFF; out[3] = (h.seq >> 8) & 0xFF;
  out[4] = (h.seq >> 16) & 0xFF; out[5] = (h.seq >> 24) & 0xFF;
  out[6] = h.profile;
  out[7] = h.enh_fid & 0xFF; out[8] = h.enh_fid >> 8;
}

bool parse_hdr(const uint8_t* p, size_t len, ProbeHdr* h) {
  if (len < kProbeHdrLen) return false;
  if ((p[0] | (p[1] << 8)) != kProbeMagic) return false;
  h->seq = static_cast<uint32_t>(p[2]) | (static_cast<uint32_t>(p[3]) << 8) |
           (static_cast<uint32_t>(p[4]) << 16) | (static_cast<uint32_t>(p[5]) << 24);
  h->profile = p[6];
  h->enh_fid = static_cast<uint16_t>(p[7] | (p[8] << 8));
  return true;
}

size_t probe_body_len(int bpb, int block_payload) {
  return static_cast<size_t>(SBI_HDR_LEN) +
         static_cast<size_t>(bpb) * (2 + static_cast<size_t>(block_payload));
}

std::vector<uint8_t> build_probe_body(const ProbeHdr& h, int bpb, int block_payload) {
  if (block_payload < static_cast<int>(kProbeHdrLen)) return {};
  std::vector<uint8_t> out;
  out.reserve(probe_body_len(bpb, block_payload));
  out.push_back(SBI_MAGIC & 0xFF); out.push_back(SBI_MAGIC >> 8);
  out.push_back(SBI_VER);
  out.push_back(kProbeStreamId);
  out.push_back(block_payload & 0xFF); out.push_back((block_payload >> 8) & 0xFF);
  out.push_back(static_cast<uint8_t>(bpb));
  out.push_back(0); out.push_back(0);  // q_ms (patched by the TX thread)
  out.push_back(0); out.push_back(0);  // enc_us (meaningless for a probe)
  std::vector<uint8_t> payload(static_cast<size_t>(block_payload));
  uint64_t rng = h.seq;
  for (int i = 0; i < bpb; ++i) {
    pack_hdr(payload.data(), h);
    for (size_t k = kProbeHdrLen; k < payload.size(); ++k)
      payload[k] = static_cast<uint8_t>(splitmix64(rng));
    const uint16_t crc = crc16_ccitt(payload.data(), payload.size());
    out.push_back(crc & 0xFF); out.push_back(crc >> 8);
    out.insert(out.end(), payload.begin(), payload.end());
  }
  return out;
}

bool parse_probe_body(const uint8_t* body, size_t len, int block_payload, ProbeRx* out) {
  if (sbi_peek_stream_id(body, len) != kProbeStreamId) return false;
  // Geometry sanity: the body's own SBI header must agree with the
  // caller's block_payload. A drone/GS FEC-geometry mismatch (different
  // symbol_size/block_payload config on the two ends) would otherwise walk
  // the body at the wrong stride and read every block as CRC-garbage --
  // 100% probe loss with no obvious cause. Rejecting it here instead reads
  // loud on the sideport: rx stays 0 while classes.probe.frames climbs.
  const int wire_block_payload = body[4] | (body[5] << 8);
  if (wire_block_payload != block_payload) return false;
  int n = body[6];
  if (n < 1 || n > 32) return false;
  const size_t stride = 2 + static_cast<size_t>(block_payload);
  if (len < SBI_HDR_LEN + static_cast<size_t>(n) * stride) return false;
  ProbeRx rx;
  rx.n_blocks = n;
  bool have = false;
  for (int i = 0; i < n; ++i) {
    const uint8_t* blk = body + SBI_HDR_LEN + static_cast<size_t>(i) * stride;
    const uint16_t crc = static_cast<uint16_t>(blk[0] | (blk[1] << 8));
    if (crc != crc16_ccitt(blk + 2, static_cast<size_t>(block_payload))) continue;
    ProbeHdr h;
    if (!parse_hdr(blk + 2, static_cast<size_t>(block_payload), &h)) continue;
    if (have && (h.seq != rx.hdr.seq || h.profile != rx.hdr.profile ||
                 h.enh_fid != rx.hdr.enh_fid))
      return false;
    rx.hdr = h; have = true;
    rx.survivors |= 1u << i;
    ++rx.n_ok;
  }
  if (!have) return false;
  *out = rx;
  return true;
}
}  // namespace mabur::probe
