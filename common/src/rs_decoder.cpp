#include "mabur/rs_decoder.h"

#include "mabur/gf256.h"

namespace mabur {
namespace {
constexpr uint16_t kRsMagic = 0xF540;
constexpr size_t kRsHeaderLen = 11;
constexpr size_t kLenPrefix = 2;
uint16_t rd_u16(const uint8_t* p) {
  return static_cast<uint16_t>(p[0] | (p[1] << 8));
}
}  // namespace

RsDecoder::RsDecoder(const RsConfig& cfg) : cfg_(cfg) {}

std::vector<std::vector<uint8_t>> RsDecoder::add_symbol(const uint8_t* env,
                                                        size_t len,
                                                        uint64_t now_ms) {
  // Reject if envelope too short, magic wrong, or version (env[2]) nonzero.
  // Version check matches Python's stream_fec_rs.py _unpack_header.
  if (len < kRsHeaderLen || rd_u16(env) != kRsMagic || env[2] != 0) return {};
  const int k = env[3], kreal = env[4];
  const int ss = rd_u16(env + 5);
  const uint16_t block_id = rd_u16(env + 7);
  const int esi = env[9], n = env[10];
  if (k != cfg_.k || ss != cfg_.symbol_size) { ++symbols_dropped_bad_cfg_; return {}; }
  if (kreal < 1 || kreal > k || esi < 0 || esi >= n || n < k) {
    ++symbols_dropped_bad_cfg_;
    return {};
  }
  if (len - kRsHeaderLen != static_cast<size_t>(ss)) { ++symbols_dropped_bad_cfg_; return {}; }
  ++symbols_in_;

  auto it = blocks_.find(block_id);
  if (it == blocks_.end()) {
    it = blocks_.emplace(block_id, Block{k, n, kreal, now_ms, {}, false}).first;
  } else if (it->second.decoded) {
    ++symbols_dropped_stale_block_;
    return {};
  }
  Block& st = it->second;
  // emplace = Python's setdefault: a duplicate ESI never overwrites.
  st.symbols.emplace(esi, std::vector<uint8_t>(env + kRsHeaderLen, env + len));
  if (static_cast<int>(st.symbols.size()) < k) return {};
  return solve(st);
}

std::vector<std::vector<uint8_t>> RsDecoder::solve(Block& st) {
  const int k = st.k, ss = cfg_.symbol_size;
  std::vector<int> esis;  // k smallest ESIs — std::map iterates sorted
  for (const auto& [e, sym] : st.symbols) {
    esis.push_back(e);
    if (static_cast<int>(esis.size()) == k) break;
  }
  std::vector<std::vector<uint8_t>> source;
  if (esis.back() < k) {  // all k systematic symbols present: no inverse
    source.reserve(k);
    for (int e = 0; e < k; ++e) source.push_back(st.symbols[e]);
  } else {
    const auto& A = gf::encoding_matrix(k, st.n);
    gf::Matrix sub;
    sub.reserve(k);
    for (int e : esis) sub.push_back(A[static_cast<size_t>(e)]);
    gf::Matrix inv = gf::mat_inv(sub);
    source.assign(k, std::vector<uint8_t>(static_cast<size_t>(ss), 0));
    for (int j = 0; j < k; ++j)
      for (int i = 0; i < k; ++i)
        gf::lincomb(source[static_cast<size_t>(j)].data(),
                    st.symbols[esis[static_cast<size_t>(i)]].data(),
                    inv[static_cast<size_t>(j)][static_cast<size_t>(i)],
                    static_cast<size_t>(ss));
  }
  st.decoded = true;
  st.symbols.clear();  // deviation: free payloads now, keep the stale marker
  ++blocks_decoded_;
  auto pkts = unpack(source, st.kreal);
  packets_out_ += pkts.size();
  return pkts;
}

std::vector<std::vector<uint8_t>> RsDecoder::unpack(
    const std::vector<std::vector<uint8_t>>& source, int kreal) const {
  std::vector<std::vector<uint8_t>> out;
  for (int i = 0; i < kreal; ++i) {
    const auto& sym = source[static_cast<size_t>(i)];
    size_t pos = 0;
    while (pos + kLenPrefix <= sym.size()) {
      const size_t ln = rd_u16(sym.data() + pos);
      if (ln == 0) break;
      const size_t end = pos + kLenPrefix + ln;
      if (end > sym.size()) break;
      out.emplace_back(sym.begin() + static_cast<long>(pos + kLenPrefix),
                       sym.begin() + static_cast<long>(end));
      pos = end;
    }
  }
  return out;
}

int RsDecoder::expire_blocks_older_than(uint64_t max_age_ms, uint64_t now_ms) {
  // Precondition: now_ms must be monotonic non-decreasing across calls.
  // Passing a now_ms below a block's first_seen_ms underflows and expires it immediately.
  int unrecoverable = 0;
  for (auto it = blocks_.begin(); it != blocks_.end();) {
    if (now_ms - it->second.first_seen_ms > max_age_ms) {
      if (!it->second.decoded) ++unrecoverable;
      it = blocks_.erase(it);
    } else {
      ++it;
    }
  }
  blocks_unrecoverable_ += static_cast<uint64_t>(unrecoverable);
  return unrecoverable;
}

}  // namespace mabur
