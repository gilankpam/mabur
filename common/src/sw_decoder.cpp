#include "mabur/sw_decoder.h"

#include "mabur/gf256.h"
#include "mabur/sw_wire.h"

namespace mabur {
namespace {
// A source seq this far from the newest is an encoder restart, not wrap
// (wrap-adjacent seqs land near 0 after int32 casting; a reboot lands
// ~2^32 away). ~1 minute of symbols at video rates.
constexpr int64_t kResetSpan = 1 << 20;
constexpr uint64_t kAnchor = 1ull << 32;  // headroom below the first vseq
uint16_t rd16(const uint8_t* p) { return static_cast<uint16_t>(p[0] | (p[1] << 8)); }
}  // namespace

SwDecoder::SwDecoder(const SwConfig& cfg, uint32_t seq_horizon)
    : cfg_(cfg),
      horizon_(seq_horizon ? seq_horizon : 4ull * static_cast<uint64_t>(cfg.window)) {}

uint64_t SwDecoder::unwrap(uint32_t s) const {
  const int64_t d = static_cast<int32_t>(s - static_cast<uint32_t>(newest_v_));
  return newest_v_ + static_cast<uint64_t>(d);
}

void SwDecoder::reset_state(uint64_t v) {
  known_.clear();
  rows_.clear();
  newest_v_ = v;
  base_ = v;
  ++resets_;
}

void SwDecoder::advance(uint64_t newest_candidate) {
  if (newest_candidate <= newest_v_) return;
  newest_v_ = newest_candidate;
  const uint64_t nb = newest_v_ - horizon_;  // vseqs start at 2^32 >> horizon
  if (nb <= base_) return;
  // Every seq in [base_, nb) that never became known is lost for good.
  uint64_t known_in_range = 0;
  for (auto it = known_.begin(); it != known_.end() && it->first < nb;) {
    ++known_in_range;
    it = known_.erase(it);
  }
  syms_abandoned_ += (nb - base_) - known_in_range;
  // Rows are keyed by pivot = their smallest referenced seq, so everything
  // that references an evicted seq is at the front of the map.
  while (!rows_.empty() && rows_.begin()->first < nb) rows_.erase(rows_.begin());
  base_ = nb;
}

void SwDecoder::unpack_symbol(const uint8_t* sym, std::vector<std::vector<uint8_t>>& out) {
  const size_t ss = static_cast<size_t>(cfg_.symbol_size);
  size_t pos = 0;
  while (pos + 2 <= ss) {
    const size_t ln = rd16(sym + pos);
    if (ln == 0) break;
    const size_t end = pos + 2 + ln;
    if (end > ss) break;
    out.emplace_back(sym + pos + 2, sym + end);
    ++packets_out_;
    pos = end;
  }
}

void SwDecoder::insert_row(Row r, std::vector<std::pair<uint64_t, std::vector<uint8_t>>>& solved) {
  const size_t ss = static_cast<size_t>(cfg_.symbol_size);
  for (;;) {
    if (r.coeffs.empty()) return;  // linearly dependent — nothing new
    const uint64_t pivot = r.coeffs.begin()->first;
    auto it = rows_.find(pivot);
    if (it == rows_.end()) {
      const uint8_t lead = r.coeffs.begin()->second;
      if (lead != 1) {
        const uint8_t ilead = gf::inv(lead);
        for (auto& [s, c] : r.coeffs) c = gf::mul(c, ilead);
        std::vector<uint8_t> scaled(ss, 0);
        gf::lincomb(scaled.data(), r.payload.data(), ilead, ss);
        r.payload = std::move(scaled);
      }
      if (r.coeffs.size() == 1) {
        solved.emplace_back(pivot, std::move(r.payload));
        return;
      }
      rows_.emplace(pivot, std::move(r));
      return;
    }
    // Eliminate pivot with the existing (lead-normalized) row.
    const Row& e = it->second;
    const uint8_t f = r.coeffs.begin()->second;
    for (const auto& [s, c] : e.coeffs) {
      const auto jt = r.coeffs.find(s);
      const uint8_t nv = static_cast<uint8_t>((jt == r.coeffs.end() ? 0 : jt->second) ^ gf::mul(f, c));
      if (nv)
        r.coeffs[s] = nv;
      else if (jt != r.coeffs.end())
        r.coeffs.erase(jt);
    }
    gf::lincomb(r.payload.data(), e.payload.data(), f, ss);
  }
}

void SwDecoder::ingest(uint64_t v, std::vector<uint8_t> sym, bool source,
                       std::vector<std::vector<uint8_t>>& out) {
  std::vector<std::pair<uint64_t, std::vector<uint8_t>>> queue;
  queue.emplace_back(v, std::move(sym));
  bool first = true;
  while (!queue.empty()) {
    auto [s, payload] = std::move(queue.back());
    queue.pop_back();
    const bool count_as_source = source && first;
    first = false;
    if (s < base_ || known_.count(s)) continue;
    unpack_symbol(payload.data(), out);
    if (count_as_source) ++syms_delivered_; else ++syms_recovered_;
    auto [kit, ok] = known_.emplace(s, std::move(payload));
    (void)ok;
    // Pull every row referencing s, reduce, re-insert (rows only ever
    // reference unknown seqs — this keeps that invariant).
    std::vector<Row> affected;
    for (auto it = rows_.begin(); it != rows_.end();) {
      if (it->second.coeffs.count(s)) {
        affected.push_back(std::move(it->second));
        it = rows_.erase(it);
      } else {
        ++it;
      }
    }
    for (auto& r : affected) {
      const uint8_t c = r.coeffs[s];
      gf::lincomb(r.payload.data(), kit->second.data(), c, static_cast<size_t>(cfg_.symbol_size));
      r.coeffs.erase(s);
      std::vector<std::pair<uint64_t, std::vector<uint8_t>>> solved;
      insert_row(std::move(r), solved);
      for (auto& sv : solved) queue.push_back(std::move(sv));
    }
  }
}

std::vector<std::vector<uint8_t>> SwDecoder::add_symbol(const uint8_t* env, size_t len,
                                                        uint64_t now_ms) {
  std::vector<std::vector<uint8_t>> out;
  sw::SwHeader h;
  if (!sw::parse_header(env, len, &h)) return out;
  if (h.symbol_size != cfg_.symbol_size ||
      len != sw::kSwHeaderLen + static_cast<size_t>(cfg_.symbol_size)) {
    ++symbols_dropped_bad_cfg_;
    return out;
  }
  ++symbols_in_;
  const uint8_t* payload = env + sw::kSwHeaderLen;
  const size_t ss = static_cast<size_t>(cfg_.symbol_size);

  if (!h.repair) {
    if (!have_seq_) {
      have_seq_ = true;
      newest_v_ = kAnchor + h.seq;
      base_ = newest_v_;
    }
    uint64_t v = unwrap(h.seq);
    if (v > newest_v_ + static_cast<uint64_t>(kResetSpan) ||
        v + static_cast<uint64_t>(kResetSpan) < newest_v_) {
      reset_state(kAnchor + h.seq);
      v = newest_v_;
    }
    if (v < base_ || known_.count(v)) {
      ++symbols_dropped_stale_;
      return out;
    }
    advance(v);
    ingest(v, std::vector<uint8_t>(payload, payload + ss), /*source=*/true, out);
    return out;
  }

  // Repair. Before the first source there is no seq anchor — drop (join-time
  // only; sources outnumber repairs, the next one anchors us).
  if (!have_seq_) {
    ++symbols_dropped_stale_;
    return out;
  }
  const uint64_t ws = unwrap(h.seq);
  const int wl = h.window_len;
  const uint64_t wend = ws + static_cast<uint64_t>(wl);  // one past last covered
  if (wend > newest_v_ + static_cast<uint64_t>(kResetSpan) ||
      wend + static_cast<uint64_t>(kResetSpan) < newest_v_) {
    ++symbols_dropped_stale_;  // pre-reset stragglers after an encoder restart
    return out;
  }
  advance(wend - 1);  // a repair implies its whole window was sent
  if (ws < base_) {
    // References an evicted seq — unsolvable, and (if any covered seq is
    // still unknown) that seq is already booked abandoned by the horizon.
    ++symbols_dropped_stale_;
    return out;
  }

  Row r;
  r.payload.assign(payload, payload + ss);
  r.first_seen_ms = now_ms;
  uint8_t coeffs[sw::kMaxWindow];
  sw::repair_coeffs(h.repair_key, wl, coeffs);
  for (int i = 0; i < wl; ++i) {
    const uint64_t s = ws + static_cast<uint64_t>(i);
    const auto kit = known_.find(s);
    if (kit != known_.end())
      gf::lincomb(r.payload.data(), kit->second.data(), coeffs[i], ss);
    else
      r.coeffs.emplace(s, coeffs[i]);
  }
  std::vector<std::pair<uint64_t, std::vector<uint8_t>>> solved;
  insert_row(std::move(r), solved);
  for (auto& [sv, sym] : solved) ingest(sv, std::move(sym), /*source=*/false, out);
  return out;
}

int SwDecoder::expire_rows_older_than(uint64_t deadline_ms, uint64_t now_ms) {
  // now_ms > guard against underflow — bodies routinely carry stamps newer
  // than the poll clock (bench 2026-07-13).
  int dropped = 0;
  for (auto it = rows_.begin(); it != rows_.end();) {
    if (now_ms > it->second.first_seen_ms &&
        now_ms - it->second.first_seen_ms > deadline_ms) {
      it = rows_.erase(it);
      ++dropped;
    } else {
      ++it;
    }
  }
  return dropped;
}

}  // namespace mabur
