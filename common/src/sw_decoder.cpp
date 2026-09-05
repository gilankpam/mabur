#include "mabur/sw_decoder.h"

#include <algorithm>

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
  recovered_await_src_.clear();
  newest_v_ = v;
  base_ = v;
  arr_.reset(v);
  ++resets_;
  wm_open_ = false;
  wm_valid_ = false;
}

void SwDecoder::mark_transition() {
  if (!have_seq_) {  // no traffic yet: nothing to attribute
    wm_open_ = false;
    wm_valid_ = false;
    return;
  }
  wm_ = newest_v_;
  wm_valid_ = true;
  wm_open_ = true;
}

// The floor below which a seq is genuinely unrecoverable (older than the
// horizon behind the newest symbol seen). This — NOT base_ — is what the
// admit/drop checks must use. base_ only catches up to newest_v_ - horizon_
// after the first `horizon` symbols; for that opening window base_ sits AT
// the first source seen, so any earlier-or-reordered source (leading loss in
// a join burst, or merely a lower seq that arrived after a higher one at
// bpb=1 where each symbol is its own air frame) tested `< base_` and was
// dropped stale — never delivered, recovered, nor abandoned. Whole RTP
// packets vanished with every loss counter frozen (hardware 2026-07-15,
// bpb=1 big-symbol video corruption). Using the horizon floor admits them
// during the join window and is identical to base_ in steady state.
uint64_t SwDecoder::live_floor() const { return newest_v_ - horizon_; }

void SwDecoder::advance(uint64_t newest_candidate) {
  if (newest_candidate <= newest_v_) return;
  newest_v_ = newest_candidate;
  const uint64_t nb = newest_v_ - horizon_;  // vseqs start at 2^32 >> horizon
  if (nb <= base_) return;
  // Every seq in [base_, nb) that never became known is lost for good.
  // Evict all known seqs below nb, but only those in [base_, nb) offset the
  // abandoned count: seqs below base_ (delivered/recovered after arriving
  // reordered-earlier than the join anchor, via live_floor()) were never in
  // the [base_, nb) accounting span, so counting them would underflow it.
  uint64_t known_in_range = 0;
  uint64_t known_stale = 0;
  // Stale span: [base_, stale_end). Open boundary => the whole eviction
  // range is stale; closed+valid => seqs <= wm_; inactive => empty span.
  const uint64_t stale_end =
      wm_open_ ? nb : (wm_valid_ ? std::min(nb, wm_ + 1) : base_);
  for (auto it = known_.begin(); it != known_.end() && it->first < nb;) {
    if (it->first >= base_) {
      ++known_in_range;
      if (it->first < stale_end) ++known_stale;
    }
    it = known_.erase(it);
  }
  syms_abandoned_ += (nb - base_) - known_in_range;
  if (stale_end > base_)
    syms_abandoned_stale_ += (stale_end - base_) - known_stale;
  // A recovered seq evicted before its direct copy showed stays "recovered"
  // for good: within the horizon the channel never delivered it.
  recovered_await_src_.erase(recovered_await_src_.begin(),
                             recovered_await_src_.lower_bound(nb));
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
    if (s < live_floor() || known_.count(s)) continue;
    unpack_symbol(payload.data(), out);
    if (count_as_source) {
      ++syms_delivered_;
    } else {
      ++syms_recovered_;
      recovered_await_src_.insert(s);
    }
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
                                                        uint64_t now_ms, SwBoundary b) {
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
    if (b == SwBoundary::kPre && wm_valid_ && v > wm_) wm_ = v;
    if (b == SwBoundary::kPost && wm_open_) {
      wm_open_ = false;
      if (wm_valid_ && v > 0 && v - 1 > wm_) wm_ = v - 1;
    }
    // Arrival accounting BEFORE the dedup/stale early-return: a second-card
    // copy still sets the heard bit (idempotent), a copy behind the settle
    // line counts late.
    arr_.on_source(v, arr_stale_end());
    arr_.advance(v, arr_stale_end());
    if (v < live_floor() || known_.count(v)) {
      // First direct copy of a repair-recovered symbol: the channel did
      // deliver it, the repair just won the race. Not a stale dup.
      if (recovered_await_src_.erase(v))
        ++syms_recovered_arrived_;
      else
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
  if (wl > repair_window_hwm_) repair_window_hwm_ = wl;
  const uint64_t wend = ws + static_cast<uint64_t>(wl);  // one past last covered
  if (wend > newest_v_ + static_cast<uint64_t>(kResetSpan) ||
      wend + static_cast<uint64_t>(kResetSpan) < newest_v_) {
    ++symbols_dropped_stale_;  // pre-reset stragglers after an encoder restart
    return out;
  }
  if (b == SwBoundary::kPre && wm_valid_ && wend - 1 > wm_) wm_ = wend - 1;
  advance(wend - 1);  // a repair implies its whole window was sent
  arr_.advance(wend - 1, arr_stale_end());
  if (ws < live_floor()) {
    // References a seq older than the horizon — unsolvable, and (if any
    // covered seq is still unknown) that seq is already booked abandoned.
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
