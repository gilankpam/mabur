#include <cstdint>
#include <vector>

#include "mabur/sw_decoder.h"
#include "mabur/sw_encoder.h"
#include "mabur/sw_wire.h"
#include "mtest.h"
#include "vectors.h"

using namespace mabur;

namespace {
std::vector<uint8_t> pat(size_t n, int seed) {
  std::vector<uint8_t> v(n);
  for (size_t i = 0; i < n; ++i)
    v[i] = static_cast<uint8_t>((i * 31 + static_cast<size_t>(seed) * 17 + 7) & 0xFF);
  return v;
}
sw::SwHeader hdr(const std::vector<uint8_t>& env) {
  sw::SwHeader h;
  CHECK(sw::parse_header(env.data(), env.size(), &h));
  return h;
}
// Feed packets sized to seal exactly one symbol each (symbol_size 64:
// payload 62 + 2B length prefix fills the symbol; the NEXT packet seals it).
std::vector<std::vector<uint8_t>> feed(SwEncoder& e, int n_packets) {
  std::vector<std::vector<uint8_t>> out;
  for (int i = 0; i < n_packets; ++i) {
    auto p = pat(62, i);
    for (auto& env : e.add_packet(p.data(), p.size())) out.push_back(std::move(env));
  }
  return out;
}
}  // namespace

TEST(source_envelopes_carry_monotonic_seq) {
  SwEncoder e(SwConfig{64, 8, 0.0});
  auto envs = feed(e, 5);          // 5 full-size packets: symbols 0..3 sealed
  auto tail = e.flush();           // seals the 5th + one tail repair
  for (auto& t : tail) envs.push_back(std::move(t));
  // 5 sources then exactly one tail repair (flush's burst-tail protection
  // fires even at overhead 0 — it is gated on seals, not credit).
  CHECK(envs.size() == 6);
  for (uint32_t i = 0; i < 5; ++i) {
    auto h = hdr(envs[i]);
    CHECK(!h.repair);
    CHECK(h.seq == i);
    CHECK(h.symbol_size == 64);
    CHECK(envs[i].size() == sw::kSwHeaderLen + 64);
  }
  CHECK(hdr(envs[5]).repair);
  CHECK(e.repairs_out() == 1);
}

TEST(credit_emits_one_repair_per_source_at_overhead_1) {
  SwEncoder e(SwConfig{64, 8, 1.0});
  auto envs = feed(e, 5);          // 4 seals -> 4 source + 4 repair
  int src = 0, rep = 0;
  for (auto& env : envs) (hdr(env).repair ? rep : src)++;
  CHECK(src == 4);
  CHECK(rep == 4);
  CHECK(e.sources_out() == 4);
  CHECK(e.repairs_out() == 4);
}

TEST(credit_emits_every_other_at_overhead_half) {
  SwEncoder e(SwConfig{64, 8, 0.5});
  auto envs = feed(e, 9);          // 8 seals -> 8 source + 4 repair
  int rep = 0;
  for (auto& env : envs) if (hdr(env).repair) ++rep;
  CHECK(rep == 4);
}

TEST(repair_window_slides_and_caps) {
  SwEncoder e(SwConfig{64, 4, 1.0});
  auto envs = feed(e, 7);          // 6 seals; last repair after seq 5 sealed
  sw::SwHeader last{};
  for (auto& env : envs) { auto h = hdr(env); if (h.repair) last = h; }
  // After sealing seqs 0..5 with window 4, the newest repair covers [2, 6).
  CHECK(last.window_len == 4);
  CHECK(last.seq == 2);
}

TEST(repair_keys_increment) {
  SwEncoder e(SwConfig{64, 8, 1.0});
  auto envs = feed(e, 4);
  std::vector<uint32_t> keys;
  for (auto& env : envs) { auto h = hdr(env); if (h.repair) keys.push_back(h.repair_key); }
  CHECK(keys.size() == 3);
  CHECK(keys[0] == 0);
  CHECK(keys[1] == 1);
  CHECK(keys[2] == 2);
}

TEST(flush_seals_partial_and_emits_tail_repair_once) {
  SwEncoder e(SwConfig{64, 8, 0.25});
  auto p = pat(10, 1);
  CHECK(e.add_packet(p.data(), p.size()).empty());  // partial symbol pending
  CHECK(e.has_pending());
  auto f1 = e.flush();
  // source (sealed partial) + exactly one tail repair
  CHECK(f1.size() == 2);
  CHECK(!hdr(f1[0]).repair);
  CHECK(hdr(f1[1]).repair);
  CHECK(!e.has_pending());
  auto f2 = e.flush();               // idle flush: nothing new, no repair spam
  CHECK(f2.empty());
}

TEST(oversize_packet_dropped_counted) {
  SwEncoder e(SwConfig{64, 8, 0.5});
  auto p = pat(63, 0);               // > max_packet_size() == 62
  CHECK(e.add_packet(p.data(), p.size()).empty());
  CHECK(e.oversize_drops() == 1);
}

TEST(vectors_byte_exact) {
  auto j = mtest::load_json(std::string(MABUR_VECTOR_DIR) + "/sw.json");
  for (const auto& c : j["cases"]) {
    SwEncoder e(SwConfig{c["symbol_size"], c["window"], c["overhead"]});
    std::vector<std::string> got;
    for (const auto& ph : c["packets"]) {
      auto p = mtest::unhex(ph.get<std::string>());
      for (auto& env : e.add_packet(p.data(), p.size())) got.push_back(mtest::hex(env));
    }
    size_t i = 0;
    for (const auto& s : c["stream"]) CHECK(got.at(i++) == s.get<std::string>());
    CHECK(i == got.size());
    std::vector<std::string> fl;
    for (auto& env : e.flush()) fl.push_back(mtest::hex(env));
    CHECK(fl.size() == c["flush"].size());
    for (size_t k = 0; k < fl.size(); ++k) CHECK(fl[k] == c["flush"][k].get<std::string>());
  }
}

TEST(packets_never_span_symbols) {
  // 40B packet then 30B packet: 2+40=42 used, 2+30 doesn't fit in the
  // remaining 22 -> first symbol seals padded, second packet starts fresh.
  SwEncoder e(SwConfig{64, 8, 0.0});
  auto a = pat(40, 0), b = pat(30, 1);
  CHECK(e.add_packet(a.data(), a.size()).empty());
  auto envs = e.add_packet(b.data(), b.size());
  CHECK(envs.size() == 1);           // sealed symbol 0
  const uint8_t* sym = envs[0].data() + sw::kSwHeaderLen;
  CHECK(sym[0] == 40 && sym[1] == 0);                 // len prefix
  for (int i = 0; i < 40; ++i) CHECK(sym[2 + i] == a[static_cast<size_t>(i)]);
  for (int i = 42; i < 64; ++i) CHECK(sym[i] == 0);   // zero pad, no spill
}

// The sink form of add_packet/flush must hand out the same envelopes, in
// the same order, as the vector form (sync mode), and the two-span
// overload must equal one add_packet of the concatenation — that is what
// lets the fragment header and the frame bytes go straight into the
// envelope without a fragment vector in between.
TEST(sink_form_matches_vector_form) {
  SwConfig cfg{32, 8, 0.5};
  SwEncoder a(cfg), b(cfg), c(cfg);
  std::vector<std::vector<uint8_t>> want, got_sink, got_span;
  auto sink_b = [&](const uint8_t* env, size_t n) { got_sink.emplace_back(env, env + n); };
  auto sink_c = [&](const uint8_t* env, size_t n) { got_span.emplace_back(env, env + n); };
  for (int i = 0; i < 40; ++i) {
    size_t len = 1 + static_cast<size_t>(i * 13 % 29);
    std::vector<uint8_t> pkt(len);
    for (size_t k = 0; k < len; ++k) pkt[k] = static_cast<uint8_t>(i + k);
    for (auto& e : a.add_packet(pkt.data(), len)) want.push_back(e);
    b.add_packet(pkt.data(), len, sink_b);
    size_t split = len / 2;
    c.add_packet(pkt.data(), split, pkt.data() + split, len - split, sink_c);
  }
  for (auto& e : a.flush()) want.push_back(e);
  b.flush(sink_b);
  c.flush(sink_c);
  CHECK(got_sink == want);
  CHECK(got_span == want);
  CHECK(!want.empty());
}
// repairs_with_frame: a frame's credit is held to its end, spent there
// rounded up, and the remainder forgiven.
TEST(repairs_with_frame_holds_the_credit_to_the_frame_end) {
  SwEncoder e(SwConfig{64, 32, 0.5, true});
  auto count_rep = [](const std::vector<std::vector<uint8_t>>& v) {
    int n = 0;
    for (auto& env : v) n += hdr(env).repair ? 1 : 0;
    return n;
  };
  auto a = feed(e, 5);                 // 4 sealed, credit 2.0: held
  CHECK(count_rep(a) == 0);
  auto ta = e.flush();                 // 5th sealed, 2.5: three repairs
  CHECK(ta.size() == 4 && !hdr(ta[0]).repair);
  CHECK(count_rep(ta) == 3);
  for (size_t i = 1; i < ta.size(); ++i) {
    CHECK(hdr(ta[i]).seq + hdr(ta[i]).window_len == 5);  // each covers the whole frame
    CHECK(hdr(ta[i]).window_len == 5);
  }
  auto b = feed(e, 3);
  CHECK(count_rep(b) == 0);
  auto tb = e.flush();                 // 1.5, nothing carried: two
  CHECK(count_rep(tb) == 2);
  CHECK(e.flush().empty());            // an idle re-flush adds nothing
}

TEST(repairs_with_frame_windows_still_span_frames) {
  SwEncoder e(SwConfig{64, 32, 0.5, true});
  feed(e, 4);
  e.flush();
  auto b = feed(e, 2);
  for (auto& t : e.flush()) b.push_back(std::move(t));
  bool reaches_back = false;
  for (auto& env : b) {
    auto h = hdr(env);
    if (h.repair && h.seq < 4) reaches_back = true;
  }
  CHECK(reaches_back);
}

// A frame longer than the window spends its credit a window at a time, so
// no symbol is left without repairs built over it.
TEST(repairs_with_frame_long_frame_spends_per_window) {
  SwEncoder e(SwConfig{64, 8, 0.5, true});
  auto out = feed(e, 20);
  for (auto& t : e.flush()) out.push_back(std::move(t));
  uint32_t covered_to = 0;
  int reps = 0;
  for (auto& env : out) {
    auto h = hdr(env);
    if (!h.repair) continue;
    ++reps;
    CHECK(h.window_len == 8);
    CHECK(h.seq <= covered_to);        // no gap between repair windows
    covered_to = std::max(covered_to, h.seq + h.window_len);
  }
  CHECK(covered_to == 20);
  CHECK(reps == 10);                   // 4 + 4 at the window marks, 2 at the end
}

// The point of the mode: a loss late in a frame is repaired by envelopes
// that arrive with that frame, before anything of the next frame.
TEST(repairs_with_frame_loss_recovers_within_the_frame) {
  const SwConfig cfg{64, 32, 0.25, true};
  SwEncoder e(cfg);
  auto prev = feed(e, 8);
  for (auto& t : e.flush()) prev.push_back(std::move(t));
  auto a = feed(e, 8);
  for (auto& t : e.flush()) a.push_back(std::move(t));
  SwDecoder d(cfg);
  size_t got = 0;
  for (auto& env : prev) got += d.add_symbol(env.data(), env.size(), 1000).size();
  int src = 0;
  for (auto& env : a) {
    if (!hdr(env).repair && ++src >= 7) continue;  // lose frame A's last two sources
    got += d.add_symbol(env.data(), env.size(), 1000).size();
  }
  CHECK(got == 16);
  CHECK(d.syms_recovered() == 2);
}

TEST(default_mode_windows_still_span_frames) {
  SwEncoder e(SwConfig{64, 32, 1.0});
  feed(e, 4);
  e.flush();
  auto b = feed(e, 3);
  bool reaches_back = false;
  for (auto& env : b) {
    auto h = hdr(env);
    if (h.repair && h.seq < 4) reaches_back = true;
  }
  CHECK(reaches_back);
}

MTEST_MAIN
