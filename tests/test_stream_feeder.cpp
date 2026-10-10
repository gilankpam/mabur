// StreamFeeder against a fake decoder (spec 2026-10-10-h265-slices §6.3/§6.5):
// event order, START/append/LAST, every fallback, abort. Every kOpen/kGrow
// here ends on a NAL boundary -- ring v4's writer contract for split AUs
// (Tasks 4/5), which the feeder relies on.
#include <cstdint>
#include <string>
#include <vector>

#include "au_ring.h"
#include "mabur/frame_wire.h"
#include "mtest.h"
#include "ring_client.h"
#include "stream_feeder.h"

using maburplay::AuEvent;
using maburplay::AuEventKind;
using maburplay::StreamFeeder;
using maburplay::StreamOff;

namespace {

struct FakeDecoder : maburplay::StreamDecoder {
  struct Call { char op; std::vector<uint8_t> b; uint8_t n; uint32_t pts; bool last; };
  std::vector<Call> calls;
  int probes = 0;
  bool probe_ok = true, start_ok = true, append_ok = true;
  bool probe() override { ++probes; return probe_ok; }
  bool start(const uint8_t* p, size_t n, uint8_t ns, uint32_t pts) override {
    calls.push_back({'S', std::vector<uint8_t>(p, p + n), ns, pts, false});
    return start_ok;
  }
  bool append(const uint8_t* p, size_t n, uint32_t pts, bool last) override {
    if (n > (1u << 20)) {   // an underflowed length: record it, never read it
      calls.push_back({'!', {}, 0, pts, last});
      return append_ok;
    }
    calls.push_back({'A', std::vector<uint8_t>(p, p + n), 0, pts, last});
    return append_ok;
  }
  void abort(uint32_t pts) override { calls.push_back({'X', {}, 0, pts, true}); }
  std::vector<uint8_t> fed() const {   // every byte handed over, in order
    std::vector<uint8_t> b;
    for (const auto& c : calls) b.insert(b.end(), c.b.begin(), c.b.end());
    return b;
  }
};

// One Annex-B AU: an optional prefix SEI, then n TRAIL_R slices (header
// 02 01); slice k = 4-byte start code + header + (20 + 7k) bytes of 0x40+k.
// *starts gets each slice's start-code offset.
std::vector<uint8_t> make_au(int n, bool sei, std::vector<size_t>* starts) {
  std::vector<uint8_t> au;
  if (sei) {
    const uint8_t s[] = {0, 0, 0, 1, 0x4E, 0x01, 0x05, 0x01, 0xAA, 0x80};
    au.insert(au.end(), s, s + sizeof s);
  }
  for (int k = 0; k < n; ++k) {
    starts->push_back(au.size());
    const uint8_t sc[] = {0, 0, 0, 1, 0x02, 0x01};
    au.insert(au.end(), sc, sc + sizeof sc);
    au.insert(au.end(), static_cast<size_t>(20 + 7 * k), static_cast<uint8_t>(0x40 + k));
  }
  return au;
}

std::vector<uint8_t> prefix(const std::vector<uint8_t>& au, size_t n) {
  return std::vector<uint8_t>(au.begin(), au.begin() + static_cast<long>(n));
}
std::vector<uint8_t> range(const std::vector<uint8_t>& au, size_t a, size_t b) {
  return std::vector<uint8_t>(au.begin() + static_cast<long>(a), au.begin() + static_cast<long>(b));
}

AuEvent ev(AuEventKind kind, uint64_t rec, uint32_t pts, uint8_t ns, std::vector<uint8_t> bytes,
           uint8_t flags = 0, bool flush = false, bool aborted = false) {
  AuEvent e{};
  e.meta.rec_no = rec;
  e.meta.pts_us = pts;
  e.meta.nslices = ns;
  e.meta.flags = flags;
  e.meta.len = static_cast<uint32_t>(bytes.size());
  e.au = std::move(bytes);
  e.flush_before = flush;
  e.kind = kind;
  e.aborted = aborted;
  return e;
}

constexpr uint8_t kComplete = maburgs::kRecFlagComplete;
constexpr uint8_t kSalvaged = maburgs::kRecFlagSliceSalvaged;

}  // namespace

TEST(slice_is_fed_on_the_grow_that_makes_it_whole) {   // Review Focus 6
  // maburgs publishes slice k when slice k+1's start code arrives (the
  // assembler's one hold-back), and valid_len then ends exactly at slice
  // k's end. The feeder must hand slice k over on THAT event -- one call
  // per grow -- not wait for a later one: that is the whole latency win.
  std::vector<size_t> s;
  const auto au = make_au(4, false, &s);
  FakeDecoder d;
  StreamFeeder f(true);
  f.set_decoder(&d);
  REQUIRE(f.off_reason() == StreamOff::kNone);
  f.on_open(ev(AuEventKind::kOpen, 5, 500, 4, prefix(au, s[1])), true);   // slice 0 whole
  REQUIRE(d.calls.size() == 1);
  CHECK(d.calls[0].op == 'S');
  CHECK(d.calls[0].b == prefix(au, s[1]));
  CHECK(d.calls[0].n == 4);
  CHECK(d.calls[0].pts == 500u);
  f.on_open(ev(AuEventKind::kGrow, 5, 500, 4, prefix(au, s[2])), true);   // slice 1 whole
  REQUIRE(d.calls.size() == 2);
  CHECK(d.calls[1].op == 'A');
  CHECK(d.calls[1].b == range(au, s[1], s[2]));
  CHECK(!d.calls[1].last);
  f.on_open(ev(AuEventKind::kGrow, 5, 500, 4, prefix(au, s[3])), true);   // slice 2 whole
  REQUIRE(d.calls.size() == 3);
  CHECK(d.calls[2].b == range(au, s[2], s[3]));
  f.on_open(ev(AuEventKind::kGrow, 5, 500, 4, au), true);                 // slice 3 whole, still open
  CHECK(d.calls.size() == 3);                                             // the last waits for the close
  CHECK(f.owns(5, 500));
  CHECK(f.on_close(ev(AuEventKind::kClose, 5, 500, 4, au, kComplete), true) ==
        StreamFeeder::Close::kStreamed);
  REQUIRE(d.calls.size() == 4);
  CHECK(d.calls[3].op == 'A');
  CHECK(d.calls[3].last);
  CHECK(d.calls[3].b == range(au, s[3], au.size()));
  CHECK(d.fed() == au);                       // every byte once, in order
  CHECK(f.streamed() == 1);
  CHECK(f.stream_aborted() == 0);
  CHECK(!f.owns(5, 500));
}

TEST(the_last_slice_waits_for_the_close) {
  std::vector<size_t> s;
  const auto au = make_au(4, false, &s);
  FakeDecoder d;
  StreamFeeder f(true);
  f.set_decoder(&d);
  f.on_open(ev(AuEventKind::kOpen, 2, 20, 4, au), true);   // every slice valid, still open
  REQUIRE(d.calls.size() == 3);                            // S + slices 1, 2
  for (const auto& c : d.calls) CHECK(!c.last);
  CHECK(f.on_close(ev(AuEventKind::kClose, 2, 20, 4, au, kComplete), true) ==
        StreamFeeder::Close::kStreamed);
  REQUIRE(d.calls.size() == 4);
  CHECK(d.calls[3].last);
}

TEST(parameter_sets_and_sei_ride_with_slice_zero) {
  std::vector<size_t> s;
  const auto au = make_au(4, true, &s);
  FakeDecoder d;
  StreamFeeder f(true);
  f.set_decoder(&d);
  CHECK(s[0] == 10);
  f.on_open(ev(AuEventKind::kOpen, 3, 30, 4, prefix(au, s[0])), true);   // the SEI alone
  CHECK(d.calls.empty());                                                 // no slice yet
  f.on_open(ev(AuEventKind::kGrow, 3, 30, 4, prefix(au, s[1])), true);
  REQUIRE(d.calls.size() == 1);
  CHECK(d.calls[0].b == prefix(au, s[1]));   // SEI + slice 0
}

TEST(passthrough_after_streaming_aborts_once_never_whole) {   // Review Focus 3
  std::vector<size_t> s;
  const auto au = make_au(4, false, &s);
  FakeDecoder d;
  StreamFeeder f(true);
  f.set_decoder(&d);
  f.on_open(ev(AuEventKind::kOpen, 5, 500, 4, prefix(au, s[2])), true);
  REQUIRE(d.calls.size() == 2);              // S + slice 1
  CHECK(f.owns(5, 500));
  // FrameStream gave up: passthrough, neither complete nor salvaged. The
  // closed record also holds the unaligned remainder (it ends mid-slice 2).
  CHECK(f.on_close(ev(AuEventKind::kClose, 5, 500, 4, prefix(au, s[2] + 30), 0), false) ==
        StreamFeeder::Close::kAborted);
  REQUIRE(d.calls.size() == 3);
  CHECK(d.calls[2].op == 'X');               // exactly one empty LAST ...
  CHECK(d.calls[2].pts == 500u);
  CHECK(d.fed() == prefix(au, s[2]));        // ... nothing past the last NAL-aligned valid_len ...
  CHECK(f.stream_aborted() == 1);            // ... counted ...
  CHECK(f.streamed() == 0);
  CHECK(!f.owns(5, 500));
}

TEST(open_while_previous_stream_unfinished_aborts_it_first) {   // Review Focus 5
  std::vector<size_t> s;
  const auto au = make_au(4, false, &s);
  FakeDecoder d;
  StreamFeeder f(true);
  f.set_decoder(&d);
  f.on_open(ev(AuEventKind::kOpen, 5, 500, 4, prefix(au, s[2])), true);
  REQUIRE(d.calls.size() == 2);
  // The close of rec 5 was lost; rec 6 opens.
  f.on_open(ev(AuEventKind::kOpen, 6, 517, 4, prefix(au, s[1])), true);
  REQUIRE(d.calls.size() == 4);
  CHECK(d.calls[2].op == 'X');
  CHECK(d.calls[2].pts == 500u);             // the old picture ends first
  CHECK(d.calls[3].op == 'S');
  CHECK(d.calls[3].pts == 517u);
  CHECK(f.stream_aborted() == 1);
  // rec 5's close straggles in after all: still never whole, nothing sent.
  CHECK(f.owns(5, 500));
  CHECK(f.on_close(ev(AuEventKind::kClose, 5, 500, 4, au, kComplete), true) ==
        StreamFeeder::Close::kAborted);
  CHECK(d.calls.size() == 4);
  CHECK(f.stream_aborted() == 1);            // counted once
  CHECK(f.on_close(ev(AuEventKind::kClose, 6, 517, 4, au, kComplete), true) ==
        StreamFeeder::Close::kStreamed);
}

TEST(discont_or_flush_au_is_never_streamed) {
  std::vector<size_t> s;
  const auto au = make_au(4, false, &s);
  FakeDecoder d;
  StreamFeeder f(true);
  f.set_decoder(&d);
  f.on_open(ev(AuEventKind::kOpen, 1, 10, 4, au, mabur::framewire::kFlagDiscont), true);
  CHECK(d.calls.empty());
  CHECK(!f.owns(1, 10));
  CHECK(f.on_close(ev(AuEventKind::kClose, 1, 10, 4, au, kComplete | mabur::framewire::kFlagDiscont, true),
                   true) == StreamFeeder::Close::kWhole);
  f.on_open(ev(AuEventKind::kOpen, 2, 20, 4, au, 0, /*flush preview*/ true), true);
  CHECK(d.calls.empty());
  CHECK(f.on_close(ev(AuEventKind::kClose, 2, 20, 4, au, kComplete, true), true) ==
        StreamFeeder::Close::kWhole);
}

TEST(unarmed_single_slice_disabled_or_unprobed_goes_whole) {
  std::vector<size_t> s;
  const auto au = make_au(4, false, &s);
  {
    FakeDecoder d;
    StreamFeeder f(true);
    f.set_decoder(&d);
    f.on_open(ev(AuEventKind::kOpen, 1, 10, 4, au), /*armed=*/false);
    CHECK(d.calls.empty());
    CHECK(f.on_close(ev(AuEventKind::kClose, 1, 10, 4, au, kComplete), true) ==
          StreamFeeder::Close::kWhole);
    f.on_open(ev(AuEventKind::kOpen, 2, 20, 1, au), true);   // nslices 1: whole AU
    CHECK(d.calls.empty());
  }
  {
    FakeDecoder d;
    StreamFeeder f(false);                     // [decoder] stream = false
    f.set_decoder(&d);
    CHECK(f.off_reason() == StreamOff::kConfig);
    CHECK(d.probes == 0);                      // not even probed
    f.on_open(ev(AuEventKind::kOpen, 1, 10, 4, au), true);
    CHECK(d.calls.empty());
  }
  {
    FakeDecoder d;
    d.probe_ok = false;
    StreamFeeder f(true);
    f.set_decoder(&d);
    CHECK(f.off_reason() == StreamOff::kProbe);
    f.on_open(ev(AuEventKind::kOpen, 1, 10, 4, au), true);
    CHECK(d.calls.empty());
  }
  {
    StreamFeeder f(true);
    f.set_decoder(nullptr);                    // null backend
    CHECK(f.off_reason() == StreamOff::kNoDecoder);
    f.on_open(ev(AuEventKind::kOpen, 1, 10, 4, au), true);
    CHECK(f.on_close(ev(AuEventKind::kClose, 1, 10, 4, au, kComplete), true) ==
          StreamFeeder::Close::kWhole);
  }
  CHECK(std::string(maburplay::stream_off_name(StreamOff::kNone)) == "on");
  CHECK(std::string(maburplay::stream_off_name(StreamOff::kNoDecoder)) == "off:no_decoder");
}

TEST(ring_aborted_close_ends_the_stream) {
  std::vector<size_t> s;
  const auto au = make_au(4, false, &s);
  FakeDecoder d;
  StreamFeeder f(true);
  f.set_decoder(&d);
  f.on_open(ev(AuEventKind::kOpen, 7, 70, 4, prefix(au, s[2])), true);
  CHECK(f.on_close(ev(AuEventKind::kClose, 7, 70, 4, {}, 0, false, /*aborted=*/true), false) ==
        StreamFeeder::Close::kAborted);
  REQUIRE(d.calls.size() == 3);
  CHECK(d.calls[2].op == 'X');
  CHECK(f.stream_aborted() == 1);
}

TEST(flush_mid_stream_aborts_and_the_close_never_goes_whole) {
  std::vector<size_t> s;
  const auto au = make_au(4, false, &s);
  FakeDecoder d;
  StreamFeeder f(true);
  f.set_decoder(&d);
  f.on_open(ev(AuEventKind::kOpen, 8, 80, 4, prefix(au, s[1])), true);
  f.on_flush();                              // watchdog/discont reset with a picture open
  REQUIRE(d.calls.size() == 2);
  CHECK(d.calls[1].op == 'X');
  f.on_open(ev(AuEventKind::kGrow, 8, 80, 4, au), true);   // late growth: ignored
  CHECK(d.calls.size() == 2);
  CHECK(f.on_close(ev(AuEventKind::kClose, 8, 80, 4, au, kComplete), true) ==
        StreamFeeder::Close::kAborted);
  CHECK(d.calls.size() == 2);
  CHECK(f.stream_aborted() == 1);
}

TEST(slice_count_mismatch_at_close_aborts) {
  std::vector<size_t> s;
  const auto au = make_au(5, false, &s);     // 5 slices on the wire, 4 announced
  FakeDecoder d;
  StreamFeeder f(true);
  f.set_decoder(&d);
  f.on_open(ev(AuEventKind::kOpen, 9, 90, 4, au), true);
  REQUIRE(d.calls.size() == 3);              // never more than n - 1 slices before the close
  CHECK(f.on_close(ev(AuEventKind::kClose, 9, 90, 4, au, kComplete), true) ==
        StreamFeeder::Close::kAborted);
  CHECK(d.calls.back().op == 'X');
}

TEST(close_shorter_than_what_was_fed_aborts) {
  // A writer-contract violation (the closed record is shorter than the
  // prefix already handed to the decoder) must abort the picture -- one
  // empty LAST -- not underflow the last piece's length into a wild read.
  std::vector<size_t> s;
  const auto au = make_au(4, false, &s);
  FakeDecoder d;
  StreamFeeder f(true);
  f.set_decoder(&d);
  f.on_open(ev(AuEventKind::kOpen, 6, 60, 4, au), true);
  REQUIRE(d.calls.size() == 3);              // S + slices 1, 2: fed up to s[3]
  const auto shorter = prefix(au, s[2]);     // decodable-looking, but < fed
  CHECK(f.on_close(ev(AuEventKind::kClose, 6, 60, 4, shorter, kComplete), true) ==
        StreamFeeder::Close::kAborted);
  REQUIRE(d.calls.size() == 4);              // exactly one more call: the abort
  CHECK(d.calls[3].op == 'X');
  CHECK(d.calls[3].b.empty());
  for (const auto& c : d.calls) CHECK(c.op != '!');
  CHECK(f.stream_aborted() == 1);
  CHECK(f.streamed() == 0);
}

TEST(refused_start_leaves_the_au_to_the_whole_path) {
  std::vector<size_t> s;
  const auto au = make_au(4, false, &s);
  FakeDecoder d;
  d.start_ok = false;
  StreamFeeder f(true);
  f.set_decoder(&d);
  f.on_open(ev(AuEventKind::kOpen, 4, 40, 4, au), true);
  REQUIRE(d.calls.size() == 1);              // the refused START, nothing after it
  CHECK(!f.owns(4, 40));
  CHECK(f.on_close(ev(AuEventKind::kClose, 4, 40, 4, au, kComplete), true) ==
        StreamFeeder::Close::kWhole);
  CHECK(f.stream_aborted() == 0);
}

TEST(refused_append_aborts_and_the_close_is_not_whole) {
  std::vector<size_t> s;
  const auto au = make_au(4, false, &s);
  FakeDecoder d;
  d.append_ok = false;
  StreamFeeder f(true);
  f.set_decoder(&d);
  f.on_open(ev(AuEventKind::kOpen, 3, 33, 4, prefix(au, s[2])), true);
  REQUIRE(d.calls.size() == 3);              // S, refused A, X
  CHECK(d.calls[2].op == 'X');
  CHECK(f.on_close(ev(AuEventKind::kClose, 3, 33, 4, au, kComplete), true) ==
        StreamFeeder::Close::kAborted);
  CHECK(d.calls.size() == 3);
}

TEST(salvaged_close_streams_and_is_counted) {
  std::vector<size_t> s;
  const auto au = make_au(4, false, &s);
  FakeDecoder d;
  StreamFeeder f(true);
  f.set_decoder(&d);
  f.on_open(ev(AuEventKind::kOpen, 6, 60, 4, prefix(au, s[1])), true);
  CHECK(f.on_close(ev(AuEventKind::kClose, 6, 60, 4, au, kSalvaged), true) ==
        StreamFeeder::Close::kStreamed);
  CHECK(f.streamed() == 1);
  CHECK(f.streamed_salvaged() == 1);
  CHECK(d.calls.back().last);
  CHECK(d.fed() == au);
}

TEST(decoder_rebuild_keeps_the_aborted_picture_aborted) {
  // Task 9 rebuilds the decoder after a watchdog fires: on_flush() aborts
  // the open picture first, then set_decoder(new). That picture's close
  // must still report kAborted -- set_decoder must not clear ended_.
  std::vector<size_t> s;
  const auto au = make_au(4, false, &s);
  FakeDecoder d1, d2;
  StreamFeeder f(true);
  f.set_decoder(&d1);
  f.on_open(ev(AuEventKind::kOpen, 11, 110, 4, prefix(au, s[2])), true);
  REQUIRE(d1.calls.size() == 2);             // S + slice 1
  f.on_flush();
  REQUIRE(d1.calls.size() == 3);
  CHECK(d1.calls[2].op == 'X');
  f.set_decoder(&d2);
  CHECK(f.on_close(ev(AuEventKind::kClose, 11, 110, 4, au, kComplete), true) ==
        StreamFeeder::Close::kAborted);
  CHECK(d2.calls.empty());
  CHECK(f.whole_submits() == 0);
}

TEST(flush_before_start_never_starts) {
  // A picture seen open (eligible) but not yet started -- only its leading
  // prefix SEI is valid -- when the decoder is flushed (watchdog reset, or a
  // flush_before AU). Its later growth must not send START on the decoder
  // that was just flushed: on_flush() ends its eligibility too, and its
  // close takes the whole-AU path.
  std::vector<size_t> s;
  const auto au = make_au(4, true, &s);
  FakeDecoder d;
  StreamFeeder f(true);
  f.set_decoder(&d);
  f.on_open(ev(AuEventKind::kOpen, 12, 120, 4, prefix(au, s[0])), true);   // SEI only
  CHECK(d.calls.empty());
  f.on_flush();
  CHECK(d.calls.empty());                    // nothing started: nothing to abort
  f.on_open(ev(AuEventKind::kGrow, 12, 120, 4, prefix(au, s[2])), true);   // slices 0-1 valid
  CHECK(d.calls.empty());                    // never START after the flush
  CHECK(!f.owns(12, 120));
  CHECK(f.on_close(ev(AuEventKind::kClose, 12, 120, 4, au, kComplete), true) ==
        StreamFeeder::Close::kWhole);
  CHECK(d.calls.empty());
  CHECK(f.stream_aborted() == 0);
}

TEST(stop_new_lets_the_streaming_picture_finish_and_starts_no_other) {
  std::vector<size_t> s;
  const auto au = make_au(4, false, &s);
  FakeDecoder d;
  StreamFeeder f(true);
  f.set_decoder(&d);
  f.on_open(ev(AuEventKind::kOpen, 7, 70, 4, prefix(au, s[2])), true);   // S + slice 1
  REQUIRE(f.streaming());
  CHECK(f.streaming_pts() == 70u);
  f.stop_new();
  f.on_open(ev(AuEventKind::kGrow, 7, 70, 4, au), true);                 // slice 2 still goes in
  CHECK(d.calls.size() == 3);
  CHECK(f.on_close(ev(AuEventKind::kClose, 7, 70, 4, au, kComplete), true) ==
        StreamFeeder::Close::kStreamed);
  CHECK(!f.streaming());
  CHECK(d.fed() == au);
  f.on_open(ev(AuEventKind::kOpen, 8, 80, 4, au), true);                 // the next picture
  CHECK(d.calls.size() == 4);                                            // never started
  CHECK(!f.owns(8, 80));
  CHECK(f.on_close(ev(AuEventKind::kClose, 8, 80, 4, au, kComplete), true) ==
        StreamFeeder::Close::kWhole);
}

TEST(drain_stream_finishes_the_picture_on_its_close) {
  std::vector<size_t> s;
  const auto au = make_au(4, false, &s);
  FakeDecoder d;
  StreamFeeder f(true);
  f.set_decoder(&d);
  f.on_open(ev(AuEventKind::kOpen, 7, 70, 4, prefix(au, s[2])), true);
  uint64_t now = 5000;
  int pumps = 0;
  const auto r = maburplay::drain_stream(
      f,
      [&](int ms) {
        now += static_cast<uint64_t>(ms);
        if (++pumps == 3) f.on_close(ev(AuEventKind::kClose, 7, 70, 4, au, kComplete), true);
      },
      [&] { return now; }, 100);
  CHECK(r.kind == maburplay::DrainResult::kFinished);
  CHECK(r.pts == 70u);
  CHECK(r.waited_ms == 3);
  REQUIRE(!d.calls.empty());
  CHECK(d.calls.back().op == 'A');
  CHECK(d.calls.back().last);
  CHECK(!d.calls.back().b.empty());   // its own last slice, not the empty LAST
  CHECK(f.stream_aborted() == 0);
}

TEST(drain_stream_ends_the_picture_once_when_its_close_never_comes) {
  std::vector<size_t> s;
  const auto au = make_au(4, false, &s);
  FakeDecoder d;
  StreamFeeder f(true);
  f.set_decoder(&d);
  f.on_open(ev(AuEventKind::kOpen, 7, 70, 4, prefix(au, s[2])), true);
  uint64_t now = 0;
  const auto r = maburplay::drain_stream(
      f, [&](int ms) { now += static_cast<uint64_t>(ms); }, [&] { return now; }, 20);
  CHECK(r.kind == maburplay::DrainResult::kAborted);
  CHECK(r.waited_ms == 20);
  CHECK(d.calls.back().op == 'X');
  CHECK(f.stream_aborted() == 1);
  CHECK(f.on_close(ev(AuEventKind::kClose, 7, 70, 4, au, kComplete), true) ==
        StreamFeeder::Close::kAborted);   // its straggling close never goes whole
}

TEST(drain_stream_is_idle_when_nothing_streams) {
  FakeDecoder d;
  StreamFeeder f(true);
  f.set_decoder(&d);
  int pumps = 0;
  const auto r = maburplay::drain_stream(f, [&](int) { ++pumps; }, [] { return uint64_t{0}; }, 100);
  CHECK(r.kind == maburplay::DrainResult::kIdle);
  CHECK(pumps == 0);
  CHECK(d.calls.empty());
}

TEST(drain_stream_reports_an_aborted_close_as_aborted) {
  std::vector<size_t> s;
  const auto au = make_au(4, false, &s);
  FakeDecoder d;
  StreamFeeder f(true);
  f.set_decoder(&d);
  f.on_open(ev(AuEventKind::kOpen, 7, 70, 4, prefix(au, s[2])), true);
  uint64_t now = 0;
  const auto r = maburplay::drain_stream(
      f,
      [&](int ms) {
        now += static_cast<uint64_t>(ms);
        f.on_close(ev(AuEventKind::kClose, 7, 70, 4, {}, 0, false, /*aborted=*/true), false);
      },
      [&] { return now; }, 100);
  CHECK(r.kind == maburplay::DrainResult::kAborted);
  CHECK(d.calls.back().op == 'X');
}

MTEST_MAIN
