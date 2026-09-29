#include <cstdio>
#include <fstream>
#include <iterator>
#include <memory>
#include <string>
#include <vector>

#include "mabur/raw_dvr.h"
#include "mtest.h"

using mabur::RawDvr;

namespace {

std::string scratch(const char* name) { return std::string(MABUR_TEST_SCRATCH_DIR) + "/" + name; }

bool exists(const std::string& p) {
  std::ifstream f(p, std::ios::binary);
  return f.good();
}

void nal(std::vector<uint8_t>& au, std::initializer_list<uint8_t> bytes) {
  au.insert(au.end(), {0, 0, 0, 1});
  au.insert(au.end(), bytes);
}

// A GDR refresh start: VPS+SPS+PPS + a TRAIL_R slice (never an IRAP).
// SPS = the 720p x265 SPS from test_hevc_params.cpp, so dimensions parse.
std::vector<uint8_t> param_au(uint8_t tag) {
  std::vector<uint8_t> au;
  nal(au, {0x40, 0x01, 0x0c, 0x01, 0xff, 0xff});  // VPS (content irrelevant to RawDvr)
  au.insert(au.end(), {0, 0, 1});
  const uint8_t sps[] = {0x42, 0x01, 0x01, 0x01, 0x60, 0x00, 0x00, 0x03, 0x00, 0x90, 0x00, 0x00,
                         0x03, 0x00, 0x00, 0x03, 0x00, 0x78, 0xa0, 0x02, 0x80, 0x80, 0x2d, 0x16,
                         0x59, 0x59, 0xa4, 0x93, 0x2b, 0xc0, 0x5a, 0x02, 0x00, 0x00, 0x03, 0x00,
                         0x02, 0x00, 0x00, 0x03, 0x00, 0x78, 0x10};
  au.insert(au.end(), std::begin(sps), std::end(sps));
  nal(au, {0x44, 0x01, 0xc1, 0x72});              // PPS
  nal(au, {0x02, 0x01, tag, tag});                // TRAIL_R
  return au;
}

// A real IDR: VPS+SPS+PPS + an IDR_W_RADL slice. The only AU a recording
// may begin on, and the only sync sample.
std::vector<uint8_t> idr_au(uint8_t tag) {
  std::vector<uint8_t> au = param_au(tag);
  au[au.size() - 4] = 0x26;                       // TRAIL_R slice -> IDR_W_RADL
  return au;
}

std::vector<uint8_t> p_au(uint8_t tag) {
  std::vector<uint8_t> au;
  nal(au, {0x02, 0x01, tag, tag, tag});
  return au;
}

void feed(RawDvr& d, const std::vector<uint8_t>& au, uint32_t pts, bool complete = true) {
  d.feed(au.data(), au.size(), pts, complete);
}

}  // namespace

TEST(param_set_detection) {
  const auto k = param_au(1), p = p_au(2);
  CHECK(mabur::au_has_param_sets(k.data(), k.size()));
  CHECK(!mabur::au_has_param_sets(p.data(), p.size()));
}

TEST(irap_detection) {
  const auto i = idr_au(1), g = param_au(2), p = p_au(3);
  CHECK(mabur::au_is_irap(i.data(), i.size()));
  CHECK(!mabur::au_is_irap(g.data(), g.size()));   // GDR refresh start: a P slice
  CHECK(!mabur::au_is_irap(p.data(), p.size()));
}

// The live encoder is GDR: VPS/SPS/PPS ride on a TRAIL_R refresh start
// every gop_s, real IDRs are rare. A file that begins on the refresh start
// has no IRAP up front, and Apple's decoder (QuickTime, VLC on macOS)
// refuses it -- the 2026-09-29 web-GS recording. Only an IDR opens.
TEST(nothing_written_before_first_idr) {
  const std::string path = scratch("rawdvr_wait.mp4");
  std::remove(path.c_str());
  RawDvr d;
  d.start(path, 0, 0);
  CHECK(d.state() == RawDvr::State::WaitSync);
  feed(d, p_au(1), 0);
  feed(d, p_au(2), 16667);
  feed(d, param_au(3), 33334);                   // refresh start: not an IDR
  CHECK(d.state() == RawDvr::State::WaitSync);
  CHECK(!exists(path));
  feed(d, idr_au(4), 50001);
  CHECK(d.state() == RawDvr::State::Recording);
  CHECK(exists(path));
  feed(d, p_au(5), 66668);
  d.stop();
  CHECK(d.state() == RawDvr::State::Off);
  CHECK(d.samples() == 2);
  CHECK(d.err() == RawDvr::Err::None);
}

TEST(truncated_aus_are_skipped_even_idrs) {
  const std::string path = scratch("rawdvr_trunc.mp4");
  std::remove(path.c_str());
  RawDvr d;
  d.start(path, 0, 0);
  feed(d, idr_au(1), 0, /*complete=*/false);
  CHECK(d.state() == RawDvr::State::WaitSync);   // a truncated refresh never opens
  feed(d, idr_au(2), 16667);
  feed(d, p_au(3), 33334, false);
  feed(d, p_au(4), 50001);
  d.stop();
  CHECK(d.samples() == 2);
}

// Only IDR AUs are sync samples: base P AUs (old "sid 0") must not
// cut fragments. 5 P AUs inside one 1 s fragment window -> 1 fragment.
TEST(plain_aus_do_not_cut_fragments) {
  const std::string path = scratch("rawdvr_frag.mp4");
  std::remove(path.c_str());
  RawDvr d;
  d.start(path, 0, 0, 1000);
  feed(d, idr_au(1), 0);
  for (uint32_t i = 1; i <= 5; ++i) feed(d, p_au(static_cast<uint8_t>(i)), i * 16667);
  d.stop();
  CHECK(d.samples() == 6);
  CHECK(d.fragments() == 1);
}

// A refresh start mid-recording is an ordinary sample: marking it sync
// told players it was a random-access point it is not (702 of 844 "sync"
// samples in the 2026-09-29 recording were P slices).
TEST(gdr_refresh_start_is_not_a_sync_point) {
  const std::string path = scratch("rawdvr_gdr.mp4");
  std::remove(path.c_str());
  RawDvr d;
  d.start(path, 0, 0, 1000);
  feed(d, idr_au(1), 0);
  feed(d, p_au(2), 16667);
  feed(d, param_au(3), 33334);                   // must not cut
  feed(d, p_au(4), 50001);
  feed(d, idr_au(5), 66668);                     // cuts
  d.stop();
  CHECK(d.samples() == 5);
  CHECK(d.fragments() == 2);
}

TEST(second_start_waits_for_next_idr) {
  const std::string a = scratch("rawdvr_a.mp4"), b = scratch("rawdvr_b.mp4");
  std::remove(a.c_str());
  std::remove(b.c_str());
  RawDvr d;
  d.start(a, 0, 0);
  feed(d, idr_au(1), 0);
  feed(d, p_au(2), 16667);
  d.stop();
  d.start(b, 0, 0);                 // params are complete (sticky) already
  feed(d, p_au(3), 33334);          // must NOT open on this P AU
  feed(d, param_au(5), 41000);      // nor on a refresh start
  CHECK(d.state() == RawDvr::State::WaitSync);
  CHECK(!exists(b));
  feed(d, idr_au(4), 50001);
  CHECK(d.state() == RawDvr::State::Recording);
  d.stop();
  CHECK(d.samples() == 1);
}

TEST(stop_before_sync_writes_nothing) {
  const std::string path = scratch("rawdvr_early.mp4");
  std::remove(path.c_str());
  RawDvr d;
  d.start(path, 0, 0);
  feed(d, p_au(1), 0);
  d.stop();
  CHECK(d.state() == RawDvr::State::Off);
  CHECK(!d.opened());
  CHECK(!exists(path));
}

TEST(open_failure_is_error_and_ignores_feed) {
  RawDvr d;
  d.start(scratch("no_such_dir/x.mp4"), 0, 0);
  feed(d, idr_au(1), 0);
  CHECK(d.state() == RawDvr::State::Error);
  CHECK(d.err() == RawDvr::Err::Open);
  CHECK(!d.opened());
  feed(d, idr_au(2), 16667);          // ignored, no crash
  CHECK(d.state() == RawDvr::State::Error);
  d.stop();                             // clears to Off, err kept for reporting
  CHECK(d.state() == RawDvr::State::Off);
  CHECK(d.err() == RawDvr::Err::Open);
}

// ---- sink path (the web GS's OPFS file goes through this) ----
struct MemSink : mabur::DvrSink {
  std::vector<uint8_t>* out;
  bool* destroyed;
  int writes_left;   // fail every write after this many (-1 = never)
  int* calls;        // optional: bumped on every write() invocation, success or not
  MemSink(std::vector<uint8_t>* o, bool* d, int left = -1, int* c = nullptr)
      : out(o), destroyed(d), writes_left(left), calls(c) {}
  ~MemSink() override { if (destroyed) *destroyed = true; }
  bool write(const uint8_t* p, size_t n) override {
    if (calls) ++*calls;
    if (writes_left == 0) return false;
    if (writes_left > 0) --writes_left;
    out->insert(out->end(), p, p + n);
    return true;
  }
  bool flush() override { return true; }
};

std::vector<uint8_t> read_all(const std::string& path) {
  std::ifstream f(path, std::ios::binary);
  return std::vector<uint8_t>((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
}

TEST(sink_output_matches_file_output) {
  const std::string path = scratch("rawdvr_same.mp4");
  std::remove(path.c_str());
  std::vector<uint8_t> mem;
  RawDvr a, b;
  a.start(path, 0, 0);
  b.start(std::make_unique<MemSink>(&mem, nullptr), 0, 0);
  const std::vector<std::vector<uint8_t>> aus = {p_au(1), idr_au(2), p_au(3), p_au(4), idr_au(5), p_au(6)};
  uint32_t pts = 0;
  for (const auto& au : aus) { feed(a, au, pts); feed(b, au, pts); pts += 16667; }
  a.stop();
  b.stop();
  CHECK(!mem.empty());
  CHECK(read_all(path) == mem);
  CHECK(b.bytes() == mem.size());
}

TEST(null_sink_is_open_error) {
  RawDvr d;
  d.start(std::unique_ptr<mabur::DvrSink>{}, 0, 0);
  CHECK(d.state() == RawDvr::State::Error);
  CHECK(d.err() == RawDvr::Err::Open);
  feed(d, idr_au(1), 0);               // ignored
  CHECK(d.state() == RawDvr::State::Error);
}

// Like a full OPFS quota: the init segment lands, the first fragment write fails.
TEST(write_failure_is_sticky_error) {
  std::vector<uint8_t> mem;
  bool destroyed = false;
  RawDvr d;
  d.start(std::make_unique<MemSink>(&mem, &destroyed, 1), 0, 0);
  feed(d, idr_au(1), 0);               // opens: init segment = write #1
  CHECK(d.state() == RawDvr::State::Recording);
  feed(d, p_au(2), 16667);
  feed(d, idr_au(3), 33334);           // key: cuts the first fragment -> write fails
  CHECK(d.state() == RawDvr::State::Error);
  CHECK(d.err() == RawDvr::Err::Write);
  CHECK(destroyed);                      // sink closed, what was written stays
  CHECK(d.bytes() > 0);
  feed(d, p_au(4), 50001);               // ignored
  CHECK(d.state() == RawDvr::State::Error);
}

// Once a fragment's write starts failing, DvrMux must not keep issuing more
// sink writes for the rest of that fragment, nor from the close() a failed
// write triggers (2026-09-28 final review: "DvrMux keeps writing into a
// failed sink").
TEST(write_failure_stops_further_sink_writes) {
  std::vector<uint8_t> mem;
  bool destroyed = false;
  int calls = 0;
  RawDvr d;
  // 2 successful writes: the init segment (moov, on open) and the cut
  // fragment's moof/mdat header. Every write from the first pending sample
  // onward must be refused -- and, with the fix, never even attempted.
  d.start(std::make_unique<MemSink>(&mem, &destroyed, 2, &calls), 0, 0);
  feed(d, idr_au(1), 0);       // opens: write #1 (moov)
  feed(d, p_au(2), 16667);       // buffered: not a cut
  feed(d, p_au(3), 33334);       // buffered
  feed(d, idr_au(4), 50001);   // key: cuts -> header write #2 ok, then sample writes fail
  CHECK(d.state() == RawDvr::State::Error);
  CHECK(d.err() == RawDvr::Err::Write);
  CHECK(calls == 3);             // moov, moof/mdat header, ONE failed sample write -- no more
  const size_t bytes_after_failure = mem.size();
  feed(d, p_au(5), 66668);       // ignored: state is already Error
  d.stop();
  CHECK(calls == 3);             // stop()'s close() must not touch the dead sink again
  CHECK(mem.size() == bytes_after_failure);
}

TEST(unopened_sink_is_dropped_unwritten_on_stop) {
  std::vector<uint8_t> mem;
  bool destroyed = false;
  RawDvr d;
  d.start(std::make_unique<MemSink>(&mem, &destroyed), 0, 0);
  feed(d, p_au(1), 0);                   // before any sync point
  d.stop();
  CHECK(destroyed);
  CHECK(mem.empty());
  CHECK(d.bytes() == 0);
  CHECK(!d.opened());
}

TEST(start_while_recording_seals_previous_file) {
  const std::string a = scratch("rawdvr_c.mp4"), b = scratch("rawdvr_d.mp4");
  std::remove(a.c_str());
  std::remove(b.c_str());
  RawDvr d;
  d.start(a, 0, 0);
  feed(d, idr_au(1), 0);
  d.start(b, 0, 0);
  CHECK(d.state() == RawDvr::State::WaitSync);
  CHECK(d.err() == RawDvr::Err::None);
  std::ifstream f(a, std::ios::binary | std::ios::ate);
  CHECK(f.good() && f.tellg() > 0);     // a was flushed and closed
}

MTEST_MAIN
