// AuRouter (design-reader-thread.md, Task 11b): maburplay's ring sink as a
// unit. Single-threaded here; FeedLoop (Task 11c) runs it on the feed thread.
#include <cstdint>
#include <memory>
#include <vector>

#include "au_ring.h"
#include "au_router.h"
#include "mtest.h"
#include "ring_client.h"
#include "stream_feeder.h"
#include "video_backend.h"

using maburplay::AuEvent;
using maburplay::AuEventKind;
using maburplay::AuRouter;
using maburplay::FeedNote;
using maburplay::StreamFeeder;

namespace {

struct FakeBackend : maburplay::VideoBackend, maburplay::StreamDecoder {
  struct Call { char op; uint32_t pts; bool last; size_t notes_before; };
  std::vector<Call> calls;
  const std::vector<FeedNote>* notes = nullptr;
  void rec(char op, uint32_t pts, bool last) {
    calls.push_back({op, pts, last, notes ? notes->size() : 0});
  }
  bool init(const maburplay::BackendCfg&, FrameSink) override { return true; }
  void submit_au(const uint8_t*, size_t, uint32_t pts) override { rec('W', pts, false); }
  void flush() override { rec('F', 0, false); }
  void release_frame(const maburplay::DmaFrame&) override {}
  bool probe() override { return true; }
  bool start(const uint8_t*, size_t, uint8_t, uint32_t pts) override { rec('S', pts, false); return true; }
  bool append(const uint8_t*, size_t, uint32_t pts, bool last) override { rec('A', pts, last); return true; }
  void abort(uint32_t pts) override { rec('X', pts, true); }
};

// n TRAIL_R slices; *starts = each slice's start-code offset.
std::vector<uint8_t> make_au(int n, std::vector<size_t>* starts) {
  std::vector<uint8_t> au;
  for (int k = 0; k < n; ++k) {
    starts->push_back(au.size());
    const uint8_t sc[] = {0, 0, 0, 1, 0x02, 0x01};
    au.insert(au.end(), sc, sc + sizeof sc);
    au.insert(au.end(), static_cast<size_t>(20 + 7 * k), static_cast<uint8_t>(0x40 + k));
  }
  return au;
}

AuEvent ev(AuEventKind kind, uint64_t rec, uint32_t pts, uint8_t sid, uint8_t ns,
           std::vector<uint8_t> bytes, uint8_t flags = 0, bool flush = false) {
  AuEvent e{};
  e.meta.rec_no = rec;
  e.meta.pts_us = pts;
  e.meta.sid = sid;
  e.meta.nslices = ns;
  e.meta.flags = flags;
  e.meta.len = static_cast<uint32_t>(bytes.size());
  e.au = std::move(bytes);
  e.flush_before = flush;
  e.kind = kind;
  return e;
}

constexpr uint8_t kComplete = maburgs::kRecFlagComplete;

struct Rig {
  std::unique_ptr<maburplay::VideoBackend> backend;
  FakeBackend* fake = nullptr;
  StreamFeeder feeder{true};
  AuRouter router{feeder, backend};
  std::vector<FeedNote> notes;
  int flushes = 0;
  bool flush_ok = true;
  uint64_t t = 1000;
  Rig() {
    auto f = std::make_unique<FakeBackend>();
    fake = f.get();
    backend = std::move(f);
    fake->notes = &notes;
    feeder.set_decoder(fake);
    router.set_hooks({[this](FeedNote&& n) { notes.push_back(std::move(n)); },
                      [this]() {
                        ++flushes;
                        feeder.on_flush();
                        backend->flush();
                        router.disarm();
                        return flush_ok;
                      },
                      [this]() { return t++; }});
  }
};

}  // namespace

TEST(whole_au_waits_for_a_complete_sid0_and_its_submit_note_goes_first) {
  Rig r;
  std::vector<size_t> s;
  const auto au = make_au(1, &s);
  r.router.on_event(ev(AuEventKind::kClose, 1, 10, 1, 1, au, kComplete));  // enhance before sync
  CHECK(r.fake->calls.empty());
  REQUIRE(r.notes.size() == 1);
  CHECK(r.notes[0].kind == FeedNote::Kind::kDelivery);
  CHECK(r.notes[0].au == au);  // the DVR still gets it
  CHECK(r.notes[0].decodable);
  r.router.on_event(ev(AuEventKind::kClose, 2, 20, 0, 1, au, kComplete));  // sync point
  REQUIRE(r.fake->calls.size() == 1);
  CHECK(r.fake->calls[0].op == 'W');
  CHECK(r.fake->calls[0].pts == 20u);
  CHECK(r.fake->calls[0].notes_before == 2);  // delivery(10), then submit(20) BEFORE the call
  REQUIRE(r.notes.size() == 3);
  CHECK(r.notes[1].kind == FeedNote::Kind::kSubmit);
  CHECK(r.notes[1].meta.pts_us == 20u);
  CHECK(r.notes[2].kind == FeedNote::Kind::kDelivery);
  CHECK(r.notes[2].t_us < r.notes[1].t_us);  // stamped at delivery, before the submit
  CHECK(r.notes[2].bytes == au.size());
  CHECK(r.router.armed());
  CHECK(r.router.submits() == 1);
  CHECK(r.router.synced());
}

TEST(truncated_au_is_never_submitted_but_still_delivered) {
  Rig r;
  std::vector<size_t> s;
  const auto au = make_au(1, &s);
  r.router.on_event(ev(AuEventKind::kClose, 1, 10, 0, 1, au, 0));  // incomplete sid0
  CHECK(r.fake->calls.empty());
  CHECK(!r.router.armed());  // a truncated sid0 must not arm
  CHECK(r.router.truncated_skipped() == 1);
  REQUIRE(r.notes.size() == 1);
  CHECK(!r.notes[0].decodable);
  CHECK(!r.notes[0].complete);
}

TEST(flush_before_flushes_before_the_record_reaches_the_decoder) {
  Rig r;
  std::vector<size_t> s;
  const auto au = make_au(1, &s);
  r.router.on_event(ev(AuEventKind::kClose, 1, 10, 0, 1, au, kComplete));
  r.router.on_event(ev(AuEventKind::kClose, 2, 20, 0, 1, au, kComplete, /*flush=*/true));
  CHECK(r.flushes == 1);
  REQUIRE(r.fake->calls.size() == 3);
  CHECK(r.fake->calls[1].op == 'F');
  CHECK(r.fake->calls[2].op == 'W');  // re-armed by the flush record itself
  CHECK(r.fake->calls[2].pts == 20u);
}

TEST(a_refused_flush_drops_the_record) {
  Rig r;
  r.flush_ok = false;
  std::vector<size_t> s;
  const auto au = make_au(1, &s);
  r.router.on_event(ev(AuEventKind::kClose, 1, 10, 0, 1, au, kComplete, true));
  CHECK(r.fake->calls.size() == 1);  // the flush only
  CHECK(r.notes.empty());
}

TEST(streamed_close_sends_its_submit_note_before_the_last_append) {
  Rig r;
  std::vector<size_t> s0;
  r.router.on_event(ev(AuEventKind::kClose, 1, 10, 0, 1, make_au(1, &s0), kComplete));  // arm
  std::vector<size_t> s;
  const auto au = make_au(4, &s);
  r.router.on_event(ev(AuEventKind::kOpen, 2, 20, 0, 4,
                       std::vector<uint8_t>(au.begin(), au.begin() + static_cast<long>(s[2]))));
  const size_t before = r.notes.size();
  r.router.on_event(ev(AuEventKind::kClose, 2, 20, 0, 4, au, kComplete));
  const auto& c = r.fake->calls;
  REQUIRE(c.size() == 5);  // W(10) S A | A A(last)
  CHECK(c[1].op == 'S');
  CHECK(c[4].op == 'A');
  CHECK(c[4].last);
  CHECK(c[4].notes_before == before + 1);  // kSubmit(20) already out
  CHECK(r.notes[before].kind == FeedNote::Kind::kSubmit);
  CHECK(r.notes.back().kind == FeedNote::Kind::kDelivery);
  CHECK(r.router.submits() == 2);
  CHECK(r.feeder.streamed() == 1);
}

TEST(draining_submits_nothing_new) {
  Rig r;
  std::vector<size_t> s0;
  const auto one = make_au(1, &s0);
  r.router.on_event(ev(AuEventKind::kClose, 1, 10, 0, 1, one, kComplete));  // armed
  r.router.set_draining();
  r.router.on_event(ev(AuEventKind::kClose, 2, 20, 0, 1, one, kComplete));
  std::vector<size_t> s;
  const auto au = make_au(4, &s);
  r.router.on_event(ev(AuEventKind::kOpen, 3, 30, 0, 4, au));
  CHECK(r.fake->calls.size() == 1);  // W(10) only: no whole submit, no START
  CHECK(r.notes.back().kind == FeedNote::Kind::kDelivery);  // DVR/stats still see 20
}

MTEST_MAIN
