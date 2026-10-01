// webgs: the web GS entry point. Native: CLI (replay for parity gates, live
// for bench A/B vs the browser). Emscripten: the same live loop, with AUs
// and 1 Hz stats posted to the page (spec 2026-09-27-web-gs §2.3).
#include <algorithm>
#include <atomic>
#include <chrono>
#include <climits>
#include <cerrno>
#include <csignal>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

// The browser page build (Emscripten + live): AUs/stats/errors go to the page
// through Module callbacks. The Node replay build (webgs_node) is plain CLI.
#if defined(__EMSCRIPTEN__) && defined(WEBGS_LIVE)
#define WEBGS_PAGE 1
#endif

#include "au_file.h"
#include "config.h"
#include "frame_file_source.h"
#include "mabur/raw_dvr.h"
#include "web_gs.h"
#ifdef WEBGS_LIVE
#include <libusb.h>
#include "IRtlRadio.h"
#include "RxPacket.h"
#include "UsbDeviceLock.h"
#include "UsbOpen.h"
#include "WiFiDriver.h"
#include "body_queue.h"
#include "dot11.h"
#include "logger.h"
#include "mabur/ht40.h"
#include "relay_link.h"
#include "relay_transport.h"
#endif
#ifdef WEBGS_PAGE
#include <emscripten/em_asm.h>
#include <emscripten/emscripten.h>
#include "mabur/hevc_params.h"
#include "mabur/nal.h"
#include "opfs_file.h"
#endif

#ifndef WEBGS_CONFIG_PATH
#ifdef __EMSCRIPTEN__
#define WEBGS_CONFIG_PATH "/maburgs.toml"   // embedded in the WASM FS (Task 6)
#else
#define WEBGS_CONFIG_PATH MABUR_SOURCE_DIR "/gs/bundle/maburgs.default.toml"
#endif
#endif

namespace {

// Page -> core requests (spec 2026-09-27-web-ui §3.1/§3.3). Written from the
// browser main thread (Module._webgs_stop/_webgs_set_rec) or a native signal
// handler; read by the live loop. Shared memory under pthreads: no proxying.
std::atomic<bool> g_stop{false};
std::atomic<int> g_rec{-1};   // -1 never pressed, 0 off, 1 on

// Page -> core IDR requests (webgs_request_idr): a monotonically increasing
// count; the loop forwards it, and its low byte rides every RCF.
std::atomic<uint32_t> g_idr_req{0};

// Browser storage for local recordings (spec 2026-09-28-web-local-recording
// §1.3): true once the startup OPFS probe passed. Always false natively.
std::atomic<bool> g_opfs_ok{false};

// Page -> core local-recording request (webgs_set_local_rec). seq bumps on
// every call so two presses between loop iterations are both applied.
struct LocalRecReq {
  std::mutex mu;
  uint32_t seq = 0;
  bool on = false;
  std::string name;
} g_lrec;

// ---- page / console reporting --------------------------------------------

// `ERROR <reason>` lines: the page maps the reason to text. Natively a
// stdout line; in the browser Module.onError.
void report_error(const char* fmt, ...) {
  char buf[512];
  va_list ap;
  va_start(ap, fmt);
  std::vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
#ifdef WEBGS_PAGE
  MAIN_THREAD_ASYNC_EM_ASM({ Module['onError'](UTF8ToString($0)); _free($0); }, strdup(buf));
#endif
  std::printf("ERROR %s\n", buf);
  std::fflush(stdout);
}

void report_stats(const std::string& json) {
#ifdef WEBGS_PAGE
  MAIN_THREAD_ASYNC_EM_ASM({ Module['onStats'](UTF8ToString($0)); _free($0); },
                           strdup(json.c_str()));
#else
  std::printf("STATS %s\n", json.c_str());
  std::fflush(stdout);
#endif
}

// One per armed recording, when it is sealed (spec §1.4): the page downloads
// `name` from OPFS when bytes > 0. err: 0 ok, 1 open, 2 write, 3 no OPFS.
void report_rec_closed(const std::string& name, uint64_t bytes, int err) {
#ifdef WEBGS_PAGE
  MAIN_THREAD_ASYNC_EM_ASM({ if (Module['onRecClosed']) Module['onRecClosed'](UTF8ToString($0), $1, $2); _free($0); },
                           strdup(name.c_str()), static_cast<double>(bytes), err);
#else
  std::printf("RECCLOSED %s %llu %d\n", name.c_str(), static_cast<unsigned long long>(bytes), err);
  std::fflush(stdout);
#endif
}

int lrec_err_code(const mabur::RawDvr& d) {
  switch (d.err()) {
    case mabur::RawDvr::Err::Open: return 1;
    case mabur::RawDvr::Err::Write: return 2;
    default: return 0;
  }
}

// Hands one AU to the page. Native live: nothing (the STATS line counts AUs).
void emit_au(webgs::Au&& a) {
#ifdef WEBGS_PAGE
  // Page-side hand-off clock, taken on THIS (core) thread so the page can
  // subtract it from its own arrival time on the same epoch-aligned clock.
  const double t_emit_ms = EM_ASM_DOUBLE({ return performance.timeOrigin + performance.now(); });
  // Copy out of the WASM heap on the core thread, then free on the main
  // thread; the page receives a plain transferable ArrayBuffer.
  auto* p = static_cast<uint8_t*>(std::malloc(a.data.empty() ? 1 : a.data.size()));
  if (!a.data.empty()) std::memcpy(p, a.data.data(), a.data.size());
  // hvcC for WebCodecs' `description`: WebCodecs refuses a non-IRAP key
  // chunk in Annex-B mode and the drone is GDR (parameter sets ride a
  // TRAIL_R refresh, never an IRAP), so the page configures with hvcC and
  // feeds length-prefixed chunks. Sent with every complete AU that carries
  // a VPS, built by mabur's own HevcParams (the DVR muxer's).
  static mabur::HevcParams hp;
  uint8_t* hv = nullptr;
  int hv_len = 0;
  if (a.complete) {
    bool has_vps = false;
    for (const auto& nal : mabur::split_nals(a.data.data(), a.data.size()))
      has_vps |= nal.type == 32;
    if (hp.feed(a.data.data(), a.data.size()) && has_vps) {
      const std::vector<uint8_t> rec = hp.hvcc();
      if (!rec.empty()) {   // $8 == 0 tells the page "no hvcC": nothing to free
        hv = static_cast<uint8_t*>(std::malloc(rec.size()));
        std::memcpy(hv, rec.data(), rec.size());
        hv_len = static_cast<int>(rec.size());
      }
    }
  }
  const double cap_us =
      a.cap_to_complete_us ? static_cast<double>(*a.cap_to_complete_us) : -1.0;
  MAIN_THREAD_ASYNC_EM_ASM(
      {
        const buf = HEAPU8.slice($0, $0 + $1).buffer;
        _free($0);
        let hvcc = null;
        if ($8) { hvcc = HEAPU8.slice($7, $7 + $8).buffer; _free($7); }
        Module['onAu'](buf, $2, $3, $4, $5, $6, hvcc, $9, $10, $11);
      },
      p, static_cast<int>(a.data.size()), static_cast<double>(a.pts_us), a.sid, a.flags,
      a.complete ? 1 : 0, static_cast<double>(a.t_complete_us), hv, hv_len, cap_us, t_emit_ms,
      // FEC/assembly segment (fix round 1, Ruling R7): first body -> AU
      // complete, core clock; 0 when FrameStream never saw a nonzero
      // body_mono_us for this AU (gs/src/frame_stream.cpp). Page-only
      // (WEBGS_PAGE); replay's AuFileWriter path is untouched.
      static_cast<double>(a.t_first_us));
#else
  (void)a;
#endif
}

// Hands one MSP OSD screen to the page (spec 2026-09-27-web-msp-osd).
// Native live: nothing (the STATS line counts osd_screens).
void emit_osd(int rows, int cols, const uint16_t* cells) {
#ifdef WEBGS_PAGE
  const size_t bytes = static_cast<size_t>(rows) * static_cast<size_t>(cols) * sizeof(uint16_t);
  if (bytes == 0) return;
  auto* p = static_cast<uint8_t*>(std::malloc(bytes));
  std::memcpy(p, cells, bytes);
  // HEAPU8.slice returns a fresh, 0-aligned ArrayBuffer, so viewing it as
  // Uint16Array is safe; HEAPU16 itself is not relied on from EM_ASM.
  MAIN_THREAD_ASYNC_EM_ASM(
      {
        const cells = new Uint16Array(HEAPU8.slice($0, $0 + $1).buffer);
        _free($0);
        if (Module['onOsd']) Module['onOsd']($2, $3, cells);
      },
      p, static_cast<int>(bytes), rows, cols);
#else
  (void)rows;
  (void)cols;
  (void)cells;
#endif
}

#ifdef WEBGS_PAGE
// DvrSink over an OPFS file (spec §1.3). Destruction only closes -- never
// removes: it can run inside the AU callback (RawDvr seals on a write
// error there), where an async remove() must not unwind. seal_local_rec()
// removes an empty file from the loop instead.
class OpfsSink final : public mabur::DvrSink {
 public:
  explicit OpfsSink(std::unique_ptr<webgs::OpfsFile> f) : f_(std::move(f)) {}
  bool write(const uint8_t* p, size_t n) override { return f_->write(p, n); }
  bool flush() override { return f_->flush(); }

 private:
  std::unique_ptr<webgs::OpfsFile> f_;
};
#endif

bool load_cfg(const std::string& path, const std::string& overlay, maburgs::Config& cfg) {
  try {
    cfg = maburgs::load_config(path, nullptr, overlay);
  } catch (const std::exception& e) {
    report_error("bad config: %s", e.what());
    return false;
  }
  return true;
}

bool parse_mode(const std::string& s, webgs::Mode& m) {
  if (s == "gs") m = webgs::Mode::Gs;
  else if (s == "spotter") m = webgs::Mode::Spotter;
  else return false;
  return true;
}

// ---- replay ----------------------------------------------------------------

struct ReplayOpts {
  std::string in, out, config = WEBGS_CONFIG_PATH, trace, record;
  webgs::Mode mode = webgs::Mode::Gs;
  int drop_pct = 0;
  uint32_t seed = 1;
  bool fixed_gap = false, fake_ack = false;
};

// maburgs --dry-run's source and clock: FrameFileSource with one card,
// tick() after every body on the body's own stamp, then one final tick at
// last + frame_gap_timeout_ms + 1 (main.cpp's closing fstream.poll).
int run_replay(const ReplayOpts& o) {
  maburgs::Config cfg;
  if (!load_cfg(o.config, "", cfg)) return 2;
  maburgs::FrameFileSource src(o.in, {1, o.drop_pct, o.seed});
  if (!src.ok()) { std::fprintf(stderr, "error: cannot read %s\n", o.in.c_str()); return 2; }
  webgs::AuFileWriter w;
  if (!w.open(o.out.c_str())) { std::fprintf(stderr, "error: cannot write %s\n", o.out.c_str()); return 2; }
  FILE* trace = nullptr;
  if (!o.trace.empty() && !(trace = std::fopen(o.trace.c_str(), "w"))) {
    std::fprintf(stderr, "error: cannot write %s\n", o.trace.c_str());
    return 2;
  }

  // --record: the web page's local recorder over the replay (armed from the
  // first AU, sealed at the end) -- the native/WASM mp4 parity input.
  mabur::RawDvr rec;
  if (!o.record.empty()) rec.start(o.record, 0, 0);

  webgs::Io io;
  io.on_au = [&](webgs::Au&& a) {
    rec.feed(a.data.data(), a.data.size(), a.pts_us, a.complete);
    w.write(a);
  };
  // Gs mode needs a send path; replay has no radio, so count only.
  uint64_t sends = 0;
  if (o.mode == webgs::Mode::Gs) io.send = [&](const std::vector<uint8_t>&) { ++sends; };
  int last_rung = INT_MIN;
  if (trace)
    io.on_control_tick = [&](double now_ms, const maburgs::LinkHealth& h, int rung,
                             const std::vector<uint8_t>* sent) {
      if (!sent && rung == last_rung) return;
      last_rung = rung;
      std::fprintf(trace,
                   "%.0f rung=%d valid=%d pre=%.6f resid=%.6f s3pre=%.6f s3resid=%.6f "
                   "pv=%d pl=%.6f starved=%d sent=",
                   now_ms, rung, h.sample_valid ? 1 : 0, h.pre_fec_loss, h.residual_loss,
                   h.s3_pre_fec_loss, h.s3_residual_loss, h.probe_valid ? 1 : 0, h.probe_loss,
                   h.video_starved ? 1 : 0);
      if (sent)
        for (uint8_t b : *sent) std::fprintf(trace, "%02x", b);
      else
        std::fputc('-', trace);
      std::fputc('\n', trace);
    };

  webgs::Opts wo;
  wo.adaptive_gap = !o.fixed_gap;
  webgs::WebGs g(cfg, o.mode, cfg.radio.channel, cfg.radio.width, std::move(io), wo);

  uint64_t last_us = 0;
  bool first = true;
  while (auto m = src.next()) {
    if (first && o.fake_ack) g.inject_disc_ack_for_replay(m->mono_us);
    first = false;
    g.on_rx(*m);
    g.tick(m->mono_us);
    last_us = m->mono_us;
  }
  // Match the dry-run's final poll on the ms clock: last_ms + gap + 1.
  g.tick((last_us / 1000 + static_cast<uint64_t>(cfg.video.frame_gap_timeout_ms) + 1) * 1000);
  rec.stop();
  if (!o.record.empty() && rec.err() != mabur::RawDvr::Err::None) {
    std::fprintf(stderr, "error: --record %s failed\n", o.record.c_str());
    return 2;
  }
  if (trace) std::fclose(trace);

  const webgs::Stats st = g.stats();
  std::fprintf(stderr, "webgs replay: frames=%llu dropped=%llu aus_out=%llu sends=%llu %s\n",
               static_cast<unsigned long long>(src.frames_read()),
               static_cast<unsigned long long>(src.dropped()),
               static_cast<unsigned long long>(w.written()),
               static_cast<unsigned long long>(sends), webgs::stats_json(st).c_str());
  return 0;
}

// ---- live ------------------------------------------------------------------

#ifdef WEBGS_LIVE
uint64_t now_us() {
  static const auto t0 = std::chrono::steady_clock::now();
  return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
                                   std::chrono::steady_clock::now() - t0)
                                   .count());
}

// USB delivery lateness (host arrival - chip RX TSF, above the per-second
// minimum), rxprobe's measure, on CRC-good frames.
struct UsbLate {
  std::mutex mu;
  std::vector<int64_t> offs;
  uint32_t last_tsf = 0;
  int64_t tsf_hi = 0;
  void add(int64_t host, uint32_t tsf) {
    std::lock_guard<std::mutex> lk(mu);
    if (tsf < last_tsf) tsf_hi += int64_t{1} << 32;
    last_tsf = tsf;
    offs.push_back(host - (tsf_hi + tsf));
  }
  // p99/max of this second's lateness above its minimum; resets the window.
  void take(int64_t& p99, int64_t& mx) {
    std::vector<int64_t> v;
    {
      std::lock_guard<std::mutex> lk(mu);
      v.swap(offs);
    }
    p99 = mx = 0;
    if (v.empty()) return;
    std::sort(v.begin(), v.end());
    const int64_t mn = v.front();
    p99 = v[v.size() * 99 / 100] - mn;
    mx = v.back() - mn;
  }
};

struct LiveOpts {
  std::string config = WEBGS_CONFIG_PATH;
  std::string overlay;       // optional TOML merged over config (the page's /overlay.toml)
  webgs::Mode mode = webgs::Mode::Gs;
  int ch = -1, width = -1;   // -1 = the config's radio.channel / radio.width
  int secs = 0;              // 0 = until the card goes away
  std::string bad_chw;       // non-numeric --ch/--w, reported by run_live
  std::string relay;         // --relay host:port: mabur-relay v3 over UDP instead of USB
};

// Strict decimal int: the whole string, no sign games, fits an int.
bool parse_int(const char* v, int& out) {
  if (!v || !*v) return false;
  char* end = nullptr;
  errno = 0;
  const long x = std::strtol(v, &end, 10);
  if (errno || *end || x < INT_MIN || x > INT_MAX) return false;
  out = static_cast<int>(x);
  return true;
}

// The radio seam run_live's loop needs; filled by the USB path now and the
// relay path in Task 9.
struct LiveRadio {
  maburgs::BodyQueue* q = nullptr;
  std::function<void(const std::vector<uint8_t>& frame)> send_frame;   // build_control_frame() output
  std::function<bool()> ended;                   // radio gone (checked once the queue is empty)
  const char* lost_error = "card lost";          // report_error text when ended() fires
  std::function<const char*()> lost_reason;      // optional, overrides lost_error when set (chosen at report time)
  std::function<void()> on_pass;                 // optional, every loop pass (relay keepalive)
  std::function<std::string()> extra_stats;      // ",\"k\":v..." appended to each STATS
};

// The body-queue-consuming half of live: local recording, page requests
// (vtx rec / IDR / local rec), the on_rx/tick loop and its STATS cadence.
// Radio-specific bits (RX feed, TX send, "gone" check, per-radio stats)
// come in through r; run_live builds them for the USB path.
int live_loop(const LiveOpts& o, const maburgs::Config& cfg, uint8_t ch, int width, LiveRadio& r) {
  // Local recording (spec 2026-09-28-web-local-recording §1.2), fed on this
  // thread from the AU callback, before the page hand-off.
  mabur::RawDvr dvr;
  std::string lrec_name;       // the armed/last recording's file name
  bool lrec_armed = false;     // a recording is armed or open and not yet reported
  int lrec_err = 0;            // last reported code (3 = no OPFS)
  // lrec_err stays set after an error, so the stats show state 3 until the
  // next start: the UI keeps REC! until the next press.
  uint32_t lrec_seq = 0;
  auto seal_local_rec = [&]() {
    if (!lrec_armed) return;
    const int err = lrec_err_code(dvr);   // read before stop(): Error is sticky
    const uint64_t bytes = dvr.bytes();
    dvr.stop();                           // closes the sync access handle
#ifdef WEBGS_PAGE
    // Stopped before the first sync point (or the open failed): no file to
    // keep. Here, in the loop, the async remove may unwind.
    if (bytes == 0) webgs::OpfsFile::remove(lrec_name);
#endif
    lrec_armed = false;
    lrec_err = err;
    report_rec_closed(lrec_name, bytes, err);
  };

  webgs::Io io;
  // Can fire from inside on_rx (spotter drone-restart reset) as well as from
  // tick(): emit_au keeps no glue state.
  io.on_au = [&dvr](webgs::Au&& a) {
    dvr.feed(a.data.data(), a.data.size(), a.pts_us, a.complete);
    emit_au(std::move(a));
  };
  io.on_osd = emit_osd;
  uint16_t tx_seq = 0;
  if (o.mode == webgs::Mode::Gs)
    io.send = [&](const std::vector<uint8_t>& body) {
      const auto f = maburgs::build_control_frame(tx_seq, body.data(), body.size());
      tx_seq = static_cast<uint16_t>((tx_seq + 1) & 0xFFF);
      r.send_frame(f);
    };
  webgs::WebGs g(cfg, o.mode, ch, width, std::move(io));

#ifndef WEBGS_PAGE
  std::signal(SIGINT, [](int) { g_stop.store(true); });
  std::signal(SIGTERM, [](int) { g_stop.store(true); });
#endif

  int rc = 0;
  std::vector<mabur::node::RxBody> batch;
  uint64_t next_stat = now_us() + 1000000;
  int applied_rec = -1;
  uint32_t applied_idr = 0;
  for (int s = 0; o.secs == 0 || s < o.secs;) {
    if (g_stop.load(std::memory_order_acquire)) break;
    if (const int rw = g_rec.load(std::memory_order_acquire); rw != applied_rec && rw >= 0) {
      g.set_vtx_rec(rw == 1);
      applied_rec = rw;
    }
    if (const uint32_t ir = g_idr_req.load(std::memory_order_acquire); ir != applied_idr) {
      g.set_idr_requests(ir);
      applied_idr = ir;
    }
    {
      bool on = false;
      std::string name;
      bool changed = false;
      {
        std::lock_guard<std::mutex> lk(g_lrec.mu);
        if (g_lrec.seq != lrec_seq) {
          lrec_seq = g_lrec.seq;
          on = g_lrec.on;
          name = g_lrec.name;
          changed = true;
        }
      }
      if (changed) {
        seal_local_rec();
        if (on && !g_opfs_ok.load()) {
          lrec_err = 3;
          report_rec_closed(name, 0, 3);
        } else if (on) {
          lrec_name = name;
          lrec_err = 0;
#ifdef WEBGS_PAGE
          // Created now, from the loop (the open awaits through ASYNCIFY);
          // a failed open arms an Error that the check below reports as 1.
          std::unique_ptr<mabur::DvrSink> sink;
          if (auto f = webgs::OpfsFile::open(name)) sink = std::make_unique<OpfsSink>(std::move(f));
          dvr.start(std::move(sink), 0, 0);
#endif
          // The file opens on an IRAP only, and the GDR link sends one
          // seconds apart: ask the drone for one now (applied by the IDR
          // check at the top of the next pass; a no-op for a spotter).
          g_idr_req.fetch_add(1, std::memory_order_acq_rel);
          lrec_armed = true;
        }
      }
    }
    batch.clear();
    if (r.on_pass) r.on_pass();
    r.q->drain(batch, 5);
    for (const auto& m : batch) g.on_rx(m);
    g.tick(now_us());
    if (lrec_armed && dvr.state() == mabur::RawDvr::State::Error) seal_local_rec();
    if (r.ended() && batch.empty()) {
      report_error("%s", r.lost_reason ? r.lost_reason() : r.lost_error);
      rc = 1;
      break;
    }
    if (now_us() < next_stat) continue;
    next_stat += 1000000;
    ++s;
    std::string j = webgs::stats_json(g.stats());
    j.pop_back();   // '}'
    const int lst = lrec_armed ? (dvr.state() == mabur::RawDvr::State::Recording ? 2 : 1)
                               : (lrec_err ? 3 : 0);
    // lrec_name is page-built from [A-Za-z0-9.-] only (Task 7's recFileName),
    // so it needs no JSON escaping.
    char extra[384];
    std::snprintf(extra, sizeof extra, ",\"qdrop\":%llu,\"lrec_avail\":%d,\"lrec_state\":%d,\"lrec_bytes\":%llu,"
                  "\"lrec_err\":%d,\"lrec_name\":\"%s\"",
                  static_cast<unsigned long long>(r.q->dropped()), g_opfs_ok.load() ? 1 : 0, lst,
                  static_cast<unsigned long long>(dvr.bytes()), lrec_err, lrec_name.c_str());
    report_stats(j + extra + (r.extra_stats ? r.extra_stats() : "") + "}");
  }
  // Every exit (Disconnect, card lost) seals the local recording before the
  // module goes away, so the page can download it (spec §1.4).
  seal_local_rec();
  return rc;
}

// Live over a CPE510 mabur-relay (protocol v3) instead of the USB card:
// RelayLink owns the transport + RX thread, feeding the same BodyQueue
// live_loop already drains. Bounded by RelayClient's own timers -- no local
// timeout logic here.
int run_live_relay(const LiveOpts& o, const maburgs::Config& cfg, uint8_t ch, int width) {
  std::unique_ptr<webgs::RelayTransport> t;
#ifdef __EMSCRIPTEN__
  t = webgs::open_ring_transport();
#else
  std::string err;
  t = webgs::open_udp_transport(o.relay, err);
  if (!t) { report_error("relay unreachable: %s", err.c_str()); return 1; }
#endif
  const uint8_t sec = width == 40 ? mabur::ht40_offset(ch) : 0;
  maburgs::BodyQueue q;
  webgs::RelayLink link(std::move(t), ch, sec, q, now_us);
  link.start();
  const bool gs = o.mode == webgs::Mode::Gs;
  for (;;) {   // bounded by RelayClient: Unreachable after kLostMs, Refused/TuneFailed after kTuneWindowMs
    if (g_stop.load(std::memory_order_acquire)) { link.stop(); std::printf("DONE\n"); return 0; }
    link.tick();
    const auto r = link.ready(gs);
    if (r == webgs::RelayLink::Ready::Owned || r == webgs::RelayLink::Ready::Listening) break;
    if (r == webgs::RelayLink::Ready::Unreachable) { report_error("relay unreachable"); link.stop(); return 1; }
    if (r == webgs::RelayLink::Ready::Refused) { report_error("relay owned by another client"); link.stop(); return 1; }
    if (r == webgs::RelayLink::Ready::TuneFailed) { report_error("relay cannot tune"); link.stop(); return 1; }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  LiveRadio r;
  r.q = &q;
  r.send_frame = [&](const std::vector<uint8_t>& f) { link.send_frame(f); };
  // GS mode only: a UDP subscriber (native webgs --relay, a future
  // RemoteCard) can take ownership away from us at any time -- the relay
  // prefers the oldest UDP subscriber. A spotter never owns, so it has
  // nothing to lose.
  r.ended = [&] { return link.lost() || (gs && link.ownership_lost()); };
  r.lost_error = "relay lost";
  r.lost_reason = [&] { return (gs && link.ownership_lost()) ? "relay taken by another client" : "relay lost"; };
  r.on_pass = [&] { link.tick(); };
  r.extra_stats = [&] { return link.stats_fields(); };
  const int rc = live_loop(o, cfg, ch, width, r);
  link.stop();
  q.close();
  std::printf("DONE\n");
  std::fflush(stdout);
  return rc;
}

int run_live(const LiveOpts& o) {
  maburgs::Config cfg;
  if (!load_cfg(o.config, o.overlay, cfg)) return 2;
  std::fprintf(stderr, "webgs: link key %s (%s)\n",
               mabur::key_fingerprint(cfg.link.key).c_str(), cfg.link.key_source.c_str());
  if (!o.bad_chw.empty()) {
    report_error("bad channel/width: %s", o.bad_chw.c_str());
    return 2;
  }
  const int ch_i = o.ch >= 0 ? o.ch : cfg.radio.channel;
  const int width = o.width >= 0 ? o.width : cfg.radio.width;
  // maburgs's own loader checks on the override (the page picks ch/w, not
  // the config): range, 20|40, HT40 pair, and in GS mode no 40 MHz rung
  // while tuned 20.
  if (auto e = webgs::channel_width_error(cfg, o.mode, ch_i, width)) {
    report_error("bad channel/width: %s", e->c_str());
    return 2;
  }
  const uint8_t ch = static_cast<uint8_t>(ch_i);
  std::printf("webgs live: mode %s ch %u width %d\n",
              o.mode == webgs::Mode::Gs ? "gs" : "spotter", ch, width);
  std::fflush(stdout);
  if (!o.relay.empty()) return run_live_relay(o, cfg, ch, width);

  libusb_context* ctx = nullptr;
  if (libusb_init(&ctx) != 0) { report_error("libusb_init"); return 1; }
  libusb_device** list = nullptr;
  const ssize_t n = libusb_get_device_list(ctx, &list);
  libusb_device* dev = nullptr;
  for (ssize_t i = 0; i < n && !dev; ++i) {
    libusb_device_descriptor dd;
    if (libusb_get_device_descriptor(list[i], &dd) != 0 || dd.idVendor != 0x0bda) continue;
    for (uint16_t p : {0xa81a, 0x881a, 0x8812})
      if (dd.idProduct == p) dev = list[i];
  }
  libusb_device_handle* h = nullptr;
  if (!dev || libusb_open(dev, &h) != 0) {
    if (list) libusb_free_device_list(list, 1);
    report_error("no RTL card");
    libusb_exit(ctx);
    return 1;
  }
  libusb_free_device_list(list, 1);
  // Early-error unwind once the handle is open (the page reconnects in the
  // same tab, so nothing may leak).
  auto bail = [&](int code) {
    if (h) libusb_close(h);
    libusb_exit(ctx);
    return code;
  };
  auto logger = std::make_shared<Logger>();
  std::shared_ptr<devourer::UsbDeviceLock> lock;
  if (int rc = devourer::claim_interface_then_reset(h, 0, logger, true, lock); rc != 0) {
    report_error("claim failed rc=%d", rc);
    return bail(1);
  }
  devourer::DeviceConfig dcfg;
  dcfg.rx.enable_with_tx = true;
  dcfg.rx.keep_corrupted = true;
  dcfg.tuning.disable_cca = false;
  dcfg.tx.no_cancel_multipkt = true;
  dcfg.usb.rx_zerocopy = false;
  WiFiDriver driver(logger);
  auto radio = driver.CreateRadio(h, ctx, lock, dcfg);
  auto* rtl = dynamic_cast<IRtlRadio*>(radio.get());
  if (!rtl) {
    report_error("unsupported chip");
    radio.reset();
    lock.reset();
    return bail(1);
  }
  rtl->InitWrite(width == 40 ? SelectedChannel{ch, mabur::ht40_offset(ch), CHANNEL_WIDTH_40}
                             : SelectedChannel{ch, 0, CHANNEL_WIDTH_20});

  maburgs::BodyQueue q;
  UsbLate late;
  std::atomic<bool> rx_ended{false};
  std::thread rx([&] {
    rtl->StartRxLoop([&](const Packet& pkt) {
      const uint64_t host = now_us();
      const auto& a = pkt.RxAtrib;
      maburgs::RxMeta meta;
      meta.crc_err = a.crc_err;
      meta.data_rate = a.data_rate;
      meta.rssi[0] = a.rssi[0]; meta.rssi[1] = a.rssi[1];
      meta.snr[0] = a.snr[0];   meta.snr[1] = a.snr[1];
      meta.evm[0] = a.evm[0];   meta.evm[1] = a.evm[1];
      meta.physt = a.physt;
      meta.tsfl = a.tsfl;
      mabur::node::RxBody m;
      if (maburgs::fill_rx_body(pkt.Data.data(), pkt.Data.size(), meta, m) !=
          maburgs::RxVerdict::Body)
        return;
      m.card_id = 0;
      m.rx_channel = ch;
      m.mono_us = host;
      if (m.crc_ok) late.add(static_cast<int64_t>(host), a.tsfl);
      q.push(std::move(m));
    });
    // StartRxLoop returns on StopRxLoop or when the device goes away.
    rx_ended.store(true, std::memory_order_release);
    q.close();
  });

  uint64_t txfail = 0;
  LiveRadio r;
  r.q = &q;
  r.send_frame = [&](const std::vector<uint8_t>& f) { if (!rtl->send_packet(f.data(), f.size())) ++txfail; };
  r.ended = [&] { return rx_ended.load(std::memory_order_acquire); };
  r.extra_stats = [&] {
    int64_t p99 = 0, mx = 0;
    late.take(p99, mx);
    char b[160];
    std::snprintf(b, sizeof b, ",\"radio\":\"usb\",\"usb_p99_us\":%lld,\"usb_max_us\":%lld,\"txfail\":%llu",
                  static_cast<long long>(p99), static_cast<long long>(mx),
                  static_cast<unsigned long long>(txfail));
    return std::string(b);
  };
  const int rc = live_loop(o, cfg, ch, width, r);

  const uint64_t t_stop0 = now_us();
  if (!rx_ended.load()) rtl->StopRxLoop();
  q.close();
  // WebUSB has no transfer cancel (libusb's emscripten backend cancel is a
  // no-op): on a quiet channel the RX loop's pending transferIn calls never
  // complete and rx.join() would wait for the next received frame -- forever
  // with the drone off. Releasing the interface makes Chrome abort them
  // (AbortError), which ends the loop. Re-claimed below for Stop()'s de-init.
  bool released_early = false;
  for (int waited = 0; !rx_ended.load(std::memory_order_acquire) && waited < 300; waited += 10)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  if (!rx_ended.load(std::memory_order_acquire)) {
    libusb_release_interface(h, 0);
    released_early = true;
  }
  rx.join();
  if (released_early) libusb_claim_interface(h, 0);
  const uint64_t t_rx = now_us();
  // In-page Disconnect needs a real teardown (the page builds a fresh module
  // on the next Connect): radio before the handle it drives, interface
  // released before close, context last (devourer's DeviceSession order).
  // Stop() powers the chip down to a re-enumerable state first, as maburgs'
  // RadioFrontend::stop does; on a lost card its de-init writes fail and it
  // swallows that. The lock is not a claim: it releases no interface.
  // Returning from main lets -sEXIT_RUNTIME fire Module.onExit(rc).
  rtl->Stop();
  const uint64_t t_chip = now_us();
  radio.reset();
  libusb_release_interface(h, 0);
  libusb_close(h);
  lock.reset();
  libusb_exit(ctx);
  std::printf("teardown: rx %llu ms%s, chip stop %llu ms, close %llu ms\n",
              static_cast<unsigned long long>((t_rx - t_stop0) / 1000),
              released_early ? " (reads aborted)" : "",
              static_cast<unsigned long long>((t_chip - t_rx) / 1000),
              static_cast<unsigned long long>((now_us() - t_chip) / 1000));
  std::printf("DONE\n");
  std::fflush(stdout);
  return rc;
}
#endif  // WEBGS_LIVE

int usage(FILE* out, int rc) {
  std::fprintf(out,
               "usage: webgs replay <frames.bin> <out-aus> [-c config.toml] [--mode gs|spotter]\n"
               "                    [--drop-pct P] [--seed S] [--fixed-gap]\n"
               "                    [--control-trace <file>] [--fake-ack] [--record <file.mp4>]\n"
#ifdef WEBGS_LIVE
               "       webgs live [-c config.toml] [--overlay file.toml] [--ch N] [--w 20|40]\n"
               "                  [--secs 0] [--mode gs|spotter] [--relay host:port]\n"
               "                  (ch/w default to radio.channel/width)\n"
#endif
               "default config: %s\n",
               WEBGS_CONFIG_PATH);
  return rc;
}

bool is_help(const std::string& a) { return a == "-h" || a == "--help"; }

#ifdef WEBGS_LIVE
// argv[first..] are live options. Returns -1 on success, else the exit code.
int parse_live(int argc, char** argv, int first, LiveOpts& o) {
  for (int i = first; i < argc; ++i) {
    const std::string k = argv[i];
    if (is_help(k)) return usage(stdout, 0);
    if (i + 1 >= argc) return usage(stderr, 2);
    const char* v = argv[++i];
    if (k == "-c") o.config = v;
    else if (k == "--overlay") o.overlay = v;
    else if (k == "--ch") {
      if (!parse_int(v, o.ch) || o.ch < 0)
        o.bad_chw = std::string("channel '") + v + "' is not a channel number";
    } else if (k == "--w") {
      if (!parse_int(v, o.width) || o.width < 0)
        o.bad_chw = std::string("width '") + v + "' is not 20 or 40";
    }
    else if (k == "--secs") o.secs = std::atoi(v);
    else if (k == "--mode") { if (!parse_mode(v, o.mode)) return usage(stderr, 2); }
    else if (k == "--relay") o.relay = v;
    else return usage(stderr, 2);
  }
  return -1;
}
#endif

}  // namespace

#ifdef WEBGS_PAGE
extern "C" {
EMSCRIPTEN_KEEPALIVE void webgs_stop() { g_stop.store(true); }
EMSCRIPTEN_KEEPALIVE void webgs_set_rec(int on) { g_rec.store(on ? 1 : 0); }
EMSCRIPTEN_KEEPALIVE void webgs_request_idr() { g_idr_req.fetch_add(1, std::memory_order_acq_rel); }
EMSCRIPTEN_KEEPALIVE void webgs_set_local_rec(int on, const char* name) {
  std::lock_guard<std::mutex> lk(g_lrec.mu);
  g_lrec.on = on != 0;
  g_lrec.name = (on && name) ? name : "";
  ++g_lrec.seq;
}
}
#endif

int main(int argc, char** argv) {
#ifdef WEBGS_PAGE
  // The page build. The page passes `arguments` straight through: live
  // options only (--mode, --ch, --w); an optional leading "live" is accepted.
  // (The Node build, webgs_node, has no WEBGS_LIVE and takes the CLI below:
  // replay for the native/WASM parity gate.)
  LiveOpts lo;
  const int first = (argc > 1 && std::string(argv[1]) == "live") ? 2 : 1;
  g_opfs_ok.store(webgs::opfs_probe());
  std::printf("webgs: opfs %s\n", g_opfs_ok.load() ? "ok" : "unavailable");
  std::fflush(stdout);
  if (int rc = parse_live(argc, argv, first, lo); rc >= 0) return rc;
  return run_live(lo);
#else
  if (argc < 2) return usage(stderr, 2);
  const std::string cmd = argv[1];
  if (is_help(cmd)) return usage(stdout, 0);
  if (cmd == "replay") {
    ReplayOpts o;
    int pos = 0;
    for (int i = 2; i < argc; ++i) {
      const std::string k = argv[i];
      if (is_help(k)) return usage(stdout, 0);
      if (k == "--fixed-gap") { o.fixed_gap = true; continue; }
      if (k == "--fake-ack") { o.fake_ack = true; continue; }
      if (k.size() > 1 && k[0] == '-') {
        if (i + 1 >= argc) return usage(stderr, 2);
        const char* v = argv[++i];
        if (k == "-c") o.config = v;
        else if (k == "--mode") { if (!parse_mode(v, o.mode)) return usage(stderr, 2); }
        else if (k == "--drop-pct") o.drop_pct = std::atoi(v);
        else if (k == "--seed") o.seed = static_cast<uint32_t>(std::atol(v));
        else if (k == "--control-trace") o.trace = v;
        else if (k == "--record") o.record = v;
        else return usage(stderr, 2);
        continue;
      }
      if (pos == 0) o.in = k;
      else if (pos == 1) o.out = k;
      else return usage(stderr, 2);
      ++pos;
    }
    if (pos != 2) return usage(stderr, 2);
    return run_replay(o);
  }
#ifdef WEBGS_LIVE
  if (cmd == "live") {
    LiveOpts o;
    if (int rc = parse_live(argc, argv, 2, o); rc >= 0) return rc;
    return run_live(o);
  }
#endif
  return usage(stderr, 2);
#endif
}
