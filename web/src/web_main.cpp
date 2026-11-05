// webgs: the web GS entry point. Native: CLI (replay for parity gates, live
// for bench A/B vs the browser). Emscripten: the same live loop, with AUs
// and 1 Hz stats posted to the page.
#include <atomic>
#include <chrono>
#include <climits>
#include <cerrno>
#include <csignal>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
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
#include "mabur/config_params.h"
#include "mabur/raw_dvr.h"
#include "mav_ring.h"
#include "web_gs.h"
#include "web_phase.h"
#ifdef WEBGS_LIVE
#include "body_queue.h"
#include "link_card.h"
#include "radio_frontend.h"
#include "relay_ring_transport.h"
#include "relay_transport.h"
#include "remote_card.h"
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
#define WEBGS_CONFIG_PATH "/maburgs.toml"   // embedded in the WASM FS
#else
#define WEBGS_CONFIG_PATH MABUR_SOURCE_DIR "/gs/bundle/maburgs.default.toml"
#endif
#endif

namespace {

// Page -> core requests. Written from the
// browser main thread (Module._webgs_stop/_webgs_set_rec) or a native signal
// handler; read by the live loop. Shared memory under pthreads: no proxying.
std::atomic<bool> g_stop{false};
std::atomic<int> g_rec{-1};   // -1 never pressed, 0 off, 1 on

// Page -> core IDR requests (webgs_request_idr): a monotonically increasing
// count; the loop forwards it, and its low byte rides every RCF.
std::atomic<uint32_t> g_idr_req{0};

// Page -> core phase-lock offset (webgs_phase_tick, web_phase.h): the page's
// performance.now() and the core thread's now_us() are different clocks, so
// the page's tick only hands over its RETURN VALUE plus a sequence bump; the
// live loop re-stamps the value's set_us on its own clock when it sees the
// seq change (see live_loop), then feeds webgs::snr_off_to_send() into the
// RCF every iteration so a stalled or closed page fades back to 0.
std::atomic<int> g_phase_val{0};
std::atomic<uint32_t> g_phase_seq{0};

#ifdef WEBGS_PAGE
// One per module, created by webgs_phase_reset; the other webgs_phase_*
// exports are no-ops before it exists.
std::unique_ptr<webgs::WebPhase> g_phase;
#endif

// Browser storage for local recordings: true once the startup OPFS probe
// passed. Always false natively.
std::atomic<bool> g_opfs_ok{false};

// Page -> core local-recording request (webgs_set_local_rec). seq bumps on
// every call so two presses between loop iterations are both applied.
struct LocalRecReq {
  std::mutex mu;
  uint32_t seq = 0;
  bool on = false;
  std::string name;
} g_lrec;

// Page -> core parameter requests (webgs_param_refresh/webgs_param_set):
// queued here, applied to WebGs::params() once per loop iteration. `sets`
// is (id, text) pairs in call order; an id the table doesn't have
// (mabur::cfgparam::find returns -1) is applied as an out-of-range row,
// which ParamClient::set refuses without touching any row -- there is
// nothing to record a refusal against.
struct ParamReq {
  std::mutex mu;
  bool refresh = false;
  std::vector<std::pair<std::string, std::string>> sets;
} g_param;

// MAVLink tunnel (mav_tunnel.h, no UI yet): the page's webgs_mav_push /
// webgs_mav_take exports hand raw bytes across the page/core thread
// boundary (WebGs itself is single-threaded, so this mutex is the seam).
// `out`: outbound bytes queued by the page, drained into
// WebGs::mav_push_outbound once per loop iteration. `in`: inbound packets
// the core drained out of WebGs::mav_take_inbound one at a time -- a
// bounded MavRing (mav_ring.h), since nothing calls webgs_mav_take without
// a page MAVLink UI and the drone's once-a-second heartbeat would
// otherwise grow it for the module's whole lifetime.
struct MavBridge {
  std::mutex mu;
  std::vector<uint8_t> out;
  webgs::MavRing in;
} g_mav;
// False before the core exists and after it exits: webgs_mav_push then
// has nothing to drain into, so it refuses rather than growing `out`
// forever.
std::atomic<bool> g_mav_running{false};

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

// `PARAMS <json>` line: the ParamClient readout (webgs::params_json), pushed
// once at start (push_params_at_start) and then whenever its seq() moves
// (push_params_if_changed).
void report_params(const std::string& json) {
#ifdef WEBGS_PAGE
  MAIN_THREAD_ASYNC_EM_ASM({ if (Module['onParams']) Module['onParams'](UTF8ToString($0)); _free($0); },
                           strdup(json.c_str()));
#else
  std::printf("PARAMS %s\n", json.c_str());
  std::fflush(stdout);
#endif
}

// The start-of-run push, in both modes: the page shows the table's rows
// (greyed, values null) from the moment the core runs, not only once a GS
// session links. Spotter has no client, so this is its only push; in GS
// mode it also syncs the caller's seq cursor, so push_params_if_changed
// only fires on what moves after it.
void push_params_at_start(const webgs::WebGs& g, uint64_t now_ms, uint32_t& param_seq_seen) {
  param_seq_seen = g.params() ? g.params()->seq() : 0;
  report_params(webgs::params_json(g, now_ms));
}

// Pushes params_json to the page/console iff the client's seq moved since
// the caller's own cursor -- never more than once per poll, never while
// nothing changed. Spotter has no client (g.params() == nullptr): its only
// push is push_params_at_start.
void push_params_if_changed(const webgs::WebGs& g, uint64_t now_ms, uint32_t& param_seq_seen) {
  if (!g.params()) return;
  const uint32_t sq = g.params()->seq();
  if (sq == param_seq_seen) return;
  param_seq_seen = sq;
  report_params(webgs::params_json(g, now_ms));
}

// One per armed recording, when it is sealed: the page downloads
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
      p, static_cast<int>(a.data.size()), static_cast<double>(a.pts_us), a.sid,
      static_cast<int>(a.flags | (a.salvaged ? 0x40 : 0)),
      a.complete ? 1 : 0, static_cast<double>(a.t_complete_us), hv, hv_len, cap_us, t_emit_ms,
      // FEC/assembly segment: first body -> AU
      // complete, core clock; 0 when FrameStream never saw a nonzero
      // body_mono_us for this AU (gs/src/frame_stream.cpp). Page-only
      // (WEBGS_PAGE); replay's AuFileWriter path is untouched.
      static_cast<double>(a.t_first_us));
#else
  (void)a;
#endif
}

// Hands one MSP OSD screen to the page.
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
// DvrSink over an OPFS file. Destruction only closes -- never
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
  wo.rz_nonce = 1;   // reproducible DISC nonce + RCF tags (native/WASM parity)
  webgs::WebGs g(cfg, o.mode, cfg.radio.channels.front(), maburgs::derived_width(cfg.link), {}, 0,
                std::move(io), wo);
  // The PARAMS push (see push_params_if_changed): replay has no page to
  // drive webgs_param_refresh/webgs_param_set, but it is the only CLI
  // driver of WebGs there is, so it is what a by-hand check of the `PARAMS`
  // line runs against (--fake-ack opens a GS session to list over).
  uint32_t param_seq_seen = 0;
  push_params_at_start(g, 0, param_seq_seen);

  uint64_t last_us = 0;
  bool first = true;
  while (auto m = src.next()) {
    if (first && o.fake_ack) g.inject_disc_ack_for_replay(m->mono_us);
    first = false;
    g.on_rx(*m);
    g.tick(m->mono_us);
    last_us = m->mono_us;
    push_params_if_changed(g, m->mono_us / 1000, param_seq_seen);
  }
  // Match the dry-run's final poll on the ms clock: last_ms + gap + 1.
  g.tick((last_us / 1000 + static_cast<uint64_t>(cfg.video.frame_gap_timeout_ms) + 1) * 1000);
  push_params_if_changed(g, last_us / 1000, param_seq_seen);
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
// steady_clock since epoch: the same clock RadioFrontend stamps bodies with
// (mono_us_now) and RemoteCard's now_ms, so WebGs sees one clock.
uint64_t now_us() {
  return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
                                   std::chrono::steady_clock::now().time_since_epoch())
                                   .count());
}

struct LiveOpts {
  std::string config = WEBGS_CONFIG_PATH;
  std::string overlay;       // optional TOML merged over config (the page's /overlay.toml)
  webgs::Mode mode = webgs::Mode::Gs;
  int ch = -1, width = -1;   // -1 = the pin or first member / the derived width
  int secs = 0;              // 0 = until the card goes away
  std::string bad_chw;       // non-numeric --ch/--w, reported by run_live
  std::string relay;         // --relay host:port: mabur-relay v4 over UDP instead of USB
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

// The body-queue-consuming half of live: local recording, page requests
// (vtx rec / IDR / local rec), the on_rx/tick loop and its STATS cadence.
// The roster (one card: USB or relay) is run_live's; WebGs here ticks the
// cards, transmits through them (GS mode) and runs the channel core. g is
// local, so its core threads are joined before run_live stops the cards.
int live_loop(const LiveOpts& o, const maburgs::Config& cfg, uint8_t ch, int width,
              std::vector<std::unique_ptr<maburgs::LinkCard>>& cards, int n_usb,
              maburgs::BodyQueue& q) {
  // Local recording, fed on this thread from the AU callback, before the
  // page hand-off.
  mabur::RawDvr dvr;
  std::string lrec_name;       // the armed/last recording's file name
  std::string lrec_base_name;  // the name it was armed with, before any roll suffix
  uint32_t lrec_roll_n = 0;    // roll suffix counter, reset whenever a recording (re)arms
  bool lrec_armed = false;     // a recording is armed or open and not yet reported
  int lrec_err = 0;            // last reported code (3 = no OPFS)
  // lrec_err stays set after an error, so the stats show state 3 until the
  // next start: the UI keeps REC! until the next press.
  uint32_t lrec_seq = 0;
#ifdef WEBGS_PAGE
  // A coded-size change mid-recording seals the current OPFS file -- reported
  // the same way the record button's own stop reports a seal -- and rolls to
  // <base>-r<N>.mp4, N counting up within this recording; recFileName (the
  // page's collision check) never produces that suffix, so it can never
  // collide with a later recording's own name.
  //
  // This callback runs on the core thread from inside the AU callback
  // (RawDvr::feed() calls it synchronously), where OpfsFile::open() must
  // never run: its ASYNCIFY await needs the core loop, not an AU callback
  // (opfs_file.h). So it only computes the rolled name and returns nullptr
  // with it set, deferring the open -- picked up below, in the loop.
  dvr.set_next_sink([&](std::string* name) {
    report_rec_closed(lrec_name, dvr.last_sealed_bytes(), 0);
    std::string base = lrec_base_name;
    const std::string suffix = ".mp4";
    if (base.size() >= suffix.size() &&
        base.compare(base.size() - suffix.size(), suffix.size(), suffix) == 0) {
      base.resize(base.size() - suffix.size());
    }
    const std::string rolled = base + "-r" + std::to_string(++lrec_roll_n) + ".mp4";
    *name = rolled;
    lrec_name = rolled;
    return std::unique_ptr<mabur::DvrSink>();
  });
#endif
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
  io.on_log = [](const std::string& l) { std::fprintf(stderr, "%s\n", l.c_str()); };
  io.on_channel_store = [](uint8_t c) {
    std::printf("CHANNEL %u\n", static_cast<unsigned>(c));
    std::fflush(stdout);
  };
  std::vector<maburgs::LinkCard*> ptrs;
  for (auto& c : cards) ptrs.push_back(c.get());
  webgs::WebGs g(cfg, o.mode, ch, width, ptrs, n_usb, std::move(io));
  {
    // Fresh buffers for this run: a page reconnect must not replay bytes
    // queued against the previous core.
    std::lock_guard<std::mutex> lk(g_mav.mu);
    g_mav.out.clear();
    g_mav.in.clear();
  }
  g_mav_running.store(true, std::memory_order_release);
  // The table's rows reach the page now, before any session (and, in
  // Spotter, ever) gives the client a seq move to push on.
  uint32_t param_seq_seen = 0;
  push_params_at_start(g, now_us() / 1000, param_seq_seen);

#ifndef WEBGS_PAGE
  std::signal(SIGINT, [](int) { g_stop.store(true); });
  std::signal(SIGTERM, [](int) { g_stop.store(true); });
#endif

  int rc = 0;
  bool relay_owned = false;   // Health::Owned seen (run_live waited for it; latched here anyway)
  std::vector<mabur::node::RxBody> batch;
  uint64_t next_stat = now_us() + 1000000;
  int applied_rec = -1;
  uint32_t applied_idr = 0;
  // The page's phase-lock offset, re-stamped on THIS clock (now_us(), not
  // the page's performance.now()) whenever webgs_phase_tick bumps the seq.
  uint32_t phase_seq_seen = 0;
  // param_seq_seen (above) is the ParamClient's last-pushed seq, so the
  // loop pushes a fresh PARAMS line at most once per iteration, only when
  // something in it moved.
  int16_t phase_val = 0;
  uint64_t phase_set_us = 0;   // 0: snr_off_to_send reads this as stale -> 0
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
          lrec_base_name = name;
          lrec_roll_n = 0;
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
#ifdef WEBGS_PAGE
    // lrec_armed is belt and braces: stop() (seal_local_rec(), above)
    // already clears roll_deferred() itself, but a deferred roll of a
    // recording the operator has since stopped must never open a file
    // under any future reordering of this loop.
    if (dvr.roll_deferred() && lrec_armed) {
      // The size-change roll deferred here (the next_sink lambda above
      // never opens OPFS itself -- that await needs the core loop, not an
      // AU callback, opfs_file.h): finish it now, from the loop, exactly
      // like a fresh start(). A failed open leaves dvr in Error with
      // lrec_armed still set, which the Error check below seals and
      // reports the same way any other open failure is.
      std::unique_ptr<mabur::DvrSink> sink;
      if (auto f = webgs::OpfsFile::open(dvr.path())) sink = std::make_unique<OpfsSink>(std::move(f));
      dvr.start(std::move(sink), 0, 0);
      // The new file opens on an IRAP only: ask the drone for one now,
      // same as a fresh start() does.
      g_idr_req.fetch_add(1, std::memory_order_acq_rel);
    }
#endif
    {
      // Outbound MAVLink (webgs_mav_push): whatever the page queued since
      // the last iteration, forwarded to the tunnel's parser once per
      // tick -- flush() (inside g.tick()) is what actually sends it.
      std::vector<uint8_t> mav_out;
      { std::lock_guard<std::mutex> lk(g_mav.mu); mav_out.swap(g_mav.out); }
      if (!mav_out.empty()) g.mav_push_outbound(mav_out.data(), mav_out.size());
    }
    batch.clear();
    q.drain(batch, 5);
    for (const auto& m : batch) g.on_rx(m);
    g.tick(now_us());
    {
      // Page param requests (webgs_param_refresh/webgs_param_set): queued
      // since the last iteration, applied to the client now. A set's own
      // PARAM_EXT_SET only reaches the tunnel's outbound queue here --
      // mav_->flush() already ran this iteration's g.tick(), so it goes out
      // on the NEXT one, the same one-iteration lag webgs_set_rec and the
      // IDR bump already have.
      bool refresh = false;
      std::vector<std::pair<std::string, std::string>> sets;
      {
        std::lock_guard<std::mutex> lk(g_param.mu);
        refresh = g_param.refresh;
        g_param.refresh = false;
        sets.swap(g_param.sets);
      }
      for (const auto& kv : sets) {
        const int idx = mabur::cfgparam::find(kv.first.c_str());
        std::string why;
        g.param_set(idx >= 0 ? static_cast<size_t>(idx) : static_cast<size_t>(-1), kv.second, &why);
      }
      if (refresh) g.param_refresh();
      push_params_if_changed(g, now_us() / 1000, param_seq_seen);
    }
    {
      // Inbound MAVLink (webgs_mav_take): drain whatever the tunnel
      // delivered this tick into the bridge, one whole packet at a time
      // (mav_take_inbound's own contract); the bridge's ring applies its
      // own cap, dropping the oldest queued packet(s) if the page never
      // calls webgs_mav_take to make room.
      uint8_t mbuf[512];   // > MAVLINK_MAX_PACKET_LEN (~280 B)
      for (;;) {
        const size_t n = g.mav_take_inbound(mbuf, sizeof mbuf);
        if (n == 0) break;
        std::lock_guard<std::mutex> lk(g_mav.mu);
        g_mav.in.push(mbuf, n);
      }
    }
    if (const uint32_t sq = g_phase_seq.load(std::memory_order_acquire); sq != phase_seq_seen) {
      phase_seq_seen = sq;
      phase_val = static_cast<int16_t>(g_phase_val.load(std::memory_order_acquire));
      phase_set_us = now_us();
    }
    if (g.vrx()) g.vrx()->set_snr_off(webgs::snr_off_to_send(phase_val, phase_set_us, now_us()));
    if (lrec_armed && dvr.state() == mabur::RawDvr::State::Error) seal_local_rec();
    const char* lost = nullptr;
    if (auto* rcard = dynamic_cast<maburgs::RemoteCard*>(cards[0].get())) {
      const auto h = rcard->health();
      if (h == maburgs::RemoteCard::Health::Owned) relay_owned = true;
      // After ownership: Lost is fatal; Taken OR Refused means another
      // client holds the relay (restart_when_refused is off, so a refusal
      // is never reset back to Connecting). TuneFailed and Connecting are
      // NOT fatal here: a core-ordered relay TUNE during a hop reads
      // not-tuned for ~50 ms, and RelayClient::tune_failed()'s window is
      // measured from start(), so it reads true during any post-ownership
      // retune.
      if (relay_owned) {
        switch (h) {
          case maburgs::RemoteCard::Health::Lost:    lost = "relay lost"; break;
          case maburgs::RemoteCard::Health::Taken:
          case maburgs::RemoteCard::Health::Refused: lost = "relay taken by another client"; break;
          default: break;
        }
      }
    } else if (!cards[0]->alive()) {
      lost = "card lost";
    }
    if (lost && batch.empty()) {
      report_error("%s", lost);
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
    // lrec_name is page-built from [A-Za-z0-9.-] only (localrec.js recFileName),
    // so it needs no JSON escaping.
    char extra[384];
    std::snprintf(extra, sizeof extra, ",\"qdrop\":%llu,\"lrec_avail\":%d,\"lrec_state\":%d,\"lrec_bytes\":%llu,"
                  "\"lrec_err\":%d,\"lrec_name\":\"%s\"",
                  static_cast<unsigned long long>(q.dropped()), g_opfs_ok.load() ? 1 : 0, lst,
                  static_cast<unsigned long long>(dvr.bytes()), lrec_err, lrec_name.c_str());
    std::string extra2;
    if (auto* rcard = dynamic_cast<maburgs::RemoteCard*>(cards[0].get())) {
      if (const auto st = rcard->relay_stats()) extra2 = webgs::relay_stats_fields(*st);
    } else if (auto* fe = dynamic_cast<maburgs::RadioFrontend*>(cards[0].get())) {
      int64_t p99 = 0, mx = 0;
      fe->take_usb_late(p99, mx);
      char b[160];
      std::snprintf(b, sizeof b, ",\"radio\":\"usb\",\"usb_p99_us\":%lld,\"usb_max_us\":%lld,\"txfail\":%llu",
                    static_cast<long long>(p99), static_cast<long long>(mx),
                    static_cast<unsigned long long>(fe->tx_fail()));
      extra2 = b;
    }
    report_stats(j + extra + extra2 + "}");
  }
  // Every exit (Disconnect, card lost) seals the local recording before the
  // module goes away, so the page can download it.
  seal_local_rec();
  // WebGs (and `g`) is about to go out of scope: webgs_mav_push must stop
  // accepting bytes it would never drain into a core again.
  g_mav_running.store(false, std::memory_order_release);
  return rc;
}

// Cards the web core opens; devourer's chip-id read picks the driver.
// 0bda:a81a/881a = RTL8812EU (Jaguar3), 0bda:8812 = RTL8812AU or EU,
// 0bda:c812 = RTL8812CU (Jaguar3 8822C, e.g. Comfast CF-924AC V3; the OS
// must eject its 0bda:1a2b driver-CD mode first, docs/web-gs.md),
// 2357:011e/0120/0122 = TP-Link RTL8821AU
// (Jaguar1, 1T1R). The page's WebUSB chooser filters by vendor only
// (logic.mjs USB_FILTERS).
const std::vector<maburgs::RadioFrontend::UsbId> kCards = {
    {0x0bda, 0xa81a}, {0x0bda, 0x881a}, {0x0bda, 0x8812}, {0x0bda, 0xc812},
    {0x2357, 0x011e}, {0x2357, 0x0120}, {0x2357, 0x0122},
};

int run_live(const LiveOpts& o) {
  maburgs::Config cfg;
  if (!load_cfg(o.config, o.overlay, cfg)) return 2;
  std::fprintf(stderr, "webgs: link key %s (%s)\n",
               mabur::key_fingerprint(cfg.link.key).c_str(), cfg.link.key_source.c_str());
  if (!o.bad_chw.empty()) {
    report_error("bad channel/width: %s", o.bad_chw.c_str());
    return 2;
  }
  const int ch_i = o.ch >= 0 ? o.ch : (cfg.radio.pin ? *cfg.radio.pin : cfg.radio.channels.front());
  const int width = o.width >= 0 ? o.width : maburgs::derived_width(cfg.link);
  // maburgs's own loader checks on the override (the page picks ch/w, not
  // the config): set membership, range, 20|40, HT40 pair.
  if (auto e = webgs::channel_width_error(cfg, o.mode, ch_i, width)) {
    report_error("bad channel/width: %s", e->c_str());
    return 2;
  }
  const uint8_t ch = static_cast<uint8_t>(ch_i);
  std::string set;
  for (size_t i = 0; i < cfg.radio.channels.size(); ++i)
    set += (i ? "," : "") + std::to_string(cfg.radio.channels[i]);
  std::printf("webgs live: mode %s ch %u width %d set [%s] %s\n",
              o.mode == webgs::Mode::Gs ? "gs" : "spotter", ch, width, set.c_str(),
              cfg.radio.pin ? "pinned" : "auto");
  std::fflush(stdout);

  maburgs::BodyQueue q;
  std::vector<std::unique_ptr<maburgs::LinkCard>> cards;
  int n_usb = 0;
  if (o.relay.empty()) {
    maburgs::RadioFrontend::Cfg fc;
    fc.ids = kCards;
    fc.channel = ch;
    // Every card opens at the link width, the GS mode's boot scout card
    // included (it scans at the link width, as maburgs's does).
    fc.width_mhz = static_cast<uint8_t>(width);
    fc.card_id = 0;
    fc.usb_late_gauge = true;
    auto fe = std::make_unique<maburgs::RadioFrontend>(fc, q);
    if (!fe->open_and_start()) {
      const std::string& e = fe->open_error();
      report_error("%s", e == "no device" ? "no RTL card" : e.c_str());
      return 1;
    }
    cards.push_back(std::move(fe));
    n_usb = 1;
  } else {
    maburgs::RemoteCard::Cfg rcfg;
    rcfg.addr = o.relay;
    rcfg.channel = ch;
    rcfg.width_mhz = static_cast<uint8_t>(width);
    rcfg.card_id = 0;
    rcfg.restart_when_refused = false;   // one relay, one client: a refusal is reported, not retried
    maburgs::RemoteCard::OpenFn open =
#ifdef __EMSCRIPTEN__
        [](const std::string&, std::string&) { return webgs::open_ring_transport(); };
#else
        [](const std::string& a, std::string& err) { return maburgs::open_udp_transport(a, err); };
#endif
    auto card = std::make_unique<maburgs::RemoteCard>(rcfg, q, open, [] { return now_us() / 1000; });
    if (!card->open_and_start()) {
      report_error("relay unreachable");
      return 1;
    }
    // Wait for ownership (both modes: one relay, one client). Bounded by
    // RelayClient's own windows: Lost after kLostMs, Refused/TuneFailed after
    // kTuneWindowMs. No WebGs yet, so the card is ticked here.
    for (;;) {
      if (g_stop.load(std::memory_order_acquire)) {
        card->stop();
        std::printf("DONE\n");
        std::fflush(stdout);
        return 0;
      }
      card->tick(now_us() / 1000);
      const auto h = card->health();
      if (h == maburgs::RemoteCard::Health::Owned) break;
      const char* err = nullptr;
      switch (h) {
        case maburgs::RemoteCard::Health::Lost:       err = "relay unreachable"; break;
        case maburgs::RemoteCard::Health::Refused:    err = "relay owned by another client"; break;
        case maburgs::RemoteCard::Health::TuneFailed: err = "relay cannot tune"; break;
        case maburgs::RemoteCard::Health::Taken:      err = "relay taken by another client"; break;
        default: break;
      }
      if (err) {
        report_error("%s", err);
        card->stop();
        return 1;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    cards.push_back(std::move(card));
  }
  // WebGs (and its core threads) are gone when live_loop returns.
  const int rc = live_loop(o, cfg, ch, width, cards, n_usb, q);
  // RadioFrontend::stop holds the WebUSB early-release teardown. Returning
  // from main lets -sEXIT_RUNTIME fire Module.onExit(rc).
  for (auto& c : cards) c->stop();
  q.close();
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
               "                  (ch/w default to the pin or the first member / the derived width)\n"
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

// MAVLink tunnel seam (no UI): raw packet bytes both ways. webgs_mav_push
// queues bytes from a local MAVLink peer for the core to parse and tunnel
// to the drone; webgs_mav_take pops one whole packet (drone -> local peer)
// per call, up to `cap` bytes -- never split, and never more than one
// packet per call (loop until it returns 0 to drain everything queued). A
// `cap` of at least MAVLINK_MAX_PACKET_LEN (~280 B) always succeeds; a
// smaller one leaves a too-big packet queued rather than returning a
// partial one. The ring behind this is bounded (mav_ring.h): with no page
// MAVLink UI yet, nothing may call this for the module's whole lifetime,
// so unread packets age out oldest-first instead of growing forever.
EMSCRIPTEN_KEEPALIVE int webgs_mav_push(const uint8_t* p, int n) {
  if (!g_mav_running.load(std::memory_order_acquire) || n <= 0) return 0;
  std::lock_guard<std::mutex> lk(g_mav.mu);
  g_mav.out.insert(g_mav.out.end(), p, p + n);
  return n;
}
EMSCRIPTEN_KEEPALIVE int webgs_mav_take(uint8_t* out, int cap) {
  if (cap <= 0) return 0;
  std::lock_guard<std::mutex> lk(g_mav.mu);
  return static_cast<int>(g_mav.in.take(out, static_cast<size_t>(cap)));
}

// webgs_param_refresh/webgs_param_set: queue a request for the loop to
// apply to WebGs::params() (g_param, like g_lrec) -- both copy their
// arguments into it before returning, since the page frees the strings it
// passed right after the call. 0 queued, 1 refused (the core is not
// running, so there is nothing to apply it to). An unknown id or a bad
// value is not refused here: it reaches the client's own refusal path
// (mabur::cfgparam::find, ParamClient::set) once the loop drains the
// request, reported through the next `onParams`/`PARAMS` push, not through
// this call's return value.
EMSCRIPTEN_KEEPALIVE int webgs_param_refresh() {
  if (!g_mav_running.load(std::memory_order_acquire)) return 1;
  std::lock_guard<std::mutex> lk(g_param.mu);
  g_param.refresh = true;
  return 0;
}
EMSCRIPTEN_KEEPALIVE int webgs_param_set(const char* id, const char* text) {
  if (!g_mav_running.load(std::memory_order_acquire)) return 1;
  std::lock_guard<std::mutex> lk(g_param.mu);
  g_param.sets.emplace_back(id ? id : "", text ? text : "");
  return 0;
}

// webgs_phase_*: the page's phase lock (web_phase.h). The page calls
// webgs_phase_tick from a 50 ms timer and webgs_phase_frame once per decoded
// frame; its rAF loop only fits the vsync grid and releases held frames --
// the browser has no release thread to run this from. All main-thread; one
// WebPhase per module, (re)created by webgs_phase_reset. The other exports
// are no-ops (state -1, zeros) before that first call.
EMSCRIPTEN_KEEPALIVE void webgs_phase_reset() {
  g_phase = std::make_unique<webgs::WebPhase>();
}
EMSCRIPTEN_KEEPALIVE void webgs_phase_frame(double pts_us, double margin_us, double period_us,
                                             double now_us) {
  // pts_us is the drone's 32-bit pts (the page passes the low 32 bits of
  // its unwrapped timeline); going through uint64_t keeps a larger value
  // a defined wrap instead of an out-of-range float-to-uint32 conversion.
  if (g_phase)
    g_phase->on_frame(static_cast<uint32_t>(static_cast<uint64_t>(pts_us)), margin_us, period_us,
                       static_cast<uint64_t>(now_us));
}
EMSCRIPTEN_KEEPALIVE void webgs_phase_discontinuity(double now_us) {
  if (g_phase) g_phase->on_discontinuity(static_cast<uint64_t>(now_us));
}
EMSCRIPTEN_KEEPALIVE int webgs_phase_tick(double now_us, double panel_hz, int low_power,
                                          int visible) {
  if (!g_phase) return 0;
  const auto lp = low_power == 1 ? maburplay::RateMatch::LowPower::kYes
                : low_power == 2 ? maburplay::RateMatch::LowPower::kUnknown
                                 : maburplay::RateMatch::LowPower::kNo;
  const int16_t v = g_phase->tick(static_cast<uint64_t>(now_us), panel_hz, lp, visible != 0);
  // Handed to the core thread as a value + seq bump, not the WebPhase
  // itself: the core thread stamps its OWN clock on the value (live_loop),
  // since now_us here is the page's performance.now(), not now_us().
  g_phase_val.store(v, std::memory_order_relaxed);
  g_phase_seq.fetch_add(1, std::memory_order_release);
  return v;
}
EMSCRIPTEN_KEEPALIVE int webgs_phase_state() {
  return g_phase ? static_cast<int>(g_phase->state()) : -1;
}
EMSCRIPTEN_KEEPALIVE double webgs_phase_err_ms() { return g_phase ? g_phase->err_ms() : 0.0; }
EMSCRIPTEN_KEEPALIVE double webgs_phase_centre() { return g_phase ? g_phase->centre_mfps() : 0.0; }
EMSCRIPTEN_KEEPALIVE int webgs_phase_mismatch() { return g_phase && g_phase->mismatch() ? 1 : 0; }
EMSCRIPTEN_KEEPALIVE double webgs_phase_src_hz() { return g_phase ? g_phase->src_hz() : 0.0; }
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
