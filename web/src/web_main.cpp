// webgs: the web GS entry point. Native: CLI (replay for parity gates, live
// for bench A/B vs the browser). Emscripten: the same live loop, with AUs
// and 1 Hz stats posted to the page (spec 2026-09-27-web-gs §2.3).
#include <algorithm>
#include <atomic>
#include <chrono>
#include <climits>
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
#endif
#ifdef WEBGS_PAGE
#include <emscripten/em_asm.h>
#include "mabur/hevc_params.h"
#include "mabur/nal.h"
#endif

#ifndef WEBGS_CONFIG_PATH
#ifdef __EMSCRIPTEN__
#define WEBGS_CONFIG_PATH "/maburgs.toml"   // embedded in the WASM FS (Task 6)
#else
#define WEBGS_CONFIG_PATH MABUR_SOURCE_DIR "/gs/bundle/maburgs.default.toml"
#endif
#endif

namespace {

// ---- page / console reporting --------------------------------------------

// `ERROR <reason>` lines: the page maps the reason to text. Natively a
// stdout line; in the browser Module.onError.
void report_error(const char* fmt, ...) {
  char buf[256];
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
      hv = static_cast<uint8_t*>(std::malloc(rec.size()));
      std::memcpy(hv, rec.data(), rec.size());
      hv_len = static_cast<int>(rec.size());
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
        Module['onAu'](buf, $2, $3, $4, $5, $6, hvcc, $9, $10);
      },
      p, static_cast<int>(a.data.size()), static_cast<double>(a.pts_us), a.sid, a.flags,
      a.complete ? 1 : 0, static_cast<double>(a.t_complete_us), hv, hv_len, cap_us, t_emit_ms);
#else
  (void)a;
#endif
}

bool load_cfg(const std::string& path, maburgs::Config& cfg) {
  try {
    cfg = maburgs::load_config(path);
  } catch (const std::exception& e) {
    report_error("config %s", e.what());
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
  std::string in, out, config = WEBGS_CONFIG_PATH, trace;
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
  if (!load_cfg(o.config, cfg)) return 2;
  maburgs::FrameFileSource src(o.in, {1, o.drop_pct, o.seed});
  if (!src.ok()) { std::fprintf(stderr, "error: cannot read %s\n", o.in.c_str()); return 2; }
  webgs::AuFileWriter w;
  if (!w.open(o.out.c_str())) { std::fprintf(stderr, "error: cannot write %s\n", o.out.c_str()); return 2; }
  FILE* trace = nullptr;
  if (!o.trace.empty() && !(trace = std::fopen(o.trace.c_str(), "w"))) {
    std::fprintf(stderr, "error: cannot write %s\n", o.trace.c_str());
    return 2;
  }

  webgs::Io io;
  io.on_au = [&](webgs::Au&& a) { w.write(a); };
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
  webgs::Mode mode = webgs::Mode::Gs;
  int ch = -1, width = -1;   // -1 = the config's radio.channel / radio.width
  int secs = 0;              // 0 = until the card goes away
};

int run_live(const LiveOpts& o) {
  maburgs::Config cfg;
  if (!load_cfg(o.config, cfg)) return 2;
  const uint8_t ch = static_cast<uint8_t>(o.ch >= 0 ? o.ch : cfg.radio.channel);
  const int width = o.width >= 0 ? o.width : cfg.radio.width;
  if (width != 20 && width != 40) { report_error("bad width %d", width); return 2; }
  std::printf("webgs live: mode %s ch %u width %d\n",
              o.mode == webgs::Mode::Gs ? "gs" : "spotter", ch, width);
  std::fflush(stdout);

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
    return 1;
  }
  libusb_free_device_list(list, 1);
  auto logger = std::make_shared<Logger>();
  std::shared_ptr<devourer::UsbDeviceLock> lock;
  if (int rc = devourer::claim_interface_then_reset(h, 0, logger, true, lock); rc != 0) {
    report_error("claim failed rc=%d", rc);
    return 1;
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
  if (!rtl) { report_error("unsupported chip"); return 1; }
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

  webgs::Io io;
  // Can fire from inside on_rx (spotter drone-restart reset) as well as from
  // tick(): emit_au keeps no glue state.
  io.on_au = [](webgs::Au&& a) { emit_au(std::move(a)); };
  uint16_t tx_seq = 0;
  uint64_t txfail = 0;
  if (o.mode == webgs::Mode::Gs)
    io.send = [&](const std::vector<uint8_t>& body) {
      const auto f = maburgs::build_control_frame(tx_seq, body.data(), body.size());
      tx_seq = static_cast<uint16_t>((tx_seq + 1) & 0xFFF);
      if (!rtl->send_packet(f.data(), f.size())) ++txfail;
    };
  webgs::WebGs g(cfg, o.mode, ch, width, std::move(io));

  int rc = 0;
  std::vector<mabur::node::RxBody> batch;
  uint64_t next_stat = now_us() + 1000000;
  for (int s = 0; o.secs == 0 || s < o.secs;) {
    batch.clear();
    q.drain(batch, 5);
    for (const auto& m : batch) g.on_rx(m);
    g.tick(now_us());
    if (rx_ended.load(std::memory_order_acquire) && batch.empty()) {
      report_error("card lost");
      rc = 1;
      break;
    }
    if (now_us() < next_stat) continue;
    next_stat += 1000000;
    ++s;
    int64_t p99 = 0, mx = 0;
    late.take(p99, mx);
    std::string j = webgs::stats_json(g.stats());
    j.pop_back();   // '}'
    char extra[160];
    std::snprintf(extra, sizeof extra, ",\"usb_p99_us\":%lld,\"usb_max_us\":%lld,\"txfail\":%llu,"
                  "\"qdrop\":%llu}",
                  static_cast<long long>(p99), static_cast<long long>(mx),
                  static_cast<unsigned long long>(txfail),
                  static_cast<unsigned long long>(q.dropped()));
    report_stats(j + extra);
  }
  if (!rx_ended.load()) rtl->StopRxLoop();
  q.close();
  rx.join();
  std::printf("DONE\n");
  std::fflush(stdout);
  // devourer/libusb teardown order is not worth getting right for a CLI
  // that is exiting anyway (the spike's choice).
  std::_Exit(rc);
}
#endif  // WEBGS_LIVE

int usage(FILE* out, int rc) {
  std::fprintf(out,
               "usage: webgs replay <frames.bin> <out-aus> [-c config.toml] [--mode gs|spotter]\n"
               "                    [--drop-pct P] [--seed S] [--fixed-gap]\n"
               "                    [--control-trace <file>] [--fake-ack]\n"
#ifdef WEBGS_LIVE
               "       webgs live [-c config.toml] [--ch N] [--w 20|40] [--secs 0]\n"
               "                  [--mode gs|spotter]   (ch/w default to radio.channel/width)\n"
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
    else if (k == "--ch") o.ch = std::atoi(v);
    else if (k == "--w") o.width = std::atoi(v);
    else if (k == "--secs") o.secs = std::atoi(v);
    else if (k == "--mode") { if (!parse_mode(v, o.mode)) return usage(stderr, 2); }
    else return usage(stderr, 2);
  }
  return -1;
}
#endif

}  // namespace

int main(int argc, char** argv) {
#ifdef WEBGS_PAGE
  // The page build. The page passes `arguments` straight through: live
  // options only (--mode, --ch, --w); an optional leading "live" is accepted.
  // (The Node build, webgs_node, has no WEBGS_LIVE and takes the CLI below:
  // replay for the native/WASM parity gate.)
  LiveOpts lo;
  const int first = (argc > 1 && std::string(argv[1]) == "live") ? 2 : 1;
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
