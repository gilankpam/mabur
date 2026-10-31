// maburplay -- GS native player: AU ring -> VideoBackend, with fMP4 DVR.
// Host builds only ever link the null backend; MppBackend /
// DrmPresenter are cross-only (MABUR_PLAYER_HW).
#include <sys/statvfs.h>  // DVR free-space check behind the GS recording block

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "au_ring.h"
#include "au_router.h"
#include "colortrans.h"  // ColorTrans, build_cubic_lut, LutAxis (docs/colortrans.md)
#include "feed_loop.h"
#include "mabur/dvr_mux.h"
#include "mabur/dvr_name.h"
#include "mabur/raw_dvr.h"
#include "gs_font.h"
#include "gs_metrics.h"
#include "gs_layer.h"
#include "osd_compose.h"
#include "gs_source.h"
#include "mabur/hevc_params.h"
#include "mabur/frame_wire.h"
#include "mode_pick.h"  // parse_screen_want, used by parse_screen_mode below
#include "osd_font.h"
#include "osd_raster.h"
#include "osd_source.h"
#include "player_config.h"
#include "rec_button.h"
#include "rec_control.h"  // maburgs::kRecControlPort
#include "ring_client.h"
#include "stream_feeder.h"
#include "video_backend.h"
#include "core_control_client.h"  // sends the button's VTX wish + refresh reports to maburgs
#include "splash_image.h"  // startup splash asset + cover-fit painter

#ifdef MABUR_PLAYER_HW
#include "burn_recorder.h"  // dvr.mode "burned": re-encode with the OSD burnt in
#include "decode_pump.h"    // the decode-output thread: drives backend->poll(), hands frames over
#include "drm_presenter.h"  // KMS atomic NV12 presenter, the default display path
#include "frame_regulator.h"  // phase-aware pts+D display release
#include "phase_lock.h"  // PhaseLock, the sensor-rate controller for the phase lock
#include "rate_match.h"  // RateMatch: is the panel within the phase lock's reach
#include "recorder_tap.h"  // the burned DVR's hook on the display's flip
#include "release_loop.h"  // the release thread: regulator decisions and every present()
#include "wake_queue.h"    // the hand-over and release queues between the threads
#include "gs_draw.h"         // set_colour_inverse() for the GS overlay pre-invert
#include "lat_tracker.h"    // tail latency segments + 1 Hz lat line
#include "lat_log.h"        // lat.log (1 Hz lat: line) and frame.log (per frame) in the session dir
#include "osd_palette.h"    // build_palette() for the encoder-side OSD
#endif
#ifdef MABUR_PLAYER_MPP
#include "mpp_backend.h"    // MppBackend::info_changes()/errors() for --decode-only
#endif
#ifdef MABUR_PLAYER_V4L2REQ
#include "v4l2req_backend.h"  // V4l2ReqBackend counters for --decode-only / the 5 s line
#endif

namespace {

std::atomic<bool> g_stop{false};
void on_signal(int) { g_stop.store(true); }
// SIGUSR1 = one press of the record button, for benches with no hand on the
// GPIO (`kill -USR1 $(pidof maburplay)` toggles a recording mid-session).
std::atomic<bool> g_rec_toggle{false};
void on_rec_toggle(int) { g_rec_toggle.store(true); }

// Monotonic milliseconds. Everything the GS overlay measures -- AU arrival
// jitter, the 1 Hz video/recording marks, the recording clock -- is an
// interval, so it must not be readable from a clock the RTC can step.
uint64_t mono_ms() {
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

// Microsecond twin, for the stamps and marks that feed microsecond
// consumers (the session logs, the phase lock, the rate-match window).
uint64_t mono_us() {
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

void usage() {
  std::fprintf(stderr,
               "usage: maburplay [-c <config.toml>] [--oneshot] "
               "[--backend null|mpp|v4l2req]\n"
               "                 [--decode-only --seconds N] [--fps-log]\n"
               "       maburplay --mux-annexb <in.265> <out.mp4>\n"
               "           (test support: mux a raw Annex-B HEVC elementary\n"
               "            stream straight through HevcParams+DvrMux to an\n"
               "            fMP4 file, bypassing the ring/backend entirely --\n"
               "            used by the host e2e's real-HEVC decode gate)\n"
               "       maburplay --osd-render <snap.bin> --out-osd <out.bin>\n"
               "                 [--font <f.mfont>] [--screen WxH] "
               "[--scale sharp|fill]\n"
               "           (test support: render one MSP DisplayPort snapshot\n"
               "            through the real font/layout/raster path into a\n"
               "            heap surface and dump it -- no ring, no backend,\n"
               "            no DRM. Dump format: 16-byte header {u32 'OSDR',\n"
               "            u32 width, u32 height, u32 stride_px} followed by\n"
               "            height*stride_px little-endian ARGB32 words. This\n"
               "            is the host e2e's OSD pixel-path gate.)\n"
               "       maburplay --gs-render <snap.json> --out-gs <out.bin>\n"
               "                 [--gsfont F] [--screen WxH] [--stale]\n"
               "                 [--rec recording|armed|fault] "
               "[--rec-elapsed N]\n"
               "                 [--fps N] [--jit N] [--mbps N] [--res WxH]\n"
               "                 [--lat P50/P99]\n"
               "           (test support: render one stats-sideport snapshot\n"
               "            through the live GS overlay path and dump ARGB.\n"
               "            No ring, no backend, no DRM. Same 16-byte dump\n"
               "            header as --osd-render. This is the host e2e's GS\n"
               "            pixel-path gate.)\n"
               "\n"
               "--decode-only --seconds N: drive the backend straight off\n"
               "  the ring for N seconds with no presenter, counting frames\n"
               "  via the FrameSink; prints one line of stats JSON\n"
               "  {\"frames\":N,\"fps\":X,\"fps_active\":X,\"info_changes\":N,\n"
               "   \"errors\":N,\"errors_after_sync_3s\":N,\"concealed\":N} and exits. fps is\n"
               "  over the whole window; fps_active is over first-to-last\n"
               "  decoded frame only (excludes the cold-attach/sid0-join\n"
               "  wait). This is the hardware decode gate.\n"
               "\n"
               "--fps-log: normal (non-decode-only) run only -- once per\n"
               "  second, prints \"fps-log: fps=X flips/s=X repl=N frames=N "
               "commit_errors=N async=on|off|probing osd_screens=N\n"
               "  osd_dgrams=N osd_commit_errors=N\\n\" to stderr. Under\n"
               "  MABUR_PLAYER_HW with the DrmPresenter display path active;\n"
               "  a no-op flag otherwise (host/null-backend builds just log\n"
               "  fps/frames with no presenter fields). With dvr.mode\n"
               "  \"burned\" and the recorder running, six more fields are\n"
               "  APPENDED: burn_in/burn_enc/burn_drop/burn_flush/burn_err/\n"
               "  burn_osdrej -- flips tapped on the vblank grid after\n"
               "  dvr.burned.every thinning / encode() calls that produced a\n"
               "  packet / flips DISPLACED in a recorder mailbox or refused\n"
               "  a reference, i.e. the overload signal / frames released by a\n"
               "  flush or reset, i.e. hygiene, NOT overload / frames the\n"
               "  encoder refused / OSD index maps refused for disagreeing\n"
               "  with the encoder's OSD region (nonzero means the file has\n"
               "  NO overlay).\n");
}

// Reads a whole file into memory; empty vector on any failure (including a
// genuinely empty file -- callers that need to distinguish check errno via
// a nonzero-size probe first, none currently do).
std::vector<uint8_t> read_whole_file(const std::string& path) {
  std::FILE* f = std::fopen(path.c_str(), "rb");
  if (!f) return {};
  std::fseek(f, 0, SEEK_END);
  const long sz = std::ftell(f);
  std::vector<uint8_t> buf;
  if (sz > 0) {
    buf.resize(static_cast<size_t>(sz));
    std::fseek(f, 0, SEEK_SET);
    if (std::fread(buf.data(), 1, buf.size(), f) != buf.size()) buf.clear();
  }
  std::fclose(f);
  return buf;
}

// Test support (see usage()): groups an Annex-B elementary stream into AUs
// by NAL type 35 (AUD -- Access Unit Delimiter, one per frame when the
// encoder is run with aud=1) and feeds them through the exact same
// HevcParams/DvrMux calls main()'s live sink uses, with synthetic 60 fps
// pts. This is how the host e2e proves the muxer decodes cleanly against
// real HEVC content, independent of any particular fixture's realism.
int run_mux_annexb(const std::string& in_path, const std::string& out_path) {
  const std::vector<uint8_t> data = read_whole_file(in_path);
  if (data.empty()) {
    std::fprintf(stderr, "maburplay: --mux-annexb: cannot read %s\n", in_path.c_str());
    return 2;
  }

  std::vector<std::vector<uint8_t>> aus;
  std::vector<uint8_t> cur;
  for (const mabur::NalView& nal : mabur::split_nals(data.data(), data.size())) {
    if (nal.type == 35 && !cur.empty()) {  // AUD: starts a new AU
      aus.push_back(std::move(cur));
      cur.clear();
    }
    cur.push_back(0);
    cur.push_back(0);
    cur.push_back(0);
    cur.push_back(1);
    cur.insert(cur.end(), nal.p, nal.p + nal.n);
  }
  if (!cur.empty()) aus.push_back(std::move(cur));

  mabur::HevcParams params;
  mabur::DvrMux dvr;
  bool dvr_open = false;
  uint32_t pts_us = 0;
  const maburplay::BackendCfg bcfg;  // 1920x1080 default; tkhd/stsd metadata
                                      // only -- decode uses the SPS, not this.
  for (const std::vector<uint8_t>& au : aus) {
    const bool key = mabur::au_is_irap(au.data(), au.size());
    if (key) params.feed(au.data(), au.size());
    if (!dvr_open && params.complete()) {
      dvr_open = dvr.open(out_path, params.hvcc(), bcfg.width, bcfg.height);
      if (!dvr_open) {
        std::fprintf(stderr, "maburplay: --mux-annexb: cannot open %s\n", out_path.c_str());
        return 2;
      }
    }
    if (dvr_open) dvr.write_sample(au.data(), au.size(), pts_us, key);
    pts_us += 16667;
  }
  if (!dvr_open) {
    std::fprintf(stderr, "maburplay: --mux-annexb: no VPS/SPS/PPS found in %s\n",
                 in_path.c_str());
    return 2;
  }
  dvr.close();
  return 0;
}

// player_config validates osd.scale at load time, so anything that isn't
// the one alternative is "sharp" -- the default, and the safe one (it never
// upscales fractionally).
maburplay::ScaleMode scale_mode(const std::string& s) {
  return s == "fill" ? maburplay::ScaleMode::kFill : maburplay::ScaleMode::kSharp;
}

// Test support (see usage()): renders one MSP DisplayPort snapshot file
// through the exact OsdFont/compute_layout/OsdRaster path the live OSD uses
// and dumps the ARGB surface. No ring, no backend, no DRM -- the DRM half
// of the OSD cannot run off-hardware, so this is the host-side gate for
// everything below it (atlas scaling, layout arithmetic, the blitter).
int run_osd_render(const std::string& snap_path, const std::string& out_path,
                   const std::string& font_path, int width, int height,
                   maburplay::ScaleMode mode) {
  const std::vector<uint8_t> snap = read_whole_file(snap_path);
  if (snap.empty()) {
    std::fprintf(stderr, "maburplay: --osd-render: cannot read %s\n", snap_path.c_str());
    return 2;
  }
  maburplay::OsdFont font;
  std::string err;
  if (!font.load(font_path, &err)) {
    std::fprintf(stderr, "maburplay: --osd-render: %s\n", err.c_str());
    return 2;
  }
  maburplay::OsdSource src;
  src.feed_open();
  src.set_min_interval_ms(0);  // one-shot: no rate limiting to apply
  if (!src.feed(snap.data(), snap.size(), 1000)) {
    std::fprintf(stderr, "maburplay: --osd-render: no complete screen in %s\n",
                 snap_path.c_str());
    return 2;
  }
  std::vector<uint32_t> px(static_cast<size_t>(width) * height, 0u);
  const maburplay::Surface surf{px.data(), width, height, width};
  maburplay::OsdRaster raster(font, mode);
  raster.draw(src.screen(), surf, nullptr);

  std::FILE* f = std::fopen(out_path.c_str(), "wb");
  if (!f) {
    std::fprintf(stderr, "maburplay: --osd-render: cannot write %s\n", out_path.c_str());
    return 2;
  }
  const uint32_t hdr[4] = {0x5244534FU, static_cast<uint32_t>(width),
                           static_cast<uint32_t>(height), static_cast<uint32_t>(width)};
  std::fwrite(hdr, sizeof(hdr), 1, f);
  std::fwrite(px.data(), 4, px.size(), f);
  std::fclose(f);
  return 0;
}

// Test support (see usage()): renders one sideport snapshot through the
// exact GsFont/GsCompactBar/gs_draw path the live overlay uses and dumps the
// ARGB surface. No ring, no backend, no DRM -- the DRM half of the overlay
// cannot run off-hardware, so this is the host-side gate for everything
// below it (layout arithmetic, thresholds, formatting, the mask blitter).
// Dump format is byte-identical to --osd-render's.
int run_gs_render(const std::string& snap_path, const std::string& out_path,
                  const std::string& font_path, int width, int height, bool stale,
                  const maburplay::GsPlayerState& ps) {
  const std::vector<uint8_t> snap = read_whole_file(snap_path);
  if (snap.empty()) {
    std::fprintf(stderr, "maburplay: --gs-render: cannot read %s\n", snap_path.c_str());
    return 2;
  }
  maburplay::GsFont font;
  std::string err;
  if (!font.load(font_path, &err)) {
    std::fprintf(stderr, "maburplay: --gs-render: %s\n", err.c_str());
    return 2;
  }
  maburplay::GsSource src;
  src.feed_open();
  if (!src.feed(snap.data(), snap.size(), 1000)) {
    std::fprintf(stderr, "maburplay: --gs-render: %s is not a sideport snapshot\n",
                 snap_path.c_str());
    return 2;
  }
  std::unique_ptr<maburplay::GsLayer> ovp = maburplay::make_gs_layer(font);
  maburplay::GsLayer& ov = *ovp;
  if (!ov.layout(width, height, &err)) {
    std::fprintf(stderr, "maburplay: --gs-render: %s\n", err.c_str());
    return 2;
  }
  std::vector<uint32_t> px(static_cast<size_t>(width) * height, 0u);
  const maburplay::Surface surf{px.data(), width, height, width};
  std::vector<maburplay::DirtyRect> rects;
  ov.update(src.snapshot(), stale, ps, surf, &rects);

  std::FILE* f = std::fopen(out_path.c_str(), "wb");
  if (!f) {
    std::fprintf(stderr, "maburplay: --gs-render: cannot write %s\n", out_path.c_str());
    return 2;
  }
  const uint32_t hdr[4] = {0x5244534FU, static_cast<uint32_t>(width),
                           static_cast<uint32_t>(height), static_cast<uint32_t>(width)};
  std::fwrite(hdr, sizeof(hdr), 1, f);
  std::fwrite(px.data(), 4, px.size(), f);
  std::fclose(f);
  return 0;
}

// DVR recording paths come from DvrNamer (dvr_name.h): dir/record-NNNN.mp4,
// indexed off the card rather than stamped with a clock the GS does not
// have. Function-local so both call sites -- the raw path's sync-point open
// and rec_start()'s burned start() -- share one high-water mark.
std::string dvr_filename(const std::string& dir) {
  // Main-loop-thread only (the ring sink and rec_start's
  // start_burn_if_needed), so a plain static needs no synchronisation.
  static mabur::DvrNamer namer;
  return namer.next(dir);
}

// Parses player_config's "WIDTHxHEIGHT@RATE" screen_mode (mode_pick.h);
// the backend gets the rate rounded to whole fps. Falls back to
// BackendCfg's default (1920x1080@60) if malformed.
maburplay::BackendCfg parse_screen_mode(const std::string& s) {
  maburplay::BackendCfg cfg;
  const maburplay::ScreenWant w = maburplay::parse_screen_want(s);
  if (w.ok) {
    cfg.width = w.w;
    cfg.height = w.h;
    cfg.fps = static_cast<int>(std::lround(w.hz));
  }
  return cfg;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc >= 2 && std::string(argv[1]) == "--mux-annexb") {
    if (argc != 4) {
      usage();
      return 2;
    }
    return run_mux_annexb(argv[2], argv[3]);
  }
  if (argc >= 2 && std::string(argv[1]) == "--osd-render") {
    std::string snap, out, font = maburplay::Config().osd.font, scale = "sharp";
    int w = 1920, h = 1080;
    for (int i = 2; i < argc; ++i) {
      const std::string a = argv[i];
      if (a == "--out-osd" && i + 1 < argc) {
        out = argv[++i];
      } else if (a == "--font" && i + 1 < argc) {
        font = argv[++i];
      } else if (a == "--screen" && i + 1 < argc) {
        if (std::sscanf(argv[++i], "%dx%d", &w, &h) != 2 || w <= 0 || h <= 0) {
          usage();
          return 2;
        }
      } else if (a == "--scale" && i + 1 < argc) {
        scale = argv[++i];
      } else if (snap.empty() && !a.empty() && a[0] != '-') {
        snap = a;
      } else {
        usage();
        return 2;
      }
    }
    if (snap.empty() || out.empty()) {
      usage();
      return 2;
    }
    return run_osd_render(snap, out, font, w, h, scale_mode(scale));
  }
  if (argc >= 2 && std::string(argv[1]) == "--gs-render") {
    std::string snap, out;
    std::string font = maburplay::Config().osd.gs.font;
    int w = 1920, h = 1080;
    bool stale = false;
    maburplay::GsPlayerState ps;
    for (int i = 2; i < argc; ++i) {
      const std::string a = argv[i];
      if (a == "--out-gs" && i + 1 < argc) {
        out = argv[++i];
      } else if (a == "--gsfont" && i + 1 < argc) {
        font = argv[++i];
      } else if (a == "--screen" && i + 1 < argc) {
        if (std::sscanf(argv[++i], "%dx%d", &w, &h) != 2 || w <= 0 || h <= 0) {
          usage();
          return 2;
        }
      } else if (a == "--stale") {
        stale = true;
      } else if (a == "--res" && i + 1 < argc) {
        if (std::sscanf(argv[++i], "%dx%d", &ps.vid_w, &ps.vid_h) != 2) {
          usage();
          return 2;
        }
      } else if (a == "--lat" && i + 1 < argc) {
        // p50/p99, the pair the compact bar's lat: field renders.
        if (std::sscanf(argv[++i], "%d/%d", &ps.lat_p50_e2e_ms, &ps.lat_e2e_ms) != 2) {
          usage();
          return 2;
        }
        ps.lat_valid = true;
      } else if (a == "--fps" && i + 1 < argc) {
        ps.fps = std::atof(argv[++i]);
      } else if (a == "--jit" && i + 1 < argc) {
        ps.jitter_ms = std::atof(argv[++i]);
      } else if (a == "--mbps" && i + 1 < argc) {
        ps.mbps = std::atof(argv[++i]);
      } else if (a == "--rec-elapsed" && i + 1 < argc) {
        ps.rec.elapsed_s = std::atoi(argv[++i]);
      } else if (a == "--rec" && i + 1 < argc) {
        // An unrecognised state is rejected rather than silently defaulted:
        // a typo would otherwise produce a dump that still looks like a
        // successful render.
        const std::string r = argv[++i];
        if (r == "recording") {
          ps.rec.kind = maburplay::RecState::Kind::kRecording;
        } else if (r == "armed") {
          ps.rec.kind = maburplay::RecState::Kind::kArmed;
        } else if (r == "fault") {
          ps.rec.kind = maburplay::RecState::Kind::kFault;
        } else {
          usage();
          return 2;
        }
      } else if (snap.empty() && !a.empty() && a[0] != '-') {
        snap = a;
      } else {
        usage();
        return 2;
      }
    }
    if (snap.empty() || out.empty()) {
      usage();
      return 2;
    }
    return run_gs_render(snap, out, font, w, h, stale, ps);
  }

  std::string config_path = "/etc/maburplay.toml";
  bool oneshot = false;
  bool decode_only = false;
  double decode_only_seconds = 30.0;
  bool fps_log = false;
  std::string backend_override;

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "-c" && i + 1 < argc) {
      config_path = argv[++i];
    } else if (a == "--oneshot") {
      oneshot = true;
    } else if (a == "--backend" && i + 1 < argc) {
      backend_override = argv[++i];
    } else if (a == "--decode-only") {
      decode_only = true;
    } else if (a == "--seconds" && i + 1 < argc) {
      decode_only_seconds = std::atof(argv[++i]);
    } else if (a == "--fps-log") {
      fps_log = true;
    } else {
      usage();
      return 2;
    }
  }

  maburplay::Config cfg;
  std::vector<std::string> defaulted;
  try {
    cfg = maburplay::load_config(config_path, &defaulted);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "maburplay: config: %s\n", e.what());
    return 2;
  }
  if (!defaulted.empty()) {
    std::fprintf(stderr, "maburplay: config: %zu key(s) defaulted:\n",
                 defaulted.size());
    for (const std::string& d : defaulted)
      std::fprintf(stderr, "  %s\n", d.c_str());
  }
  if (!backend_override.empty()) cfg.backend = backend_override;

  {
    maburplay::BuildCaps caps;
#ifdef MABUR_PLAYER_MPP
    caps.mpp = true;
#endif
#ifdef MABUR_PLAYER_V4L2REQ
    caps.v4l2req = true;
#endif
    const std::string why = maburplay::unsupported_on_build(cfg, caps);
    if (!why.empty()) {
      std::fprintf(stderr, "maburplay: %s\n", why.c_str());
      return 2;
    }
  }

  // colortrans (docs/colortrans.md). The evaluator is always constructed
  // (cheap); whether it is USED is decided per consumer below: the display
  // stage needs the CRTC's CUBIC_LUT or the video plane's CTM
  // (pick_display_colortrans), the OSD inverse follows the LUT only (an
  // inverted OSD over an un-LUT'd picture is worse than nothing; the CTM
  // never touches the OSD plane), and the burned DVR follows the config
  // alone.
  const maburplay::ColorTrans ct;
  maburplay::LutAxis lut_axis = maburplay::LutAxis::kRedFastest;
  if (const char* ax = std::getenv("MABUR_COLORTRANS_AXIS")) {
    // Bench-only override for the axis probe (docs/colortrans.md).
    if (std::string(ax) == "bgr") lut_axis = maburplay::LutAxis::kBlueFastest;
    std::fprintf(stderr, "maburplay: colortrans: MABUR_COLORTRANS_AXIS=%s\n", ax);
  }
#ifndef MABUR_PLAYER_HW
  // lut_axis is consumed only under MABUR_PLAYER_HW (install_colortrans)
  // below, so a host build would otherwise warn that it is unused.
  (void)lut_axis;
#endif

  // Record button. An accessory: every failure here is non-fatal and said
  // exactly once. A player that refuses to show video because a pin is
  // missing is worse than one that shows video without a button -- same
  // rule the .gfont atlas follows.
  maburplay::RecButton rec_button;
  if (cfg.input.rec.configured) {
    maburplay::RecButtonCfg rbc;
    rbc.pin = cfg.input.rec.pin;
    rbc.active_low = cfg.input.rec.active_low;
    rbc.bias = cfg.input.rec.bias;
    std::string rberr;
    if (rec_button.open(rbc, &rberr)) {
      std::fprintf(stderr, "maburplay: rec button: PIN_%d -> %s line %u, %s, %s\n", rbc.pin,
                   rec_button.chip_path().c_str(), rec_button.offset(),
                   rbc.active_low ? "active_low" : "active_high", rbc.bias.c_str());
    } else {
      std::fprintf(stderr, "maburplay: rec button: %s -- running without it\n", rberr.c_str());
    }
  }

  std::unique_ptr<maburplay::VideoBackend> backend = maburplay::make_backend(cfg.backend);
  if (!backend) {
    std::fprintf(stderr,
                 "maburplay: backend \"%s\" not available (not in this build?)\n",
                 cfg.backend.c_str());
    return 2;
  }

  const maburplay::BackendCfg bcfg = parse_screen_mode(cfg.screen_mode);
  // NOTE: `backend` may be DESTROYED AND RECREATED mid-run by the decode
  // watchdog's escalation path (a kernel-side rkvdec2 force-reset leaves
  // the MPP session unrecoverable by mpi->reset() -- observed live: 52
  // consecutive no-op resets). Every capture below therefore goes through
  // the unique_ptr, never a cached raw pointer.

  // Both overlays: font load ONLY, done here -- BEFORE the presenter is
  // constructed/initialised -- because DrmPresenter::init() needs to know up
  // front whether ANY overlay is wanted. `want_osd` is that decision, and it
  // is what decides whether the presenter claims the primary plane and
  // allocates two full-screen ARGB buffers at all. Deciding this AFTER
  // init() (deciding it too late is what F1 of the review that introduced
  // this ordering fixed) left the presenter always claiming the plane
  // regardless of config -- with everything off, or a missing font, there
  // was no way back to the pre-OSD plane layout (backdrop, no OSD buffers).
  // Every failure here is non-fatal and logged exactly once: a missing font,
  // a port already bound, or (on hardware) no spare ARGB plane all mean "run
  // without that overlay" -- an overlay must never cost video a session.
  // --decode-only stays overlay-free: it has no presenter and is the
  // hardware decode gate, not a display run.
  maburplay::OsdFont osd_font;
  std::unique_ptr<maburplay::OsdSource> osd_src;
  std::unique_ptr<maburplay::OsdRaster> osd_raster;
  bool want_msp_osd = false;
  if (cfg.osd.enable && !decode_only) {
    std::string err;
    if (!osd_font.load(cfg.osd.font, &err)) {
      std::fprintf(stderr, "maburplay: msp osd disabled -- %s\n", err.c_str());
    } else {
      want_msp_osd = true;
    }
  }

  // GS link-status overlay: same up-front font load, same reason.
  maburplay::GsFont gs_font;
  std::unique_ptr<maburplay::GsSource> gs_src;
  // Both overlays share ONE surface pair, so who draws what onto which
  // buffer, in which order, is a single problem and lives in a single unit
  // -- osd_compose.{h,cpp}, which owns every per-buffer shadow and is
  // tested directly (tests/test_osd_coexist.cpp). Everything below is
  // wiring: sources, fonts, and the decision to invoke it.
  maburplay::OsdComposer composer;
  bool want_gs_osd = false;
  if (cfg.osd.gs.enable && !decode_only) {
    std::string err;
    if (!gs_font.load(cfg.osd.gs.font, &err)) {
      std::fprintf(stderr, "maburplay: gs osd disabled -- %s\n", err.c_str());
    } else {
      want_gs_osd = true;
    }
  }

  // EITHER overlay alone must bring the plane up: a GS-only configuration
  // (osd.enable false, osd.gs.enable true) is supported and is the natural
  // one for an aircraft with no MSP-capable FC. With both off, init() skips
  // straight to the pre-OSD topology (video + black backdrop on the
  // primary), which is the escape hatch back to the old plane layout.
  // maybe_unused: its only consumer is DrmPresenter::init(), which exists
  // only in the cross build.
  [[maybe_unused]] const bool want_osd = want_msp_osd || want_gs_osd;

  // Display path: the DrmPresenter becomes the frame owner for the normal
  // (non-decode-only) run -- release_frame() is no longer called inline
  // here, it happens when the presenter is done with a frame (see
  // drm_presenter.h's ownership contract). --decode-only stays
  // presenter-free by design: frames are counted
  // and released immediately.
#ifdef MABUR_PLAYER_HW
  // Tail latency segments + 1 Hz lat line. Declared ahead of the
  // presenter (whose flip sink feeds it) and the regulator (whose release
  // feeds it too) so both are in scope by the time either is wired below.
  maburplay::LatTracker lat;
  // Phase-aware display release (display.regulate_ms; frame_regulator.h).
  // The regulator is a THIRD holder of decoder buffers next to the
  // presenter and the burn recorder; the release thread owns its decisions
  // and returns what it holds whenever it parks (every flush point below).
  // Declared here (ahead of the presenter) so both set_flip_sink closures
  // below -- startup and hotplug reacquire -- can capture it by reference.
  maburplay::FrameRegulator regulator(cfg.display.regulate_ms,
                                      cfg.display.vsync_lock,
                                      cfg.display.vsync_lead_ms,
                                      cfg.display.chain_budget,
                                      cfg.display.sched_cap_ms);
  // The burned DVR's hook on the flip (recorder_tap.h): both flip-sink
  // closures end in tap.on_flip(), on the release thread. Declared ahead of
  // the presenter for the same reason as the regulator, and so it outlives
  // every closure that captures it. Without an installed hook it does
  // nothing; housekeeping installs one after the recorder starts and clears
  // it before the recorder stops.
  maburplay::RecorderTap tap;
  // Phase lock (display.phase_lock; phase_lock.h): steers the drone's
  // sensor rate through maburgs and the RCF, and owns capture-time placement
  // while it holds.
  std::unique_ptr<maburplay::PhaseLock> phase_lock;
  if (cfg.display.phase_lock) {
    phase_lock = std::make_unique<maburplay::PhaseLock>(cfg.display.phase_margin_ms);
    regulator.set_phase_lock_mode(true);
  }
  uint64_t phase_disconts_seen = 0;
  // The drone's measured frame rate (the decode pump's) against the
  // panel's: a pair too far apart for the phase lock pauses it and warns on
  // the OSD fps cell.
  maburplay::RateMatch rate_match;
  uint64_t rate_match_mark_us = 0;
  maburplay::LatLog lat_log;
  // Per-frame rows of the same window as the lat: line (docs/observability.md
  // "frame.log"), written in the same idle-gated 1 Hz burst so a stalled
  // DVR write can never sit on the flip or decode path. Batched: the
  // burst's rows leave in one write at flush(), not one syscall per row.
  maburplay::LatLog frame_log(maburplay::kSessionMarker, "frame.log",
                              maburplay::kFrameLogHeader, /*batched=*/true);
  // The 1 Hz regulator: line, into the session too (docs/observability.md
  // "reg.log"): its pend/pdrop/heals counters are the presenter's side of
  // a missed latch, and /tmp/maburplay.log does not outlive a power-off.
  maburplay::LatLog reg_log(maburplay::kSessionMarker, "reg.log", maburplay::kRegLogHeader);
  // The two queues between the threads. Declared ahead of the presenter so
  // they outlive it: its ReleaseFn pushes into `releases`, up to and
  // including its own destructor.
  //  - handover: decode-output thread -> release thread, every decoded frame
  //    with its decode stamp;
  //  - releases: any thread -> decode-output thread, every frame given back
  //    to the decoder (displaced, off screen, dropped). Only the thread that
  //    owns the decoder's output side drains it: the decode pump, or
  //    housekeeping while the pump is parked.
  maburplay::WakeQueue<maburplay::HandedFrame> handover;
  maburplay::WakeQueue<maburplay::DmaFrame> releases;
  // Installs the display stage: the CUBIC_LUT (always a table on a LUT
  // CRTC, identity when off) or the video-plane CTM, and pre-inverts both
  // overlays only for the LUT. Runs once per presenter acquire, BEFORE
  // splash_show() and before any OSD composition. The OSD half starts from
  // un-inverted on every call, so a presenter rebuilt mid-run whose install
  // ends flat (colortrans off, or no LUT on the new CRTC) does not keep the
  // previous presenter's pre-inversion.
  auto install_colortrans = [&](maburplay::DrmPresenter* p) {
    osd_font.clear_inverse();
    maburplay::set_colour_inverse(nullptr);
    const bool lut_avail = p->color_lut_available();
    const bool ctm_avail = p->plane_ctm_available();
    float m[12] = {};
    const bool affine_ok = maburplay::affine_3x4(ct, m);
    const maburplay::DisplayCtChoice pick =
        maburplay::pick_display_colortrans(cfg.colortrans.enable, lut_avail, ctm_avail, affine_ok);
    bool installed = false;
    if (lut_avail) {
      installed = p->set_color_lut(maburplay::build_cubic_lut(
          pick.how == maburplay::DisplayCt::kLut ? &ct : nullptr, lut_axis));
      if (!installed)
        std::fprintf(stderr,
                     "maburplay: colortrans: CUBIC_LUT install failed -- display stays flat\n");
    } else if (pick.how == maburplay::DisplayCt::kPlaneCtm) {
      uint64_t words[12];
      maburplay::pack_ctm_3x4(m, words);
      installed = p->set_plane_ctm(words);
      if (!installed)
        std::fprintf(stderr,
                     "maburplay: colortrans: video-plane CTM install failed -- display stays "
                     "flat\n");
    }
    if (cfg.colortrans.enable && pick.how == maburplay::DisplayCt::kFlat)
      std::fprintf(stderr,
                   "maburplay: colortrans: %s -- display stays flat, OSD not inverted"
#ifdef MABUR_PLAYER_MPP
                   " (burned DVR still corrected)"
#endif
                   "\n",
                   !ctm_avail ? "CRTC has no 729-entry CUBIC_LUT and the video plane has no CTM"
                              : "gamma != 1 is not affine, the video-plane CTM cannot carry it");
    if (!installed) return;
    if (pick.invert_osd) {
      osd_font.set_inverse(ct);
      maburplay::set_colour_inverse(&ct);
      std::fprintf(stderr, "maburplay: colortrans: display LUT on (axis=%s), OSD pre-inverted\n",
                   lut_axis == maburplay::LutAxis::kRedFastest ? "rgb" : "bgr");
    } else if (pick.how == maburplay::DisplayCt::kPlaneCtm) {
      std::fprintf(stderr, "maburplay: colortrans: video-plane CTM on, OSD untouched\n");
    }
  };
  std::unique_ptr<maburplay::DrmPresenter> presenter;
  if (!decode_only) {
    presenter = std::make_unique<maburplay::DrmPresenter>();
    // Frames the presenter is done with go to the release queue, never
    // straight into the decoder: the presenter runs on the release thread,
    // and only the decode-output side may call release_frame().
    if (!presenter->init(cfg.screen_mode, want_osd,
                         [&releases](const maburplay::DmaFrame& f) { releases.push(f); })) {
      std::fprintf(stderr,
                   "maburplay: DrmPresenter init failed -- no display; frames will be "
                   "decoded and dropped\n");
      presenter.reset();
    } else {
      install_colortrans(presenter.get());
      // Seed the servo's vblank estimator with the committed mode's exact
      // period BEFORE the first flip (splash_show below starts flipping):
      // under the default 60 Hz seed a 90/120 Hz panel's flip deltas round
      // to k=0 and are all dropped, so the servo would never engage.
      const double panel_us = presenter->mode_period_us();
      regulator.set_panel_period(panel_us);
      if (cfg.display.vsync_lock && cfg.display.vsync_lead_ms * 1000.0 >= panel_us)
        std::fprintf(stderr,
                     "maburplay: display.vsync_lead_ms %d >= the "
                     "%.1f ms panel period -- every servo release rolls past one extra "
                     "vblank (a full period of added glass latency); lower "
                     "the lead for this mode\n",
                     cfg.display.vsync_lead_ms, panel_us / 1000.0);
      presenter->set_flip_sink(
          [&lat, &regulator, &tap](const maburplay::DmaFrame& f, uint64_t t, bool ex) {
            lat.on_flip(f.pts_us, t, ex);
            regulator.on_flip(t, ex);
            tap.on_flip(f, t);
          });
    }
  }

  // Startup splash. Painted and shown the moment a presenter exists, so the
  // sink locks a mode NOW rather than at the first decoded frame -- a display
  // powered up in between would otherwise see no signal at all. Defined as a
  // lambda because the hotplug retry below runs the identical sequence on a
  // late acquire.
  auto show_splash = [](maburplay::DrmPresenter* p) {
    const maburplay::Surface s = p->splash_surface();
    if (!s.pixels) return;  // no usable primary plane; DrmPresenter said why
    std::string err;
    if (!maburplay::paint_splash(maburplay::kSplashPath, s, &err))
      std::fprintf(stderr,
                   "maburplay: splash image unavailable (%s) -- showing black; the display still "
                   "comes up now\n",
                   err.c_str());
    p->splash_show();
  };
  if (presenter) show_splash(presenter.get());
  // 1 Hz mark for the lat/regulator observability block in the housekeeping
  // loop (independent of the GS-OSD block).
  uint64_t lat_mark_ms = 0;
#endif

  // MSP DisplayPort OSD: network intake + raster, gated on want_msp_osd (the
  // font already loaded above). presenter->osd_available() is re-checked
  // per frame further down -- this can be true here and the OSD still
  // end up unavailable (no qualifying plane, or the 3-strike runtime
  // disable), which is fine: osd_src/osd_raster just have nothing to feed.
  if (want_msp_osd) {
    std::string err;
    osd_src = std::make_unique<maburplay::OsdSource>();
    if (!osd_src->open(cfg.osd.port, &err)) {
      std::fprintf(stderr, "maburplay: msp osd disabled -- %s\n", err.c_str());
      osd_src.reset();
    } else {
      osd_src->set_stale_ms(cfg.osd.stale_ms);
      osd_raster =
          std::make_unique<maburplay::OsdRaster>(osd_font, scale_mode(cfg.osd.scale));
      composer.set_raster(osd_raster.get());
      std::fprintf(stderr,
                   "maburplay: osd on udp 127.0.0.1:%d font=%s (%dx%d glyphs) scale=%s "
                   "stale_ms=%d\n",
                   osd_src->port(), cfg.osd.font.c_str(), osd_font.native().glyph_w,
                   osd_font.native().glyph_h, cfg.osd.scale.c_str(), cfg.osd.stale_ms);
    }
  }

  // GS link-status overlay: network intake + overlay object, gated on
  // want_gs_osd. Same shape as the MSP intake above, and the same caveat:
  // presenter->osd_available() is re-checked per iteration, so getting here
  // does not guarantee anything is ever drawn.
  if (want_gs_osd) {
    std::string err;
    gs_src = std::make_unique<maburplay::GsSource>();
    if (!gs_src->open(cfg.osd.gs.port, &err)) {
      // gs_src is the live gate from here on -- want_gs_osd has already
      // been folded into want_osd and is never read again.
      std::fprintf(stderr, "maburplay: gs osd disabled -- %s\n", err.c_str());
      gs_src.reset();
    } else {
      gs_src->set_stale_ms(cfg.osd.gs.stale_ms);
      // Three, not one: two DRM buffers plus the burned DVR's index map, all
      // three needing their own record of what they already show. See
      // osd_compose.h for why a shared shadow strobes.
      // dvr.target sizes the REC field (gs_layer.h rec_worst): "gs" keeps
      // the pre-VTX-recorder box. player_config already rejected anything
      // parse_rec_target would.
      maburplay::RecTarget rec_target = maburplay::RecTarget::kGs;
      maburplay::parse_rec_target(cfg.dvr.target, &rec_target);
      composer.set_gs(maburplay::make_gs_layer(gs_font, rec_target),
                      maburplay::make_gs_layer(gs_font, rec_target),
                      maburplay::make_gs_layer(gs_font, rec_target));
      std::fprintf(stderr,
                   "maburplay: gs osd on udp 127.0.0.1:%d font=%s stale_ms=%d\n",
                   gs_src->port(), cfg.osd.gs.font.c_str(), cfg.osd.gs.stale_ms);
    }
  }

  // Player-measured half of the GS overlay's inputs, recomputed once a
  // second in the housekeeping loop. maburgs cannot supply these: its fps
  // counts AUs published to the ring and still reads 60 while a wedged
  // decoder shows a frozen picture -- the exact failure the pilot has to be
  // able to see.
  maburplay::GsPlayerState gs_ps;
  uint64_t gs_frames_at_mark = 0;  // decoded-frame count at the last 1 Hz mark
  uint64_t gs_bytes_at_mark = 0;   // gs_bytes_total at the last 1 Hz mark
  uint64_t gs_bytes_total = 0;     // running AU byte total, summed at delivery
  uint64_t gs_video_mark_ms = 0;   // 0 = no mark yet, so the first tick only seeds
  // COMPLETE AUs, not deliveries: this is what the raw DVR can actually
  // write, and it is what tells a stalled writer apart from a link so bad
  // that every AU arrives truncated (gs_metrics.h, RecTracker::Inputs::feed).
  uint64_t gs_complete_aus = 0;
  // Both split out of this loop so they can be tested on the host with no
  // ring and no DRM -- see tests/test_gs_metrics.cpp.
  maburplay::AuJitter gs_jitter;
  maburplay::RecTracker gs_rec;
  maburplay::VtxRecTracker vtx_trk;
  bool statvfs_warned = false;

  // Whether a recording is running RIGHT NOW. Seeded from dvr.autostart;
  // the record button flips it. Declared here because start_burn_if_needed
  // below reads it.
  bool rec_on = cfg.dvr.autostart;
  // dvr.target: which recorders the button drives.
  const bool rec_gs = cfg.dvr.target != "vtx";
  const bool rec_vtx = cfg.dvr.target != "gs";
  maburplay::CoreControlClient core_ctl;
  if (!core_ctl.open(maburgs::kRecControlPort))
    std::fprintf(stderr,
                 "maburplay: rec: core control socket failed -- the drone recorder will not "
                 "follow the button and refresh reports will not reach maburgs\n");
  // dvr.mode "burned": the recording is produced by re-encoding the frames
  // the display flips to, with the OSD composited in by the hardware
  // encoder, so the BurnRecorder owns the file and the raw remux in the ring
  // sink below is SKIPPED. This flag -- not "the recorder actually started"
  // -- is what gates that skip: a burned run whose encoder refused to come
  // up must record nothing and say so, never quietly leave a raw file
  // behind that the user would mistake for a burned one.
  const bool burned_mode = cfg.dvr.mode == "burned";
  // The regulator line's flushes=/recstarts=: output parks (a ring flush, a
  // decode-watchdog reset or recreate, a display hotplug, shutdown) and
  // recorder starts, cumulative. Both disturb presentation for a few
  // frames, so a gate that counts missed latches (tools/bench/latchgate.py)
  // keys its exclusions on them. Housekeeping thread only.
  [[maybe_unused]] uint64_t output_parks = 0;
  [[maybe_unused]] uint64_t rec_starts = 0;
#ifdef MABUR_PLAYER_HW
  // Declared AFTER `presenter` and `backend` so it is destroyed FIRST on
  // unwind: stop() joins the encode thread and releases the decoder buffers
  // it still holds while the decoder that owns them is still alive.
  std::unique_ptr<maburplay::BurnRecorder> burn;
  // The recorder is started with the record size `dvr.burned.size` names,
  // and "" means the decoded picture's size -- unknown until the first frame
  // decodes. A start that needs it before then is DEFERRED, not refused:
  // this is set, nothing is logged as a failure, and housekeeping retries
  // start_burn_if_needed() on the first iteration that sees a frame.
  bool burn_deferred = false;
  // The decode-output thread and the release thread (decode_pump.h,
  // release_loop.h). Declared AFTER `burn`, `presenter` and `backend` so
  // they are destroyed FIRST on unwind (each destructor stops and joins):
  // neither may outlive the recorder the release thread's flip tap
  // submits to, the presenter the release thread presents to and the pump
  // primes, or the decoder the pump polls. Both exist in every hardware run
  // but start only in the display run below: --decode-only keeps its
  // one-thread loop and inline sink, and --oneshot never decodes.
  maburplay::DecodePump pump(lat, regulator, handover, releases);
  maburplay::ReleaseLoop release(regulator, lat, handover, releases, presenter.get(),
                                 maburplay::ReleaseLoop::Cfg{cfg.display.rt_priority});
  // burn_in/burn_enc/burn_drop on the regulator line: the burned recorder's
  // cumulative frames_in/frames_encoded/frames_dropped (burn_recorder.h),
  // zero while `burn` holds no recorder -- no burned recording running, or
  // a build whose BurnRecorder is the v4l2req stub that never starts one.
  struct BurnCounts {
    uint64_t in = 0, enc = 0, drop = 0;
  };
  auto burn_counts = [&burn]() -> BurnCounts {
#ifdef MABUR_PLAYER_HW
    if (burn) return {burn->frames_in(), burn->frames_encoded(), burn->frames_dropped()};
#endif
    return {};
  };
  // Regulator stderr line: printed both at 1 Hz from the housekeeping loop's
  // stats block (below) and once more at exit as the final tally -- same
  // format either way, factored here so the two call sites can't drift.
  // pend= is the DrmPresenter's mailbox engagement count (frame parked, or
  // displacing one already parked because a flip was in flight) -- 0 when
  // there is no presenter (decode-only, or init failed). Every call closes
  // the release thread's stats window (rlate=, hand=, present_jitter=).
  auto log_regulator_line = [&regulator, &presenter, &release, &reg_log, &phase_lock,
                             &output_parks, &rec_starts, &burn_counts]() {
    char line[768];
    const maburplay::ReleaseLoop::Stats st = release.take_stats();
    if (regulator.enabled()) {
      const uint64_t pend = presenter ? presenter->mailbox_engagements() : 0;
      std::snprintf(line, sizeof(line),
                   "regulator: held=%llu late=%llu replaced=%llu disconts=%llu "
                   "hold_ema=%.2fms present_jitter=%.2fms vsync=%s skips=%llu "
                   "fallback=%llu pend=%llu heals=%llu pdrop=%llu "
                   "chained=%llu chain=%llu chain_max=%llu chains=%llu cuts=%llu "
                   "alw=%.1fms lslot=%llu",
                   static_cast<unsigned long long>(regulator.held_count()),
                   static_cast<unsigned long long>(regulator.late_count()),
                   static_cast<unsigned long long>(regulator.replaced_count()),
                   static_cast<unsigned long long>(regulator.discont_count()),
                   regulator.hold_ema_ms(), st.present_jitter_ema_ms,
                   regulator.servo_locked() ? "locked" : "fallback",
                   static_cast<unsigned long long>(regulator.vsync_skips()),
                   static_cast<unsigned long long>(regulator.fallback_frames()),
                   static_cast<unsigned long long>(pend),
                   static_cast<unsigned long long>(regulator.heals()),
                   static_cast<unsigned long long>(
                       presenter ? presenter->mailbox_dropped_paced() : 0),
                   static_cast<unsigned long long>(regulator.chained_count()),
                   static_cast<unsigned long long>(regulator.chain_run()),
                   static_cast<unsigned long long>(regulator.chain_max()),
                   static_cast<unsigned long long>(regulator.chains_count()),
                   static_cast<unsigned long long>(regulator.chain_cuts()),
                   regulator.allowance_ms(),
                   static_cast<unsigned long long>(regulator.late_slot_count()));
      {
        const size_t used = std::strlen(line);
        if (phase_lock) {
          std::snprintf(line + used, sizeof(line) - used,
                        " lock=%s lerr=%.2fms loff=%d lctr=%.1f",
                        maburplay::phase_lock_state_name(phase_lock->state()),
                        phase_lock->error_ms(), static_cast<int>(phase_lock->offset_mfps()),
                        phase_lock->centre_mfps());
        } else {
          std::snprintf(line + used, sizeof(line) - used,
                        " lock=off lerr=0.00ms loff=0 lctr=0.0");
        }
      }
      if (presenter) {
        // present() stage cost over the last window (µs p50/max), on the
        // release thread: framebuffer acquire (cached dma-buf import +
        // AddFB2), request build, atomic commit ioctl. The lead
        // (display.vsync_lead_ms) has to cover the release thread's own
        // wake lateness (rlate= below) plus their sum; a frame past it
        // misses its latch and the next one is pdropped.
        const auto pt = presenter->present_timing();
        const size_t used = std::strlen(line);
        std::snprintf(line + used, sizeof(line) - used,
                      " present=%u acq=%u/%u req=%u/%u ioctl=%u/%u",
                      pt.n, pt.p50[0], pt.max[0], pt.p50[1], pt.max[1],
                      pt.p50[2], pt.max[2]);
      }
    } else {
      std::snprintf(line, sizeof(line), "regulator: off present_jitter=%.2fms",
                    st.present_jitter_ema_ms);
    }
    {
      // The release thread over the same window (µs p99/max): rlate = a held
      // frame's release behind its schedule, hand = decode stamp to offer;
      // rt = 1 when it runs SCHED_FIFO. Then the cumulative output parks
      // and recorder starts (see output_parks above), and the burned
      // recorder's own cumulative counters (burn_counts above).
      const size_t used = std::strlen(line);
      const BurnCounts bc = burn_counts();
      std::snprintf(line + used, sizeof(line) - used,
                    " rlate=%u/%u hand=%u/%u rt=%d flushes=%llu recstarts=%llu "
                    "burn_in=%llu burn_enc=%llu burn_drop=%llu",
                    st.rlate_p99_us, st.rlate_max_us, st.hand_p99_us, st.hand_max_us,
                    release.rt_applied() ? 1 : 0, static_cast<unsigned long long>(output_parks),
                    static_cast<unsigned long long>(rec_starts),
                    static_cast<unsigned long long>(bc.in),
                    static_cast<unsigned long long>(bc.enc),
                    static_cast<unsigned long long>(bc.drop));
    }
    std::fprintf(stderr, "%s\n", line);
    reg_log.write(mono_us(), line);
  };
  // The encoder-side OSD palette, built once per input set and memoised.
  // build_palette() histograms the whole MSP atlas (~2 M px through a hash
  // map) and runs a 255-step median cut: ~150 ms on the A55, all of it on
  // the housekeeping loop (OSD, stats) at a record start. Its inputs -- the
  // atlas, the GS seeds, the forward map -- are fixed for a run, so the first build is
  // done at startup below, before video flows; a later key change (a
  // presenter acquired after startup flipping colour_inverse()) rebuilds
  // once, logged.
  struct BurnPaletteMemo {
    bool valid = false;
    bool raster = false, gs = false;
    const maburplay::ColorTrans* fwd = nullptr;
    maburplay::OsdPalette pal;
  } burn_palette_memo;
  auto burn_palette = [&]() -> const maburplay::OsdPalette& {
    const bool raster = osd_raster != nullptr;
    const bool gs = composer.gs_present();
    const maburplay::ColorTrans* fwd = maburplay::colour_inverse();
    BurnPaletteMemo& m = burn_palette_memo;
    if (!m.valid || m.raster != raster || m.gs != gs || m.fwd != fwd) {
      size_t n_seeds = 0;
      // Seeds are needed even when the MSP atlas IS loaded: median cut over
      // the Betaflight atlas alone reproduces the atlas's own hues, and the
      // GS status colours are not among them -- they would be recorded as
      // whatever foreign colour sits nearest. With no MSP font at all the
      // empty GlyphAtlas yields a palette built from the seeds alone.
      const uint32_t* seeds = gs ? maburplay::gs_palette_seeds(&n_seeds) : nullptr;
      const maburplay::GlyphAtlas empty{};
      // Forward map follows the surface's actual inversion state (the
      // process-wide colour_inverse() pointer set_colour_inverse installed
      // for the OSD plane), not whether the display is corrected: on the
      // video-plane CTM path the picture is corrected while the OSD is not
      // inverted, and the palette's invariant is with the inversion.
      m.pal = maburplay::build_palette(raster ? osd_font.native() : empty, seeds, n_seeds, fwd);
      m.valid = true;
      m.raster = raster;
      m.gs = gs;
      m.fwd = fwd;
      std::fprintf(stderr, "maburplay: burn palette built: %d entries\n", m.pal.n);
    }
    return m.pal;
  };

  // Tears a running burned recorder down, in the order its hooks need: the
  // flip tap first -- once clear() returns no submit() is in flight on the
  // release thread and none can start, which stop() relies on -- then the
  // composer's sink, both of which hold a RAW pointer to the recorder; then
  // the recorder itself, and only then the composer's burn size, which
  // returns the burn canvas to following the screen. Housekeeping only.
  // Returns the recording's encoded-frame count.
  auto stop_burn = [&]() -> uint64_t {
    burn_deferred = false;
    if (!burn) return 0;
    tap.clear();
    composer.set_burn_sink({});
    const uint64_t n = burn->frames_encoded();
    burn->stop();
    burn.reset();
    std::string err;
    if (!composer.set_burn_size(0, 0, &err)) {
      // Re-laying the GS burn layer back at the screen size failed, so the
      // composer dropped all three GS layers: the screen's GS overlay is off.
      std::fprintf(stderr, "maburplay: gs osd disabled -- %s\n", err.c_str());
      gs_src.reset();  // the live gate; the overlays are already dropped
    }
    return n;
  };

  // Idempotent, and deliberately callable more than once. The recording
  // samples the display's vblank grid, so there is no recorder without a
  // presenter: a display acquired late starts it from the hotplug retry in
  // the run loop. With dvr.burned.size "" the record size is the decoded
  // picture's, so there is none before the first decoded frame either: the
  // start is deferred (burn_deferred) and housekeeping retries it on the
  // first iteration that sees a frame. Leaving it unbuilt on either path
  // would mean a lit screen, playing video, and nothing recorded.
  auto start_burn_if_needed = [&]() {
    if (burn || !burned_mode || !rec_on || !rec_gs || !presenter) return;
    // The record size: dvr.burned.size, or (0x0) the decoded picture's
    // size now. A later change of the decoded size (a rung change) does not
    // restart the recorder: its stage scales every frame of another size to
    // this one (burn_recorder.h BurnCfg).
    int rw = cfg.dvr.burned.size_w;
    int rh = cfg.dvr.burned.size_h;
    if (rw <= 0 || rh <= 0) {
      rw = pump.video_w();
      rh = pump.video_h();
      if (rw <= 0 || rh <= 0) {
        if (!burn_deferred)
          std::fprintf(stderr,
                       "maburplay: rec: START deferred -- waiting for the first decoded "
                       "frame\n");
        burn_deferred = true;
        return;
      }
    }
    burn_deferred = false;
    auto rec = std::make_unique<maburplay::BurnRecorder>();
    maburplay::BurnCfg bc;
    bc.width = rw;
    bc.height = rh;
    // Palette (and with it the encoder's OSD region) whenever EITHER overlay
    // is live. Gating on osd_raster alone would leave a GS-only topology
    // recording pixels the encoder has no entries for, i.e. a burned file
    // with no overlay at all, indistinguishable from a deliberate plain
    // transcode. The burned overlay is composed at the record size
    // (set_burn_size below), not at the panel's OSD surface, so the region
    // is the record size and does not depend on the panel having an OSD
    // plane. With neither overlay there is no palette and set_osd() costs
    // nothing, which IS the plain transcode.
    // MUST precede start(), which uploads it before the thread exists.
    if (osd_raster || composer.gs_present()) {
      bc.osd_width = rw;
      bc.osd_height = rh;
      rec->set_palette(burn_palette());  // memoised; see burn_palette above
    }
    // The vblank grid the tap samples: the committed mode's exact period,
    // thinned to one flip in `every`. A presenter is only ever acquired
    // while none exists, and the recorder needs one, so this period holds
    // for the recorder's whole life.
    bc.period_us = presenter->mode_period_us();
    bc.every = cfg.dvr.burned.every;
    bc.bitrate_kbps = cfg.dvr.burned.bitrate_kbps;
    bc.fragment_ms = cfg.dvr.fragment_ms;
    bc.colortrans = cfg.colortrans.enable ? &ct : nullptr;
    // backend.get() is CHECKED, not retained (see burn_recorder.h): the
    // decode watchdog may destroy and recreate it mid-run.
    if (rec->start(bc, dvr_filename(cfg.dvr.dir), backend.get())) burn = std::move(rec);
    if (!burn) return;
    ++rec_starts;
    // The burn canvas and the GS burn layer at the record size: the OSD
    // surface set_osd() is handed must be exactly osd_width x osd_height.
    // A GS bar that does not fit the record size refuses the recording:
    // the composer puts the GS burn layer back at the screen size, so the
    // screen's GS overlay carries on.
    {
      std::string err;
      if (!composer.set_burn_size(rw, rh, &err)) {
        std::fprintf(stderr,
                     "maburplay: rec: recording refused -- the GS bar does not fit the record "
                     "size %dx%d (%s); the burned recorder is stopped, NOTHING is being "
                     "recorded; the screen's GS overlay is unaffected\n",
                     rw, rh, err.c_str());
        // Only if putting the burn layer back at the screen size failed
        // too, which drops all three layers (the all-or-none rule).
        if (!composer.gs_present()) gs_src.reset();
        stop_burn();
        return;
      }
    }
    // Raw pointer, not a reference to the unique_ptr. `burn` IS reset --
    // rec_stop() tears the recorder down on a button press -- so the
    // safety comes from ordering instead: stop_burn() removes both hooks
    // BEFORE it stops and destroys the recorder. The composer runs on this
    // (housekeeping) thread; the tap's hook runs on the release thread, and
    // RecorderTap's lock is the fence against it: the hook goes in only
    // after start() returned, and once tap.clear() returns no submit() is
    // in flight (burn_recorder.h THREADING).
    maburplay::BurnRecorder* b = burn.get();
    composer.set_burn_sink([b](const maburplay::Surface& s, const maburplay::DirtyRect* r,
                               size_t n) { b->set_osd(s, r, n); });
    tap.set([b](const maburplay::DmaFrame& f, uint64_t pts) { b->submit(f, pts); },
            bc.period_us, bc.every);
    std::fprintf(stderr, "maburplay: rec: burned %dx%d every=%d\n", rw, rh, bc.every);
  };
  // Warm the palette memo now, while nothing is on screen yet, so the
  // first record start does not pay for it on the housekeeping loop.
  if (burned_mode && presenter && (osd_raster || composer.gs_present())) burn_palette();
  start_burn_if_needed();
  // rec_gs: with dvr.target "vtx" the GS recorder is never started, so
  // its absence is not news. Nor is a deferred start's (it waits for the
  // first decoded frame and said so).
  if (rec_on && rec_gs && burned_mode && !burn && !burn_deferred) {
    if (presenter) {
      // A real recorder failure: BurnRecorder::start() already logged why.
      std::fprintf(stderr,
                   "maburplay: dvr.mode \"burned\" requested but the recorder is not "
                   "running -- NOTHING is being recorded (no raw fallback: it would look "
                   "like a burned file)\n");
    } else {
      std::fprintf(stderr,
                   "maburplay: dvr.mode \"burned\" requested but there is no display yet -- "
                   "NOTHING is being recorded; recording starts if a display is acquired\n");
    }
  }
#else
  if (rec_on && rec_gs && burned_mode) {
    std::fprintf(stderr,
                 "maburplay: dvr.mode \"burned\" needs the hardware build (mpp encoder); "
                 "NOTHING is being recorded\n");
  }
#endif

  // The inline frame sink: what the backend calls from inside poll() when
  // no decode pump runs -- the host build, and --decode-only on hardware
  // (the hardware decode gate keeps its one-thread loop). It counts the
  // frame and gives it straight back. The display run on hardware hands
  // the decoder pump.sink() instead, and these stay untouched there.
  uint64_t frame_count = 0;
  bool have_first_frame = false;
  bool have_decoded_pts = false;  // shutdown waits for the drained picture's frame
  uint32_t last_decoded_pts = 0;
  std::chrono::steady_clock::time_point t_first_frame, t_last_frame;
  const maburplay::VideoBackend::FrameSink frame_sink = [&](const maburplay::DmaFrame& f) {
    ++frame_count;
    have_decoded_pts = true;
    last_decoded_pts = f.pts_us;
    // The DECODED picture size, which is what the OSD's res: field means --
    // not screen_mode (the panel) and not bcfg (the backend's configured
    // size), both of which stay at 1080p while the drone streams 720p.
    gs_ps.vid_w = f.width;
    gs_ps.vid_h = f.height;
    const auto now = std::chrono::steady_clock::now();
    if (!have_first_frame) {
      t_first_frame = now;
      have_first_frame = true;
    }
    t_last_frame = now;
    backend->release_frame(f);
  };
  // The sink the backend is initialised with -- here and in the watchdog's
  // decoder re-creation, which must wire the fresh instance the same way.
#ifdef MABUR_PLAYER_HW
  const maburplay::VideoBackend::FrameSink decoder_sink = decode_only ? frame_sink : pump.sink();
#else
  const maburplay::VideoBackend::FrameSink& decoder_sink = frame_sink;
#endif
#ifdef MABUR_PLAYER_MPP
  // [decoder] stream: MppBackend sets its parser up for streaming at init.
  if (auto* mpp = dynamic_cast<maburplay::MppBackend*>(backend.get()))
    mpp->set_stream(cfg.decoder.stream);
#endif
  const bool init_ok = backend->init(bcfg, decoder_sink);
  if (!init_ok) {
    std::fprintf(stderr, "maburplay: backend \"%s\" init failed\n", cfg.backend.c_str());
    return 2;
  }
#ifdef MABUR_PLAYER_HW
  // After init(): MppBackend takes the pump's blocking poll timeout only
  // once initialised. Not in --decode-only, whose loop polls the backend
  // itself and must not block in it.
  if (!decode_only) pump.set_backend(backend.get());
#endif
  // Streamed decode: split AUs go to the
  // decoder slice by slice as ring v4 publishes them. The VideoBackend seam
  // stays frozen; a backend that can stream also implements StreamDecoder
  // (MppBackend, V4l2ReqBackend), reached by dynamic_cast like its other
  // extensions.
  maburplay::StreamFeeder feeder(cfg.decoder.stream);
  feeder.set_decoder(dynamic_cast<maburplay::StreamDecoder*>(backend.get()));
  std::fprintf(stderr, "maburplay: decoder stream: %s\n",
               maburplay::stream_off_name(feeder.off_reason()));

  // Raw-mode recorder (common/raw_dvr.h): sync gate + params + mux. Its
  // Error state is what the GS overlay's REC "broken" reads.
  mabur::RawDvr dvr;
  // A coded-size change mid-recording (the drone picked a new camera mode)
  // seals the current file and rolls to the next namer-assigned one rather
  // than erroring the recording out.
  dvr.set_next_sink([&](std::string* name) {
    *name = dvr_filename(cfg.dvr.dir);
    std::fprintf(stderr, "maburplay: rec: ROLL (%s)\n", name->c_str());
    return mabur::open_file_sink(*name);
  });
  // Autostart: dvr.autostart armed rec_on before this point (see rec_on's
  // declaration above), but the actual start() call has to wait until here,
  // where bcfg is in scope.
  if (rec_on && rec_gs && !burned_mode) {
    dvr.start(dvr_filename(cfg.dvr.dir), bcfg.width, bcfg.height, cfg.dvr.fragment_ms);
    ++rec_starts;
    core_ctl.send_line("refresh dvr");
  }

  // The button's two actions. Both modes are handled here so the press
  // handler stays one line; the raw path is a flag plus a close, the
  // burned path tears the recorder down and builds a fresh one.
  auto rec_start = [&]() {
    if (rec_on) return;
    rec_on = true;
    gs_rec.reset();  // the OSD clock counts THIS file
    vtx_trk.reset();
    if (!rec_gs) {
      std::fprintf(stderr, "maburplay: rec: START (vtx only)\n");
      return;
    }
#ifdef MABUR_PLAYER_HW
    if (burned_mode) {
      start_burn_if_needed();
      if (!burn) {
        // Deferred: no frame has decoded yet to size the recording from.
        // start_burn_if_needed() said so, and housekeeping starts it on the
        // first decoded frame.
        if (burn_deferred) return;
        // Same discrimination as the startup path above: with a presenter
        // already in hand, start_burn_if_needed() ran for real and the
        // recorder REFUSED -- and nothing will retry it (the hotplug retry
        // only fires while `presenter` is null), so "deferred" would be a
        // lie in the one log the post-mortem reads.
        if (presenter) {
          std::fprintf(stderr,
                       "maburplay: rec: START refused -- the burned recorder did not start "
                       "(see BurnRecorder above); NOTHING is being recorded\n");
        } else {
          std::fprintf(stderr,
                       "maburplay: rec: START deferred -- no display yet; recording begins "
                       "when one is acquired\n");
        }
        return;
      }
    }
#endif
    if (!burned_mode) {
      dvr.start(dvr_filename(cfg.dvr.dir), bcfg.width, bcfg.height, cfg.dvr.fragment_ms);
      ++rec_starts;
      core_ctl.send_line("refresh dvr");
    }
    std::fprintf(stderr, "maburplay: rec: START (%s)\n", cfg.dvr.mode.c_str());
  };

  auto rec_stop = [&]() {
    if (!rec_on) return;
    rec_on = false;
    if (!rec_gs) {
      std::fprintf(stderr, "maburplay: rec: STOP (vtx only)\n");
      return;
    }
    if (!burned_mode) {
      // Read BEFORE the stop: samples() is 0 unless this recording opened a
      // file -- the raw path only opens on the next parameter-set AU (~2 s),
      // so a quick START->STOP lands before any file exists.
      const unsigned long long n = dvr.samples();
      dvr.stop();
      std::fprintf(stderr, "maburplay: rec: STOP (%llu samples)\n", n);
      return;
    }
#ifdef MABUR_PLAYER_HW
    if (burn) {
      const uint64_t n = stop_burn();
      std::fprintf(stderr, "maburplay: rec: STOP (%llu frames)\n",
                   static_cast<unsigned long long>(n));
      return;
    }
    burn_deferred = false;  // a deferred start never ran; it must not run now
#endif
    // There was no recorder to tear down: START was deferred (no display
    // or no decoded frame yet) or refused (the encoder would not come up),
    // or this is a build with no encoder at all. Say so anyway -- a press that leaves NO line
    // in /tmp/maburplay.log is the worst case for a post-flight read.
    std::fprintf(stderr, "maburplay: rec: STOP (no recorder was running)\n");
  };

  // Decoder input.
  // AuRouter is the ring sink: the sid-0 join gate (this encoder's GDR+SVC-T
  // config emits ONE real IRAP per session -- a mid-session join treats the
  // ~2 s parameter-set sid0 refresh as its cut point), the decodable gate,
  // whole and streamed submits. What it hands back is processed here.
  maburplay::AuRouter router(feeder, backend);
  // Delivery notes only. Submit notes (lat submit stamps, incomplete-AU
  // flags) belong to the decode pump, which takes them from the feed ahead
  // of each decoded frame; one reaching this hook comes from a run with no
  // pump (--decode-only, --oneshot) and has nobody to stamp it for.
  auto process_note = [&](maburplay::FeedNote&& n) {
    if (n.kind == maburplay::FeedNote::Kind::kSubmit) return;
    // GS overlay video figures, measured at AU delivery (t_us is stamped where
    // the ring was read) -- not at flip: the presenter is a mailbox on the
    // panel vsync, so flip deltas are quantized to vsync-period multiples.
    gs_jitter.on_au(n.t_us / 1000);
    // Delivered bitrate is what arrived, truncated AUs included.
    gs_bytes_total += n.bytes;
    if (n.complete) ++gs_complete_aus;
    // !burned_mode: in burned mode the BurnRecorder owns the recording. RawDvr
    // applies the raw path's rules itself (complete AUs only, file begins at a
    // parameter-set AU) and ignores AUs while not armed.
    if (rec_on && !burned_mode && rec_gs && !n.au.empty())
      dvr.feed(n.au.data(), n.au.size(), n.meta.pts_us, n.decodable);
  };
  maburplay::FeedLoop* feed_ptr = nullptr;
#ifdef MABUR_PLAYER_HW
  // Gives every frame in the release queue back to the decoder. Only while
  // the decode pump is parked: the queue's consumer is the decoder's output
  // side, which housekeeping owns only then.
  std::vector<maburplay::DmaFrame> rel_buf;
  auto return_releases = [&]() {
    rel_buf.clear();
    releases.drain(&rel_buf);
    for (const maburplay::DmaFrame& f : rel_buf) backend->release_frame(f);
  };
  // Parks the decode-output thread, then the release thread, and returns
  // every frame they let go of to the decoder: the release thread's park
  // drops what the presenter shows or has queued, every frame the regulator
  // holds and every frame still in the hand-over queue. Until
  // resume_output() housekeeping owns the decoder's output side and the
  // display -- nothing is polled, presented or primed. The pump goes first
  // so nothing is handed over behind the release thread's park. A thread
  // that does not park in 2 s is wedged beyond this process: respawn, as
  // for a feed thread that will not park.
  auto park_output = [&](const char* why) {
    ++output_parks;
    if (!pump.park(std::chrono::seconds(2)) || !release.park(std::chrono::seconds(2))) {
      std::fprintf(stderr,
                   "maburplay: %s -- the decode-output or release thread did not park in 2 s; "
                   "exiting for respawn\n",
                   why);
      std::fflush(stderr);
      std::_Exit(1);
    }
    return_releases();
  };
  auto resume_output = [&]() {
    release.resume();
    pump.resume();
  };
#endif
  // Flush-ordering contract: MppBackend::flush() / mpi->reset() with
  // DmaFrames still held is an unverified interaction, so every holder --
  // presenter and regulator (both emptied by park_output), recorder (the
  // SECOND holder; resealed with an IDR) -- lets go BEFORE flush() runs.
  // Runs with the feed and the output side parked.
  auto drop_holders = [&]() {
#ifdef MABUR_PLAYER_HW
    // Submit notes still queued belong to AUs the reset discards: none of
    // them will decode, and lat.flush_all() below forgets the session.
    if (feed_ptr) {
      std::vector<maburplay::FeedNote> stale;
      feed_ptr->take_submit_notes(&stale);
    }
    // The release thread is parked (park_output), so its flip tap cannot
    // submit() into this (burn_recorder.h).
    if (burn) {
      burn->drop_pending();
      burn->request_idr();
    }
    lat.flush_all();  // the old session's pts space is dead
#endif
  };
  auto do_flush = [&]() {
#ifdef MABUR_PLAYER_HW
    park_output("flush");
#endif
    drop_holders();
    feeder.on_flush();  // never reset under an open streamed picture
    backend->flush();
    router.disarm();    // resync at the next sid0
#ifdef MABUR_PLAYER_HW
    resume_output();
#endif
  };
  router.set_hooks({process_note, [&]() { do_flush(); return true; }, [] { return mono_us(); }});
  auto sink = [&](maburplay::AuEvent&& ev) {
    if (feed_ptr && !feed_ptr->checkpoint()) return;  // parked for the main thread, or stopping
    router.on_event(std::move(ev));
  };

  maburplay::RingClient ring({cfg.ring_path, cfg.socket}, sink);
  // Wait indefinitely for maburgs to create the ring (GS boot race):
  // the init scripts start us milliseconds after maburgs, so
  // losing the race is normal, not a config error -- and exiting made it
  // permanent, because S97maburplay reads our exit 2 as "config error, skip
  // restart". --oneshot is the exception: there the ring is the test fixture,
  // so its absence is a real failure and must not hang the host suite.
  if (!ring.open(oneshot ? 0 : -1)) {
    std::fprintf(stderr, "maburplay: cannot open ring %s\n", cfg.ring_path.c_str());
    return 2;
  }
  // A fresh decoder has no key frame to arm on and nothing periodic will
  // bring one: ask through maburgs. The core retries until an IRAP lands.
  if (!oneshot) core_ctl.send_line("refresh start");

  if (oneshot) {
    ring.oneshot_drain();
    dvr.stop();
    std::printf(
        "{\"delivered\":%llu,\"dropped_enhance_incomplete\":%llu,"
        "\"truncated_base\":%llu,\"salvaged_base\":%llu,\"salvaged_enhance\":%llu,"
        "\"resyncs\":%llu,\"dvr_samples\":%llu,"
        "\"dvr_fragments\":%llu,\"backend_submits\":%llu,"
        "\"streamed\":%llu,\"stream_aborted\":%llu,\"stream\":\"%s\"}\n",
        static_cast<unsigned long long>(ring.delivered()),
        static_cast<unsigned long long>(ring.dropped_enhance_incomplete()),
        static_cast<unsigned long long>(ring.truncated_base()),
        static_cast<unsigned long long>(ring.salvaged_base()),
        static_cast<unsigned long long>(ring.salvaged_enhance()),
        static_cast<unsigned long long>(ring.resyncs()), static_cast<unsigned long long>(dvr.samples()),
        static_cast<unsigned long long>(dvr.fragments()),
        static_cast<unsigned long long>(router.submits()),
        static_cast<unsigned long long>(feeder.streamed()),
        static_cast<unsigned long long>(feeder.stream_aborted()),
        maburplay::stream_off_name(feeder.off_reason()));
    return 0;
  }

  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);
  std::signal(SIGUSR1, on_rec_toggle);

  if (decode_only) {
    // Hardware decode gate (amended per the sid0-join finding above):
    // drive the backend straight off the live ring for
    // `decode_only_seconds`, no presenter -- the frame_sink above (wired
    // at backend->init() time, shared with the normal run loop) counts
    // frames and timestamps the first/last decoded frame. info_changes
    // /errors are MppBackend-internal decode-loop counters with no
    // VideoBackend hook (that interface is frozen), so they're read via a
    // dynamic_cast that only fires under MABUR_PLAYER_HW; a host build (or
    // --backend null) just reports 0, correct since NullBackend never
    // decodes anything.
    const auto t_start = std::chrono::steady_clock::now();
    const auto deadline =
        t_start + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                      std::chrono::duration<double>(decode_only_seconds));
    // Sampled once, the first time the loop observes t_sync + 3s having
    // passed: MppBackend's cumulative error count at that instant, so the
    // final report can separate "errors during the first 3s post-sync"
    // (the sid0-refresh-picture concealment window; some are expected)
    // from "errors afterward" (should be zero -- see the amended gate).
    bool sampled_3s = false;
    uint64_t errors_at_3s = 0;
    bool ring_died = false;
    while (!g_stop.load() && std::chrono::steady_clock::now() < deadline) {
      ring.pump(100);
      backend->poll();
      if (!sampled_3s && router.synced() &&
          mono_us() >= router.t_sync_us() + 3'000'000) {
        sampled_3s = true;
#ifdef MABUR_PLAYER_MPP
        if (auto* mpp = dynamic_cast<maburplay::MppBackend*>(backend.get())) {
          errors_at_3s = mpp->errors();
        }
#endif
#ifdef MABUR_PLAYER_V4L2REQ
        if (auto* v4 = dynamic_cast<maburplay::V4l2ReqBackend*>(backend.get())) {
          errors_at_3s = v4->errors();
        }
#endif
      }
      if (ring.dead()) {
        std::fprintf(stderr, "maburplay: ring reader dead, exiting\n");
        ring_died = true;
        break;
      }
    }

    const double elapsed_s =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();
    const double fps = elapsed_s > 0.0 ? static_cast<double>(frame_count) / elapsed_s : 0.0;
    // fps_active: rate across the frames actually decoded (first to last),
    // excluding the cold-attach/sid0-join wait baked into whole-window fps
    // above -- (frame_count - 1) intervals span (t_last - t_first).
    double fps_active = 0.0;
    if (have_first_frame && frame_count >= 2) {
      const double active_s =
          std::chrono::duration<double>(t_last_frame - t_first_frame).count();
      if (active_s > 0.0) fps_active = static_cast<double>(frame_count - 1) / active_s;
    }

    uint64_t info_change_count = 0;
    uint64_t error_count = 0;
    uint64_t concealed_count = 0;
#ifdef MABUR_PLAYER_MPP
    if (auto* mpp = dynamic_cast<maburplay::MppBackend*>(backend.get())) {
      info_change_count = mpp->info_changes();
      // Counter semantics under the shipped decoder config (review
      // finding): DISABLE_ERROR suppresses MPP's errinfo marking, so
      // errors() now means HARD failures only (no buffer / bad fd /
      // put_packet), and concealed() counts errinfo frames that were
      // emitted for display (rare under DISABLE_ERROR by construction).
      // The gate criterion "errors_after_sync_3s == 0" therefore asserts
      // pipeline integrity, not bitstream cleanliness.
      error_count = mpp->errors();
      concealed_count = mpp->concealed();
    }
#endif
#ifdef MABUR_PLAYER_V4L2REQ
    if (auto* v4 = dynamic_cast<maburplay::V4l2ReqBackend*>(backend.get())) {
      error_count = v4->errors();
      std::fprintf(stderr,
                   "maburplay: v4l2req frames=%llu bitstream=%llu shape_drops=%llu gated=%llu\n",
                   static_cast<unsigned long long>(v4->frames()),
                   static_cast<unsigned long long>(v4->bitstream()),
                   static_cast<unsigned long long>(v4->dropped_shape()),
                   static_cast<unsigned long long>(v4->gated()));
    }
#endif
    // If the 3s-post-sync mark was never reached (run ended too soon, or
    // sync never happened), errors_at_3s stays 0 and errors_after_sync_3s
    // degrades to the full total -- an honest "can't claim post-window
    // cleanliness" rather than a false pass.
    const uint64_t errors_after_sync_3s = error_count - errors_at_3s;

    dvr.stop();
    std::printf(
        "{\"frames\":%llu,\"fps\":%.2f,\"fps_active\":%.2f,\"info_changes\":%llu,"
        "\"errors\":%llu,\"errors_after_sync_3s\":%llu,\"concealed\":%llu}\n",
        static_cast<unsigned long long>(frame_count), fps, fps_active,
        static_cast<unsigned long long>(info_change_count),
        static_cast<unsigned long long>(error_count),
        static_cast<unsigned long long>(errors_after_sync_3s),
        static_cast<unsigned long long>(concealed_count));
    return ring_died ? 1 : 0;
  }

  // The decoder-input thread (design-reader-thread.md). From here on the
  // ring, the StreamFeeder and the router are its. On hardware the decoder's
  // output side is the decode pump's and the display the release thread's
  // (started below); this thread -- housekeeping -- keeps the OSD, DVR,
  // stats, the phase lock, the watchdog and the hotplug retry.
  maburplay::FeedLoop feed(ring, router, feeder, {});
  auto drain_notes = [&]() {
    std::vector<maburplay::FeedNote> got;
#ifndef MABUR_PLAYER_HW
    // No decode pump takes the submit notes here; they would pile up.
    feed.take_submit_notes(&got);
    got.clear();
#endif
    feed.take_notes(&got);
    for (maburplay::FeedNote& n : got) process_note(std::move(n));
  };
  // Notes first, then a waiting flush: everything before the flush record
  // reaches the DVR before the flush runs.
  auto service_feed = [&]() {
    drain_notes();
    if (feed.flush_waiting()) {
      drain_notes();
      do_flush();
      feed.resume();
    }
  };
  router.set_hooks({[&feed](maburplay::FeedNote&& n) { feed.push_note(std::move(n)); },
                    [&feed]() { return feed.park_for_flush(); }, [] { return mono_us(); }});
  feed_ptr = &feed;
#ifdef MABUR_PLAYER_MPP
  if (auto* mpp = dynamic_cast<maburplay::MppBackend*>(backend.get()))
    mpp->set_input_cancel(&feed.cancel_flag());
#endif
  // Streamed-decode counters + ring wake latency, every 5 s, from the feed
  // thread (it owns every input). wake = closed-record delivery - t_complete.
  feed.set_periodic([&]() {
    const auto wk = ring.take_wake();
    uint64_t stream_refused = 0;
#ifdef MABUR_PLAYER_MPP
    // backend read on the feed thread: safe only because the watchdog re-creates it while the feed is parked.
    if (auto* mpp = dynamic_cast<maburplay::MppBackend*>(backend.get()))
      stream_refused = mpp->stream_errors();
#endif
    std::fprintf(stderr,
                 "stream: %s streamed=%llu salvaged=%llu aborted=%llu whole=%llu "
                 "opened=%llu open_aborted=%llu wake_us=%u/%u n=%u refused=%llu parks=%llu\n",
                 maburplay::stream_off_name(feeder.off_reason()),
                 static_cast<unsigned long long>(feeder.streamed()),
                 static_cast<unsigned long long>(feeder.streamed_salvaged()),
                 static_cast<unsigned long long>(feeder.stream_aborted()),
                 static_cast<unsigned long long>(feeder.whole_submits()),
                 static_cast<unsigned long long>(ring.opened()),
                 static_cast<unsigned long long>(ring.open_aborted()), wk.p50, wk.p99, wk.n,
                 static_cast<unsigned long long>(stream_refused),
                 static_cast<unsigned long long>(feed.parks()));
#ifdef MABUR_PLAYER_V4L2REQ
    // backend read on the feed thread: safe only because the watchdog re-creates it while the feed is parked.
    if (auto* v4 = dynamic_cast<maburplay::V4l2ReqBackend*>(backend.get()))
      std::fprintf(stderr, "v4l2req: frames=%llu errors=%llu bitstream=%llu shape_drops=%llu gated=%llu\n",
                   static_cast<unsigned long long>(v4->frames()),
                   static_cast<unsigned long long>(v4->errors()),
                   static_cast<unsigned long long>(v4->bitstream()),
                   static_cast<unsigned long long>(v4->dropped_shape()),
                   static_cast<unsigned long long>(v4->gated()));
#endif
  }, 5000);
#ifdef MABUR_PLAYER_HW
  // The output side, before the feed: the pump takes the submit notes the
  // feed starts pushing, and every frame it hands over needs the release
  // thread. The prime hook reads `presenter`, which the hotplug retry
  // assigns only while the pump is parked.
  pump.set_feed(&feed);
  pump.set_prime([&presenter](const maburplay::DmaFrame& f) {
    if (presenter) presenter->prime(f);
  });
  pump.start();
  release.start();
#endif
  feed.start();
  std::fprintf(stderr, "maburplay: feed thread on (ring reader + decoder input off the render loop)\n");

  // Decoded-frame figures for the loop below: the decode pump's on
  // hardware, the inline sink's otherwise.
  const auto decoded_frames = [&]() -> uint64_t {
#ifdef MABUR_PLAYER_HW
    return pump.frames();
#else
    return frame_count;
#endif
  };
  const auto have_decoded = [&]() -> bool {
#ifdef MABUR_PLAYER_HW
    return pump.have_frame();
#else
    return have_first_frame;
#endif
  };
  // --fps-log bookkeeping: a once-per-second stderr line, computed off the
  // same decoded-frame count.
  auto t_last_fps_log = std::chrono::steady_clock::now();
  uint64_t frames_at_last_fps_log = 0;
#ifdef MABUR_PLAYER_HW
  maburplay::BurnRecorder::StageUs burn_stage_prev;  // fps-log per-window stage means
#endif

  uint64_t flips_at_last_fps_log = 0;
  // Decode watchdog: rkvdec2 can hang on malformed/mid-session bitstream
  // and the kernel's force-reset leaves userspace MPP wedged (parser+hal
  // parked on futexes, decode_get_frame silent forever, kernel log shows
  // "mpp_rkvdec2 ... task timeout ... resetting"). If AUs keep flowing but
  // no frame emerges for 2 s, reset the whole pipeline and resync at the
  // next sid0 -- self-healing beats a frozen screen.
  uint64_t wd_frames = 0, wd_submits = 0;
  int wd_consecutive = 0;  // fruitless reset cycles since the last decoded frame
  auto wd_last_progress = std::chrono::steady_clock::now();
  // OSD silence diagnostic: osd.port must equal maburgs' msp.out.port, a
  // deploy-time pairing across two config files that neither daemon can
  // validate. Production runs maburplay WITHOUT --fps-log (S97maburplay), so
  // the osd_dgrams counter that would expose a mismatch is invisible -- warn
  // once, on stderr, if nothing has ever arrived.
  const uint64_t osd_start_ms = (uint64_t)std::chrono::duration_cast<std::chrono::milliseconds>(
                                    std::chrono::steady_clock::now().time_since_epoch())
                                    .count();
  bool osd_silence_warned = false;
  bool gs_silence_warned = false;
  // Render gate. The overlay's inputs are exactly three -- the snapshot, the
  // player-measured figures, and the staleness flag -- so re-running
  // update() on every 2 ms iteration would re-format 33 fields ~500 times a
  // second to discover nothing changed. Seeded true so the never-received
  // dash state paints as soon as there is a surface, without waiting for a
  // first datagram that may never come.
  [[maybe_unused]] bool gs_needs_render = true;  // read only in the cross build
  bool gs_stale_last = false;
  // Both overlays' state for the composer, filled by the two intake blocks
  // and consumed by the single composition block after them.
  [[maybe_unused]] bool msp_ready = false;
  [[maybe_unused]] bool msp_stale = false;
  [[maybe_unused]] bool gs_stale = false;
#ifdef MABUR_PLAYER_HW
  // Layout is deferred to the first iteration that sees a real surface: its
  // size is the DRM buffer's, which DrmPresenter may have allocated at a
  // mode the connector offered rather than the one cfg.screen_mode asked
  // for. Laying out against the config would misplace every field.
  bool gs_laid_out = false;
  // Display hotplug recovery state. init() only accepts an already-CONNECTED
  // connector, so before this a display plugged in or powered on after
  // startup stayed invisible for the life of the process -- the screen was
  // black until someone restarted maburplay by hand.
  const uint64_t display_retry_t0_ms = mono_ms();
  uint64_t display_retry_next_ms = 0;
  bool display_retry_announced = false;
  // Phase margins taken from the release thread each iteration.
  std::vector<maburplay::ReleaseLoop::MarginSample> margins;
  margins.reserve(maburplay::ReleaseLoop::kMarginCap);
  // Builds a FRESH presenter on `mode` and makes it the live display,
  // never a re-init of an existing one: every connector/mode/plane/zpos
  // decision is re-derived from what the display offers now. Two callers:
  // the hotplug retry (no presenter) and a live change of the display mode
  // or the overlay setup (a presenter is running). Housekeeping only.
  //  - No presenter: the new one is built first and output parks only once
  //    it succeeded, so a retry that finds no display costs nothing.
  //  - A live presenter: it is torn down first -- the burned recorder (it
  //    samples the old display's vblank grid) is stopped, both output
  //    threads park so nothing is held on the old presenter, the release
  //    thread drops its sink, and only then is the presenter destroyed. If
  //    the new one then fails, output resumes with no display and the
  //    hotplug retry takes over on cfg.screen_mode.
  // Returns whether a presenter is live on `mode` afterwards.
  auto reacquire_display = [&](const std::string& mode, bool log_failures) -> bool {
    const bool tear_down = presenter != nullptr;
    if (tear_down) {
      if (burn) stop_burn();
      park_output("display rebuild");
      release.set_sink(nullptr);
      presenter.reset();
    }
    auto p = std::make_unique<maburplay::DrmPresenter>();
    if (!p->init(mode, want_osd, [&releases](const maburplay::DmaFrame& f) { releases.push(f); },
                 log_failures)) {
      if (tear_down) {
        resume_output();
        std::fprintf(stderr,
                     "maburplay: display rebuild on %s failed -- no display; the 1 s retry takes "
                     "over\n",
                     mode.c_str());
      }
      return false;
    }
    // Both output threads park across the swap: the pump's prime hook
    // reads `presenter`, and the release thread takes the new sink only
    // while parked. Sink-less until now, the release thread's park returns
    // just the frames the regulator holds.
    if (!tear_down) park_output("display acquired");
    presenter = std::move(p);
    install_colortrans(presenter.get());
    // Same pre-first-flip reseed as the startup path: the new panel may run
    // a different mode/refresh than the one configured.
    regulator.set_panel_period(presenter->mode_period_us());
    // Unlike the startup path, the lock may already be locked or acquiring
    // against the OLD panel's grid here: restart it so the new mode gets a
    // fresh window instead of up to 2 s of capture placement judged against
    // a stale vblank period. Margins taken against the old grid go with it.
    if (phase_lock) phase_lock->on_discontinuity(mono_us());
    margins.clear();
    release.take_margins(&margins);
    margins.clear();
    presenter->set_flip_sink(
        [&lat, &regulator, &tap](const maburplay::DmaFrame& f, uint64_t t, bool ex) {
          lat.on_flip(f.pts_us, t, ex);
          regulator.on_flip(t, ex);
          tap.on_flip(f, t);
        });
    // The GS bar re-lays out on the new surface, whose size may differ.
    gs_laid_out = false;
    // Startup-only, and this is where that invariant is enforced for a late
    // or rebuilt display: if this process has ever decoded a frame, the
    // aircraft is flying and a still photo on the goggles would read as a
    // live feed. The screen still lights (needs_modeset defaults true and
    // was never cleared since splash_show() was skipped, so the first
    // present() performs the full modeset); it simply comes up on video, or
    // on nothing until video returns.
    if (!pump.have_frame()) show_splash(presenter.get());
    release.set_sink(presenter.get());
    resume_output();
    // The recorder samples this display's vblank grid, so it could not
    // exist before this moment.
    start_burn_if_needed();
    return true;
  };
#endif
  // The ring reader fell behind and skipped AUs that held references.
  uint64_t resyncs_seen = ring.resyncs();
  while (!g_stop.load()) {
    // 2 ms ceiling, for the delivery notes and the record button; the feed
    // thread ends the wait earlier only when it parks for a flush or exits.
    // Nothing deadline-bound runs on this thread: on hardware the decode
    // pump collects every decoded frame and the release thread releases it
    // on time whatever this loop is doing.
    feed.wait_main(2);
    service_feed();
#ifndef MABUR_PLAYER_HW
    backend->poll();
#endif
    // One ioctl per iteration (~500/s, microseconds each) -- far under the
    // ~1.5 ms budget tools/bench/gs_osd_bench.cpp polices for this loop.
    if ((rec_button.is_open() && rec_button.poll(mono_ms())) || g_rec_toggle.exchange(false)) {
      if (rec_on) {
        rec_stop();
      } else {
        rec_start();
      }
    }
    // Every iteration: sends on change, re-sends each second (CoreControlClient).
    core_ctl.tick(rec_on && rec_vtx, mono_ms());
#ifdef MABUR_PLAYER_HW
    // A burned start deferred for want of a decoded frame to size the
    // recording from runs as soon as one exists.
    if (burn_deferred && pump.have_frame()) {
      burn_deferred = false;
      start_burn_if_needed();
    }
    // The phase margin of every offer the release thread made since the
    // last iteration, oldest first. The lock takes them only while it
    // runs: with the source and panel rates out of its reach (rate
    // mismatch) the margins mean nothing to it.
    margins.clear();
    release.take_margins(&margins);
    if (phase_lock && !rate_match.mismatch())
      for (const maburplay::ReleaseLoop::MarginSample& m : margins)
        phase_lock->on_margin(m.margin_us, m.period_us, m.t_us);
    // The DECODED picture size, which is what the OSD's res: field means --
    // not screen_mode (the panel) and not bcfg (the backend's configured
    // size), both of which stay at 1080p while the drone streams 720p.
    gs_ps.vid_w = pump.video_w();
    gs_ps.vid_h = pump.video_h();
    // Drone fps vs panel Hz, once a second, whether or not the GS OSD (the
    // only sideport reader) is up; its low-power flag counts only while
    // fresh.
    if (mono_us() - rate_match_mark_us >= 1'000'000) {
      rate_match_mark_us = mono_us();
      const uint64_t now_ms = rate_match_mark_us / 1000;
      const auto lp = maburplay::low_power_report(
          gs_src != nullptr, gs_src && gs_src->have_any() && !gs_src->stale(now_ms),
          gs_src && gs_src->snapshot().low_power);
      const double panel_hz =
          regulator.panel_period_us() > 0 ? 1e6 / regulator.panel_period_us() : 0.0;
      const double src_hz = pump.source_hz();
      const bool was = rate_match.mismatch();
      rate_match.on_window(src_hz, panel_hz, lp);
      gs_ps.rate_mismatch = rate_match.mismatch();
      gs_ps.src_hz = src_hz;
      gs_ps.panel_hz = panel_hz;
      if (rate_match.mismatch() != was)
        std::fprintf(stderr,
                     rate_match.mismatch()
                         ? "maburplay: rate mismatch: source %.2f Hz, panel %.3f Hz%s\n"
                         : "maburplay: rate mismatch cleared: source %.2f Hz, panel %.3f Hz%s\n",
                     src_hz, panel_hz,
                     rate_match.mismatch() && phase_lock ? " -- phase lock paused" : "");
    }
    if (phase_lock) {
      const uint64_t now = mono_us();
      const auto before = phase_lock->state();
      if (regulator.discont_count() != phase_disconts_seen) {
        phase_disconts_seen = regulator.discont_count();
        phase_lock->on_discontinuity(now);
      }
      if (rate_match.mismatch()) {
        // Out of the steer's reach: no offsets at all, so maburgs' RCF
        // snr_off falls back to 0 and the drone runs on its trim. Reset to
        // Waiting so a later match acquires from scratch.
        if (phase_lock->state() != maburplay::PhaseLock::State::Waiting)
          phase_lock->on_discontinuity(now);
        regulator.set_capture_placement(false);
      } else {
        phase_lock->tick(now);
        regulator.set_capture_placement(phase_lock->locked());
        core_ctl.tick_phase(phase_lock->offset_mfps(), now / 1000);
      }
      gs_ps.phase_oor = phase_lock->state() == maburplay::PhaseLock::State::OutOfRange;
      if (phase_lock->state() != before) {
        std::fprintf(stderr, "phase-lock: %s err=%.2fms off=%d centre=%.1f\n",
                     maburplay::phase_lock_state_name(phase_lock->state()),
                     phase_lock->error_ms(), static_cast<int>(phase_lock->offset_mfps()),
                     phase_lock->centre_mfps());
      }
    }
#endif
    if (ring.resyncs() != resyncs_seen) {
      resyncs_seen = ring.resyncs();
      core_ctl.send_line("refresh resync");
    }
#ifdef MABUR_PLAYER_HW
    // Retry acquisition once a second while there is no display. A FRESH
    // presenter every time, never a re-init of the failed one: a display that
    // appears late supplies its own EDID and mode list, and every
    // connector/mode/plane/zpos decision has to be re-derived from it.
    // Constructing only when `presenter` is null means no frames are ever
    // held across a recreation, so drm_presenter.h's ownership contract is
    // untouched.
    if (!presenter && !decode_only) {
      const uint64_t now_ms = mono_ms();
      if (!display_retry_announced) {
        display_retry_announced = true;
        display_retry_next_ms = now_ms;
        std::fprintf(stderr,
                     "maburplay: no display at startup -- retrying every 1 s; frames are decoded "
                     "and dropped until one appears\n");
      }
      if (now_ms >= display_retry_next_ms) {
        display_retry_next_ms = now_ms + 1000;
        // log_failures=false: this runs 86,400 times a day on a GS with no
        // screen, and /tmp is tmpfs.
        if (reacquire_display(cfg.screen_mode, /*log_failures=*/false)) {
          std::fprintf(stderr, "maburplay: display acquired after %.1f s\n",
                       (now_ms - display_retry_t0_ms) / 1000.0);
          // rec_on, not burned_mode alone: a player deliberately stopped (or
          // never started -- autostart:false) has nothing to report here, and
          // saying otherwise would be a false alarm on every late display. A
          // deferred start (no decoded frame yet) said so and is retried.
          if (rec_on && rec_gs && burned_mode && !burn && !burn_deferred)
            std::fprintf(stderr,
                         "maburplay: display acquired but the burned recorder did not start -- "
                         "NOTHING is being recorded\n");
        }
      }
    }
#endif
    // OSD: drain the UDP intake every iteration whether or not anything can
    // be drawn (the counters stay honest, and an undrained socket would just
    // fill), then repaint only when a fresh complete screen arrived.
    if (osd_src) {
      const uint64_t now_ms = (uint64_t)std::chrono::duration_cast<std::chrono::milliseconds>(
                                  std::chrono::steady_clock::now().time_since_epoch())
                                  .count();
      // Sticky until a composition consumes it: the composition can be
      // held off (the OSD buffers are busy until the display takes the last
      // publish), and a fresh screen must not be lost meanwhile.
      if (osd_src->poll(now_ms)) msp_ready = true;
      msp_stale = osd_src->stale(now_ms);
      // Fires at most once per process: either traffic arrived (nothing to
      // report, latch it shut) or 10 s passed with none (report, latch shut).
      if (!osd_silence_warned) {
        if (osd_src->datagrams() > 0) {
          osd_silence_warned = true;
        } else if (now_ms - osd_start_ms >= 10000) {
          std::fprintf(stderr,
                       "maburplay: warning: no MSP snapshot on udp 127.0.0.1:%d after 10 s -- "
                       "maburgs must have msp.enable true and its msp.out.port MUST equal this "
                       "player's osd.port; the OSD stays empty until one arrives\n",
                       osd_src->port());
          osd_silence_warned = true;
        }
      }
    }
    // GS link-status overlay. Drained every iteration whether or not
    // anything can be drawn -- the counters stay honest and an undrained
    // socket would just fill -- then repainted when one of its three inputs
    // moved.
    if (gs_src) {
      const uint64_t now_ms = mono_ms();
      if (gs_src->poll(now_ms)) gs_needs_render = true;
      gs_stale = gs_src->stale(now_ms);
      if (gs_stale != gs_stale_last) {
        gs_stale_last = gs_stale;
        gs_needs_render = true;
      }

      // Same one-shot diagnostic as the MSP intake's, and for the same
      // reason: osd.gs.port must equal one of maburgs' stats.out ports, a
      // pairing across two config files that neither daemon can validate.
      if (!gs_silence_warned) {
        if (gs_src->datagrams() > 0) {
          gs_silence_warned = true;
        } else if (now_ms - osd_start_ms >= 10000) {
          std::fprintf(stderr,
                       "maburplay: warning: no stats snapshot on udp 127.0.0.1:%d after 10 s -- "
                       "maburgs must have stats.enable true and one of its stats.out ports MUST "
                       "equal this player's osd.gs.port; the GS overlay stays empty until one "
                       "arrives\n",
                       gs_src->port());
          gs_silence_warned = true;
        }
      }

      // Video and recording figures at 1 Hz. The sideport itself runs at
      // 2 Hz, but these are the player's own and recomputing them per
      // iteration would only add noise to numbers read at a glance.
      if (now_ms - gs_video_mark_ms >= 1000) {
        // The gate guarantees dt >= 1.0, so there is nothing to divide by
        // zero; gs_video_mark_ms == 0 is the first tick, which has no
        // interval behind it and therefore only seeds the marks.
        const double dt = (double)(now_ms - gs_video_mark_ms) / 1000.0;
        if (gs_video_mark_ms != 0) {
          // DECODED frames, not AUs: maburgs' own fps counts what it
          // published to the ring and still reads 60 while a wedged decoder
          // shows a frozen picture. This one falls to zero.
          gs_ps.fps = (double)(decoded_frames() - gs_frames_at_mark) / dt;
          gs_ps.mbps = (double)(gs_bytes_total - gs_bytes_at_mark) * 8.0 / 1e6 / dt;
        }
        gs_frames_at_mark = decoded_frames();
        gs_bytes_at_mark = gs_bytes_total;
        gs_video_mark_ms = now_ms;
        gs_jitter.on_tick(now_ms);  // an outage must not leave a stale figure
        gs_ps.jitter_ms = gs_jitter.ms();
        gs_needs_render = true;

        // Recording state. Both the samples and the thing that would make
        // them advance differ by mode: the raw path muxes complete AUs
        // straight off the ring and does not care whether anything decodes,
        // while burned mode encodes the decoded frames the display flips to.
        maburplay::RecTracker::Inputs rin;
        // dvr.samples() still holds the SEALED file's count until the next
        // open(), so a stopped raw recorder would keep the OSD at REC.
        rin.samples = rec_on ? dvr.samples() : 0;
        rin.feed = gs_complete_aus;
        rin.open = dvr.state() == mabur::RawDvr::State::Recording;
        // rec_on, for the same reason as the burned branch below: a failed
        // open is only a FAULT while we are supposed to be recording. Once
        // the user has stopped, a stale latch would keep the OSD red until
        // the next start (rec_start is what clears it).
        rin.broken = rec_on && dvr.state() == mabur::RawDvr::State::Error;
        if (burned_mode) {
          rin.feed = decoded_frames();
#ifdef MABUR_PLAYER_HW
          rin.samples = burn ? burn->frames_encoded() : 0;
          rin.open = burn != nullptr;
          // Only a MISSING recorder we are supposed to have is broken. A
          // deliberately stopped one is ARMED, not FAULT, and so is a start
          // deferred until the first decoded frame sizes the recording: it
          // is waiting for video, like the raw path waiting for a sync
          // point, and comes up on its own.
          rin.broken = rec_on && burn == nullptr && !burn_deferred;
#else
          rin.samples = 0;
          rin.open = false;
          rin.broken = rec_on;  // burned mode needs the hardware build; already logged
#endif
        }
        // Polled at this same 1 Hz, and BEFORE the file exists: a card that
        // is already full must fault rather than sit at ARMED forever.
        {
          struct statvfs vfs {};
          if (::statvfs(cfg.dvr.dir.c_str(), &vfs) == 0) {
            const uint64_t free_bytes = (uint64_t)vfs.f_bavail * (uint64_t)vfs.f_frsize;
            rin.low_space = free_bytes < 64ull * 1024 * 1024;
          } else if (!statvfs_warned) {
            // A failed syscall is not evidence of a full disk, so low_space
            // stays false and the state is left alone. Said once; retried
            // silently thereafter.
            statvfs_warned = true;
            std::fprintf(stderr,
                         "maburplay: gs osd: statvfs(%s) failed (%s) -- recording free-space "
                         "checks will be skipped whenever it keeps failing\n",
                         cfg.dvr.dir.c_str(), std::strerror(errno));
          }
        }
        gs_ps.rec = gs_rec.update(rin, now_ms);
        {
          // VTX leg from the drone's drone.rec via maburgs.
          const maburplay::GsSnapshot& snap = gs_src->snapshot();
          maburplay::VtxRecTracker::Inputs vin;
          vin.requested = rec_on && rec_vtx;
          vin.fresh = gs_src->have_any() && !gs_src->stale(now_ms) && snap.drone_tlm_age_ms &&
                      *snap.drone_tlm_age_ms <= static_cast<int>(maburplay::VtxRecTracker::kFreshMs);
          vin.state = snap.rec_state;
          vin.err = snap.rec_err;
          const auto leg = vtx_trk.update(vin, now_ms);
          gs_ps.rec.gs_target = rec_gs;
          gs_ps.rec.vtx = leg.leg;
          gs_ps.rec.vtx_elapsed_s = leg.elapsed_s;
        }

      }
    }
#ifdef MABUR_PLAYER_HW
    // Player tail-latency aggregates + regulator line, 1 Hz. Deliberately
    // OUTSIDE the gs_src gate (nested in the GS-OSD block, disabling the
    // OSD would silently kill the lat:/regulator: lines and the DVR lat
    // log). Its cost lands on this thread only, never on a release.
    if (mono_ms() - lat_mark_ms >= 1000) {
      lat_mark_ms = mono_ms();
      const auto L = lat.flush_line();
      // OSD LAT rows: p99_frame() returns the REAL p99-by-e2e
      // frame, computed as a side effect inside flush_line() above from
      // the window it is about to clear -- it is a member that
      // persists across flush_line() calls, not derived
      // from `completed_` at call time, so calling it after flush_line()
      // here is safe and gets THIS window's frame (see lat_tracker.h and
      // lat_tracker.cpp's flush_line()/p99_frame()). A window with zero
      // completed frames leaves it holding the last valid frame rather
      // than clearing it -- an idle 1 Hz tick between two spiky ones
      // still shows the most recent real spike instead of flickering to
      // "LAT --" and back.
      const auto bd = lat.p99_frame();
      // `bd.valid` alone is NOT enough: p99_frame_ is a member that
      // survives flush_all() (lat_tracker.cpp only clears map_/anchor_/
      // completed_/the chk+dsp accumulators there, not p99_frame_), so
      // after any decoder/session reset it keeps reporting the LAST
      // pre-reset frame as if it were current. `L.anchor_ok`, from this
      // SAME flush_line() call, is what actually reflects whether the
      // window just flushed has a usable anchor -- gate on both, or a
      // reset shows a stale number labeled current for ~1s+ instead of
      // "LAT P99 --".
      gs_ps.lat_valid = bd.valid && L.anchor_ok;
      // Gated on gs_ps.lat_valid (the AND), not bd.valid alone -- same
      // reasoning as above: a stale post-reset bd would otherwise still
      // get copied into gs_ps.lat_e2e_ms even though the OSD is about to
      // ignore it, leaving gs_ps holding a phantom number.
      // link-rtt: absolute floor = own anchor + the sideport
      // pts offset (floor_us_from owns the 32-bit-seed wrap rule). Folded
      // into e2e, turning it from "excess above the luckiest transit" into
      // capture-stamp→scanout-start truth. (The jsonl
      // lat: line below is deliberately untouched by the floor -- it is
      // the relative record the older recordings are in.) Sanity-bounded:
      // an offset that puts the floor outside (-1ms, 1s) is estimator
      // garbage — stay relative (~) rather than display a fabricated
      // absolute.
      // The floor stays in µs: abs_lat_ms() rounds the sum once, so the
      // cell never sits a millisecond off the rounded true sum.
      int64_t floor_us = 0;
      gs_ps.lat_abs = false;
      if (gs_src && lat.anchor().usable()) {
        if (const auto& off = gs_src->snapshot().pts_off_us) {
          const int64_t f_us =
              maburgs::floor_us_from(lat.anchor().base_us(), *off);
          if (f_us > -1000 && f_us < 1'000'000) {
            floor_us = f_us;
            gs_ps.lat_abs = true;
          }
        }
      }
      if (gs_ps.lat_valid) {
        gs_ps.lat_e2e_ms = maburplay::abs_lat_ms(bd.e2e_us, floor_us);
        // Median row: same window, same anchor gate, same staleness
        // reasoning as the p99 one above -- p50_frame_ likewise survives
        // flush_all(), so it is gated on the AND, not on bd50.valid alone.
        const auto bd50 = lat.p50_frame();
        if (bd50.valid)
          gs_ps.lat_p50_e2e_ms = maburplay::abs_lat_ms(bd50.e2e_us, floor_us);
      }
      if (L.n > 0) {
        char lat_buf[256];
        std::snprintf(lat_buf, sizeof(lat_buf),
            "lat: n=%d e2e=%u/%u enc=%u/%u dq=%u/%u air=%u/%u fec=%u/%u "
            "dec=%u/%u reg=%u/%u dsp%s=%u/%u chk=%.1f anchor=%s",
            L.n, L.p50[7]/1000, L.p99[7]/1000, L.p50[0]/1000, L.p99[0]/1000,
            L.p50[1]/1000, L.p99[1]/1000, L.p50[2]/1000, L.p99[2]/1000,
            L.p50[3]/1000, L.p99[3]/1000, L.p50[4]/1000, L.p99[4]/1000,
            L.p50[5]/1000, L.p99[5]/1000, L.dsp_exact ? "" : "~",
            L.p50[6]/1000, L.p99[6]/1000, L.chk_ms,
            L.anchor_ok ? "ok" : "warm");
        std::fprintf(stderr, "%s\n", lat_buf);
        lat_log.write(mono_us(), lat_buf);
        // t_flip is the row's own time column; the rest is the chain.
        for (const auto& r : L.rows) {
          char row[256];
          maburplay::format_frame_row(r, row, sizeof(row));
          frame_log.write(r.t_flip_us, row);
        }
        frame_log.flush();
      }
      // Same 1 Hz tick as the lat: line above, so the bench fallback-drill
      // gate ("fallback= climbs then stops after re-warm") has a periodic
      // line to watch live -- not just the final tally at exit.
      log_regulator_line();
    }
#endif
    // ONE composition, for both overlays, into the buffer that is back right
    // now. Not two independent render blocks: they share the back index and
    // the dirty flag, so whichever of them publishes decides what is scanned
    // out -- and a per-buffer piece of state that only the OTHER one
    // refreshes then strobes at that publish rate. osd_compose.{h,cpp} owns
    // that whole problem and is tested directly; this is only the trigger.
#ifdef MABUR_PLAYER_HW
    // Re-checked every iteration, never cached: the presenter switches the
    // OSD off mid-session after repeated commit failures, and from that point
    // osd_back_surface() is a null Surface. The screen composes only while
    // no publish is pending: a published back buffer belongs to the release
    // thread until a commit takes it, and the buffer that commit swaps in as
    // the new back is still on screen until its flip completes -- the
    // presenter keeps osd_publish_pending() true through both, so this never
    // draws into a buffer that is on screen or about to be.
    {
      const bool screen_ok =
          presenter && presenter->osd_available() && !presenter->osd_publish_pending();
      const maburplay::Surface surf =
          screen_ok ? presenter->osd_back_surface() : maburplay::Surface{};
      if (surf.pixels && composer.gs_present() && !gs_laid_out) {
        // Deferred to the first real surface: its size is the DRM buffer's,
        // which DrmPresenter may have allocated at a mode the connector
        // offered rather than the one cfg.screen_mode asked for.
        std::string err;
        if (!composer.gs_layout(surf.width, surf.height, &err)) {
          std::fprintf(stderr, "maburplay: gs osd disabled -- %s\n", err.c_str());
          gs_src.reset();  // the live gate; the overlays are already dropped
        } else {
          gs_laid_out = true;
        }
      }
      maburplay::OsdComposeIn in;
      if (osd_src && osd_raster) {
        in.screen = &osd_src->screen();
        in.msp_fresh = msp_ready;
        in.msp_stale = msp_stale;
      }
      if (gs_src) {
        in.snap = &gs_src->snapshot();
        in.gs_stale = gs_stale;
        in.gs_ps = gs_ps;
        in.gs_dirty = gs_needs_render;
      }
      const bool want = composer.wants(in);
      // The recording's overlay, at the record size, on its own trigger: the
      // same input the screen would compose, whether or not the screen can
      // compose this iteration -- a pending publish holds the screen back,
      // never the recording. It runs BEFORE the screen half clears
      // msp_ready/gs_needs_render, and does not clear them itself: a screen
      // held back by a pending publish still consumes them on its next
      // composition. A repeat of the same sample draws nothing.
      if (burn && want) composer.compose_burn(in);
      if (surf.pixels && want) {
        const int idx = presenter->osd_back_index();
        const maburplay::OsdComposeOut out = composer.compose_screen(idx, surf, in);
        gs_needs_render = false;
        msp_ready = false;
        // Unconditional whenever a composition ran, even if it drew
        // nothing: "nothing differs from this buffer" does not mean the
        // screen is right -- the screen is the OTHER buffer. With a value
        // that alternates (fps 60/59/60, an RSSI wobbling by one) each
        // buffer keeps matching its own last composition, and a publish
        // gated on "something changed" strands the front buffer on the
        // wrong sample forever.
        if (out.published) presenter->osd_publish();
        if (out.announce_blank)
          std::fprintf(stderr, "maburplay: osd blanked after %d ms without a snapshot\n",
                       cfg.osd.stale_ms);
      }
    }
#endif
    {
      const auto now = std::chrono::steady_clock::now();
      if (decoded_frames() != wd_frames) {
        wd_frames = decoded_frames();
        wd_submits = router.submits();
        wd_last_progress = now;
        wd_consecutive = 0;
      } else if (have_decoded() && router.submits() > wd_submits + 60 &&
                 now - wd_last_progress > std::chrono::seconds(2)) {
        // have_decoded() gate: before the decoder has EVER produced a
        // frame (cold attach waiting for the encoder's session sync), a
        // fire here would be spurious -- there is nothing to reset yet.
        ++wd_consecutive;
        std::fprintf(stderr,
                     "maburplay: decode watchdog -- %llu AUs submitted with no decoded frame "
                     "for 2 s; resetting decoder and resyncing (attempt %d)\n",
                     static_cast<unsigned long long>(router.submits() - wd_submits),
                     wd_consecutive);
        // The decoder's input side is the feed thread's: park it (between two
        // events, outside every MPP call) before anything below resets or
        // re-creates the decoder. A feed that cannot park in 2 s is wedged
        // beyond this process: respawn.
        if (!feed.park(std::chrono::seconds(2))) {
          std::fprintf(stderr,
                       "maburplay: decode watchdog -- the feed thread did not park in 2 s; "
                       "exiting for respawn\n");
          std::fflush(stderr);
          std::_Exit(1);
        }
        drain_notes();
#ifdef MABUR_PLAYER_HW
        // And the output side: the pump out of every backend call, the
        // release thread off the display, every frame back in the decoder.
        park_output("decode watchdog");
#endif
        drop_holders();
        feeder.on_flush();  // end a streamed picture before reset/teardown
        if (wd_consecutive < 3) {
          backend->flush();
        } else if (wd_consecutive == 3) {
          // Escalation: mpi->reset() is provably insufficient after a
          // kernel-side rkvdec2 force-reset (observed live: 52 consecutive
          // no-op resets while the ring flowed clean). Tear the whole MPP
          // context down and build a fresh one.
          std::fprintf(stderr,
                       "maburplay: decode watchdog -- reset ineffective; recreating the "
                       "decoder context\n");
          backend.reset();  // destroy the wedged context BEFORE creating anew
          backend = maburplay::make_backend(cfg.backend);
#ifdef MABUR_PLAYER_MPP
          if (auto* mpp = dynamic_cast<maburplay::MppBackend*>(backend.get()))
            mpp->set_stream(cfg.decoder.stream);
#endif
          if (!backend || !backend->init(bcfg, decoder_sink)) {
            std::fprintf(stderr,
                         "maburplay: decoder recreation failed -- exiting for respawn\n");
            return 1;
          }
#ifdef MABUR_PLAYER_HW
          // While parked and after init(), as at startup. Anything queued for
          // release since the park belongs to the destroyed decoder and is
          // discarded, never handed to the new one.
          pump.set_backend(backend.get());
#endif
          feeder.set_decoder(dynamic_cast<maburplay::StreamDecoder*>(backend.get()));
#ifdef MABUR_PLAYER_MPP
          if (auto* mpp = dynamic_cast<maburplay::MppBackend*>(backend.get()))
            mpp->set_input_cancel(&feed.cancel_flag());
#endif
        } else {
          // Even a fresh context won't decode: something below us (VPU,
          // kernel, stream) needs a full process restart. The init wrapper
          // respawns us in ~1 s with a clean slate.
          std::fprintf(stderr,
                       "maburplay: decode watchdog -- recreation ineffective; exiting for "
                       "respawn\n");
          return 1;
        }
#ifdef MABUR_PLAYER_HW
        resume_output();
#endif
        core_ctl.send_line("refresh decoder");
        router.disarm();  // resync at the next sid0 AU
        feed.resume();
        wd_submits = router.submits();
        wd_last_progress = now;
      }
    }
    if (fps_log) {
      const auto now = std::chrono::steady_clock::now();
      const double dt = std::chrono::duration<double>(now - t_last_fps_log).count();
      if (dt >= 1.0) {
        const uint64_t frames_now = decoded_frames();
        const double fps = static_cast<double>(frames_now - frames_at_last_fps_log) / dt;
#ifdef MABUR_PLAYER_HW
        if (presenter) {
          const uint64_t flips = presenter->flips();
          // Burned-DVR counters, APPENDED to the existing fields (existing
          // names and order untouched) and only in burned mode, so a raw run
          // logs exactly the line it always did. Built into one buffer rather
          // than a second fprintf: the recorder thread writes to stderr too.
          // burn_enc counts encode() calls that produced a packet -- NOT
          // samples written, since a zero-length packet reaches the mux sink
          // and is dropped there.
          char burn_fields[320] = {0};
          if (burn) {
            // Per-window mean stage times (ms): the recorder thread's
            // colortrans stage and encode() -- what bounds the DVR's fps.
            const maburplay::BurnRecorder::StageUs su = burn->stage_us();
            const uint64_t dct_n = su.ct_n - burn_stage_prev.ct_n;
            const uint64_t dct = su.ct_sum - burn_stage_prev.ct_sum;
            const uint64_t denc_n = su.enc_n - burn_stage_prev.enc_n;
            const uint64_t denc = su.enc_sum - burn_stage_prev.enc_sum;
            burn_stage_prev = su;
            std::snprintf(burn_fields, sizeof(burn_fields),
                          " burn_in=%llu burn_enc=%llu burn_drop=%llu burn_flush=%llu "
                          "burn_err=%llu burn_osdrej=%llu burn_ctfb=%llu burn_ct_ms=%.1f "
                          "burn_enc_ms=%.1f",
                          static_cast<unsigned long long>(burn->frames_in()),
                          static_cast<unsigned long long>(burn->frames_encoded()),
                          static_cast<unsigned long long>(burn->frames_dropped()),
                          static_cast<unsigned long long>(burn->frames_flushed()),
                          static_cast<unsigned long long>(burn->encode_errors()),
                          static_cast<unsigned long long>(burn->osd_rejects()),
                          static_cast<unsigned long long>(burn->colortrans_fallbacks()),
                          dct_n ? dct / 1000.0 / dct_n : 0.0, denc_n ? denc / 1000.0 / denc_n : 0.0);
          } else {
            burn_stage_prev = maburplay::BurnRecorder::StageUs{};
          }
          std::fprintf(stderr,
                       "fps-log: fps=%.1f flips/s=%.1f repl=%llu frames=%llu commit_errors=%llu "
                       "async=%s osd_screens=%llu osd_dgrams=%llu osd_commit_errors=%llu%s\n",
                       fps, static_cast<double>(flips - flips_at_last_fps_log) / dt,
                       static_cast<unsigned long long>(presenter->busy_replaced()),
                       static_cast<unsigned long long>(frames_now),
                       static_cast<unsigned long long>(presenter->commit_errors()),
                       !presenter->async_probed() ? "probing"
                       : presenter->async_flip_active() ? "on" : "off",
                       static_cast<unsigned long long>(osd_src ? osd_src->screens() : 0),
                       static_cast<unsigned long long>(osd_src ? osd_src->datagrams() : 0),
                       static_cast<unsigned long long>(presenter->osd_commit_errors()),
                       burn_fields);
          flips_at_last_fps_log = flips;
          // Stall diagnostic: a zero-fps second mid-session means the
          // delivery chain froze somewhere -- dump where the reader sits
          // relative to the ring so the stuck stage is identifiable.
          if (fps < 0.5 && frames_now > 0)
            feed.request_debug_line();  // printed by the feed thread, which owns the ring
        } else
#endif
        {
          std::fprintf(stderr, "fps-log: fps=%.1f frames=%llu\n", fps,
                       static_cast<unsigned long long>(frames_now));
        }
        t_last_fps_log = now;
        frames_at_last_fps_log = frames_now;
      }
    }
    if (feed.dead()) {
      std::fprintf(stderr, "maburplay: ring reader dead, exiting\n");
      break;
    }
  }

  // Clean shutdown. A streamed picture still
  // open in rkvdec when MPP is torn down gets mpp_dec_stop's empty LAST
  // (mpp_dec.c:877) -- 64 zero bytes in the kernel -- and decodes into an
  // error IRQ: one rkvdec reset per such kill (3 of 13). Finish it with its
  // own last slice instead (its close is at most a frame away), then let it
  // decode. Only on a signal: the watchdog and dead-ring exits are faults.
  auto report_drain = [&](const maburplay::DrainResult& dr) {
    bool decoded = false;
    if (dr.kind == maburplay::DrainResult::kFinished) {
      const uint64_t t1 = mono_ms();
      for (;;) {
#ifdef MABUR_PLAYER_HW
        // The decode pump is still running: it collects the picture.
        decoded = pump.have_frame() && pump.last_pts() == dr.pts;
#else
        decoded = have_decoded_pts && last_decoded_pts == dr.pts;
#endif
        if (decoded || mono_ms() - t1 >= 50) break;
#ifndef MABUR_PLAYER_HW
        backend->poll();
#endif
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }
    std::fprintf(stderr, "maburplay: shutdown: %s pts=%u waited=%u ms%s\n",
                 maburplay::drain_name(dr.kind), dr.pts, dr.waited_ms,
                 dr.kind == maburplay::DrainResult::kFinished
                     ? (decoded ? ", decoded" : ", not decoded in 50 ms")
                 : dr.kind == maburplay::DrainResult::kAborted
                     ? " -- its close never came: expect one rkvdec reset"
                     : "");
  };
  if (g_stop.load() && !feed.dead()) {
    feed.begin_drain();
    const uint64_t t0 = mono_ms();
    while (!feed.finished() && mono_ms() - t0 < 500) {  // the feed's own budget is 100 ms
      feed.wait_main(1);
      service_feed();   // a flush can still park it
#ifndef MABUR_PLAYER_HW
      backend->poll();  // keep the output side moving (a BUFFER_FULL needs it)
#endif
    }
  }
  feed.stop_and_join();
  drain_notes();
  if (g_stop.load() && !feed.dead()) {  // a dead ring asked for no drain: nothing to report
    if (feed.drained()) {
      report_drain(feed.drain_result());
    } else {
      std::fprintf(stderr, "maburplay: shutdown: the feed thread did not finish its drain\n");
    }
  }
  std::fprintf(stderr, "maburplay: feed: delivered=%llu submits=%llu parks=%llu notes_dropped=%llu\n",
               static_cast<unsigned long long>(ring.delivered()),
               static_cast<unsigned long long>(router.submits()),
               static_cast<unsigned long long>(feed.parks()),
               static_cast<unsigned long long>(feed.notes_dropped()));

#ifdef MABUR_PLAYER_HW
  // The output side comes down before anything it touches does (the
  // recorder, the presenter, the decoder all unwind after it): park both
  // threads, which empties the presenter, the regulator and the hand-over
  // queue into `releases`, give every frame back to the decoder, then stop
  // them. stop_and_join() alone returns no frames.
  park_output("shutdown");
  // The tap's hook holds a raw pointer to the recorder, which unwinds
  // after this; the release thread is parked, so nothing is in flight.
  tap.clear();
  composer.set_burn_sink({});
  release.stop_and_join();
  pump.stop_and_join();
  // Final tally -- same line the 1 Hz stats block above already printed all
  // session long (log_regulator_line(), declared next to the threads).
  log_regulator_line();
  {
    const auto L = lat.flush_line();
    if (L.n > 0)
      std::fprintf(stderr,
          "lat-final: n=%d e2e=%u/%u enc=%u/%u dq=%u/%u air=%u/%u fec=%u/%u "
          "dec=%u/%u reg=%u/%u dsp%s=%u/%u chk=%.1f anchor=%s\n",
          L.n, L.p50[7]/1000, L.p99[7]/1000, L.p50[0]/1000, L.p99[0]/1000,
          L.p50[1]/1000, L.p99[1]/1000, L.p50[2]/1000, L.p99[2]/1000,
          L.p50[3]/1000, L.p99[3]/1000, L.p50[4]/1000, L.p99[4]/1000,
          L.p50[5]/1000, L.p99[5]/1000, L.dsp_exact ? "" : "~",
          L.p50[6]/1000, L.p99[6]/1000, L.chk_ms,
          L.anchor_ok ? "ok" : "warm");
  }
#endif
  dvr.stop();
  return 0;
}
