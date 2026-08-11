// maburplay -- GS native player: AU ring -> VideoBackend, with fMP4 DVR.
// Host builds only ever link the null backend (this task); MppBackend /
// DrmPresenter are cross-only (MABUR_PLAYER_HW, Task 7/8).
#include <sys/statvfs.h>  // DVR free-space check behind the GS recording block

#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "au_ring.h"
#include "dvr_mux.h"
#include "gs_font.h"
#include "gs_metrics.h"
#include "gs_overlay.h"
#include "osd_compose.h"
#include "gs_source.h"
#include "hevc_params.h"
#include "mabur/frame_wire.h"
#include "mabur/player_fb.h"
#include "osd_font.h"
#include "osd_raster.h"
#include "osd_source.h"
#include "player_config.h"
#include "player_idr_latch.h"
#include "rec_button.h"
#include "ring_client.h"
#include "udp_sink.h"
#include "video_backend.h"
#include "splash_image.h"  // startup splash asset + cover-fit painter

#ifdef MABUR_PLAYER_HW
#include "burn_recorder.h"  // dvr.mode "burned": re-encode with the OSD burnt in
#include "drm_presenter.h"  // KMS atomic NV12 presenter, the default display path
#include "mpp_backend.h"    // MppBackend::info_changes()/errors() for --decode-only
#include "osd_palette.h"    // build_palette() for the encoder-side OSD
#endif

namespace {

std::atomic<bool> g_stop{false};
void on_signal(int) { g_stop.store(true); }

// Monotonic milliseconds. Everything the GS overlay measures -- AU arrival
// jitter, the 1 Hz video/recording marks, the recording clock -- is an
// interval, so it must not be readable from a clock the RTC can step.
uint64_t mono_ms() {
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

void usage() {
  std::fprintf(stderr,
               "usage: maburplay [-c <config.json>] [--oneshot] "
               "[--backend null|mpp]\n"
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
               "                 [--fps N] [--jit N] [--mbps N]\n"
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
               "  wait). This is the hardware decode gate (see\n"
               "  .superpowers/sdd/2026-08-02-pr-b-maburplay/task-8-brief.md).\n"
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
               "  burn_osdrej -- frames past the fps cap / encode() calls\n"
               "  that produced a packet / frames DISPLACED in the recorder's\n"
               "  mailbox, i.e. the overload signal / frames released by a\n"
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
  for (const maburplay::NalView& nal : maburplay::split_nals(data.data(), data.size())) {
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

  maburplay::HevcParams params;
  maburplay::DvrMux dvr;
  bool dvr_open = false;
  uint32_t pts_us = 0;
  const maburplay::BackendCfg bcfg;  // 1920x1080 default; tkhd/stsd metadata
                                      // only -- decode uses the SPS, not this.
  for (const std::vector<uint8_t>& au : aus) {
    const bool key = maburplay::au_is_irap(au.data(), au.size());
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
// exact GsFont/GsOverlay/gs_draw path the live overlay uses and dumps the
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
  maburplay::GsOverlay ov(font);
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

// DVR filename convention: record_%Y-%m-%d_%H-%M-%S.mp4 under dvr.dir.
//
// The stamp has ONE-SECOND resolution and both writers open "wb", so two
// recordings started inside the same second would silently truncate the
// first. That was unreachable while a run produced exactly one file; the
// record button makes it a double-tap away (kDebounceMs is 50). The
// realistic collider is BURNED mode, where this function is called
// synchronously from rec_start(), so the second name is minted within
// milliseconds of the first. The raw path is far harder to collide with --
// it only opens on the next sid-0 sync point, up to ~2 s later -- but it
// shares the same convention and the same "wb", so it gets the same
// protection. So the last stem this process emitted is remembered and a
// repeat gets "-1", "-2", ... The
// FIRST recording of any second is spelled exactly as before -- the
// /media/dvr naming is a user-facing convention -- and the disambiguation
// is driven only by string equality with the previous name, never by an
// elapsed-time threshold, so it cannot decide differently on a faster host.
std::string dvr_filename(const std::string& dir) {
  std::time_t t = std::time(nullptr);
  std::tm tmv{};
  ::localtime_r(&t, &tmv);
  char buf[64];
  std::strftime(buf, sizeof(buf), "record_%Y-%m-%d_%H-%M-%S", &tmv);
  // Main-loop-thread only (the ring sink and rec_start's
  // start_burn_if_needed), so plain statics need no synchronisation.
  static std::string last_stem;
  static unsigned dup_n = 0;
  std::string stem(buf);
  if (stem == last_stem) {
    stem += "-" + std::to_string(++dup_n);
  } else {
    last_stem = stem;
    dup_n = 0;
  }
  return dir + "/" + stem + ".mp4";
}

// Parses player_config's "WIDTHxHEIGHT@FPS" screen_mode convention; falls
// back to BackendCfg's default (1920x1080@60) if malformed.
maburplay::BackendCfg parse_screen_mode(const std::string& s) {
  maburplay::BackendCfg cfg;
  int w = 0, h = 0, fps = 0;
  if (std::sscanf(s.c_str(), "%dx%d@%d", &w, &h, &fps) == 3 && w > 0 && h > 0 && fps > 0) {
    cfg.width = w;
    cfg.height = h;
    cfg.fps = fps;
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

  std::string config_path = "/etc/maburplay.json";
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
  try {
    cfg = maburplay::load_config(config_path);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "maburplay: config: %s\n", e.what());
    return 2;
  }
  if (!backend_override.empty()) cfg.backend = backend_override;

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
                 "maburplay: backend \"%s\" not available (cross build without "
                 "MABUR_PLAYER_HW?)\n",
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
  // presenter-free per its own brief ("no presenter"): frames are counted
  // and released immediately, same as before Task 9.
#ifdef MABUR_PLAYER_HW
  std::unique_ptr<maburplay::DrmPresenter> presenter;
  if (!decode_only) {
    presenter = std::make_unique<maburplay::DrmPresenter>();
    // Declaration order is load-bearing: `backend` is declared before
    // `presenter`, so the presenter (and any frames it still holds) is
    // destroyed FIRST on unwind, releasing into a live backend. The null
    // guard covers the watchdog's recreation-failed exit path, where
    // drop_all() has already emptied the presenter.
    if (!presenter->init(cfg.screen_mode, want_osd,
                         [&backend](const maburplay::DmaFrame& f) {
                           if (backend) backend->release_frame(f);
                         })) {
      std::fprintf(stderr,
                   "maburplay: DrmPresenter init failed -- no display; frames will be "
                   "decoded and released immediately\n");
      presenter.reset();
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
      composer.set_gs(std::make_unique<maburplay::GsOverlay>(gs_font),
                      std::make_unique<maburplay::GsOverlay>(gs_font),
                      std::make_unique<maburplay::GsOverlay>(gs_font));
      std::fprintf(stderr,
                   "maburplay: gs osd on udp 127.0.0.1:%d font=%s stale_ms=%d\n",
                   gs_src->port(), cfg.osd.gs.font.c_str(), cfg.osd.gs.stale_ms);
    }
  }

  // Player-measured half of the GS overlay's inputs, recomputed once a
  // second in the main loop. maburgs cannot supply these: its fps counts AUs
  // published to the ring and still reads 60 while a wedged decoder shows a
  // frozen picture -- the exact failure the pilot has to be able to see.
  maburplay::GsPlayerState gs_ps;
  uint64_t gs_frames_at_mark = 0;  // frame_count at the last 1 Hz mark
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
  bool statvfs_warned = false;

  // Whether a recording is running RIGHT NOW. Seeded from dvr.autostart;
  // the record button flips it. Declared here because start_burn_if_needed
  // below reads it.
  bool rec_on = cfg.dvr.autostart;
  // dvr.mode "burned": the recording is produced by re-encoding decoded
  // frames with the OSD composited in by the hardware encoder, so the
  // BurnRecorder owns the file and the raw remux in the ring sink below is
  // SKIPPED. This flag -- not "the recorder actually started" -- is what
  // gates that skip: a burned run whose encoder refused to come up must
  // record nothing and say so, never quietly leave a raw file behind that
  // the user would mistake for a burned one.
  const bool burned_mode = cfg.dvr.mode == "burned";
#ifdef MABUR_PLAYER_HW
  // Declared AFTER `presenter` and `backend` so it is destroyed FIRST on
  // unwind: stop() joins the encode thread and releases the decoder buffers
  // it still holds while the decoder that owns them is still alive.
  std::unique_ptr<maburplay::BurnRecorder> burn;
  // Idempotent, and deliberately callable more than once: the display may be
  // acquired late (see the hotplug retry in the run loop), and the recorder
  // is sized from presenter->osd_back_surface(), so it cannot be built before
  // a presenter exists. Leaving it unbuilt on that path would mean a lit
  // screen, playing video, and nothing recorded.
  auto start_burn_if_needed = [&]() {
    if (burn || !burned_mode || !rec_on || !presenter) return;
    auto rec = std::make_unique<maburplay::BurnRecorder>();
    maburplay::BurnCfg bc;
    // Palette (and with it the encoder's OSD region) whenever EITHER overlay
    // is live AND the presenter really allocated an OSD surface. Gating on
    // osd_raster alone -- as this did before the GS overlay existed -- leaves
    // a GS-only topology recording pixels the encoder has no entries for,
    // i.e. a burned file with no overlay at all, indistinguishable from a
    // deliberate plain transcode. With neither overlay there is no palette
    // and set_osd() costs nothing, which IS the plain transcode.
    // MUST precede start(), which uploads it before the thread exists.
    const maburplay::Surface osd_surf = presenter->osd_back_surface();
    if ((osd_raster || composer.gs_present()) && osd_surf.pixels && osd_surf.width > 0 &&
        osd_surf.height > 0) {
      // The region is sized from the SURFACE, not from screen_mode: the
      // connector may not have offered that mode, in which case
      // DrmPresenter fell back to another one and allocated the OSD at THAT
      // size. A region built from the config would then reject every index
      // map the quantizer produces -- a recording with no OSD at all, which
      // looks exactly like a deliberate plain transcode.
      bc.osd_width = osd_surf.width;
      bc.osd_height = osd_surf.height;
      size_t n_seeds = 0;
      const uint32_t* seeds = composer.gs_present()
                                  ? maburplay::GsOverlay::palette_seeds(&n_seeds)
                                  : nullptr;
      // Seeds are needed even when the MSP atlas IS loaded: median cut over
      // the Betaflight atlas alone reproduces the atlas's own hues, and the
      // GS status colours are not among them -- they would be recorded as
      // whatever foreign colour sits nearest. With no MSP font at all the
      // empty GlyphAtlas yields a palette built from the seeds alone.
      const maburplay::GlyphAtlas empty{};
      rec->set_palette(maburplay::build_palette(
          osd_raster ? osd_font.native() : empty, seeds, n_seeds));
    }
    // Track-header FALLBACK only. The encoded picture size is the DECODED
    // frame's, latched by MppEncoder on the first frame it sees: it comes
    // from the bitstream, and screen_mode is an unrelated quantity that only
    // coincides with it on this bench.
    bc.width = bcfg.width;
    bc.height = bcfg.height;
    bc.fps_cap = cfg.dvr.burned.fps_cap;
    bc.bitrate_kbps = cfg.dvr.burned.bitrate_kbps;
    bc.fragment_ms = cfg.dvr.fragment_ms;
    // backend.get() is CHECKED, not retained (see burn_recorder.h): the
    // decode watchdog may destroy and recreate it mid-run.
    if (rec->start(bc, dvr_filename(cfg.dvr.dir), backend.get())) burn = std::move(rec);
    if (!burn) return;
    // Raw pointer, not a reference to the unique_ptr. `burn` IS reset --
    // rec_stop() tears the recorder down on a button press -- so the
    // safety comes from ordering instead: rec_stop() clears the sink
    // BEFORE it destroys the recorder, so the composer can never call
    // through a dangling pointer. Both happen on the main loop thread,
    // and MppBackend has no threads (frame_sink runs inside
    // backend->poll()), so there is nothing to synchronise against.
    maburplay::BurnRecorder* b = burn.get();
    composer.set_burn_sink([b](const maburplay::Surface& s, const maburplay::DirtyRect* r,
                               size_t n) { b->set_osd(s, r, n); });
  };
  start_burn_if_needed();
  if (rec_on && burned_mode && !burn) {
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
  if (rec_on && burned_mode) {
    std::fprintf(stderr,
                 "maburplay: dvr.mode \"burned\" needs the hardware build (mpp encoder); "
                 "NOTHING is being recorded\n");
  }
#endif

  // Counted unconditionally (cheap, backend-agnostic): --decode-only reads
  // frame_count/first-last frame timestamps directly; the normal run loop
  // just carries the extra bookkeeping for free.
  uint64_t frame_count = 0;
  bool have_first_frame = false;
  std::chrono::steady_clock::time_point t_first_frame, t_last_frame;
  // Named so the watchdog's backend-recreation path can re-wire the same
  // sink into the fresh decoder instance.
  const maburplay::VideoBackend::FrameSink frame_sink = [&](const maburplay::DmaFrame& f) {
    ++frame_count;
    const auto now = std::chrono::steady_clock::now();
    if (!have_first_frame) {
      t_first_frame = now;
      have_first_frame = true;
    }
    t_last_frame = now;
#ifdef MABUR_PLAYER_HW
    if (presenter) {
      // Burned DVR, BEFORE present(): submit() only takes an MPP buffer
      // reference (O(1), no copy, no ownership), but present() hands the
      // DmaFrame to the presenter and the frame must not be read through
      // afterwards -- so the second reader goes first, where the rule is
      // obvious. The two holders are decoupled by MPP's own refcount; the
      // presenter's ownership contract is unchanged.
      if (burn) burn->submit(f);
      presenter->present(f);
      return;
    }
#endif
    backend->release_frame(f);
  };
  const bool init_ok = backend->init(bcfg, frame_sink);
  if (!init_ok) {
    std::fprintf(stderr, "maburplay: backend \"%s\" init failed\n", cfg.backend.c_str());
    return 2;
  }

  maburplay::HevcParams params;
  maburplay::DvrMux dvr;
  bool dvr_open = false;
  // Latched so the GS overlay can tell "no file yet" (armed) from "the file
  // could not be created" (fault). Nothing else needs the distinction.
  bool dvr_open_failed = false;

  // The button's two actions. Both modes are handled here so the press
  // handler stays one line; the raw path is a flag plus a close, the
  // burned path tears the recorder down and builds a fresh one.
  auto rec_start = [&]() {
    if (rec_on) return;
    rec_on = true;
    gs_rec.reset();  // the OSD clock counts THIS file
    dvr_open_failed = false;
#ifdef MABUR_PLAYER_HW
    if (burned_mode) {
      start_burn_if_needed();
      if (!burn) {
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
    std::fprintf(stderr, "maburplay: rec: START (%s)\n", cfg.dvr.mode.c_str());
  };

  auto rec_stop = [&]() {
    if (!rec_on) return;
    rec_on = false;
    if (!burned_mode) {
      // Read BEFORE the close, and gated on dvr_open. samples() survives
      // close() and is only cleared by the next open(), so with a file in
      // hand it reports the one just sealed -- but with NO file open it
      // would report the PREVIOUS recording's count and claim a file
      // /media/dvr never gained. That window is real and ~2 s wide: the
      // raw path only opens on the next sid-0 sync point, so a quick
      // START->STOP lands inside it.
      const unsigned long long n = dvr_open ? dvr.samples() : 0;
      if (dvr_open) {
        dvr.close();
        dvr_open = false;
      }
      std::fprintf(stderr, "maburplay: rec: STOP (%llu samples)\n", n);
      return;
    }
#ifdef MABUR_PLAYER_HW
    if (burn) {
      const uint64_t n = burn->frames_encoded();
      burn->stop();
      // MUST precede burn.reset(): the composer holds a RAW pointer to the
      // recorder (see start_burn_if_needed), so the sink has to be dropped
      // while the object it points at is still alive.
      composer.set_burn_sink({});
      burn.reset();
      std::fprintf(stderr, "maburplay: rec: STOP (%llu frames)\n",
                   static_cast<unsigned long long>(n));
      return;
    }
#endif
    // There was no recorder to tear down: START was deferred (no display
    // yet) or refused (the encoder would not come up), or this is a build
    // with no encoder at all. Say so anyway -- a press that leaves NO line
    // in /tmp/maburplay.log is the worst case for a post-flight read.
    std::fprintf(stderr, "maburplay: rec: STOP (no recorder was running)\n");
  };

  uint64_t backend_submits = 0;

  // Sync-point gate for the backend feed (and DVR's hvcC collection below):
  // this live encoder's "rally" resilience mode emits exactly ONE real IRAP
  // NAL, at session start, and never again -- au_is_irap()/kFlagIdr both
  // stay false for the rest of the session (measured on hardware: neither
  // ever fires past the opening IDR). The ~2 s periodic meta.sid==0 AUs are
  // NOT IRAPs: they're parameter-set retransmission (fresh VPS/SPS/PPS)
  // bundled with an ordinary refresh picture on sid 0 (the CRIT stream),
  // while sid 1 (T0 base) and sid 3 (SVC-T enhance) carry ordinary P
  // slices in between. A decoder that attaches mid-session (the normal
  // deployment path -- the player comes up independently of the encoder)
  // has to treat sid0 as the join/cut point instead of a true IRAP: drop
  // AUs until the first sid==0 arrives, then feed everything in order.
  // Because sid0 is a refresh picture, not a clean random-access point,
  // decoders should expect concealment/errors for up to one refresh cycle
  // (~2 s) right after joining, then clean decode. Re-armed on
  // flush_before (discontinuity/reset drops whatever parameter-set state
  // the decoder had) -- same join rule applies again there. au_is_irap()
  // itself is untouched/still correct for genuinely IRAP-keyed input (see
  // --mux-annexb's real x265 stream, used by the host e2e).
  bool backend_armed = false;
  // First-ever sync-point timestamp (decode-only diagnostics: buckets
  // MppBackend's error count into "within the first 3s post-sync" vs.
  // after, per the amended gate's concealment-window allowance).
  bool t_sync_seen = false;
  std::chrono::steady_clock::time_point t_sync;
  uint64_t truncated_skipped = 0;

  // Player -> GS IDR feedback (spec 2026-08-12 decoder-idr-backchannel).
  // The latch is set by the player's OWN bookkeeping -- flush, join at a
  // non-IRAP sid0, watchdog -- and cleared when a complete IDR reaches the
  // decoder. No decoder introspection: concealed() is measured dead.
  maburplay::PlayerIdrLatch idr_latch;
  std::unique_ptr<maburgs::UdpSink> fb_sink;
  if (cfg.feedback.enable) {
    fb_sink = std::make_unique<maburgs::UdpSink>(cfg.feedback.gs_host,
                                                 cfg.feedback.gs_port);
    if (!fb_sink->ok()) {
      // Non-fatal, logged once: the same policy as the GPIO button. A player
      // that cannot reach the GS still plays video.
      std::fprintf(stderr,
                   "maburplay: feedback: cannot open %s:%d -- running without "
                   "IDR feedback\n",
                   cfg.feedback.gs_host.c_str(), cfg.feedback.gs_port);
      fb_sink.reset();
    } else {
      std::fprintf(stderr, "maburplay: feedback -> %s:%d every %d ms\n",
                   cfg.feedback.gs_host.c_str(), cfg.feedback.gs_port,
                   cfg.feedback.report_ms);
    }
  }
  uint64_t fb_last_send_ms = 0;
  auto send_feedback = [&](uint64_t now_ms) {
    if (!fb_sink) return;
    mabur::playerfb::Msg m;
    m.idr = idr_latch.want();
    m.reason = static_cast<uint8_t>(idr_latch.reason());
    m.flushes = idr_latch.count(maburplay::PlayerIdrLatch::Reason::kFlush);
    m.joins = idr_latch.count(maburplay::PlayerIdrLatch::Reason::kJoin);
    m.watchdogs = idr_latch.count(maburplay::PlayerIdrLatch::Reason::kWatchdog);
    m.episodes = idr_latch.episodes();
    char buf[192];
    const size_t n = mabur::playerfb::format(m, buf, sizeof(buf));
    if (n) fb_sink->send(reinterpret_cast<const uint8_t*>(buf), n);
    fb_last_send_ms = now_ms;
  };

  // RingClient sink: (a) DVR write (must not depend on decode health, so it
  // happens before the backend ever sees the AU), then (b) backend submit.
  auto sink = [&](maburplay::AuEvent&& ev) {
    const bool complete = (ev.meta.flags & maburgs::kRecFlagComplete) != 0;
    // DVR's join/cut signal: same sid0-is-the-sync-point reasoning as the
    // backend gate below, replacing the kFlagIdr check -- this live
    // encoder never sets it past the opening session IDR (see comment on
    // backend_armed above).
    const bool is_key = ev.meta.sid == 0;

    // GS overlay video figures, measured HERE -- at AU delivery -- and not
    // at flip: the presenter is a mailbox on a 60 Hz vsync, so flip deltas
    // are quantized to 16.67 ms multiples and a 3 ms arrival wobble is
    // invisible in them. RingClient::pump(2) is a poll() on the doorbell fd
    // with a 2 ms CEILING, not a 2 ms sample grid, so the doorbell wakes us
    // promptly and steady_clock resolves sub-millisecond here.
    gs_jitter.on_au(mono_ms());
    // Delivered bitrate is what arrived, truncated AUs included: the figure
    // answers "what is the link carrying", not "what decoded".
    gs_bytes_total += ev.au.size();
    if (complete) ++gs_complete_aus;

    // !burned_mode: in burned mode the BurnRecorder owns the recording and
    // writes the encoder's output to its own DvrMux instead. Everything
    // inside is the raw path, byte-for-byte unchanged.
    if (rec_on && !burned_mode) {
      if (!complete) {
        // Truncated base AU: DVR records complete AUs only, so this one is
        // skipped whole. No explicit fragment cut needed here: DvrMux
        // already cuts unconditionally on every key AU, so the resulting
        // decode gap is sealed off automatically the next time one arrives.
      } else {
        if (is_key) params.feed(ev.au.data(), ev.au.size());
        // `&& is_key`: a file must BEGIN at a sync point. params.complete()
        // is sticky and params is never reset, so on the SECOND and later
        // recordings of a run it is already true when the button re-arms
        // this path -- without this clause the file would open on whatever
        // AU arrived next and record it with key=false, i.e. start with P
        // slices whose references are not in the file. A no-op for the
        // first recording: params.feed() only runs on is_key, so
        // complete() cannot first become true anywhere but a key AU.
        if (!dvr_open && params.complete() && is_key) {
          const std::string path = dvr_filename(cfg.dvr.dir);
          dvr_open = dvr.open(path, params.hvcc(), bcfg.width, bcfg.height, cfg.dvr.fragment_ms);
          if (!dvr_open) {
            std::fprintf(stderr, "maburplay: dvr: cannot open %s\n", path.c_str());
            dvr_open_failed = true;
          } else {
            dvr_open_failed = false;
          }
        }
        if (dvr_open) {
          dvr.write_sample(ev.au.data(), ev.au.size(), ev.meta.pts_us, is_key);
        }
      }
    }

    if (ev.flush_before) {
      // Flush-ordering contract carried from Task 8's review:
      // MppBackend::flush()/mpi->reset() with DmaFrames still held by the
      // presenter is an unverified interaction, so every held frame MUST
      // be released back to the backend first -- present nothing until a
      // new frame arrives -- BEFORE flush() runs.
#ifdef MABUR_PLAYER_HW
      if (presenter) presenter->drop_all();
      if (burn) {
        // The recorder is the SECOND holder of decoder buffers, so it has
        // to let go here too -- same reason, same place. Then reseal the
        // recording: the next encoded frame after a discontinuity is an IDR.
        burn->drop_pending();
        burn->request_idr();
      }
#endif
      backend->flush();
      backend_armed = false;
      if (idr_latch.on_break(maburplay::PlayerIdrLatch::Reason::kFlush, mono_ms())) {
        std::fprintf(stderr, "maburplay: idr-fb: set (flush)\n");
        send_feedback(mono_ms());
      }
    }
    // Never feed a truncated AU to the decoder. The spec's original policy
    // (submit truncated base, let MPP conceal) HANGS rkvdec2 on this
    // hardware: a truncated slice declares more bitstream than exists, the
    // VPU waits for bytes that never arrive, and the kernel force-resets
    // the session ("mpp_rkvdec2 ... task timeout ... resetting") leaving
    // userspace MPP wedged. Counted; corruption washes out via the
    // encoder's rolling refresh. (The decode watchdog in the main loop is
    // the second line of defense if the VPU wedges anyway.)
    // MUST run BEFORE the arming check: a truncated sid0 would otherwise
    // arm the decoder and then discard the very parameter sets that made
    // it a sync point (review finding -- everything until the next sid0
    // would be param-less P slices, spuriously tripping the watchdog).
    if (!complete) {
      ++truncated_skipped;
      return;
    }
    if (!backend_armed) {
      if (ev.meta.sid != 0) return;
      backend_armed = true;
      // sid0 is a join/cut point, NOT an IRAP (measured: zero IRAP NALs in
      // normal operation), so the decoder resumes predicting from pictures it
      // never saw. Ask for a real one.
      if (idr_latch.on_break(maburplay::PlayerIdrLatch::Reason::kJoin, mono_ms())) {
        std::fprintf(stderr, "maburplay: idr-fb: set (join)\n");
        send_feedback(mono_ms());
      }
      if (!t_sync_seen) {
        t_sync_seen = true;
        t_sync = std::chrono::steady_clock::now();
      }
    }
    backend->submit_au(ev.au.data(), ev.au.size(), ev.meta.pts_us);
    ++backend_submits;
    // Reached only for complete AUs (truncated ones returned above), so
    // `complete` is true by construction here.
    if (idr_latch.on_au_submitted(
            true, (ev.meta.flags & mabur::framewire::kFlagIdr) != 0, mono_ms())) {
      std::fprintf(stderr, "maburplay: idr-fb: cleared after %llu ms\n",
                   static_cast<unsigned long long>(idr_latch.last_wait_ms()));
      send_feedback(mono_ms());
    }
  };

  maburplay::RingClient ring({cfg.ring_path, cfg.socket}, sink);
  if (!ring.open()) {
    std::fprintf(stderr, "maburplay: cannot open ring %s\n", cfg.ring_path.c_str());
    return 2;
  }

  if (oneshot) {
    ring.oneshot_drain();
    if (dvr_open) dvr.close();
    std::printf(
        "{\"delivered\":%llu,\"dropped_enhance_incomplete\":%llu,"
        "\"truncated_base\":%llu,\"resyncs\":%llu,\"dvr_samples\":%llu,"
        "\"dvr_fragments\":%llu,\"backend_submits\":%llu}\n",
        static_cast<unsigned long long>(ring.delivered()),
        static_cast<unsigned long long>(ring.dropped_enhance_incomplete()),
        static_cast<unsigned long long>(ring.truncated_base()),
        static_cast<unsigned long long>(ring.resyncs()), static_cast<unsigned long long>(dvr.samples()),
        static_cast<unsigned long long>(dvr.fragments()),
        static_cast<unsigned long long>(backend_submits));
    return 0;
  }

  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  if (decode_only) {
    // Hardware decode gate (task-8-brief.md, amended per the sid0-join
    // finding above): drive the backend straight off the live ring for
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
      if (!sampled_3s && t_sync_seen &&
          std::chrono::steady_clock::now() >= t_sync + std::chrono::seconds(3)) {
        sampled_3s = true;
#ifdef MABUR_PLAYER_HW
        if (auto* mpp = dynamic_cast<maburplay::MppBackend*>(backend.get())) {
          errors_at_3s = mpp->errors();
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
#ifdef MABUR_PLAYER_HW
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
    // If the 3s-post-sync mark was never reached (run ended too soon, or
    // sync never happened), errors_at_3s stays 0 and errors_after_sync_3s
    // degrades to the full total -- an honest "can't claim post-window
    // cleanliness" rather than a false pass.
    const uint64_t errors_after_sync_3s = error_count - errors_at_3s;

    if (dvr_open) dvr.close();
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

  // --fps-log bookkeeping: a once-per-second stderr line, computed off the
  // same frame_count the FrameSink above already maintains.
  auto t_last_fps_log = std::chrono::steady_clock::now();
  uint64_t frames_at_last_fps_log = 0;

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
#endif
  while (!g_stop.load()) {
    // 2 ms pump: flip events must be reaped at sub-vsync latency or the
    // mailbox frame waits for the NEXT AU burst before anyone submits it
    // (observed live as juddery ~2-3-vsync-late presentation with a
    // 100 ms pump -- the doorbell was the only wakeup source).
    ring.pump(2);
    {
      const uint64_t now_fb = mono_ms();
      if (now_fb - fb_last_send_ms >= static_cast<uint64_t>(cfg.feedback.report_ms))
        send_feedback(now_fb);
    }
    backend->poll();
    // One ioctl per iteration (~500/s, microseconds each) -- far under the
    // ~1.5 ms budget tools/bench/gs_overlay_bench.cpp polices for this loop.
    if (rec_button.is_open() && rec_button.poll(mono_ms())) {
      if (rec_on) {
        rec_stop();
      } else {
        rec_start();
      }
    }
#ifdef MABUR_PLAYER_HW
    if (presenter) presenter->poll_events();
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
        auto p = std::make_unique<maburplay::DrmPresenter>();
        // log_failures=false: this runs 86,400 times a day on a GS with no
        // screen, and /tmp is tmpfs.
        if (p->init(cfg.screen_mode, want_osd,
                    [&backend](const maburplay::DmaFrame& f) {
                      if (backend) backend->release_frame(f);
                    },
                    /*log_failures=*/false)) {
          presenter = std::move(p);
          std::fprintf(stderr, "maburplay: display acquired after %.1f s\n",
                       (now_ms - display_retry_t0_ms) / 1000.0);
          // Startup-only, and this is where that invariant is enforced for the
          // hotplug path: if this process has ever decoded a frame, the aircraft is
          // flying and a still photo on the goggles would read as a live feed. The
          // screen still lights (needs_modeset defaults true and was never cleared
          // since splash_show() was skipped, so the first present() performs the
          // full modeset that used to run before splash existed); it simply comes
          // up on video, or on nothing until video returns.
          if (!have_first_frame) show_splash(presenter.get());
          // The recorder is sized from the OSD surface, so it could not exist
          // before this moment.
          start_burn_if_needed();
          // rec_on, not burned_mode alone: a player deliberately stopped (or
          // never started -- autostart:false) has nothing to report here, and
          // saying otherwise would be a false alarm on every late display.
          if (rec_on && burned_mode && !burn)
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
      msp_ready = osd_src->poll(now_ms);
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
          gs_ps.fps = (double)(frame_count - gs_frames_at_mark) / dt;
          gs_ps.mbps = (double)(gs_bytes_total - gs_bytes_at_mark) * 8.0 / 1e6 / dt;
        }
        gs_frames_at_mark = frame_count;
        gs_bytes_at_mark = gs_bytes_total;
        gs_video_mark_ms = now_ms;
        gs_jitter.on_tick(now_ms);  // an outage must not leave a stale figure
        gs_ps.jitter_ms = gs_jitter.ms();
        gs_needs_render = true;

        // Recording state. Both the samples and the thing that would make
        // them advance differ by mode: the raw path muxes complete AUs
        // straight off the ring and does not care whether anything decodes,
        // while burned mode encodes decoded frames.
        maburplay::RecTracker::Inputs rin;
        // dvr.samples() still holds the SEALED file's count until the next
        // open(), so a stopped raw recorder would keep the OSD at REC.
        rin.samples = rec_on ? dvr.samples() : 0;
        rin.feed = gs_complete_aus;
        rin.open = dvr_open;
        // rec_on, for the same reason as the burned branch below: a failed
        // open is only a FAULT while we are supposed to be recording. Once
        // the user has stopped, a stale latch would keep the OSD red until
        // the next start (rec_start is what clears it).
        rin.broken = rec_on && dvr_open_failed;
        if (burned_mode) {
          rin.feed = frame_count;
#ifdef MABUR_PLAYER_HW
          rin.samples = burn ? burn->frames_encoded() : 0;
          rin.open = burn != nullptr;
          // Only a MISSING recorder we are supposed to have is broken. A
          // deliberately stopped one is ARMED, not FAULT.
          rin.broken = rec_on && burn == nullptr;
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
      }
    }
    // ONE composition, for both overlays, into the buffer that is back right
    // now. Not two independent render blocks: they share the back index and
    // the dirty flag, so whichever of them publishes decides what is scanned
    // out -- and a per-buffer piece of state that only the OTHER one
    // refreshes then strobes at that publish rate. osd_compose.{h,cpp} owns
    // that whole problem and is tested directly; this is only the trigger.
#ifdef MABUR_PLAYER_HW
    // Re-checked every iteration, never cached: the presenter switches the
    // OSD off mid-session after repeated commit failures, and from that point
    // osd_back_surface() is a null Surface.
    if (presenter && presenter->osd_available()) {
      const maburplay::Surface surf = presenter->osd_back_surface();
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
      if (surf.pixels) {
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
        const int idx = presenter->osd_back_index();
        if (composer.wants(in)) {
          const maburplay::OsdComposeOut out = composer.compose(idx, surf, in);
          gs_needs_render = false;
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
    }
#endif
    {
      const auto now = std::chrono::steady_clock::now();
      if (frame_count != wd_frames) {
        wd_frames = frame_count;
        wd_submits = backend_submits;
        wd_last_progress = now;
        wd_consecutive = 0;
      } else if (have_first_frame && backend_submits > wd_submits + 60 &&
                 now - wd_last_progress > std::chrono::seconds(2)) {
        // have_first_frame gate: before the decoder has EVER produced a
        // frame (cold attach waiting for the encoder's session sync), a
        // fire here would be spurious -- there is nothing to reset yet.
        ++wd_consecutive;
        std::fprintf(stderr,
                     "maburplay: decode watchdog -- %llu AUs submitted with no decoded frame "
                     "for 2 s; resetting decoder and resyncing (attempt %d)\n",
                     static_cast<unsigned long long>(backend_submits - wd_submits),
                     wd_consecutive);
#ifdef MABUR_PLAYER_HW
        if (presenter) presenter->drop_all();
        if (burn) {
          // Same pairing as flush_before: release the buffer the recorder is
          // holding before the decoder is reset or torn down, and reseal the
          // recording with an IDR once frames come back. The recorder never
          // holds a backend pointer, so the recreation path below needs
          // nothing else from it.
          burn->drop_pending();
          burn->request_idr();
        }
#endif
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
          if (!backend || !backend->init(bcfg, frame_sink)) {
            std::fprintf(stderr,
                         "maburplay: decoder recreation failed -- exiting for respawn\n");
            return 1;
          }
        } else {
          // Even a fresh context won't decode: something below us (VPU,
          // kernel, stream) needs a full process restart. The init wrapper
          // respawns us in ~1 s with a clean slate.
          std::fprintf(stderr,
                       "maburplay: decode watchdog -- recreation ineffective; exiting for "
                       "respawn\n");
          return 1;
        }
        backend_armed = false;  // resync at the next sid0 AU
        if (idr_latch.on_break(maburplay::PlayerIdrLatch::Reason::kWatchdog,
                               mono_ms())) {
          std::fprintf(stderr, "maburplay: idr-fb: set (watchdog)\n");
          send_feedback(mono_ms());
        }
        wd_submits = backend_submits;
        wd_last_progress = now;
      }
    }
    if (fps_log) {
      const auto now = std::chrono::steady_clock::now();
      const double dt = std::chrono::duration<double>(now - t_last_fps_log).count();
      if (dt >= 1.0) {
        const double fps = static_cast<double>(frame_count - frames_at_last_fps_log) / dt;
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
          char burn_fields[224] = {0};
          if (burn) {
            std::snprintf(burn_fields, sizeof(burn_fields),
                          " burn_in=%llu burn_enc=%llu burn_drop=%llu burn_flush=%llu "
                          "burn_err=%llu burn_osdrej=%llu",
                          static_cast<unsigned long long>(burn->frames_in()),
                          static_cast<unsigned long long>(burn->frames_encoded()),
                          static_cast<unsigned long long>(burn->frames_dropped()),
                          static_cast<unsigned long long>(burn->frames_flushed()),
                          static_cast<unsigned long long>(burn->encode_errors()),
                          static_cast<unsigned long long>(burn->osd_rejects()));
          }
          std::fprintf(stderr,
                       "fps-log: fps=%.1f flips/s=%.1f repl=%llu frames=%llu commit_errors=%llu "
                       "async=%s osd_screens=%llu osd_dgrams=%llu osd_commit_errors=%llu%s\n",
                       fps, static_cast<double>(flips - flips_at_last_fps_log) / dt,
                       static_cast<unsigned long long>(presenter->busy_replaced()),
                       static_cast<unsigned long long>(frame_count),
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
          if (fps < 0.5 && frame_count > 0)
            std::fprintf(stderr, "fps-log: STALL %s trunc_skip=%llu\n", ring.debug_line().c_str(),
                         static_cast<unsigned long long>(truncated_skipped));
        } else
#endif
        {
          std::fprintf(stderr, "fps-log: fps=%.1f frames=%llu\n", fps,
                       static_cast<unsigned long long>(frame_count));
        }
        t_last_fps_log = now;
        frames_at_last_fps_log = frame_count;
      }
    }
    if (ring.dead()) {
      std::fprintf(stderr, "maburplay: ring reader dead, exiting\n");
      break;
    }
  }

  if (dvr_open) dvr.close();
  return 0;
}
