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
#include "gs_overlay.h"
#include "gs_source.h"
#include "hevc_params.h"
#include "mabur/frame_wire.h"
#include "osd_font.h"
#include "osd_raster.h"
#include "osd_source.h"
#include "player_config.h"
#include "ring_client.h"
#include "video_backend.h"

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
               "[--backend null|mpp] [--no-dvr]\n"
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
               "                 [--rec recording|armed|fault|absent] "
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
               "  --no-dvr is respected as usual.\n"
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
std::string dvr_filename(const std::string& dir) {
  std::time_t t = std::time(nullptr);
  std::tm tmv{};
  ::localtime_r(&t, &tmv);
  char buf[64];
  std::strftime(buf, sizeof(buf), "record_%Y-%m-%d_%H-%M-%S.mp4", &tmv);
  return dir + "/" + buf;
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
        // An unrecognised state is rejected rather than folded into
        // kAbsent: kAbsent renders NOTHING, so a typo would silently
        // produce a dump missing the whole recording field and still look
        // like a successful render.
        const std::string r = argv[++i];
        if (r == "recording") {
          ps.rec.kind = maburplay::RecState::Kind::kRecording;
        } else if (r == "armed") {
          ps.rec.kind = maburplay::RecState::Kind::kArmed;
        } else if (r == "fault") {
          ps.rec.kind = maburplay::RecState::Kind::kFault;
        } else if (r == "absent") {
          ps.rec.kind = maburplay::RecState::Kind::kAbsent;
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
  bool no_dvr = false;
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
    } else if (a == "--no-dvr") {
      no_dvr = true;
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
  if (no_dvr) cfg.dvr.enabled = false;

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
  std::unique_ptr<maburplay::GsOverlay> gs_overlay;
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
#ifdef MABUR_PLAYER_HW
  // One ShadowGrid per OSD buffer, indexed by the presenter's back index:
  // the two buffers hold different pixels, so a shared shadow would suppress
  // the redraw of cells that are current in one buffer and stale in the
  // other. Owned here because the presenter deliberately does not own them.
  maburplay::ShadowGrid osd_shadow[2];
  bool osd_blanked = false;
  // A THIRD shadow, for the burned DVR's index map. It cannot share the two
  // above: those track one DRM buffer each and so report the change since
  // two renders ago, whereas the index map is refreshed on every render.
  // Hoisted out of the loop with its rect list so neither allocates in
  // steady state.
  maburplay::ShadowGrid burn_shadow;
  std::vector<maburplay::DirtyRect> burn_dirty;
#endif

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
      gs_overlay = std::make_unique<maburplay::GsOverlay>(gs_font);
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
  uint64_t gs_frames_at_mark = 0;   // frame_count at the last 1 Hz mark
  uint64_t gs_bytes_at_mark = 0;    // gs_bytes_total at the last 1 Hz mark
  uint64_t gs_bytes_total = 0;      // running AU byte total, summed at delivery
  uint64_t gs_video_mark_ms = 0;    // 0 = no mark yet, so the first tick only seeds
  uint64_t gs_last_au_ms = 0;
  double gs_last_au_interval_ms = -1.0;
  double gs_jitter_ms = 0.0;
  // Recording state, recomputed at the same 1 Hz from the DVR's own counters.
  uint64_t dvr_open_ms = 0;          // steady_clock ms when the file was opened
  uint64_t rec_samples_at_check = 0;
  uint64_t rec_feed_at_check = 0;  // AUs (raw) or decoded frames (burned)
  uint64_t rec_stall_since_ms = 0;
  bool statvfs_warned = false;

  // dvr.mode "burned": the recording is produced by re-encoding decoded
  // frames with the OSD composited in by the hardware encoder, so the
  // BurnRecorder owns the file and the raw remux in the ring sink below is
  // SKIPPED. This flag -- not "the recorder actually started" -- is what
  // gates that skip: a burned run whose encoder refused to come up must
  // record nothing and say so, never quietly leave a raw file behind that
  // the user would mistake for a burned one.
  const bool burned_mode = cfg.dvr.enabled && cfg.dvr.mode == "burned";
#ifdef MABUR_PLAYER_HW
  // Declared AFTER `presenter` and `backend` so it is destroyed FIRST on
  // unwind: stop() joins the encode thread and releases the decoder buffers
  // it still holds while the decoder that owns them is still alive.
  std::unique_ptr<maburplay::BurnRecorder> burn;
  if (burned_mode && presenter) {
    auto rec = std::make_unique<maburplay::BurnRecorder>();
    maburplay::BurnCfg bc;
    // Palette (and with it the encoder's OSD region) ONLY when the OSD path
    // is actually live -- osd_raster is non-null exactly when osd.enable is
    // set, the font loaded AND the MSP socket opened -- AND the presenter
    // really allocated an OSD surface. Otherwise no palette at all: a burned
    // recording with no overlay is a legal configuration (a plain
    // transcode), not an error, and set_osd() then costs nothing.
    // MUST precede start(), which uploads it before the thread exists.
    const maburplay::Surface osd_surf = presenter->osd_back_surface();
    if (osd_raster && osd_surf.pixels && osd_surf.width > 0 && osd_surf.height > 0) {
      // The region is sized from the SURFACE, not from screen_mode: the
      // connector may not have offered that mode, in which case
      // DrmPresenter fell back to another one and allocated the OSD at THAT
      // size. A region built from the config would then reject every index
      // map the quantizer produces -- a recording with no OSD at all, which
      // looks exactly like a deliberate plain transcode.
      bc.osd_width = osd_surf.width;
      bc.osd_height = osd_surf.height;
      rec->set_palette(maburplay::build_palette(osd_font.native()));
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
    if (rec->start(bc, dvr_filename(cfg.dvr.dir), backend.get())) {
      burn = std::move(rec);
      // The GS overlay's recording clock. Started here rather than at the
      // first encoded frame because that is when the pilot armed it; the
      // block still reads ARMED until frames actually appear.
      dvr_open_ms = mono_ms();
    }
  }
  if (burned_mode && !burn) {
    // Wrong backend, no presenter (--decode-only), or an encoder that would
    // not init. BurnRecorder::start() already logged the specific reason.
    std::fprintf(stderr,
                 "maburplay: dvr.mode \"burned\" requested but the recorder is not "
                 "running -- NOTHING is being recorded (no raw fallback: it would look "
                 "like a burned file)\n");
  }
#else
  if (burned_mode) {
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
    {
      const uint64_t now_ms = mono_ms();
      if (gs_last_au_ms != 0) {
        const double iv = (double)(now_ms - gs_last_au_ms);
        if (iv > 1000.0) {
          // A stall is a stall, not jitter: folding a link outage into the
          // EMA would leave a huge number sitting on the OSD for the next
          // ~16 AUs after video came back.
          gs_last_au_interval_ms = -1.0;
          gs_jitter_ms = 0.0;
        } else if (gs_last_au_interval_ms >= 0.0) {
          const double d = iv > gs_last_au_interval_ms ? iv - gs_last_au_interval_ms
                                                       : gs_last_au_interval_ms - iv;
          gs_jitter_ms += (d - gs_jitter_ms) / 16.0;
          gs_last_au_interval_ms = iv;
        } else {
          gs_last_au_interval_ms = iv;  // first interval after a stall: no delta yet
        }
      }
      gs_last_au_ms = now_ms;
      // Delivered bitrate is what arrived, truncated AUs included: the
      // figure answers "what is the link carrying", not "what decoded".
      gs_bytes_total += ev.au.size();
    }

    // !burned_mode: in burned mode the BurnRecorder owns the recording and
    // writes the encoder's output to its own DvrMux instead. Everything
    // inside is the raw path, byte-for-byte unchanged.
    if (cfg.dvr.enabled && !burned_mode) {
      if (!complete) {
        // Truncated base AU: DVR records complete AUs only, so this one is
        // skipped whole. No explicit fragment cut needed here: DvrMux
        // already cuts unconditionally on every key AU, so the resulting
        // decode gap is sealed off automatically the next time one arrives.
      } else {
        if (is_key) params.feed(ev.au.data(), ev.au.size());
        if (!dvr_open && params.complete()) {
          const std::string path = dvr_filename(cfg.dvr.dir);
          dvr_open = dvr.open(path, params.hvcc(), bcfg.width, bcfg.height, cfg.dvr.fragment_ms);
          if (!dvr_open) {
            std::fprintf(stderr, "maburplay: dvr: cannot open %s\n", path.c_str());
            dvr_open_failed = true;
          } else {
            dvr_open_failed = false;
            dvr_open_ms = mono_ms();  // the GS overlay's recording clock
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
      if (!t_sync_seen) {
        t_sync_seen = true;
        t_sync = std::chrono::steady_clock::now();
      }
    }
    backend->submit_au(ev.au.data(), ev.au.size(), ev.meta.pts_us);
    ++backend_submits;
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
#ifdef MABUR_PLAYER_HW
  // Layout is deferred to the first iteration that sees a real surface: its
  // size is the DRM buffer's, which DrmPresenter may have allocated at a
  // mode the connector offered rather than the one cfg.screen_mode asked
  // for. Laying out against the config would misplace every field.
  bool gs_laid_out = false;
  std::vector<maburplay::DirtyRect> gs_dirty;
  // The OSD is DOUBLE buffered and GsOverlay keeps ONE shadow, so a field
  // redrawn into the buffer that is back right now is missing from the
  // other one -- which becomes back again after the next commit, and would
  // then show the value from two renders ago. gs_owed[i] is the set of
  // boxes buffer i has not been given yet; they are repainted (at current
  // values) the next time that buffer is drawn into. Invalidating on every
  // swap instead would make every render a full repaint of every field.
  //
  // KNOWN RESIDUAL: this covers value changes, which is every render bar
  // one. It does NOT cover a card DISAPPEARING -- GsOverlay clears the
  // vacated row itself, inside the one update() that observes the new
  // count, and repaint_intersecting cannot clear (it is gated on active
  // fields), so the other buffer keeps that row lit. Closing it properly
  // means giving GsOverlay a per-buffer shadow the way OsdRaster has one.
  std::vector<maburplay::DirtyRect> gs_owed[2];
#endif
  while (!g_stop.load()) {
    // 2 ms pump: flip events must be reaped at sub-vsync latency or the
    // mailbox frame waits for the NEXT AU burst before anyone submits it
    // (observed live as juddery ~2-3-vsync-late presentation with a
    // 100 ms pump -- the doorbell was the only wakeup source).
    ring.pump(2);
    backend->poll();
#ifdef MABUR_PLAYER_HW
    if (presenter) presenter->poll_events();
#endif
    // OSD: drain the UDP intake every iteration whether or not anything can
    // be drawn (the counters stay honest, and an undrained socket would just
    // fill), then repaint only when a fresh complete screen arrived.
    if (osd_src) {
      const uint64_t now_ms = (uint64_t)std::chrono::duration_cast<std::chrono::milliseconds>(
                                  std::chrono::steady_clock::now().time_since_epoch())
                                  .count();
      const bool ready = osd_src->poll(now_ms);
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
#ifdef MABUR_PLAYER_HW
      // Re-checked every iteration, never cached: the presenter switches the
      // OSD off mid-session if a commit carrying it fails repeatedly, and
      // from that point osd_back_surface() is a null Surface. (OsdRaster
      // tolerates that too, but there is nothing to publish.)
      if (presenter && presenter->osd_available()) {
        if (ready) {
          const int idx = presenter->osd_back_index();
          const maburplay::Surface surf = presenter->osd_back_surface();
          osd_raster->draw(osd_src->screen(), surf, &osd_shadow[idx]);
          // Recording tracks the screen. BEFORE osd_publish(), which swaps
          // the pair -- afterwards osd_back_surface() is the OTHER buffer.
          if (burn) {
            // burn_shadow, NOT osd_shadow[idx]: the draw shadows are per DRM
            // buffer and describe the change since two renders ago, while
            // the burn's index map is updated on every render. See
            // OsdRaster::diff(). Without this the map goes silently stale on
            // any cell that flips back within one render (X -> Y -> X).
            osd_raster->diff(osd_src->screen(), surf, &burn_shadow, &burn_dirty);
            // Empty => nothing changed => nothing to publish. A full
            // re-quantize runs only when diff() says "whole surface".
            if (!burn_dirty.empty()) burn->set_osd(surf, burn_dirty.data(), burn_dirty.size());
          }
          presenter->osd_publish();
          osd_blanked = false;
        } else if (!osd_blanked && osd_src->stale(now_ms)) {
          const int idx = presenter->osd_back_index();
          const maburplay::Surface surf = presenter->osd_back_surface();
          osd_raster->clear(surf, &osd_shadow[idx]);
          if (burn) {
            burn_shadow = maburplay::ShadowGrid{};  // next diff is a full one
            burn->set_osd(surf);                    // blank the burn too (full)
          }
          presenter->osd_publish();
          osd_blanked = true;
          std::fprintf(stderr, "maburplay: osd blanked after %d ms without a snapshot\n",
                       cfg.osd.stale_ms);
        }
      }
#else
      (void)ready;  // host build: no presenter, nothing to draw on
#endif
    }
    // GS link-status overlay. Drained every iteration whether or not
    // anything can be drawn -- the counters stay honest and an undrained
    // socket would just fill -- then repainted when one of its three inputs
    // moved.
    if (gs_src) {
      const uint64_t now_ms = mono_ms();
      if (gs_src->poll(now_ms)) gs_needs_render = true;
      const bool gs_stale = gs_src->stale(now_ms);
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
        gs_ps.jitter_ms = gs_jitter_ms;
        gs_needs_render = true;

        // Recording state. `samples` and the thing that feeds them differ
        // by mode: the raw path muxes AUs straight off the ring and does not
        // care whether anything decodes, while burned mode encodes decoded
        // frames.
        uint64_t samples = dvr.samples();
        uint64_t feed = ring.delivered();
        bool open = dvr_open;
        bool broken = dvr_open_failed;
        if (burned_mode) {
          feed = frame_count;
#ifdef MABUR_PLAYER_HW
          samples = burn ? burn->frames_encoded() : 0;
          open = burn != nullptr;
          broken = burn == nullptr;
#else
          samples = 0;
          open = false;
          broken = true;  // burned mode needs the hardware build; already logged
#endif
        }

        if (!cfg.dvr.enabled) {
          gs_ps.rec.kind = maburplay::RecState::Kind::kAbsent;
          gs_ps.rec.elapsed_s = 0;
        } else if (broken) {
          // The recorder could not be brought up at all. Whichever path it
          // was has already logged why; this is the pilot-visible half.
          gs_ps.rec.kind = maburplay::RecState::Kind::kFault;
        } else if (!open) {
          gs_ps.rec.kind = maburplay::RecState::Kind::kArmed;
        } else {
          // Either progress or nothing to make progress FROM resets the
          // stall clock. Without the second half a link outage -- no AUs,
          // so no samples -- would read as a recording fault, which is a
          // lie about the one subsystem still working.
          if (samples != rec_samples_at_check || feed == rec_feed_at_check)
            rec_stall_since_ms = now_ms;
          rec_samples_at_check = samples;
          rec_feed_at_check = feed;
          const bool stalled = samples > 0 && now_ms - rec_stall_since_ms >= 3000;

          bool low_space = false;
          struct statvfs vfs {};
          if (::statvfs(cfg.dvr.dir.c_str(), &vfs) == 0) {
            const uint64_t free_bytes = (uint64_t)vfs.f_bavail * (uint64_t)vfs.f_frsize;
            low_space = free_bytes < 64ull * 1024 * 1024;
          } else if (!statvfs_warned) {
            // A failed syscall is not evidence of a full disk, so the
            // recording state is left alone. Said once; retried silently.
            statvfs_warned = true;
            std::fprintf(stderr,
                         "maburplay: gs osd: statvfs(%s) failed (%s) -- recording free-space "
                         "checks will be skipped whenever it keeps failing\n",
                         cfg.dvr.dir.c_str(), std::strerror(errno));
          }

          if (stalled || low_space) {
            gs_ps.rec.kind = maburplay::RecState::Kind::kFault;
          } else if (samples == 0) {
            // Open but nothing written yet. RECORDING must never show while
            // no bytes are moving.
            gs_ps.rec.kind = maburplay::RecState::Kind::kArmed;
          } else {
            gs_ps.rec.kind = maburplay::RecState::Kind::kRecording;
            gs_ps.rec.elapsed_s = dvr_open_ms ? (int)((now_ms - dvr_open_ms) / 1000) : 0;
          }
        }
      }

#ifdef MABUR_PLAYER_HW
      bool gs_disable = false;
      // Re-checked every iteration, never cached: the presenter switches the
      // OSD off mid-session after repeated commit failures, and from that
      // point osd_back_surface() is a null Surface.
      if (gs_overlay && presenter && presenter->osd_available()) {
        const maburplay::Surface surf = presenter->osd_back_surface();
        if (surf.pixels && !gs_laid_out) {
          std::string err;
          if (!gs_overlay->layout(surf.width, surf.height, &err)) {
            std::fprintf(stderr, "maburplay: gs osd disabled -- %s\n", err.c_str());
            gs_overlay.reset();
            gs_disable = true;
          } else {
            gs_laid_out = true;
          }
        }
        if (gs_laid_out && gs_needs_render && surf.pixels) {
          gs_needs_render = false;
          const int idx = presenter->osd_back_index();
          gs_dirty.clear();
          gs_overlay->update(gs_src->snapshot(), gs_stale, gs_ps, surf, &gs_dirty);
          // Bring this buffer forward over the renders it was not back for.
          if (!gs_owed[idx].empty()) {
            gs_overlay->repaint_intersecting(gs_owed[idx].data(), gs_owed[idx].size(), surf,
                                             &gs_dirty);
            gs_owed[idx].clear();
          }
          if (!gs_dirty.empty()) {
            std::vector<maburplay::DirtyRect>& other = gs_owed[idx ^ 1];
            other.insert(other.end(), gs_dirty.begin(), gs_dirty.end());
            // Collapse instead of growing without bound when the other
            // buffer never comes round -- no video means no commit, so no
            // swap. One rect over every field box repaints all of them.
            if ((int)other.size() > gs_overlay->field_count())
              other.assign(1, gs_overlay->bounds());
            if (burn) burn->set_osd(surf, gs_dirty.data(), gs_dirty.size());
            presenter->osd_publish();
          }
        }
      }
      // Deferred to here so nothing above dereferences a reset gs_src.
      if (gs_disable) gs_src.reset();
#endif
    }
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
