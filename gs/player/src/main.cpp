// maburplay -- GS native player: AU ring -> VideoBackend, with fMP4 DVR.
// Host builds only ever link the null backend (this task); MppBackend /
// DrmPresenter are cross-only (MABUR_PLAYER_HW, Task 7/8).
#include <atomic>
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
#include "hevc_params.h"
#include "mabur/frame_wire.h"
#include "player_config.h"
#include "ring_client.h"
#include "video_backend.h"

#ifdef MABUR_PLAYER_HW
#include "drm_presenter.h"  // KMS atomic NV12 presenter, the default display path
#include "mpp_backend.h"    // MppBackend::info_changes()/errors() for --decode-only
#endif

namespace {

std::atomic<bool> g_stop{false};
void on_signal(int) { g_stop.store(true); }

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
               "\n"
               "--decode-only --seconds N: drive the backend straight off\n"
               "  the ring for N seconds with no presenter, counting frames\n"
               "  via the FrameSink; prints one line of stats JSON\n"
               "  {\"frames\":N,\"fps\":X,\"fps_active\":X,\"info_changes\":N,\n"
               "   \"errors\":N,\"errors_after_sync_3s\":N} and exits. fps is\n"
               "  over the whole window; fps_active is over first-to-last\n"
               "  decoded frame only (excludes the cold-attach/sid0-join\n"
               "  wait). This is the hardware decode gate (see\n"
               "  .superpowers/sdd/2026-08-02-pr-b-maburplay/task-8-brief.md).\n"
               "  --no-dvr is respected as usual.\n"
               "\n"
               "--fps-log: normal (non-decode-only) run only -- once per\n"
               "  second, prints \"fps-log: fps=X frames=N "
               "commit_errors=N async=on|off|probing\\n\" to stderr. Under\n"
               "  MABUR_PLAYER_HW with the DrmPresenter display path active;\n"
               "  a no-op flag otherwise (host/null-backend builds just log\n"
               "  fps/frames with no presenter fields).\n");
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
    if (!presenter->init(cfg.screen_mode, [&backend](const maburplay::DmaFrame& f) {
          backend->release_frame(f);
        })) {
      std::fprintf(stderr,
                   "maburplay: DrmPresenter init failed -- no display; frames will be "
                   "decoded and released immediately\n");
      presenter.reset();
    }
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

    if (cfg.dvr.enabled) {
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
#endif
      backend->flush();
      backend_armed = false;
    }
    if (!backend_armed) {
      if (ev.meta.sid != 0) return;
      backend_armed = true;
      if (!t_sync_seen) {
        t_sync_seen = true;
        t_sync = std::chrono::steady_clock::now();
      }
    }
    // Never feed a truncated AU to the decoder. The spec's original policy
    // (submit truncated base, let MPP conceal) HANGS rkvdec2 on this
    // hardware: a truncated slice declares more bitstream than exists, the
    // VPU waits for bytes that never arrive, and the kernel force-resets
    // the session ("mpp_rkvdec2 ... task timeout ... resetting") leaving
    // userspace MPP wedged. Counted; corruption washes out via rally's
    // rolling refresh. (The decode watchdog in the main loop is the
    // second line of defense if the VPU wedges anyway.)
    if (!complete) {
      ++truncated_skipped;
      return;
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
#ifdef MABUR_PLAYER_HW
    if (auto* mpp = dynamic_cast<maburplay::MppBackend*>(backend.get())) {
      info_change_count = mpp->info_changes();
      error_count = mpp->errors();
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
        "\"errors\":%llu,\"errors_after_sync_3s\":%llu}\n",
        static_cast<unsigned long long>(frame_count), fps, fps_active,
        static_cast<unsigned long long>(info_change_count),
        static_cast<unsigned long long>(error_count),
        static_cast<unsigned long long>(errors_after_sync_3s));
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
    {
      const auto now = std::chrono::steady_clock::now();
      if (frame_count != wd_frames) {
        wd_frames = frame_count;
        wd_submits = backend_submits;
        wd_last_progress = now;
        wd_consecutive = 0;
      } else if (backend_submits > wd_submits + 60 &&
                 now - wd_last_progress > std::chrono::seconds(2)) {
        ++wd_consecutive;
        std::fprintf(stderr,
                     "maburplay: decode watchdog -- %llu AUs submitted with no decoded frame "
                     "for 2 s; resetting decoder and resyncing (attempt %d)\n",
                     static_cast<unsigned long long>(backend_submits - wd_submits),
                     wd_consecutive);
#ifdef MABUR_PLAYER_HW
        if (presenter) presenter->drop_all();
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
          std::fprintf(stderr,
                       "fps-log: fps=%.1f flips/s=%.1f repl=%llu frames=%llu commit_errors=%llu "
                       "async=%s\n",
                       fps, static_cast<double>(flips - flips_at_last_fps_log) / dt,
                       static_cast<unsigned long long>(presenter->busy_replaced()),
                       static_cast<unsigned long long>(frame_count),
                       static_cast<unsigned long long>(presenter->commit_errors()),
                       !presenter->async_probed() ? "probing"
                       : presenter->async_flip_active() ? "on" : "off");
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
