// maburplay -- GS native player: AU ring -> VideoBackend, with fMP4 DVR.
// Host builds only ever link the null backend (this task); MppBackend /
// DrmPresenter are cross-only (MABUR_PLAYER_HW, Task 7/8).
#include <atomic>
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

namespace {

std::atomic<bool> g_stop{false};
void on_signal(int) { g_stop.store(true); }

void usage() {
  std::fprintf(stderr,
               "usage: maburplay [-c <config.json>] [--oneshot] "
               "[--backend null|mpp] [--no-dvr]\n"
               "       maburplay --mux-annexb <in.265> <out.mp4>\n"
               "           (test support: mux a raw Annex-B HEVC elementary\n"
               "            stream straight through HevcParams+DvrMux to an\n"
               "            fMP4 file, bypassing the ring/backend entirely --\n"
               "            used by the host e2e's real-HEVC decode gate)\n");
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
  maburplay::VideoBackend* const backend_ptr = backend.get();
  const bool init_ok = backend->init(
      bcfg, [backend_ptr](const maburplay::DmaFrame& f) { backend_ptr->release_frame(f); });
  if (!init_ok) {
    std::fprintf(stderr, "maburplay: backend \"%s\" init failed\n", cfg.backend.c_str());
    return 2;
  }

  maburplay::HevcParams params;
  maburplay::DvrMux dvr;
  bool dvr_open = false;
  uint64_t backend_submits = 0;

  // RingClient sink: (a) DVR write (must not depend on decode health, so it
  // happens before the backend ever sees the AU), then (b) backend submit.
  auto sink = [&](maburplay::AuEvent&& ev) {
    const bool complete = (ev.meta.flags & maburgs::kRecFlagComplete) != 0;
    const bool is_key = (ev.meta.flags & mabur::framewire::kFlagIdr) != 0;

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

    if (ev.flush_before) backend_ptr->flush();
    backend_ptr->submit_au(ev.au.data(), ev.au.size(), ev.meta.pts_us);
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

  while (!g_stop.load()) {
    ring.pump(100);
    backend_ptr->poll();
    if (ring.dead()) {
      std::fprintf(stderr, "maburplay: ring reader dead, exiting\n");
      break;
    }
  }

  if (dvr_open) dvr.close();
  return 0;
}
