// maburd — drone-side daemon: reads whole encoded frames off the in-process
// venc core's frame-shm ring (or, in --dry-run, a fixture file), runs them
// through the UEP/FEC pipeline, and hands radio-bound bodies to the
// adaptive-link-controlled RadioTx. A parallel agent thread runs RcAgent against inbound RC
// frames + periodic radio-health ticks, publishing AppliedOp changes the hot
// path picks up via a lock-free shared_ptr handoff.
//
// Two modes:
//   maburd -c /etc/mabur.toml                     — real mode (devourer USB radio)
//   maburd -c cfg.toml --dry-run --in F --out F [--rc-in F]  — file-driven, no radio
//
// Dry-run is the tested path (see tests/fixtures/frame_stream.bin smoke test);
// real mode must compile and be structurally sound but is not exercised here
// (no bench hardware in this environment).
#include <cstdint>
#include <cstdio>
#include <exception>
#include <string>
#include <vector>

#include "build_ident.h"
#include "config.h"
#include "config_migrate.h"
#include "dry_run.h"
#include "real_mode.h"
#include "mabur/channel_file.h"
#include "mabur/channel_set.h"
#include "mabur/link_key.h"

#if defined(MABUR_DRY_RUN_ONLY)
// Not used; real mode is always compiled in, guarded at runtime by --dry-run
// so the same binary runs (in dry-run) on a machine with no dongle attached.
#endif

namespace {

using namespace mabur;
using namespace mabur::maburd;

// ---------------------------------------------------------------------------
// CLI
// ---------------------------------------------------------------------------

void print_usage(const char* argv0) {
  std::fprintf(stderr,
               "usage: %s -c <config.toml> [--dry-run --in <file> --out <file> [--rc-in <file>]]\n"
               "       %s --migrate-config <in.toml> <out.toml>\n"
               "       %s --identify\n",
               argv0, argv0, argv0);
}

}  // namespace

int main(int argc, char** argv) {
  // Runs and exits before anything else: no radio, MI or USB is touched.
  if (argc == 4 && std::string(argv[1]) == "--migrate-config")
    return mabur::migrate_config_cli(argv[2], argv[3]);

  if (argc == 2 && std::string(argv[1]) == "--identify") {
    std::printf("version=%s board=%s\n", mabur::build_release(), mabur::build_board());
    return 0;
  }

  std::string cfg_path;
  bool dry_run = false;
  std::string in_path, out_path, rc_in_path;
  std::string msp_in_path, msp_out_path;

  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "-c" && i + 1 < argc) {
      cfg_path = argv[++i];
    } else if (a == "--dry-run") {
      dry_run = true;
    } else if (a == "--in" && i + 1 < argc) {
      in_path = argv[++i];
    } else if (a == "--out" && i + 1 < argc) {
      out_path = argv[++i];
    } else if (a == "--rc-in" && i + 1 < argc) {
      rc_in_path = argv[++i];
    } else if (a == "--msp-in" && i + 1 < argc) {
      msp_in_path = argv[++i];
    } else if (a == "--msp-out" && i + 1 < argc) {
      msp_out_path = argv[++i];
    } else if (a == "-h" || a == "--help") {
      print_usage(argv[0]);
      return 0;
    } else {
      std::fprintf(stderr, "unknown argument: %s\n", a.c_str());
      print_usage(argv[0]);
      return 1;
    }
  }

  if (cfg_path.empty()) {
    print_usage(argv[0]);
    return 1;
  }

  Config cfg;
  std::vector<std::string> defaulted;
  try {
    cfg = load_config(cfg_path, &defaulted);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
  if (!defaulted.empty()) {
    std::fprintf(stderr, "config: %zu key(s) defaulted:\n", defaulted.size());
    for (const std::string& d : defaulted)
      std::fprintf(stderr, "  %s\n", d.c_str());
  }

  // Auto channel set: park on the remembered member, else the first. A
  // missing/garbage/non-member file is silently the first member -- nothing
  // to configure, nothing to repair.
  const auto remembered_ch = mabur::read_channel_file(mabur::kDroneChannelFile);
  const bool use_remembered =
      remembered_ch && mabur::channel_set_member(cfg.radio.channels, *remembered_ch);
  const uint8_t start_ch = use_remembered ? *remembered_ch : cfg.radio.channels.front();
  {
    std::string set_str;
    for (size_t i = 0; i < cfg.radio.channels.size(); ++i) {
      if (i) set_str += ",";
      set_str += std::to_string(cfg.radio.channels[i]);
    }
    std::fprintf(stderr, "maburd: channel set [%s], parking on %u%s\n", set_str.c_str(),
                 static_cast<unsigned>(start_ch), use_remembered ? " (remembered)" : "");
  }

  std::fprintf(stderr,
               "fec: symbol_size=[%d,%d] bpb=[%d,%d] window=%d\n",
               cfg.fec.symbol_size[0], cfg.fec.symbol_size[1],
               cfg.fec.blocks_per_body[0], cfg.fec.blocks_per_body[1],
               cfg.fec.window);

  std::fprintf(stderr, "link: key %s (%s)\n",
               mabur::key_fingerprint(cfg.link.key).c_str(), cfg.link.key_source.c_str());
  if (cfg.link.key_is_default)
    std::fprintf(stderr, "link: DEFAULT key in use -- any default install can control this drone; "
                         "see docs/deploy.md 'Pairing'\n");

  if (dry_run) {
    if (in_path.empty() || out_path.empty()) {
      std::fprintf(stderr, "error: --dry-run requires --in and --out\n");
      print_usage(argv[0]);
      return 1;
    }
    return run_dry_run(cfg, start_ch, in_path, out_path, rc_in_path, msp_in_path, msp_out_path);
  }

  return run_real_mode(cfg, start_ch, cfg_path);
}
