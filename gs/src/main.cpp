// maburgs — mabur ground station daemon.
// Plan 1 scope: the dry-run datapath (frame file -> aggregator -> RTP out).
// The radio front-ends and the control plane land with the control-plane
// plan; until then real-radio mode exits with an error.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>

#include "aggregator.h"
#include "config.h"
#include "frame_file_source.h"
#include "udp_sink.h"

namespace {

void usage() {
  std::fprintf(stderr,
               "usage: maburgs -c <config.json> --dry-run --in <frames.bin>\n"
               "               [--cards N] [--drop-pct P] [--seed S] [--out-rtp <file>]\n");
}

struct RtpFileOut {
  FILE* f = nullptr;
  uint64_t written = 0;
  bool open(const char* path) { return (f = fopen(path, "wb")) != nullptr; }
  void write(const std::vector<uint8_t>& pkt) {
    const uint8_t len[2] = {static_cast<uint8_t>(pkt.size() & 0xFF),
                            static_cast<uint8_t>(pkt.size() >> 8)};
    fwrite(len, 1, 2, f);
    fwrite(pkt.data(), 1, pkt.size(), f);
    ++written;
  }
  ~RtpFileOut() { if (f) fclose(f); }
};

}  // namespace

int main(int argc, char** argv) {
  std::string config_path = "/etc/maburgs.json";
  std::string in_path, out_rtp_path;
  bool dry_run = false;
  maburgs::FrameFileSource::Options src_opt;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "-c" && i + 1 < argc) config_path = argv[++i];
    else if (a == "--dry-run") dry_run = true;
    else if (a == "--in" && i + 1 < argc) in_path = argv[++i];
    else if (a == "--cards" && i + 1 < argc) src_opt.cards = std::atoi(argv[++i]);
    else if (a == "--drop-pct" && i + 1 < argc) src_opt.drop_pct = std::atoi(argv[++i]);
    else if (a == "--seed" && i + 1 < argc) src_opt.seed = static_cast<uint32_t>(std::atol(argv[++i]));
    else if (a == "--out-rtp" && i + 1 < argc) out_rtp_path = argv[++i];
    else if (a == "-h" || a == "--help") { usage(); return 0; }
    else { std::fprintf(stderr, "error: unknown arg %s\n", a.c_str()); usage(); return 2; }
  }
  if (!dry_run) {
    std::fprintf(stderr, "error: radio mode lands with the control-plane plan; use --dry-run\n");
    return 2;
  }
  if (in_path.empty()) { usage(); return 2; }

  maburgs::Config cfg;
  try {
    cfg = maburgs::load_config(config_path);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 2;
  }

  maburgs::FrameFileSource src(in_path, src_opt);
  if (!src.ok()) {
    std::fprintf(stderr, "error: cannot read %s\n", in_path.c_str());
    return 2;
  }

  const int n_cards = src_opt.cards;
  maburgs::Aggregator agg(cfg.uep_layers(),
                          static_cast<uint64_t>(cfg.fec.block_max_age_ms), n_cards);
  RtpFileOut file_out;
  std::unique_ptr<maburgs::UdpSink> udp;
  if (!out_rtp_path.empty()) {
    if (!file_out.open(out_rtp_path.c_str())) {
      std::fprintf(stderr, "error: cannot write %s\n", out_rtp_path.c_str());
      return 2;
    }
    agg.set_rtp_sink([&](const mabur::DecodedRtp& r) { file_out.write(r.pkt); });
  } else {
    udp = std::make_unique<maburgs::UdpSink>(cfg.video_out.host, cfg.video_out.port);
    if (!udp->ok())
      std::fprintf(stderr, "warning: video_out %s:%d unusable; decoding anyway\n",
                   cfg.video_out.host.c_str(), cfg.video_out.port);
    agg.set_rtp_sink([&](const mabur::DecodedRtp& r) {
      udp->send(r.pkt.data(), r.pkt.size());
    });
  }
  uint64_t rc_frames = 0;
  agg.set_rc_sink([&](uint8_t, const std::vector<uint8_t>&, uint64_t) { ++rc_frames; });

  uint64_t last_ms = 0;
  while (auto m = src.next()) {
    agg.on_rx_body(*m);
    const uint64_t now_ms = m->mono_us / 1000;
    if (now_ms >= last_ms + 1000) {
      agg.poll(now_ms);
      last_ms = now_ms;
    }
  }
  // Final expiry so unrecoverable blocks are accounted before the report.
  agg.poll(last_ms + static_cast<uint64_t>(cfg.fec.block_max_age_ms) + 1);

  std::fprintf(stderr, "frames=%llu dropped=%llu malformed=%llu rc=%llu\n",
               static_cast<unsigned long long>(src.frames_read()),
               static_cast<unsigned long long>(src.dropped()),
               static_cast<unsigned long long>(src.malformed()),
               static_cast<unsigned long long>(rc_frames));
  for (int c = 0; c < n_cards; ++c) {
    const auto& t = agg.card(c);
    std::fprintf(stderr,
                 "card %d: frames=%llu crc_fail=%llu video=%llu seq %llu/%llu "
                 "rssiB=%.1f snr=%.1f\n",
                 c, static_cast<unsigned long long>(t.frames),
                 static_cast<unsigned long long>(t.crc_fail),
                 static_cast<unsigned long long>(t.video_bodies),
                 static_cast<unsigned long long>(t.seq_received),
                 static_cast<unsigned long long>(t.seq_expected), t.rssi_b_ema,
                 t.snr_ema);
  }
  for (int s = 0; s < 4; ++s) {
    const auto st = agg.decoder().stats(s);
    std::fprintf(stderr,
                 "stream %d: bodies=%llu sub_fail=%llu blocks=%llu unrec=%llu "
                 "pkts=%llu delivery=%d%%\n",
                 s, static_cast<unsigned long long>(st.bodies),
                 static_cast<unsigned long long>(st.subblocks_failed),
                 static_cast<unsigned long long>(st.blocks_decoded),
                 static_cast<unsigned long long>(st.blocks_unrecoverable),
                 static_cast<unsigned long long>(st.packets_out),
                 agg.decoder().window_delivery_pct(s));
  }
  if (!out_rtp_path.empty())
    std::fprintf(stderr, "rtp_out=%llu (file)\n",
                 static_cast<unsigned long long>(file_out.written));
  else
    std::fprintf(stderr, "rtp_out=%llu udp_failed=%llu\n",
                 static_cast<unsigned long long>(udp->sent()),
                 static_cast<unsigned long long>(udp->failed()));
  return 0;
}
