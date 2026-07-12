#pragma once

#include <array>
#include <optional>
#include <vector>

#include "controller.h"
#include "op_table.h"
#include "rendezvous.h"
#include "score.h"

namespace maburgs {

struct VrxCfg {
  uint32_t vtx_id = 1;
  uint8_t op_channel = 149;
  int feedback_ms = 100;
  int beacon_keepalive_ms = 1000;
  ControllerConfig ctrl;
  ScoreConfig score;
  std::vector<uint8_t> bw_set;  // rungs only when >1 entry
};

class VrxController {
 public:
  VrxController(const LinkTable& lt, VrxCfg cfg);
  void on_video(double rssi, double snr, bool crc_err, uint16_t seq,
                double now_ms);
  void on_rc_frame(const uint8_t* buf, size_t len, double now_ms);
  struct Out {
    std::vector<uint8_t> frame;
    bool is_disc;
  };
  // video_starved: the decode window since the last RCF completed ZERO
  // base-layer packets while video frames were still arriving. The SNR
  // estimate is survivor-biased in that regime (only the luckiest frames
  // decode, reading 30+ dB while the stream is effectively dead), so the
  // controller update is skipped and its blind-side on_tick restores
  // MAX_RANGE after feedback_timeout_ms (bench 2026-07-12 deadlock fix).
  std::optional<Out> step(double now_ms,
                          const std::array<uint8_t, 4>& layer_delivery,
                          std::optional<double> residual_loss,
                          bool video_starved = false);
  const OpPoint& cur_op() const;
  VrxState link_state() const;
  uint16_t rcf_seq() const;

 private:
  VrxCfg cfg_;
  Controller ctrl_;
  ScoreWindow win_;
  std::optional<RungWindow> rungs_;
  VrxRendezvous rz_;
  double last_fb_ms_ = -1e18;
  double last_keepalive_ms_ = -1e18;
  uint16_t seq_ = 0;
  OpPoint cur_op_;
  int cur_txagc_ = 32;
};

}  // namespace maburgs
