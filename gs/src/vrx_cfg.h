#pragma once
// Config -> VrxCfg: the one field mapping every VrxController owner uses
// (maburgs run_radio, run_hop_inject_test, the web GS core). Kept out of
// vrx_controller.h so that header stays independent of the TOML config.
#include <cstdint>

#include "config.h"
#include "vrx_controller.h"

namespace maburgs {

inline VrxCfg vrx_cfg_from(const Config& cfg, uint8_t op_channel) {
  VrxCfg v;
  v.vtx_id = cfg.link.vtx_id;
  v.op_channel = op_channel;
  v.feedback_ms = cfg.link.feedback_ms;
  v.beacon_keepalive_ms = cfg.link.beacon_keepalive_ms;
  v.ladder = cfg.link.ladder_cfg;
  v.pin_mcs = cfg.link.static_mcs;
  v.pin_bw = cfg.link.static_bw;
  v.pin_overhead_base = cfg.link.static_overhead_base;
  v.pin_overhead_enh = cfg.link.static_overhead_enh;
  v.probe_pin_mcs = cfg.link.ladder_cfg.probe.pin_mcs;
  return v;
}

}  // namespace maburgs
