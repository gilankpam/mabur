#pragma once
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "channel_plan.h"
#include "channel_ranker.h"
#include "channel_scout.h"
#include "hop_controller.h"
#include "hop_verdict.h"
#include "log_writer.h"
#include "scout_radio.h"

namespace maburgs {

// scan.log (scanlog 3) -- the channel-selection record in the debug-log
// session directory (spec 2026-09-14-inflight-channel-hop). Formats are
// LOCKED by tests/test_scan_log.cpp:
//
//   scanlog 3 <header_info>
//   C <t> <card> <chip> <gen> <tx>x<rx> <bw_mask_hex> <tune5g_lo>-<tune5g_hi>
//     <fast_retune> <fa_ok> <igi_ok> <nhm_ok> <floor_ok>        # card caps
//   D <t> <card> <ch> <round> <observe_ms> <cca> <fa> <own> <foreign> <igi|->
//     <floor_dbm|nan> <flags_hex> <sess> <to_us> <read_us> <back_us> <bw>
//                                                                 # one scout dwell
//     (scanlog 3: <bw> = tuned width of the dwell, 20 during the boot
//     scan, radio.width in session)
//   K <t> <picked|none> <rounds> <ch>:<worst_busy>[:<floor>] ... pair=<lo>+<hi>|-
//                                                                 # the pick
//     (scanlog 3: the picked channel's standard 40 MHz pair, - when none)
//   M <t> <card|all> <from> <to> <reason>                        # a link move
//   V <t> <verdict> <evidence_hex> <ref_rung|-> <link_loss_pct> <recovered>
//     [<card> <foreign> <fa> <cca> <crc> <rssi> <snr> <drssi>]... # a verdict window
//   H <t> <kind> <epoch> <target> <score> <elapsed_ms>            # a hop event
class ScanLog {
 public:
  ScanLog(LogWriter& w, const std::string& dir, const std::string& header_info);
  ScanLog(const ScanLog&) = delete;
  ScanLog& operator=(const ScanLog&) = delete;

  bool ok() const { return s_ != LogWriter::kBadStream; }
  void rotate(const std::string& dir) { w_.reopen(s_, dir); }
  const std::string& path() const { return w_.path(s_); }

  void caps(double t_ms, int card, const CardCaps& c);
  void dwell(double t_ms, int card, const ScoutDwell& d);
  void pick(double t_ms, std::optional<uint8_t> picked, uint64_t rounds,
            const std::vector<RankEntry>& all, int min_rounds);
  void move(const MoveEvent& e);
  void verdict(double t_ms, const VerdictOut& o, const std::vector<VerdictCardIn>& cards,
               const VerdictLinkIn& link);
  void hop(const HopEvent& e);

 private:
  void put_(const char* b, int n);
  LogWriter& w_;
  LogWriter::Stream s_;
};

}  // namespace maburgs
