#ifndef MABUR_PLAYER_GS_SNAPSHOT_H_
#define MABUR_PLAYER_GS_SNAPSHOT_H_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace maburplay {

// One receiving card's signal, read from the s0 traffic class ONLY -- s0 is
// the layer whose loss breaks the picture, and per-card display is what
// reveals a dead antenna. Never pool cards.
struct GsCard {
  int id = 0;
  // True only when BOTH rssi and snr arrived as numbers. A card that is
  // present but silent renders its row with unlit bars and "never heard",
  // which must never be confused with a weak signal.
  bool heard = false;
  std::optional<double> rssi_dbm;
  std::optional<double> snr_db;
  // s0 combined (best-chain) EVM in dB, negative = clean. Deliberately NOT
  // part of `heard`: the aggregator only folds EVM from frames that carried
  // PHY status, so a perfectly healthy card reports null here until one
  // arrives. Gating `heard` on it would render a live antenna as dead.
  std::optional<double> evm_db;
};

// The link half of the OSD's inputs, decoded from one sideport datagram.
// Empty optional means "never received" and renders as an em-dash pair --
// it is NOT zero, and the distinction is the whole point of using optional
// here rather than sentinel values.
//
// Unit conversions happen at parse time, once, so the overlay formats what
// it is given: the wire carries `ov` and both loss figures as fractions and
// they arrive here already multiplied by 100. `air_pct` is a percent on the
// wire and passes through untouched.
struct GsSnapshot {
  // link.channel: the GS radio's operating channel. Straight from the GS
  // config, not measured, so it never goes null on a bad window -- an
  // empty optional here means an older maburgs that did not export it.
  std::optional<int> channel;
  // scan.state != "off": the GS's boot-time channel scan is enabled
  // (auto channel select, docs/channel-select.md), so `channel` may be a
  // pick rather than the configured home. False when the block is absent
  // (older maburgs) or malformed.
  bool scan_auto = false;
  // In-flight channel hop (spec 2026-09-14-inflight-channel-hop): true
  // only while `channel` equals hop.target and that target isn't
  // link.home -- i.e. the live channel is one the hop feature itself put
  // it on, right now. Deliberately NOT "a hop has ever happened this
  // session" (hop.hops is a monotonic counter that never resets on
  // withdraw or on hopping back to home), and NOT a plain
  // channel != home check either (that also fires for a boot-scan pick,
  // which already has its own "(a)" mark and is unrelated to this
  // feature). False when the block is absent (older maburgs) or the link
  // is on home / a stale hop target.
  bool hopped = false;
  // link.ctl.rung.mcs / .ov_base x 100, falling back to link.op.mcs /
  // .overhead_base x 100 when the ladder block is absent -- which is the
  // normal, permanent state of a static-pinned link (link.static_mcs >= 0
  // never ticks the controller, so the exporter emits link.ctl: null).
  std::optional<int> mcs;
  std::optional<double> fec_pct;
  std::optional<double> air_pct;        // link.air_pct
  // link.ctl.pre_fec_loss x 100, falling back to the link-level
  // link.pre_fec_loss x 100 in static-pin mode (same reason as mcs above).
  std::optional<double> pre_loss_pct;
  // link.residual_loss x 100. Since 2026-09-02 that key is symbol
  // abandonment (base+enh pooled), not the old packet-seq delivery window --
  // it now reads 0 on a clean link instead of blipping on reordered FEC
  // repairs, so this row is quieter than pre-break recordings suggest.
  std::optional<double> post_loss_pct;
  // link.rtt (link-rtt 2026-09-02): control-path RTT EWMA (telem queues
  // behind video on the drone TX — reads high under saturation, honest
  // congestion signal) and the min-RTT-filtered (pts − GS-mono) offset the
  // player combines with its OWN PtsAnchor for the absolute LAT floor.
  // pts_off_us empty ≠ zero: a 0 offset is a real value that would shift
  // every absolute latency number.
  std::optional<double> rtt_ms;         // link.rtt.ms
  std::optional<int64_t> pts_off_us;    // link.rtt.pts_off_us
  std::vector<GsCard> cards;            // in wire order
};

// Decodes a sideport datagram. Returns false on unparseable input or a
// non-object top level, leaving *out reset; true otherwise, with whatever
// fields were present and well-typed. Missing and wrongly-typed fields are
// dropped individually -- one bad key must never blank the whole overlay.
//
// NEVER throws: this runs on maburplay's 2 ms main loop, where an escaping
// exception is a dead player.
bool parse_gs_snapshot(const char* data, size_t n, GsSnapshot* out);

}  // namespace maburplay

#endif  // MABUR_PLAYER_GS_SNAPSHOT_H_
