#pragma once
#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "ladder_controller.h"
#include "mabur/channel_set.h"
#include "mabur/link_key.h"
#include "mabur/uep_encoder.h"
#include "mabur/nack_tracker.h"

namespace maburgs {

/// USB device identifier for a radio card (VID, PID, instance index).
struct CardCfg {
  uint16_t usb_vid = 0x0bda;
  uint16_t usb_pid = 0;
  int index = 0;
};

/// Channel search + measurement over radio.channels. dwell/settle/min_rounds
/// are the measurement; search_ms the DISC burst at the start of an unlinked
/// dwell; op_window_ms the one-card beacon window on op; search_after_ms how
/// long every card holds op after a loss before sweeping; pick_margin how
/// much a candidate must beat the current channel by; one_card_ms the
/// one-card silent measurement prelude; max_ms the ceiling on an open pick.
/// What counts as busy air. Shared by every measurer -- the boot scout
/// and its pair ranker, the in-flight scout and its ranker, the verdict
/// engine -- so it lives under [radio.scan], not [hop]: a pinned GS with
/// no reactive hop still measures.
struct BusyCfg {
  int busy_dbm = -83;         // nf::kNhmAbsThDbm bucket edge (nhm_busy.h::busy_dbm_is_edge)
  // Foreign busy airtime % that makes a window/channel "blocked". One
  // threshold for every measurer on purpose: the hop trigger, the in-flight
  // ranker's blocked tier, the boot pick and the calibration pick, so a hop
  // target the ranker calls unblocked is unblocked to the verdict too and
  // its verify does not fail on landing.
  double blocked_pct = 30.0;
};

struct ScanCfg {
  int dwell_ms = 250;
  int settle_ms = 30;
  int min_rounds = 3;
  int search_ms = 100;
  int op_window_ms = 300;
  int search_after_ms = 5000;
  int pick_margin = 20;
  // 40 MHz boot pair pick: a candidate pair's busy % must beat op's by this
  // many percentage points before it takes the link; an exact busy tie
  // still falls to pick_margin (frame-score units, 20 MHz semantics).
  int pick_margin_pct = 10;
  int one_card_ms = 5000;
  int max_ms = 30000;
  BusyCfg busy;
};

/// In-flight channel hop verdict thresholds (the verdict engine).
struct HopVerdictCfg {
  double loss_pct = 3.0;
  double recovered_x = 3.0;
  // Floor on the recovered-symbols impaired term: fewer recovered symbols
  // than this never mark a window impaired, whatever the trailing mean.
  // 0 disables. Bench session 0232: 97 % of recovered-only
  // impaired windows at <= 3 % loss had 1-8 recovered.
  int recovered_min = 8;
  int weak_rssi_dbm = -78;
  int weak_snr_db = 12;
  int fading_drop_db = 6;
  int foreign_pps = 50;
  int fa_pps = 100;
  // AU rate below this fraction of the trailing per-window AU mean reads
  // `starved` even when a trickle of own frames still arrives (bench
  // session 0232: 5-30 own frames/s under a long-frame jam).
  // 0 disables the AU term. 60->30 fps low-power is 0.5, so keep it below.
  double starved_frac = 0.25;
  // Consecutive blocked windows that read `occupied` -> impaired on their
  // own, with no loss or weak signal needed, and (with that window's own
  // blocked reading) the hop's only route to `interfered`. A window with no
  // busy reading neither advances nor resets the count.
  int occupied_windows = 5;
};

/// In-flight channel hop (docs/inflight-channel-hop.md). Runs in
/// auto mode only: radio.channel = N pins the link and the reactive layer
/// is off (ChannelCore::reactive_). What counts as busy air is
/// radio.scan.busy, shared with the boot scout.
struct HopCfg {
  int window_ms = 150;
  int persist = 2;
  int dwell_observe_ms = 20;
  int dwell_period_ms = 1000;
  // USB freshness-burst pacing: at most one sweep of every candidate per
  // this, while a hop trigger is live and no hop is in flight.
  // dwell_period_ms is the periodic spare-card dwell, and also paces the
  // USB burst in a Hold whose candidates are all ranked. A spare card's
  // burst is skipped once every candidate is ranked (the periodic dwell
  // already keeps that ranking).
  int burst_period_ms = 333;
  int rank_visits = 5;
  int rank_max_age_ms = 10000;
  int confirm_ms = 500;
  // While the op channel reads blocked, an unconfirmed order is kept this
  // long (from the order) before it is withdrawn -- the jam that blocks the
  // op usually blocks the uplink carrying the order too (bench session
  // 0232). <= confirm_ms disables the extension.
  int confirm_extend_ms = 3000;
  int verify_ms = 1000;
  int cooldown_ms = 2000;
  int max_hops_per_min = 4;
  int backoff_ms = 30000;
  int one_card_repeats = 5;
  // Freshness-burst pacing when the burst card is a relay: its sweep blanks
  // it for ~280 ms, so 333 ms (burst_period_ms) would leave it ~85 % deaf.
  int relay_burst_period_ms = 1000;
  HopVerdictCfg verdict;
};

/// Radio hardware: channel, bandwidth, cards, and transmit card selection.
struct RadioCfg {
  // The channel set: the drone parks on a member, the
  // GS picks among them. Both ends list the same set (the drone's may be a
  // superset). Validated by mabur::channel_set_issue.
  std::vector<uint8_t> channels{40, 64, 112, 144};
  // `channel = "auto"` -> nullopt (measure the set, boot hop); a member ->
  // pinned there, no measurement.
  std::optional<uint8_t> pin;
  // HT20/HT40: validated to 20 or 40 in
  // config.cpp, and 40 additionally requires `channel` to sit on a standard
  // 5 GHz pair (mabur::ht40_offset) -- a 20-tuned receiver cannot hear a 40
  // MHz transmission at all (docs/bw40.md).
  uint8_t width = 20;
  // Empty + auto_scan: probe the bus and use every supported card found
  // (card_scan.h). A non-empty list pins exactly those devices and skips
  // the probe entirely -- that is the config-only way to fly one card.
  std::vector<CardCfg> cards;
  bool auto_scan = true;
  int tx_card = -1;            // -1 = auto-select (Plan 2)
  // CPE510 mabur-relay units (protocol v4 over UDP), "ipv4:port" each
  // (numeric; the UDP transport resolves nothing). Every entry becomes a RemoteCard AFTER the USB cards, in this order. Empty =
  // none. Several are supported by the code; the CPE firmware fixes every
  // unit at 10.83.11.1, so more than one needs a firmware addressing change.
  std::vector<std::string> relays;
  ScanCfg scan;
};

/// FEC configuration: sliding-window decoder parameters.
struct FecCfg {
  // Per-layer symbol size (stream 0..1); must match the drone's fec config
  // layer-for-layer or that layer's SBI framing misparses on the receive
  // side: subblocks_failed (sbf) climbs for that layer, or — if the
  // mismatched stride exceeds the body region — bodies increments while
  // symbols_in (si) stays frozen at 0. Not silent, but not bad_cfg either;
  // symbols never reach the sliding-window decoder to be flagged there.
  // JSON: scalar fans out, or 2-array.
  std::array<int, 2> symbol_size = {64, 64};
  int seq_horizon = 512;
};

/// Link-layer configuration: feedback rate, keepalive.
struct LinkCfg {
  int feedback_ms = 100;
  int beacon_keepalive_ms = 1000;
  // RCF slotting (gs-uplink-self-blanking findings): while video
  // flows, hold each control frame until the next AU completes so the send
  // lands in the drone's inter-AU idle instead of killing the next PPDU on
  // both RX cards. Max hold before sending anyway; 0 disables.
  int rcf_slot_hold_ms = 30;
  // ArrivalTracker guard, symbols: a seq is booked missing -- and shows as
  // link.pre_fec_loss, the OSD's LOSS row and the ladder's util input -- once
  // a later seq arrives this far ahead of it; a symbol heard after that
  // counts `late` and is never un-booked. The drone's parallel USB TX pool
  // (tx_threads x 3-frame URBs, ~12 bodies = 48 symbols in flight) reorders
  // bodies on air, so the old 32 (one FEC window) booked ~1 % of every burst
  // as missing-then-late on a bench with 0.008 % real loss. Bench sweep at
  // rung 5 (OSD pre-FEC loss / arr_late per s): 64 -> 0.67 % / 47,
  // 96 -> 0.23 % / 16, 128 -> 0.09 % / 5, 192 -> 0.02 % / 0.2, real repairs
  // flat at 0.1-0.6/s throughout. Cost of a bigger guard is booking delay on the
  // ladder's util input: 192 symbols is ~60 ms at rung 5, ~165 ms at rung 0,
  // both under the feedback period + probation.
  int arrival_guard_syms = 192;
  // Software NACK, base layer (docs/fec-nack.md). Optional
  // [link.nack]: absent = off. lookback < fec.seq_horizon (checked).
  mabur::NackCfg nack;
  // Static-link mode: when static_mcs >= 0 the adaptive controller is
  // bypassed entirely and every RCF commands exactly this MCS/FEC overhead
  // (HT, 20 MHz). Rendezvous/keep-alive/failsafe machinery is unaffected.
  // For bench debugging with a fixed operating point.
  int static_mcs = -1;
  // Actual-air overhead (airtime-balance-uep): literal, not a scaled cmd
  // value. Default 0.5 is the old cmd-value default (0.25) x2 -- see the
  // rule note in gs/src/config.cpp's load_config. Same-rate fixed pairs:
  // a base/enh pair, both defaulting to the old scalar's value, each
  // applied to its own stream.
  double static_overhead_base = 0.5;
  double static_overhead_enh = 0.5;
  // Width of the static-pin op (link.static_mcs >= 0), 20 or 40; 40 needs
  // radio.width = 40. Lets the bench pin a 40 MHz rung.
  int static_bw = 20;

  // Measured-loss ladder controller config (docs/link-adaptation.md): rungs
  // (post-`max_mcs` filter) plus the util/timing thresholds LadderController
  // decides on. Default ladder is the static feasibility floor — rung 0 is
  // the failsafe every controller starts and falls back to. mcs6 rides ov
  // 0.5, i.e. cmd-value 0.25 (not the originally designed 0.15): with bpb=4
  // a dead body is a 4-symbol cluster, and 0.15 leaves the s3 window unable
  // to absorb two dead bodies — see
  // docs/findings/2026-07-27-mcs6-bench-anomaly.md ov0.25 experiment.
  // Values are actual-air overhead (airtime-balance-uep, global rule:
  // every cmd-value default this migration touches x2) — {0,1.0}->{0,2.0},
  // {2,0.5}->{2,1.0}, {4,0.25}->{4,0.5}, {5,0.25}->{5,0.5}, {6,0.25}->{6,0.5},
  // {7,0.1}->{7,0.2}.
  // Same-rate fixed pairs: overhead is a base/enh pair; the
  // struct default duplicates each rung's value into both fields.
  LadderCfg ladder_cfg{{{0, 2.0, 2.0},
                        {2, 1.0, 1.0},
                        {4, 0.5, 0.5},
                        {5, 0.5, 0.5},
                        {6, 0.5, 0.5},
                        {7, 0.2, 0.2}}};

  // Pairing key (docs/link-pairing.md): the key FILE path. The
  // file is read at load; a missing file means the compiled-in default (and
  // a boot log line), a malformed one fails boot naming the file.
  std::string key_file = "/etc/mabur.key";
  mabur::LinkKey key = mabur::kDefaultLinkKey;   // resolved at load
  bool key_is_default = true;
  std::string key_source;                        // path, "default", or "link.key" (GS overlay)
};

/// Video reassembly tuning (PR C: the RTP output destination is gone --
/// video leaves maburgs via the shm AU ring; see AuRingOutCfg):
/// gap_timeout_ms before an unfilled chunk gap is truncated, and lookahead
/// frames ahead of head-of-line before it is force-advanced.
struct VideoCfg {
  int frame_gap_timeout_ms = 50;
  // Rate-aware ceiling for the gap timeout (gap_timeout_policy.h): at low
  // rungs the TX window's repair lifetime outlives the fixed floor, so the
  // timeout stretches toward the window's time-span, clamped here. 0
  // disables the stretch (fixed floor). Must stay below FrameStream's
  // 500 ms stall-reset backstop; the config range enforces that.
  int frame_gap_timeout_max_ms = 150;
  int frame_lookahead = 8;
};

/// MSP DisplayPort OSD side-channel output: whole reassembled snapshots go
/// out as UDP datagrams (maburplay renders them).
/// symbol_size/window must match the drone's msp config.
struct MspCfg {
  bool enable = false;
  std::string out_host = "127.0.0.1";
  int out_port = 14560;
  int symbol_size = 1312;
  int window = 16;
};

/// One stats-sideport destination.
struct StatsOut {
  std::string host = "127.0.0.1";
  int port = 8300;
};

/// Stats sideport: periodic UDP JSON datagram with link/FEC/video stats
/// (docs/observability.md).
///
/// `out` is the destination list and is ALWAYS non-empty after load: with
/// no `out` key it holds the single legacy host/port pair. It exists
/// because UDP unicast delivers a datagram to exactly one socket
/// (SO_REUSEPORT load-balances, it does not duplicate), so consumers
/// cannot simply share a port -- which is why statsrec has to re-emit, and
/// why maburplay's OSD would otherwise depend on statsrec staying alive.
///
/// `enable` gates the UDP sinks ONLY -- :8302 feeds maburplay's GS OSD, so
/// turning debug logging on or off must never blank it, and turning `enable`
/// off must not disable flight.jsonl (gs/src/stats_sink.h). `interval_ms` is
/// the snapshot cadence, shared by the sideport and flight.jsonl; there is
/// deliberately no separate debug_log.flight_period_ms.
struct StatsCfg {
  bool enable = false;
  int interval_ms = 500;  // clamped to [100, 10000] at load
  std::vector<StatsOut> out{StatsOut{}};
};

/// Shared-memory AU ring for the native player / ausniff gate.
struct AuRingOutCfg {
  bool enable = false;
  std::string path = "/dev/shm/mabur-au";
  std::string socket = "/run/mabur-au.sock";
  int slot_kb = 512;    // payload capacity per slot
  int slot_count = 16;  // 16 x 512 KiB = 8 MiB default ring
};

/// Debug logs: the per-session directory of machine-parsed files (ctl, probe,
/// au, flight jsonl written by maburgs; lat written by maburplay, which
/// follows the /tmp/mabur-session marker and has no config of its own).
/// Default OFF: nothing is written until this is turned on.
struct DebugLogCfg {
  bool enable = false;
  std::string dir = "/media/dvr/log";
  int ctl_period_ms = 1000;  // ctl.log S lines
  int rung_period_s = 10;    // ctl.log R lines (the per-rung store snapshot)
};

/// MAVLink UDP endpoint (docs/mavlink.md): the socket a ground app talks to,
/// bridged onto the T_MAV tunnel. Loopback by default: whoever can reach the
/// socket can set drone parameters, so a LAN bind is a deliberate choice.
struct MavlinkCfg {
  bool enable = true;
  std::string bind = "127.0.0.1";   // dotted IPv4
  int port = 14550;                 // MAVLink's conventional GCS port
};

/// Ground station configuration: radio, FEC, link, video reassembly, AU ring, MSP OSD.
struct Config {
  RadioCfg radio;
  FecCfg fec;
  LinkCfg link;
  VideoCfg video;
  MspCfg msp;
  StatsCfg stats;
  AuRingOutCfg au_ring;
  DebugLogCfg debug_log;
  HopCfg hop;
  MavlinkCfg mavlink;

  /// Builds decoder configuration with per-stream RS and UEP overhead
  /// (2 streams since the airtime-balance-uep fold-in).
  std::array<mabur::UepLayerCfg, 2> uep_layers() const;
};

/// One radio/width validation failure: `field` and `why` exactly as
/// load_config reports them ("config: ...: <field>: <why>").
struct ConfigIssue {
  std::string field, why;
};

/// radio.width's own checks: 20 or 40, and 40 only on a channel with a
/// standard 5 GHz HT40 pair (mabur::ht40_offset). load_config runs it on the
/// file's radio section; the web GS runs it on its page channel/width
/// override. std::nullopt = OK.
std::optional<ConfigIssue> radio_width_issue(uint8_t channel, int width);

/// Ladder/static-pin width vs the receiver's tuned width: a 40 MHz rung or
/// static_bw pin needs width 40 (a 20-tuned receiver cannot hear HT40).
/// load_config runs it against radio.width; the web GS (GS mode) against
/// its override. std::nullopt = OK.
std::optional<ConfigIssue> link_width_issue(const LinkCfg& link, int width);

/// Loads configuration from a TOML file (MABUR_GS_BUNDLE_DIR/maburgs.default.toml).
/// Fail-fast: missing keys use struct defaults; unknown keys, out-of-range values,
/// or missing file throw std::runtime_error("config: <file>:<line>: <field>: <why>").
///
/// `defaulted`, when non-null, receives "dotted.key=value" for every known
/// key the file did not set. main() prints it once at startup so a
/// hand-transcribed config shows its gaps in the log, not in the air.
///
/// `overlay_path`, when non-empty, names a second TOML file deep-merged into
/// the main document BEFORE any validation: tables merge key-by-key, every
/// other value (arrays included -- `[[link.ladder]]` replaces the whole
/// ladder) replaces. Strict-key and range checks then run on the merged
/// document exactly as for a single file. The web GS writes its config form
/// as an overlay; maburgs never passes one.
Config load_config(const std::string& path,
                   std::vector<std::string>* defaulted = nullptr,
                   const std::string& overlay_path = {});

}  // namespace maburgs
