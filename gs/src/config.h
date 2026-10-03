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

namespace maburgs {

/// USB device identifier for a radio card (VID, PID, instance index).
struct CardCfg {
  uint16_t usb_vid = 0x0bda;
  uint16_t usb_pid = 0;
  int index = 0;
};

/// Channel search + measurement over radio.channels (spec
/// 2026-10-03-auto-channel-set §2). dwell/settle/min_rounds are the
/// measurement; search_ms the DISC burst at the start of an unlinked dwell;
/// op_window_ms the one-card beacon window on op; search_after_ms how long
/// every card holds op after a loss before sweeping; pick_margin how much a
/// candidate must beat the current channel by; one_card_ms the one-card
/// silent measurement prelude; max_ms the ceiling on an open pick.
struct ScanCfg {
  int dwell_ms = 250;
  int settle_ms = 30;
  int min_rounds = 3;
  int search_ms = 100;
  int op_window_ms = 300;
  int search_after_ms = 5000;
  int pick_margin = 20;
  int one_card_ms = 5000;
  int max_ms = 30000;
};

/// In-flight channel hop verdict thresholds (spec 2026-09-14-inflight-
/// channel-hop, section 2: the verdict engine).
struct HopVerdictCfg {
  double loss_pct = 3.0;
  double recovered_x = 3.0;
  // Floor on the recovered-symbols impaired term: fewer recovered symbols
  // than this never mark a window impaired, whatever the trailing mean.
  // 0 disables. Bench session 0232 (2026-09-26): 97 % of recovered-only
  // impaired windows at <= 3 % loss had 1-8 recovered.
  int recovered_min = 8;
  int weak_rssi_dbm = -78;
  int weak_snr_db = 12;
  int fading_drop_db = 6;
  int foreign_pps = 50;
  int fa_pps = 100;
  // NHM busy-airtime evidence (spec 2026-09-25-nhm-airtime §6). busy_dbm
  // must be an nf::kNhmAbsThDbm bucket edge (nhm_busy.h::busy_dbm_is_edge).
  // blocked_pct default is 50, not the spec's 30 -- the hw spike found
  // busy_dbm -83 / blocked_pct 50 the working pair (docs/nhm-airtime-
  // spike-findings-2026-09-25.md).
  int busy_dbm = -83;
  double blocked_pct = 50.0;  // foreign busy airtime that makes a window/channel "blocked"
  // AU rate below this fraction of the trailing per-window AU mean reads
  // `starved` even when a trickle of own frames still arrives (bench
  // session 0232, 2026-09-26: 5-30 own frames/s under a long-frame jam).
  // 0 disables the AU term. 60->30 fps low-power is 0.5, so keep it below.
  double starved_frac = 0.25;
};

/// In-flight channel hop (spec 2026-09-14-inflight-channel-hop). enable
/// governs whether a bad verdict actually retunes; scout_when_disabled lets
/// the scout keep ranking candidates for observability even while disabled.
struct HopCfg {
  bool enable = false;
  bool scout_when_disabled = true;
  int window_ms = 150;
  int persist = 2;
  int dwell_observe_ms = 5;
  int dwell_period_ms = 333;
  int rank_visits = 5;
  int rank_max_age_ms = 10000;
  int confirm_ms = 500;
  // While the op channel reads blocked, an unconfirmed order is kept this
  // long (from the order) before it is withdrawn -- the jam that blocks the
  // op usually blocks the uplink carrying the order too (bench session
  // 0232, 2026-09-26). <= confirm_ms disables the extension.
  int confirm_extend_ms = 3000;
  int verify_ms = 1000;
  int cooldown_ms = 2000;
  int max_hops_per_min = 4;
  int backoff_ms = 30000;
  int one_card_repeats = 5;
  HopVerdictCfg verdict;
};

/// Radio hardware: channel, bandwidth, cards, and transmit card selection.
struct RadioCfg {
  // The channel set (spec 2026-10-03 §2): the drone parks on a member, the
  // GS picks among them. Both ends list the same set (the drone's may be a
  // superset). Validated by mabur::channel_set_issue.
  std::vector<uint8_t> channels{40, 64, 112, 144};
  // `channel = "auto"` -> nullopt (measure the set, boot hop); a member ->
  // pinned there, no measurement.
  std::optional<uint8_t> pin;
  // HT20/HT40 (2026-09-24 HT40 top rungs): validated to 20 or 40 in
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
  // CPE510 mabur-relay units (protocol v3 over UDP), "ipv4:port" each
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
  // RCF slotting (gs-uplink-self-blanking findings 2026-09-02): while video
  // flows, hold each control frame until the next AU completes so the send
  // lands in the drone's inter-AU idle instead of killing the next PPDU on
  // both RX cards. Max hold before sending anyway; 0 disables.
  int rcf_slot_hold_ms = 30;
  // ArrivalTracker guard, symbols (cca-on 2026-09-23, from the tx-windows
  // spec §5.2): a seq is booked missing -- and shows as link.pre_fec_loss,
  // the OSD's LOSS row and the ladder's util input -- once a later seq
  // arrives this far ahead of it; a symbol heard after that counts `late`
  // and is never un-booked. The drone's parallel USB TX pool (tx_threads x
  // 3-frame URBs, ~12 bodies = 48 symbols in flight) reorders bodies on
  // air, so the old 32 (one FEC window) booked ~1 % of every burst as
  // missing-then-late on a bench with 0.008 % real loss. Bench sweep
  // 2026-09-23 at rung 5 (OSD pre-FEC loss / arr_late per s): 64 -> 0.67 %
  // / 47, 96 -> 0.23 % / 16, 128 -> 0.09 % / 5, 192 -> 0.02 % / 0.2, real
  // repairs flat at 0.1-0.6/s throughout. Cost of a bigger guard is booking
  // delay on the ladder's util input: 192 symbols is ~60 ms at rung 5,
  // ~165 ms at rung 0, both under the feedback period + probation.
  int arrival_guard_syms = 192;
  // Static-link mode: when static_mcs >= 0 the adaptive controller is
  // bypassed entirely and every RCF commands exactly this MCS/FEC overhead
  // (HT, 20 MHz). Rendezvous/keep-alive/failsafe machinery is unaffected.
  // For bench debugging with a fixed operating point (2026-07-12).
  int static_mcs = -1;
  // Actual-air overhead (airtime-balance-uep): literal, not a scaled cmd
  // value. Default 0.5 is the old cmd-value default (0.25) x2 -- see the
  // rule note in gs/src/config.cpp's load_config. Same-rate-fixed-pairs
  // (Task 3): split into a base/enh pair, both defaulting to the old
  // scalar's value -- Task 4 gives them independent semantics.
  double static_overhead_base = 0.5;
  double static_overhead_enh = 0.5;
  // Width of the static-pin op (link.static_mcs >= 0), 20 or 40; 40 needs
  // radio.width = 40. Lets the bench pin a 40 MHz rung.
  int static_bw = 20;

  // Measured-loss ladder controller config (spec
  // docs/superpowers/specs/2026-07-27-ladder-controller-design.md): rungs
  // (post-`max_mcs` filter) plus the util/timing thresholds LadderController
  // decides on. Default ladder is the spec's static feasibility floor —
  // rung 0 is the failsafe every controller starts and falls back to.
  // mcs6 rides ov 0.5, i.e. cmd-value 0.25 (not the spec's cmd-value
  // 0.15): with bpb=4 a dead body is a 4-symbol cluster, and 0.15 leaves
  // the s3 window unable to absorb two dead bodies — see
  // docs/mcs6-bench-anomaly.md ov0.25 experiment. Values are actual-air
  // overhead (airtime-balance-uep, global rule: every cmd-value default
  // this migration touches x2) — {0,1.0}->{0,2.0}, {2,0.5}->{2,1.0},
  // {4,0.25}->{4,0.5}, {5,0.25}->{5,0.5}, {6,0.25}->{6,0.5}, {7,0.1}->{7,0.2}.
  // Same-rate-fixed-pairs (Task 3): overhead is now a base/enh pair; the
  // struct default duplicates each rung's value into both fields.
  LadderCfg ladder_cfg{{{0, 2.0, 2.0},
                        {2, 1.0, 1.0},
                        {4, 0.5, 0.5},
                        {5, 0.5, 0.5},
                        {6, 0.5, 0.5},
                        {7, 0.2, 0.2}}};

  // Pairing key (spec 2026-10-01 link-pairing §2): the key FILE path. The
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
/// out as UDP datagrams (maburplay renders them -- see
/// docs/superpowers/specs/2026-08-03-maburplay-msp-osd-design.md).
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
/// (docs/superpowers/specs/2026-07-25-gs-stats-sideport-design.md).
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

/// Shared-memory AU ring for the native player / ausniff gate
/// (docs/superpowers/specs/2026-08-02-gs-player-au-ring-design.md).
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
/// as an overlay (spec 2026-09-27-web-ui §3.2); maburgs never passes one.
Config load_config(const std::string& path,
                   std::vector<std::string>* defaulted = nullptr,
                   const std::string& overlay_path = {});

}  // namespace maburgs
