#ifndef MABUR_PLAYER_CONFIG_H_
#define MABUR_PLAYER_CONFIG_H_

#include <string>
#include <vector>

namespace maburplay {

struct DvrCfg {
  // Begin recording as soon as parameters arrive. NOT "the DVR exists":
  // the DVR is always available, and autostart:false is a live, armed
  // player waiting for a press on the input.rec button. There is
  // deliberately no config kill switch -- autostart:false with no button
  // configured is a player that never records, by a simpler route.
  bool autostart = true;
  std::string dir = "/media/dvr";
  int fragment_ms = 1000;
  // "raw"    — remux the received AUs untouched (byte-exact, the default).
  // "burned" — transcode with the MSP OSD composited in by the encoder.
  std::string mode = "raw";
  struct BurnedCfg {
    int bitrate_kbps = 12000;
    int fps_cap = 30;   // encode is capped independently of display rate
  } burned;
  // Where the record button records:
  // "gs" = this GS's DVR only, "vtx" = the drone's SD card only, "both".
  std::string target = "gs";
};

// MSP DisplayPort OSD. `port` must match maburgs' msp.out.port -- separate
// daemons, separate config files, so this pairing is a deploy-time
// invariant neither binary can validate on its own.
struct OsdCfg {
  bool enable = false;
  int port = 14560;
  std::string font = "/usr/local/share/mabur/font_btfl.mfont";
  std::string scale = "sharp";  // "sharp" | "fill"
  // Blank the OSD after this much silence; 0 = never. MUST stay several
  // multiples of the drone's msp.update_rate_hz period (default 1 Hz = one
  // snapshot per second, so 5000 = 5 missed snapshots): at ~2x the period a
  // SINGLE dropped snapshot blanks the whole overlay and the next one
  // repaints it, which reads as a strobe rather than as staleness.
  int stale_ms = 5000;

  // GS link-status overlay. Independent of the MSP OSD above: either may be
  // enabled alone, and a GS-only configuration is the natural one for an
  // aircraft with no MSP-capable FC.
  //
  // `port` must equal one of maburgs' stats.out ports -- separate daemons,
  // separate config files, so this pairing is a deploy-time invariant
  // neither binary can validate on its own, exactly like osd.port <->
  // msp.out.port. The 10 s silence warning is the mitigation.
  struct GsCfg {
    bool enable = false;
    int port = 8302;
    std::string font = "/usr/local/share/mabur/gs_osd.gfont";
    // Dim (never blank) after this much sideport silence. 3000 = 6 missed
    // samples at the 500 ms sideport cadence.
    int stale_ms = 3000;
  } gs;
};

// GPIO buttons. One button, one job: toggle the DVR.
struct InputCfg {
  struct RecCfg {
    // False when the config has no input.rec block at all, which is the
    // shipped default -- a ground station with no button wired.
    bool configured = false;
    // Header pin number, resolved to a gpiochip + line offset at startup
    // by matching the kernel's line names (PIN_<n> / GPIO<n> / <n>). The
    // Radxa ZERO 3 names its 40-pin header lines PIN_7..PIN_40 across
    // gpiochip1/3/4.
    int pin = 0;
    // Defaults describe a button between the pin and GND with the kernel's
    // internal pull-up: the line is requested ACTIVE_LOW so "pressed"
    // reads 1. Overridable because goggle builds differ and a silently
    // inverted button is a miserable bug to chase.
    bool active_low = true;
    std::string bias = "pull-up";  // "pull-up" | "pull-down" | "none"
  } rec;
};

struct DisplayCfg {
  // Phase-aware release delay for decoded frames (frame_regulator.h):
  // present at floor(pts) + regulate_ms instead of on arrival. 0 = off
  // (present the instant decode finishes, the pre-regulator behavior).
  // With vsync_lock this is the FALLBACK rule, used while the vblank
  // estimator is cold or stale.
  int regulate_ms = 12;
  // Servo release to next_vblank - vsync_lead_ms instead of the fixed
  // hold. lead covers
  // the present-path submission cost incl. the ~2 ms main-loop tick.
  bool vsync_lock = true;
  // Default 6 (bench-measured): with release-deadline wakeups
  // the submit lands ~0.2 ms after schedule, but decode work can still
  // block the drain for a few ms -- 6 keeps misses at ~0.2/s where 4
  // measured ~12/s and 9 wastes 3 ms of glass latency per frame.
  int vsync_lead_ms = 6;
  // Frames a sequential-slot chain may run before the regulator cuts it
  // with one dropped frame (frame_regulator.h chain_cuts). 0 = unbounded,
  // no chain cut at all: the default, and the bundle's. On a clean link the
  // refresh-start frame misses its release in about half of each
  // sensor/panel beat cycle, and a budget then turns every miss into a
  // repeat plus a drop; with no cut the stream runs one refresh late
  // through that phase and nothing is dropped (+12.5 ms e2e p50, judder
  // seconds 73 % -> 14 %; docs/findings/2026-10-15-regulator-chain-cuts.md).
  int chain_budget = 0;
  // Capture-time scheduling (frame_regulator.h place_by_capture), servo
  // mode only: a frame's vblank follows its capture time plus an adaptive
  // allowance (p99 - p1 of recent frame lateness + 1 ms), capped at this
  // many ms, instead of following its arrival. Late-tail frames within the
  // allowance keep their slot, so the clean-link repeat/skip judder in the
  // phases where frames arrive near the latch goes away, at up to one extra
  // refresh of latency in those phases. 0 = off (arrival-based servo),
  // also the bundle's choice: at 16 it cost ~+10 ms median on the bench.
  int sched_cap_ms = 0;
};

// GS-side reverse of the drone's ColorTrans sensor tuning (docs/colortrans.md).
// One knob. The transform's constants are compiled in (colortrans.cpp,
// kColorTrans3); retuning is a rebuild. Default off so a config without the
// block boots unchanged.
struct ColorTransCfg {
  bool enable = false;
};

// [decoder]. stream = true hands a split
// picture to MPP slice by slice while maburgs still receives it (rkvdec2
// stream mode); false submits every AU whole, the non-streamed path. A/B and
// kill switch. Needs the stream-mode GS image (docs/slices.md); anywhere
// else the decoder's probe refuses and maburplay submits whole AUs anyway.
struct DecoderCfg {
  bool stream = true;
};

struct Config {
  std::string ring_path = "/dev/shm/mabur-au";
  std::string socket = "/run/mabur-au.sock";
  std::string backend = "mpp";            // "mpp" | "v4l2req" | "null"
  std::string screen_mode = "1920x1080@60";
  DvrCfg dvr;
  OsdCfg osd;
  InputCfg input;
  DisplayCfg display;
  ColorTransCfg colortrans;
  DecoderCfg decoder;
};

Config load_config(const std::string& path,
                   std::vector<std::string>* defaulted = nullptr);  // strict; throws like maburgs

// Which decoders this binary carries (main.cpp fills it from the build's
// MABUR_PLAYER_MPP / MABUR_PLAYER_V4L2REQ). A host build has neither.
struct BuildCaps {
  bool mpp = false;
  bool v4l2req = false;
};

// "" when `c` can run on a build with `caps`; otherwise a one-line reason
// for the operator. A config error, like an unknown key: main exits 2.
std::string unsupported_on_build(const Config& c, const BuildCaps& caps);

}  // namespace maburplay

#endif  // MABUR_PLAYER_CONFIG_H_
