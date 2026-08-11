#ifndef MABUR_PLAYER_CONFIG_H_
#define MABUR_PLAYER_CONFIG_H_

#include <string>

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

// Player -> GS feedback (spec 2026-08-12 decoder-idr-backchannel). Tells
// maburgs when the player has broken its own decoder reference chain, so the
// GS can request an IDR -- the only deterministic repair in the system, since
// this stream carries no IRAP and the encoder's GDR sweep does not repaint.
// `gs_port` must equal maburgs' link.player_fb_port; as with osd.port <->
// msp.out.port, neither binary can validate the pairing on its own, and the
// sideport's player.age_ms is the mitigation.
struct FeedbackCfg {
  bool enable = true;
  std::string gs_host = "127.0.0.1";
  int gs_port = 8303;   // mabur::playerfb::kDefaultPort
  int report_ms = 500;  // heartbeat; set edges are sent immediately anyway
};

struct Config {
  std::string ring_path = "/dev/shm/mabur-au";
  std::string socket = "/run/mabur-au.sock";
  std::string backend = "mpp";            // "mpp" | "null"
  std::string screen_mode = "1920x1080@60";
  DvrCfg dvr;
  OsdCfg osd;
  InputCfg input;
  FeedbackCfg feedback;
};

Config load_config(const std::string& path);  // strict; throws like maburgs

}  // namespace maburplay

#endif  // MABUR_PLAYER_CONFIG_H_
