#ifndef MABUR_PLAYER_CONFIG_H_
#define MABUR_PLAYER_CONFIG_H_

#include <string>

namespace maburplay {

struct DvrCfg {
  bool enabled = true;
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

struct Config {
  std::string ring_path = "/dev/shm/mabur-au";
  std::string socket = "/run/mabur-au.sock";
  std::string backend = "mpp";            // "mpp" | "null"
  std::string screen_mode = "1920x1080@60";
  DvrCfg dvr;
  OsdCfg osd;
};

Config load_config(const std::string& path);  // strict; throws like maburgs

}  // namespace maburplay

#endif  // MABUR_PLAYER_CONFIG_H_
