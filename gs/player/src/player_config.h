#ifndef MABUR_PLAYER_CONFIG_H_
#define MABUR_PLAYER_CONFIG_H_

#include <string>

namespace maburplay {

struct DvrCfg {
  bool enabled = true;
  std::string dir = "/media/dvr";
  int fragment_ms = 1000;
};

// MSP DisplayPort OSD. `port` must match maburgs' msp.out.port -- separate
// daemons, separate config files, so this pairing is a deploy-time
// invariant neither binary can validate on its own.
struct OsdCfg {
  bool enable = false;
  int port = 14560;
  std::string font = "/usr/local/share/mabur/font_btfl.mfont";
  std::string scale = "sharp";  // "sharp" | "fill"
  int stale_ms = 2000;          // blank after this much silence; 0 = never
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
