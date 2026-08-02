#ifndef MABUR_PLAYER_CONFIG_H_
#define MABUR_PLAYER_CONFIG_H_

#include <string>

namespace maburplay {

struct DvrCfg {
  bool enabled = true;
  std::string dir = "/media/dvr";
  int fragment_ms = 1000;
};

struct Config {
  std::string ring_path = "/dev/shm/mabur-au";
  std::string socket = "/run/mabur-au.sock";
  std::string backend = "mpp";            // "mpp" | "null"
  std::string screen_mode = "1920x1080@60";
  DvrCfg dvr;
};

Config load_config(const std::string& path);  // strict; throws like maburgs

}  // namespace maburplay

#endif  // MABUR_PLAYER_CONFIG_H_
