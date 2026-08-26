#include "dvr_name.h"

#include <algorithm>
#include <cstdio>

#include <dirent.h>

namespace maburplay {

std::string DvrNamer::next(const std::string& dir) {
  // Best-effort scan of a directory maburplay does not own exclusively
  // (the card also holds flight-NNNN.jsonl, ctl-NNNN logs and recordings
  // from older naming). Malformed and foreign entries are ignored, and an
  // unreadable dir is not fatal: the fopen that follows reports it, and
  // refusing to name would turn a missing SD card into a silent path.
  int idx = last_issued_ + 1;
  if (DIR* d = opendir(dir.c_str())) {
    struct dirent* ent;
    while ((ent = readdir(d)) != nullptr) {
      int n = -1;
      if (std::sscanf(ent->d_name, "record-%d", &n) == 1 && n >= 0)
        idx = std::max(idx, n + 1);
    }
    closedir(d);
  }
  last_issued_ = idx;

  // %04d is a minimum width, not a cap: past 9999 the name grows a digit
  // rather than wrapping onto a file that already exists.
  char buf[32];
  std::snprintf(buf, sizeof(buf), "record-%04d.mp4", idx);
  return dir + "/" + buf;
}

}  // namespace maburplay
