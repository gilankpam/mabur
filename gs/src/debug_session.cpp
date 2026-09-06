#include "debug_session.h"

#include <dirent.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstdlib>   // std::atoi
#include <cstring>
#include <fstream>

namespace maburgs {

namespace {

bool is_dir(const std::string& p) {
  struct stat st{};
  return ::stat(p.c_str(), &st) == 0 && S_ISDIR(st.st_mode);
}

// Trailing whitespace/newline tolerated: the marker is also read by a shell
// or by hand during a deploy.
std::string read_marker(const char* path) {
  std::ifstream f(path);
  if (!f.good()) return {};
  std::string s;
  std::getline(f, s);
  while (!s.empty() && (s.back() == '\n' || s.back() == '\r' || s.back() == ' '))
    s.pop_back();
  return s;
}

// Index is the last path segment; -1 when it is not NNNN.
int index_of_dir(const std::string& dir) {
  const size_t slash = dir.find_last_of('/');
  const std::string base = slash == std::string::npos ? dir : dir.substr(slash + 1);
  if (base.empty()) return -1;
  for (char c : base)
    if (c < '0' || c > '9') return -1;
  return std::atoi(base.c_str());
}

}  // namespace

DebugSession::DebugSession(const std::string& root, bool enable,
                           const char* marker_path) {
  if (!enable) {
    // Deliberate: clearing the marker is how one knob stops maburplay too.
    std::remove(marker_path);
    return;
  }
  if (adopt_marker_(marker_path)) return;
  allocate_(root, marker_path);
}

bool DebugSession::adopt_marker_(const char* marker_path) {
  const std::string m = read_marker(marker_path);
  if (m.empty() || !is_dir(m)) return false;
  const int idx = index_of_dir(m);
  if (idx < 0) return false;
  dir_ = m;
  index_ = idx;
  rejoined_ = true;
  ok_ = true;
  return true;
}

bool DebugSession::allocate_(const std::string& root, const char* marker_path) {
  // Best-effort: the root may legitimately not exist yet on a fresh DVR.
  ::mkdir(root.c_str(), 0755);
  DIR* d = opendir(root.c_str());
  if (!d) {
    std::fprintf(stderr, "debug-log: opendir '%s' failed: %s\n", root.c_str(),
                 std::strerror(errno));
    return false;
  }
  // One past the highest existing NNNN directory. Foreign entries are
  // ignored -- the DVR is not a directory maburgs owns exclusively.
  int next_idx = 0;
  struct dirent* ent;
  while ((ent = readdir(d)) != nullptr) {
    const std::string name = ent->d_name;
    if (name.size() != 4) continue;
    bool numeric = true;
    for (char c : name)
      if (c < '0' || c > '9') numeric = false;
    if (!numeric) continue;
    if (!is_dir(root + "/" + name)) continue;
    next_idx = std::max(next_idx, std::atoi(name.c_str()) + 1);
  }
  closedir(d);

  char buf[16];
  std::snprintf(buf, sizeof(buf), "%04d", next_idx);
  const std::string dir = root + "/" + buf;
  if (::mkdir(dir.c_str(), 0755) != 0 && errno != EEXIST) {
    std::fprintf(stderr, "debug-log: mkdir '%s' failed: %s\n", dir.c_str(),
                 std::strerror(errno));
    return false;
  }
  // Marker written last: a reader never sees a marker naming a directory
  // that does not exist yet.
  std::ofstream mf(marker_path, std::ios::trunc);
  if (!mf.good()) {
    std::fprintf(stderr, "debug-log: cannot write marker '%s'\n", marker_path);
    return false;
  }
  mf << dir;
  mf.close();

  dir_ = dir;
  index_ = next_idx;
  ok_ = true;
  return true;
}

}  // namespace maburgs
