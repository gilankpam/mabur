#include "stats_sink.h"

#include <utility>

namespace maburgs {

std::function<bool(const std::string&)> make_stats_sink(
    std::function<bool(const std::string&)> udp,
    std::function<void(const std::string&)> file) {
  return [udp = std::move(udp), file = std::move(file)](const std::string& s) {
    bool any = false;
    if (udp && udp(s)) any = true;
    if (file) {
      file(s);
      any = true;  // a file write cannot fail visibly here: LogWriter drops
                   // and counts rather than reporting per-line failure.
    }
    return any;
  };
}

}  // namespace maburgs
