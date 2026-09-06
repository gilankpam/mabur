#pragma once

#include <functional>
#include <string>

namespace maburgs {

// Fans one sideport datagram out to its destinations and reports whether any
// of them took it.
//
// Either argument may be null: `udp` is null when stats.enable is false (the
// UDP sinks are the only thing that key gates), `file` is null when there is
// no debug session. A dead UDP consumer must never stop the others, and must
// never stop the file -- that asymmetry is the reason this is a named
// function with tests rather than a lambda in main().
std::function<bool(const std::string&)> make_stats_sink(
    std::function<bool(const std::string&)> udp,
    std::function<void(const std::string&)> file);

}  // namespace maburgs
