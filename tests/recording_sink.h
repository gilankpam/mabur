#pragma once
#include <string>
#include <vector>

#include "channel_core.h"

struct RecordingSink : maburgs::ChannelSink {
  std::vector<std::string> lines;                 // log()
  std::vector<maburgs::MoveEvent> moves;          // move()
  std::vector<maburgs::HopEvent> hops;            // hop()
  std::vector<std::pair<int, maburgs::ScoutDwell>> dwells;
  std::vector<std::optional<uint8_t>> picks;
  int verdicts = 0;
  void dwell(double, int card, const maburgs::ScoutDwell& d) override { dwells.emplace_back(card, d); }
  void pick(double, std::optional<uint8_t> p, uint64_t, const std::vector<maburgs::RankEntry>&, int) override { picks.push_back(p); }
  void move(const maburgs::MoveEvent& e) override { moves.push_back(e); }
  void verdict(double, const maburgs::VerdictOut&, const std::vector<maburgs::VerdictCardIn>&,
               const maburgs::VerdictLinkIn&) override { ++verdicts; }
  void hop(const maburgs::HopEvent& e) override { hops.push_back(e); }
  void log(const std::string& l) override { lines.push_back(l); }
  bool has_line(const std::string& needle) const {
    for (const auto& l : lines) if (l.find(needle) != std::string::npos) return true;
    return false;
  }
  bool has_hop(const char* kind) const { for (auto& e : hops) if (e.kind == kind) return true; return false; }
  bool has_move(maburgs::MoveReason r) const { for (auto& e : moves) if (e.reason == r) return true; return false; }
};
