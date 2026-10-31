#include "cal_wiring.h"

#include <algorithm>
#include <utility>

#include "mabur/cal_mav.h"

namespace maburgs {
namespace cal = mabur::cal;

namespace {

std::string k_of_n(size_t i, size_t n) {
  return std::to_string(i + 1) + "/" + std::to_string(n);
}

}  // namespace

const char* cal_state_name(CalSession::State st) {
  switch (st) {
    case CalSession::State::Idle: return "idle";
    case CalSession::State::Picking: return "picking";
    case CalSession::State::AwaitAck: return "await_ack";
    case CalSession::State::Sweep: return "sweep";
    case CalSession::State::AwaitDone: return "await_done";
    case CalSession::State::Analyze: return "analyze";
    case CalSession::State::Apply: return "apply";
    case CalSession::State::Verify: return "verify";
    case CalSession::State::Done: return "done";
    case CalSession::State::Failed: return "failed";
  }
  return "?";
}

const char* cal_kind_name(const CalSnapshot& snap) {
  if (snap.state == CalSession::State::Verify) return "verify";
  return snap.cur.kind == cal::Kind::Capacity ? "capacity" : "power";
}

std::string cal_sweep_token(const cal::SweepSpec& spec) {
  const char* k = spec.kind == cal::Kind::Capacity ? "cap" : "power";
  return std::string(k) + std::to_string(spec.bw);
}

size_t route_cal_body(CalSession& s, int card, const uint8_t* body, size_t len, int rssi_dbm,
                      bool crc_ok, uint64_t now_ms) {
  cal::CalFrameInfo info;
  if (!cal::parse_cal_payload(body, len, &info)) return 0;
  // The body may still end in the 802.11 FCS (devourer keeps it on every
  // Realtek chip) or not (a MAC that strips it); cal_air_bytes adds the FCS
  // itself, so it takes the payload length the parse matched.
  const size_t air = cal_air_bytes(info.len);
  s.on_cal_frame(card, info, rssi_dbm, crc_ok, air, now_ms);
  return air;
}

void feed_cal_packet(CalSession& s, const uint8_t* bytes, size_t n, uint64_t now_ms) {
  for (const cal::CalMsg& m : cal::decode_all(bytes, n)) s.on_mav(m, now_ms);
}

uint8_t cal_effective_width(uint8_t gs_width, bool linked, uint8_t peer_width) {
  if (!linked || peer_width == 0) return gs_width;
  return std::min(gs_width, peer_width);
}

StatsCalIn cal_sideport(const CalSession& s) {
  StatsCalIn out;
  if (!s.running()) return out;
  const CalSnapshot snap = s.snapshot();
  out.present = true;
  out.state = cal_state_name(snap.state);
  out.sweep_k = static_cast<uint32_t>(snap.sweep_n ? snap.sweep_i + 1 : 0);
  out.sweep_n = static_cast<uint32_t>(snap.sweep_n);
  out.kind = cal_kind_name(snap);
  out.bw = snap.cur.bw;
  out.cell_i = snap.cell_i;
  out.cell_n = snap.cell_n;
  return out;
}

std::vector<std::string> CalNarrator::observe(const CalSnapshot& snap,
                                              const std::vector<cal::SweepSpec>& sweeps) {
  std::vector<std::string> out;
  const CalSnapshot prev = std::move(prev_);
  prev_ = snap;
  const bool was_running = CalSession::is_running(prev.state);
  const bool now_running = CalSession::is_running(snap.state);

  if (!was_running && now_running) {
    std::string list;
    for (size_t i = 0; i < sweeps.size(); ++i) {
      if (i) list += ",";
      list += cal_sweep_token(sweeps[i]);
    }
    out.push_back("calibration started -> " + log_path_ + " sweeps=[" + list +
                  "] ch=" + std::to_string(snap.channel) + " (" + snap.pick_note + ")");
    return out;
  }
  if (!was_running) {
    // A pick that never reached the drone can still end the run.
    if (prev.state == CalSession::State::Picking && snap.state == CalSession::State::Failed)
      out.push_back("calibration failed: " + snap.fail_reason);
    else if (prev.state == CalSession::State::Picking && snap.state == CalSession::State::Idle)
      out.push_back("calibration aborted: " + snap.fail_reason);
    return out;
  }

  // The run was live last tick: report what ended since.
  const bool same_sweep = snap.sweep_i == prev.sweep_i;
  if (prev.state == CalSession::State::Apply &&
      (snap.state != CalSession::State::Apply || !same_sweep)) {
    const std::string what = "calibration apply " + k_of_n(prev.sweep_i, prev.sweep_n) + " " +
                             cal_sweep_token(prev.cur) + ": ";
    if (snap.state == CalSession::State::Failed || snap.state == CalSession::State::Idle) {
      out.push_back(what + snap.fail_reason);
    } else {
      // An adopted apply that kept something the GS did not send carries
      // the drone's note ("ok; mcs6 kept from the 20 MHz table").
      const std::string& note = snap.width[prev.cur.bw == 40 ? 1 : 0].apply_reason;
      out.push_back(what + (note.empty() ? std::string("ok") : note));
    }
  }
  if (snap.state != CalSession::State::Failed && snap.state != CalSession::State::Idle) {
    const size_t end = snap.state == CalSession::State::Done ? snap.sweep_i + 1 : snap.sweep_i;
    for (size_t i = prev.sweep_i; i < end && i < sweeps.size(); ++i)
      out.push_back("calibration sweep " + k_of_n(i, sweeps.size()) + " " +
                    cal_sweep_token(sweeps[i]) + " done");
  }
  if (snap.state == CalSession::State::Done)
    out.push_back("calibration done");
  else if (snap.state == CalSession::State::Failed)
    out.push_back("calibration failed: " + snap.fail_reason);
  else if (snap.state == CalSession::State::Idle)
    out.push_back("calibration aborted: " + snap.fail_reason);
  return out;
}

}  // namespace maburgs
