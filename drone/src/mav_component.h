#pragma once
// The drone's MAVLink component: answers MAVLink extended-parameter traffic
// (PARAM_EXT_REQUEST_LIST / _REQUEST_READ / _SET) for the config keys in
// cfgparam::kTable (common/include/mabur/config_params.h), and rewrites the
// config file one row at a time.
//
// The failure mode this file exists to prevent is the one config_write.h
// guards against: a config maburd cannot load is a drone stuck in a 2 s
// respawn loop with no console. A set therefore never hands on anything the
// real loader would refuse. Under lock_config_writes() it reads the file,
// patches the row's own line(s) (mavparam::patch_row, built on
// patch_toml_key, so every comment survives) and loads the candidate with
// mabur::load_config; cross-key rules
// (record.fps <= venc.fps, venc.size against the fps mode, low_power.enable
// needs msp.enable) are the loader's, not this file's, so a set that breaks
// one is answered PARAM_ACK_FAILED with the loader's message in a
// STATUSTEXT and the operator learns which key must move first. A candidate
// that loads is submitted to ConfigApplier (ApplyPort), acked
// PARAM_ACK_IN_PROGRESS, and resolved by poll_results once the applier has
// applied it to the running daemon and published the file, or refused it.
//
// Reads and acks report the file: the final ack carries the value read
// back from the published file, and a set whose rendered line is already
// the one in the file, or whose value equals what the file loads as, is
// acked without submitting (no flash wear for a ground app that re-sends).
// While an apply is in flight a repeat of the pending set is acked
// IN_PROGRESS again and any other set is refused "apply in progress",
// both without taking the lock the applier holds.
//
// Packets: inbound is MAVLink 2 only; a v1 packet, a bad CRC or a packet
// cut off at the end of a payload is dropped and counted, and the parser
// is reset at the end of every on_payload so a partial never bleeds into
// the next payload. Replies are queued as whole packets and drained by
// take_outbound into buffers of at most rc::kMavMaxPayload bytes. The
// component itself does no timing.
//
// The outbound queue holds at most kMavQueueCap packets. The cap applies
// to every packet alike (values, acks, status text, heartbeats): a packet
// that would exceed it is dropped and counted, never one already queued,
// and the ground app's own retry recovers it. A PARAM_EXT_REQUEST_LIST
// that arrives while an earlier list's values are still queued is
// coalesced into that list (counted, not answered again): the values on
// their way already answer it, and a flooding sender can't make the
// component reload the config per request.
//
// mabur/mav.h's helpers keep per-translation-unit channel state, so
// mav_component.cpp is the drone's one TU that packs or parses MAVLink. It
// parses on kChanDroneRx; that channel state is process-wide, so one
// MavComponent per process, and every call comes from one thread.

#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <string>
#include <vector>

#include "config_apply.h"
#include "mabur/param_value.h"
#include "mav_params.h"

namespace mabur {

// The slice of ConfigApplier a set needs: submit a patched row, poll for
// its result, and ask whether an apply is in flight. A real ConfigApplier's
// submit/take_result/busy bind directly; a test supplies a fake so it can
// drive the timing of a completion without a second thread.
struct ApplyPort {
  std::function<bool(size_t row, const mavparam::Value& want, std::string* why)> submit;
  std::function<bool(ConfigApplyResult*)> take_result;
  // True from an accepted submit until take_result hands its result over.
  // The applier holds lock_config_writes() for the whole apply, so a set
  // checks this before taking the lock: the MAV thread never waits out an
  // apply (a rebuild, or the cold rung's several seconds).
  std::function<bool()> busy;
};

// The drone sends a T_MAV frame (drone -> GS) at most this often, each
// carrying one take_outbound() of queued packets: at kMavMaxPayload per
// frame that caps the tunnel near 12 kB/s, far above a 13-value list, and
// leaves the 1 Hz Telem and the OSD stream their airtime.
constexpr uint64_t kMavUplinkMinMs = 50;

// Outbound queue bound, in packets: room for several lists' worth of
// replies, a few seconds of uplink at kMavUplinkMinMs.
constexpr size_t kMavQueueCap = 64;

struct MavComponentStats {
  uint64_t rx_packets = 0,       // good MAVLink 2 packets, before routing
      drop_not_for_us = 0,       // another target, or a message this component doesn't handle
      drop_bad_packet = 0,       // bad CRC, MAVLink 1, a header the parser rejects, a partial
      sets_in_progress = 0,      // PARAM_EXT_SET acked IN_PROGRESS and submitted to the applier
      sets_ok = 0,               // a set's final outcome: PARAM_EXT_SET acked ACCEPTED
      sets_refused = 0,          // a set's final outcome: acked VALUE_UNSUPPORTED or FAILED
      drop_queue_full = 0,       // an outbound packet dropped at kMavQueueCap
      list_coalesced = 0;        // a REQUEST_LIST answered by a list still queued
};

class MavComponent {
 public:
  // cfg_path: the file maburd loaded. port: ConfigApplier's submit/
  // take_result. log: one line per event, prefixed "[mav] ", no trailing
  // newline; may be null.
  MavComponent(std::string cfg_path, ApplyPort port, std::function<void(const std::string&)> log);

  // Inbound T_MAV payload bytes (already tag-verified by the caller).
  // Parses, routes, and queues replies.
  void on_payload(const uint8_t* bytes, size_t n);
  // Queues a HEARTBEAT (MAV_TYPE_ONBOARD_CONTROLLER, MAV_STATE_ACTIVE).
  void queue_heartbeat();
  // Appends whole queued packets to *out while out->size() stays within
  // rc::kMavMaxPayload; a packet is never split. Returns packets taken.
  int take_outbound(std::vector<uint8_t>* out);
  size_t outbound_packets() const { return queue_.size(); }
  // Drains the applier's result for the one set in flight, if any, and
  // queues its final PARAM_EXT_ACK (plus a STATUSTEXT on failure, or one
  // warning STATUSTEXT per notice of an accepted apply). The MAV
  // loop calls this every pass; a no-op when nothing is pending.
  void poll_results();
  const MavComponentStats& stats() const { return stats_; }
  // The MAVLink identity this component answers as (mabur/mav.h's kSysId
  // and kCompId), for callers that don't include the MAVLink headers.
  static uint8_t sys_id();
  static uint8_t comp_id();

 private:
  struct Msg;   // wraps mavlink_message_t, which this header doesn't include

  void route(const Msg& m);
  void handle_list();
  void handle_read(const char id16[16], int16_t index);
  void handle_set(const char id16[16], const char value[128], uint8_t type);
  void queue(const Msg& m, bool list_value = false);
  // A PARAM_EXT_VALUE for row idx; list_value marks one queued by a list.
  void queue_value(size_t idx, const mavparam::Value& v, bool list_value = false);
  // A PARAM_EXT_ACK, shared by handle_set's immediate acks and poll_results'
  // completion ack.
  void queue_ack(const char id16[16], const mavparam::Value* v, uint8_t mav_type, uint8_t result);
  void queue_status(const std::string& text);
  // A MAV_SEVERITY_WARNING STATUSTEXT, chunked when longer than one packet
  // holds (an accepted apply's notices).
  void queue_warning(const std::string& text);
  void log(const std::string& line) const;

  std::string cfg_path_;
  ApplyPort port_;
  std::function<void(const std::string&)> log_;
  struct Out {
    std::vector<uint8_t> bytes;
    bool list_value;   // a PARAM_EXT_VALUE queued by handle_list
  };
  std::deque<Out> queue_;
  size_t list_queued_ = 0;   // list values in queue_
  uint16_t status_id_ = 0;   // last long-STATUSTEXT id used (0 is never one)
  MavComponentStats stats_;

  // The one set submitted to the applier and not yet resolved: ConfigApplier
  // itself allows only one apply in flight, so at most one of these is ever
  // pending. Remembered so poll_results can ack with the row's identity and
  // pre-set value without re-reading the file.
  struct Pending {
    size_t idx = 0;
    char id16[16] = {};
    uint8_t row_type = 0;
    mavparam::Value cur{cfgparam::Type::Bool};
    std::string rendered;
    std::string field;
  };
  bool has_pending_ = false;
  Pending pending_;
};

}  // namespace mabur
