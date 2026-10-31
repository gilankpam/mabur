#pragma once
// ParamClient -- the ground-side MAVLink extended-parameter protocol
// client: list, read, set and link/heartbeat state against the drone's
// component (system 1, component 191). Pure data handling, driven only by
// a send callback, on_packet and tick(now); the caller owns the MAVLink
// tunnel and the clock.
//
// List. set_linked(true) clears every row, sets list = Pending and sends
// one PARAM_EXT_REQUEST_LIST. While Pending, every row that no value has
// reached since the round began is re-read individually by
// PARAM_EXT_REQUEST_READ (param_index set, id left blank) once every
// kParamListRetryMs, for at most kParamListRounds rounds; list reaches
// Complete as soon as every row has answered this round (checked as
// values arrive, not only on tick) or Partial once the rounds run out with
// rows still silent. A PARAM_EXT_VALUE whose param_count disagrees with
// the table's own row count flags table_mismatch, but the value is still
// stored when its id is known; an id the table doesn't have is dropped and
// counted (drop_unknown). refresh() starts the same round again without
// clearing the rows already held: they keep their values (have_value stays
// true) but are re-read like missing ones until they answer again.
// set_linked(false) drops every row back to the unlinked defaults.
//
// Set. One set is ever in flight: set() refuses a second one ("apply in
// progress"), an unknown row ("no such parameter"), a value that doesn't
// parse for the row's type ("bad value for <type>"), or any set while
// unlinked ("not linked") -- a refusal on a known row also records it
// (rows[row].st = Failed, status = the refusal, result_ms = now). An
// accepted set sends one PARAM_EXT_SET, marks the row Sending and clears
// its status, so an earlier reason never outlives the set that replaced
// it; while Sending, tick() resends it every kParamSetRetryMs, at most
// kParamSetRetries times, giving up with Timeout (result_ms = now, and one
// PARAM_EXT_REQUEST_READ of the row) if nothing ever answers. The first
// PARAM_ACK_IN_PROGRESS for the pending row moves it to InProgress and
// starts a kParamFinalMs deadline from that moment -- a repeated
// IN_PROGRESS updates the row's value but never extends the deadline, and
// tick() times the row out the same way if the deadline passes with no
// final answer. PARAM_ACK_ACCEPTED/FAILED/VALUE_UNSUPPORTED for the
// pending row is terminal: the row's state and value are set, pending_row
// clears, and an ACCEPTED on a row whose apply is Rebuild starts a fresh
// list round (a VENC_MODE set can move REC_SIZE's value along with it). An
// ack for any other row only updates that row's value. A STATUSTEXT from
// 1/191 lands on the pending row if one is set (so a reply that beats, or
// replaces, a lost ack still reaches the right row); otherwise it lands on
// whichever row's terminal Accepted/Failed/Unsupported result_ms is within
// kParamStatusMs of now (the latest one), or is dropped -- a refusal's
// reason follows its FAILED ack, and a warning (RADIO_LDPC against walls
// measured under the other setting) follows its ACCEPTED one. A long text
// in MAVLink 2's chunked form (nonzero id, chunk_seq from 0, the last
// chunk NUL-terminated) is reassembled first and lands whole; a missing
// chunk drops the text. set_linked(false) while a set is pending still
// ends in the full unlinked-defaults clear.
//
// Heartbeat. A HEARTBEAT from 1/191 stamps alive_ms; alive() is true while
// now - alive_ms is within kParamAliveMs. A heartbeat that arrives while
// already alive only refreshes the timestamp and never bumps seq() on its
// own -- the flip from not-alive to alive (or the timeout flip back, caught
// in tick) is what counts.
//
// seq() bumps once per call (on_packet/tick/set_linked/refresh/set) that
// changed anything a caller could observe through state(), so a UI can
// poll it cheaply instead of diffing the whole struct. A resend that
// changes nothing observable -- the common case while a set is in flight
// -- does not bump it.
//
// Parsing: each on_packet call gets one whole MAVLink packet (the MAVLink
// tunnel's deliver callback already splits the tunnel payload into
// packets) and parses it with its own local mavlink_message_t/
// mavlink_status_t, fresh every call -- see mabur/mav.h for why this
// never touches the tunnel's own channel state.

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>

#include "mabur/config_params.h"
#include "mabur/param_value.h"

namespace maburgs {

enum class ParamRowState : uint8_t {
  Idle,
  Sending,
  InProgress,
  Accepted,
  Failed,
  Unsupported,
  Timeout
};

enum class ParamListState : uint8_t { None, Pending, Complete, Partial };

struct ParamRow {
  bool have_value = false;
  mabur::paramval::Value value;
  ParamRowState st = ParamRowState::Idle;
  std::string status;
  uint64_t result_ms = 0;
};

struct ParamClientState {
  bool linked = false;
  uint64_t alive_ms = 0;
  bool table_mismatch = false;
  ParamListState list = ParamListState::None;
  int pending_row = -1;
  ParamRow rows[mabur::cfgparam::kCount];
};

struct ParamClientStats {
  uint64_t tx = 0, rx = 0, drop_unknown = 0, drop_bad = 0, timeouts = 0;
};

constexpr uint64_t kParamListRetryMs = 2000, kParamSetRetryMs = 2000, kParamFinalMs = 15000,
                    kParamStatusMs = 1500, kParamAliveMs = 3000;
constexpr int kParamListRounds = 3, kParamSetRetries = 3;

class ParamClient {
 public:
  using SendFn = std::function<void(const uint8_t* pkt, size_t n)>;

  // sysid/compid: this client's own MAVLink identity in outgoing packets
  // (255/190, matching QGroundControl's convention -- the drone answers
  // whoever asks, see docs/mavlink.md "Ids").
  explicit ParamClient(SendFn send, uint8_t sysid = 255, uint8_t compid = 190);

  // The caller's own session link state, independent of the drone
  // heartbeat's alive(): true starts a fresh list round, false drops
  // every row back to the unlinked defaults. A call that repeats the
  // current value is a no-op.
  void set_linked(bool linked, uint64_t now_ms);
  // One whole MAVLink packet. Parses it statelessly; a packet from
  // anyone but system 1 / component 191 is ignored.
  void on_packet(const uint8_t* p, size_t n, uint64_t now_ms);
  // Drives the list's retry rounds and the alive() timeout. A no-op when
  // nothing is pending and alive() hasn't changed.
  void tick(uint64_t now_ms);
  // Starts a new list round without clearing the rows already held. A
  // no-op while unlinked.
  void refresh(uint64_t now_ms);
  // Sets row's value to text, parsed for the row's own type. False with
  // *why on a refusal (see the class comment's "Set" section); true once
  // the PARAM_EXT_SET is on its way, with the row marked Sending.
  bool set(size_t row, const std::string& text, uint64_t now_ms, std::string* why);
  bool alive(uint64_t now_ms) const;
  const ParamClientState& state() const { return state_; }
  const ParamClientStats& stats() const { return stats_; }
  uint32_t seq() const { return seq_; }

 private:
  void send_list_request();
  void send_read_request(size_t idx);
  void send_set_packet(size_t idx, const mabur::paramval::Value& v);
  void begin_list(uint64_t now_ms);  // the shared body of refresh() and an ACCEPTED Rebuild row
  void refuse(size_t row, const std::string& why, uint64_t now_ms);
  void timeout_pending(uint64_t now_ms);  // Timeout from either tick() path: resends exhausted or the final deadline

  SendFn send_;
  uint8_t sysid_, compid_;
  ParamClientState state_;
  ParamClientStats stats_;
  uint32_t seq_ = 0;
  bool alive_cached_ = false;      // alive() as of the end of the last call, to detect a flip
  uint64_t list_last_req_ms_ = 0;  // when the list request, or the last retry round, went out
  int list_rounds_ = 0;            // retry rounds sent for the current list
  // Per row: a value (PARAM_EXT_VALUE or an ack) arrived since the current
  // list round began. The round's completion and its re-reads go by this,
  // not have_value, so a row held from an earlier round is still re-read.
  bool fresh_[mabur::cfgparam::kCount] = {};

  // The one set in flight, if any (state_.pending_row is the public half).
  mabur::paramval::Value set_value_{};  // re-sent verbatim by every resend
  uint64_t set_send_ms_ = 0;            // when the SET, or the last resend, went out
  int set_resends_ = 0;                 // resends already sent for the pending set
  bool set_in_progress_ = false;        // the pending row has seen its first IN_PROGRESS
  uint64_t set_deadline_ms_ = 0;        // first IN_PROGRESS + kParamFinalMs; valid iff set_in_progress_

  // A long STATUSTEXT being reassembled from its chunks (text_id_ 0: none).
  uint16_t text_id_ = 0;
  uint8_t text_next_seq_ = 0;
  std::string text_buf_;
};

}  // namespace maburgs
