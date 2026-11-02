#pragma once
// ParamHost -- maburgs' hosted MAVLink extended-parameter client plus the
// policy wrapped around it: when it is linked, its stats-sideport block
// (link.vtx, StatsVtxIn), and the loopback `vtx set` verb maburplay drives
// to edit the drone's settings. See param_client.h for the client's own
// list/set/heartbeat rules; ParamHost adds nothing to the wire, only to
// when the client lists and what a UI sees of it.
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "param_client.h"
#include "stats_exporter.h"

namespace maburgs {

// ParamRowState as the sideport and the daemon's log spell it: "idle"
// "sending" "in_progress" "accepted" "failed" "unsupported" "timeout".
const char* param_row_state_name(ParamRowState s);
// ParamListState the same way: "none" "pending" "complete" "partial".
const char* param_list_state_name(ParamListState s);

// Owns the ParamClient maburgs hosts and the policy around it: when it is
// linked (the session edge and a vtx_nonce change under one), its
// sideport block, and the `vtx set` verb.
class ParamHost {
 public:
  explicit ParamHost(ParamClient::SendFn send);

  // Once per core tick, before the tunnel flushes: mirrors the session
  // into the client (false->true lists; a nonce change while linked is a
  // false,true pair, since a re-rendezvous never drops through BEACONING
  // on its own), then ticks it.
  void tick(bool in_session, uint32_t vtx_nonce, uint64_t now_ms);
  void on_packet(const uint8_t* p, size_t n, uint64_t now_ms);
  // "vtx set <ID> <text>" -> "ok" | "err <why>"; any other verb -> nullopt.
  std::optional<std::string> handle_verb(const std::string& verb, const std::string& rest,
                                         uint64_t now_ms);
  StatsVtxIn sideport(uint64_t now_ms) const;
  const ParamClient& client() const;
  // One line per set submitted and per terminal outcome, for the daemon's
  // log: "vtx set VENC_MODE 100@1280x720/binned" / "vtx VENC_MODE
  // accepted" / "vtx VENC_MODE failed <status>".
  std::vector<std::string> take_log_lines();

 private:
  // Logs the outcome when the row that was pending before the call just
  // left pending (an ack or a resend timeout) -- compares
  // client_.state().pending_row before and after, so the caller never has
  // to.
  void note_terminal(int row_before);

  ParamClient client_;
  bool linked_ = false;
  uint32_t nonce_seen_ = 0;
  std::vector<std::string> log_;
};

}  // namespace maburgs
