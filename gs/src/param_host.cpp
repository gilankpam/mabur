#include "param_host.h"

#include <cctype>
#include <cstring>
#include <utility>

#include "mabur/config_params.h"
#include "mabur/param_value.h"

namespace maburgs {

namespace {

std::string trim(const std::string& s) {
  size_t b = 0, e = s.size();
  while (b < e && std::isspace(static_cast<unsigned char>(s[b]))) ++b;
  while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1]))) --e;
  return s.substr(b, e - b);
}

// accepted/failed/unsupported/timeout: the terminal states a result_ms was
// stamped for, so an age or a log line means something.
bool row_terminal(ParamRowState s) {
  return s == ParamRowState::Accepted || s == ParamRowState::Failed ||
         s == ParamRowState::Unsupported || s == ParamRowState::Timeout;
}

// Looks `id` up in kTable by exact string match -- a UI-typed id, not a
// wire-format 16-byte buffer, so a plain NUL-terminated compare is enough.
int find_row_by_id(const std::string& id) {
  for (size_t i = 0; i < mabur::cfgparam::kCount; ++i) {
    if (std::strcmp(mabur::cfgparam::kTable[i].id, id.c_str()) == 0) return static_cast<int>(i);
  }
  return -1;
}

}  // namespace

const char* param_row_state_name(ParamRowState s) {
  switch (s) {
    case ParamRowState::Idle: return "idle";
    case ParamRowState::Sending: return "sending";
    case ParamRowState::InProgress: return "in_progress";
    case ParamRowState::Accepted: return "accepted";
    case ParamRowState::Failed: return "failed";
    case ParamRowState::Unsupported: return "unsupported";
    case ParamRowState::Timeout: return "timeout";
  }
  return "?";
}

const char* param_list_state_name(ParamListState s) {
  switch (s) {
    case ParamListState::None: return "none";
    case ParamListState::Pending: return "pending";
    case ParamListState::Complete: return "complete";
    case ParamListState::Partial: return "partial";
  }
  return "?";
}

ParamHost::ParamHost(ParamClient::SendFn send) : client_(std::move(send)) {}

void ParamHost::note_terminal(int row_before) {
  if (row_before < 0) return;
  if (client_.state().pending_row == row_before) return;  // still pending: nothing to log yet
  const auto& row = client_.state().rows[static_cast<size_t>(row_before)];
  if (!row_terminal(row.st)) return;
  std::string line = std::string("vtx ") + mabur::cfgparam::kTable[row_before].id + " " +
                      param_row_state_name(row.st);
  if (!row.status.empty()) line += " " + row.status;
  log_.push_back(std::move(line));
}

void ParamHost::tick(bool in_session, uint32_t vtx_nonce, uint64_t now_ms) {
  const int before = client_.state().pending_row;
  // ParamClient re-lists on every session edge, and on a vtx_nonce change
  // while still linked (a re-rendezvous that never drops through
  // BEACONING) -- its held rows would otherwise go on describing a drone
  // that is no longer the one on the other end.
  if (in_session != linked_ || (in_session && vtx_nonce != nonce_seen_)) {
    if (linked_) client_.set_linked(false, now_ms);
    if (in_session) client_.set_linked(true, now_ms);
    linked_ = in_session;
    nonce_seen_ = vtx_nonce;
  }
  client_.tick(now_ms);
  note_terminal(before);
}

void ParamHost::on_packet(const uint8_t* p, size_t n, uint64_t now_ms) {
  const int before = client_.state().pending_row;
  client_.on_packet(p, n, now_ms);
  note_terminal(before);
}

std::optional<std::string> ParamHost::handle_verb(const std::string& verb, const std::string& rest,
                                                   uint64_t now_ms) {
  if (verb != "vtx") return std::nullopt;
  static const std::string kUsage = "err usage: vtx set <PARAM_ID> <value>";
  size_t sp = 0;
  while (sp < rest.size() && !std::isspace(static_cast<unsigned char>(rest[sp]))) ++sp;
  if (rest.substr(0, sp) != "set") return kUsage;
  const std::string tail = trim(rest.substr(sp));
  size_t sp2 = 0;
  while (sp2 < tail.size() && !std::isspace(static_cast<unsigned char>(tail[sp2]))) ++sp2;
  const std::string id = tail.substr(0, sp2);
  const std::string text = trim(tail.substr(sp2));
  if (id.empty() || text.empty()) return kUsage;
  const int row = find_row_by_id(id);
  if (row < 0) return "err no such parameter";
  std::string why;
  if (!client_.set(static_cast<size_t>(row), text, now_ms, &why)) return "err " + why;
  log_.push_back("vtx set " + id + " " + text);
  return "ok";
}

StatsVtxIn ParamHost::sideport(uint64_t now_ms) const {
  StatsVtxIn out;
  out.present = true;
  const auto& st = client_.state();
  out.linked = st.linked;
  out.alive = st.alive_ms == 0 ? std::nullopt : std::optional<bool>(client_.alive(now_ms));
  out.table_mismatch = st.table_mismatch;
  out.list = param_list_state_name(st.list);
  out.pending = st.pending_row;
  out.stats = client_.stats();
  out.rows.reserve(mabur::cfgparam::kCount);
  for (size_t i = 0; i < mabur::cfgparam::kCount; ++i) {
    const auto& row = st.rows[i];
    StatsVtxRow r;
    r.id = mabur::cfgparam::kTable[i].id;
    r.value = row.have_value ? std::optional<std::string>(mabur::paramval::format_text(row.value))
                              : std::nullopt;
    r.st = param_row_state_name(row.st);
    r.status = row.status;
    r.age_ms = row_terminal(row.st)
                   ? std::optional<int64_t>(static_cast<int64_t>(now_ms - row.result_ms))
                   : std::nullopt;
    out.rows.push_back(std::move(r));
  }
  return out;
}

const ParamClient& ParamHost::client() const { return client_; }

std::vector<std::string> ParamHost::take_log_lines() {
  std::vector<std::string> out = std::move(log_);
  log_.clear();
  return out;
}

}  // namespace maburgs
