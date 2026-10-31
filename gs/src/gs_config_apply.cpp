#include "gs_config_apply.h"

#include <arpa/inet.h>
#include <pthread.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <map>
#include <system_error>
#include <utility>

#include "mabur/config_write.h"
#include "mabur/toml.h"

namespace maburgs {

const std::vector<LiveKey> kGsLiveKeys = {
    {"radio.channel", "radio", "channel", LiveType::Channel},
    {"link.static_mcs", "link", "static_mcs", LiveType::Int},
    {"link.static_bw", "link", "static_bw", LiveType::Int},
    {"link.static_overhead_base", "link", "static_overhead_base", LiveType::Real},
    {"link.static_overhead_enh", "link", "static_overhead_enh", LiveType::Real},
    {"link.ladder", "link", "ladder", LiveType::Ladder},
    {"link.nack.enable", "link.nack", "enable", LiveType::Bool},
    {"debug_log.enable", "debug_log", "enable", LiveType::Bool},
    {"msp.enable", "msp", "enable", LiveType::Bool},
};

namespace {

constexpr const char* kLadderTable = "link.ladder";
constexpr std::array<const char*, 4> kRungKeys = {"mcs", "bw", "overhead_base", "overhead_enh"};

bool is(const LiveKey& k, const char* dotted) { return std::strcmp(k.key, dotted) == 0; }

std::string trim(const std::string& s) {
  size_t b = 0, e = s.size();
  while (b < e && std::isspace(static_cast<unsigned char>(s[b]))) ++b;
  while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1]))) --e;
  return s.substr(b, e - b);
}

bool has_space(const std::string& s) {
  for (char c : s)
    if (std::isspace(static_cast<unsigned char>(c))) return true;
  return false;
}

// The shortest text that reads back as `v`, always with a decimal point or
// exponent so it stays a TOML float: 0.5, 0.25, 2.0.
std::string render_real(double v) {
  char buf[40];
  for (int prec = 1; prec <= 17; ++prec) {
    std::snprintf(buf, sizeof(buf), "%.*g", prec, v);
    if (std::strtod(buf, nullptr) == v) break;
  }
  std::string s = buf;
  if (s.find_first_of(".eEni") == std::string::npos) s += ".0";
  return s;
}

std::string render_bool(bool b) { return b ? "true" : "false"; }

// One rung of a submitted ladder: each of kRungKeys' value text, verbatim.
using RungText = std::map<std::string, std::string>;

// Splits the inline ladder `[{k = v, ...}, ...]` into its rungs' value
// text. The config parser rejects inline tables, so the value is checked
// for shape here and its value syntax by parsing the same rungs written as
// [[...]] blocks; types and ranges stay the loader's job.
bool split_ladder(const std::string& value, std::vector<RungText>* rungs, std::string* why) {
  const std::string want =
      "link.ladder: want [{mcs = .., bw = .., overhead_base = .., overhead_enh = ..}, ...]";
  const std::string v = trim(value);
  if (v.size() < 2 || v.front() != '[' || v.back() != ']') {
    *why = want;
    return false;
  }
  const std::string inner = v.substr(1, v.size() - 2);
  std::vector<std::string> bodies;
  size_t pos = 0;
  auto skip_ws = [&] {
    while (pos < inner.size() && std::isspace(static_cast<unsigned char>(inner[pos]))) ++pos;
  };
  for (;;) {
    skip_ws();
    if (pos >= inner.size()) break;
    if (inner[pos] != '{') {
      *why = want;
      return false;
    }
    const size_t close = inner.find('}', pos + 1);
    if (close == std::string::npos) {
      *why = want;
      return false;
    }
    const std::string body = inner.substr(pos + 1, close - pos - 1);
    if (body.find('{') != std::string::npos) {
      *why = "link.ladder: a rung value cannot be a table";
      return false;
    }
    bodies.push_back(body);
    pos = close + 1;
    skip_ws();
    if (pos < inner.size()) {
      if (inner[pos] != ',') {
        *why = want;
        return false;
      }
      ++pos;
    }
  }
  if (bodies.empty()) {
    *why = "link.ladder: must be a non-empty array";
    return false;
  }

  std::vector<RungText> out;
  std::string doc;
  for (size_t i = 0; i < bodies.size(); ++i) {
    const std::string where = "link.ladder[" + std::to_string(i) + "]";
    RungText rung;
    size_t start = 0;
    const std::string& body = bodies[i];
    for (;;) {
      size_t comma = body.find(',', start);
      if (comma == std::string::npos) comma = body.size();
      const std::string pair = body.substr(start, comma - start);
      const size_t eq = pair.find('=');
      const std::string key = eq == std::string::npos ? "" : trim(pair.substr(0, eq));
      const std::string val = eq == std::string::npos ? "" : trim(pair.substr(eq + 1));
      if (key.empty() || val.empty()) {
        *why = where + ": want key = value";
        return false;
      }
      bool known = false;
      for (const char* k : kRungKeys) known = known || key == k;
      if (!known) {
        *why = where + ": unknown key " + key;
        return false;
      }
      if (!rung.emplace(key, val).second) {
        *why = where + ": " + key + " given twice";
        return false;
      }
      if (comma >= body.size()) break;
      start = comma + 1;
    }
    for (const char* k : kRungKeys) {
      if (!rung.count(k)) {
        *why = where + ": missing " + k;
        return false;
      }
    }
    doc += "[[r]]\n";
    for (const char* k : kRungKeys) doc += std::string(k) + " = " + rung[k] + "\n";
    out.push_back(std::move(rung));
  }
  try {
    (void)mabur::toml::parse_toml_string(doc, kLadderTable);
  } catch (const std::exception& e) {
    *why = std::string(kLadderTable) + ": " + mabur::loader_reason(e.what());
    return false;
  }
  *rungs = std::move(out);
  return true;
}

// The run of blank and comment lines that ends the [[link.ladder]] region
// right before the next table header: by position it is the next table's
// lead-in ("# Before stepping up, ..." above [link.probe]), so it survives
// the replace. "" when the region has none or there is no ladder.
std::string ladder_region_tail(const std::string& text) {
  const std::string ours = std::string("[[") + kLadderTable + "]]";
  bool in_region = false;
  size_t region_end = text.size();
  std::vector<std::pair<size_t, size_t>> lines;  // [begin, end) incl. '\n', inside the region
  size_t start = 0;
  while (start < text.size()) {
    size_t nl = text.find('\n', start);
    const size_t end = nl == std::string::npos ? text.size() : nl + 1;
    const std::string t = trim(text.substr(start, end - start));
    const bool header = !t.empty() && t[0] == '[';
    if (!in_region) {
      in_region = header && t == ours;
    } else if (header && t != ours) {
      region_end = start;
      break;
    }
    if (in_region) lines.emplace_back(start, end);
    start = end;
  }
  if (!in_region) return "";
  size_t tail = region_end;
  for (auto it = lines.rbegin(); it != lines.rend(); ++it) {
    const std::string t = trim(text.substr(it->first, it->second - it->first));
    if (!t.empty() && t[0] != '#') break;
    tail = it->first;
  }
  return text.substr(tail, region_end - tail);
}

// publish_config's verifier: the loader maburgs boots with, so the file is
// never replaced by one the daemon would refuse to start from.
bool verify_with_load_config(const std::string& candidate_path, std::string* err) {
  try {
    (void)load_config(candidate_path);
    return true;
  } catch (const std::exception& e) {
    if (err) *err = e.what();
    return false;
  }
}

// The config's real path: the candidate scratch goes beside the file a
// symlinked config points at, where publish_config writes its own.
std::string real_path_of(const std::string& path) {
  char* r = ::realpath(path.c_str(), nullptr);
  if (!r) return path;
  std::string out = r;
  std::free(r);
  return out;
}

// Copies the one live field `k` names from `src` into `dst`.
void copy_live_field(const LiveKey& k, const Config& src, Config* dst) {
  if (is(k, "radio.channel")) dst->radio.pin = src.radio.pin;
  else if (is(k, "link.static_mcs")) dst->link.static_mcs = src.link.static_mcs;
  else if (is(k, "link.static_bw")) dst->link.static_bw = src.link.static_bw;
  else if (is(k, "link.static_overhead_base")) dst->link.static_overhead_base = src.link.static_overhead_base;
  else if (is(k, "link.static_overhead_enh")) dst->link.static_overhead_enh = src.link.static_overhead_enh;
  else if (is(k, "link.ladder")) dst->link.ladder_cfg.ladder = src.link.ladder_cfg.ladder;
  else if (is(k, "link.nack.enable")) dst->link.nack.enable = src.link.nack.enable;
  else if (is(k, "debug_log.enable")) dst->debug_log.enable = src.debug_log.enable;
  else if (is(k, "msp.enable")) dst->msp.enable = src.msp.enable;
}

}  // namespace

const LiveKey* gs_live_key(const std::string& dotted) {
  for (const LiveKey& k : kGsLiveKeys)
    if (dotted == k.key) return &k;
  return nullptr;
}

std::string render_live_value(const Config& c, const LiveKey& k) {
  if (is(k, "radio.channel"))
    return c.radio.pin ? std::to_string(static_cast<int>(*c.radio.pin)) : "\"auto\"";
  if (is(k, "link.static_mcs")) return std::to_string(c.link.static_mcs);
  if (is(k, "link.static_bw")) return std::to_string(c.link.static_bw);
  if (is(k, "link.static_overhead_base")) return render_real(c.link.static_overhead_base);
  if (is(k, "link.static_overhead_enh")) return render_real(c.link.static_overhead_enh);
  if (is(k, "link.ladder")) {
    std::string out = "[";
    const auto& ladder = c.link.ladder_cfg.ladder;
    for (size_t i = 0; i < ladder.size(); ++i) {
      if (i) out += ", ";
      out += "{mcs = " + std::to_string(ladder[i].mcs) + ", bw = " + std::to_string(ladder[i].bw) +
             ", overhead_base = " + render_real(ladder[i].overhead_base) +
             ", overhead_enh = " + render_real(ladder[i].overhead_enh) + "}";
    }
    return out + "]";
  }
  if (is(k, "link.nack.enable")) return render_bool(c.link.nack.enable);
  if (is(k, "debug_log.enable")) return render_bool(c.debug_log.enable);
  if (is(k, "msp.enable")) return render_bool(c.msp.enable);
  return "";
}

bool patch_live_key(std::string* text, const LiveKey& k, const std::string& value,
                    std::string* why) {
  if (value.find_first_of("\r\n") != std::string::npos) {
    *why = std::string(k.key) + ": value spans lines";
    return false;
  }
  if (k.type != LiveType::Ladder) {
    if (!mabur::patch_toml_key(text, k.section, k.name, value)) {
      *why = std::string(k.key) + ": not set in the config file";
      return false;
    }
    return true;
  }
  std::vector<RungText> rungs;
  if (!split_ladder(value, &rungs, why)) return false;
  // Written in the bundle's block layout. The replaced region's own
  // comments (between or inside the old blocks) are not carried over; the
  // comment run that leads into the next table is.
  std::string blocks;
  for (size_t i = 0; i < rungs.size(); ++i) {
    if (i) blocks += "\n";
    blocks += std::string("[[") + kLadderTable + "]]\n" +
              "mcs = " + rungs[i]["mcs"] + "\n" +
              "bw  = " + rungs[i]["bw"] + "\n" +
              "overhead_base = " + rungs[i]["overhead_base"] + "\n" +
              "overhead_enh  = " + rungs[i]["overhead_enh"] + "\n";
  }
  const std::string tail = ladder_region_tail(*text);
  if (tail.empty() || tail[0] != '\n') blocks += "\n";
  blocks += tail;
  if (!mabur::replace_toml_array_tables(text, kLadderTable, blocks)) {
    *why = std::string(k.key) + ": no [[link.ladder]] in the config file";
    return false;
  }
  return true;
}

// --- GsConfigApplier -------------------------------------------------------

GsConfigApplier::GsConfigApplier(std::string cfg_path, Config live, GsApplyHooks hooks,
                                 std::function<void(const std::string&)> log)
    : cfg_path_(std::move(cfg_path)),
      live_(std::move(live)),
      hooks_(std::move(hooks)),
      log_(std::move(log)) {}

GsConfigApplier::~GsConfigApplier() {
  {
    std::lock_guard<std::mutex> l(mu_);
    stop_ = true;
  }
  cv_.notify_all();
  // A worker waiting for the owner's next step takes stop_ as "release":
  // nothing more is published.
  if (thread_.joinable()) thread_.join();
}

void GsConfigApplier::log(const std::string& line) const {
  if (log_) log_(line);
}

bool GsConfigApplier::busy() const { return in_flight_; }

const Config& GsConfigApplier::live() const { return live_; }

std::string GsConfigApplier::get(const std::string& key, std::string* why) const {
  const LiveKey* k = gs_live_key(key);
  if (!k) {
    if (why) *why = "not live: " + key;
    return "";
  }
  return render_live_value(live_, *k);
}

std::string GsConfigApplier::get_all() const {
  std::string out;
  for (const LiveKey& k : kGsLiveKeys) out += std::string(k.key) + " = " + render_live_value(live_, k) + "\n";
  return out;
}

bool GsConfigApplier::submit(const std::string& key, const std::string& value, std::string* why) {
  auto refuse = [&](const std::string& text) {
    if (why) *why = text;
    return false;
  };
  const LiveKey* k = gs_live_key(key);
  if (!k) return refuse("not live: " + key);
  if (in_flight_) return refuse("busy");
  const std::string v = trim(value);
  if (v.empty()) return refuse("empty value");
  if (v.find_first_of("\r\n") != std::string::npos) return refuse("value spans lines");

  cur_key_ = k;
  cur_value_ = v;
  t0_ = std::chrono::steady_clock::now();
  settled_.reset();
  if (k->type == LiveType::Channel && hooks_.hop_or_cal_busy()) {
    settled_ = GsApplyResult{false, "hop or calibration in flight", k->key, 0};
  } else if (v == render_live_value(live_, *k)) {
    settled_ = GsApplyResult{true, "", k->key, 0};
  } else {
    std::lock_guard<std::mutex> l(mu_);
    if (stop_) return refuse("shutting down");
    // The thread first, so a refusal for a thread that cannot start leaves
    // no job behind; it waits on mu_, held here, until the job is in place.
    if (!thread_.joinable()) {
      try {
        thread_ = std::thread([this] { run(); });
      } catch (const std::system_error&) {
        return refuse("apply thread failed to start");
      }
    }
    job_ = Job{k, v, !backup_written_};
    has_job_ = true;
    phase_ = Phase::Reading;
    cv_.notify_all();
  }
  in_flight_ = true;
  return true;
}

void GsConfigApplier::tell(Step s, Phase next) {
  {
    std::lock_guard<std::mutex> l(mu_);
    step_ = s;
    phase_ = next;
  }
  cv_.notify_all();
}

bool GsConfigApplier::poll(GsApplyResult* out) {
  if (!in_flight_) return false;
  if (settled_) {
    finish(settled_->ok, settled_->reason, out);
    return true;
  }
  const LiveKey& k = *cur_key_;
  const bool channel = k.type == LiveType::Channel;
  Phase ph;
  {
    std::lock_guard<std::mutex> l(mu_);
    ph = phase_;
  }
  switch (ph) {
    case Phase::Refused: {
      std::string reason;
      {
        // The worker sets nothing after Refused; Idle lets the next submit in.
        std::lock_guard<std::mutex> l(mu_);
        reason = std::move(refusal_);
        phase_ = Phase::Idle;
      }
      finish(false, reason, out);
      return true;
    }
    case Phase::Ready: {
      // The process gets what it runs now plus this one key, as the loader
      // read it from the candidate: anything else that differs in the file
      // (an ssh edit not yet restarted into) is not applied by this set.
      Config cand;
      {
        std::lock_guard<std::mutex> l(mu_);
        cand = std::move(ready_cand_);
      }
      cand_ = live_;
      copy_live_field(k, cand, &cand_);
      std::string reason;
      if (channel || apply_to_process(k, cand_, &reason)) {
        tell(Step::Publish, Phase::Publishing);
        return false;
      }
      tell(Step::Release, Phase::Idle);
      finish(false, reason, out);
      return true;
    }
    case Phase::Published: {
      bool ok;
      std::string err;
      {
        std::lock_guard<std::mutex> l(mu_);
        ok = pub_ok_;
        err = pub_err_;
      }
      if (!ok) {
        // The file still holds the previous value: put the process back on it.
        if (!channel) {
          log("maburgs: cfg set " + std::string(k.key) + ": publish failed, reverting");
          std::string rr;
          if (!apply_to_process(k, live_, &rr))
            log("maburgs: cfg set " + std::string(k.key) + ": revert failed: " + rr);
        }
        tell(Step::Release, Phase::Idle);
        finish(false, "publish failed: " + err, out);
        return true;
      }
      backup_written_ = true;
      if (channel) {
        std::optional<std::string> refused;
        if (cand_.radio.pin) refused = hooks_.set_pin(*cand_.radio.pin);
        else hooks_.set_auto();
        if (refused) {
          // The file now claims a pin the link did not take: the worker,
          // still holding the write lock, publishes the previous text.
          restore_reason_ = *refused;
          tell(Step::Restore, Phase::Restoring);
          return false;
        }
      }
      live_ = cand_;
      tell(Step::Release, Phase::Idle);
      finish(true, "", out);
      return true;
    }
    case Phase::Restored: {
      bool ok;
      std::string err;
      {
        // The worker sets nothing after Restored.
        std::lock_guard<std::mutex> l(mu_);
        ok = pub_ok_;
        err = pub_err_;
        phase_ = Phase::Idle;
      }
      if (!ok)
        log("maburgs: cfg set " + std::string(k.key) + ": restoring the previous file failed: " + err);
      finish(false, restore_reason_, out);
      return true;
    }
    case Phase::Idle:
    case Phase::Reading:
    case Phase::Publishing:
    case Phase::Restoring:
      return false;
  }
  return false;
}

bool GsConfigApplier::apply_to_process(const LiveKey& k, const Config& c, std::string* reason) {
  switch (k.type) {
    case LiveType::Int:
    case LiveType::Real:
      hooks_.set_static(c);  // link.static_*
      return true;
    case LiveType::Ladder:
      hooks_.replace_ladder(c);
      return true;
    case LiveType::Bool:
      if (is(k, "link.nack.enable")) {
        hooks_.nack(c.link.nack.enable);
        return true;
      }
      if (is(k, "debug_log.enable")) {
        std::string err;
        if (hooks_.debug_log(c.debug_log.enable, &err)) return true;
        *reason = err.empty() ? "debug log refused" : err;
        return false;
      }
      if (is(k, "msp.enable")) {
        hooks_.msp(c.msp.enable, c);
        return true;
      }
      break;
    case LiveType::Str:
    case LiveType::Channel:
      break;
  }
  *reason = std::string(k.key) + ": no apply for this key";
  return false;
}

void GsConfigApplier::finish(bool ok, const std::string& reason, GsApplyResult* out) {
  const uint64_t ms = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0_)
          .count());
  GsApplyResult r{ok, ok ? "" : reason, cur_key_->key, ms};
  log("maburgs: cfg set " + r.key + " = " + cur_value_ + " -> " +
      (ok ? std::string("ok") : "err " + r.reason) + " (" + std::to_string(ms) + " ms)");
  settled_.reset();
  in_flight_ = false;
  *out = std::move(r);
}

void GsConfigApplier::run() {
  pthread_setname_np(pthread_self(), "mbr-cfgset");
  std::unique_lock<std::mutex> l(mu_);
  for (;;) {
    cv_.wait(l, [this] { return stop_ || has_job_; });
    if (stop_) return;
    const Job job = std::move(job_);
    has_job_ = false;
    l.unlock();
    work(job);
    l.lock();
  }
}

GsConfigApplier::Step GsConfigApplier::await_step(std::unique_lock<std::mutex>& l) {
  cv_.wait(l, [this] { return stop_ || step_ != Step::None; });
  const Step s = step_ == Step::None ? Step::Release : step_;
  step_ = Step::None;
  return s;
}

void GsConfigApplier::work(const Job& job) {
  const LiveKey& k = *job.key;
  std::string original, patched, refusal;
  Config cand;
  bool ready = false;
  // Held from the re-read through the publish (and a restore): a
  // calibration publish between them would be overwritten by this one's
  // stale text. Declared before `l` so mu_ is let go first.
  auto write_lock = mabur::lock_config_writes();
  try {
    if (!mabur::read_text_file(cfg_path_, &original)) {
      refusal = "config file unreadable";
    } else {
      patched = original;
      if (patch_live_key(&patched, k, job.value, &refusal)) {
        // The candidate is the patched text as the boot loader reads it:
        // the process is never handed a config maburgs would refuse to
        // start from.
        const std::string tmp = real_path_of(cfg_path_) + ".new";
        if (!mabur::write_file_fsync(tmp, patched)) {
          refusal = "failed to write " + tmp;
        } else {
          try {
            cand = load_config(tmp);
            ready = true;
          } catch (const std::exception& e) {
            refusal = mabur::loader_reason(e.what());
          }
        }
        // Gone before the apply: a refused set leaves no trace, and
        // publish_config writes its own copy.
        ::unlink(tmp.c_str());
      }
    }
    if (ready && k.type == LiveType::Ladder) {
      // The loader drops rungs above link.max_mcs without a word; a set
      // that names one is refused rather than flown short. The filter keeps
      // order, so the first rung that does not line up is the dropped one.
      std::vector<RungText> rungs;
      std::string ignored;
      split_ladder(job.value, &rungs, &ignored);
      const auto& eff = cand.link.ladder_cfg.ladder;
      size_t j = 0;
      for (size_t i = 0; i < rungs.size(); ++i) {
        if (j < eff.size() && eff[j].mcs == std::atoi(rungs[i]["mcs"].c_str()) &&
            eff[j].bw == std::atoi(rungs[i]["bw"].c_str())) {
          ++j;
          continue;
        }
        refusal = "rung " + std::to_string(i) + " exceeds link.max_mcs";
        ready = false;
        break;
      }
    }
  } catch (const std::exception& e) {
    ready = false;
    refusal = std::string("apply threw: ") + e.what();
  }

  std::unique_lock<std::mutex> l(mu_);
  if (!ready) {
    refusal_ = refusal;
    phase_ = Phase::Refused;
    return;
  }
  ready_cand_ = std::move(cand);
  phase_ = Phase::Ready;
  if (await_step(l) != Step::Publish) return;

  auto publish = [&](const std::string& content, bool write_backup) {
    std::string err;
    mabur::PublishResult pr;
    try {
      pr = mabur::publish_config(cfg_path_, content, "pre-live", &err, verify_with_load_config,
                                 write_backup);
    } catch (const std::exception& e) {
      pr = mabur::PublishResult::WriteFailed;
      err = e.what();
    }
    pub_ok_ = pr == mabur::PublishResult::Ok;
    pub_err_ = pr == mabur::PublishResult::ReparseFailed ? mabur::loader_reason(err) : err;
  };

  l.unlock();
  publish(patched, job.write_backup);
  l.lock();
  phase_ = Phase::Published;
  if (await_step(l) != Step::Restore) return;

  l.unlock();
  publish(original, false);
  l.lock();
  phase_ = Phase::Restored;
}

// --- GsConfigControl -------------------------------------------------------

GsConfigControl::~GsConfigControl() {
  if (fd_ >= 0) ::close(fd_);
}

bool GsConfigControl::open(int port) {
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(port));
  if (inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr) != 1) return false;
  const int fd = ::socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);
  if (fd < 0) return false;
  if (::bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
    ::close(fd);
    return false;
  }
  fd_ = fd;
  return true;
}

void GsConfigControl::poll(GsConfigApplier& a) {
  if (fd_ >= 0) {
    // A full eight-rung ladder set is about 600 bytes.
    char buf[2048];
    for (;;) {
      sockaddr_in from{};
      socklen_t flen = sizeof(from);
      const ssize_t n = ::recvfrom(fd_, buf, sizeof(buf), 0, reinterpret_cast<sockaddr*>(&from), &flen);
      if (n < 0) break;  // drained; a 0-length datagram is still answered
      bool pending = false;
      const std::string reply = handle(std::string(buf, static_cast<size_t>(n)), a, &pending);
      if (pending) {
        has_pending_ = true;
        pending_from_ = from;
        pending_len_ = flen;
        continue;
      }
      ::sendto(fd_, reply.data(), reply.size(), 0, reinterpret_cast<sockaddr*>(&from), flen);
    }
  }
  GsApplyResult r;
  if (!a.poll(&r)) return;
  if (has_pending_ && fd_ >= 0) {
    const std::string reply = r.ok ? "ok" : "err " + r.reason;
    ::sendto(fd_, reply.data(), reply.size(), 0, reinterpret_cast<sockaddr*>(&pending_from_),
             pending_len_);
  }
  has_pending_ = false;
}

std::string GsConfigControl::handle(const std::string& line, GsConfigApplier& a, bool* pending) {
  static const std::string kUsage = "err usage: set <key> = <value> | get [<key>]";
  *pending = false;
  const std::string s = trim(line);
  size_t sp = 0;
  while (sp < s.size() && !std::isspace(static_cast<unsigned char>(s[sp]))) ++sp;
  const std::string verb = s.substr(0, sp);
  const std::string rest = trim(s.substr(sp));

  if (verb == "get") {
    if (rest.empty()) return "ok\n" + a.get_all();
    if (has_space(rest)) return kUsage;
    std::string why;
    const std::string v = a.get(rest, &why);
    return why.empty() ? "ok " + v : "err " + why;
  }
  if (verb == "set") {
    const size_t eq = rest.find('=');
    if (eq == std::string::npos) return kUsage;
    const std::string key = trim(rest.substr(0, eq));
    const std::string value = trim(rest.substr(eq + 1));
    if (key.empty() || value.empty() || has_space(key)) return kUsage;
    std::string why;
    if (!a.submit(key, value, &why)) return "err " + why;
    *pending = true;
    return "";
  }
  return kUsage;
}

}  // namespace maburgs
