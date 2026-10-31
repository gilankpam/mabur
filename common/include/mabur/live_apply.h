#pragma once
// The live-config pipeline both GS daemons run: one `set <key> = <value>`
// from a loopback listener, put into the running process and then into the
// daemon's own TOML config, in the drone's order (drone/src/config_apply.h)
// and through the same writer (mabur/config_write.h):
//
//   re-read the file -> patch the key's line(s) -> load the result with the
//   boot loader (the candidate) -> apply the candidate to the process ->
//   publish
//
// with one split the drone does not need: the file work runs on a worker
// thread, while the apply runs on the thread that owns what the key
// changes (maburgs' core loop, maburplay's housekeeping loop). That owning
// thread never blocks on file I/O or on the worker: poll() picks up
// whatever the worker finished and hands it the next step. The worker
// holds lock_config_writes() from the re-read through the publish, so
// another writer of the same file (a calibration publish) can neither slip
// between them nor be overwritten by a stale read.
//
// WARNING: the owning thread must never take lock_config_writes() itself,
// nor call publish_config(), which takes it: the worker holds that lock
// while it waits for the owner's next step, so an owner blocked on it
// waits for a worker that is waiting for the owner. Hand such a publish to
// a thread of its own.
//
// The file is published only after the process accepted the candidate: a
// refused or failed apply leaves the file byte-identical and writes no
// ".pre-live" backup (one per run, before the first publish). A publish
// that fails after a successful apply re-applies the previous config,
// putting the process back in line with the unchanged file. A key whose
// apply is an order placed only once the file holds it (maburgs'
// radio.channel) skips the apply before the publish; an order refused
// after the publish publishes the previous text again.
//
// A set equal to the live value answers ok and writes nothing: caught at
// submit when the text matches the live rendering, and otherwise once the
// candidate is loaded, by comparing the candidate's rendering of the key
// with the live one (0.50 against a live 0.5, a respaced ladder).
//
// What is daemon-specific -- the key table, how a key renders and patches,
// the loader, extra refusals, the apply -- comes in through LiveApplySpec;
// each daemon wraps the engine with its own hook struct
// (gs/src/gs_config_apply.h, gs/player/src/player_config_apply.h).
#include <arpa/inet.h>
#include <netinet/in.h>
#include <pthread.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cctype>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <system_error>
#include <thread>
#include <utility>
#include <vector>

#include "mabur/config_write.h"

namespace mabur {

struct LiveApplyResult {
  bool ok;
  std::string reason;  // empty on ok
  std::string key;
  uint64_t ms;         // submit to result
};

namespace live_apply_detail {

inline std::string trim(const std::string& s) {
  size_t b = 0, e = s.size();
  while (b < e && std::isspace(static_cast<unsigned char>(s[b]))) ++b;
  while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1]))) --e;
  return s.substr(b, e - b);
}

inline bool has_space(const std::string& s) {
  for (char c : s)
    if (std::isspace(static_cast<unsigned char>(c))) return true;
  return false;
}

// The config's real path: the candidate scratch goes beside the file a
// symlinked config points at, where publish_config writes its own.
inline std::string real_path_of(const std::string& path) {
  char* r = ::realpath(path.c_str(), nullptr);
  if (!r) return path;
  std::string out = r;
  std::free(r);
  return out;
}

}  // namespace live_apply_detail

// One daemon's half of the pipeline. `Key` is its live-key row, with at
// least `const char* key` (the dotted name). Functions marked optional may
// be left empty.
template <class Config, class Key>
struct LiveApplySpec {
  const char* daemon = "";       // leads every log line: "<daemon>: cfg set ..."
  const char* thread_name = "";  // the worker's name, at most 15 chars
  const std::vector<Key>* table = nullptr;  // the closed live table, in `get` order

  // The live value as the file would write it (strings quoted).
  std::function<std::string(const Config&, const Key&)> render;

  // Worker thread.
  // Patches `text` for one key; false with *why, `*text` unchanged.
  std::function<bool(std::string* text, const Key&, const std::string& value, std::string* why)>
      patch;
  // The boot loader. Throws with its message on a config it refuses.
  std::function<Config(const std::string& path)> load;
  // Optional: "" or why a config the loader took still cannot run here.
  // Checked on the candidate and again by the publish's verifier.
  std::function<std::string(const Config&)> refuse_config;
  // Optional: "" or why this one set is refused, given the loaded candidate.
  std::function<std::string(const Key&, const std::string& value, const Config& cand)> refuse_set;

  // Owning thread.
  // Copies the one field `k` names from `src` into `dst`.
  std::function<void(const Key&, const Config& src, Config* dst)> copy_field;
  // Optional: a refusal decided at submit, before any file work.
  std::function<std::optional<std::string>(const Key&)> precheck;
  // Puts `to` (the live config with `k` changed) into the process, `from`
  // being what it runs now. False with *reason when it refuses; the revert
  // after a failed publish runs it with the two swapped.
  std::function<bool(const Key&, const Config& from, const Config& to, std::string* reason)> apply;
  // Optional: true for a key applied by an order placed after the publish
  // (`order`), not by `apply` before it.
  std::function<bool(const Key&)> order_after_publish;
  // The order for such a key; a value is its refusal.
  std::function<std::optional<std::string>(const Key&, const Config&)> order;
};

template <class Config, class Key>
class LiveApplyEngine {
 public:
  using Spec = LiveApplySpec<Config, Key>;

  // `live` is the config the process runs (the one it booted with). `log`,
  // when set, receives one "<daemon>: cfg set ..." line per set, plus a
  // line for each revert.
  LiveApplyEngine(std::string cfg_path, Config live, Spec spec,
                  std::function<void(const std::string&)> log)
      : cfg_path_(std::move(cfg_path)),
        live_(std::move(live)),
        spec_(std::move(spec)),
        log_(std::move(log)) {}
  ~LiveApplyEngine();  // lets the worker finish its step, then joins it
  LiveApplyEngine(const LiveApplyEngine&) = delete;
  LiveApplyEngine& operator=(const LiveApplyEngine&) = delete;

  // Owning thread. Starts applying `value` (TOML text, a string carrying
  // its own quotes) to `key`. False with *why, nothing started, for a key
  // that is not live ("not live: <key>"), while a set is in flight
  // ("busy"), or for a value that is empty or spans lines. Two outcomes
  // are settled here with no file work, their result handed out by the
  // next poll(): a spec precheck refusal, and a value whose text equals
  // the live rendering (ok). A set is in flight from an accepted submit
  // until poll() hands out its result.
  bool submit(const std::string& key, const std::string& value, std::string* why);
  // Owning thread, every tick: applies a candidate the worker has ready,
  // hands an applied candidate to the publish, re-applies the previous
  // config when a publish failed, and returns true once with the result.
  bool poll(LiveApplyResult* out);
  bool busy() const { return in_flight_; }      // owning thread
  const Config& live() const { return live_; }  // owning thread
  // Owning thread: the live value, or "" with *why ("not live: <key>").
  std::string get(const std::string& key, std::string* why) const;
  std::string get_all() const;  // "<key> = <value>\n" per live key, in table order

 private:
  // The worker's position in one set. The owner moves it on from the
  // states marked (owner); the worker from the rest.
  enum class Phase {
    Idle,
    Reading,     // worker: re-read, patch, load the candidate
    Refused,     // (owner) the worker refused: no candidate
    Ready,       // (owner) candidate loaded, waiting for the apply
    Publishing,  // worker
    Published,   // (owner) publish outcome in pub_ok_/pub_err_
    Restoring,   // worker: publishing the previous text again
    Restored,    // (owner)
  };
  // What the owner tells the worker holding the write lock to do next.
  enum class Step { None, Publish, Restore, Release };

  struct Job {
    const Key* key = nullptr;
    std::string value;
    bool write_backup = false;
  };

  const Key* find(const std::string& dotted) const {
    for (const Key& k : *spec_.table)
      if (dotted == k.key) return &k;
    return nullptr;
  }
  bool order_after_publish(const Key& k) const {
    return spec_.order_after_publish && spec_.order_after_publish(k);
  }
  void run();
  void work(const Job& job);
  // Waits (worker, mu_ held via `l`) for the owner's next step.
  Step await_step(std::unique_lock<std::mutex>& l);
  void tell(Step s, Phase next);
  void finish(bool ok, const std::string& reason, LiveApplyResult* out);
  void log(const std::string& line) const {
    if (log_) log_(line);
  }
  // publish_config's verifier: the boot loader plus refuse_config, so the
  // file is never replaced by one the daemon would refuse to start from.
  bool verify(const std::string& candidate_path, std::string* err) const;

  const std::string cfg_path_;
  Config live_;
  const Spec spec_;
  std::function<void(const std::string&)> log_;

  // Owning thread only.
  bool in_flight_ = false;
  const Key* cur_key_ = nullptr;
  std::string cur_value_;
  std::chrono::steady_clock::time_point t0_{};
  std::optional<LiveApplyResult> settled_;  // a result submit() decided
  Config cand_;                             // the applied candidate, until it is live
  std::string restore_reason_;              // an order refused after the publish
  bool backup_written_ = false;

  // Shared with the worker, under mu_.
  mutable std::mutex mu_;
  std::condition_variable cv_;
  std::thread thread_;
  Job job_;
  bool has_job_ = false;
  bool stop_ = false;
  Phase phase_ = Phase::Idle;
  Step step_ = Step::None;
  std::string refusal_;  // Refused
  Config ready_cand_;    // Ready
  bool pub_ok_ = false;  // Published, Restored
  std::string pub_err_;  // Published, Restored
};

// Loopback-only UDP listener for `set` / `get`. Binds 127.0.0.1 ONLY: a
// set changes the running daemon and rewrites its config, so whoever sends
// one must be logged into the GS itself.
//
// Replies: "ok", "ok <value>", "ok\n<key> = <value>\n..." (a bare `get`),
// "err <reason>" ("err busy", "err not live: <key>", the usage line, a
// datagram too long to read whole).
template <class Applier>
class LiveConfigControl {
 public:
  // The largest datagram read; a full eight-rung maburgs ladder set is
  // about 600 bytes.
  static constexpr size_t kMaxDatagram = 2048;

  LiveConfigControl() = default;
  ~LiveConfigControl() {
    if (fd_ >= 0) ::close(fd_);
  }
  LiveConfigControl(const LiveConfigControl&) = delete;
  LiveConfigControl& operator=(const LiveConfigControl&) = delete;

  bool open(int port);  // 127.0.0.1 only
  bool ok() const { return fd_ >= 0; }

  // Owning thread, every tick: drains datagrams (each answered at once
  // unless handle() set *pending, whose sender address is kept), then
  // calls a.poll() and, when a result arrives, answers the kept sender
  // with "ok" / "err <reason>". One pending sender at a time: the
  // applier's busy refusal keeps a second `set` from queueing. The
  // cfg-set log line is the applier's.
  void poll(Applier& a);

  // One line in, one reply out; "" with *pending = true when a `set` was
  // accepted and its answer comes from a later poll(). Static and I/O-free
  // so the command language is testable without a socket.
  //   set <key> = <value>
  //   get [<key>]
  static std::string handle(const std::string& line, Applier& a, bool* pending);

 private:
  int fd_ = -1;
  bool has_pending_ = false;
  sockaddr_in pending_from_{};
  socklen_t pending_len_ = 0;
};

// --- LiveApplyEngine --------------------------------------------------------

template <class Config, class Key>
LiveApplyEngine<Config, Key>::~LiveApplyEngine() {
  {
    std::lock_guard<std::mutex> l(mu_);
    stop_ = true;
  }
  cv_.notify_all();
  // A worker waiting for the owner's next step takes stop_ as "release":
  // nothing more is published.
  if (thread_.joinable()) thread_.join();
}

template <class Config, class Key>
std::string LiveApplyEngine<Config, Key>::get(const std::string& key, std::string* why) const {
  const Key* k = find(key);
  if (!k) {
    if (why) *why = "not live: " + key;
    return "";
  }
  return spec_.render(live_, *k);
}

template <class Config, class Key>
std::string LiveApplyEngine<Config, Key>::get_all() const {
  std::string out;
  for (const Key& k : *spec_.table) out += std::string(k.key) + " = " + spec_.render(live_, k) + "\n";
  return out;
}

template <class Config, class Key>
bool LiveApplyEngine<Config, Key>::submit(const std::string& key, const std::string& value,
                                          std::string* why) {
  auto refuse = [&](const std::string& text) {
    if (why) *why = text;
    return false;
  };
  const Key* k = find(key);
  if (!k) return refuse("not live: " + key);
  if (in_flight_) return refuse("busy");
  const std::string v = live_apply_detail::trim(value);
  if (v.empty()) return refuse("empty value");
  if (v.find_first_of("\r\n") != std::string::npos) return refuse("value spans lines");

  cur_key_ = k;
  cur_value_ = v;
  t0_ = std::chrono::steady_clock::now();
  settled_.reset();
  std::optional<std::string> pre;
  if (spec_.precheck) pre = spec_.precheck(*k);
  if (pre) {
    settled_ = LiveApplyResult{false, *pre, k->key, 0};
  } else if (v == spec_.render(live_, *k)) {
    settled_ = LiveApplyResult{true, "", k->key, 0};
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

template <class Config, class Key>
void LiveApplyEngine<Config, Key>::tell(Step s, Phase next) {
  {
    std::lock_guard<std::mutex> l(mu_);
    step_ = s;
    phase_ = next;
  }
  cv_.notify_all();
}

template <class Config, class Key>
bool LiveApplyEngine<Config, Key>::poll(LiveApplyResult* out) {
  if (!in_flight_) return false;
  if (settled_) {
    finish(settled_->ok, settled_->reason, out);
    return true;
  }
  const Key& k = *cur_key_;
  const bool ordered = order_after_publish(k);
  auto prefix = [&] { return std::string(spec_.daemon) + ": cfg set " + k.key; };
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
      spec_.copy_field(k, cand, &cand_);
      if (spec_.render(cand_, k) == spec_.render(live_, k)) {
        // The live value, spelled differently: nothing to apply or write.
        tell(Step::Release, Phase::Idle);
        finish(true, "", out);
        return true;
      }
      std::string reason;
      if (ordered || spec_.apply(k, live_, cand_, &reason)) {
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
        if (!ordered) {
          log(prefix() + ": publish failed, reverting");
          std::string rr;
          if (!spec_.apply(k, cand_, live_, &rr)) log(prefix() + ": revert failed: " + rr);
        }
        tell(Step::Release, Phase::Idle);
        finish(false, "publish failed: " + err, out);
        return true;
      }
      backup_written_ = true;
      if (ordered) {
        std::optional<std::string> refused = spec_.order(k, cand_);
        if (refused) {
          // The file now claims a value the process did not take: the
          // worker, still holding the write lock, publishes the previous
          // text.
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
      if (!ok) log(prefix() + ": restoring the previous file failed: " + err);
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

template <class Config, class Key>
void LiveApplyEngine<Config, Key>::finish(bool ok, const std::string& reason,
                                          LiveApplyResult* out) {
  const uint64_t ms = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0_)
          .count());
  LiveApplyResult r{ok, ok ? "" : reason, cur_key_->key, ms};
  log(std::string(spec_.daemon) + ": cfg set " + r.key + " = " + cur_value_ + " -> " +
      (ok ? std::string("ok") : "err " + r.reason) + " (" + std::to_string(ms) + " ms)");
  settled_.reset();
  in_flight_ = false;
  *out = std::move(r);
}

template <class Config, class Key>
void LiveApplyEngine<Config, Key>::run() {
  pthread_setname_np(pthread_self(), spec_.thread_name);
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

template <class Config, class Key>
typename LiveApplyEngine<Config, Key>::Step LiveApplyEngine<Config, Key>::await_step(
    std::unique_lock<std::mutex>& l) {
  cv_.wait(l, [this] { return stop_ || step_ != Step::None; });
  const Step s = step_ == Step::None ? Step::Release : step_;
  step_ = Step::None;
  return s;
}

template <class Config, class Key>
bool LiveApplyEngine<Config, Key>::verify(const std::string& candidate_path,
                                          std::string* err) const {
  try {
    const Config c = spec_.load(candidate_path);
    const std::string why = spec_.refuse_config ? spec_.refuse_config(c) : "";
    if (why.empty()) return true;
    if (err) *err = why;
    return false;
  } catch (const std::exception& e) {
    if (err) *err = e.what();
    return false;
  }
}

template <class Config, class Key>
void LiveApplyEngine<Config, Key>::work(const Job& job) {
  const Key& k = *job.key;
  std::string original, patched, refusal;
  Config cand;
  bool ready = false;
  // Held from the re-read through the publish (and a restore): another
  // writer's publish between them would be overwritten by this one's stale
  // text. Declared before `l` so mu_ is let go first.
  auto write_lock = lock_config_writes();
  try {
    if (!read_text_file(cfg_path_, &original)) {
      refusal = "config file unreadable";
    } else {
      patched = original;
      if (spec_.patch(&patched, k, job.value, &refusal)) {
        // The candidate is the patched text as the boot loader reads it:
        // the process is never handed a config the daemon would refuse to
        // start from.
        const std::string tmp = live_apply_detail::real_path_of(cfg_path_) + ".new";
        if (!write_file_fsync(tmp, patched)) {
          refusal = "failed to write " + tmp;
        } else {
          try {
            cand = spec_.load(tmp);
            ready = true;
          } catch (const std::exception& e) {
            refusal = loader_reason(e.what());
          }
        }
        // Gone before the apply: a refused set leaves no trace, and
        // publish_config writes its own copy.
        ::unlink(tmp.c_str());
      }
    }
    if (ready && spec_.refuse_config) {
      refusal = spec_.refuse_config(cand);
      ready = refusal.empty();
    }
    if (ready && spec_.refuse_set) {
      refusal = spec_.refuse_set(k, job.value, cand);
      ready = refusal.empty();
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

  // Runs with mu_ released (the publish is slow file I/O); its outcome is
  // stored once mu_ is held again.
  auto publish = [&](const std::string& content, bool write_backup) {
    std::string err;
    PublishResult pr;
    try {
      pr = publish_config(
          cfg_path_, content, "pre-live", &err,
          [this](const std::string& p, std::string* e) { return verify(p, e); }, write_backup);
    } catch (const std::exception& e) {
      pr = PublishResult::WriteFailed;
      err = e.what();
    }
    l.lock();
    pub_ok_ = pr == PublishResult::Ok;
    pub_err_ = pr == PublishResult::ReparseFailed ? loader_reason(err) : err;
  };

  l.unlock();
  publish(patched, job.write_backup);
  phase_ = Phase::Published;
  if (await_step(l) != Step::Restore) return;

  l.unlock();
  publish(original, false);
  phase_ = Phase::Restored;
}

// --- LiveConfigControl ------------------------------------------------------

template <class Applier>
bool LiveConfigControl<Applier>::open(int port) {
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

template <class Applier>
void LiveConfigControl<Applier>::poll(Applier& a) {
  if (fd_ >= 0) {
    char buf[kMaxDatagram];
    for (;;) {
      sockaddr_in from{};
      socklen_t flen = sizeof(from);
      // MSG_TRUNC: the return is the datagram's full length, so one longer
      // than the buffer is refused instead of acted on as its first bytes.
      const ssize_t n = ::recvfrom(fd_, buf, sizeof(buf), MSG_TRUNC,
                                   reinterpret_cast<sockaddr*>(&from), &flen);
      if (n < 0) break;  // drained; a 0-length datagram is still answered
      bool pending = false;
      const std::string reply =
          static_cast<size_t>(n) > sizeof(buf)
              ? "err line too long (max " + std::to_string(sizeof(buf)) + " bytes)"
              : handle(std::string(buf, static_cast<size_t>(n)), a, &pending);
      if (pending) {
        has_pending_ = true;
        pending_from_ = from;
        pending_len_ = flen;
        continue;
      }
      ::sendto(fd_, reply.data(), reply.size(), 0, reinterpret_cast<sockaddr*>(&from), flen);
    }
  }
  LiveApplyResult r;
  if (!a.poll(&r)) return;
  if (has_pending_ && fd_ >= 0) {
    const std::string reply = r.ok ? "ok" : "err " + r.reason;
    ::sendto(fd_, reply.data(), reply.size(), 0, reinterpret_cast<sockaddr*>(&pending_from_),
             pending_len_);
  }
  has_pending_ = false;
}

template <class Applier>
std::string LiveConfigControl<Applier>::handle(const std::string& line, Applier& a,
                                               bool* pending) {
  using live_apply_detail::has_space;
  using live_apply_detail::trim;
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

}  // namespace mabur
