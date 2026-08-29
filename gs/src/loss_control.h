#pragma once
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "loss_sim.h"

namespace maburgs {

// BENCH RIG — compiled only under MABUR_LOSS_SIM (CMake option, default
// OFF; prod maburgs contains none of this). Mainlined 2026-08-29 so
// resilience gates can build it with -DMABUR_LOSS_SIM=ON instead of
// merging a scaffolding branch.
//
// Loopback-only UDP command listener for retuning LossSim live, so an operator
// can walk loss upward while watching maburplay instead of restarting maburgs
// between every step of a sweep.
//
// Binds 127.0.0.1 ONLY. The GS answers on 10.18.0.1 and this must never be
// reachable from the drone or a bench laptop.
//
// Non-blocking; poll() is called from the core loop next to Aggregator::poll,
// so command handling runs on the thread that owns the LossSim. No locking.
//
// TWO RATES, ALWAYS BOTH REPORTED
// ------------------------------
// LossSim injects independently per (card, sid) — real fading is per-card, and
// the multi-card union is exactly what the diversity exists to exploit. A body
// therefore only reaches the FEC decoder as *lost* when EVERY card that heard
// it dropped it, so with N cards the loss the decoder sees is roughly the
// per-card rate raised to the Nth power. On the 2-card bench GS, dialling 20%
// per card delivers ~4% to the decoder; dialling 50% delivers ~25%. Reporting
// only the per-card number invites an operator to write down "s3 tolerates 50%
// loss" for what was really a 25% test, so every reply here states BOTH:
//
//   percard=<pct>   what each card independently injects (what `loss=` sets)
//   eff=<pct>       the NOMINAL union rate seen by the decoder (percard^ncards,
//                   what `eff=` sets)
//
// `eff` is NOMINAL, not measured: it assumes every card heard every body, which
// is only true on a clean link. When the real link is already losing bodies on
// one card, those bodies have fewer surviving copies and the true injected loss
// is HIGHER than percard^N. So eff is an estimate good enough for dialling and
// nothing more — the loss actually written into a findings document must be
// read from the stats sideport's s3 stream counters, never from this dial.
// That is what the `note=eff-nominal` token on every reply is there to remind
// whoever greps the log.
class LossControl {
 public:
  ~LossControl() { if (fd_ >= 0) close(fd_); }
  LossControl() = default;
  LossControl(const LossControl&) = delete;
  LossControl& operator=(const LossControl&) = delete;

  bool ok() const { return fd_ >= 0; }

  // `n_cards` is the number of receive cards actually feeding the union; it is
  // what turns a per-card rate into an effective one and back. Aggregator
  // knows it, so main.cpp passes Aggregator::n_cards() straight through.
  bool open(int port, int n_cards) {
    n_cards_ = n_cards;
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port));
    if (inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr) != 1) return false;
    const int fd = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);
    if (fd < 0) return false;
    if (bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
      close(fd);
      return false;
    }
    fd_ = fd;
    return true;
  }

  // Drains every pending datagram, applies it, and replies to the sender.
  void poll(LossSim& sim) {
    if (fd_ < 0) return;
    char buf[512];
    for (;;) {
      sockaddr_in from{};
      socklen_t flen = sizeof(from);
      const ssize_t n = recvfrom(fd_, buf, sizeof(buf) - 1, 0,
                                 reinterpret_cast<sockaddr*>(&from), &flen);
      if (n < 0) return;    // EAGAIN/error when drained -- a genuine 0-length
                            // datagram must still be answered, not treated
                            // as end-of-drain (that would leave any
                            // datagrams still queued behind it unanswered
                            // until the next tick).
      buf[n] = '\0';
      std::string reply;
      const bool changed = apply(buf, sim, n_cards_, &reply);
      sendto(fd_, reply.data(), reply.size(), 0,
             reinterpret_cast<sockaddr*>(&from), flen);
      if (changed) {
        // Loud, on every change: /tmp/maburgs.log is the post-mortem surface,
        // and a recording made with injection on must never read as a real
        // bad link.
        std::fprintf(stderr,
                     "maburgs: LOSS-SIM %s (INJECTED loss, not real) -> %s\n",
                     buf, reply.c_str());
        std::fflush(stderr);
      }
    }
  }

  // Parses one command and mutates `sim`. Returns true iff state changed.
  // `*reply` always gets the datagram to send back. Static and I/O-free so the
  // command language is unit-testable without a socket — hence `n_cards` as a
  // parameter rather than a member read.
  //
  // Command language:
  //   sN loss=<pct> [burst=<n>]   set the PER-CARD rate (unchanged meaning)
  //   sN eff=<pct>  [burst=<n>]   set the NOMINAL EFFECTIVE (union) rate;
  //                               per-card is solved as eff^(1/ncards)
  //   sN off | off | reset | status
  static bool apply(const std::string& line, LossSim& sim, int n_cards,
                    std::string* reply) {
    const std::vector<std::string> tok = split(line);
    if (tok.empty()) { *reply = "err empty"; return false; }

    if (tok[0] == "status") { *reply = status(sim, n_cards); return false; }

    if (tok[0] == "off" || tok[0] == "reset") {
      for (int s = 0; s < LossSim::kStreams; ++s) sim.configure(s, 0.0, 1.0);
      *reply = "ok all streams zero";
      return true;
    }

    // sN ...
    if (tok[0].size() != 2 || tok[0][0] != 's' ||
        tok[0][1] < '0' || tok[0][1] >= '0' + LossSim::kStreams) {
      *reply = "err want s0..s3 | off | status";
      return false;
    }
    const int sid = tok[0][1] - '0';

    if (tok.size() >= 2 && tok[1] == "off") {
      sim.configure(sid, 0.0, 1.0);
      *reply = fmt(sid, sim, n_cards);
      return true;
    }

    double loss_pct = -1.0, eff_pct = -1.0, burst = 1.0;
    for (size_t i = 1; i < tok.size(); ++i) {
      double v = 0.0;
      if (kv(tok[i], "loss=", &v)) { loss_pct = v; continue; }
      if (kv(tok[i], "eff=", &v)) { eff_pct = v; continue; }
      if (kv(tok[i], "burst=", &v)) { burst = v; continue; }
      *reply = "err bad token: " + tok[i];
      return false;
    }
    // Accepting both would leave it to token order which one won, and the
    // operator would never see which knob actually moved.
    if (loss_pct >= 0.0 && eff_pct >= 0.0) {
      *reply = "err loss= and eff= are exclusive";
      return false;
    }
    if (loss_pct < 0.0 && eff_pct < 0.0) {
      *reply = "err want loss=<pct-per-card> | eff=<pct-union> [burst=<n>]";
      return false;
    }

    const double percard = eff_pct >= 0.0 ? percard_from_eff(eff_pct / 100.0, n_cards)
                                          : loss_pct / 100.0;
    sim.configure(sid, percard, burst);
    *reply = fmt(sid, sim, n_cards);
    return true;
  }

  // Cards feeding the union, clamped to something the model can honour: at
  // least one, and never more than LossSim tracks state for (cards beyond
  // kMaxCards never drop at all, so counting them would overstate eff).
  static int clamp_cards(int n) {
    if (n < 1) return 1;
    if (n > LossSim::kMaxCards) return LossSim::kMaxCards;
    return n;
  }

  // NOMINAL union rate: independent per-card injection means a body is only
  // lost when all N cards drop it. Assumes every card heard every body — see
  // the class comment; on a link already losing bodies the true figure is
  // higher, so this is a dialling aid, not a measurement.
  static double eff_from_percard(double percard, int n_cards) {
    if (percard <= 0.0) return 0.0;
    return std::pow(percard, static_cast<double>(clamp_cards(n_cards)));
  }

  // Inverse of the above: what each card must inject for the union to land on
  // `eff`. Same nominal caveat.
  static double percard_from_eff(double eff, int n_cards) {
    if (eff <= 0.0) return 0.0;
    return std::pow(eff, 1.0 / static_cast<double>(clamp_cards(n_cards)));
  }

 private:
  static std::vector<std::string> split(const std::string& s) {
    std::vector<std::string> out;
    size_t i = 0;
    while (i < s.size()) {
      while (i < s.size() && std::isspace(static_cast<unsigned char>(s[i]))) ++i;
      const size_t start = i;
      while (i < s.size() && !std::isspace(static_cast<unsigned char>(s[i]))) ++i;
      if (i > start) out.push_back(s.substr(start, i - start));
    }
    return out;
  }

  // Strict numeric parse: "loss=abc" must be rejected, not silently read as 0.
  static bool kv(const std::string& tok, const char* key, double* out) {
    const size_t klen = std::strlen(key);
    if (tok.size() <= klen || tok.compare(0, klen, key) != 0) return false;
    const std::string val = tok.substr(klen);
    char* end = nullptr;
    const double v = std::strtod(val.c_str(), &end);
    if (end == val.c_str() || *end != '\0') return false;
    *out = v;
    return true;
  }

  // One stream's rates, both of them, labelled. Format is a contract: the
  // Python driver greps it and the operator's sweep log preserves it, so it
  // must stay stable for as long as tools/bench/losssim.py parses it.
  //   ok s3 percard=14.14 eff=2.000 burst=3.0 ncards=2 note=eff-nominal
  static std::string fmt(int sid, const LossSim& sim, int n_cards) {
    char b[160];
    std::snprintf(b, sizeof(b),
                  "ok s%d percard=%.2f eff=%.3f burst=%.1f ncards=%d "
                  "note=eff-nominal",
                  sid, sim.loss(sid) * 100.0,
                  eff_from_percard(sim.loss(sid), n_cards) * 100.0,
                  sim.burst(sid), clamp_cards(n_cards));
    return b;
  }

  // Every stream, both rates each, plus the injected-drop counters (which are
  // summed over cards, so they count per-card drops, NOT bodies lost to the
  // decoder — another reason the real number comes from the sideport).
  //   ok ncards=2 s0[percard=0.00 eff=0.000 burst=1.0] s1[...] s2[...]
  //   s3[percard=14.14 eff=2.000 burst=3.0] drops=0,0,0,912 note=eff-nominal
  static std::string status(const LossSim& sim, int n_cards) {
    char head[48];
    std::snprintf(head, sizeof(head), "ok ncards=%d", clamp_cards(n_cards));
    std::string out = head;
    for (int s = 0; s < LossSim::kStreams; ++s) {
      char b[96];
      std::snprintf(b, sizeof(b), " s%d[percard=%.2f eff=%.3f burst=%.1f]", s,
                    sim.loss(s) * 100.0,
                    eff_from_percard(sim.loss(s), n_cards) * 100.0,
                    sim.burst(s));
      out += b;
    }
    out += " drops=";
    for (int s = 0; s < LossSim::kStreams; ++s) {
      char b[32];
      std::snprintf(b, sizeof(b), "%s%llu", s ? "," : "",
                    static_cast<unsigned long long>(sim.dropped(s)));
      out += b;
    }
    out += " note=eff-nominal";
    return out;
  }

  int fd_ = -1;
  int n_cards_ = 1;
};

}  // namespace maburgs
