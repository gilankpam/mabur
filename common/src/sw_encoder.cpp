#include "mabur/sw_encoder.h"

#include <cstring>

#include "mabur/gf256.h"
#include "mabur/sw_wire.h"
#include "mabur/fec_worker.h"

namespace mabur {

SwEncoder::SwEncoder(const SwConfig& cfg, uint32_t initial_seq, FecWorker* worker)
    : cfg_(cfg), worker_(worker), next_seq_(initial_seq) {
  if (cfg_.window < 2) cfg_.window = 2;
  if (cfg_.window > sw::kMaxWindow) cfg_.window = sw::kMaxWindow;
  stride_ = (static_cast<size_t>(cfg_.symbol_size) + 15) & ~static_cast<size_t>(15);
  cap_ = static_cast<size_t>(cfg_.window) + kSlackRows;
  ring_raw_.resize(stride_ * cap_ + 15);
  ring_ = ring_raw_.data();
  ring_ += (16 - (reinterpret_cast<uintptr_t>(ring_) & 15)) & 15;
  if (worker_) async_ = std::make_unique<AsyncState>();
}

SwEncoder::~SwEncoder() {
  if (async_) join();
}

const uint8_t* SwEncoder::row(size_t oldest_i) const {
  const size_t start = (next_slot_ + cap_ - count_) % cap_;
  return ring_ + ((start + oldest_i) % cap_) * stride_;
}

void SwEncoder::append_to_current(const uint8_t* data, size_t len) {
  const uint16_t ln = static_cast<uint16_t>(len);
  current_symbol_.push_back(static_cast<uint8_t>(ln & 0xFF));
  current_symbol_.push_back(static_cast<uint8_t>((ln >> 8) & 0xFF));
  current_symbol_.insert(current_symbol_.end(), data, data + len);
}

std::vector<std::vector<uint8_t>> SwEncoder::add_packet(const uint8_t* data, size_t len) {
  std::vector<std::vector<uint8_t>> out;
  if (async_) drain_done(out);
  if (static_cast<int>(len) > cfg_.max_packet_size()) {
    ++oversize_drops_;
    return out;
  }
  const size_t needed = 2 + len;
  const size_t remaining = static_cast<size_t>(cfg_.symbol_size) - current_symbol_.size();
  if (needed > remaining) seal_current(out);
  append_to_current(data, len);
  return out;
}

void SwEncoder::seal_current(std::vector<std::vector<uint8_t>>& out) {
  if (current_symbol_.empty()) return;
  const size_t ss = static_cast<size_t>(cfg_.symbol_size);
  current_symbol_.resize(ss, 0);

  sw::SwHeader h;
  h.repair = false;
  h.symbol_size = static_cast<uint16_t>(cfg_.symbol_size);
  h.seq = next_seq_;
  std::vector<uint8_t> env;
  env.reserve(sw::kSwHeaderLen + ss);
  sw::pack_header(env, h);
  env.insert(env.end(), current_symbol_.begin(), current_symbol_.end());
  out.push_back(std::move(env));

  // A ring row stays readable for kSlackRows seals after leaving the
  // window; join if a queued job might still reference the row this seal
  // overwrites (backstop — not reached at production rates, where the
  // worker drains jobs in well under 62 seals).
  if (async_ && async_->outstanding.load(std::memory_order_acquire) != 0 &&
      ++seals_since_join_ >= static_cast<long>(kSlackRows) - 2)
    join();
  std::memcpy(ring_ + next_slot_ * stride_, current_symbol_.data(), ss);
  next_slot_ = (next_slot_ + 1) % cap_;
  if (count_ < static_cast<size_t>(cfg_.window)) ++count_;
  current_symbol_.clear();
  ++next_seq_;
  ++sources_out_;
  tail_repair_pending_ = true;

  credit_ += cfg_.overhead;
  while (credit_ >= 1.0) {
    emit_or_enqueue_repair(out);
    credit_ -= 1.0;
  }
}

std::vector<uint8_t> SwEncoder::build_repair(uint32_t repair_key,
                                             uint32_t header_seq,
                                             int window_len,
                                             size_t start_slot) const {
  const size_t ss = static_cast<size_t>(cfg_.symbol_size);
  sw::SwHeader h;
  h.repair = true;
  h.symbol_size = static_cast<uint16_t>(cfg_.symbol_size);
  h.seq = header_seq;
  h.window_len = static_cast<uint8_t>(window_len);
  h.repair_key = repair_key;

  std::vector<uint8_t> env;
  env.reserve(sw::kSwHeaderLen + ss);
  sw::pack_header(env, h);
  const size_t off = env.size();
  env.insert(env.end(), ss, 0);

  uint8_t coeffs[sw::kMaxWindow];
  sw::repair_coeffs(repair_key, window_len, coeffs);
  for (int i = 0; i < window_len; ++i)
    gf::lincomb(env.data() + off,
                ring_ + ((start_slot + static_cast<size_t>(i)) % cap_) * stride_,
                coeffs[i], ss);
  return env;
}

void SwEncoder::emit_or_enqueue_repair(std::vector<std::vector<uint8_t>>& out) {
  const int wl = static_cast<int>(count_);
  FecRepairJob j;
  j.eng = this;
  j.repair_key = repair_key_++;
  j.header_seq = next_seq_ - static_cast<uint32_t>(wl);
  j.window_len = wl;
  j.start_slot = (next_slot_ + cap_ - static_cast<size_t>(wl)) % cap_;
  ++repairs_out_;
  tail_repair_pending_ = false;
  if (!async_) {
    out.push_back(build_repair(j.repair_key, j.header_seq, j.window_len, j.start_slot));
    return;
  }
  if (async_->outstanding.load(std::memory_order_relaxed) == 0)
    seals_since_join_ = 0;
  async_->outstanding.fetch_add(1, std::memory_order_relaxed);
  if (!worker_->try_enqueue(j)) execute_repair_job(j);  // full: inline
}

void SwEncoder::execute_repair_job(const FecRepairJob& j) {
  auto env = build_repair(j.repair_key, j.header_seq, j.window_len, j.start_slot);
  {
    std::lock_guard<std::mutex> l(async_->done_m);
    async_->done.push_back(std::move(env));
  }
  async_->outstanding.fetch_sub(1, std::memory_order_release);
}

void SwEncoder::drain_done(std::vector<std::vector<uint8_t>>& out) {
  std::lock_guard<std::mutex> l(async_->done_m);
  for (auto& e : async_->done) out.push_back(std::move(e));
  async_->done.clear();
}

void SwEncoder::join() {
  while (async_->outstanding.load(std::memory_order_relaxed) != 0) { /* spin */ }
  std::atomic_thread_fence(std::memory_order_acquire);
  seals_since_join_ = 0;
}

std::vector<std::vector<uint8_t>> SwEncoder::flush() {
  std::vector<std::vector<uint8_t>> out;
  seal_current(out);
  if (tail_repair_pending_ && count_ > 0) emit_or_enqueue_repair(out);
  if (async_) {
    join();
    drain_done(out);
  }
  return out;
}

}  // namespace mabur
