#include "mabur/sw_encoder.h"

#include "mabur/gf256.h"
#include "mabur/sw_wire.h"

namespace mabur {

SwEncoder::SwEncoder(const SwConfig& cfg) : cfg_(cfg) {
  if (cfg_.window < 2) cfg_.window = 2;
  if (cfg_.window > sw::kMaxWindow) cfg_.window = sw::kMaxWindow;
}

void SwEncoder::append_to_current(const uint8_t* data, size_t len) {
  const uint16_t ln = static_cast<uint16_t>(len);
  current_symbol_.push_back(static_cast<uint8_t>(ln & 0xFF));
  current_symbol_.push_back(static_cast<uint8_t>((ln >> 8) & 0xFF));
  current_symbol_.insert(current_symbol_.end(), data, data + len);
}

std::vector<std::vector<uint8_t>> SwEncoder::add_packet(const uint8_t* data, size_t len) {
  std::vector<std::vector<uint8_t>> out;
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

  window_.push_back(std::move(current_symbol_));
  current_symbol_.clear();
  while (window_.size() > static_cast<size_t>(cfg_.window)) window_.pop_front();
  ++next_seq_;
  ++sources_out_;
  tail_repair_pending_ = true;

  credit_ += cfg_.overhead;
  while (credit_ >= 1.0) {
    out.push_back(make_repair());
    credit_ -= 1.0;
  }
}

std::vector<uint8_t> SwEncoder::make_repair() {
  const size_t ss = static_cast<size_t>(cfg_.symbol_size);
  const int wl = static_cast<int>(window_.size());
  sw::SwHeader h;
  h.repair = true;
  h.symbol_size = static_cast<uint16_t>(cfg_.symbol_size);
  h.seq = next_seq_ - static_cast<uint32_t>(wl);  // window_start
  h.window_len = static_cast<uint8_t>(wl);
  h.repair_key = repair_key_++;

  std::vector<uint8_t> env;
  env.reserve(sw::kSwHeaderLen + ss);
  sw::pack_header(env, h);
  const size_t off = env.size();
  env.insert(env.end(), ss, 0);

  uint8_t coeffs[sw::kMaxWindow];
  sw::repair_coeffs(h.repair_key, wl, coeffs);
  for (int i = 0; i < wl; ++i)
    gf::lincomb(env.data() + off, window_[static_cast<size_t>(i)].data(), coeffs[i], ss);

  ++repairs_out_;
  tail_repair_pending_ = false;
  return env;
}

std::vector<std::vector<uint8_t>> SwEncoder::flush() {
  std::vector<std::vector<uint8_t>> out;
  seal_current(out);
  if (tail_repair_pending_ && !window_.empty()) out.push_back(make_repair());
  return out;
}

}  // namespace mabur
