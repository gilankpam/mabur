#include "msp_sink.h"
#include "mabur/sbi.h"
#include "mabur/sw_wire.h"  // mabur::sw::kSwHeaderLen

namespace maburgs {

MspSink::MspSink(int symbol_size, int window, EmitFn emit)
    : block_payload_(symbol_size + static_cast<int>(mabur::sw::kSwHeaderLen)),
      dec_(mabur::SwConfig{symbol_size, window, 0.0}),
      emit_(std::move(emit)) {}

void MspSink::on_body(const uint8_t* body, size_t len, uint64_t now_ms) {
  auto u = mabur::sbi_unpack(body, len, block_payload_);
  for (auto& env : u.survivors)
    for (auto& pkt : dec_.add_symbol(env.data(), env.size(), now_ms)) {
      emit_(pkt.data(), pkt.size());
      ++snapshots_out_;
    }
}

void MspSink::tick(uint64_t now_ms) {
  dec_.expire_rows_older_than(2000, now_ms);
}

}  // namespace maburgs
