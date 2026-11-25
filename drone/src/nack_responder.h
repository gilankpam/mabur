#pragma once
// Software NACK answer (fec-nack), RX thread. Verified request -> ring
// lookups -> fresh SBI bodies marked retx -> TxQueue head, under a token
// bucket of nack.air_pct of the base layer's delivered capacity,
// nack.burst_ms of that capacity deep (flight 0026: a fixed 64-symbol depth
// was 4 ms of air at rung 5 and 30 ms at rung 0, and refused half of every
// cascade's requests). A repeat-flagged request is answered twice (and
// draws two tokens per symbol). The bucket, packer and scratch envelope are
// RX-thread-only; the ring is lock-free. The caller pushes the bodies as
// UepBody stream_id 0, so RadioTx::build_frame and the air clock see plain
// base-layer bodies; the retx mark lives only in the SBI stream byte, which
// the GS routes as base and decodes as a retransmit.
#include <cstddef>
#include <cstdint>
#include <vector>

#include "config.h"
#include "mabur/rc_proto.h"
#include "mabur/retx_ring.h"
#include "mabur/sbi.h"
#include "nack_bucket.h"
#include "rc_agent.h"

namespace mabur {

class NackResponder {
 public:
  struct Tally { uint64_t syms = 0, refused = 0, miss = 0; };

  NackResponder(size_t env_len, int base_blocks_per_body);

  // Answers a verified request. op == nullptr (no op applied yet) keeps the
  // bucket's depth and refills at rate 0. *push_order is cleared, then holds
  // the bodies in push_front order: push_front stacks, so each pass (two if
  // repeat-flagged) is listed back-to-front and the bodies leave in request
  // order (pass 1 then pass 2) ahead of queued video.
  Tally answer(const rc::Nack& n, const AppliedOp* op, const AirClockCfg& air, const NackDroneCfg& cfg,
               uint64_t now_us, const RetxRing& ring, std::vector<std::vector<uint8_t>>* push_order);

 private:
  size_t env_len_;
  TokenBucket bucket_{64.0};  // re-sized from the op at every refill
  SbiPacker packer_;
  std::vector<uint8_t> env_;
};

}  // namespace mabur
