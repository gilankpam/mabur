// Real mode: the RX callback (real_mode.h).

#include <atomic>
#include <cstdint>
#include <memory>
#include <vector>

#include "nack_responder.h"
#include "real_mode.h"
#include "rx_classify.h"
#include "RxPacket.h"

namespace mabur::maburd {

using namespace mabur;

// RX callback: pulls RC frames (rc::frame_type >= 0) off the air and
// queues them for the agent thread. Runs on the main thread (inside
// rtl_device->Init's blocking RX loop).
void rx_on_packet(RealMode& m, const Packet& pkt) {
  m.rx_beat.fetch_add(1, std::memory_order_relaxed);
  if (pkt.RxAtrib.crc_err) m.rx_crcfail_frames.fetch_add(1, std::memory_order_relaxed);
  if (pkt.Data.size() < kDot11HeaderLen + 4) return;
  const uint8_t* body = pkt.Data.data() + kDot11HeaderLen;
  size_t body_len = pkt.Data.size() - kDot11HeaderLen;
  const RxVerdict v = classify_rx(body, body_len, pkt.RxAtrib.crc_err);
  switch (v.route) {
    case RxRoute::Nack:
      if (m.nack_hook) m.nack_hook(body, body_len);  // RX thread (fec-nack)
      break;
    case RxRoute::Mav:
      // Calibration rides T_MAV (the mabur dialect): the MAV thread verifies
      // it and hands its cal packets to the TX writer thread.
      m.mav_queue.push(body, body_len);
      break;
    case RxRoute::Rc:
      m.rc_queue.push(body, body_len);
      break;
    case RxRoute::Foreign:
      // Everything CRC-clean that is not ours (Telem rx_foreign); the frame
      // is still dropped.
      m.rx_foreign_frames.fetch_add(1, std::memory_order_relaxed);
      if (v.foreign_version) log_foreign_rc_version(body[2]);
      break;
    case RxRoute::Drop:
      break;
  }
  if (v.clean_own) {
    m.uplink_track.on_rc_frame(pkt.RxAtrib.rssi, pkt.RxAtrib.snr);
    m.rx_own_frames.fetch_add(1, std::memory_order_relaxed);
  }
}

namespace {

// RX-thread-only state of the NACK answer, owned by the hook.
struct NackState {
  NackResponder responder;
  std::vector<std::vector<uint8_t>> push_order;
  explicit NackState(RealMode& m) : responder(m.retx_env_len, m.cfg.fec.blocks_per_body[0]) {}
};

}  // namespace

// Software NACK answer (NackResponder), RX thread.
void install_nack_hook(RealMode& m) {
  auto st = std::make_shared<NackState>(m);
  m.nack_hook = [&m, st](const uint8_t* body, size_t len) {
    // A malformed frame (radio corruption, sid != 0) counts nack_bad only;
    // check_nack raises auth_reject itself for a tag/counter failure.
    rc::Nack nk;
    if (m.agent.check_nack(body, len, &nk) != RcAgent::NackCheck::kOk) {
      m.nack_bad.fetch_add(1, std::memory_order_relaxed);
      return;
    }
    m.nack_rx.fetch_add(1, std::memory_order_relaxed);
    const auto op = m.shared_op.load();
    const NackResponder::Tally t = st->responder.answer(nk, op.get(), m.cfg.air_clock, m.cfg.nack, now_steady_us(),
                                                        m.retx, &st->push_order);
    m.retx_miss.fetch_add(t.miss, std::memory_order_relaxed);
    m.retx_refused.fetch_add(t.refused, std::memory_order_relaxed);
    m.retx_syms.fetch_add(t.syms, std::memory_order_relaxed);
    const uint32_t now_ms = static_cast<uint32_t>(now_steady_ms());
    for (auto& b : st->push_order) {
      const uint64_t p_us = now_steady_us();
      const size_t bytes = b.size();
      m.txq.push_front(UepBody{0, std::move(b), now_ms, p_us, false});
      m.air_clock.book(p_us, bytes, 0);
    }
  };
}
}  // namespace mabur::maburd
