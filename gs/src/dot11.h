#pragma once
// Pure dot11 helpers shared by RadioFrontend (maburgs) and the web GS glue
// (web/src/web_main.cpp). No device state; devourer is linked only for the
// radiotap builder behind build_control_frame.
#include <cstddef>
#include <cstdint>
#include <vector>

#include "mabur/node.h"

namespace maburgs {

// Pure: MAX_RANGE radiotap + 24-byte dot11 probe-req header (canonical SA
// 57:42:75:05:d6:00, broadcast DA, seq<<4) + body. Builds the GS's own
// uplink 0x40 control frame. The drone's video downlink switched to QoS-Data
// (A-MPDU), but the uplink RCF remains probe-req (unchanged by the wire change).
std::vector<uint8_t> build_control_frame(uint16_t seq, const uint8_t* body, size_t len);

// Pure: true when the dot11 header's SA (bytes 10..15) is the canonical
// mabur SA. Frames too short to carry an SA are not canonical.
bool sa_canonical(const uint8_t* dot11, size_t len);

// Pure: byte offset of the mabur body inside a dot11 frame, keyed on the
// frame-control type. QoS-Data (0x88, the post-A-MPDU drone wire) carries a
// 26-byte header; everything else (the legacy probe-req 0x40 wire, and any
// frame the SA filter passes) parses at the legacy 24-byte offset. Returns
// 0 when len cannot hold the header plus at least one body byte. seq_ctl
// sits at bytes 22-23 in BOTH layouts, so mac_seq extraction is unchanged.
size_t dot11_body_offset(const uint8_t* dot11, size_t len);

// devourer RxAtrib fields the conversion reads, without devourer types.
// rssi is uint8_t (matches RxAtrib::rssi and RxBody::rssi); snr/evm are
// int8_t (matches both as well).
struct RxMeta {
  bool crc_err = false;
  uint16_t data_rate = 0;
  uint8_t rssi[2] = {0, 0};
  int8_t snr[2] = {0, 0};
  int8_t evm[2] = {0, 0};
  bool physt = false;
  uint32_t tsfl = 0;
};

enum class RxVerdict { Body, Short, Foreign };

// dot11 frame (radiotap already stripped) -> RxBody. Foreign = CRC-good
// with a non-canonical SA (never reaches the queue: it polluted per-card
// EMAs and the seq-loss walk). CRC-failed frames pass -- their intact SBI
// sub-blocks are salvageable. Fills body/mac_seq/mcs/crc_ok/rssi/snr/evm/
// phy_valid/tsfl; card_id, mono_us and rx_channel are the caller's.
RxVerdict fill_rx_body(const uint8_t* dot11, size_t len, const RxMeta& meta,
                       mabur::node::RxBody& out);

}  // namespace maburgs
