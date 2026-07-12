#pragma once
#include <cstddef>
#include <cstdint>
#include <vector>
namespace mabur {

// Sub-Block Integrity (SBI) framing constants. Byte-exact port of devourer's
// tools/precoder/fec_subblock.py (SBI_MAGIC, SBI_HDR_LEN, SBI_HDR_STRUCT
// "<HBBHB" = MAGIC, VER, STREAM_ID, BLOCK_PAYLOAD, N_BLOCKS).
constexpr uint16_t SBI_MAGIC = 0xF5B0;
constexpr int SBI_HDR_LEN = 7;

// Packs fixed-size FEC envelopes into SBI radio bodies, each sub-block
// guarded by its own CRC16-CCITT so a corrupted body still yields its
// surviving sub-blocks as usable symbols. Byte-exact port of
// devourer/tools/precoder/fec_subblock.py's SubBlockPacker (crc_bytes fixed
// at 2, matching the default used throughout the precoder toolchain).
//
// Wire body: header <u16 MAGIC LE, u8 ver=0, u8 stream_id, u16 block_payload
// LE, u8 n_blocks> followed by, per accumulated envelope,
// <u16 crc16_ccitt(payload) LE, payload>.
class SbiPacker {
 public:
  SbiPacker(int block_payload, int blocks_per_body, uint8_t stream_id);

  // Feeds one fixed-size FEC envelope in. Returns a freshly completed body
  // once blocks_per_body envelopes have accumulated (zero or more bodies;
  // in practice at most one per call). A deviation from the Python
  // reference: an envelope whose length != block_payload just returns
  // empty, instead of raising (hot path can't throw).
  std::vector<std::vector<uint8_t>> add(const uint8_t* env, size_t len);

  // Emits a short final body with whatever envelopes are pending. Returns
  // empty if nothing is pending.
  std::vector<std::vector<uint8_t>> flush();

  // Per-block wire size: crc16 (2 bytes) + block_payload.
  int block_stride() const;

 private:
  std::vector<uint8_t> build_body(const std::vector<std::vector<uint8_t>>& batch);

  int block_payload_;
  int blocks_per_body_;
  uint8_t stream_id_;
  std::vector<std::vector<uint8_t>> pending_;
};

// Receiver-side split of a radio body into CRC-surviving sub-blocks. Port of
// fec_subblock.py's unpack(): block_payload comes from the receiver's CONFIG
// and is authoritative — the body's header is sanity-checked (header_ok) but
// never trusted to drive partitioning, so a corrupted header cannot desync
// the scan. Works identically on clean and kept-corrupt bodies.
struct SbiUnpackResult {
  std::vector<std::vector<uint8_t>> survivors;  // CRC-valid sub-block payloads
  int n_blocks = 0;                             // sub-blocks scanned
  int n_failed = 0;                             // CRC-mismatched (erasures)
  bool header_ok = false;
  uint8_t stream_id = 0;                        // 0 when the header is short
};
SbiUnpackResult sbi_unpack(const uint8_t* body, size_t len, int block_payload);

// SBI STREAM_ID peek for routing (fixed 7-byte header, independent of
// block_payload), or -1 on a short/bad-magic/bad-version header. A corrupt
// header may misroute a body, but the wrong stream's decoder then rejects
// the mismatched sub-blocks — a dropped body, never a mis-decode.
int sbi_peek_stream_id(const uint8_t* body, size_t len);

}  // namespace mabur
