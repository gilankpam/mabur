#pragma once
#include <cstddef>
#include <cstdint>
#include <vector>
namespace mabur {

// MSP command + DisplayPort subcommand constants (msposd osd/msp/msp.h,
// msp_displayport.h).
constexpr uint8_t MSP_CMD_DISPLAYPORT = 182;
constexpr uint8_t MSP_DP_KEEPALIVE = 0;
constexpr uint8_t MSP_DP_CLOSE = 1;
constexpr uint8_t MSP_DP_CLEAR = 2;
constexpr uint8_t MSP_DP_DRAW_STRING = 3;
constexpr uint8_t MSP_DP_DRAW_SCREEN = 4;
constexpr uint8_t MSP_DP_SET_OPTIONS = 5;

// One decoded MSP v1 message.
struct MspMessage {
  uint8_t cmd = 0;
  std::vector<uint8_t> payload;
};

// Appends one MSP v1 frame ($M< size cmd payload xor-checksum) to `out`.
void msp_append_message(std::vector<uint8_t>& out, uint8_t cmd,
                        const uint8_t* payload, size_t n);

// Streaming MSP v1 parser. Byte-for-byte port of msposd's msp_process_data:
// resyncs on garbage, drops checksum-failures. Feed arbitrary spans.
class MspParser {
 public:
  std::vector<MspMessage> feed(const uint8_t* p, size_t n);

 private:
  enum State { IDLE, VERSION, DIRECTION, SIZE, CMD, PAYLOAD, CHECKSUM };
  State state_ = IDLE;
  uint8_t size_ = 0, cmd_ = 0, checksum_ = 0, buf_ptr_ = 0;
  std::vector<uint8_t> payload_;
};

// DisplayPort screen model: a rows x cols grid of uint16 cells (font page in
// the high byte, matching msposd's `char |= page*0x100`). Ported from
// msp_displayport.c, but writing to a buffer instead of a render vtable.
class MspScreen {
 public:
  // Applies one message; returns true iff it was DRAW_SCREEN (frame complete).
  bool apply(const MspMessage& m);

  // Serializes the current buffer as a self-contained MSP DisplayPort byte
  // sequence: CLEAR, SET_OPTIONS, DRAW_STRING runs of non-blank cells, then
  // DRAW_SCREEN. Never exceeds max_bytes; if content would exceed it, later
  // rows are dropped and *truncated is set true (if non-null).
  std::vector<uint8_t> serialize_snapshot(size_t max_bytes,
                                          bool* truncated = nullptr) const;

  int rows() const { return rows_; }
  int cols() const { return cols_; }
  uint16_t cell(int r, int c) const {
    return buf_[static_cast<size_t>(r) * static_cast<size_t>(cols_) + c];
  }

 private:
  void set_canvas(uint8_t hd_option);
  static bool is_blank(uint16_t cell) {
    uint8_t ch = static_cast<uint8_t>(cell & 0xFF);
    return ch == 0 || ch == 0x20;
  }

  int rows_ = 18, cols_ = 50;          // default HD_50_18
  uint8_t font_ = 0, hd_option_ = 1;   // 1 == MSP_HD_OPTION_50_18
  std::vector<uint16_t> buf_ = std::vector<uint16_t>(18 * 50, 0);
};

}  // namespace mabur
