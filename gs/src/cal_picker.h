#pragma once
// CalPicker: a calibration run's view of the channel machinery. Before its
// first sweep the run moves the pair onto the quietest member of the
// channel set: one fresh measure round, then -- when the best member is not
// op -- one relocate, then the wait for the confirm. CalSession talks only
// to this interface so it can be tested with a fake; ChannelCorePicker is
// the real one over ChannelCore. Neither pulls in a maburgs-only header:
// the web GS links ChannelCore too.
#include <cstdint>
#include <optional>

namespace maburgs {

class ChannelCore;

class CalPicker {
 public:
  // Auto: two or more cards, channel = "auto" -- a round can be measured.
  // Pinned: the operator pinned the channel. OneCard: no spare card to
  // measure with (one USB card, or a relay-only roster). For Pinned and
  // OneCard no round runs and measured_best() is op() at once.
  enum class Mode { Auto, Pinned, OneCard };
  virtual ~CalPicker() = default;
  virtual Mode mode() const = 0;
  virtual uint8_t op() const = 0;
  // One measure round over the set. Call before the run sets cal_running:
  // a running calibration freezes every channel change, the scout included.
  virtual void begin_measure(uint64_t now_ms) = 0;
  // The round's best member; nullopt while the round runs.
  virtual std::optional<uint8_t> measured_best() = 0;
  // Move the pair to `ch` (the boot pick's relocate order, reason cal).
  // false: refused -- not a member, already there, a hop or relocate in
  // flight, the hop rate limit reached, the link down.
  virtual bool order_relocate(uint8_t ch) = 0;
  // true: the drone followed (confirmed); false: it did not (withdrawn,
  // session lost, never placed); nullopt: pending, or nothing ordered.
  virtual std::optional<bool> relocate_settled() = 0;
};

class ChannelCorePicker : public CalPicker {
 public:
  explicit ChannelCorePicker(ChannelCore& core) : core_(core) {}
  Mode mode() const override;
  uint8_t op() const override;
  void begin_measure(uint64_t now_ms) override;
  std::optional<uint8_t> measured_best() override;
  bool order_relocate(uint8_t ch) override;
  std::optional<bool> relocate_settled() override;

 private:
  ChannelCore& core_;
};

}  // namespace maburgs
