#include "cal_picker.h"

#include "channel_core.h"

namespace maburgs {

CalPicker::Mode ChannelCorePicker::mode() const {
  if (core_.pinned()) return Mode::Pinned;
  return core_.cal_can_measure() ? Mode::Auto : Mode::OneCard;
}

uint8_t ChannelCorePicker::op() const { return core_.op(); }

void ChannelCorePicker::begin_measure(uint64_t now_ms) {
  core_.begin_cal_measure(static_cast<double>(now_ms));
}

std::optional<uint8_t> ChannelCorePicker::measured_best() { return core_.cal_measure_best(); }

bool ChannelCorePicker::order_relocate(uint8_t ch) { return core_.order_cal_relocate(ch); }

std::optional<bool> ChannelCorePicker::relocate_settled() { return core_.cal_relocate_settled(); }

void ChannelCorePicker::cancel() { core_.cancel_cal_measure(); }

}  // namespace maburgs
