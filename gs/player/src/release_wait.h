#ifndef MABUR_PLAYER_RELEASE_WAIT_H_
#define MABUR_PLAYER_RELEASE_WAIT_H_

#include <algorithm>
#include <cstdint>

namespace maburplay {

// Sleeps until `until_us` in slices of at most `step_us`, calling `poll`
// before the first slice, between slices and once at the deadline.
//
// The render loop's only way to learn that a picture finished decoding is
// backend->poll() (MPP has no fd to wait on), so any uninterrupted sleep
// is a window in which a decoded frame sits uncollected. The pre-release
// micro-sleep (up to a few ms, right before the regulator's release point)
// was exactly that: frames finishing inside it were collected only after
// the release and the commit, ~4 ms late, and since the window sits at a
// fixed vblank phase the loss swept across frames with the sensor/panel
// beat. Polling every `step_us` bounds that to one step; decode_get_frame
// with nothing ready is a microsecond ioctl, so the extra calls cost
// nothing measurable.
template <class NowFn, class SleepFn, class PollFn>
void wait_until_polling(uint64_t until_us, uint64_t step_us, NowFn now,
                        SleepFn sleep_us, PollFn poll) {
  for (;;) {
    poll();
    const uint64_t t = now();
    if (t >= until_us) return;
    sleep_us(std::min(until_us - t, step_us));
  }
}

}  // namespace maburplay

#endif  // MABUR_PLAYER_RELEASE_WAIT_H_
