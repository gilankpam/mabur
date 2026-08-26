#ifndef MABUR_PLAYER_DVR_NAME_H_
#define MABUR_PLAYER_DVR_NAME_H_

#include <string>

namespace maburplay {

// Mints DVR recording paths: `<dir>/record-NNNN.mp4`, %04d, zero-based.
//
// No timestamp, by design. The GS RTC is wrong at boot, so a stamped name
// carries no information and sorts arbitrarily across boots -- the same
// reason CtlLog indexes its files (`ctl-NNNN_<date>.log`) and the stats
// recorder writes `flight-NNNN.jsonl`. The index is one past the highest
// `record-NNNN` already on the card, so it survives reboots and never
// overwrites an earlier flight. Legacy `record_<date>.mp4` files do not
// match the pattern: they are ignored and keep their names.
//
// The high-water mark is not redundant with the scan. NEITHER writer
// creates the file when the name is minted -- burned mode opens the mux on
// the encoder thread at the first encoded frame, raw mode at the next
// sid-0 sync point (up to ~2 s later) -- so a stop/start pair inside that
// window would scan an unchanged directory twice and hand out one index
// twice, and both writers open "wb". The record button makes that a
// double-tap away (kDebounceMs is 50).
class DvrNamer {
 public:
  // Main-loop-thread only (the ring sink and rec_start's
  // start_burn_if_needed), so no synchronisation.
  std::string next(const std::string& dir);

 private:
  int last_issued_ = -1;
};

}  // namespace maburplay

#endif  // MABUR_PLAYER_DVR_NAME_H_
