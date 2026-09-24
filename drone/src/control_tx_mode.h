#pragma once
// devourer::TxMode for the MAX_RANGE control channel. DISC_ACK, telemetry
// and every other control frame fly at the same robustness as the
// MAX_RANGE data profile -- MCS0 / 20 MHz WITH LDPC+STBC (flying them
// without the flags was weaker than rung 0's own data, and a DISC_ACK lost
// at exactly the range where MAX_RANGE is needed defeats the floor).
//
// no_agg (2026-09-24, 40 MHz rungs): with A-MPDU on, the MAC folds
// co-queued frames into one PPDU at ONE rate/width regardless of their
// own descriptor (docs/bw40-sweep-findings-2026-09-23.md "Aggregation");
// a control frame folded into an mcs4/40 aggregate airs at mcs4/40. The
// devourer-private TX_FLAGS bit keeps it a PPDU of its own (Jaguar3:
// AGG_EN=0 + BK=1). Same queue, ordering unchanged.
#include "TxMode.h"

namespace mabur {

inline devourer::TxMode control_tx_mode() {
  devourer::TxMode m;
  m.mode = devourer::TxMode::Mode::HT;
  m.ht_mcs = 0;
  m.bw_mhz = 20;
  m.sgi = false;
  m.ldpc = true;
  m.stbc = true;
  m.no_agg = true;
  return m;
}

}  // namespace mabur
