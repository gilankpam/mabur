#pragma once
#include <cstdint>
#include <string>

namespace maburgs {

// mabur-owned subset of devourer's RxEnergy: the fields the ranker and the
// scan.log D/A lines use. Validity flags are the sensor-support record.
struct ScoutEnergy {
  bool fa_valid = false;
  uint32_t cca_ofdm = 0, fa_ofdm = 0;
  bool igi_valid = false;
  uint8_t igi = 0;
  bool nhm_valid = false;
  bool floor_valid = false;
  int8_t floor_dbm = 0;
};

// Cumulative decoded-frame counts on one card: canonical-SA (own) vs the
// rest. The scout diffs two snapshots to attribute a dwell's occupancy.
struct ScoutFrames {
  uint64_t own = 0, foreign = 0;
};

// "What does this card support" -- the scan.log C record. Static identity
// from devourer's AdapterCaps plus the validity flags of one energy read.
struct CardCaps {
  bool valid = false;
  std::string chip, gen;
  int tx_chains = 0, rx_chains = 0;
  uint8_t bw_mask = 0;
  uint16_t tune5g_lo = 0, tune5g_hi = 0;
  bool fast_retune = false;
  bool fa_ok = false, igi_ok = false, nhm_ok = false, floor_ok = false;
};

// The one card's control plane the scout drives. RadioFrontend implements
// it; tests use a fake. All three are called from the scout thread only.
class ScoutRadio {
 public:
  virtual ~ScoutRadio() = default;
  virtual bool retune(uint8_t ch) = 0;
  virtual ScoutEnergy read_energy(bool with_nhm) = 0;
  // Cheapest frame-free OFDM FA+CCA delta (devourer's GetRxEnergyScout):
  // only fa_valid/fa_ofdm/cca_ofdm are meaningful, everything else
  // (including nhm) is left invalid. For the ~10 ms in-flight dwell
  // (inflight_scout.h), which cannot afford GetRxEnergy's NHM window.
  virtual ScoutEnergy read_energy_scout() = 0;
  virtual ScoutFrames frames() const = 0;
};

}  // namespace maburgs
