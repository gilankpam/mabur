#include "radio_frontend.h"

#include <functional>

#include <libusb.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>

#include "AdapterCaps.h"
#include "RadiotapBuilder.h"
#include "RxPacket.h"
#include "RxSense.h"
#include "TxMode.h"
#include "UsbDeviceLock.h"
#include "UsbOpen.h"
#include "WiFiDriver.h"
#include "logger.h"
#include "mabur/ht40.h"
#include "mabur/node.h"

namespace maburgs {
namespace {
constexpr size_t kDot11 = 24;
constexpr uint8_t kSa[6] = {0x57, 0x42, 0x75, 0x05, 0xd6, 0x00};
constexpr uint16_t kScanPids[] = {0xa81a, 0x881a, 0x8812};

const std::vector<uint8_t>& max_range_radiotap() {
  static const std::vector<uint8_t> rt = [] {
    devourer::TxMode m;
    m.mode = devourer::TxMode::Mode::HT;
    m.ht_mcs = 0;
    m.bw_mhz = 20;
    m.ldpc = true;
    m.stbc = true;
    return devourer::build_stream_radiotap(m);
  }();
  return rt;
}

uint64_t mono_us_now() {
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}
}  // namespace

std::vector<uint8_t> build_control_frame(uint16_t seq, const uint8_t* body,
                                         size_t len) {
  const auto& rt = max_range_radiotap();
  std::vector<uint8_t> f(rt.size() + kDot11 + len);
  std::memcpy(f.data(), rt.data(), rt.size());
  uint8_t* d = f.data() + rt.size();
  d[0] = 0x40;
  d[1] = 0x00;
  d[2] = 0x00;
  d[3] = 0x00;
  std::memset(d + 4, 0xff, 6);
  std::memcpy(d + 10, kSa, 6);
  std::memcpy(d + 16, kSa, 6);
  const uint16_t seq_ctl = static_cast<uint16_t>(seq << 4);
  d[22] = static_cast<uint8_t>(seq_ctl & 0xff);
  d[23] = static_cast<uint8_t>(seq_ctl >> 8);
  if (len) std::memcpy(d + kDot11, body, len);
  return f;
}

bool sa_canonical(const uint8_t* dot11, size_t len) {
  return len >= 16 && std::memcmp(dot11 + 10, kSa, 6) == 0;
}

size_t dot11_body_offset(const uint8_t* dot11, size_t len) {
  const size_t off = (len >= 1 && dot11[0] == 0x88) ? 26 : 24;
  return len >= off + 1 ? off : 0;
}

// --- device management (mirrors drone/src/main.cpp bring-up) ----------------

RadioFrontend::RadioFrontend(Cfg cfg, BodyQueue& out)
    : cfg_(cfg), out_(out), width_(cfg.width_mhz) {}
RadioFrontend::~RadioFrontend() { stop(); }

bool RadioFrontend::open_and_start() {
  if (libusb_init(&usb_ctx_) != 0) return false;
  // Two ways to name the device. Auto-scan (the default) hands us a
  // physical port, which survives the card re-enumerating at a new address;
  // an explicit [[radio.cards]] entry names the index-th VID/PID match, as
  // it always did.
  libusb_device** list = nullptr;
  const ssize_t n = libusb_get_device_list(usb_ctx_, &list);
  int match = 0;
  libusb_device* dev = nullptr;
  for (ssize_t i = 0; i < n; ++i) {
    if (cfg_.by_port) {
      if (device_at_port(list[i], cfg_.port)) { dev = list[i]; break; }
      continue;
    }
    libusb_device_descriptor dd;
    if (libusb_get_device_descriptor(list[i], &dd) != 0) continue;
    if (dd.idVendor != cfg_.usb_vid) continue;
    bool pid_ok = cfg_.usb_pid != 0 ? dd.idProduct == cfg_.usb_pid : false;
    if (cfg_.usb_pid == 0)
      for (uint16_t p : kScanPids) pid_ok = pid_ok || dd.idProduct == p;
    if (!pid_ok) continue;
    if (match++ == cfg_.index) { dev = list[i]; break; }
  }
  if (dev == nullptr || libusb_open(dev, &handle_) != 0) {
    if (list) libusb_free_device_list(list, 1);
    libusb_exit(usb_ctx_);
    usb_ctx_ = nullptr;
    handle_ = nullptr;
    return false;
  }
  libusb_free_device_list(list, 1);

  logger_ = std::make_shared<Logger>();
  int rc = devourer::claim_interface_then_reset(handle_, 0, logger_, /*do_reset=*/true, usb_lock_);
  if (rc != 0) {
    libusb_close(handle_);
    libusb_exit(usb_ctx_);
    handle_ = nullptr;
    usb_ctx_ = nullptr;
    return false;
  }

  devourer::DeviceConfig dev_cfg;
  dev_cfg.rx.enable_with_tx = true;  // TX+RX duplex: mandatory on the 8822E
  // Keep FCS-failed frames (RCR ACRC32|AICV on the 8822E). A corrupt body
  // still yields its intact SBI sub-blocks -- each carries its own CRC16 --
  // and that salvage path was unreachable until devourer honoured this on
  // Jaguar3 (2026-09-08): the WMAC dropped the frames before the host saw
  // them, which is why per-card crc_fail sat at 0 for the project's whole
  // history. Cost measured on the bench: ~19 foreign junk frames in 4 min
  // against ~150k real frames/card/min. on_packet() below lets crc_err
  // frames past the SA filter; the aggregator counts them (crc_fail) and
  // keeps them out of the seq walk; UepDecoder::add_body salvages.
  dev_cfg.rx.keep_corrupted = true;
  // Absolute idle floor (jgr3-nhm-abs-floor): only the with_nhm=true read
  // pays for it (the scout's dwell read and the one-off caps read). The
  // 1 Hz A record uses with_nhm=false and must stay a handful of register
  // reads -- bench check in the auto-channel-select plan, Task 14.
  dev_cfg.rx.abs_noise_floor = true;
  // Debug passthrough: devourer's env->config translation lives in its
  // examples/, not the library, so these two register-dump levers (used to
  // diff a live card against the vendor kernel's end state) must be wired
  // here explicitly. Inert unless the env vars are set.
  dev_cfg.debug.dump_canary = std::getenv("DEVOURER_DUMP_CANARY") != nullptr;
  dev_cfg.debug.bb_dump = std::getenv("DEVOURER_BB_DUMP") != nullptr;
  // MAC carrier sense ON (chip default) -- same decision as maburd, see the
  // block in drone/src/main.cpp: OFF from 2026-08-05 to 2026-09-23, back ON
  // because a blind GS send kills a drone PPDU on both of these cards and
  // carrier sense on BOTH ends removes that (docs/cca-on-findings-
  // 2026-09-23.md). The two must move together: GS-only carrier sense
  // measured WORSE than blind (docs/rcf-uplink-loss-findings-2026-08-14.md
  // §6). RadioFrontend is constructed per card, so every card gets the flag
  // and logs its own bring-up line below -- inert on RX-only cards, since
  // this gates TX only.
  dev_cfg.tuning.disable_cca = false;
  // Same pins as maburd (see drone/src/main.cpp): data sends never cancel a
  // multi-packet bulk-OUT (a mid-transfer timeout wedges the TX endpoint),
  // and RX stays on the heap path, not zerocopy.
  dev_cfg.tx.no_cancel_multipkt = true;
  dev_cfg.usb.rx_zerocopy = false;
  // (The 0x41e8 protect_pathb_agc knob was chased here too — exonerated:
  // the real path-B killer was the DPDT pin-mux, fixed by devourer's eFEM
  // pinmux port; see DEVOURER_DPDT_MODE in RtlJaguar3Device.)
  driver_ = std::make_unique<WiFiDriver>(logger_);
  device_ = driver_->CreateRtlDevice(handle_, usb_ctx_, usb_lock_, dev_cfg);
  if (!device_) { stop(); return false; }
  // width_, not the constructor's cfg_.width_mhz: a set_width() that landed
  // while the card was down (the boot scout card dying mid-scan) is the
  // width a revive must come up at.
  device_->InitWrite(width() == 40
                         ? SelectedChannel{cfg_.channel, mabur::ht40_offset(cfg_.channel), CHANNEL_WIDTH_40}
                         : SelectedChannel{cfg_.channel, 0, CHANNEL_WIDTH_20});
  channel_.store(cfg_.channel, std::memory_order_release);
  rx_channel_.store(cfg_.channel, std::memory_order_release);
  {
    const devourer::AdapterCaps ac = device_->GetAdapterCaps();
    const RxEnergy e = device_->GetRxEnergy(/*with_nhm=*/true);
    caps_.valid = ac.supported;
    // "?" rather than "": the scan.log C record is a positional,
    // space-separated line, so an empty field silently shifts every column
    // after it. devourer returns a null chip_name on an unrecognised chip
    // and generation_name can return an empty string for an unmapped
    // generation.
    caps_.chip = (ac.chip_name && *ac.chip_name) ? ac.chip_name : "?";
    const char* gen = devourer::generation_name(ac.generation);
    caps_.gen = (gen && *gen) ? gen : "?";
    caps_.tx_chains = ac.tx_chains;
    caps_.rx_chains = ac.rx_chains;
    caps_.bw_mask = ac.bw_mask;
    caps_.tune5g_lo = ac.tune_5g.valid ? ac.tune_5g.min_mhz : 0;
    caps_.tune5g_hi = ac.tune_5g.valid ? ac.tune_5g.max_mhz : 0;
    caps_.fast_retune = ac.fastretune_ok;
    caps_.fa_ok = e.valid_fa;
    caps_.igi_ok = e.valid_igi;
    caps_.nhm_ok = e.valid_nhm;
    caps_.floor_ok = e.valid_noise_floor;
  }
  // Bring-up record for the non-standard MAC state requested via
  // dev_cfg.tuning.disable_cca above. devourer logs its own carrier-sense line at
  // info, and the production cross-build compiles info out
  // (DEVOURER_LOG_MAX_LEVEL=WARN), so without this the deployed daemon leaves no
  // trace of which way it was built. Once per card, tagged with card_id,
  // since RadioFrontend is constructed per card. Records what maburgs
  // REQUESTED of devourer for this card, not a register readback.
  std::fprintf(stderr,
               "maburgs radio card %d: MAC carrier sense (CCA+EDCCA) requested %s\n",
               static_cast<int>(cfg_.card_id),
               dev_cfg.tuning.disable_cca
                   ? "OFF -- TX will not defer to co-channel traffic"
                   : "ON -- TX defers to any decodable 802.11 preamble (chip default, 2026-09-23)");
  ready_.store(true, std::memory_order_release);
  alive_.store(true, std::memory_order_release);
  rx_thread_ = std::thread([this] {
    device_->StartRxLoop([this](const Packet& pkt) { on_packet(pkt); });
    alive_.store(false, std::memory_order_release);
  });
  return true;
}

void RadioFrontend::on_packet(const Packet& pkt) {
  rx_frames_.fetch_add(1, std::memory_order_relaxed);
  const size_t body_off = dot11_body_offset(pkt.Data.data(), pkt.Data.size());
  if (body_off == 0) return;
  // Foreign traffic never reaches the queue: it polluted per-card EMAs and
  // the seq-loss walk (spec revision 2). CRC-failed frames pass — a corrupt
  // SA proves nothing, and they never fed EMAs/seq anyway.
  if (!pkt.RxAtrib.crc_err && !sa_canonical(pkt.Data.data(), pkt.Data.size())) {
    foreign_.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  if (!pkt.RxAtrib.crc_err) {
    own_.fetch_add(1, std::memory_order_relaxed);
    // Own airtime (spec 2026-09-25-nhm-airtime §5), CRC-good own frames only.
    const uint16_t r = pkt.RxAtrib.data_rate;
    const uint8_t mcs = (r >= 0x0C && r <= 0x13) ? static_cast<uint8_t>(r - 0x0C)
                        : (r >= 0x80 && r <= 0x87) ? static_cast<uint8_t>(r - 0x80) : 255;
    own_air_.on_frame(pkt.Data.size(), mcs, pkt.RxAtrib.physt,
                      pkt.RxAtrib.bw == 1 ? 40 : 20, pkt.RxAtrib.stbc != 0, pkt.RxAtrib.sgi != 0);
    own_air_us_.store(own_air_.total_us(), std::memory_order_relaxed);
  }
  mabur::node::RxBody m;
  m.card_id = cfg_.card_id;
  m.mono_us = mono_us_now();
  m.rssi[0] = pkt.RxAtrib.rssi[0];
  m.rssi[1] = pkt.RxAtrib.rssi[1];
  m.snr[0] = pkt.RxAtrib.snr[0];
  m.snr[1] = pkt.RxAtrib.snr[1];
  m.evm[0] = pkt.RxAtrib.evm[0];
  m.evm[1] = pkt.RxAtrib.evm[1];
  m.phy_valid = pkt.RxAtrib.physt;
  m.crc_ok = !pkt.RxAtrib.crc_err;
  // RX rate code -> HT MCS index. devourer's RxAtrib.data_rate carries TWO
  // encodings depending on chip family, and both HT-1SS ranges are mapped
  // here (they cannot collide):
  //  - jaguar1/2/3 (8812/8822B/8822E...): the raw Realtek DESC_RATE index --
  //    HT MCS0..7 = 0x0C..0x13 (DESC_RATEMCS0, ieee80211_radiotap.h). This
  //    is what the GS's 8822E cards produce; verified 2026-08-14 on the
  //    bench when the original 0x80-only mapping left every attribution
  //    boundary unclosed (link.attrib.close_ms null through 5 promotes).
  //  - kestrel (8852B/C): the AX 9-bit code, HT = 0x80 + mcs (the encoding
  //    RxPacket.h's comment describes; it does NOT apply to jaguar chips).
  // Everything else (legacy CCK/OFDM, VHT, HE, 2SS) is "unknown" for
  // attribution purposes -- the drone injects HT-1SS only.
  const uint16_t dr = pkt.RxAtrib.data_rate;
  m.mcs = (dr >= 0x0C && dr <= 0x13) ? static_cast<uint8_t>(dr - 0x0C)
          : (dr >= 0x80 && dr <= 0x87) ? static_cast<uint8_t>(dr - 0x80)
                                       : 255;
  m.mac_seq = static_cast<uint16_t>(
      (static_cast<uint16_t>(pkt.Data[22] | (pkt.Data[23] << 8))) >> 4);
  m.body.assign(pkt.Data.begin() + static_cast<long>(body_off), pkt.Data.end());
  m.tsfl = pkt.RxAtrib.tsfl;
  // Receive-channel provenance, stamped HERE -- on the producer thread, at
  // the moment the frame is lifted off this card -- so no amount of
  // queueing between here and the core loop can change it (mabur/node.h).
  m.rx_channel = rx_channel_.load(std::memory_order_acquire);
  const uint64_t mono = m.mono_us;
  const uint32_t tsfl = m.tsfl;
  out_.push(std::move(m));

  // rx_pace gauge (see header). uint32 subtraction handles the ~71 min TSF
  // wrap; the first body after start (last==0) only seeds.
  if (rp_last_mono_ != 0) {
    const uint64_t hd = mono - rp_last_mono_;
    const uint32_t td = tsfl - rp_last_tsfl_;
    if (hd < 5000 && td < 5000) {
      static constexpr uint32_t kEdge[kPaceBuckets - 1] = {
          100, 150, 200, 250, 300, 400, 600, 1200};
      ++rp_n_;
      rp_host_sum_ += hd;
      rp_tsfl_sum_ += td;
      int hb = kPaceBuckets - 1, tb = kPaceBuckets - 1;
      for (int i = 0; i < kPaceBuckets - 1; ++i) {
        if (hb == kPaceBuckets - 1 && hd <= kEdge[i]) hb = i;
        if (tb == kPaceBuckets - 1 && td <= kEdge[i]) tb = i;
      }
      ++rp_host_hist_[hb];
      ++rp_tsfl_hist_[tb];
    } else {
      ++rp_gaps_;
    }
  }
  rp_last_mono_ = mono;
  rp_last_tsfl_ = tsfl;
  if (rp_last_report_us_ == 0) rp_last_report_us_ = mono;
  if (mono - rp_last_report_us_ >= 5000000) {
    rp_last_report_us_ = mono;
    if (rp_n_ > 0) {
      char th[128], hh[128];
      int tp = 0, hp = 0;
      for (int i = 0; i < kPaceBuckets; ++i) {
        tp += std::snprintf(th + tp, sizeof(th) - static_cast<size_t>(tp),
                            "%s%llu", i ? "/" : "",
                            (unsigned long long)rp_tsfl_hist_[i]);
        hp += std::snprintf(hh + hp, sizeof(hh) - static_cast<size_t>(hp),
                            "%s%llu", i ? "/" : "",
                            (unsigned long long)rp_host_hist_[i]);
      }
      std::fprintf(stderr,
                   "maburgs rx_pace card %d: n=%llu gaps=%llu "
                   "tsfl_d mean=%llu hist<=100/150/200/250/300/400/600/1200/"
                   "inf=%s host_d mean=%llu hist=%s\n",
                   static_cast<int>(cfg_.card_id), (unsigned long long)rp_n_,
                   (unsigned long long)rp_gaps_,
                   (unsigned long long)(rp_tsfl_sum_ / rp_n_), th,
                   (unsigned long long)(rp_host_sum_ / rp_n_), hh);
    }
    rp_n_ = rp_gaps_ = rp_tsfl_sum_ = rp_host_sum_ = 0;
    for (int i = 0; i < kPaceBuckets; ++i)
      rp_tsfl_hist_[i] = rp_host_hist_[i] = 0;
  }
}

bool RadioFrontend::retune(uint8_t ch) {
  if (!ready_.load(std::memory_order_acquire) || !device_) return false;
  // Blind the body stamp for the duration of the move (the RX thread keeps
  // running through it). A frame the chip had already handed us before the
  // retune but that reaches on_packet() during it is then stamped 0 =
  // unknown instead of being mis-attributed to the new channel -- which is
  // what lets the hop's confirmation trust the stamp. The residual window
  // is USB-pipeline lag longer than FastRetune's own duration (~4 ms of
  // control transfers on this path); devourer exposes no RX flush to close
  // it outright.
  static const bool hopdbg = std::getenv("MABUR_HOP_DEBUG") != nullptr;
  const uint64_t dbg_t0 = hopdbg ? mono_us_now() : 0;
  const uint8_t dbg_from = channel_.load(std::memory_order_acquire);
  rx_channel_.store(0, std::memory_order_release);
  device_->FastRetune(ch, /*cache_rf=*/true);
  channel_.store(ch, std::memory_order_release);
  if (hopdbg)
    std::fprintf(stderr, "retunedbg card=%u tid=%lu %u->%u t0=%llu dur_us=%llu rf_central=%d\n",
                 static_cast<unsigned>(cfg_.card_id),
                 static_cast<unsigned long>(std::hash<std::thread::id>{}(std::this_thread::get_id()) % 100000),
                 static_cast<unsigned>(dbg_from), static_cast<unsigned>(ch),
                 static_cast<unsigned long long>(dbg_t0 / 1000),
                 static_cast<unsigned long long>(mono_us_now() - dbg_t0), device_->ReadTunedCentral());
  rx_channel_.store(ch, std::memory_order_release);
  return true;
}

bool RadioFrontend::set_width(uint8_t ch, uint8_t width_mhz) {
  if (width_mhz == 40 && mabur::ht40_offset(ch) == 0) return false;
  const uint8_t was = width();
  // Desired state, like the channel: recorded even when the card is down so
  // the next open_and_start() (a revive) InitWrites at it. The caller still
  // sees false -- nothing was tuned now.
  width_.store(width_mhz, std::memory_order_release);
  if (!ready_.load(std::memory_order_acquire) || !device_) return false;
  rx_channel_.store(0, std::memory_order_release);   // same blinding as retune()
  device_->SetMonitorChannel(width_mhz == 40
                                 ? SelectedChannel{ch, mabur::ht40_offset(ch), CHANNEL_WIDTH_40}
                                 : SelectedChannel{ch, 0, CHANNEL_WIDTH_20});
  channel_.store(ch, std::memory_order_release);
  rx_channel_.store(ch, std::memory_order_release);
  std::fprintf(stderr, "maburgs radio: card %u width %u -> %u MHz on ch %u\n",
               static_cast<unsigned>(cfg_.card_id), static_cast<unsigned>(was),
               static_cast<unsigned>(width_mhz), static_cast<unsigned>(ch));
  return true;
}

ScoutEnergy RadioFrontend::read_energy(bool with_nhm) {
  ScoutEnergy out;
  if (!ready_.load(std::memory_order_acquire) || !device_) return out;
  const RxEnergy e = device_->GetRxEnergy(with_nhm);
  out.fa_valid = e.valid_fa;
  out.cca_ofdm = e.cca_ofdm;
  out.fa_ofdm = e.fa_ofdm;
  out.igi_valid = e.valid_igi;
  out.igi = e.igi;
  out.nhm_valid = e.valid_nhm;
  out.floor_valid = e.valid_noise_floor;
  out.floor_dbm = e.abs_noise_floor_dbm;
  return out;
}

ScoutEnergy RadioFrontend::read_energy_scout() {
  ScoutEnergy out;
  if (!ready_.load(std::memory_order_acquire) || !device_) return out;
  const RxEnergy e = device_->GetRxEnergyScout();
  out.fa_valid = e.valid_fa;
  out.cca_ofdm = e.cca_ofdm;
  out.fa_ofdm = e.fa_ofdm;
  out.igi_valid = e.valid_igi;
  out.igi = e.igi;
  out.nhm_valid = e.valid_nhm;
  out.floor_valid = e.valid_noise_floor;
  out.floor_dbm = e.abs_noise_floor_dbm;
  return out;
}

bool RadioFrontend::arm_nhm_busy(uint16_t period_4us) {
  if (!ready_.load(std::memory_order_acquire) || !device_) return false;
  return device_->ArmNhmBusy(period_4us);
}

int RadioFrontend::tuned_central() {
  return (ready_.load(std::memory_order_acquire) && device_) ? device_->ReadTunedCentral() : -1;
}

NhmBusyRead RadioFrontend::read_nhm_busy() {
  NhmBusyRead out;
  if (!ready_.load(std::memory_order_acquire) || !device_) return out;
  const NhmBusy b = device_->ReadNhmBusy();
  out.valid = b.valid;
  for (int i = 0; i < 12; ++i) out.buckets[i] = b.buckets[i];
  out.duration = b.duration;
  out.period = b.period;
  return out;
}

bool RadioFrontend::send_control(const std::vector<uint8_t>& body) {
  if (!ready_.load(std::memory_order_acquire) || !device_) {
    tx_fail_.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  const auto frame = build_control_frame(tx_seq_, body.data(), body.size());
  tx_seq_ = static_cast<uint16_t>((tx_seq_ + 1) & 0xFFF);
  const bool ok = device_->send_packet(frame.data(), frame.size());
  (ok ? tx_frames_ : tx_fail_).fetch_add(1, std::memory_order_relaxed);
  return ok;
}

void RadioFrontend::stop() {
  if (device_ && alive_.load(std::memory_order_acquire)) device_->StopRxLoop();
  if (rx_thread_.joinable()) rx_thread_.join();
  if (device_) device_->Stop();
  device_.reset();
  driver_.reset();
  ready_.store(false, std::memory_order_release);
  alive_.store(false, std::memory_order_release);
  if (handle_) { libusb_release_interface(handle_, 0); libusb_close(handle_); handle_ = nullptr; }
  // Release the per-adapter advisory lock (claim_interface_then_reset filled
  // it) or the next open_and_start() on this same card refuses with "already
  // in use by another devourer process" — the process deadlocks against its
  // own stale lock and a replugged card can never reopen (bench 2026-07-12).
  usb_lock_.reset();
  if (usb_ctx_) { libusb_exit(usb_ctx_); usb_ctx_ = nullptr; }
}

bool RadioFrontend::ready() const { return ready_.load(std::memory_order_acquire); }
bool RadioFrontend::alive() const { return alive_.load(std::memory_order_acquire); }
uint64_t RadioFrontend::rx_frames() const { return rx_frames_.load(std::memory_order_relaxed); }
uint64_t RadioFrontend::foreign() const { return foreign_.load(std::memory_order_relaxed); }
uint64_t RadioFrontend::tx_frames() const { return tx_frames_.load(std::memory_order_relaxed); }
uint64_t RadioFrontend::tx_fail() const { return tx_fail_.load(std::memory_order_relaxed); }

}  // namespace maburgs
