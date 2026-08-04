#pragma once
namespace maburgs {
// devourer reports SNR in HALF-dB units (third_party/devourer/src/
// LinkHealth.h:49; its own RxQuality divides by 2). radio_frontend.cpp
// copies RxAtrib.snr through untouched and aggregator.cpp EMAs the raw
// value without correcting it, so this is where the wire value becomes
// what the "snr"/"snr_a"/"snr_b" keys (and any dB-labeled SNR elsewhere,
// e.g. LinkHealth::s1_snr_db) have always claimed to hold. NOTE: .jsonl
// recorded before this change is on the old (2x) scale and is not
// numerically comparable to recordings made after it. Multiply by this
// before showing/logging/deciding anything in dB. Single definition — the
// sideport exporter and the controller's SNR labels must never disagree
// (scale-break history: CLAUDE.md, 2026-08-04).
constexpr double kSnrRawToDb = 0.5;
}  // namespace maburgs
