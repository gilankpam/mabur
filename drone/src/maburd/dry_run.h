#pragma once
// Dry-run mode: file-driven, no radio.
#include <cstdint>
#include <string>

#include "config.h"

namespace mabur::maburd {

int run_dry_run(const Config& cfg, uint8_t start_ch, const std::string& in_path,
                const std::string& out_path, const std::string& rc_in_path,
                const std::string& msp_in_path, const std::string& msp_out_path);

}  // namespace mabur::maburd
