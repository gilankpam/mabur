#pragma once
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>
#include "json.hpp"
namespace mtest {
inline nlohmann::json load_json(const std::string& path) {
  std::ifstream f(path);
  if (!f) throw std::runtime_error("cannot open " + path);
  return nlohmann::json::parse(f);
}
inline std::vector<uint8_t> unhex(const std::string& s) {
  std::vector<uint8_t> out(s.size() / 2);
  for (size_t i = 0; i < out.size(); ++i)
    out[i] = static_cast<uint8_t>(std::stoi(s.substr(2 * i, 2), nullptr, 16));
  return out;
}
inline std::string hex(const std::vector<uint8_t>& v) {
  static const char* d = "0123456789abcdef";
  std::string s;
  for (uint8_t b : v) { s += d[b >> 4]; s += d[b & 0xF]; }
  return s;
}
}
