// Bus half of the startup scan (see card_scan.h). Lives in mabur_gs_radio
// because it needs libusb and devourer; the policy half stays USB-free so
// the boot-race behaviour is testable on a host with no radio attached.
#include <libusb.h>

#include <algorithm>
#include <cstdio>
#include <string>

#include "DeviceProbe.h"
#include "card_scan.h"

namespace maburgs {

std::vector<ScannedCard> enumerate_supported_cards(libusb_context* ctx) {
  std::vector<ScannedCard> out;
  libusb_device** list = nullptr;
  const ssize_t n = libusb_get_device_list(ctx, &list);
  for (ssize_t i = 0; i < n; ++i) {
    libusb_device_descriptor dd{};
    if (libusb_get_device_descriptor(list[i], &dd) != 0) continue;
    // Two gates before anything is opened: devourer's candidate set (Realtek
    // VID, or a seller id in its PID tables) and then the chip itself. A
    // Realtek card reader clears the first and fails the second by STALLing
    // one control read -- it is never claimed or reset.
    if (!devourer::is_probe_candidate(dd.idVendor, dd.idProduct)) continue;
    const devourer::ChipGeneration gen = devourer::probe_generation(list[i]);
    if (gen == devourer::ChipGeneration::Unknown) continue;

    ScannedCard c;
    c.bus = libusb_get_bus_number(list[i]);
    uint8_t ports[8] = {0};
    const int np = libusb_get_port_numbers(list[i], ports, sizeof(ports));
    if (np > 0) c.port_path.assign(ports, ports + np);
    c.usb_vid = dd.idVendor;
    c.usb_pid = dd.idProduct;
    out.push_back(std::move(c));
  }
  if (list != nullptr) libusb_free_device_list(list, 1);
  return out;
}

bool device_at_port(libusb_device* dev, const ScannedCard& card) {
  if (dev == nullptr) return false;
  if (libusb_get_bus_number(dev) != card.bus) return false;
  uint8_t ports[8] = {0};
  const int np = libusb_get_port_numbers(dev, ports, sizeof(ports));
  if (np <= 0) return card.port_path.empty();
  return card.port_path.size() == static_cast<size_t>(np) &&
         std::equal(card.port_path.begin(), card.port_path.end(), ports);
}

std::string port_name(const ScannedCard& c) {
  std::string s = std::to_string(c.bus) + "-";
  for (size_t i = 0; i < c.port_path.size(); ++i)
    s += (i ? "." : "") + std::to_string(c.port_path[i]);
  return s;
}

}  // namespace maburgs
