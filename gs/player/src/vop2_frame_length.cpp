// The RK356x VOP2 frame-length control through /dev/mem (HW build only).

#include "vop2_frame_length.h"

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>

#include "frame_length.h"

namespace maburplay {

namespace {

constexpr size_t kMapBytes = 0x1000;
// The video ports whose copy of the register still lies in the mapped page.
constexpr int kMaxCrtcIndex = 3;

class Vop2FrameLength : public FrameLength {
 public:
  Vop2FrameLength(void* map, volatile uint32_t* reg) : map_(map), reg_(reg) {}
  ~Vop2FrameLength() override { ::munmap(map_, kMapBytes); }

  bool take_mode(int vtotal, int sync_lines, Seen* seen) override {
    const uint32_t raw = *reg_;
    const uint32_t v = raw & ~vop2::kImd;
    if (seen) {
      seen->value = v;
      seen->total = vop2::vtotal_of(v);
      seen->held = (raw & vop2::kImd) != 0;
    }
    if (!vop2::is_mode(v, vtotal, sync_lines)) return false;
    orig_ = vop2::mode_value(v, vtotal);
    return true;
  }

  void set_total(int total) override { *reg_ = vop2::total_value(orig_, total); }

  // The mode's own value with the immediate bit clear: it latches at the
  // next frame boundary, as the driver's own writes do, and the held
  // immediate value is gone from then on.
  void restore() override { *reg_ = orig_; }

 private:
  void* map_;
  volatile uint32_t* reg_;
  uint32_t orig_ = 0;  // the register as the mode has it, immediate bit clear
};

}  // namespace

std::unique_ptr<FrameLength> map_frame_length(int crtc_index, std::string* why) {
  auto fail = [why](std::string s) -> std::unique_ptr<FrameLength> {
    if (why) *why = std::move(s);
    return nullptr;
  };
  if (crtc_index < 0 || crtc_index > kMaxCrtcIndex) return fail("no video port for this CRTC");

  // The compatible list is NUL-separated; spaces make it one string to
  // search.
  char compat[512] = {};
  if (FILE* f = std::fopen("/proc/device-tree/compatible", "r")) {
    const size_t n = std::fread(compat, 1, sizeof(compat) - 1, f);
    std::fclose(f);
    for (size_t i = 0; i < n; ++i)
      if (compat[i] == '\0') compat[i] = ' ';
  }
  if (!vop2::on_this_soc(compat)) return fail("not an RK356x");

  const int fd = ::open("/dev/mem", O_RDWR | O_SYNC);
  if (fd < 0) return fail(std::string("/dev/mem: ") + std::strerror(errno));
  void* m = ::mmap(nullptr, kMapBytes, PROT_READ | PROT_WRITE, MAP_SHARED, fd, vop2::kBase);
  const int map_errno = errno;
  ::close(fd);
  if (m == MAP_FAILED) return fail(std::string("mmap: ") + std::strerror(map_errno));
  auto* reg = reinterpret_cast<volatile uint32_t*>(static_cast<uint8_t*>(m) + vop2::kVtotalReg +
                                                   0x100u * static_cast<uint32_t>(crtc_index));
  return std::make_unique<Vop2FrameLength>(m, reg);
}

}  // namespace maburplay
