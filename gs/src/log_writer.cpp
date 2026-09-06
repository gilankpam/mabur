#include "log_writer.h"

#include <cerrno>
#include <chrono>
#include <cstring>

namespace maburgs {

namespace {
constexpr auto kIdleSleep = std::chrono::milliseconds(2);
constexpr auto kFlushEvery = std::chrono::milliseconds(1000);
}  // namespace

LogWriter::LogWriter() : buf_(kRingBytes) {
  th_ = std::thread([this] { run_(); });
}

LogWriter::~LogWriter() {
  stop_.store(true, std::memory_order_release);
  if (th_.joinable()) th_.join();
  drain_();  // anything queued after the thread's last pass
  const size_t n = n_outs_.load(std::memory_order_acquire);
  for (size_t i = 0; i < n; ++i) {
    if (outs_[i] && outs_[i]->f) {
      std::fflush(outs_[i]->f);
      std::fclose(outs_[i]->f);
    }
  }
}

LogWriter::Stream LogWriter::open(const std::string& dir, const char* name,
                                  const std::string& header, bool mark_drops) {
  const std::string p = dir + "/" + name;
  std::FILE* f = std::fopen(p.c_str(), "a");
  if (!f) {
    std::fprintf(stderr, "debug-log: fopen '%s' failed: %s\n", p.c_str(),
                 std::strerror(errno));
    return kBadStream;
  }
  const size_t n = n_outs_.load(std::memory_order_relaxed);
  if (n >= kMaxStreams) {
    std::fprintf(stderr, "debug-log: too many streams, '%s' not opened\n",
                 p.c_str());
    std::fclose(f);
    return kBadStream;
  }
  // Big buffer: the writer thread controls when bytes reach the disk, via
  // its own 1 Hz fflush. A power cut loses at most one second.
  std::setvbuf(f, nullptr, _IOFBF, 1 << 16);
  auto out = std::make_unique<Out>();
  out->f = f;
  out->path = p;
  out->mark_drops = mark_drops;
  outs_[n] = std::move(out);
  // Publish the slot only after it is fully built.
  n_outs_.store(n + 1, std::memory_order_release);
  const Stream s = static_cast<Stream>(n);
  // An empty header is a file with no marker line (flight.jsonl). Passing it
  // to line() would be a zero-length record, i.e. a counted drop.
  if (!header.empty()) line(s, header.data(), header.size());
  return s;
}

void LogWriter::put_(uint64_t at, const char* src, size_t n) {
  const size_t off = static_cast<size_t>(at % kRingBytes);
  const size_t first = std::min(n, kRingBytes - off);
  std::memcpy(buf_.data() + off, src, first);
  if (first < n) std::memcpy(buf_.data(), src + first, n - first);
}

void LogWriter::get_(uint64_t at, char* dst, size_t n) const {
  const size_t off = static_cast<size_t>(at % kRingBytes);
  const size_t first = std::min(n, kRingBytes - off);
  std::memcpy(dst, buf_.data() + off, first);
  if (first < n) std::memcpy(dst + first, buf_.data(), n - first);
}

void LogWriter::line(Stream s, const char* text, size_t len) {
  // Producer thread owns n_outs_'s writes, so a relaxed load is enough here.
  if (s < 0 || static_cast<size_t>(s) >= n_outs_.load(std::memory_order_relaxed))
    return;
  Out& o = *outs_[static_cast<size_t>(s)];
  if (len == 0 || len > kMaxLine) {
    o.dropped.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  const uint64_t h = head_.load(std::memory_order_relaxed);
  const uint64_t t = tail_.load(std::memory_order_acquire);
  const size_t need = kRecHdr + len;
  if (kRingBytes - static_cast<size_t>(h - t) < need) {
    o.dropped.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  const uint32_t hdr[2] = {static_cast<uint32_t>(len),
                           static_cast<uint32_t>(s)};
  put_(h, reinterpret_cast<const char*>(hdr), kRecHdr);
  put_(h + kRecHdr, text, len);
  head_.store(h + need, std::memory_order_release);
}

const std::string& LogWriter::path(Stream s) const {
  if (s < 0 || static_cast<size_t>(s) >= n_outs_.load(std::memory_order_acquire))
    return empty_;
  return outs_[static_cast<size_t>(s)]->path;
}

uint64_t LogWriter::dropped(Stream s) const {
  if (s < 0 || static_cast<size_t>(s) >= n_outs_.load(std::memory_order_acquire))
    return 0;
  return outs_[static_cast<size_t>(s)]->dropped.load(std::memory_order_relaxed);
}

void LogWriter::drain_() {
  std::lock_guard<std::mutex> lk(drain_mu_);
  std::vector<char> line_buf(kMaxLine);
  for (;;) {
    const uint64_t t = tail_.load(std::memory_order_relaxed);
    const uint64_t h = head_.load(std::memory_order_acquire);
    if (h - t < kRecHdr) return;
    uint32_t hdr[2];
    get_(t, reinterpret_cast<char*>(hdr), kRecHdr);
    const size_t len = hdr[0];
    const size_t idx = hdr[1];
    if (h - t < kRecHdr + len) return;  // producer mid-write
    get_(t + kRecHdr, line_buf.data(), len);
    tail_.store(t + kRecHdr + len, std::memory_order_release);
    // Re-load per record, not once for the whole call: a single drain_()
    // invocation can run long enough to drain records for a stream that
    // open() published only after this call started (e.g. the writer
    // thread's very first pass, racing the producer's first open()s). A
    // snapshot taken before the loop would then be stale for those later
    // records, and since tail_ has already advanced past them, a stale
    // "idx < n_outs" miss would silently lose the bytes -- not even
    // counted as a drop. This record's bytes were made visible by the
    // head_ acquire-load above, which -- same producer thread, program
    // order -- happens after the n_outs_ release-store for this stream's
    // slot, so acquire-loading n_outs_ here is always current enough for
    // the record just read.
    const size_t n_outs = n_outs_.load(std::memory_order_acquire);
    if (idx < n_outs && outs_[idx]->f) {
      std::fwrite(line_buf.data(), 1, len, outs_[idx]->f);
      std::fputc('\n', outs_[idx]->f);
    }
  }
}

void LogWriter::report_and_flush_() {
  std::lock_guard<std::mutex> lk(drain_mu_);
  const size_t n = n_outs_.load(std::memory_order_acquire);
  for (size_t i = 0; i < n; ++i) {
    auto& o = outs_[i];
    if (!o || !o->f) continue;
    // Report drops INTO the affected file so a gap is visible in the data
    // itself, not only in a counter nobody reads -- except where the format
    // cannot carry a comment line (flight.jsonl), where the counter and the
    // datagram's own seq field are the evidence.
    const uint64_t d = o->dropped.load(std::memory_order_relaxed);
    if (d != o->reported) {
      if (o->mark_drops)
        std::fprintf(o->f, "# dropped %llu\n",
                     static_cast<unsigned long long>(d - o->reported));
      o->reported = d;
    }
    std::fflush(o->f);
  }
}

void LogWriter::run_() {
  auto next_flush = std::chrono::steady_clock::now() + kFlushEvery;
  while (!stop_.load(std::memory_order_acquire)) {
    const uint64_t before = tail_.load(std::memory_order_relaxed);
    drain_();
    const bool idle = tail_.load(std::memory_order_relaxed) == before;
    const auto now = std::chrono::steady_clock::now();
    if (now >= next_flush) {
      next_flush = now + kFlushEvery;
      report_and_flush_();
    }
    if (idle) std::this_thread::sleep_for(kIdleSleep);
  }
}

void LogWriter::flush_now() {
  // The writer thread may be mid-pass; drain what it left and flush here.
  // Safe because drain_() is the only tail_ writer and this is called from
  // the producer thread while the writer is idle-sleeping at most 2 ms.
  std::this_thread::sleep_for(kIdleSleep * 3);
  drain_();
  report_and_flush_();
}

}  // namespace maburgs
