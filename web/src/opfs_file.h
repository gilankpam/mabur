#ifndef WEBGS_OPFS_FILE_H_
#define WEBGS_OPFS_FILE_H_

// A file in the browser's Origin Private File System, written from the core
// pthread -- a dedicated Worker, where FileSystemSyncAccessHandle exists
// (spec 2026-09-28-web-local-recording §1.3). Page build only. open() and
// remove() await promises through ASYNCIFY: call them from the core loop,
// never from inside an AU callback. write()/flush()/close() are synchronous.

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace webgs {

class OpfsFile {
 public:
  // Creates `name` in the OPFS root. Null on any failure. A same-named file
  // left over from an earlier recording (or a same-second name collision)
  // is never clobbered: open() refuses and returns null unless `overwrite`
  // is set, which truncates it -- only opfs_probe()'s ".probe" file passes
  // that, so a leftover non-empty probe from a crash doesn't wedge the
  // startup check forever.
  static std::unique_ptr<OpfsFile> open(const std::string& name, bool overwrite = false);
  static bool remove(const std::string& name);

  ~OpfsFile() { close(); }
  OpfsFile(const OpfsFile&) = delete;
  OpfsFile& operator=(const OpfsFile&) = delete;

  // All n bytes at the current end, or false.
  bool write(const uint8_t* p, size_t n);
  bool flush();
  void close();
  uint64_t bytes() const { return bytes_; }
  const std::string& name() const { return name_; }

 private:
  friend bool opfs_probe();  // reads the size of its probe file through id_
  OpfsFile(int id, std::string name) : id_(id), name_(std::move(name)) {}
  int id_;  // 0 once closed
  std::string name_;
  uint64_t bytes_ = 0;
};

// Startup check: create ".probe", write one byte from the WASM heap, flush,
// check the size, close, remove. False = no usable OPFS in this browser.
bool opfs_probe();

}  // namespace webgs

#endif  // WEBGS_OPFS_FILE_H_
