#include "opfs_file.h"

#include <climits>

#include <emscripten/em_js.h>

namespace {

// Open handles live in this worker's global scope, keyed by a small int.
EM_ASYNC_JS(int, opfs_js_open, (const char* name, int overwrite), {
  const n = UTF8ToString(name);
  try {
    const root = await navigator.storage.getDirectory();
    const fh = await root.getFileHandle(n, { create: true });
    const h = await fh.createSyncAccessHandle();
    if (!overwrite && h.getSize() > 0) {
      h.close();
      err('[webgs] opfs open ' + n + ': refusing to clobber a non-empty existing file');
      return 0;
    }
    h.truncate(0);
    const t = (globalThis.__webgsOpfs ??= { next: 1, files: new Map() });
    const id = t.next++;
    t.files.set(id, { h, at: 0 });
    return id;
  } catch (e) {
    err('[webgs] opfs open ' + n + ': ' + e);
    return 0;
  }
});

// Written straight from a view on the WASM heap: no copy.
EM_JS(int, opfs_js_write, (int id, const uint8_t* p, int n), {
  const f = globalThis.__webgsOpfs?.files.get(id);
  if (!f) return 0;
  try {
    const w = f.h.write(HEAPU8.subarray(p, p + n), { at: f.at });
    f.at += w;
    return w === n ? 1 : 0;
  } catch (e) {
    err('[webgs] opfs write: ' + e);
    return 0;
  }
});

EM_JS(int, opfs_js_flush, (int id), {
  const f = globalThis.__webgsOpfs?.files.get(id);
  if (!f) return 0;
  try { f.h.flush(); return 1; } catch (e) { err('[webgs] opfs flush: ' + e); return 0; }
});

EM_JS(double, opfs_js_size, (int id), {
  const f = globalThis.__webgsOpfs?.files.get(id);
  try { return f ? f.h.getSize() : -1; } catch (e) { return -1; }
});

EM_JS(void, opfs_js_close, (int id), {
  const t = globalThis.__webgsOpfs;
  const f = t?.files.get(id);
  if (!f) return;
  try { f.h.close(); } catch (e) { err('[webgs] opfs close: ' + e); }
  t.files.delete(id);
});

EM_ASYNC_JS(int, opfs_js_remove, (const char* name), {
  try {
    const root = await navigator.storage.getDirectory();
    await root.removeEntry(UTF8ToString(name));
    return 1;
  } catch (e) {
    return 0;
  }
});

}  // namespace

namespace webgs {

std::unique_ptr<OpfsFile> OpfsFile::open(const std::string& name, bool overwrite) {
  const int id = opfs_js_open(name.c_str(), overwrite ? 1 : 0);
  if (id <= 0) return nullptr;
  return std::unique_ptr<OpfsFile>(new OpfsFile(id, name));
}

bool OpfsFile::remove(const std::string& name) { return opfs_js_remove(name.c_str()) == 1; }

bool OpfsFile::write(const uint8_t* p, size_t n) {
  if (!id_) return false;
  while (n > 0) {
    const int chunk = n > INT_MAX ? INT_MAX : static_cast<int>(n);
    if (!opfs_js_write(id_, p, chunk)) return false;
    bytes_ += static_cast<uint64_t>(chunk);
    p += chunk;
    n -= static_cast<size_t>(chunk);
  }
  return true;
}

bool OpfsFile::flush() { return id_ && opfs_js_flush(id_) == 1; }

void OpfsFile::close() {
  if (!id_) return;
  opfs_js_close(id_);
  id_ = 0;
}

bool opfs_probe() {
  static const uint8_t kByte = 0x6d;  // lives in the WASM heap: tests the heap-view write
  auto f = OpfsFile::open(".probe", /*overwrite=*/true);
  if (!f) return false;
  const bool ok = f->write(&kByte, 1) && f->flush() && opfs_js_size(f->id_) == 1.0;
  f->close();
  OpfsFile::remove(".probe");
  return ok;
}

}  // namespace webgs
