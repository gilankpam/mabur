#ifndef MABUR_PLAYER_FB_CACHE_H_
#define MABUR_PLAYER_FB_CACHE_H_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

namespace maburplay {

// Keeps a decoded frame's imported GEM handle and its KMS framebuffer alive
// across flips, so a buffer the decoder's pool hands out again costs a
// lookup instead of a fresh import.
//
// Why: importing a dma-buf (drmPrimeFDToHandle) is a dma-buf attach plus
// an IOMMU map of the whole picture -- 2.6 ms typical and up to ~7 ms on
// the GS -- and closing the handle after every flip meant paying it on
// every frame, inside the release lead (display.vsync_lead_ms). A frame
// whose import overran the lead latched a vblank late and the frame behind
// it was dropped at the presenter's mailbox: the clean-link repeat+skip
// pair. While the handle stays open the kernel's second import of the same
// dma-buf is a hash lookup that returns the existing handle, so the handle
// is the key: the same dma-buf always maps to the same handle, and a new
// dma-buf can only reuse a handle number this cache has already closed.
//
// Bounded (max_entries) with LRU eviction so a decoder that reallocates its
// pool cannot pin freed buffers forever; entries in use by a slot (on
// screen, pending, parked) are never evicted. flush() frees everything --
// call it wherever the backend's buffers are about to be reset.
class FbCache {
 public:
  struct Geom {
    int width = 0, height = 0, stride = 0, vstride = 0;
    uint32_t fourcc = 0;
    bool operator==(const Geom& o) const {
      return width == o.width && height == o.height && stride == o.stride &&
             vstride == o.vstride && fourcc == o.fourcc;
    }
  };
  struct Ops {
    std::function<bool(int dmabuf_fd, uint32_t* handle)> import;
    std::function<bool(uint32_t handle, const Geom& g, uint32_t* fb_id)> add_fb;
    std::function<void(uint32_t fb_id)> rm_fb;
    std::function<void(uint32_t handle)> close_handle;
  };
  struct Entry {
    uint32_t handle = 0;
    uint32_t fb_id = 0;
    Geom geom;
    uint64_t last_use = 0;
  };
  using Pinned = std::function<bool(uint32_t handle)>;

  FbCache(Ops ops, size_t max_entries) : ops_(std::move(ops)), max_(max_entries) {}
  ~FbCache() { flush(); }
  FbCache(const FbCache&) = delete;
  FbCache& operator=(const FbCache&) = delete;

  // The frame's framebuffer, importing and adding only when needed. 0 on
  // failure (nothing cached). `now` orders the LRU; `pinned` names handles
  // a slot still references, which eviction skips.
  uint32_t acquire(int dmabuf_fd, const Geom& g, uint64_t now, const Pinned& pinned,
                   uint32_t* handle_out) {
    uint32_t handle = 0;
    if (!ops_.import(dmabuf_fd, &handle) || handle == 0) return 0;
    ++imports_;
    if (handle_out) *handle_out = handle;
    for (Entry& e : entries_) {
      if (e.handle != handle) continue;
      ++hits_;
      e.last_use = now;
      if (e.geom == g) return e.fb_id;
      // Same buffer, new geometry (a resolution change on a reused pool
      // slot): replace the framebuffer, keep the handle.
      ops_.rm_fb(e.fb_id);
      e.fb_id = 0;
      if (!ops_.add_fb(handle, g, &e.fb_id)) {
        remove_(handle, /*close=*/true);
        return 0;
      }
      ++adds_;
      e.geom = g;
      return e.fb_id;
    }
    if (entries_.size() >= max_) evict_lru_(pinned);
    Entry e;
    e.handle = handle;
    e.geom = g;
    e.last_use = now;
    if (!ops_.add_fb(handle, g, &e.fb_id)) {
      ops_.close_handle(handle);
      return 0;
    }
    ++adds_;
    entries_.push_back(e);
    return e.fb_id;
  }

  void flush() {
    for (const Entry& e : entries_) {
      ops_.rm_fb(e.fb_id);
      ops_.close_handle(e.handle);
    }
    entries_.clear();
  }

  size_t size() const { return entries_.size(); }
  uint64_t imports() const { return imports_; }
  uint64_t hits() const { return hits_; }
  uint64_t adds() const { return adds_; }
  uint64_t evictions() const { return evictions_; }

 private:
  void remove_(uint32_t handle, bool close) {
    for (size_t i = 0; i < entries_.size(); ++i) {
      if (entries_[i].handle != handle) continue;
      if (entries_[i].fb_id) ops_.rm_fb(entries_[i].fb_id);
      if (close) ops_.close_handle(handle);
      entries_.erase(entries_.begin() + static_cast<long>(i));
      return;
    }
  }
  void evict_lru_(const Pinned& pinned) {
    int victim = -1;
    for (size_t i = 0; i < entries_.size(); ++i) {
      if (pinned && pinned(entries_[i].handle)) continue;
      if (victim < 0 || entries_[i].last_use < entries_[static_cast<size_t>(victim)].last_use)
        victim = static_cast<int>(i);
    }
    if (victim < 0) return;  // everything in use: grow by one rather than break a slot
    ++evictions_;
    remove_(entries_[static_cast<size_t>(victim)].handle, /*close=*/true);
  }

  Ops ops_;
  size_t max_;
  std::vector<Entry> entries_;
  uint64_t imports_ = 0, hits_ = 0, adds_ = 0, evictions_ = 0;
};

}  // namespace maburplay

#endif  // MABUR_PLAYER_FB_CACHE_H_
