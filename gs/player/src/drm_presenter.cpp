#include "drm_presenter.h"

#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>

#include <drm_fourcc.h>
#include <xf86drm.h>
#include <xf86drmMode.h>

// Real KMS atomic implementation (Task 9). See drm_presenter.h for the
// ownership/flush-ordering contract. All libdrm includes live here, per
// the Task-7 design constraint that the header stays SDK-free.
namespace maburplay {

namespace {

constexpr const char* kCardPath = "/dev/dri/card0";

// Resolves a KMS object property id by name (and optionally its current
// value) via drmModeObjectGetProperties + drmModeGetProperty. Returns 0 if
// not found -- every call site treats that as a hard init failure except
// the plane "type" lookup, which tolerates absence (legacy-only drivers).
uint32_t find_property(int fd, uint32_t obj_id, uint32_t obj_type, const char* name,
                        uint64_t* value_out = nullptr) {
  drmModeObjectPropertiesPtr props = drmModeObjectGetProperties(fd, obj_id, obj_type);
  if (!props) return 0;
  uint32_t id = 0;
  for (uint32_t i = 0; i < props->count_props; ++i) {
    drmModePropertyPtr prop = drmModeGetProperty(fd, props->props[i]);
    if (!prop) continue;
    if (std::strcmp(prop->name, name) == 0) {
      id = prop->prop_id;
      if (value_out) *value_out = props->prop_values[i];
      drmModeFreeProperty(prop);
      break;
    }
    drmModeFreeProperty(prop);
  }
  drmModeFreeObjectProperties(props);
  return id;
}

void close_gem_handle(int fd, uint32_t handle) {
  if (fd < 0 || !handle) return;
  struct drm_gem_close req {};
  req.handle = handle;
  ioctl(fd, DRM_IOCTL_GEM_CLOSE, &req);
}

// The ten standard properties of a KMS plane object, resolved once.
struct PlaneProps {
  uint32_t fb_id = 0, crtc_id = 0;
  uint32_t src_x = 0, src_y = 0, src_w = 0, src_h = 0;
  uint32_t crtc_x = 0, crtc_y = 0, crtc_w = 0, crtc_h = 0;

  bool resolve(int fd, uint32_t plane_id) {
    fb_id = find_property(fd, plane_id, DRM_MODE_OBJECT_PLANE, "FB_ID");
    crtc_id = find_property(fd, plane_id, DRM_MODE_OBJECT_PLANE, "CRTC_ID");
    src_x = find_property(fd, plane_id, DRM_MODE_OBJECT_PLANE, "SRC_X");
    src_y = find_property(fd, plane_id, DRM_MODE_OBJECT_PLANE, "SRC_Y");
    src_w = find_property(fd, plane_id, DRM_MODE_OBJECT_PLANE, "SRC_W");
    src_h = find_property(fd, plane_id, DRM_MODE_OBJECT_PLANE, "SRC_H");
    crtc_x = find_property(fd, plane_id, DRM_MODE_OBJECT_PLANE, "CRTC_X");
    crtc_y = find_property(fd, plane_id, DRM_MODE_OBJECT_PLANE, "CRTC_Y");
    crtc_w = find_property(fd, plane_id, DRM_MODE_OBJECT_PLANE, "CRTC_W");
    crtc_h = find_property(fd, plane_id, DRM_MODE_OBJECT_PLANE, "CRTC_H");
    return fb_id && crtc_id && src_x && src_y && src_w && src_h && crtc_x && crtc_y && crtc_w &&
           crtc_h;
  }
};

}  // namespace

struct DrmPresenter::Impl {
  int fd = -1;
  ReleaseFn release;

  uint32_t connector_id = 0;
  uint32_t crtc_id = 0;
  int crtc_index = -1;
  drmModeModeInfo mode{};
  uint32_t mode_blob_id = 0;

  uint32_t plane_id = 0;   // the NV12 plane we actually present on
  bool plane_is_primary = false;
  PlaneProps video_props;

  // Primary-black fallback (brief: "else overlay + keep primary black via
  // a dumb buffer") -- only populated when plane_is_primary is false.
  uint32_t primary_plane_id = 0;
  uint32_t primary_fb_id = 0;
  uint32_t primary_gem_handle = 0;
  PlaneProps primary_props;

  uint32_t prop_connector_crtc_id = 0;
  uint32_t prop_crtc_mode_id = 0;
  uint32_t prop_crtc_active = 0;

  bool inited = false;
  bool needs_modeset = true;  // next commit must be a full blocking ALLOW_MODESET commit
  bool flip_pending = false;  // a NONBLOCK|PAGE_FLIP_EVENT commit is outstanding

  bool async_probed = false;
  bool async_active = false;

  uint64_t commit_errors = 0;
  uint64_t frames_dropped_busy = 0;

  struct Slot {
    bool valid = false;
    uint32_t fb_id = 0;
    uint32_t gem_handle = 0;
    DmaFrame frame;
  };
  Slot on_screen;
  Slot pending;

  ~Impl();

  bool init(const std::string& screen_mode, ReleaseFn rel);
  bool present(const DmaFrame& frame);
  void poll_events();
  void drop_all();

  void release_slot(Slot& s);
  void on_flip();

  static void on_flip_static(int fd, unsigned int sequence, unsigned int tv_sec,
                              unsigned int tv_usec, unsigned int crtc_id, void* user_data);
};

void DrmPresenter::Impl::release_slot(Slot& s) {
  if (!s.valid) return;
  if (fd >= 0) {
    drmModeRmFB(fd, s.fb_id);
    close_gem_handle(fd, s.gem_handle);
  }
  if (release) release(s.frame);
  s = Slot{};
}

void DrmPresenter::Impl::on_flip() {
  flip_pending = false;
  if (!pending.valid) return;  // spurious/duplicate event -- nothing to promote
  if (on_screen.valid) release_slot(on_screen);
  on_screen = pending;
  pending = Slot{};
}

void DrmPresenter::Impl::on_flip_static(int /*fd*/, unsigned int /*sequence*/,
                                         unsigned int /*tv_sec*/, unsigned int /*tv_usec*/,
                                         unsigned int /*crtc_id*/, void* user_data) {
  auto* self = static_cast<Impl*>(user_data);
  if (self) self->on_flip();
}

bool DrmPresenter::Impl::init(const std::string& screen_mode, ReleaseFn rel) {
  release = std::move(rel);

  fd = open(kCardPath, O_RDWR | O_CLOEXEC);
  if (fd < 0) {
    std::fprintf(stderr, "DrmPresenter: open(%s) failed: %s\n", kCardPath, std::strerror(errno));
    return false;
  }

  if (drmSetClientCap(fd, DRM_CLIENT_CAP_UNIVERSAL_PLANES, 1) != 0 ||
      drmSetClientCap(fd, DRM_CLIENT_CAP_ATOMIC, 1) != 0) {
    std::fprintf(stderr, "DrmPresenter: drmSetClientCap(ATOMIC/UNIVERSAL_PLANES) failed: %s\n",
                 std::strerror(errno));
    return false;
  }

  drmModeResPtr res = drmModeGetResources(fd);
  if (!res) {
    std::fprintf(stderr, "DrmPresenter: drmModeGetResources failed: %s\n", std::strerror(errno));
    return false;
  }

  // Connector select: prefer a connected HDMI-A-* connector; fall back to
  // any other connected connector with a warning.
  drmModeConnectorPtr chosen = nullptr;
  for (int i = 0; i < res->count_connectors; ++i) {
    drmModeConnectorPtr c = drmModeGetConnector(fd, res->connectors[i]);
    if (!c) continue;
    if (c->connection != DRM_MODE_CONNECTED) {
      drmModeFreeConnector(c);
      continue;
    }
    if (c->connector_type == DRM_MODE_CONNECTOR_HDMIA) {
      if (chosen) drmModeFreeConnector(chosen);
      chosen = c;
      break;
    }
    if (!chosen) {
      chosen = c;  // keep as fallback candidate; keep scanning for HDMI-A
    } else {
      drmModeFreeConnector(c);
    }
  }
  if (!chosen) {
    std::fprintf(stderr, "DrmPresenter: no connected connector found\n");
    drmModeFreeResources(res);
    return false;
  }
  const bool is_hdmi = chosen->connector_type == DRM_MODE_CONNECTOR_HDMIA;
  if (!is_hdmi) {
    std::fprintf(stderr,
                 "DrmPresenter: warning: no connected HDMI-A connector; using connector "
                 "%u (type %u) instead\n",
                 chosen->connector_id, chosen->connector_type);
  }
  connector_id = chosen->connector_id;

  // Mode select: "WIDTHxHEIGHT@FPS" against the connector's mode list;
  // fall back to the connector's DRM_MODE_TYPE_PREFERRED mode (or its
  // first mode if none is flagged preferred) with a warning.
  int want_w = 0, want_h = 0, want_fps = 0;
  const bool have_want =
      std::sscanf(screen_mode.c_str(), "%dx%d@%d", &want_w, &want_h, &want_fps) == 3;
  const drmModeModeInfo* selected = nullptr;
  const drmModeModeInfo* preferred = nullptr;
  for (int i = 0; i < chosen->count_modes; ++i) {
    const drmModeModeInfo* m = &chosen->modes[i];
    if (m->type & DRM_MODE_TYPE_PREFERRED) preferred = m;
    if (have_want && static_cast<int>(m->hdisplay) == want_w &&
        static_cast<int>(m->vdisplay) == want_h && static_cast<int>(m->vrefresh) == want_fps) {
      selected = m;
    }
  }
  if (!selected) {
    selected = preferred ? preferred : (chosen->count_modes > 0 ? &chosen->modes[0] : nullptr);
    if (selected) {
      std::fprintf(stderr,
                   "DrmPresenter: warning: screen_mode \"%s\" not offered by connector %u; "
                   "falling back to %ux%u@%u\n",
                   screen_mode.c_str(), connector_id, selected->hdisplay, selected->vdisplay,
                   selected->vrefresh);
    }
  }
  if (!selected) {
    std::fprintf(stderr, "DrmPresenter: connector %u has no modes\n", connector_id);
    drmModeFreeConnector(chosen);
    drmModeFreeResources(res);
    return false;
  }
  mode = *selected;

  // CRTC select: first CRTC any of the connector's possible encoders can
  // drive.
  for (int i = 0; i < chosen->count_encoders && crtc_id == 0; ++i) {
    drmModeEncoderPtr enc = drmModeGetEncoder(fd, chosen->encoders[i]);
    if (!enc) continue;
    for (int j = 0; j < res->count_crtcs; ++j) {
      if (enc->possible_crtcs & (1u << j)) {
        crtc_id = res->crtcs[j];
        crtc_index = j;
        break;
      }
    }
    drmModeFreeEncoder(enc);
  }
  drmModeFreeConnector(chosen);
  drmModeFreeResources(res);
  if (crtc_id == 0) {
    std::fprintf(stderr, "DrmPresenter: no usable CRTC for connector %u\n", connector_id);
    return false;
  }

  prop_connector_crtc_id = find_property(fd, connector_id, DRM_MODE_OBJECT_CONNECTOR, "CRTC_ID");
  prop_crtc_mode_id = find_property(fd, crtc_id, DRM_MODE_OBJECT_CRTC, "MODE_ID");
  prop_crtc_active = find_property(fd, crtc_id, DRM_MODE_OBJECT_CRTC, "ACTIVE");
  if (!prop_connector_crtc_id || !prop_crtc_mode_id || !prop_crtc_active) {
    std::fprintf(stderr, "DrmPresenter: missing connector/crtc CRTC_ID|MODE_ID|ACTIVE property\n");
    return false;
  }

  // Plane select: iterate planes usable on our CRTC, prefer a PRIMARY-type
  // plane that lists DRM_FORMAT_NV12, else take the first OVERLAY-type one
  // that does (RK3566: the Esmart planes take NV12, per the brief).
  drmModePlaneResPtr pres = drmModeGetPlaneResources(fd);
  if (!pres) {
    std::fprintf(stderr, "DrmPresenter: drmModeGetPlaneResources failed: %s\n",
                 std::strerror(errno));
    return false;
  }
  uint32_t overlay_candidate = 0;
  for (uint32_t i = 0; i < pres->count_planes; ++i) {
    drmModePlanePtr p = drmModeGetPlane(fd, pres->planes[i]);
    if (!p) continue;
    if (!(p->possible_crtcs & (1u << crtc_index))) {
      drmModeFreePlane(p);
      continue;
    }
    bool has_nv12 = false;
    for (uint32_t f = 0; f < p->count_formats; ++f) {
      if (p->formats[f] == DRM_FORMAT_NV12) {
        has_nv12 = true;
        break;
      }
    }
    if (has_nv12) {
      uint64_t type_val = 0;
      find_property(fd, p->plane_id, DRM_MODE_OBJECT_PLANE, "type", &type_val);
      if (type_val == DRM_PLANE_TYPE_PRIMARY) {
        plane_id = p->plane_id;
        plane_is_primary = true;
        drmModeFreePlane(p);
        break;
      }
      if (!overlay_candidate) overlay_candidate = p->plane_id;
    }
    drmModeFreePlane(p);
  }
  drmModeFreePlaneResources(pres);

  if (!plane_id && overlay_candidate) {
    plane_id = overlay_candidate;
    plane_is_primary = false;
  }
  if (!plane_id) {
    std::fprintf(stderr, "DrmPresenter: no NV12-capable plane on CRTC %u\n", crtc_id);
    return false;
  }
  if (!video_props.resolve(fd, plane_id)) {
    std::fprintf(stderr, "DrmPresenter: missing standard property on plane %u\n", plane_id);
    return false;
  }

  // Overlay case: find the real primary plane on this CRTC and give it a
  // black dumb-buffer FB so it doesn't show stale/undefined content behind
  // the video overlay. Best-effort: failure here is logged, not fatal --
  // the video overlay itself still works, just possibly over garbage.
  if (!plane_is_primary) {
    drmModePlaneResPtr pres2 = drmModeGetPlaneResources(fd);
    if (pres2) {
      for (uint32_t i = 0; i < pres2->count_planes && !primary_plane_id; ++i) {
        drmModePlanePtr p = drmModeGetPlane(fd, pres2->planes[i]);
        if (!p) continue;
        if (p->possible_crtcs & (1u << crtc_index)) {
          uint64_t type_val = 0;
          find_property(fd, p->plane_id, DRM_MODE_OBJECT_PLANE, "type", &type_val);
          if (type_val == DRM_PLANE_TYPE_PRIMARY) primary_plane_id = p->plane_id;
        }
        drmModeFreePlane(p);
      }
      drmModeFreePlaneResources(pres2);
    }
    if (primary_plane_id && primary_props.resolve(fd, primary_plane_id)) {
      uint32_t handle = 0, pitch = 0;
      uint64_t size = 0;
      if (drmModeCreateDumbBuffer(fd, mode.hdisplay, mode.vdisplay, 32, 0, &handle, &pitch,
                                  &size) == 0) {
        uint64_t map_offset = 0;
        if (drmModeMapDumbBuffer(fd, handle, &map_offset) == 0) {
          void* map = mmap(nullptr, size, PROT_WRITE, MAP_SHARED, fd,
                           static_cast<off_t>(map_offset));
          if (map != MAP_FAILED) {
            std::memset(map, 0, size);
            munmap(map, size);
          } else {
            std::fprintf(stderr, "DrmPresenter: warning: mmap black-primary buffer failed: %s\n",
                         std::strerror(errno));
          }
        }
        uint32_t handles4[4] = {handle, 0, 0, 0};
        uint32_t pitches4[4] = {pitch, 0, 0, 0};
        uint32_t offsets4[4] = {0, 0, 0, 0};
        uint32_t fbid = 0;
        if (drmModeAddFB2(fd, mode.hdisplay, mode.vdisplay, DRM_FORMAT_XRGB8888, handles4,
                          pitches4, offsets4, &fbid, 0) == 0) {
          primary_fb_id = fbid;
          primary_gem_handle = handle;
        } else {
          std::fprintf(stderr, "DrmPresenter: warning: black-primary AddFB2 failed: %s\n",
                       std::strerror(errno));
          drmModeDestroyDumbBuffer(fd, handle);
        }
      } else {
        std::fprintf(stderr, "DrmPresenter: warning: black-primary dumb buffer create failed: %s\n",
                     std::strerror(errno));
      }
    } else {
      std::fprintf(stderr,
                   "DrmPresenter: warning: no primary plane found to blank on CRTC %u (video "
                   "overlay may show stale content underneath)\n",
                   crtc_id);
    }
  }

  if (drmModeCreatePropertyBlob(fd, &mode, sizeof(mode), &mode_blob_id) != 0) {
    std::fprintf(stderr, "DrmPresenter: drmModeCreatePropertyBlob(mode) failed: %s\n",
                 std::strerror(errno));
    return false;
  }

  std::fprintf(stderr,
               "DrmPresenter: connector %u (%s) mode %ux%u@%u, crtc %u, %s NV12 plane %u%s\n",
               connector_id, is_hdmi ? "HDMI-A" : "non-HDMI-A", mode.hdisplay, mode.vdisplay,
               mode.vrefresh, crtc_id, plane_is_primary ? "primary" : "overlay", plane_id,
               primary_fb_id ? " (+ black primary backdrop)" : "");

  inited = true;
  return true;
}

bool DrmPresenter::Impl::present(const DmaFrame& frame) {
  if (!inited || fd < 0) {
    if (release) release(frame);
    return false;
  }

  if (flip_pending) {
    // Backpressure: the previous NONBLOCK commit's flip event hasn't
    // landed yet. The contract keeps only two buffers alive (on-screen +
    // queued); rather than block or grow a third slot, drop this frame --
    // by the time the queued one flips it would already be stale.
    if (release) release(frame);
    ++frames_dropped_busy;
    if (frames_dropped_busy == 1 || frames_dropped_busy % 100 == 0) {
      std::fprintf(stderr, "DrmPresenter: dropped frame while flip pending (count=%llu)\n",
                   static_cast<unsigned long long>(frames_dropped_busy));
    }
    return false;
  }

  uint32_t handle = 0;
  if (drmPrimeFDToHandle(fd, frame.dmabuf_fd, &handle) != 0) {
    std::fprintf(stderr, "DrmPresenter: drmPrimeFDToHandle failed: %s\n", std::strerror(errno));
    ++commit_errors;
    if (release) release(frame);
    return false;
  }

  const uint32_t handles[4] = {handle, handle, 0, 0};
  const uint32_t pitches[4] = {static_cast<uint32_t>(frame.stride),
                               static_cast<uint32_t>(frame.stride), 0, 0};
  const uint32_t offsets[4] = {0, static_cast<uint32_t>(frame.stride * frame.vstride), 0, 0};
  uint32_t fb_id = 0;
  if (drmModeAddFB2(fd, static_cast<uint32_t>(frame.width), static_cast<uint32_t>(frame.height),
                    DRM_FORMAT_NV12, handles, pitches, offsets, &fb_id, 0) != 0) {
    std::fprintf(stderr, "DrmPresenter: drmModeAddFB2 failed: %s\n", std::strerror(errno));
    ++commit_errors;
    close_gem_handle(fd, handle);
    if (release) release(frame);
    return false;
  }

  drmModeAtomicReqPtr req = drmModeAtomicAlloc();
  if (!req) {
    ++commit_errors;
    drmModeRmFB(fd, fb_id);
    close_gem_handle(fd, handle);
    if (release) release(frame);
    return false;
  }

  drmModeAtomicAddProperty(req, plane_id, video_props.fb_id, fb_id);
  drmModeAtomicAddProperty(req, plane_id, video_props.crtc_id, crtc_id);
  drmModeAtomicAddProperty(req, plane_id, video_props.src_x, 0);
  drmModeAtomicAddProperty(req, plane_id, video_props.src_y, 0);
  drmModeAtomicAddProperty(req, plane_id, video_props.src_w,
                           static_cast<uint64_t>(frame.width) << 16);
  drmModeAtomicAddProperty(req, plane_id, video_props.src_h,
                           static_cast<uint64_t>(frame.height) << 16);
  drmModeAtomicAddProperty(req, plane_id, video_props.crtc_x, 0);
  drmModeAtomicAddProperty(req, plane_id, video_props.crtc_y, 0);
  drmModeAtomicAddProperty(req, plane_id, video_props.crtc_w, mode.hdisplay);
  drmModeAtomicAddProperty(req, plane_id, video_props.crtc_h, mode.vdisplay);

  const bool do_modeset = needs_modeset;
  if (do_modeset) {
    drmModeAtomicAddProperty(req, connector_id, prop_connector_crtc_id, crtc_id);
    drmModeAtomicAddProperty(req, crtc_id, prop_crtc_mode_id, mode_blob_id);
    drmModeAtomicAddProperty(req, crtc_id, prop_crtc_active, 1);
    if (primary_fb_id) {
      drmModeAtomicAddProperty(req, primary_plane_id, primary_props.fb_id, primary_fb_id);
      drmModeAtomicAddProperty(req, primary_plane_id, primary_props.crtc_id, crtc_id);
      drmModeAtomicAddProperty(req, primary_plane_id, primary_props.src_x, 0);
      drmModeAtomicAddProperty(req, primary_plane_id, primary_props.src_y, 0);
      drmModeAtomicAddProperty(req, primary_plane_id, primary_props.src_w,
                               static_cast<uint64_t>(mode.hdisplay) << 16);
      drmModeAtomicAddProperty(req, primary_plane_id, primary_props.src_h,
                               static_cast<uint64_t>(mode.vdisplay) << 16);
      drmModeAtomicAddProperty(req, primary_plane_id, primary_props.crtc_x, 0);
      drmModeAtomicAddProperty(req, primary_plane_id, primary_props.crtc_y, 0);
      drmModeAtomicAddProperty(req, primary_plane_id, primary_props.crtc_w, mode.hdisplay);
      drmModeAtomicAddProperty(req, primary_plane_id, primary_props.crtc_h, mode.vdisplay);
    }
  }

  uint32_t flags = do_modeset
                       ? static_cast<uint32_t>(DRM_MODE_ATOMIC_ALLOW_MODESET)
                       : static_cast<uint32_t>(DRM_MODE_ATOMIC_NONBLOCK | DRM_MODE_PAGE_FLIP_EVENT);
  const bool probing_async = !do_modeset && !async_probed;
  if (!do_modeset && (probing_async || async_active)) flags |= DRM_MODE_PAGE_FLIP_ASYNC;

  int rc = drmModeAtomicCommit(fd, req, flags, this);
  if (rc != 0 && probing_async && rc == -EINVAL) {
    async_probed = true;
    async_active = false;
    std::fprintf(stderr,
                 "DrmPresenter: PAGE_FLIP_ASYNC rejected (EINVAL) -- latched to vsync-paced "
                 "flips\n");
    flags &= ~static_cast<uint32_t>(DRM_MODE_PAGE_FLIP_ASYNC);
    rc = drmModeAtomicCommit(fd, req, flags, this);
  } else if (rc == 0 && probing_async) {
    async_probed = true;
    async_active = true;
    std::fprintf(stderr, "DrmPresenter: PAGE_FLIP_ASYNC accepted -- async flips active\n");
  }

  drmModeAtomicFree(req);

  if (rc != 0) {
    const int err = rc < 0 ? -rc : errno;
    std::fprintf(stderr, "DrmPresenter: drmModeAtomicCommit failed (modeset=%d): %s\n", do_modeset,
                 std::strerror(err));
    ++commit_errors;
    drmModeRmFB(fd, fb_id);
    close_gem_handle(fd, handle);
    if (release) release(frame);
    return false;
  }

  Slot new_slot;
  new_slot.valid = true;
  new_slot.fb_id = fb_id;
  new_slot.gem_handle = handle;
  new_slot.frame = frame;

  if (do_modeset) {
    // Blocking commit, no flip event requested -- this frame is on screen
    // the instant the ioctl returns.
    if (on_screen.valid) release_slot(on_screen);  // defensive; shouldn't normally be reachable
    on_screen = new_slot;
    needs_modeset = false;
  } else {
    pending = new_slot;
    flip_pending = true;
  }
  return true;
}

void DrmPresenter::Impl::poll_events() {
  if (fd < 0) return;
  struct pollfd pfd {
    fd, POLLIN, 0
  };
  if (poll(&pfd, 1, 0) <= 0) return;
  if (!(pfd.revents & POLLIN)) return;

  drmEventContext evctx{};
  evctx.version = 3;  // page_flip_handler2 requires version >= 3
  evctx.page_flip_handler2 = &Impl::on_flip_static;
  drmHandleEvent(fd, &evctx);
}

void DrmPresenter::Impl::drop_all() {
  // Best-effort: pick up an already-completed flip first so a frame that
  // finished scanning out isn't torn down as "still pending" merely
  // because its event hasn't been drained yet.
  poll_events();

  // Flush-ordering contract (carried from Task 8's review): every held
  // DmaFrame is released back to the backend right now, unconditionally,
  // BEFORE the caller is allowed to call backend->flush()/mpi->reset().
  // `pending`'s FB may still be referenced by a commit the kernel hasn't
  // confirmed as flipped -- tearing it down here is the same "unverified
  // interaction" flagged in the carried requirement; see the report.
  if (pending.valid) release_slot(pending);
  if (on_screen.valid) release_slot(on_screen);
  flip_pending = false;
  needs_modeset = true;  // plane content is gone; next present() must redo the full commit
}

DrmPresenter::Impl::~Impl() {
  if (pending.valid) release_slot(pending);
  if (on_screen.valid) release_slot(on_screen);
  if (fd >= 0) {
    if (primary_fb_id) drmModeRmFB(fd, primary_fb_id);
    if (primary_gem_handle) drmModeDestroyDumbBuffer(fd, primary_gem_handle);
    if (mode_blob_id) drmModeDestroyPropertyBlob(fd, mode_blob_id);
    close(fd);
  }
}

DrmPresenter::DrmPresenter() : impl_(std::make_unique<Impl>()) {}
DrmPresenter::~DrmPresenter() = default;

bool DrmPresenter::init(const std::string& screen_mode, ReleaseFn release) {
  return impl_->init(screen_mode, std::move(release));
}

bool DrmPresenter::present(const DmaFrame& frame) { return impl_->present(frame); }

void DrmPresenter::poll_events() { impl_->poll_events(); }

void DrmPresenter::drop_all() { impl_->drop_all(); }

uint64_t DrmPresenter::commit_errors() const { return impl_->commit_errors; }

bool DrmPresenter::async_flip_active() const { return impl_->async_active; }

bool DrmPresenter::async_probed() const { return impl_->async_probed; }

}  // namespace maburplay
