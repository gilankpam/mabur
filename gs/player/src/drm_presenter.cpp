#include "drm_presenter.h"

#include "osd_surface.h"

#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <ctime>

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

// Resolves a plane's mutable "zpos" range property: id + [min, max]. Returns
// 0 when the plane has no zpos prop (immutable stacking; caller logs and
// hopes the defaults are sane). vop2 exposes zpos on every window.
uint32_t find_zpos_range(int fd, uint32_t plane_id, uint64_t* min_out, uint64_t* max_out) {
  drmModeObjectPropertiesPtr props =
      drmModeObjectGetProperties(fd, plane_id, DRM_MODE_OBJECT_PLANE);
  if (!props) return 0;
  uint32_t id = 0;
  for (uint32_t i = 0; i < props->count_props; ++i) {
    drmModePropertyPtr prop = drmModeGetProperty(fd, props->props[i]);
    if (!prop) continue;
    if (std::strcmp(prop->name, "zpos") == 0 && !(prop->flags & DRM_MODE_PROP_IMMUTABLE) &&
        prop->count_values >= 2) {
      id = prop->prop_id;
      *min_out = static_cast<uint64_t>(prop->values[0]);
      *max_out = static_cast<uint64_t>(prop->values[1]);
      drmModeFreeProperty(prop);
      break;
    }
    drmModeFreeProperty(prop);
  }
  drmModeFreeObjectProperties(props);
  return id;
}

// Resolves an enum property by name AND the numeric value of one of its
// named entries (e.g. "pixel blend mode" -> "Pre-multiplied"). Returns 0
// when the property or the entry is absent -- every caller here treats that
// as "driver doesn't expose it, use its default" rather than an error.
uint32_t find_enum_property(int fd, uint32_t obj_id, uint32_t obj_type, const char* name,
                            const char* enum_name, uint64_t* value_out) {
  drmModeObjectPropertiesPtr props = drmModeObjectGetProperties(fd, obj_id, obj_type);
  if (!props) return 0;
  uint32_t id = 0;
  for (uint32_t i = 0; i < props->count_props && id == 0; ++i) {
    drmModePropertyPtr prop = drmModeGetProperty(fd, props->props[i]);
    if (!prop) continue;
    if ((prop->flags & DRM_MODE_PROP_ENUM) && std::strcmp(prop->name, name) == 0) {
      for (int e = 0; e < prop->count_enums; ++e) {
        // enums[].name is a fixed-size char array, not guaranteed to be
        // NUL-terminated when the name fills it exactly -- bound the compare.
        if (std::strncmp(prop->enums[e].name, enum_name, sizeof(prop->enums[e].name)) == 0) {
          id = prop->prop_id;
          *value_out = static_cast<uint64_t>(prop->enums[e].value);
          break;
        }
      }
    }
    drmModeFreeProperty(prop);
  }
  drmModeFreeObjectProperties(props);
  return id;
}

uint64_t clamp_u64(uint64_t v, uint64_t lo, uint64_t hi) {
  if (hi < lo) return lo;
  return v < lo ? lo : (v > hi ? hi : v);
}

uint64_t mono_ms() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<uint64_t>(ts.tv_sec) * 1000u + static_cast<uint64_t>(ts.tv_nsec) / 1000000u;
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

  // zpos stacking (the black-screen fix): vop2's default z-order put the
  // Smart0 backdrop ABOVE the Esmart0 video plane -- commits all succeeded
  // while the video rendered underneath a black rectangle. Set explicitly,
  // now three-level: backdrop < video < osd.
  uint32_t zpos_video_prop = 0, zpos_primary_prop = 0;
  uint64_t zpos_video_val = 0, zpos_primary_val = 0;

  // MSP OSD overlay plane. osd_plane_id == 0 means "no OSD" and every
  // osd_* path below is a no-op -- an OSD that could not be set up must
  // never cost us video.
  uint32_t osd_plane_id = 0;
  PlaneProps osd_props;
  uint32_t zpos_osd_prop = 0;
  uint64_t zpos_osd_val = 0;
  uint32_t osd_blend_prop = 0;    // "pixel blend mode", when exposed
  uint64_t osd_blend_premul = 0;  // enum value for "Pre-multiplied"
  OsdSurface osd;
  int osd_back = 0;           // index the CPU draws into
  bool osd_dirty = false;     // back buffer published, not yet committed
  bool osd_on_plane = false;  // an OSD fb has been attached at least once
  bool osd_commit_warned = false;
  int osd_fail_streak = 0;             // consecutive OSD-attributed commit failures
  uint64_t osd_commit_errors = 0;      // kept OUT of commit_errors: that one is video health
  uint64_t osd_last_commit_ms = 0;
  uint64_t last_video_commit_ms = 0;   // gates the standalone OSD commit
  // Three consecutive OSD-attributed failures and the OSD is off for good.
  // Video must never pay for an OSD the driver will not accept.
  static constexpr int kOsdMaxFailStreak = 3;

  bool inited = false;
  bool needs_modeset = true;  // next commit must be a full blocking ALLOW_MODESET commit
  // Set by the first successful modeset and never cleared: the CRTC is
  // active from then on. Distinct from !needs_modeset, which drop_all()
  // re-arms on every ordinary AU-ring discontinuity without deactivating
  // anything -- gating the standalone OSD path on that would freeze the OSD
  // exactly when on-screen link telemetry matters most.
  bool crtc_active = false;
  bool flip_pending = false;  // a NONBLOCK|PAGE_FLIP_EVENT commit is outstanding
  uint64_t flip_since_ms = 0;  // when flip_pending was set (watchdog)
  // Events the kernel still owes us for flips that were force-completed or
  // drop_all()'d before their event arrived. on_flip() swallows exactly
  // this many before processing a real completion -- without it, a stale
  // event promotes the NEXT flip early (premature dmabuf release + EBUSY
  // on the following commit).
  int events_to_swallow = 0;

  bool async_probed = false;
  bool async_active = false;

  uint64_t commit_errors = 0;
  uint64_t frames_dropped_busy = 0;
  uint64_t flips_total = 0;

  struct Slot {
    bool valid = false;
    uint32_t fb_id = 0;
    uint32_t gem_handle = 0;
    DmaFrame frame;
  };
  Slot on_screen;
  Slot pending;
  // Mailbox: newest frame that arrived while a flip was outstanding. Holds
  // the raw DmaFrame only (fb_id 0 -- no FB created until submission);
  // submitted by on_flip() the moment the outstanding flip lands.
  Slot mailbox;

  ~Impl();

  bool init(const std::string& screen_mode, ReleaseFn rel);
  bool present(const DmaFrame& frame);
  void poll_events();
  void drop_all();

  void release_slot(Slot& s);
  void on_flip(bool real_event = true);

  void add_osd_props(drmModeAtomicReqPtr req, int idx, bool one_time);
  void commit_osd_only();

  static void on_flip_static(int fd, unsigned int sequence, unsigned int tv_sec,
                              unsigned int tv_usec, unsigned int crtc_id, void* user_data);
};

void DrmPresenter::Impl::release_slot(Slot& s) {
  if (!s.valid) return;
  if (fd >= 0) {
    if (s.fb_id) drmModeRmFB(fd, s.fb_id);  // mailbox slots have no FB yet
    close_gem_handle(fd, s.gem_handle);
  }
  if (release) release(s.frame);
  s = Slot{};
}

void DrmPresenter::Impl::on_flip(bool real_event) {
  if (real_event && events_to_swallow > 0) {
    --events_to_swallow;  // stale event for an already-force-completed flip
    return;
  }
  flip_pending = false;
  if (pending.valid) {
    if (real_event) ++flips_total;  // force-completes never actually flipped
    if (on_screen.valid) release_slot(on_screen);
    on_screen = pending;
    pending = Slot{};
  }
  // Submit the mailbox frame now that the pipe has a free slot. Bounded
  // re-entrancy: present() -> poll_events() -> on_flip() -> present() --
  // the inner present() sees flip_pending == false and commits without
  // recursing further (its own poll_events guard is behind flip_pending).
  if (mailbox.valid) {
    const DmaFrame f = mailbox.frame;
    mailbox = Slot{};  // hand ownership to present(); do NOT release
    present(f);
  }
}

void DrmPresenter::Impl::on_flip_static(int /*fd*/, unsigned int /*sequence*/,
                                         unsigned int /*tv_sec*/, unsigned int /*tv_usec*/,
                                         unsigned int /*crtc_id*/, void* user_data) {
  auto* self = static_cast<Impl*>(user_data);
  if (self) self->on_flip();
}

// Full property set for the OSD plane at buffer `idx`. `one_time` adds the
// properties that only need to be (re)stated on the first attach and on any
// modeset: zpos and the blend mode.
void DrmPresenter::Impl::add_osd_props(drmModeAtomicReqPtr req, int idx, bool one_time) {
  drmModeAtomicAddProperty(req, osd_plane_id, osd_props.fb_id, osd.fb_id(idx));
  drmModeAtomicAddProperty(req, osd_plane_id, osd_props.crtc_id, crtc_id);
  drmModeAtomicAddProperty(req, osd_plane_id, osd_props.src_x, 0);
  drmModeAtomicAddProperty(req, osd_plane_id, osd_props.src_y, 0);
  drmModeAtomicAddProperty(req, osd_plane_id, osd_props.src_w,
                           static_cast<uint64_t>(osd.width()) << 16);
  drmModeAtomicAddProperty(req, osd_plane_id, osd_props.src_h,
                           static_cast<uint64_t>(osd.height()) << 16);
  drmModeAtomicAddProperty(req, osd_plane_id, osd_props.crtc_x, 0);
  drmModeAtomicAddProperty(req, osd_plane_id, osd_props.crtc_y, 0);
  drmModeAtomicAddProperty(req, osd_plane_id, osd_props.crtc_w, mode.hdisplay);
  drmModeAtomicAddProperty(req, osd_plane_id, osd_props.crtc_h, mode.vdisplay);
  if (one_time) {
    if (zpos_osd_prop) drmModeAtomicAddProperty(req, osd_plane_id, zpos_osd_prop, zpos_osd_val);
    if (osd_blend_prop)
      drmModeAtomicAddProperty(req, osd_plane_id, osd_blend_prop, osd_blend_premul);
  }
}

// Standalone OSD update, for when video is not flowing and there is no
// present() commit to ride on. Callers must have established that no flip is
// in flight and that the CRTC is already active.
//
// BLOCKING on purpose -- no DRM_MODE_ATOMIC_NONBLOCK. A nonblocking commit
// stays outstanding on the CRTC and would -EBUSY the next video commit; at a
// degraded ~5 fps every frame is more than the 100 ms of "video silence" the
// caller requires, so that path could fire between EVERY pair of frames, not
// just once at link recovery. Blocking costs at most one vsync of main-loop
// stall and only after video has already been silent -- it eliminates the
// race instead of narrowing it. Also never PAGE_FLIP_EVENT (the video path
// would mistake the event for its own flip) and never PAGE_FLIP_ASYNC.
void DrmPresenter::Impl::commit_osd_only() {
  drmModeAtomicReqPtr req = drmModeAtomicAlloc();
  int rc = -ENOMEM;
  if (req) {
    add_osd_props(req, osd_back, !osd_on_plane);
    rc = drmModeAtomicCommit(fd, req, 0, this);
    drmModeAtomicFree(req);
  }

  // Stamped on failure too: it is the retry rate limiter.
  osd_last_commit_ms = mono_ms();
  if (rc != 0) {
    // osd_commit_errors, NOT commit_errors: that counter is video health,
    // printed by the fps-log and leaned on by the hardware acceptance gate.
    // An OSD that cannot commit must not make video look broken.
    ++osd_commit_errors;
    ++osd_fail_streak;
    // Clearing osd_dirty stops the retry-forever loop: the next publish is
    // what re-arms us, so a transient failure still recovers.
    osd_dirty = false;
    if (!osd_commit_warned) {
      osd_commit_warned = true;
      const int err = rc < 0 ? -rc : errno;
      std::fprintf(stderr, "DrmPresenter: standalone OSD commit failed: %s (logged once)\n",
                   std::strerror(err));
    }
    if (osd_fail_streak >= kOsdMaxFailStreak) {
      std::fprintf(stderr,
                   "DrmPresenter: OSD plane %u rejected %d commits in a row -- disabling the OSD "
                   "for the rest of this session; video continues unaffected\n",
                   osd_plane_id, osd_fail_streak);
      osd_plane_id = 0;
    }
    return;
  }
  osd_dirty = false;
  osd_on_plane = true;
  osd_fail_streak = 0;
  osd_back ^= 1;
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
    // Interlaced modes are never acceptable for this player (progressive
    // 60 fps content on a 1080i mode = combing artifacts at half the
    // temporal rate). The connector lists 1080i@60 AFTER 1080p@60, and an
    // early version of this loop kept overwriting `selected` with later
    // matches -- shipping the interlaced mode. Skip them entirely, and
    // take the FIRST acceptable match.
    if (m->flags & DRM_MODE_FLAG_INTERLACE) continue;
    if (!preferred && (m->type & DRM_MODE_TYPE_PREFERRED)) preferred = m;
    if (!selected && have_want && static_cast<int>(m->hdisplay) == want_w &&
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

  // MSP OSD plane: the first ARGB8888-capable plane on this CRTC that is
  // neither the video plane nor the backdrop. CURSOR-type planes are
  // excluded -- they take ARGB8888 but are size-capped (64x64 on vop2) and
  // would silently truncate a full-screen OSD. Every failure below is
  // best-effort: log once, leave osd_plane_id == 0, carry on with video.
  {
    std::string osd_err = "no ARGB8888 plane free on this CRTC";
    drmModePlaneResPtr pres3 = drmModeGetPlaneResources(fd);
    if (!pres3) {
      osd_err = std::string("drmModeGetPlaneResources failed: ") + std::strerror(errno);
    } else {
      for (uint32_t i = 0; i < pres3->count_planes && !osd_plane_id; ++i) {
        drmModePlanePtr p = drmModeGetPlane(fd, pres3->planes[i]);
        if (!p) continue;
        const bool usable = (p->possible_crtcs & (1u << crtc_index)) != 0 &&
                            p->plane_id != plane_id && p->plane_id != primary_plane_id;
        bool has_argb = false;
        if (usable) {
          for (uint32_t f = 0; f < p->count_formats; ++f) {
            if (p->formats[f] == DRM_FORMAT_ARGB8888) {
              has_argb = true;
              break;
            }
          }
        }
        if (has_argb) {
          uint64_t type_val = 0;
          find_property(fd, p->plane_id, DRM_MODE_OBJECT_PLANE, "type", &type_val);
          if (type_val != DRM_PLANE_TYPE_CURSOR) osd_plane_id = p->plane_id;
        }
        drmModeFreePlane(p);
      }
      drmModeFreePlaneResources(pres3);
    }

    if (osd_plane_id && !osd_props.resolve(fd, osd_plane_id)) {
      osd_err = "OSD plane is missing a standard property";
      osd_plane_id = 0;
    }
    if (osd_plane_id && !osd.init(fd, mode.hdisplay, mode.vdisplay, &osd_err)) {
      osd_plane_id = 0;
    }
    if (osd_plane_id) {
      // Optional: ask for premultiplied blending explicitly rather than
      // trusting the driver default. Absent on drivers that don't expose
      // it -- then the default (premultiplied on vop2) applies.
      osd_blend_prop = find_enum_property(fd, osd_plane_id, DRM_MODE_OBJECT_PLANE,
                                          "pixel blend mode", "Pre-multiplied", &osd_blend_premul);
    } else {
      std::fprintf(stderr, "DrmPresenter: no ARGB OSD plane (%s) -- running without OSD\n",
                   osd_err.c_str());
    }
  }

  // Resolve zpos and pin the stacking: video at the top of its range,
  // backdrop at the bottom. Without this the kernel's default zpos left the
  // backdrop covering the video (observed on vop2: Smart0 normalized-zpos 1
  // over Esmart0's 0 -- a fully black screen with every commit succeeding).
  //
  // The video/backdrop values below are the hardware-validated ones and are
  // deliberately NOT derived from the OSD plane's range: doing that can tie
  // video to the backdrop and reinstate exactly the black screen this block
  // exists to prevent (OSD [0,1] with video/backdrop [0,7] would give
  // video 0 == backdrop 0; disjoint ranges give video max == backdrop max).
  // The OSD is layered strictly on top instead, and if it cannot sit above
  // the already-chosen video value we run without it. The OSD must never
  // perturb video's stacking.
  {
    uint64_t vmin = 0, vmax = 0;
    zpos_video_prop = find_zpos_range(fd, plane_id, &vmin, &vmax);
    if (zpos_video_prop) zpos_video_val = vmax;
    uint64_t pmin = 0, pmax = 0;
    if (primary_fb_id) {
      zpos_primary_prop = find_zpos_range(fd, primary_plane_id, &pmin, &pmax);
      if (zpos_primary_prop) zpos_primary_val = pmin;
    }

    if (!zpos_video_prop)
      std::fprintf(stderr,
                   "DrmPresenter: warning: video plane %u has no mutable zpos -- stacking is at "
                   "the driver's mercy\n",
                   plane_id);
    else
      std::fprintf(stderr, "DrmPresenter: zpos pinned: video plane %u -> %llu%s\n", plane_id,
                   static_cast<unsigned long long>(zpos_video_val),
                   zpos_primary_prop ? " (backdrop -> min)" : "");

    if (osd_plane_id) {
      uint64_t omin = 0, omax = 0;
      zpos_osd_prop = find_zpos_range(fd, osd_plane_id, &omin, &omax);
      zpos_osd_val = zpos_osd_prop ? clamp_u64(zpos_video_val + 1, omin, omax) : 0;
      // Verify the whole order really came out backdrop < video < osd. The
      // backdrop term is skipped when there is no backdrop plane (video is
      // itself the primary), and a video plane with no mutable zpos gives us
      // nothing to layer above, so both of those disqualify the OSD too.
      const bool backdrop_ok = !zpos_primary_prop || zpos_primary_val < zpos_video_val;
      const bool osd_ok = zpos_osd_prop && zpos_video_prop && zpos_video_val < zpos_osd_val;
      if (!backdrop_ok || !osd_ok) {
        std::fprintf(stderr,
                     "DrmPresenter: OSD plane %u cannot be stacked above video (zpos backdrop "
                     "%llu / video %llu / osd %llu, osd range [%llu,%llu]) -- running WITHOUT the "
                     "OSD rather than disturbing the video stacking\n",
                     osd_plane_id, static_cast<unsigned long long>(zpos_primary_val),
                     static_cast<unsigned long long>(zpos_video_val),
                     static_cast<unsigned long long>(zpos_osd_val),
                     static_cast<unsigned long long>(omin), static_cast<unsigned long long>(omax));
        osd_plane_id = 0;
        zpos_osd_prop = 0;
        zpos_osd_val = 0;
        osd_blend_prop = 0;
        osd.destroy();  // nothing has been committed yet; reclaim the two buffers
      }
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
  if (osd_plane_id)
    std::fprintf(stderr,
                 "DrmPresenter: OSD plane %u ARGB8888 %ux%u, zpos %llu, blend prop %s\n",
                 osd_plane_id, mode.hdisplay, mode.vdisplay,
                 static_cast<unsigned long long>(zpos_osd_val),
                 osd_blend_prop ? "premultiplied" : "driver default");

  inited = true;
  return true;
}

bool DrmPresenter::Impl::present(const DmaFrame& frame) {
  if (!inited || fd < 0) {
    if (release) release(frame);
    return false;
  }

  if (flip_pending) {
    // The event may have already landed without the main loop reaping it
    // yet -- drain opportunistically before deciding to drop.
    poll_events();
  }
  if (flip_pending) {
    // Watchdog (the pipeline-stall fix): a lost/unreaped flip event would
    // otherwise leave flip_pending latched forever -- every frame drops,
    // the two held DmaFrames never return to MPP, its internal buffer
    // group exhausts, and decode freezes (observed live: frames counter
    // hard-stalled while the ring kept flowing). 200 ms = 12 vsyncs, far
    // beyond any legitimate flip latency: force-complete and carry on.
    const uint64_t now = mono_ms();
    if (flip_since_ms && now - flip_since_ms > 200) {
      std::fprintf(stderr,
                   "DrmPresenter: flip event lost (>200 ms) -- force-completing (fps may hitch)\n");
      // The kernel will still deliver this flip's event eventually --
      // account for it so on_flip() swallows it instead of promoting the
      // NEXT flip early. Do NOT zero flip_since_ms here: on_flip()'s
      // mailbox submission re-stamps it via present(), and zeroing it
      // afterwards disarmed the watchdog for every subsequent lost event
      // (review finding: second lost event = permanent freeze).
      ++events_to_swallow;
      on_flip(/*real_event=*/false);
    }
  }
  if (flip_pending) {
    // Mailbox backpressure: a NONBLOCK commit is still outstanding
    // (re-committing would EBUSY). SVC-T delivery is bursty -- base and
    // enhance frames of adjacent capture times decode back-to-back at the
    // VENC's alternating cadence -- so dropping the NEW frame here halved
    // the displayed rate (visible judder). Instead park the newest frame
    // in `mailbox`; on_flip() submits it the instant the outstanding flip
    // lands, and a newer arrival meanwhile replaces (releases) the parked
    // one. Displayed rate stays at vsync, always with the freshest frame.
    if (mailbox.valid) {
      ++frames_dropped_busy;  // the replaced frame is the true "drop"
      release_slot(mailbox);
    }
    Slot m;
    m.valid = true;
    m.fb_id = 0;  // FB not created yet -- deferred until submit
    m.gem_handle = 0;
    m.frame = frame;
    mailbox = m;
    return true;
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

  const bool do_modeset = needs_modeset;

  // Ride the OSD along on this commit when it has something new to show,
  // and re-state it on any modeset (a modeset re-runs the whole CRTC state;
  // don't assume the plane keeps its FB across one).
  const bool osd_ready = osd_plane_id != 0 && osd.ok();
  bool attach_osd = osd_ready && (osd_dirty || (do_modeset && osd_on_plane));
  // ...but never on the commit that would otherwise be the one-shot async
  // probe. An OSD publishing at or above the video frame rate would keep
  // `attach_osd` true forever, `probing_async` never true, and video would
  // run vsync-paced for the whole session -- visible only as "async=probing"
  // in the fps-log. Don't depend on the caller's publish cadence: skip the
  // attach on the first non-modeset commit. The OSD stays dirty and rides
  // the next one, at most ~16 ms later.
  if (!async_probed && !do_modeset) attach_osd = false;
  const int osd_idx = osd_dirty ? osd_back : (osd_back ^ 1);

  // Factored so that a commit the kernel rejects WITH the OSD attached can
  // be rebuilt without it and retried immediately (a drmModeAtomicReq has no
  // "remove property"). See the failure handling after the commit.
  auto build_req = [&](bool with_osd) -> drmModeAtomicReqPtr {
    drmModeAtomicReqPtr r = drmModeAtomicAlloc();
    if (!r) return nullptr;
    drmModeAtomicAddProperty(r, plane_id, video_props.fb_id, fb_id);
    drmModeAtomicAddProperty(r, plane_id, video_props.crtc_id, crtc_id);
    drmModeAtomicAddProperty(r, plane_id, video_props.src_x, 0);
    drmModeAtomicAddProperty(r, plane_id, video_props.src_y, 0);
    drmModeAtomicAddProperty(r, plane_id, video_props.src_w,
                             static_cast<uint64_t>(frame.width) << 16);
    drmModeAtomicAddProperty(r, plane_id, video_props.src_h,
                             static_cast<uint64_t>(frame.height) << 16);
    drmModeAtomicAddProperty(r, plane_id, video_props.crtc_x, 0);
    drmModeAtomicAddProperty(r, plane_id, video_props.crtc_y, 0);
    drmModeAtomicAddProperty(r, plane_id, video_props.crtc_w, mode.hdisplay);
    drmModeAtomicAddProperty(r, plane_id, video_props.crtc_h, mode.vdisplay);

    if (do_modeset) {
      if (zpos_video_prop) drmModeAtomicAddProperty(r, plane_id, zpos_video_prop, zpos_video_val);
      drmModeAtomicAddProperty(r, connector_id, prop_connector_crtc_id, crtc_id);
      drmModeAtomicAddProperty(r, crtc_id, prop_crtc_mode_id, mode_blob_id);
      drmModeAtomicAddProperty(r, crtc_id, prop_crtc_active, 1);
      if (primary_fb_id) {
        if (zpos_primary_prop)
          drmModeAtomicAddProperty(r, primary_plane_id, zpos_primary_prop, zpos_primary_val);
        drmModeAtomicAddProperty(r, primary_plane_id, primary_props.fb_id, primary_fb_id);
        drmModeAtomicAddProperty(r, primary_plane_id, primary_props.crtc_id, crtc_id);
        drmModeAtomicAddProperty(r, primary_plane_id, primary_props.src_x, 0);
        drmModeAtomicAddProperty(r, primary_plane_id, primary_props.src_y, 0);
        drmModeAtomicAddProperty(r, primary_plane_id, primary_props.src_w,
                                 static_cast<uint64_t>(mode.hdisplay) << 16);
        drmModeAtomicAddProperty(r, primary_plane_id, primary_props.src_h,
                                 static_cast<uint64_t>(mode.vdisplay) << 16);
        drmModeAtomicAddProperty(r, primary_plane_id, primary_props.crtc_x, 0);
        drmModeAtomicAddProperty(r, primary_plane_id, primary_props.crtc_y, 0);
        drmModeAtomicAddProperty(r, primary_plane_id, primary_props.crtc_w, mode.hdisplay);
        drmModeAtomicAddProperty(r, primary_plane_id, primary_props.crtc_h, mode.vdisplay);
      }
    }

    if (with_osd) add_osd_props(r, osd_idx, !osd_on_plane || do_modeset);
    return r;
  };

  drmModeAtomicReqPtr req = build_req(attach_osd);
  if (!req) {
    ++commit_errors;
    drmModeRmFB(fd, fb_id);
    close_gem_handle(fd, handle);
    if (release) release(frame);
    return false;
  }

  uint32_t flags = do_modeset
                       ? static_cast<uint32_t>(DRM_MODE_ATOMIC_ALLOW_MODESET)
                       : static_cast<uint32_t>(DRM_MODE_ATOMIC_NONBLOCK | DRM_MODE_PAGE_FLIP_EVENT);
  // An async page flip may only change FB_ID on a SINGLE plane -- the kernel
  // rejects an async commit that also touches the OSD plane. So any commit
  // carrying the OSD is vsync-paced, AND the one-shot async probe must not
  // run on such a commit: an EINVAL there would latch "async rejected" for
  // the wrong reason and permanently demote video to vsync-paced flips.
  const bool probing_async = !do_modeset && !async_probed && !attach_osd;
  if (!do_modeset && !attach_osd && (probing_async || async_active))
    flags |= DRM_MODE_PAGE_FLIP_ASYNC;

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

  // The commit failed with the OSD attached. Rebuild WITHOUT the OSD and
  // retry right now, so a driver that will not take the two-plane request
  // costs video nothing -- not even this frame. `osd_dirty` is cleared
  // whatever happens: leaving it set would rebuild the identical rejected
  // shape on the very next frame, and every one after it, dropping 60
  // frames a second forever with nothing to self-heal it (needs_modeset is
  // untouched, drop_all() is never called, and the decode watchdog cannot
  // fire because frames keep arriving). After kOsdMaxFailStreak consecutive
  // OSD-attributed failures the OSD is switched off for the session.
  if (rc != 0 && attach_osd) {
    const int oerr = rc < 0 ? -rc : errno;
    osd_dirty = false;
    ++osd_commit_errors;
    ++osd_fail_streak;
    std::fprintf(stderr,
                 "DrmPresenter: commit with the OSD plane attached failed: %s -- retrying "
                 "video-only (OSD failure %d/%d)\n",
                 std::strerror(oerr), osd_fail_streak, kOsdMaxFailStreak);
    if (osd_fail_streak >= kOsdMaxFailStreak) {
      std::fprintf(stderr,
                   "DrmPresenter: OSD plane %u rejected %d commits in a row -- disabling the OSD "
                   "for the rest of this session; video continues unaffected\n",
                   osd_plane_id, osd_fail_streak);
      osd_plane_id = 0;  // every osd_* entry point becomes a no-op from here
    }
    attach_osd = false;
    drmModeAtomicFree(req);
    req = build_req(false);
    // Same `flags` deliberately: they carry no PAGE_FLIP_ASYNC (attach_osd
    // suppressed it) and probing_async was false, so this retry can neither
    // run nor poison the async probe. One vsync-paced frame is the whole cost.
    rc = req ? drmModeAtomicCommit(fd, req, flags, this) : -ENOMEM;
  }

  if (req) drmModeAtomicFree(req);

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

  last_video_commit_ms = mono_ms();
  if (attach_osd) {
    if (osd_dirty) {
      osd_dirty = false;
      osd_back ^= 1;  // what we just committed becomes the front buffer
    }
    osd_on_plane = true;
    osd_fail_streak = 0;
    osd_last_commit_ms = last_video_commit_ms;
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
    crtc_active = true;  // latched: the CRTC stays active for the rest of the session
  } else {
    pending = new_slot;
    flip_pending = true;
    flip_since_ms = mono_ms();
  }
  return true;
}

void DrmPresenter::Impl::poll_events() {
  if (fd < 0) return;
  struct pollfd pfd {
    fd, POLLIN, 0
  };
  if (poll(&pfd, 1, 0) > 0 && (pfd.revents & POLLIN)) {
    drmEventContext evctx{};
    evctx.version = 3;  // page_flip_handler2 requires version >= 3
    evctx.page_flip_handler2 = &Impl::on_flip_static;
    drmHandleEvent(fd, &evctx);
  }

  // No video commit to ride on: show the OSD on its own. Gated on no flip in
  // flight AND on video having been quiet for 100 ms -- present() calls
  // poll_events() while a flip is outstanding, so without the quiet window a
  // standalone commit could be issued from inside present() itself, right in
  // front of the video commit it would then block.
  //
  // `crtc_active`, NOT `!needs_modeset`: drop_all() re-arms needs_modeset on
  // every ordinary AU-ring discontinuity (main.cpp's flush_before) and from
  // the decode watchdog, without deactivating the CRTC. Gating on it would
  // freeze the OSD exactly when on-screen link telemetry matters most.
  // crtc_active still excludes the pre-first-frame case, where the CRTC is
  // not up yet and an OSD-only commit could not display anything.
  const uint64_t now = mono_ms();
  if (osd_dirty && osd_plane_id && osd.ok() && !flip_pending && crtc_active &&
      now - last_video_commit_ms >= 100 && now - osd_last_commit_ms >= 100) {
    commit_osd_only();
  }
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
  if (mailbox.valid) release_slot(mailbox);
  if (pending.valid) release_slot(pending);
  if (on_screen.valid) release_slot(on_screen);
  if (flip_pending) ++events_to_swallow;  // the kernel still owes this event
  flip_pending = false;
  needs_modeset = true;  // plane content is gone; next present() must redo the full commit
}

DrmPresenter::Impl::~Impl() {
  // Explicit, and FIRST: `osd` is a member, so ~OsdSurface would otherwise
  // run AFTER this body -- i.e. after close(fd) below -- and issue
  // drmModeRmFB/DestroyDumbBuffer on a closed descriptor (its own fd_ is a
  // stale copy, so its guard cannot catch that). Every other resource here
  // is already released before the close; this keeps the OSD in line.
  osd.destroy();
  if (mailbox.valid) release_slot(mailbox);
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

uint64_t DrmPresenter::flips() const { return impl_->flips_total; }

uint64_t DrmPresenter::busy_replaced() const { return impl_->frames_dropped_busy; }

bool DrmPresenter::async_flip_active() const { return impl_->async_active; }

bool DrmPresenter::async_probed() const { return impl_->async_probed; }

bool DrmPresenter::osd_available() const {
  return impl_->osd_plane_id != 0 && impl_->osd.ok();
}

Surface DrmPresenter::osd_back_surface() {
  if (!osd_available()) return Surface{};
  return impl_->osd.cpu(impl_->osd_back);
}

int DrmPresenter::osd_back_index() const { return impl_->osd_back; }

void DrmPresenter::osd_publish() {
  if (osd_available()) impl_->osd_dirty = true;
}

uint64_t DrmPresenter::osd_commit_errors() const { return impl_->osd_commit_errors; }

int DrmPresenter::osd_front_prime_fd() const {
  return osd_available() ? impl_->osd.prime_fd(impl_->osd_back ^ 1) : -1;
}

}  // namespace maburplay
