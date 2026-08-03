#include "mtest.h"
#include "osd_layout.h"
#include <cstdio>
using namespace maburplay;

// plan_zpos() is the DRM plane stacking policy lifted out of DrmPresenter so
// it can be checked without a GPU. Table-driven: each row is a plausible (or
// observed) plane-range combination and the stacking it must produce.
struct Case {
  const char* name;
  bool have_osd, have_video_prop, have_backdrop;
  uint64_t vmin, vmax, pmin, omax;
  bool want_ok;
  uint64_t want_backdrop, want_video, want_osd;  // osd checked only if want_ok
};

static const Case kCases[] = {
    // THE BENCH HARDWARE (rk3566 vop2, read off the live GS with modetest):
    // Esmart0 video, Smart0 backdrop and Cluster0-win0 OSD all advertise the
    // IDENTICAL range [0,7]. The scheme this replaced computed
    // osd = clamp(video_max + 1, 0, 7) = 7 == video, failed its own
    // ordering assertion and disabled the OSD on every single run.
    {"all_planes_share_0_7", true, true, true, 0, 7, 0, 7, true, 0, 6, 7},

    // Disjoint ranges: the hazard the earlier review was defending against.
    // Video clamps into its OWN range (3, not 6), so no tie with the
    // backdrop is produced and the order still comes out strict.
    {"disjoint_osd_above_video", true, true, true, 0, 3, 0, 7, true, 0, 3, 7},

    // OSD range strictly BELOW the video's: unsatisfiable, run video-only.
    // Video must revert to the top of its own range (7), not to omax-1.
    {"osd_range_below_video", true, true, true, 4, 7, 0, 3, false, 0, 7, 0},

    // Degenerate single-value OSD range at the bottom: video would have to
    // be below 0. Video-only.
    {"degenerate_osd_range_0_0", true, true, true, 0, 7, 0, 0, false, 0, 7, 0},

    // Degenerate single-value OSD range with room underneath: usable.
    {"degenerate_osd_range_3_3", true, true, true, 0, 7, 0, 3, true, 0, 2, 3},

    // The old reviewer's stated hazard, exactly: a narrow OSD range would
    // push the video down onto the backdrop. Ties are rejected -> video-only
    // at the hardware-validated values.
    {"narrow_osd_range_would_tie_video_to_backdrop", true, true, true, 0, 7, 0, 1, false, 0, 7, 0},

    // No OSD plane at all: the pre-OSD, hardware-validated fallback.
    // Video = vmax, backdrop = pmin, untouched.
    {"no_osd_plane_keeps_video_at_max", false, true, true, 0, 7, 0, 0, false, 0, 7, 0},

    // No backdrop plane (the video plane IS the primary): nothing to be
    // tied to, so the OSD only has to clear the video.
    {"no_backdrop_plane", true, true, false, 0, 7, 0, 7, true, 0, 6, 7},

    // Video plane has no mutable zpos: nothing to layer above provably, so
    // the OSD is disqualified even though its own range is fine.
    {"no_mutable_video_zpos", true, false, true, 0, 7, 0, 7, false, 0, 0, 0},
};

TEST(plan_zpos_table) {
  for (const Case& c : kCases) {
    const ZposPlan p =
        plan_zpos(c.have_osd, c.have_video_prop, c.have_backdrop, c.vmin, c.vmax, c.pmin, c.omax);
    if (p.osd_ok != c.want_ok || p.backdrop != c.want_backdrop || p.video != c.want_video ||
        (c.want_ok && p.osd != c.want_osd))
      std::fprintf(stderr, "case %s: got backdrop=%llu video=%llu osd=%llu ok=%d\n", c.name,
                   (unsigned long long)p.backdrop, (unsigned long long)p.video,
                   (unsigned long long)p.osd, (int)p.osd_ok);
    CHECK(p.osd_ok == c.want_ok);
    CHECK(p.backdrop == c.want_backdrop);
    CHECK(p.video == c.want_video);
    if (c.want_ok) CHECK(p.osd == c.want_osd);
  }
}

// The property every row must satisfy independently of its expected values:
// when the OSD is accepted the three layers are strictly ordered, and when
// it is refused the video/backdrop pair is exactly the no-OSD assignment.
TEST(plan_zpos_invariants) {
  for (const Case& c : kCases) {
    const ZposPlan p =
        plan_zpos(c.have_osd, c.have_video_prop, c.have_backdrop, c.vmin, c.vmax, c.pmin, c.omax);
    if (p.osd_ok) {
      CHECK(p.video < p.osd);
      CHECK(p.osd <= c.omax);
      CHECK(p.video >= c.vmin && p.video <= c.vmax);
      if (c.have_backdrop) CHECK(p.backdrop < p.video);
    } else {
      const ZposPlan none = plan_zpos(false, c.have_video_prop, c.have_backdrop, c.vmin, c.vmax,
                                      c.pmin, c.omax);
      CHECK(p.video == none.video);
      CHECK(p.backdrop == none.backdrop);
    }
  }
}

MTEST_MAIN
