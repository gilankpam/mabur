#include "mtest.h"
#include "probe_source.h"
#include "mabur/probe_wire.h"
#include "mabur/sbi.h"
#include "mabur/sw_wire.h"
using namespace mabur;

TEST(probe_source_builds_sequential_bodies) {
  ProbeSource src(4, static_cast<int>(sw::kSwHeaderLen) + 332, 1000);
  UepBody a = src.build(0x06, 17);
  UepBody b = src.build(0x06, 18);
  CHECK(a.stream_id == kProbeStreamId);
  CHECK(a.body.size() == 1403);
  CHECK(!a.au_first);
  probe::ProbeRx ra, rb;
  REQUIRE(probe::parse_probe_body(a.body.data(), a.body.size(), static_cast<int>(sw::kSwHeaderLen) + 332, &ra));
  REQUIRE(probe::parse_probe_body(b.body.data(), b.body.size(), static_cast<int>(sw::kSwHeaderLen) + 332, &rb));
  CHECK(ra.hdr.seq == 1000); CHECK(rb.hdr.seq == 1001);
  CHECK(ra.hdr.enh_fid == 17); CHECK(rb.hdr.enh_fid == 18);
  CHECK(ra.hdr.profile == 0x06);
  CHECK(src.built() == 2);
}

MTEST_MAIN
