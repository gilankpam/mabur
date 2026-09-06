#include "mtest.h"
#include "debug_http.h"
TEST(debug_http_routes) {
  CHECK(debug_http_parse("GET /snapshot.jpg HTTP/1.0").kind == DebugReq::SNAPSHOT);
  CHECK(debug_http_parse("GET /venc HTTP/1.0").kind == DebugReq::STATS);
  auto r = debug_http_parse("POST /venc/set?bitrate=4000 HTTP/1.0");
  CHECK(r.kind == DebugReq::SET); CHECK(r.key == "bitrate"); CHECK(r.val == 4000);
  CHECK(debug_http_parse("POST /venc/set?gop=1 HTTP/1.0").kind == DebugReq::BAD);
  CHECK(debug_http_parse("GET /api/v1/set?video0.bitrate=1 HTTP/1.0").kind == DebugReq::BAD);
  CHECK(debug_http_parse("POST /venc/set?bitrate=nope HTTP/1.0").kind == DebugReq::BAD);
}
TEST(superframe_p_pct_key_whitelisted) {
  auto p = debug_http_parse("POST /venc/set?superframe_p_pct=200 HTTP/1.0");
  CHECK(p.kind == DebugReq::SET);
  CHECK(p.key == "superframe_p_pct");
  CHECK(p.val == 200);
}
TEST(min_qp_key_rejected) {
  // Deleted 2026-09-03 with the venc.min_qp config key (bench refuted the
  // QP-floor hypothesis); a stale bench script must get BAD, not a silent ok.
  CHECK(debug_http_parse("POST /venc/set?min_qp=24 HTTP/1.0").kind == DebugReq::BAD);
}
TEST(min_iqp_key_whitelisted) {
  auto p = debug_http_parse("POST /venc/set?min_iqp=40 HTTP/1.0");
  CHECK(p.kind == DebugReq::SET);
  CHECK(p.key == "min_iqp");
  CHECK(p.val == 40);
}

TEST(max_iqp_key_whitelisted) {
  auto p = debug_http_parse("POST /venc/set?max_iqp=51 HTTP/1.0");
  CHECK(p.kind == DebugReq::SET);
  CHECK(p.key == "max_iqp");
  CHECK(p.val == 51);
}

TEST(max_ipprop_key_whitelisted) {
  auto p = debug_http_parse("POST /venc/set?max_ipprop=2 HTTP/1.0");
  CHECK(p.kind == DebugReq::SET);
  CHECK(p.key == "max_ipprop");
  CHECK(p.val == 2);
}
MTEST_MAIN
