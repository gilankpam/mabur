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
TEST(min_qp_key_whitelisted) {
  // Bench knob for the venc-overshoot sweep: volatile u32MinQp, same
  // apply pattern as max_ipprop.
  auto p = debug_http_parse("POST /venc/set?min_qp=24 HTTP/1.0");
  CHECK(p.kind == DebugReq::SET);
  CHECK(p.key == "min_qp");
  CHECK(p.val == 24);
}
TEST(max_ipprop_key_whitelisted) {
  auto p = debug_http_parse("POST /venc/set?max_ipprop=2 HTTP/1.0");
  CHECK(p.kind == DebugReq::SET);
  CHECK(p.key == "max_ipprop");
  CHECK(p.val == 2);
}
MTEST_MAIN
