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
MTEST_MAIN
