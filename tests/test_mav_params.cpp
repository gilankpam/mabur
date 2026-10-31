#include <cmath>
#include <cstdlib>
#include <cstring>
#include <sstream>
#include <string>
#include <vector>

#include "mtest.h"
#include "mav_params.h"
#include "config.h"
#include "mabur/config_write.h"
#include "scratch.h"

using namespace mabur;
using namespace mabur::mavparam;

namespace {

// Path to the committed default bundle config, resolved the same way
// tests/test_config_write.cpp's bundle_text() does.
std::string bundle_path() {
  return std::string(MABUR_BUNDLE_DIR) + "/mabur.default.toml";
}

// read() now returns bool (false for idx out of range); tests want the
// Value itself and REQUIRE success is unremarkable noise at every call
// site below, so wrap it once.
Value rd(const Config& c, size_t idx) {
  Value v;
  REQUIRE(read(c, idx, &v));
  return v;
}

std::vector<std::string> lines_of(const std::string& t) {
  std::vector<std::string> out;
  std::istringstream in(t);
  for (std::string l; std::getline(in, l);) out.push_back(l);
  return out;
}

// Line-by-line diff count between two texts of the same length; patch_row
// never changes the line count (every patch rewrites a value in place).
int changed_lines(const std::string& a, const std::string& b) {
  const auto la = lines_of(a), lb = lines_of(b);
  REQUIRE(la.size() == lb.size());
  int n = 0;
  for (size_t i = 0; i < la.size(); ++i) n += la[i] != lb[i];
  return n;
}

}  // namespace

TEST(reads_every_row_of_the_default_bundle) {
  Config c = load_config(bundle_path());
  CHECK(rd(c, 0).f == 1.0f);            // radio.wall_margin_db
  CHECK(rd(c, 1).b == true);            // radio.ldpc
  CHECK(rd(c, 2).s == "60@1920x1080");  // venc.fps + venc.size
  CHECK(rd(c, 3).i == 4);               // venc.slices
  CHECK(rd(c, 4).i == 24000);           // encoder.bitrate_max_kbps
  CHECK(std::fabs(rd(c, 5).f - 0.65f) < 1e-6f);
  CHECK(rd(c, 6).b == true);            // record.enable
  CHECK(rd(c, 7).i == 40000 && rd(c, 8).i == 60 && rd(c, 9).s == "1920x1080");
  CHECK(rd(c, 10).b == false && rd(c, 11).b == true);
}

TEST(rec_size_falls_back_to_venc_mode_size_when_record_size_is_zero) {
  Config c = load_config(bundle_path());
  c.record.width = 0;
  c.record.height = 0;
  const std::string mode = rd(c, 2).s;
  CHECK(rd(c, 9).s == mode.substr(mode.find('@') + 1));
}

TEST(read_rejects_idx_out_of_range) {
  Config c = load_config(bundle_path());
  Value v;
  CHECK(!read(c, 12, &v));                    // cfgparam::kCount == 12
  CHECK(!read(c, static_cast<size_t>(-1), &v));
}

TEST(render_toml_forms) {
  std::string o;
  CHECK(render_toml(Value{Type::Bool, true}, &o) && o == "true");
  CHECK(render_toml(Value{Type::Int32, false, -5}, &o) && o == "-5");
  CHECK(render_toml(Value{Type::Real32, false, 0, 2.0f}, &o) && o == "2.0");
  CHECK(render_toml(Value{Type::Real32, false, 0, 0.65f}, &o) && o == "0.65");
  CHECK(render_toml(Value{Type::Real32, false, 0, 1.0f}, &o) && o == "1.0");
  CHECK(render_toml(Value{Type::Real32, false, 0, 1e-7f}, &o) && o == "1e-07");
  // A float that needs all nine digits still round-trips.
  const float awkward = 0.1f + 0.2f;
  CHECK(render_toml(Value{Type::Real32, false, 0, awkward}, &o) &&
        std::strtof(o.c_str(), nullptr) == awkward);
  CHECK(render_toml(Value{Type::Str, false, 0, 0, "1280x720"}, &o) && o == "\"1280x720\"");
  CHECK(!render_toml(Value{Type::Str, false, 0, 0, "a\"b"}, &o));
  CHECK(!render_toml(Value{Type::Str, false, 0, 0, "a\nb"}, &o));
  CHECK(!render_toml(Value{Type::Real32, false, 0, INFINITY}, &o));
  CHECK(!render_toml(Value{Type::Real32, false, 0, -INFINITY}, &o));
  CHECK(!render_toml(Value{Type::Real32, false, 0, NAN}, &o));
}

TEST(decode_wire_rejects_type_mismatch_and_bad_bytes) {
  char w[128] = {};
  Value v;
  w[0] = 1; CHECK(decode_wire(1, 1, w, &v) && v.b);           // RADIO_LDPC UINT8 1
  w[0] = 2; CHECK(!decode_wire(1, 1, w, &v));                  // not 0/1
  CHECK(!decode_wire(1, 6, w, &v));                            // INT32 sent for a Bool row
  std::memset(w, 'x', 128); CHECK(!decode_wire(9, 11, w, &v)); // no NUL (REC_SIZE)
  std::memcpy(w, "1280x720", 9); CHECK(decode_wire(9, 11, w, &v) && v.s == "1280x720");
  std::memcpy(w, "12\x01", 4); CHECK(!decode_wire(9, 11, w, &v));
  int32_t x = 90; std::memcpy(w, &x, 4); CHECK(decode_wire(3, 6, w, &v) && v.i == 90);  // VENC_SLICES
}

TEST(decode_wire_venc_mode_checks_the_shape) {
  char w[128]; Value v;
  std::memset(w, 0, 128); std::strcpy(w, "100@1728x972");
  CHECK(decode_wire(2, 11, w, &v) && v.s == "100@1728x972");
  std::strcpy(w, "100@1728");  CHECK(!decode_wire(2, 11, w, &v));
  std::strcpy(w, "1000@1x1");  CHECK(decode_wire(2, 11, w, &v));
}

TEST(decode_wire_rejects_non_finite_real32) {
  char w[128] = {};
  Value v;
  float nan_v = NAN;
  std::memcpy(w, &nan_v, 4);
  CHECK(!decode_wire(6, 9, w, &v));  // ENC_AIRTIME REAL32, NaN
  float inf_v = INFINITY;
  std::memcpy(w, &inf_v, 4);
  CHECK(!decode_wire(6, 9, w, &v));  // ENC_AIRTIME REAL32, +Inf
  float ninf_v = -INFINITY;
  std::memcpy(w, &ninf_v, 4);
  CHECK(!decode_wire(6, 9, w, &v));  // ENC_AIRTIME REAL32, -Inf
}

TEST(encode_wire_round_trips) {
  // Bool: row 1 (RADIO_LDPC), MAV_PARAM_EXT_TYPE_UINT8 == 1.
  {
    char w[128];
    Value v{Type::Bool, true};
    encode_wire(v, w);
    CHECK(static_cast<uint8_t>(w[0]) == 1);
    for (int k = 1; k < 128; ++k) CHECK(w[k] == 0);
    Value out;
    CHECK(decode_wire(1, 1, w, &out) && out.b == true);
  }
  // Int32: row 3 (VENC_SLICES), MAV_PARAM_EXT_TYPE_INT32 == 6.
  {
    char w[128];
    Value v{Type::Int32, false, -1234};
    encode_wire(v, w);
    int32_t raw;
    std::memcpy(&raw, w, 4);
    CHECK(raw == -1234);
    for (int k = 4; k < 128; ++k) CHECK(w[k] == 0);
    Value out;
    CHECK(decode_wire(3, 6, w, &out) && out.i == -1234);
  }
  // Real32: row 0 (RADIO_WALL_MARGN), MAV_PARAM_EXT_TYPE_REAL32 == 9.
  {
    char w[128];
    Value v{Type::Real32, false, 0, 1.5f};
    encode_wire(v, w);
    float raw;
    std::memcpy(&raw, w, 4);
    CHECK(raw == 1.5f);
    for (int k = 4; k < 128; ++k) CHECK(w[k] == 0);
    Value out;
    CHECK(decode_wire(0, 9, w, &out) && out.f == 1.5f);
  }
  // Str: row 9 (REC_SIZE), MAV_PARAM_EXT_TYPE_CUSTOM == 11.
  {
    char w[128];
    Value v{Type::Str, false, 0, 0, "1920x1080"};
    encode_wire(v, w);
    CHECK(std::memcmp(w, "1920x1080", 9) == 0);
    CHECK(w[9] == '\0');
    for (int k = 10; k < 128; ++k) CHECK(w[k] == 0);
    Value out;
    CHECK(decode_wire(9, 11, w, &out) && out.s == "1920x1080");
  }
}

// The bundle's own VENC_MODE (60@1920x1080) and record (enable, explicit
// 1920x1080) feed every patch_row case below; `cur` is the loaded Config
// patch_row reads record.enable/width/height from -- never the text being
// patched, which may have been edited out from under it.

TEST(patch_row_venc_mode_rewrites_both_venc_lines) {
  std::string text;
  REQUIRE(read_text_file(bundle_path(), &text));
  const Config cur = load_config(bundle_path());
  std::string patched = text, followed, err;
  // Same aspect as the bundle's record.size (1920x1080): only venc.fps
  // actually changes text; venc.size is still patched, to the same value.
  REQUIRE(patch_row(&patched, cfgparam::kVencModeRow, Value{Type::Str, false, 0, 0, "90@1920x1080"},
                    cur, &followed, &err));
  CHECK(followed.empty());
  CHECK(changed_lines(text, patched) == 1);
  std::string want = text;
  REQUIRE(patch_toml_key(&want, "venc", "fps", "90"));
  CHECK(patched == want);
}

TEST(patch_row_venc_mode_carries_record_size_across_aspect) {
  std::string text;
  REQUIRE(read_text_file(bundle_path(), &text));
  const Config cur = load_config(bundle_path());
  std::string patched = text, followed, err;
  REQUIRE(patch_row(&patched, cfgparam::kVencModeRow,
                    Value{Type::Str, false, 0, 0, "120@1728x816"}, cur, &followed, &err));
  CHECK(followed == "record.size");
  CHECK(changed_lines(text, patched) == 3);  // venc.fps, venc.size, record.size
  ScratchFile f{"test_mav_params", ".toml"};
  REQUIRE(write_file_fsync(f.path, patched));
  const Config cand = load_config(f.path);
  CHECK(cand.venc.core.fps == 120 && cand.venc.core.width == 1728 && cand.venc.core.height == 816);
  CHECK(cand.record.width == 1728 && cand.record.height == 816);
}

TEST(patch_row_leaves_record_alone_when_recorder_off) {
  std::string text;
  REQUIRE(read_text_file(bundle_path(), &text));
  Config cur = load_config(bundle_path());
  cur.record.enable = false;  // the Config, not the text
  std::string patched = text, followed, err;
  REQUIRE(patch_row(&patched, cfgparam::kVencModeRow,
                    Value{Type::Str, false, 0, 0, "120@1728x816"}, cur, &followed, &err));
  CHECK(followed.empty());
  CHECK(changed_lines(text, patched) == 2);  // venc.fps, venc.size only
}

TEST(patch_row_leaves_absent_record_size_alone) {
  std::string text;
  REQUIRE(read_text_file(bundle_path(), &text));
  const std::string rec_size_line =
      "size         = \"1920x1080\"        # recording resolution, same aspect ratio as "
      "venc.size. It may differ\n";
  const size_t at = text.find(rec_size_line);
  REQUIRE(at != std::string::npos);
  text.erase(at, rec_size_line.size());
  REQUIRE(text.find("size         = \"1920x1080\"") == std::string::npos);

  Config cur = load_config(bundle_path());
  cur.record.width = 0;   // 0x0: "same as venc.size", the Config this absent line loads as
  cur.record.height = 0;
  std::string patched = text, followed, err;
  REQUIRE(patch_row(&patched, cfgparam::kVencModeRow,
                    Value{Type::Str, false, 0, 0, "120@1728x816"}, cur, &followed, &err));
  CHECK(followed.empty());
  // No `size` line was added back under [record]: a `width > 0` guard that
  // regressed to "always follow" would try to patch a line that isn't
  // there and patch_row would have failed above instead.
  const size_t rec = patched.find("[record]");
  REQUIRE(rec != std::string::npos);
  const size_t rec_end = patched.find("\n[", rec);
  const std::string rec_section =
      rec_end == std::string::npos ? patched.substr(rec) : patched.substr(rec, rec_end - rec);
  CHECK(rec_section.find("1728x816") == std::string::npos);
}

TEST(patch_row_reports_missing_key) {
  std::string text;
  REQUIRE(read_text_file(bundle_path(), &text));
  const std::string venc_size_line = "size       = \"1920x1080\"\n";
  const size_t at = text.find(venc_size_line);
  REQUIRE(at != std::string::npos);
  text.insert(at, "# ");
  const Config cur = load_config(bundle_path());  // the unedited file
  const std::string before = text;
  std::string patched = text, followed, err;
  CHECK(!patch_row(&patched, cfgparam::kVencModeRow,
                   Value{Type::Str, false, 0, 0, "90@1280x720"}, cur, &followed, &err));
  CHECK(err == "venc.size: key not found");
  CHECK(patched == before);
}

TEST(patch_row_single_key_rows_unchanged) {
  std::string text;
  REQUIRE(read_text_file(bundle_path(), &text));
  const Config cur = load_config(bundle_path());
  std::string patched = text, followed, err;
  REQUIRE(patch_row(&patched, 4, Value{Type::Int32, false, 30000}, cur, &followed, &err));
  CHECK(followed.empty());
  std::string want = text;
  REQUIRE(patch_toml_key(&want, "encoder", "bitrate_max_kbps", "30000"));
  CHECK(patched == want);
}

MTEST_MAIN
