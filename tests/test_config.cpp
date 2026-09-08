#include <atomic>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>

#include "mtest.h"
#include "config.h"
using namespace mabur;

namespace {

// Path to the committed default bundle config, resolved relative to this
// source file's known repo layout (tests/ -> ../bundle/mabur.default.toml).
std::string default_config_path() {
  return std::string(MABUR_BUNDLE_DIR) + "/mabur.default.toml";
}

// Writes `contents` to a fresh temp file and returns its path. Caller is
// responsible for cleanup (tests remove it at the end).
std::filesystem::path write_temp_toml(const std::string& contents) {
  static std::atomic<int> counter{0};
  auto path = std::filesystem::temp_directory_path() /
              ("mabur_test_config_" + std::to_string(counter++) + ".toml");
  std::ofstream f(path);
  f << contents;
  f.close();
  return path;
}

std::string what_of(const std::function<void()>& fn) {
  try {
    fn();
  } catch (const std::exception& e) {
    return e.what();
  }
  return "";
}

}  // namespace

TEST(load_config_default_file_is_the_flight_config) {
  Config cfg = load_config(default_config_path());
  Config def;  // struct defaults

  // What this test is for: `bundle/mabur.default.toml` is not a "sensible
  // starting point" any more. Since 2026-09-08 it is a verbatim copy of the
  // drone's own /etc/mabur.toml, and openipc-builder bakes it into the image
  // as /etc/mabur.toml -- so a fresh flash boots the flight configuration
  // with no hand-editing. That makes the file a deployment artifact, and this
  // test the change-detector on it: every literal below is a value someone
  // measured and flew, with the doc that justifies it. Changing one here
  // without changing the drone (or the reverse) is the bug this catches.
  //
  // It therefore checks against `def` only where bundle and struct still
  // legitimately agree; everywhere else it pins the flown literal.

  CHECK(cfg.radio.usb_vid == def.radio.usb_vid);
  CHECK(cfg.radio.usb_pid == def.radio.usb_pid);
  CHECK(cfg.radio.channel == 136);
  CHECK(cfg.radio.width == def.radio.width);
  // Was "none" (efuse table untouched) while the bundle was a neutral seed.
  // The flown config runs the adaptive per-rate offset mode; wall_margin_db
  // is the only lever that moves TX power in it (docs/txagcbench.md).
  CHECK(cfg.radio.power_mode == "offset");

  // Unit's measured wall-equalization (Task 9), unchanged by the cutover.
  CHECK((cfg.radio.rate_walls_idx ==
         std::array<int, 8>{91, 91, 91, 91, 73, 56, 51, 49}));
  CHECK(cfg.radio.legacy_wall_idx == 91);
  CHECK(cfg.radio.wall_margin_db == 1.0);
  CHECK(cfg.radio.base_ref_idx == 53);

  // 332/w32/bpb4 is the 2026-07-29 geometry: same CPU/air profile as the
  // 2026-07-25 gated 328 (docs/fec-symbol-size-328.md), shifted +4 because
  // 328x4 = 1396 B air frames sit exactly in the mcs6+STBC PHY hole
  // (docs/mcs6-bench-anomaly.md -- air MPDUs 1392-1400 B vanish whole at RX).
  // Any new size needs the all-8-MCS hole-scan.
  CHECK((cfg.fec.symbol_size == std::array<int, 2>{332, 332}));
  CHECK(cfg.fec.window == 32);
  CHECK((cfg.fec.blocks_per_body == std::array<int, 2>{4, 4}));
  CHECK(cfg.fec.base_overhead == def.fec.base_overhead);
  // Grouped submit, paired with ampdu.max_num 6 below: agg6 + feed_batch 6
  // bought fec -2.3/-2.7 ms with a flat p99 (docs/observability.md A-MPDU).
  CHECK(cfg.fec.feed_batch == 6);
  CHECK(cfg.fec.flush_ms == 25);

  // 16 Mbps ceiling and airtime_budget 0.5: 0.5 is what killed the air-clock
  // drain (peak 47 -> 20 ms, settle 1.9 -> 0.3 s). NOTE the sign of
  // roi_qp_low: apply_roi_qp() takes a QP OFFSET for the centre region, so
  // the useful low-bitrate value is NEGATIVE. It is still carried even though
  // venc.roi is off (below) -- turning ROI back on must not also need the
  // offset re-derived. The struct default has the opposite sign and is left
  // alone deliberately; changing a compiled default is not a flag day.
  CHECK(cfg.encoder.bitrate_min_kbps == 1000);
  CHECK(cfg.encoder.bitrate_max_kbps == 16000);
  CHECK(cfg.encoder.airtime_budget == 0.50);
  CHECK(cfg.encoder.roi_threshold_kbps == 3000);
  CHECK(cfg.encoder.roi_qp_low == -24);
  CHECK(cfg.encoder.roi_qp_normal == 0);

  // air_clock: ARMED. shed 25 / efficiency 0.73 is the combination that flew
  // clean -- 14 drops, all at rung transitions, 0 phantom (docs/airtime-model.md).
  CHECK(cfg.air_clock.shed_ms == 25);
  CHECK(cfg.air_clock.efficiency == 0.73);
  CHECK(cfg.air_clock.body_us == 0);

  // venc: boot-time encoder pipeline config, bundle-pinned rather than
  // struct-default (struct defaults are all-zero/empty, not a bootable
  // encoder configuration).
  CHECK(cfg.venc.core.sensor_bin ==
        std::string("/etc/sensors/imx415_greg_fpvXIX_colortrans.bin"));
  CHECK(cfg.venc.core.width == 1920);
  CHECK(cfg.venc.core.height == 1080);
  CHECK(cfg.venc.core.fps == 60);
  CHECK(cfg.venc.core.gop_s == 2.0);
  CHECK(cfg.venc.core.qp_delta == 4);
  CHECK(cfg.venc.core.max_ipprop == 2);
  // I-frame QP floor: the only knob that caps IDR size (bench 2026-09-06,
  // min_iqp 44 -> 2.2 kB IDRs vs 4.5-24 kB). docs/iqp-cap-findings-2026-09-06.md.
  CHECK(cfg.venc.core.min_iqp == 44);
  // 3, not the 1080p-derived 4: the drone encodes 720p, where the
  // rally-equivalent row count is 3 (docs/venc-resilience).
  CHECK(cfg.venc.core.intra_refresh_rows == 3);
  CHECK(cfg.venc.core.intra_refresh_qp == 36);
  // P-frame size cap, 200 % (docs/handover-venc-overshoot-2026-09-03.md).
  CHECK(cfg.venc.core.superframe_p_pct == 200);
  CHECK(cfg.venc.core.ref_base == 1);
  CHECK(cfg.venc.core.ref_enhance == 1);
  CHECK(cfg.venc.core.ref_pred == true);
  // ROI OFF since 2026-09-06: roi_qp_low -24 was being applied to the
  // SetChnAttr IDR at a rung-0 demote and blew it up 1.7-2.6x
  // (docs/link-adaptation.md, rung-0 demote IDR).
  CHECK(cfg.venc.core.roi_enabled == false);
  CHECK(cfg.venc.core.roi_steps == 2);
  CHECK(cfg.venc.core.roi_center == 0.4);
  CHECK(cfg.venc.core.ae_fps == 15);
  CHECK(cfg.venc.core.awb_fps == 15);
  CHECK(cfg.venc.core.snapshot_quality == 80);
  CHECK(cfg.venc.debug_port == 8301);
  CHECK(cfg.venc.module_loader == "/usr/bin/load_sigmastar -i");

  CHECK(cfg.link.vtx_id == def.link.vtx_id);
  // 3 s, not the compiled 1 s: a 1 s failsafe fired on ordinary rung
  // transitions in flight.
  CHECK(cfg.link.failsafe_ms == 3000);
  CHECK(cfg.link.rendezvous_ms == def.link.rendezvous_ms);
  CHECK(cfg.link.tick_ms == def.link.tick_ms);

  // MSP OSD is on in flight (stream_id 4), 3 Hz.
  CHECK(cfg.msp.enable == true);
  CHECK(cfg.msp.serial == std::string("/dev/ttyS2"));
  CHECK(cfg.msp.update_rate_hz == 3);

  // A-MPDU agg6; see fec.feed_batch above. agg31 cascades residuals.
  CHECK(cfg.ampdu.max_num == 6);
  CHECK(cfg.ampdu.max_time == 32);

  auto layers = cfg.uep_layers();
  // Literal passthrough (Task 3): no uep_layer_overhead ladder translation
  // left -- every layer's overhead is exactly fec.base_overhead.
  CHECK(layers[0].fec.overhead == cfg.fec.base_overhead);
  CHECK(layers[1].fec.overhead == cfg.fec.base_overhead);
}

TEST(load_config_missing_file_throws) {
  bool threw = false;
  std::string msg;
  try {
    load_config("/nonexistent/path/does/not/exist/mabur.toml");
  } catch (const std::runtime_error& e) {
    threw = true;
    msg = e.what();
  }
  CHECK(threw);
  CHECK(msg.find("config:") == 0);
}

TEST(load_config_out_of_range_field_throws_naming_field) {
  auto path = write_temp_toml("[fec]\nwindow = 9999\n");
  std::string msg = what_of([&] { (void)load_config(path.string()); });
  CHECK(!msg.empty());
  CHECK(msg.find("fec.window") != std::string::npos);
  std::filesystem::remove(path);
}

TEST(load_config_unknown_top_level_key_throws_naming_it) {
  auto path = write_temp_toml("typo = 1\n");
  std::string msg = what_of([&] { (void)load_config(path.string()); });
  CHECK(!msg.empty());
  CHECK(msg.find("typo") != std::string::npos);
  std::filesystem::remove(path);
}

TEST(load_config_unknown_nested_key_throws_naming_it) {
  auto path = write_temp_toml("[fec]\nkx = 8\n");
  std::string msg = what_of([&] { (void)load_config(path.string()); });
  CHECK(!msg.empty());
  CHECK(msg.find("fec.kx") != std::string::npos);
  std::filesystem::remove(path);
}

TEST(load_config_type_mismatch_fec_window_string_throws_runtime_error_with_dotted_path) {
  auto path = write_temp_toml("[fec]\nwindow = \"wide\"\n");
  std::string msg = what_of([&] { (void)load_config(path.string()); });
  CHECK(!msg.empty());
  CHECK(msg.find("fec.window") != std::string::npos);
  CHECK(msg.find("wrong type") != std::string::npos);
  std::filesystem::remove(path);
}

TEST(load_config_unknown_radio_bw_set_key_throws) {
  // radio.bw_set (bandwidth-probe schedule) was removed 2026-07-27 (SDD
  // ladder-controller Task 5): the ladder controller never varies bw
  // independently of the commanded rung, so the probe schedule and its
  // config key are dead. Strict-keys config load (PR #7) must reject it
  // like any other unknown key rather than silently ignoring it.
  auto path = write_temp_toml("[radio]\nbw_set = [20, 40]\n");
  std::string msg = what_of([&] { (void)load_config(path.string()); });
  CHECK(!msg.empty());
  CHECK(msg.find("radio.bw_set") != std::string::npos);
  CHECK(msg.find("unknown key") != std::string::npos);
  std::filesystem::remove(path);
}

// waybeam is retired (spec 2026-08-28 venc-foldin, Task B5): the section
// and its host/port/idr_path keys are gone entirely, strict keys reject any
// config that still carries it. See waybeam_section_is_now_unknown below.
// The surviving bitrate/ROI policy fields moved to Config::encoder.

TEST(load_config_encoder_bitrate_min_not_less_than_max_throws_naming_field) {
  auto path = write_temp_toml(
      "[encoder]\nbitrate_min_kbps = 5000\nbitrate_max_kbps = 5000\n");
  std::string msg = what_of([&] { (void)load_config(path.string()); });
  CHECK(!msg.empty());
  CHECK(msg.find("encoder.bitrate_min_kbps") != std::string::npos);
  std::filesystem::remove(path);
}

TEST(load_config_encoder_bitrate_min_below_floor_throws_naming_field) {
  auto path = write_temp_toml("[encoder]\nbitrate_min_kbps = 50\n");
  std::string msg = what_of([&] { (void)load_config(path.string()); });
  CHECK(!msg.empty());
  CHECK(msg.find("encoder.bitrate_min_kbps") != std::string::npos);
  std::filesystem::remove(path);
}

TEST(load_config_encoder_airtime_budget_out_of_range_throws_naming_field) {
  auto path = write_temp_toml("[encoder]\nairtime_budget = 1.5\n");
  std::string msg = what_of([&] { (void)load_config(path.string()); });
  CHECK(!msg.empty());
  CHECK(msg.find("encoder.airtime_budget") != std::string::npos);
  std::filesystem::remove(path);
}

TEST(load_config_encoder_roi_threshold_negative_throws_naming_field) {
  auto path = write_temp_toml("[encoder]\nroi_threshold_kbps = -1\n");
  std::string msg = what_of([&] { (void)load_config(path.string()); });
  CHECK(!msg.empty());
  CHECK(msg.find("encoder.roi_threshold_kbps") != std::string::npos);
  std::filesystem::remove(path);
}

// ---- Task B5: venc section / waybeam retirement -------------------------

// A full, valid venc block matching the brief's fixture (spec 2026-08-28
// venc-foldin Task B5 Step 1), reused across the tests below.
std::string valid_venc_block() {
  return "[venc]\n"
         "sensor_bin = \"/etc/sensors/imx415_greg_fpvXIX_colortrans.bin\"\n"
         "size = \"1920x1080\"\n"
         "fps = 60\n"
         "gop_s = 2.0\n"
         "qp_delta = -4\n"
         "intra_refresh_rows = 4\n"
         "intra_refresh_qp = 36\n"
         "ref_base = 1\n"
         "ref_enhance = 1\n"
         "ref_pred = true\n"
         "ae_fps = 15\n"
         "awb_fps = 15\n"
         "snapshot_quality = 80\n"
         "debug_port = 8301\n"
         "module_loader = \"\"\n"
         "\n"
         "[venc.roi]\n"
         "enabled = true\n"
         "steps = 2\n"
         "center = 0.4\n";
}

TEST(venc_section_parses_and_validates) {
  auto path = write_temp_toml(
      valid_venc_block() +
      "\n[encoder]\n"
      "bitrate_min_kbps = 1000\n"
      "bitrate_max_kbps = 20000\n"
      "airtime_budget = 0.65\n"
      "roi_threshold_kbps = 3000\n"
      "roi_qp_low = 8\n"
      "roi_qp_normal = 0\n");
  Config c = load_config(path.string());
  CHECK(c.venc.core.width == 1920);
  CHECK(c.venc.core.height == 1080);
  CHECK(c.venc.core.intra_refresh_rows == 4);
  CHECK(c.venc.core.ref_enhance == 1);
  CHECK(c.encoder.airtime_budget == 0.65);
  CHECK(c.venc.module_loader.empty());  // "" = never run a loader, only wait
  std::filesystem::remove(path);
}

TEST(waybeam_section_is_now_unknown) {
  auto path = write_temp_toml("[waybeam]\nhost = \"127.0.0.1\"\n");
  std::string msg = what_of([&] { (void)load_config(path.string()); });
  CHECK(!msg.empty());
  CHECK(msg.find("waybeam") != std::string::npos);
  CHECK(msg.find("unknown key") != std::string::npos);
  std::filesystem::remove(path);
}

TEST(venc_rejects_bitrate_key) {
  // spec 2026-08-28 venc-foldin §3: no venc.bitrate key ever exists. It
  // simply isn't in venc's known-key set, so this hits the same unknown-key
  // path as any other stale key.
  auto path = write_temp_toml(
      "[venc]\n"
      "sensor_bin = \"/etc/sensors/imx415_greg_fpvXIX_colortrans.bin\"\n"
      "size = \"1920x1080\"\n"
      "fps = 60\n"
      "gop_s = 2.0\n"
      "qp_delta = -4\n"
      "bitrate = 8000\n"
      "ae_fps = 15\n"
      "awb_fps = 15\n"
      "snapshot_quality = 80\n"
      "debug_port = 8301\n"
      "\n"
      "[venc.roi]\n"
      "enabled = true\n"
      "steps = 2\n"
      "center = 0.4\n");
  std::string msg = what_of([&] { (void)load_config(path.string()); });
  CHECK(!msg.empty());
  CHECK(msg.find("venc.bitrate") != std::string::npos);
  std::filesystem::remove(path);
}

// Absent venc keys fall back to the spec §3 values (venc_cfg_defaults(),
// drone/venc/venc_cfg.c), NOT to the all-zero a plain `VencCfg core{}`
// would give: a zeroed VencCfg is fps 0 / 0x0 / gop 0.0 / no stripe,
// which is a malformed pipeline dressed up as a default.
// REVERT CHECK: delete the VencSectionCfg() constructor in config.h (or the
// body of venc_cfg_defaults) and every CHECK below reads 0/"".
TEST(venc_absent_keys_fall_back_to_spec_defaults) {
  // Only the one REQUIRED key present; everything else omitted.
  auto path = write_temp_toml(
      "[venc]\nsensor_bin = \"/etc/sensors/imx415_greg_fpvXIX_colortrans.bin\"\n");
  Config c = load_config(path.string());
  CHECK(c.venc.core.fps == 60);
  CHECK(c.venc.core.width == 1920);
  CHECK(c.venc.core.height == 1080);
  CHECK(c.venc.core.gop_s == 2.0);
  CHECK(c.venc.core.qp_delta == -4);
  CHECK(c.venc.core.max_ipprop == 0);
  CHECK(c.venc.core.superframe_p_pct == 0);
  // The decomposed resilience components (venc.resilience was deleted
  // 2026-09-04): defaults reproduce the old "rally" preset at 1080p60 —
  // 4 CTU rows/P at QP 36, 1:1 SVC-T with enhance prediction on.
  CHECK(c.venc.core.intra_refresh_rows == 4);
  CHECK(c.venc.core.intra_refresh_qp == 36);
  CHECK(c.venc.core.ref_base == 1);
  CHECK(c.venc.core.ref_enhance == 1);
  CHECK(c.venc.core.ref_pred == true);
  CHECK(c.venc.core.roi_enabled == true);
  CHECK(c.venc.core.roi_steps == 2);
  CHECK(c.venc.core.roi_center == 0.4);
  CHECK(c.venc.core.ae_fps == 15);
  CHECK(c.venc.core.awb_fps == 15);
  CHECK(c.venc.core.snapshot_quality == 80);
  CHECK(c.venc.debug_port == 8301);
  CHECK(c.venc.module_loader == "/usr/bin/load_sigmastar -i");
  std::filesystem::remove(path);
}

// The six decomposed knobs land on VencCfg verbatim — they are the two MI
// structs' fields (MI_VENC_IntraRefresh_t {bEnable, u32RefreshLineNum,
// u32ReqIQp} and MI_VENC_ParamRef_t {u32Base, u32Enhance, bEnablePred}),
// so config carries no derivation the operator cannot see.
TEST(venc_intra_refresh_and_ref_keys_parse) {
  auto path = write_temp_toml(
      "[venc]\n"
      "sensor_bin = \"/etc/sensors/imx415_greg_fpvXIX_colortrans.bin\"\n"
      "intra_refresh_rows = 1\n"
      "intra_refresh_qp = 28\n"
      "ref_base = 1\n"
      "ref_enhance = 4\n"
      "ref_pred = false\n");
  Config c = load_config(path.string());
  CHECK(c.venc.core.intra_refresh_rows == 1);
  CHECK(c.venc.core.intra_refresh_qp == 28);
  CHECK(c.venc.core.ref_base == 1);
  CHECK(c.venc.core.ref_enhance == 4);
  CHECK(c.venc.core.ref_pred == false);
  std::filesystem::remove(path);
}

// rows 0 is the off switch (bEnable=0): no stripe, and the QP alongside it
// is simply unused rather than an error.
TEST(venc_intra_refresh_rows_zero_is_off) {
  auto path = write_temp_toml(
      "[venc]\n"
      "sensor_bin = \"/etc/sensors/imx415_greg_fpvXIX_colortrans.bin\"\n"
      "intra_refresh_rows = 0\n");
  Config c = load_config(path.string());
  CHECK(c.venc.core.intra_refresh_rows == 0);
  std::filesystem::remove(path);
}

// The stripe cannot be wider than the picture. H.265 CTU is 32x32, so 1080
// lines is 34 CTU rows: 34 is the widest legal sweep (one row per frame is
// the slowest), 35 is a config error. The old preset path silently CLAMPED
// here and warned on stderr, which meant the encoder ran a sweep the
// config did not describe; boot failure is the forcing function instead.
TEST(venc_intra_refresh_rows_beyond_picture_rejected) {
  auto ok = write_temp_toml(
      "[venc]\n"
      "sensor_bin = \"/etc/sensors/imx415_greg_fpvXIX_colortrans.bin\"\n"
      "size = \"1920x1080\"\n"
      "intra_refresh_rows = 34\n");
  Config c = load_config(ok.string());
  CHECK(c.venc.core.intra_refresh_rows == 34);
  std::filesystem::remove(ok);

  auto bad = write_temp_toml(
      "[venc]\n"
      "sensor_bin = \"/etc/sensors/imx415_greg_fpvXIX_colortrans.bin\"\n"
      "size = \"1920x1080\"\n"
      "intra_refresh_rows = 35\n");
  std::string msg = what_of([&] { (void)load_config(bad.string()); });
  CHECK(!msg.empty());
  CHECK(msg.find("venc.intra_refresh_rows") != std::string::npos);
  std::filesystem::remove(bad);
}

// The bound tracks venc.size, and is checked against the height whether or
// not the rows key is present. Testing it only at 1920x1080 -- which is also
// the default height -- would pass against a hardcoded 34, and against a
// check that ran BEFORE "size" was parsed. 720 lines is 23 CTU rows: a
// pre-size check would see the 1080 default, compute 34, and accept 24.
TEST(venc_intra_refresh_rows_bound_tracks_venc_size) {
  auto ok = write_temp_toml(
      "[venc]\n"
      "sensor_bin = \"/etc/sensors/imx415_greg_fpvXIX_colortrans.bin\"\n"
      "size = \"1280x720\"\n"
      "intra_refresh_rows = 23\n");
  Config c = load_config(ok.string());
  CHECK(c.venc.core.intra_refresh_rows == 23);
  std::filesystem::remove(ok);

  auto bad = write_temp_toml(
      "[venc]\n"
      "sensor_bin = \"/etc/sensors/imx415_greg_fpvXIX_colortrans.bin\"\n"
      "size = \"1280x720\"\n"
      "intra_refresh_rows = 24\n");
  std::string msg = what_of([&] { (void)load_config(bad.string()); });
  CHECK(!msg.empty());
  CHECK(msg.find("venc.intra_refresh_rows") != std::string::npos);
  std::filesystem::remove(bad);
}

// The default rows value is validated too. Omitting the key does not buy a
// pass: a picture shorter than the default 4 CTU rows must fail boot, not
// reach the encoder and get silently clamped there -- boot failure is the
// whole point of the range check.
TEST(venc_default_intra_refresh_rows_validated_against_size) {
  auto path = write_temp_toml(
      "[venc]\n"
      "sensor_bin = \"/etc/sensors/imx415_greg_fpvXIX_colortrans.bin\"\n"
      "size = \"640x64\"\n");
  std::string msg = what_of([&] { (void)load_config(path.string()); });
  CHECK(!msg.empty());
  CHECK(msg.find("venc.intra_refresh_rows") != std::string::npos);
  std::filesystem::remove(path);
}

// u32ReqIQp is an H.265 QP: [1,51]. 0 reached the SDK as "codec default"
// under the preset table, but with the mode names gone there is no default
// to fall back TO, so 0 is now a plain out-of-range value.
TEST(venc_intra_refresh_qp_out_of_range_rejected) {
  for (const char* qp : {"0", "52"}) {
    auto path = write_temp_toml(
        std::string(
            "[venc]\n"
            "sensor_bin = \"/etc/sensors/imx415_greg_fpvXIX_colortrans.bin\"\n"
            "intra_refresh_qp = ") +
        qp + "\n");
    std::string msg = what_of([&] { (void)load_config(path.string()); });
    CHECK(!msg.empty());
    CHECK(msg.find("venc.intra_refresh_qp") != std::string::npos);
    std::filesystem::remove(path);
  }
}

// u32Enhance is a PERIOD (one non-referenced frame per enhance+1), so 0 is
// meaningless while SVC-T is on. The preset path papered over this with a
// `enhance ? enhance : 1` fallback at the apply site; config rejects it.
TEST(venc_ref_enhance_zero_with_svct_on_rejected) {
  auto path = write_temp_toml(
      "[venc]\n"
      "sensor_bin = \"/etc/sensors/imx415_greg_fpvXIX_colortrans.bin\"\n"
      "ref_base = 1\n"
      "ref_enhance = 0\n");
  std::string msg = what_of([&] { (void)load_config(path.string()); });
  CHECK(!msg.empty());
  CHECK(msg.find("venc.ref_enhance") != std::string::npos);
  std::filesystem::remove(path);
}

// ref_base 0 disables SVC-T outright (no MI_VENC_SetRefParam call), and
// then ref_enhance is unused — a 0 alongside it is not an error.
TEST(venc_ref_base_zero_disables_svct) {
  auto path = write_temp_toml(
      "[venc]\n"
      "sensor_bin = \"/etc/sensors/imx415_greg_fpvXIX_colortrans.bin\"\n"
      "ref_base = 0\n"
      "ref_enhance = 0\n");
  Config c = load_config(path.string());
  CHECK(c.venc.core.ref_base == 0);
  std::filesystem::remove(path);
}

// venc.resilience was deleted 2026-09-04 in favour of the six components
// above. A stale config still naming it must fail boot on the ordinary
// unknown-key path, not be silently ignored while the encoder runs
// something else (config strict-keys policy, CLAUDE.md).
TEST(venc_stale_resilience_key_throws) {
  auto path = write_temp_toml(
      "[venc]\n"
      "sensor_bin = \"/etc/sensors/imx415_greg_fpvXIX_colortrans.bin\"\n"
      "resilience = \"rally\"\n");
  std::string msg = what_of([&] { (void)load_config(path.string()); });
  CHECK(!msg.empty());
  CHECK(msg.find("resilience") != std::string::npos);
  std::filesystem::remove(path);
}

// max_ipprop is optional: absent -> 0 (spec default, tested above), a
// legal in-range value lands verbatim in VencCfg.
TEST(venc_max_ipprop_parses) {
  auto path = write_temp_toml(
      "[venc]\n"
      "sensor_bin = \"/etc/sensors/imx415_greg_fpvXIX_colortrans.bin\"\n"
      "max_ipprop = 2\n");
  Config c = load_config(path.string());
  CHECK(c.venc.core.max_ipprop == 2);
  std::filesystem::remove(path);
}

// min_iqp: 0 (default) = leave the firmware I-QP floor; 1..51 = program
// u32MinIQp at boot. The one direct IDR SIZE bound star6e honours
// (bench 2026-09-06: 12/36/42/48 -> 11.6/5.0/2.6/1.4 kB, P untouched).
TEST(venc_min_iqp_parses) {
  auto path = write_temp_toml(
      "[venc]\n"
      "sensor_bin = \"/etc/sensors/imx415_greg_fpvXIX_colortrans.bin\"\n"
      "min_iqp = 44\n");
  Config c = load_config(path.string());
  CHECK(c.venc.core.min_iqp == 44);
  std::filesystem::remove(path);
}

TEST(venc_min_iqp_absent_leaves_firmware_floor) {
  auto path = write_temp_toml(
      "[venc]\nsensor_bin = \"/etc/sensors/imx415_greg_fpvXIX_colortrans.bin\"\n");
  Config c = load_config(path.string());
  CHECK(c.venc.core.min_iqp == 0);
  std::filesystem::remove(path);
}

// superframe_p_pct: 0 (default) = off; 100..1000 = P-frame ceiling as a
// percentage of the rung's per-frame budget via MI_VENC_SetSuperFrameCfg
// REENCODE (I unlimited). Below 100 is rejected: a cap under the budget
// makes CBR re-plan far under it (fork probe 2026-08-27: 6000 B cap -> 3.2 kB
// frames) and is a quality collapse, not a burst bound.
TEST(venc_superframe_p_pct_parses) {
  auto path = write_temp_toml(
      "[venc]\n"
      "sensor_bin = \"/etc/sensors/imx415_greg_fpvXIX_colortrans.bin\"\n"
      "superframe_p_pct = 200\n");
  Config c = load_config(path.string());
  CHECK(c.venc.core.superframe_p_pct == 200);
  std::filesystem::remove(path);
}

// venc.min_qp was a one-day bench knob (2026-09-03) and is DELETED: the
// bench refuted the QP-floor hypothesis it existed for, and upstream
// characterised u32MinQp as a bit ceiling that collapses the rate. The
// key now fails boot like any other unknown key — the intended forcing
// function (CLAUDE.md compatibility policy).
TEST(venc_min_qp_is_unknown) {
  auto path = write_temp_toml(
      "[venc]\n"
      "sensor_bin = \"/etc/sensors/imx415_greg_fpvXIX_colortrans.bin\"\n"
      "min_qp = 24\n");
  bool threw = false;
  try { load_config(path.string()); } catch (const std::exception&) { threw = true; }
  CHECK(threw);
  std::filesystem::remove(path);
}

// sensor_bin is the ONE venc key with no default: it names a device-specific
// ISP calibration blob, and there is no value that is right for an unknown
// camera. Absent => boot failure, per the project's config-strict policy.
// REVERT CHECK: remove the sensor_bin[0] check at the end of parse_venc and
// this load succeeds with an empty sensor_bin.
TEST(venc_sensor_bin_is_required) {
  auto path = write_temp_toml("[venc]\nfps = 60\n");
  std::string msg = what_of([&] { (void)load_config(path.string()); });
  CHECK(!msg.empty());
  CHECK(msg.find("venc.sensor_bin") != std::string::npos);
  std::filesystem::remove(path);
}

TEST(venc_size_malformed_throws) {
  auto path = write_temp_toml("[venc]\nsize = \"1920\"\n");
  std::string msg = what_of([&] { (void)load_config(path.string()); });
  CHECK(!msg.empty());
  CHECK(msg.find("venc.size") != std::string::npos);
  std::filesystem::remove(path);
}

TEST(venc_range_checks) {
  // Each case names the field its own out-of-range value should be
  // reported against (review finding 2026-08-29: a loose "venc." find()
  // let 10/14 cases silently pass on a DIFFERENT field's error message
  // — the venc.gop_s-validated-unconditionally bug masked here because
  // every case happened to also fail gop_s's range check first).
  struct Case { const char* toml; const char* field; };
  for (const Case& c : {
           Case{"[venc]\nfps = 0\n", "venc.fps"},
           Case{"[venc]\nfps = 121\n", "venc.fps"},
           Case{"[venc]\ngop_s = 0.1\n", "venc.gop_s"},
           Case{"[venc]\ngop_s = 11\n", "venc.gop_s"},
           Case{"[venc]\nqp_delta = -13\n", "venc.qp_delta"},
           Case{"[venc]\nqp_delta = 13\n", "venc.qp_delta"},
           Case{"[venc]\nmax_ipprop = -1\n", "venc.max_ipprop"},
           Case{"[venc]\nmax_ipprop = 101\n", "venc.max_ipprop"},
           Case{"[venc]\nmin_iqp = -1\n", "venc.min_iqp"},
           Case{"[venc]\nmin_iqp = 52\n", "venc.min_iqp"},
           Case{"[venc]\nsuperframe_p_pct = 50\n", "venc.superframe_p_pct"},
           Case{"[venc]\nsuperframe_p_pct = 1001\n", "venc.superframe_p_pct"},
           Case{"[venc]\nsnapshot_quality = 0\n", "venc.snapshot_quality"},
           Case{"[venc]\nsnapshot_quality = 101\n", "venc.snapshot_quality"},
           Case{"[venc]\ndebug_port = 1023\n", "venc.debug_port"},
           Case{"[venc]\ndebug_port = 65536\n", "venc.debug_port"},
           Case{"[venc.roi]\nsteps = 0\n", "venc.roi.steps"},
           Case{"[venc.roi]\nsteps = 5\n", "venc.roi.steps"},
           Case{"[venc.roi]\ncenter = -0.1\n", "venc.roi.center"},
           Case{"[venc.roi]\ncenter = 1.1\n", "venc.roi.center"},
           // ae_fps/awb_fps are range-checked BEFORE the uint16 cast: -1
           // used to wrap to 65535 and 0 used to sail through as "run the
           // ISP loop at no rate at all", both of which reach the MI ISP
           // looking legal and misbehave on hardware instead of failing
           // boot.
           // REVERT CHECK: drop the `< 1` half of either range check and the
           // -1 and 0 cases stop throwing (the load succeeds).
           Case{"[venc]\nae_fps = -1\n", "venc.ae_fps"},
           Case{"[venc]\nae_fps = 0\n", "venc.ae_fps"},
           Case{"[venc]\nae_fps = 61\n", "venc.ae_fps"},
           Case{"[venc]\nawb_fps = -1\n", "venc.awb_fps"},
           Case{"[venc]\nawb_fps = 0\n", "venc.awb_fps"},
           Case{"[venc]\nawb_fps = 61\n", "venc.awb_fps"},
       }) {
    auto path = write_temp_toml(c.toml);
    std::string msg = what_of([&] { (void)load_config(path.string()); });
    CHECK(!msg.empty());
    CHECK(msg.find(c.field) != std::string::npos);
    std::filesystem::remove(path);
  }
}

TEST(uep_layers_overhead_is_literal_base_overhead) {
  Config cfg;  // defaults: base_overhead = 0.5, literal (Task 3)
  auto layers = cfg.uep_layers();
  CHECK(layers[0].fec.overhead == cfg.fec.base_overhead);
  CHECK(layers[1].fec.overhead == cfg.fec.base_overhead);
  CHECK(layers[0].fec.window == cfg.fec.window);
  CHECK(layers[0].fec.symbol_size == cfg.fec.symbol_size[0]);
  CHECK(layers[0].blocks_per_body == cfg.fec.blocks_per_body[0]);
  CHECK(layers[1].blocks_per_body == cfg.fec.blocks_per_body[1]);
}

TEST(fec_symbol_size_scalar_fans_out) {
  // 164 keeps every layer's body (bpb*(hdr+symbol_size)) within
  // kMaxBodyBytes at the default blocks_per_body {4,8}: 8*(14+164)=1424 <
  // 2900.
  auto path = write_temp_toml("[fec]\nsymbol_size = 164\n");
  Config cfg = load_config(path.string());
  for (int s = 0; s < 2; ++s) CHECK(cfg.fec.symbol_size[s] == 164);
  std::filesystem::remove(path);
}

TEST(fec_symbol_size_array_per_layer) {
  auto path = write_temp_toml(
      "[fec]\nsymbol_size = [164, 1312]\nblocks_per_body = [4, 1]\n");
  Config cfg = load_config(path.string());
  CHECK(cfg.fec.symbol_size[0] == 164);
  CHECK(cfg.fec.symbol_size[1] == 1312);
  auto layers = cfg.uep_layers();
  CHECK(layers[0].fec.symbol_size == 164);
  CHECK(layers[1].fec.symbol_size == 1312);
  std::filesystem::remove(path);
}

TEST(fec_feed_batch_parses_and_defaults_off) {
  Config def;
  CHECK(def.fec.feed_batch == 0);  // streaming push is the default shape
  auto path = write_temp_toml("[fec]\nfeed_batch = 3\n");
  Config cfg = load_config(path.string());
  CHECK(cfg.fec.feed_batch == 3);
  std::filesystem::remove(path);
}

TEST(fec_feed_batch_out_of_range_throws_naming_field) {
  auto path = write_temp_toml("[fec]\nfeed_batch = 9\n");
  std::string msg = what_of([&] { (void)load_config(path.string()); });
  CHECK(!msg.empty());
  CHECK(msg.find("fec.feed_batch") != std::string::npos);
  std::filesystem::remove(path);
}

TEST(fec_symbol_size_rejects_wrong_len_array) {
  auto path = write_temp_toml("[fec]\nsymbol_size = [164, 1312, 164]\n");
  bool threw = false;
  try {
    (void)load_config(path.string());
  } catch (const std::runtime_error&) {
    threw = true;
  }
  CHECK(threw);
  std::filesystem::remove(path);
}

TEST(fec_symbol_size_rejects_oversize_body) {
  // 1312B symbols at bpb 8 -> 8*(14+1312) = 10608 > kMaxBodyBytes 2900
  auto path = write_temp_toml(
      "[fec]\nsymbol_size = 1312\nblocks_per_body = [8, 8]\n");
  bool threw = false;
  try {
    (void)load_config(path.string());
  } catch (const std::runtime_error&) {
    threw = true;
  }
  CHECK(threw);
  std::filesystem::remove(path);
}

TEST(fec_symbol_size_bounds) {
  {
    auto path = write_temp_toml("[fec]\nsymbol_size = 16\n");  // <32
    bool threw = false;
    try {
      (void)load_config(path.string());
    } catch (const std::runtime_error&) {
      threw = true;
    }
    CHECK(threw);
    std::filesystem::remove(path);
  }
  {
    auto path = write_temp_toml("[fec]\nsymbol_size = 1600\n");  // >1500
    bool threw = false;
    try {
      (void)load_config(path.string());
    } catch (const std::runtime_error&) {
      threw = true;
    }
    CHECK(threw);
    std::filesystem::remove(path);
  }
}

TEST(msp_defaults_and_parse) {
  // Defaults: disabled, ttyS2, 1 Hz.
  {
    auto path = write_temp_toml("");
    auto cfg = load_config(path.string());
    CHECK(cfg.msp.enable == false);
    CHECK(cfg.msp.serial == "/dev/ttyS2");
    CHECK(cfg.msp.baud == 115200);
    CHECK(cfg.msp.update_rate_hz == 1.0);
    CHECK(cfg.msp.symbol_size == 1312);
    std::filesystem::remove(path);
  }
  // Explicit values.
  {
    auto path = write_temp_toml(
        "[msp]\n"
        "enable = true\n"
        "serial = \"/dev/ttyS1\"\n"
        "baud = 230400\n"
        "update_rate_hz = 2.0\n"
        "symbol_size = 1024\n"
        "window = 32\n"
        "overhead = 0.5\n");
    auto cfg = load_config(path.string());
    CHECK(cfg.msp.enable == true);
    CHECK(cfg.msp.serial == "/dev/ttyS1");
    CHECK(cfg.msp.baud == 230400);
    CHECK(cfg.msp.update_rate_hz == 2.0);
    CHECK(cfg.msp.symbol_size == 1024);
    CHECK(cfg.msp.window == 32);
    std::filesystem::remove(path);
  }
}

TEST(msp_rejects_bad_values) {
  {
    auto path = write_temp_toml("[msp]\nupdate_rate_hz = 0\n");
    bool threw = false;
    try {
      (void)load_config(path.string());
    } catch (const std::exception&) {
      threw = true;
    }
    CHECK(threw == true);
    std::filesystem::remove(path);
  }
  {
    auto path = write_temp_toml("[msp]\nnonsense = 1\n");
    bool threw = false;
    try {
      (void)load_config(path.string());
    } catch (const std::exception&) {
      threw = true;
    }
    CHECK(threw == true);
    std::filesystem::remove(path);
  }
}

TEST(radio_wall_equalization_keys_parse) {
  auto path = write_temp_toml(
      "[radio]\n"
      "power_mode = \"offset\"\n"
      "rate_walls_idx = [91, 91, 91, 91, 73, 56, 51, 49]\n"
      "legacy_wall_idx = 91\n"
      "wall_margin_db = 2.0\n"
      "base_ref_idx = 50\n");
  Config cfg = load_config(path.string());
  CHECK((cfg.radio.rate_walls_idx ==
         std::array<int, 8>{91, 91, 91, 91, 73, 56, 51, 49}));
  CHECK(cfg.radio.legacy_wall_idx == 91);
  CHECK(cfg.radio.wall_margin_db == 2.0);
  CHECK(cfg.radio.base_ref_idx == 50);
  std::filesystem::remove(path);
}

TEST(radio_rate_walls_idx_wrong_length_rejected) {
  auto path = write_temp_toml("[radio]\nrate_walls_idx = [91, 91, 91]\n");
  std::string msg = what_of([&] { (void)load_config(path.string()); });
  CHECK(!msg.empty());
  CHECK(msg.find("radio.rate_walls_idx") != std::string::npos);
  std::filesystem::remove(path);
}

TEST(radio_power_mode_offset_requires_rate_walls_idx) {
  auto path = write_temp_toml("[radio]\npower_mode = \"offset\"\n");
  std::string msg = what_of([&] { (void)load_config(path.string()); });
  CHECK(!msg.empty());
  CHECK(msg.find("radio.rate_walls_idx") != std::string::npos);
  std::filesystem::remove(path);
}

// The 8822E's per-rate diff field is 7-bit two's complement: diff[r] =
// walls[r] - wall_margin_db*4 - base_ref_idx must land in [-64,63], or the
// value silently wraps on air (e.g. +70 -> -58, sign-flipping per-rate
// power) with no error. base_ref_idx left at 0 (a plausible miscalibration:
// forgetting to set the unit's efuse anchor) drives every wall straight out
// of range, so config load must fail loudly rather than let power_plan.h's
// clamp paper over it silently.
TEST(radio_offset_diff_out_of_range_rejected) {
  auto path = write_temp_toml(
      "[radio]\n"
      "power_mode = \"offset\"\n"
      "rate_walls_idx = [127, 127, 127, 127, 127, 127, 127, 127]\n"
      "legacy_wall_idx = 91\n"
      "wall_margin_db = 0.0\n"
      "base_ref_idx = 0\n");
  std::string msg = what_of([&] { (void)load_config(path.string()); });
  CHECK(!msg.empty());
  CHECK(msg.find("radio.rate_walls_idx") != std::string::npos);
  CHECK(msg.find("[-64,63]") != std::string::npos);
  std::filesystem::remove(path);
}

// The transitional async gate was removed after hardware acceptance (plan
// 2026-07-17 Task 7): async is the only mode. A stale config still carrying
// the key must fail loudly, not be silently ignored.
TEST(fec_stale_async_worker_key_throws) {
  auto p = write_temp_toml("[fec]\nasync_worker = true\n");
  std::string w = what_of([&] { load_config(p.string()); });
  CHECK(w.find("async_worker") != std::string::npos);
  std::filesystem::remove(p);
}

// frame_ring_name was deleted (spec 2026-08-28 venc-foldin, controller
// ruling on Task B5): the ring name's single authority is now the
// compile-time VENC_RING_NAME in drone/venc/venc_cfg.h. A config that still
// carries the key hits the ordinary unknown-key path — see
// stale_video_input_and_ring_name_keys_throw below for the sibling
// pre-frame-shm keys that already went through this.
TEST(stale_frame_ring_name_key_throws) {
  auto path = write_temp_toml("frame_ring_name = \"mabur_f\"\n");
  std::string msg = what_of([&] { (void)load_config(path.string()); });
  CHECK(!msg.empty());
  CHECK(msg.find("frame_ring_name") != std::string::npos);
  CHECK(msg.find("unknown key") != std::string::npos);
  std::filesystem::remove(path);
}

// video_input/ring_name selected and named the pre-frame-shm RTP-packet ring.
// Their accept-and-warn grace release has passed and the drone's live
// /etc/mabur.toml no longer carries them, so they now hit the blanket
// unknown-key check like any other stale key.
TEST(stale_video_input_and_ring_name_keys_throw) {
  auto path = write_temp_toml("video_input = \"frame_ring\"\n");
  std::string msg = what_of([&] { (void)load_config(path.string()); });
  CHECK(msg.find("video_input") != std::string::npos);
  CHECK(msg.find("unknown key") != std::string::npos);
  std::filesystem::remove(path);

  auto path2 = write_temp_toml("ring_name = \"mabur\"\n");
  std::string msg2 = what_of([&] { (void)load_config(path2.string()); });
  CHECK(msg2.find("ring_name") != std::string::npos);
  CHECK(msg2.find("unknown key") != std::string::npos);
  std::filesystem::remove(path2);
}

// The flags block tuned per-rung LDPC/STBC policy. Removed 2026-07-26:
// LDPC+STBC are now hardcoded true on every rung in both ladder builders
// (the deployed all-true config was the only shape ever flown; flags-off
// T1/T2 measured 2-3 dB weaker on air). A stale block fails the boot.
TEST(stale_flags_key_throws) {
  auto path = write_temp_toml("[flags]\ncrit_ldpc = true\n");
  std::string msg = what_of([&] { (void)load_config(path.string()); });
  CHECK(msg.find("flags") != std::string::npos);
  CHECK(msg.find("unknown key") != std::string::npos);
  std::filesystem::remove(path);
}

// radio.max_txagc was the legacy TXAGC-index ceiling; Task 11 moved the
// power path to qdB offsets and nothing has read it since. The live config
// was scrubbed 2026-07-26, so a stale key fails the boot loudly (note: the
// pre-sym328 rollback config still carries it — edit before rolling back).
TEST(stale_radio_max_txagc_key_throws) {
  auto path = write_temp_toml("[radio]\nmax_txagc = 40\n");
  std::string msg = what_of([&] { (void)load_config(path.string()); });
  CHECK(msg.find("radio.max_txagc") != std::string::npos);
  CHECK(msg.find("unknown key") != std::string::npos);
  std::filesystem::remove(path);
}

// power_offset_db fed the ladder's carried-but-never-emitted per-rung field
// (radio_tx.h: no DBM_TX_POWER radiotap is ever written). Scrubbed from the
// live config 2026-07-26; stale key fails the boot.
TEST(stale_power_offset_db_key_throws) {
  auto path = write_temp_toml("power_offset_db = [0, 0, 0, 0]\n");
  std::string msg = what_of([&] { (void)load_config(path.string()); });
  CHECK(msg.find("power_offset_db") != std::string::npos);
  CHECK(msg.find("unknown key") != std::string::npos);
  std::filesystem::remove(path);
}

TEST(config_rejects_removed_power_keys) {
  // Each removed key must fail boot loudly. Reverting the deletion from
  // check_known_keys() in drone/src/config.cpp makes these keys parse again
  // and this test fails.
  for (const char* key : {"thermal_max_delta", "min_offset_qdb",
                          "power_offset_qdb"}) {
    std::string toml = std::string("[radio]\n") + key + " = 1\n";
    auto path = write_temp_toml(toml);
    bool threw = false;
    try {
      load_config(path.string());
    } catch (const std::runtime_error& e) {
      threw = true;
      CHECK(std::string(e.what()).find("unknown key") != std::string::npos);
    }
    CHECK(threw);
    std::filesystem::remove(path);
  }
}

TEST(config_rejects_power_mode_override) {
  // Reverting the removal of "override" from the accepted set in
  // parse_radio() makes this load successfully and the test fails.
  auto path = write_temp_toml("[radio]\npower_mode = \"override\"\n");
  bool threw = false;
  try {
    load_config(path.string());
  } catch (const std::runtime_error& e) {
    threw = true;
    CHECK(std::string(e.what()).find("power_mode") != std::string::npos);
  }
  CHECK(threw);
  std::filesystem::remove(path);
}

TEST(link_rc_drain_ms_default_and_bounds) {
  // Absent: the agent loop wakes every 5 ms to drain RCFs (spec 2026-08-14
  // fade-demote §3b). This is an optional key on a strict-keys config, so a
  // deployed drone with no `link.rc_drain_ms` must still boot.
  {
    auto path = write_temp_toml("");
    auto cfg = load_config(path.string());
    CHECK(cfg.link.rc_drain_ms == 5);
    std::filesystem::remove(path);
  }
  // Explicit value inside the range is taken verbatim (50 <= the default
  // 100 ms tick_ms, so the cross-check below is satisfied).
  {
    auto path = write_temp_toml("[link]\nrc_drain_ms = 50\n");
    auto cfg = load_config(path.string());
    CHECK(cfg.link.rc_drain_ms == 50);
    std::filesystem::remove(path);
  }
  // Out of range fails boot, naming the field. 0 would spin the agent
  // thread; > 1000 would make actuation slower than the legacy loop.
  for (int bad : {0, 1001}) {
    auto path = write_temp_toml(std::string("[link]\nrc_drain_ms = ") +
                                std::to_string(bad) + "\n");
    std::string msg = what_of([&] { (void)load_config(path.string()); });
    CHECK(!msg.empty());
    CHECK(msg.find("link.rc_drain_ms") != std::string::npos);
    std::filesystem::remove(path);
  }
}

// Review finding 2026-08-14 (final whole-branch review, finding 4): tick_ms
// was unvalidated, and the TickGate the agent loop now runs its housekeeping
// behind turns a bad value from "spins hot" into "silently loses the
// failsafe" — TickGate(now, -1) casts to a ~1.8e19 ms period, so the gate
// fires once at startup and never again: no failsafe transition, no
// rendezvous fallback, no watchdog, no telemetry, and nothing in the log to
// say it stopped.
TEST(link_tick_ms_bounds) {
  // Absent: the historical 100 ms housekeeping cadence.
  {
    auto path = write_temp_toml("");
    auto cfg = load_config(path.string());
    CHECK(cfg.link.tick_ms == 100);
    std::filesystem::remove(path);
  }
  // In range, taken verbatim.
  {
    auto path = write_temp_toml("[link]\ntick_ms = 20\n");
    auto cfg = load_config(path.string());
    CHECK(cfg.link.tick_ms == 20);
    std::filesystem::remove(path);
  }
  // Out of range fails boot, naming the field.
  for (int bad : {-1, 0, 1001}) {
    auto path = write_temp_toml(std::string("[link]\ntick_ms = ") +
                                std::to_string(bad) + "\nrc_drain_ms = 1\n");
    std::string msg = what_of([&] { (void)load_config(path.string()); });
    CHECK(!msg.empty());
    CHECK(msg.find("link.tick_ms") != std::string::npos);
    std::filesystem::remove(path);
  }
}

TEST(link_rc_drain_ms_must_not_exceed_tick_ms) {
  // rc_drain_ms is the loop's WAKE period and tick_ms the housekeeping
  // deadline behind it; a drain slower than the tick silently retimes every
  // per-tick job to rc_drain_ms instead (TickGate degenerates to firing on
  // every wake). Equality is legal — that is exactly the legacy loop.
  {
    auto path = write_temp_toml("[link]\ntick_ms = 50\nrc_drain_ms = 50\n");
    auto cfg = load_config(path.string());
    CHECK(cfg.link.rc_drain_ms == 50);
    CHECK(cfg.link.tick_ms == 50);
    std::filesystem::remove(path);
  }
  {
    auto path = write_temp_toml("[link]\ntick_ms = 50\nrc_drain_ms = 51\n");
    std::string msg = what_of([&] { (void)load_config(path.string()); });
    CHECK(!msg.empty());
    CHECK(msg.find("link.rc_drain_ms") != std::string::npos);
    std::filesystem::remove(path);
  }
}

// ---- Task 3: ampdu block (spec 2026-09-01-ampdu-design.md) --------------

TEST(ampdu_defaults_when_absent) {
  // A config with no "ampdu" block gets the shipped defaults — aggregation
  // OFF since the 2026-09-01 bench verdict (no fec win, RF-report damage).
  auto path = write_temp_toml("");
  auto cfg = load_config(path.string());
  CHECK(cfg.ampdu.max_num == 0);
  CHECK(cfg.ampdu.max_time == 32);
  std::filesystem::remove(path);
}

TEST(ampdu_block_parses) {
  auto path = write_temp_toml("[ampdu]\nmax_num = 4\nmax_time = 48\n");
  auto cfg = load_config(path.string());
  CHECK(cfg.ampdu.max_num == 4);
  CHECK(cfg.ampdu.max_time == 48);
  std::filesystem::remove(path);
}

TEST(ampdu_zero_disables) {
  auto path = write_temp_toml("[ampdu]\nmax_num = 0\n");
  auto cfg = load_config(path.string());
  CHECK(cfg.ampdu.max_num == 0);
  std::filesystem::remove(path);
}

TEST(ampdu_rejects_bad_values) {
  // max_num out of the 5-bit MAX_AGG_NUM field.
  {
    auto path = write_temp_toml("[ampdu]\nmax_num = 32\n");
    std::string msg = what_of([&] { (void)load_config(path.string()); });
    CHECK(!msg.empty());
    CHECK(msg.find("ampdu.max_num") != std::string::npos);
    std::filesystem::remove(path);
  }
  {
    auto path = write_temp_toml("[ampdu]\nmax_num = -1\n");
    std::string msg = what_of([&] { (void)load_config(path.string()); });
    CHECK(!msg.empty());
    CHECK(msg.find("ampdu.max_num") != std::string::npos);
    std::filesystem::remove(path);
  }
  // max_time 1..8 is the register cliff (aggregation silently disabled).
  {
    auto path = write_temp_toml("[ampdu]\nmax_time = 8\n");
    std::string msg = what_of([&] { (void)load_config(path.string()); });
    CHECK(!msg.empty());
    CHECK(msg.find("ampdu.max_time") != std::string::npos);
    std::filesystem::remove(path);
  }
  {
    auto path = write_temp_toml("[ampdu]\nmax_time = 256\n");
    std::string msg = what_of([&] { (void)load_config(path.string()); });
    CHECK(!msg.empty());
    CHECK(msg.find("ampdu.max_time") != std::string::npos);
    std::filesystem::remove(path);
  }
  // Unknown key inside the block fails boot (config-strict).
  {
    auto path = write_temp_toml("[ampdu]\ndepth = 4\n");
    std::string msg = what_of([&] { (void)load_config(path.string()); });
    CHECK(!msg.empty());
    CHECK(msg.find("ampdu.depth") != std::string::npos);
    CHECK(msg.find("unknown key") != std::string::npos);
    std::filesystem::remove(path);
  }
}

// air_clock (spec 2026-09-06): shed_ms 0 = observe only; efficiency is the
// fraction of nominal PHY rate the model treats as capacity; body_us a
// fixed per-body cost. All three are bench calibration knobs.
TEST(air_clock_defaults_when_absent) {
  auto path = write_temp_toml("[link]\ntick_ms = 100\n");
  Config c = load_config(path.string());
  CHECK(c.air_clock.shed_ms == 0);
  CHECK(c.air_clock.efficiency == 0.7);
  CHECK(c.air_clock.body_us == 0);
  std::filesystem::remove(path);
}

TEST(air_clock_section_parses) {
  auto path = write_temp_toml(
      "[air_clock]\nshed_ms = 25\nefficiency = 0.65\nbody_us = 40\n");
  Config c = load_config(path.string());
  CHECK(c.air_clock.shed_ms == 25);
  CHECK(c.air_clock.efficiency == 0.65);
  CHECK(c.air_clock.body_us == 40);
  std::filesystem::remove(path);
}

TEST(air_clock_range_checks_name_the_key) {
  struct Case { const char* toml; const char* key; };
  const Case cases[] = {
      {"[air_clock]\nshed_ms = -1\n", "air_clock.shed_ms"},
      {"[air_clock]\nshed_ms = 60001\n", "air_clock.shed_ms"},
      {"[air_clock]\nefficiency = 0\n", "air_clock.efficiency"},
      {"[air_clock]\nefficiency = 1.5\n", "air_clock.efficiency"},
      {"[air_clock]\nbody_us = -5\n", "air_clock.body_us"},
  };
  for (const auto& k : cases) {
    auto path = write_temp_toml(k.toml);
    std::string msg = what_of([&] { (void)load_config(path.string()); });
    CHECK(msg.find(k.key) != std::string::npos);
    std::filesystem::remove(path);
  }
}

TEST(air_clock_unknown_key_throws) {
  auto path = write_temp_toml("[air_clock]\nwindow_ms = 500\n");
  std::string msg = what_of([&] { (void)load_config(path.string()); });
  CHECK(msg.find("window_ms") != std::string::npos);
  CHECK(msg.find("unknown key") != std::string::npos);
  std::filesystem::remove(path);
}

// ---- Task 3: TOML swap (2026-09-07-toml-config) -------------------------

TEST(load_config_reports_defaulted_keys) {
  auto path = write_temp_toml("[fec]\nwindow = 16\n");
  std::vector<std::string> defaulted;
  Config cfg = load_config(path.string(), &defaulted);
  CHECK(cfg.fec.window == 16);
  // Present keys are not reported; absent known keys are, with their value.
  bool saw_window = false, saw_flush = false;
  for (const std::string& d : defaulted) {
    if (d.rfind("fec.window", 0) == 0) saw_window = true;
    if (d == "fec.flush_ms=15") saw_flush = true;
  }
  CHECK(!saw_window);
  CHECK(saw_flush);
  std::filesystem::remove(path);
}

TEST(load_config_errors_carry_file_and_line) {
  auto path = write_temp_toml("[radio]\nchannel = 149\nwidth = 20\n"
                              "power_mode = \"bogus\"\n");
  const std::string msg = what_of([&] { load_config(path.string()); });
  CHECK(msg.find("radio.power_mode") != std::string::npos);
  CHECK(msg.find(".toml:4:") != std::string::npos);
  std::filesystem::remove(path);
}

TEST(load_config_rejects_float_for_an_int_key) {
  auto path = write_temp_toml("[fec]\nsymbol_size = 332.0\n");
  const std::string msg = what_of([&] { load_config(path.string()); });
  CHECK(msg.find("fec.symbol_size") != std::string::npos);
  std::filesystem::remove(path);
}

TEST(load_config_accepts_int_for_a_float_key) {
  auto path = write_temp_toml("[encoder]\nairtime_budget = 1\n");
  Config cfg = load_config(path.string());
  CHECK(cfg.encoder.airtime_budget == 1.0);
  std::filesystem::remove(path);
}

// Fix round 1 finding: parse_venc wraps every key's assign_if_present in an
// outer `if (j.contains(key))`, so the built-in absent-branch reporting
// never fires there -- the [venc] section was silently missing from the
// operator-facing defaulted-key log. Pins that every venc key still gets
// reported when absent, with the REAL compiled default (parse_venc's local
// kDef), not the zero-init sentinel its validation temps start from.
TEST(load_config_reports_real_venc_defaults_not_zero) {
  // Only the one required key present; every other venc key (and the whole
  // [venc.roi] sub-table) is absent.
  auto path = write_temp_toml(
      "[venc]\nsensor_bin = \"/etc/sensors/imx415_greg_fpvXIX_colortrans.bin\"\n");
  std::vector<std::string> defaulted;
  Config cfg = load_config(path.string(), &defaulted);
  CHECK(cfg.venc.core.fps == 60);

  bool saw_fps = false, saw_qp_delta = false, saw_roi_absent = false;
  for (const std::string& d : defaulted) {
    if (d == "venc.fps=60") saw_fps = true;
    if (d == "venc.qp_delta=-4") saw_qp_delta = true;
    if (d == "venc.roi=(section absent)") saw_roi_absent = true;
    // These keys' real compiled defaults are all nonzero (venc_cfg.c); a
    // bare "=0" here would be exactly the lie a naive `else
    // note_default(key, to_text(local_temp))` would have produced.
    for (const char* wrong : {"venc.fps=0", "venc.qp_delta=0",
                              "venc.intra_refresh_rows=0",
                              "venc.intra_refresh_qp=0", "venc.ref_base=0",
                              "venc.ref_enhance=0", "venc.ae_fps=0",
                              "venc.awb_fps=0", "venc.snapshot_quality=0",
                              "venc.debug_port=0", "venc.roi.steps=0"}) {
      CHECK(d != wrong);
    }
  }
  CHECK(saw_fps);
  CHECK(saw_qp_delta);
  CHECK(saw_roi_absent);
  std::filesystem::remove(path);
}


MTEST_MAIN
