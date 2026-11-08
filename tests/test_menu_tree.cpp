// build_menu_tree: the GS rows of the goggles settings menu -- what each row
// reads, offers and sends -- over the bundle configs of both daemons. The
// values a row sends are pinned through the daemons' own setters too, so a
// row cannot send text the live key would refuse.
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "config.h"            // maburgs::Config, maburgs::load_config
#include "gs_config_apply.h"   // maburgs::patch_live_key, kGsLiveKeys
#include "gs_live_view.h"
#include "gs_snapshot.h"
#include "mabur/config_params.h"
#include "menu_model.h"
#include "menu_tree.h"
#include "mode_pick.h"
#include "mtest.h"
#include "player_config.h"
#include "player_config_apply.h"
#include "scratch.h"

using maburplay::build_menu_tree;
using maburplay::Choice;
using maburplay::drone_header;
using maburplay::drone_pill_mark;
using maburplay::GsLiveView;
using maburplay::GsSnapshot;
using maburplay::MenuNode;
using maburplay::MenuTreeDeps;
using maburplay::ModeCand;
using maburplay::NodeKind;
using maburplay::Owner;
using maburplay::Rung;
using maburplay::SetStep;
using maburplay::VtxRow;
using maburplay::VtxState;

namespace {

const std::string kPlayBundle = MABUR_PLAY_BUNDLE_DIR "/maburplay.default.toml";
const std::string kGsBundle = MABUR_SOURCE_DIR "/gs/bundle/maburgs.default.toml";

std::string slurp(const std::string& path) {
  std::ifstream f(path, std::ios::binary);
  std::ostringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

void spit(const std::string& path, const std::string& content) {
  std::ofstream f(path, std::ios::binary);
  f << content;
}

// maburgs' live keys as its own `get` dump of the bundle reads them.
GsLiveView bundle_gs() {
  const maburgs::Config c = maburgs::load_config(kGsBundle);
  std::string dump = "ok\n";
  for (const maburgs::LiveKey& k : maburgs::kGsLiveKeys)
    dump += std::string(k.key) + " = " + maburgs::render_live_value(c, k) + "\n";
  GsLiveView v;
  REQUIRE(maburplay::parse_get_all(dump, &v, 1));
  return v;
}

// CEA 1080p60 (preferred), 720p60, 1080p100, 720p100: a goggle with 60 and
// 100 Hz at two sizes.
const ModeCand k1080p60{1920, 1080, 148500, 2200, 1125, false, true};
const ModeCand k720p60{1280, 720, 74250, 1650, 750, false, false};
const ModeCand k1080p100{1920, 1080, 297000, 2640, 1125, false, false};
const ModeCand k720p100{1280, 720, 148500, 1980, 750, false, false};
const std::vector<ModeCand> kAvatar{k1080p60, k720p60, k1080p100, k720p100};

// A display offering only 75 Hz: none of the fixed rates (60/90/100/120).
const ModeCand k720p75{1280, 720, 74250, 1320, 750, false, true};
const std::vector<ModeCand> kOnly75{k720p75};

struct Fx {
  maburplay::Config cfg = maburplay::load_config(kPlayBundle);
  GsLiveView gs = bundle_gs();
  GsSnapshot snap;
  bool rec = false, burned = false;
  std::vector<ModeCand> modes;
  double cam = 0;
  std::string running;
  std::function<bool()> refresh_sync_available;

  // Fills snap.vtx (creating it linked, list "complete") with one row,
  // updating it in place when `id` is already there.
  void vtx_set(const char* id, const std::string& value, const std::string& st = "idle") {
    if (!snap.vtx) {
      snap.vtx = VtxState{};
      snap.vtx->linked = true;
      snap.vtx->list = "complete";
    }
    for (VtxRow& r : snap.vtx->rows) {
      if (r.id == id) {
        r.value = value;
        r.st = st;
        return;
      }
    }
    VtxRow row;
    row.id = id;
    row.value = value;
    row.st = st;
    snap.vtx->rows.push_back(row);
  }

  MenuTreeDeps deps() {
    MenuTreeDeps d;
    d.cfg = &cfg;
    d.gs = &gs;
    d.snap = &snap;
    d.rec_running = [this] { return rec; };
    d.burned_rec_running = [this] { return burned; };
    d.modes = [this] { return modes; };
    d.camera_hz = [this] { return cam; };
    d.running_mode = [this] { return running; };
    d.refresh_sync_available = refresh_sync_available;
    return d;
  }
  std::vector<MenuNode> tree() { return build_menu_tree(deps()); }

  // The node with `id` anywhere in the tree, visible or not.
  std::optional<MenuNode> find(const std::vector<MenuNode>& nodes, const std::string& id) {
    for (const MenuNode& n : nodes) {
      if (n.id == id) return n;
      if (n.kind == NodeKind::kGroup && n.children)
        if (auto f = find(n.children(), id)) return f;
    }
    return std::nullopt;
  }
  MenuNode node(const std::string& id) {
    auto n = find(tree(), id);
    if (!n) std::printf("no node %s\n", id.c_str());
    REQUIRE(n.has_value());
    return *n;
  }
};

std::vector<std::string> values(const std::vector<Choice>& cs) {
  std::vector<std::string> v;
  for (const Choice& c : cs) v.push_back(c.value);
  return v;
}
std::vector<std::string> labels(const std::vector<Choice>& cs) {
  std::vector<std::string> v;
  for (const Choice& c : cs) v.push_back(c.label);
  return v;
}
bool vis(const MenuNode& n) { return !n.visible || n.visible(); }
// What the model asks before sending `v`: a null confirm asks nothing.
std::string confirm(const MenuNode& n, const std::string& v) { return n.confirm ? n.confirm(v) : ""; }

const std::string kSplit = "Splits the recording (a new file starts).";

}  // namespace

TEST(the_top_level_groups_are_video_link_recording_osd_system) {
  Fx f;
  const auto t = f.tree();
  std::vector<std::string> got;
  for (const MenuNode& n : t) {
    CHECK(n.kind == NodeKind::kGroup);
    got.push_back(n.label);
  }
  REQUIRE((got ==
           std::vector<std::string>{"Video", "Link", "Recording", "OSD", "Power & FC", "System"}));
  // No Channel row and no Calibration group yet.
  for (const MenuNode& n : t[1].children()) {
    CHECK(n.label != "Channel");
    CHECK(n.label != "Calibration");
  }
}

TEST(top_level_groups_include_power_and_fc) {
  Fx f;
  std::vector<std::string> ids;
  for (const MenuNode& n : f.tree()) ids.push_back(n.id);
  CHECK((ids == std::vector<std::string>{"video", "link", "rec", "osd", "power", "sys"}));
}

TEST(rate_fixed_sends_four_steps_with_static_mcs_last) {
  Fx f;
  REQUIRE(f.gs.static_mcs == -1);
  REQUIRE(f.gs.ladder.size() == 6);
  const MenuNode rate = f.node("link.rate");
  CHECK(rate.current() == "adaptive");
  // The current rung: R3 of the bundle ladder, MCS3 at 20 MHz, 0.5/0.25.
  CHECK((rate.steps("fixed") == std::vector<SetStep>{
             {Owner::kGs, "link.static_bw", "20"},
             {Owner::kGs, "link.static_overhead_base", "0.5"},
             {Owner::kGs, "link.static_overhead_enh", "0.25"},
             {Owner::kGs, "link.static_mcs", "3"}}));
  CHECK(confirm(rate, "fixed").empty());
  // A ladder shorter than four rungs pins its last one.
  f.gs.ladder = {{0, 20, 0.6, 0.3}, {2, 20, 1.0, 0.5}};
  CHECK((f.node("link.rate").steps("fixed") == std::vector<SetStep>{
             {Owner::kGs, "link.static_bw", "20"},
             {Owner::kGs, "link.static_overhead_base", "1"},
             {Owner::kGs, "link.static_overhead_enh", "0.5"},
             {Owner::kGs, "link.static_mcs", "2"}}));
  // The fixed sub-rows show only under Fixed.
  CHECK(!vis(f.node("link.mcs")));
  f.gs.static_mcs = 2;
  CHECK(f.node("link.rate").current() == "fixed");
  CHECK(vis(f.node("link.mcs")));
  CHECK(vis(f.node("link.fec_base")));
  CHECK(vis(f.node("link.fec_enh")));
  CHECK(!vis(f.node("link.ladder")));
}

TEST(rate_adaptive_is_one_step) {
  Fx f;
  f.gs.static_mcs = 3;
  const MenuNode rate = f.node("link.rate");
  CHECK((rate.steps("adaptive") == std::vector<SetStep>{{Owner::kGs, "link.static_mcs", "-1"}}));
  CHECK(confirm(rate, "adaptive").empty());
}

// Rate flips between the ladder's width and the pin's: the confirm says so
// when the two differ, under the Width row's rule.
TEST(rate_confirms_a_receiver_retune_when_the_width_changes) {
  Fx f;
  // Adaptive over a 40 MHz ladder; Fixed pins the live rung, MCS2 at 20.
  f.gs.ladder = {{0, 40, 0.5, 0.25}, {1, 40, 0.5, 0.25}, {2, 20, 0.5, 0.25}};
  f.snap.mcs = 2;
  f.snap.bw = 20;
  CHECK(confirm(f.node("link.rate"), "fixed") == "Retunes the receiver.");
  f.snap.bw = 40;
  f.snap.mcs = 1;
  CHECK(confirm(f.node("link.rate"), "fixed").empty());          // pins a 40 rung: no flip
  // Fixed at 20 back to an adaptive ladder with a 40 rung.
  f.gs.static_mcs = 2;
  f.gs.static_bw = 20;
  CHECK(confirm(f.node("link.rate"), "adaptive") == "Retunes the receiver.");
  f.gs.static_bw = 40;
  CHECK(confirm(f.node("link.rate"), "adaptive").empty());
  // No ladder read: Fixed sends nothing, so nothing retunes.
  f.gs.static_mcs = -1;
  f.gs.ladder.clear();
  CHECK(confirm(f.node("link.rate"), "fixed").empty());
}

TEST(width_pick_sends_the_default_ladder_and_confirms_when_the_ladder_is_custom) {
  Fx f;
  const std::string l40 = maburplay::render_ladder_value(maburplay::default_ladder(40));
  MenuNode w = f.node("link.width");
  CHECK(w.value_text() == "20 MHz");
  CHECK(w.current() == "20");
  CHECK((w.steps("40") == std::vector<SetStep>{{Owner::kGs, "link.ladder", l40}}));
  // Fx: bundle ladder (all 20), adaptive -- picking 40 flips the receiver.
  CHECK(confirm(w, "40") == "Retunes the receiver.");
  CHECK(confirm(w, "20").empty());
  f.gs.ladder[0].base = 0.6;         // custom ladder
  w = f.node("link.width");
  CHECK(confirm(w, "20") == "Replaces your ladder with the default 20 MHz ladder (6 rungs).");
  CHECK(confirm(w, "40") ==
        "Replaces your ladder with the default 40 MHz ladder (5 rungs). Retunes the receiver.");
  // Under Fixed the row is the fixed rate's width: one static_bw set.
  f.gs.static_mcs = 3;
  f.gs.static_bw = 40;
  w = f.node("link.width");
  CHECK(w.value_text() == "40 MHz");
  CHECK((w.steps("20") == std::vector<SetStep>{{Owner::kGs, "link.static_bw", "20"}}));
  CHECK(confirm(w, "20") == "Retunes the receiver.");
  CHECK(confirm(w, "40").empty());
}

TEST(width_reads_mixed_and_offers_only_20_and_40) {
  Fx f;
  f.gs.ladder[0].bw = 40;
  const MenuNode w = f.node("link.width");
  CHECK(w.value_text() == "Mixed");
  CHECK((values(w.choices()) == std::vector<std::string>{"40", "20"}));
  CHECK((labels(w.choices()) == std::vector<std::string>{"40 MHz", "20 MHz"}));
  CHECK(w.current() != "40" && w.current() != "20");   // the picker opens unmarked
  CHECK(w.help().find("mixes 20 and 40 MHz") != std::string::npos);
}

TEST(a_rung_edit_sends_the_whole_ladder_with_one_change) {
  Fx f;
  f.gs.ladder = maburplay::default_ladder(40);
  const MenuNode ladder = f.node("link.ladder");
  CHECK(ladder.advanced);
  CHECK(ladder.group_value() == "5 rungs");
  const MenuNode r2 = f.node("link.ladder.r2");
  CHECK(r2.label == "R2");
  CHECK(r2.group_value() == "MCS2 · 40 MHz · 0.50/0.25");

  auto with = [&](int i, Rung r) {
    std::vector<Rung> l = f.gs.ladder;
    l[static_cast<size_t>(i)] = r;
    return std::vector<SetStep>{{Owner::kGs, "link.ladder", maburplay::render_ladder_value(l)}};
  };
  CHECK(f.node("link.ladder.r2.mcs").steps("4") == with(2, {4, 40, 0.5, 0.25}));
  CHECK(f.node("link.ladder.r2.bw").steps("20") == with(2, {2, 20, 0.5, 0.25}));
  CHECK(f.node("link.ladder.r2.enh").steps("0.4") == with(2, {2, 40, 0.5, 0.4}));
  // Base below enh drags enh down with it: the loader refuses base < enh.
  CHECK(f.node("link.ladder.r2.base").steps("0.2") == with(2, {2, 40, 0.2, 0.2}));
  CHECK(f.node("link.ladder.r2.base").steps("1") == with(2, {2, 40, 1.0, 0.25}));

  // Move up / down swap; delete asks first and is gone for a lone rung.
  std::vector<Rung> up = f.gs.ladder;
  std::swap(up[1], up[2]);
  CHECK((f.node("link.ladder.r2.up").action() ==
         std::vector<SetStep>{{Owner::kGs, "link.ladder", maburplay::render_ladder_value(up)}}));
  CHECK(!vis(f.node("link.ladder.r0.up")));
  CHECK(!vis(f.node("link.ladder.r4.down")));
  const MenuNode del = f.node("link.ladder.r2.del");
  CHECK(confirm(del, "") == "Delete this rung?");
  std::vector<Rung> gone = f.gs.ladder;
  gone.erase(gone.begin() + 2);
  CHECK((del.action() ==
         std::vector<SetStep>{{Owner::kGs, "link.ladder", maburplay::render_ladder_value(gone)}}));
  // + Add rung copies the last with MCS + 1, capped at 5, up to 8 rungs.
  std::vector<Rung> more = f.gs.ladder;
  more.push_back({5, 40, 0.5, 0.25});
  CHECK((f.node("link.ladder.add").action() ==
         std::vector<SetStep>{{Owner::kGs, "link.ladder", maburplay::render_ladder_value(more)}}));
  f.gs.ladder = more;
  more.push_back({5, 40, 0.5, 0.25});
  CHECK((f.node("link.ladder.add").action() ==
         std::vector<SetStep>{{Owner::kGs, "link.ladder", maburplay::render_ladder_value(more)}}));
  f.gs.ladder.resize(8, Rung{5, 40, 0.5, 0.25});
  CHECK(!vis(f.node("link.ladder.add")));
  f.gs.ladder = {{0, 40, 0.5, 0.25}};
  CHECK(!vis(f.node("link.ladder.r0.del")));
}

TEST(fixed_pins_the_live_rung_from_the_sideport) {
  Fx f;
  // The bundle ladder is six rungs MCS0..5, all 20 MHz, 0.5/0.25: a live
  // rung of MCS2/20 is not the index rule's R3.
  f.snap.mcs = 2;
  f.snap.bw = 20;
  CHECK((f.node("link.rate").steps("fixed") == std::vector<SetStep>{
             {Owner::kGs, "link.static_bw", "20"},
             {Owner::kGs, "link.static_overhead_base", "0.5"},
             {Owner::kGs, "link.static_overhead_enh", "0.25"},
             {Owner::kGs, "link.static_mcs", "2"}}));
  // No live rung (mcs unknown): the existing index rule, R3.
  f.snap.mcs = std::nullopt;
  CHECK((f.node("link.rate").steps("fixed") == std::vector<SetStep>{
             {Owner::kGs, "link.static_bw", "20"},
             {Owner::kGs, "link.static_overhead_base", "0.5"},
             {Owner::kGs, "link.static_overhead_enh", "0.25"},
             {Owner::kGs, "link.static_mcs", "3"}}));
}

TEST(max_bitrate_edits_megabits_and_sends_kbps) {
  Fx f;
  f.vtx_set("ENC_BITRATE_MAX", "24000");
  MenuNode n = f.node("link.bitrate");
  CHECK(!n.advanced);
  CHECK(n.number.get() == 24);
  CHECK(n.number.min == 1 && n.number.max == 100 && n.number.step == 1);
  CHECK(n.number.fmt(24) == "24 Mb/s");
  CHECK((n.steps("30") == std::vector<SetStep>{{Owner::kDrone, "ENC_BITRATE_MAX", "30000"}}));
  CHECK(confirm(n, "30").empty());
}

TEST(airtime_edits_percent_and_sends_a_fraction) {
  Fx f;
  f.vtx_set("ENC_AIRTIME", "0.6");
  MenuNode n = f.node("link.airtime");
  CHECK(n.advanced);
  CHECK(n.number.get() == 60);
  CHECK(n.number.min == 5 && n.number.max == 100 && n.number.step == 5);
  CHECK(n.number.fmt(60) == "60 %");
  CHECK((n.steps("75") == std::vector<SetStep>{{Owner::kDrone, "ENC_AIRTIME", "0.75"}}));
}

TEST(ldpc_keeps_a_warning_after_ok) {
  Fx f;
  f.vtx_set("RADIO_LDPC", "0", "accepted");
  for (VtxRow& r : f.snap.vtx->rows)
    if (r.id == "RADIO_LDPC") r.status = "walls measured with LDPC on";
  MenuNode n = f.node("link.ldpc");
  CHECK(n.advanced);
  REQUIRE(n.warn_after_ok != nullptr);
  CHECK(n.warn_after_ok() == "walls measured with LDPC on");
  for (VtxRow& r : f.snap.vtx->rows)
    if (r.id == "RADIO_LDPC") r.status = "";
  CHECK(f.node("link.ldpc").warn_after_ok().empty());
}

TEST(fc_osd_on_is_player_then_gs_and_off_is_gs_then_player) {
  Fx f;
  const MenuNode fc = f.node("osd.fc");
  CHECK((fc.steps("true") == std::vector<SetStep>{{Owner::kPlayer, "osd.enable", "true"},
                                                   {Owner::kGs, "msp.enable", "true"}}));
  CHECK((fc.steps("false") == std::vector<SetStep>{{Owner::kGs, "msp.enable", "false"},
                                                    {Owner::kPlayer, "osd.enable", "false"}}));
  // The row reads both halves.
  f.cfg.osd.enable = true;
  f.gs.msp = true;
  CHECK(f.node("osd.fc").value_text() == "On");
  f.cfg.osd.enable = false;
  f.gs.msp = false;
  CHECK(f.node("osd.fc").value_text() == "Off");
  f.gs.msp = true;
  CHECK(f.node("osd.fc").value_text() == "mixed");
  CHECK(f.node("osd.fc").help().find("feed is on") != std::string::npos);
  f.gs.msp = false;
  f.cfg.osd.enable = true;
  CHECK(f.node("osd.fc").help().find("render is on") != std::string::npos);
}

TEST(fc_osd_warns_when_the_drones_fc_link_is_off) {
  Fx f;
  f.vtx_set("MSP_ENABLE", "0");
  CHECK(f.node("osd.fc").warn() == "FC link off");
  f.vtx_set("MSP_ENABLE", "1");
  CHECK(f.node("osd.fc").warn().empty());
}

TEST(low_power_on_turns_the_fc_link_on_first) {
  Fx f;
  f.vtx_set("MSP_ENABLE", "0");
  MenuNode n = f.node("power.lowpwr");
  CHECK((n.steps("true") == std::vector<SetStep>{{Owner::kDrone, "MSP_ENABLE", "1"},
                                                  {Owner::kDrone, "LOWPWR_ENABLE", "1"}}));
  CHECK(confirm(n, "true") == "Also turns the FC link on.");
  f.vtx_set("MSP_ENABLE", "1");
  n = f.node("power.lowpwr");
  CHECK((n.steps("true") == std::vector<SetStep>{{Owner::kDrone, "LOWPWR_ENABLE", "1"}}));
  CHECK(confirm(n, "true").empty());
  CHECK((n.steps("false") == std::vector<SetStep>{{Owner::kDrone, "LOWPWR_ENABLE", "0"}}));
  CHECK(confirm(n, "false").empty());
}

TEST(fc_link_off_turns_low_power_off_first) {
  Fx f;
  f.vtx_set("LOWPWR_ENABLE", "1");
  MenuNode n = f.node("power.msp");
  CHECK((n.steps("false") == std::vector<SetStep>{{Owner::kDrone, "LOWPWR_ENABLE", "0"},
                                                   {Owner::kDrone, "MSP_ENABLE", "0"}}));
  CHECK(confirm(n, "false") == "Also turns Low power off.");
  f.vtx_set("LOWPWR_ENABLE", "0");
  n = f.node("power.msp");
  CHECK((n.steps("false") == std::vector<SetStep>{{Owner::kDrone, "MSP_ENABLE", "0"}}));
  CHECK(confirm(n, "false").empty());
}

TEST(dvr_rows_say_at_next_recording_while_recording) {
  Fx f;
  f.cfg.dvr.target = "gs";
  f.cfg.dvr.mode = "burned";
  for (const char* id : {"rec.target", "rec.mode", "rec.burn.bitrate", "rec.burn.every", "rec.burn.size"}) {
    f.rec = false;
    MenuNode n = f.node(id);
    REQUIRE(n.saved_text != nullptr);
    CHECK(n.saved_text() == "✓ saved");
    f.rec = true;
    CHECK(f.node(id).saved_text() == "✓ at next recording");
  }
  // The values the dvr rows send, as maburplay's setter takes them.
  CHECK((f.node("rec.target").steps("both") ==
         std::vector<SetStep>{{Owner::kPlayer, "dvr.target", "\"both\""}}));
  CHECK((f.node("rec.burn.size").steps("") ==
         std::vector<SetStep>{{Owner::kPlayer, "dvr.burned.size", "\"\""}}));
  CHECK((f.node("rec.burn.bitrate").steps("9000") ==
         std::vector<SetStep>{{Owner::kPlayer, "dvr.burned.bitrate_kbps", "9000"}}));
  CHECK(f.node("rec.burn.bitrate").number.fmt(8000) == "8 Mb/s");
  // Frame rate labels follow the running mode's rate.
  f.running = "1920x1080@100.000";
  const auto every = f.node("rec.burn.every").choices();
  REQUIRE(every.size() >= 2);
  CHECK(every[0].value == "1" && every[0].label == "100 fps (every frame)");
  CHECK(every[1].value == "2" && every[1].label == "50 fps (every 2nd)");
  // GS recording hides when REC records to the drone only; the Burned rows
  // hide under Raw.
  f.cfg.dvr.target = "vtx";
  CHECK(!vis(f.node("rec.mode")));
  CHECK(!vis(f.node("rec.burn.size")));
  f.cfg.dvr.target = "both";
  f.cfg.dvr.mode = "raw";
  CHECK(vis(f.node("rec.mode")));
  CHECK(!vis(f.node("rec.burn.size")));
}

TEST(sd_recorder_rows_follow_the_drones_rec_enable) {
  Fx f;
  f.vtx_set("REC_ENABLE", "0");
  CHECK(f.node("rec.vtx").current() == "false");
  CHECK(!vis(f.node("rec.vtx.bitrate")));
  CHECK(!vis(f.node("rec.vtx.fps")));
  CHECK(!vis(f.node("rec.vtx.size")));
  f.vtx_set("REC_ENABLE", "1");
  CHECK(f.node("rec.vtx").current() == "true");
  CHECK(vis(f.node("rec.vtx.bitrate")));
  CHECK(vis(f.node("rec.vtx.fps")));
  CHECK(vis(f.node("rec.vtx.size")));
  CHECK(confirm(f.node("rec.vtx"), "true") == "Video pauses about half a second.");
  CHECK(confirm(f.node("rec.vtx.bitrate"), "10") == "Video pauses about half a second.");
  CHECK(confirm(f.node("rec.vtx.fps"), "60") == "Video pauses about half a second.");
  CHECK(confirm(f.node("rec.vtx.size"), "1280x720") == "Video pauses about half a second.");
}

TEST(sd_frame_rate_offers_the_cameras_divisors_from_30) {
  Fx f;
  f.vtx_set("VENC_MODE", "100@1280x720/binned");
  CHECK((values(f.node("rec.vtx.fps").choices()) == std::vector<std::string>{"100", "50"}));
  f.vtx_set("VENC_MODE", "60@1920x1080/native");
  CHECK((values(f.node("rec.vtx.fps").choices()) == std::vector<std::string>{"60", "30"}));
  f.vtx_set("VENC_MODE", "90@1728x972/binned");
  CHECK((values(f.node("rec.vtx.fps").choices()) == std::vector<std::string>{"90", "45", "30"}));
  f.vtx_set("VENC_MODE", "120@1728x816/binned");
  CHECK((values(f.node("rec.vtx.fps").choices()) ==
         std::vector<std::string>{"120", "60", "40", "30"}));
  f.vtx_set("REC_FPS", "25");
  f.vtx_set("VENC_MODE", "100@1280x720/binned");
  const auto cs = f.node("rec.vtx.fps").choices();
  CHECK((values(cs) == std::vector<std::string>{"25", "100", "50"}));
  REQUIRE(!cs.empty());
  CHECK(cs[0].label == "25 fps");
}

TEST(sd_size_offers_the_cameras_sensor_mode_sizes) {
  Fx f;
  f.vtx_set("VENC_MODE", "100@1280x720/binned");
  CHECK((values(f.node("rec.vtx.size").choices()) ==
         std::vector<std::string>{"1728x972", "1280x720"}));
  f.vtx_set("VENC_MODE", "60@1920x1080/native");
  CHECK((values(f.node("rec.vtx.size").choices()) ==
         std::vector<std::string>{"1920x1080", "1600x900", "1280x720"}));
  // The camera's own (off-list) size is appended.
  f.vtx_set("VENC_MODE", "60@2944x1656/native");
  CHECK((values(f.node("rec.vtx.size").choices()) ==
         std::vector<std::string>{"1920x1080", "1600x900", "1280x720", "2944x1656"}));
  // An off-list current REC_SIZE comes first.
  f.vtx_set("REC_SIZE", "640x360");
  const auto cs = f.node("rec.vtx.size").choices();
  REQUIRE(!cs.empty());
  CHECK(cs[0].value == "640x360");
  CHECK(cs[0].label == "640×360");
}

TEST(rec_target_warns_when_the_sd_recorder_is_off) {
  Fx f;
  f.cfg.dvr.target = "vtx";
  f.vtx_set("REC_ENABLE", "0");
  CHECK(f.node("rec.target").warn() == "SD recorder off");
  f.cfg.dvr.target = "gs";
  CHECK(f.node("rec.target").warn().empty());
  f.cfg.dvr.target = "both";
  CHECK(f.node("rec.target").warn() == "SD recorder off");
  f.vtx_set("REC_ENABLE", "1");
  CHECK(f.node("rec.target").warn().empty());
}

TEST(display_and_colour_confirm_the_split_only_during_a_burned_recording) {
  Fx f;
  f.modes = kAvatar;
  f.cam = 100;
  CHECK(confirm(f.node("video.display"), "1280x720").empty());
  CHECK(confirm(f.node("video.colour"), "false").empty());
  CHECK(confirm(f.node("osd.gs"), "true").empty());
  CHECK(confirm(f.node("osd.fc"), "true").empty());
  f.rec = true;   // a raw recording: no split
  CHECK(confirm(f.node("video.display"), "1280x720").empty());
  f.burned = true;
  CHECK(confirm(f.node("video.display"), "1280x720") == kSplit);
  CHECK(confirm(f.node("video.colour"), "false") == kSplit);
  // The OSD switches split only when turning on.
  CHECK(confirm(f.node("osd.gs"), "true") == kSplit);
  CHECK(confirm(f.node("osd.gs"), "false").empty());
  CHECK(confirm(f.node("osd.fc"), "true") == kSplit);
  CHECK(confirm(f.node("osd.fc"), "false").empty());
}

TEST(refresh_sync_row_offers_off_nudge_follow_and_warns_when_unavailable) {
  Fx f;
  f.cfg.display.refresh_sync = "nudge";
  f.refresh_sync_available = [] { return false; };
  MenuNode n = f.node("video.refresh_sync");
  CHECK(n.advanced);
  CHECK((values(n.choices()) == std::vector<std::string>{"off", "nudge", "follow"}));
  CHECK((labels(n.choices()) == std::vector<std::string>{"Off", "Nudge", "Follow"}));
  CHECK(n.current() == "nudge");
  CHECK(n.warn() == "n/a");

  f.refresh_sync_available = [] { return true; };
  CHECK(f.node("video.refresh_sync").warn().empty());

  f.refresh_sync_available = nullptr;
  CHECK(f.node("video.refresh_sync").warn().empty());
}

TEST(display_choices_are_auto_plus_sizes_at_the_camera_rate) {
  Fx f;
  f.modes = kAvatar;
  f.cam = 100;
  f.running = "1920x1080@100.000";
  MenuNode d = f.node("video.display");
  CHECK((labels(d.choices()) == std::vector<std::string>{"Auto (largest)", "1920×1080", "1280×720"}));
  CHECK((values(d.choices()) == std::vector<std::string>{"auto", "1920x1080", "1280x720"}));
  CHECK(d.value_text() == "1920×1080 @ 100 Hz");
  CHECK(d.warn().empty());
  CHECK(d.current() == "auto");   // the bundle's screen_mode
  CHECK((d.steps("auto") == std::vector<SetStep>{{Owner::kPlayer, "screen_mode", "\"auto\""}}));
  CHECK((d.steps("1280x720") == std::vector<SetStep>{{Owner::kPlayer, "screen_mode", "\"1280x720\""}}));
  // A sized or fully pinned screen_mode reads as its size.
  f.cfg.screen_mode = "1280x720";
  CHECK(f.node("video.display").current() == "1280x720");
  f.cfg.screen_mode = "1920x1080@60";
  CHECK(f.node("video.display").current() == "1920x1080");
  // A camera rate the display has no mode for warns.
  f.cam = 90;
  d = f.node("video.display");
  CHECK(d.warn() == "≠ camera");
  CHECK((labels(d.choices()) == std::vector<std::string>{"Auto (largest)"}));
  // An unknown camera rate offers every size.
  f.cam = 0;
  d = f.node("video.display");
  CHECK(d.warn().empty());
  CHECK((labels(d.choices()) == std::vector<std::string>{"Auto (largest)", "1920×1080", "1280×720"}));
  // A rate that is not whole keeps its decimals.
  f.running = "1920x1080@59.940";
  CHECK(f.node("video.display").value_text() == "1920×1080 @ 59.94 Hz");
}

TEST(camera_mode_lists_binned_then_native_sizes_for_the_rates_the_display_shows) {
  Fx f;
  f.modes = kAvatar;   // 60 and 100 Hz, two sizes each
  f.vtx_set("VENC_MODE", "100@1280x720/binned");
  MenuNode cam = f.node("video.camera");
  CHECK((values(cam.choices()) ==
         std::vector<std::string>{"60@1920x1080/binned", "60@1280x720/binned",
                                  "60@1920x1080/native", "60@1600x900/native",
                                  "60@1280x720/native", "100@1728x972/binned",
                                  "100@1280x720/binned", "100@1920x1080/native",
                                  "100@1600x900/native", "100@1280x720/native"}));
  const auto lbls = labels(cam.choices());
  REQUIRE(lbls.size() == 10);
  CHECK(lbls[0] == "60 fps · 1920×1080 · 1.0× bin");
  CHECK(lbls[7] == "100 fps · 1920×1080 · 1.7×");
  CHECK(cam.current() == "100@1280x720/binned");
}

TEST(camera_mode_lists_every_rate_with_no_display_modes_and_tags_120) {
  Fx f;   // modes empty: every rate, binned then native, web order
  const MenuNode cam = f.node("video.camera");
  const auto cs = cam.choices();
  CHECK(cs.size() == 18);
  int n120 = 0;
  for (const Choice& c : cs) {
    if (c.value.rfind("120@", 0) == 0) {
      CHECK(c.note == "letterboxed 2.1:1");
      ++n120;
    } else {
      CHECK(c.note.empty());
    }
  }
  CHECK(n120 == 3);
}

TEST(an_off_list_camera_mode_comes_first_with_the_display_note) {
  Fx f;
  f.modes = kAvatar;   // no 90 Hz mode
  f.vtx_set("VENC_MODE", "90@1728x972/binned");
  MenuNode cam = f.node("video.camera");
  CHECK(cam.keep_current_if_unpickable);
  auto cs = cam.choices();
  REQUIRE(!cs.empty());
  CHECK(cs[0].value == "90@1728x972/binned");
  CHECK(cs[0].note == "display can't show");
  CHECK(!cs[0].pickable);

  // An off-list rate the display does show: no note, zoomed by its own
  // sensor mode even though the size itself isn't on that mode's list.
  f.vtx_set("VENC_MODE", "60@2944x1656/native");
  cam = f.node("video.camera");
  cs = cam.choices();
  REQUIRE(!cs.empty());
  CHECK(cs[0].value == "60@2944x1656/native");
  CHECK(cs[0].note.empty());
  CHECK(cs[0].label == "60 fps · 2944×1656 · 1.3×");
}

TEST(camera_mode_confirms_the_pause_and_drags_rec_fps_down_first) {
  Fx f;
  f.vtx_set("REC_ENABLE", "1");
  f.vtx_set("REC_FPS", "100");
  MenuNode cam = f.node("video.camera");
  CHECK(confirm(cam, "60@1920x1080/binned") ==
        "Video pauses about half a second. Also lowers the SD frame rate to 60.");
  CHECK((cam.steps("60@1920x1080/binned") ==
         std::vector<SetStep>{{Owner::kDrone, "REC_FPS", "60"},
                              {Owner::kDrone, "VENC_MODE", "60@1920x1080/binned"}}));

  f.vtx_set("REC_ENABLE", "0");
  cam = f.node("video.camera");
  CHECK(confirm(cam, "60@1920x1080/binned") == "Video pauses about half a second.");
  CHECK((cam.steps("60@1920x1080/binned") ==
         std::vector<SetStep>{{Owner::kDrone, "VENC_MODE", "60@1920x1080/binned"}}));

  // Picking a higher or equal rate than REC_FPS never drags it.
  f.vtx_set("REC_ENABLE", "1");
  f.vtx_set("REC_FPS", "60");
  cam = f.node("video.camera");
  CHECK(confirm(cam, "100@1920x1080/native") == "Video pauses about half a second.");
  CHECK((cam.steps("100@1920x1080/native") ==
         std::vector<SetStep>{{Owner::kDrone, "VENC_MODE", "100@1920x1080/native"}}));
}

TEST(fix_row_appears_only_for_an_unshowable_rate_and_targets_the_nearest_shown) {
  Fx f;
  f.modes = kAvatar;
  f.vtx_set("VENC_MODE", "90@1728x972/binned");
  MenuNode fix = f.node("video.fix");
  CHECK(vis(fix));
  CHECK(fix.label == "Fix: set camera to 100 fps · 1728×972");
  CHECK((fix.action() ==
         std::vector<SetStep>{{Owner::kDrone, "VENC_MODE", "100@1728x972/binned"}}));

  // 100 binned lacks 1920x1080: falls back to that mode's first size.
  f.vtx_set("VENC_MODE", "90@1920x1080/binned");
  fix = f.node("video.fix");
  CHECK(fix.label == "Fix: set camera to 100 fps · 1728×972");

  // A rate the display already shows: no Fix row.
  f.vtx_set("VENC_MODE", "100@1280x720/binned");
  CHECK(!vis(f.node("video.fix")));

  // No display modes at all: nothing to target, so no Fix row either.
  f.modes.clear();
  f.vtx_set("VENC_MODE", "90@1728x972/binned");
  CHECK(!vis(f.node("video.fix")));
}

TEST(fix_row_stays_hidden_when_the_display_has_no_mode_at_any_fixed_rate) {
  // The display offers only 75 Hz, which the camera's rate (90) does not
  // show -- but none of 60/90/100/120 is near 75 either, so there is no
  // target to fix to and the row must not offer one (it would send
  // nothing and still read "✓ saved").
  Fx f;
  f.modes = kOnly75;
  f.vtx_set("VENC_MODE", "90@1728x972/binned");
  CHECK(!vis(f.node("video.fix")));
}

TEST(mirror_and_flip_are_standard_bool_rows_that_pause) {
  Fx f;
  f.vtx_set("VENC_MIRROR", "1");
  MenuNode mirror = f.node("video.mirror");
  CHECK(mirror.current() == "true");
  CHECK(!mirror.advanced);
  const auto mcs = mirror.choices();
  REQUIRE(mcs.size() == 2);
  CHECK(mcs[0].value == "true" && mcs[0].label == "On");
  CHECK((mirror.steps("false") == std::vector<SetStep>{{Owner::kDrone, "VENC_MIRROR", "0"}}));
  CHECK(confirm(mirror, "false") == "Video pauses about half a second.");
  CHECK(mirror.help().find("mirrored") != std::string::npos);

  f.vtx_set("VENC_FLIP", "1");
  MenuNode flip = f.node("video.flip");
  CHECK(flip.current() == "true");
  CHECK((flip.steps("false") == std::vector<SetStep>{{Owner::kDrone, "VENC_FLIP", "0"}}));
  CHECK(confirm(flip, "false") == "Video pauses about half a second.");
  CHECK(flip.help().find("upside down") != std::string::npos);
}

TEST(slices_reads_on_for_any_count_above_one_and_sends_4_or_1) {
  Fx f;
  f.vtx_set("VENC_SLICES", "6");
  MenuNode sl = f.node("video.slices");
  CHECK(sl.advanced);
  CHECK(sl.current() == "4");
  f.vtx_set("VENC_SLICES", "1");
  CHECK(f.node("video.slices").current() == "1");
  CHECK((sl.steps("4") == std::vector<SetStep>{{Owner::kDrone, "VENC_SLICES", "4"}}));
  CHECK(confirm(sl, "4") == "Video pauses about half a second.");
}

TEST(drone_rows_refuse_until_their_value_lands_and_under_a_table_mismatch) {
  Fx f;
  f.snap.vtx = VtxState{};
  f.snap.vtx->linked = true;
  f.snap.vtx->list = "complete";
  CHECK(f.node("video.mirror").refuse() == "Reading the drone's settings…");

  f.snap.vtx->table_mismatch = true;
  CHECK(f.node("video.mirror").refuse() == "Drone settings differ from this GS — update");

  f.snap.vtx->table_mismatch = false;
  f.vtx_set("VENC_MIRROR", "1");
  CHECK(f.node("video.mirror").refuse().empty());

  // No sideport datagram at all yet.
  MenuTreeDeps d = f.deps();
  d.snap = nullptr;
  const auto t = build_menu_tree(d);
  const auto m = f.find(t, "video.mirror");
  REQUIRE(m.has_value());
  CHECK(m->refuse() == "Reading the drone's settings…");
}

TEST(a_row_with_no_value_under_a_partial_list_refuses_as_done_waiting) {
  // list == "partial" means the dump is finished, not still arriving: a
  // row that never answered must not read "Reading the drone's
  // settings…" forever.
  Fx f;
  f.snap.vtx = VtxState{};
  f.snap.vtx->linked = true;
  f.snap.vtx->list = "pending";
  CHECK(f.node("video.flip").refuse() == "Reading the drone's settings…");

  f.snap.vtx->list = "partial";
  CHECK(f.node("video.flip").refuse() == "This setting did not answer.");

  // A table mismatch still comes first.
  f.snap.vtx->table_mismatch = true;
  CHECK(f.node("video.flip").refuse() == "Drone settings differ from this GS — update");
}

TEST(drone_header_texts) {
  CHECK(drone_header(nullptr, false).empty());

  GsSnapshot no_vtx;
  CHECK(drone_header(&no_vtx, false).empty());

  GsSnapshot snap;
  snap.vtx = VtxState{};
  snap.vtx->linked = true;
  CHECK(drone_header(&snap, true).empty());  // stale

  snap.vtx->table_mismatch = true;
  CHECK(drone_header(&snap, false) == "Drone settings differ from this GS — update");
  snap.vtx->table_mismatch = false;

  snap.vtx->list = "pending";
  VtxRow row;
  row.id = "VENC_MODE";  // no value yet
  snap.vtx->rows = {row};
  CHECK(drone_header(&snap, false) == "Reading the drone's settings…");

  snap.vtx->rows[0].value = "100@1280x720/binned";  // every row now valued
  CHECK(drone_header(&snap, false).empty());

  snap.vtx->list = "partial";
  CHECK(drone_header(&snap, false) == "Some settings did not answer.");
}

TEST(drone_pill_mark_texts) {
  CHECK(drone_pill_mark(nullptr, false).empty());

  GsSnapshot no_vtx;
  CHECK(drone_pill_mark(&no_vtx, false).empty());

  GsSnapshot snap;
  snap.vtx = VtxState{};
  snap.vtx->linked = true;
  CHECK(drone_pill_mark(&snap, false).empty());  // alive still null: no heartbeat yet on this link

  snap.vtx->alive = true;
  CHECK(drone_pill_mark(&snap, false).empty());

  snap.vtx->alive = false;
  CHECK(drone_pill_mark(&snap, false) == "no heartbeat");
  CHECK(drone_pill_mark(&snap, true).empty());  // stale

  snap.vtx->linked = false;
  CHECK(drone_pill_mark(&snap, false).empty());  // not linked
}

TEST(video_group_order_has_the_drone_rows_in_place) {
  Fx f;
  const auto t = f.tree();
  REQUIRE(!t.empty());
  std::vector<std::string> ids;
  for (const MenuNode& n : t[0].children()) ids.push_back(n.id);
  CHECK((ids == std::vector<std::string>{"video.camera", "video.fix", "video.display",
                                         "video.colour", "video.refresh_sync", "video.mirror",
                                         "video.flip", "video.slices"}));
}

TEST(fec_enh_max_follows_base) {
  Fx f;
  f.gs.static_mcs = 3;
  f.gs.static_base = 0.6;
  f.gs.static_enh = 0.3;
  MenuNode enh = f.node("link.fec_enh");
  CHECK(enh.number.min == 0.1);
  CHECK(enh.number.max == 0.6);
  CHECK(enh.number.step == 0.05);
  f.gs.static_base = 1.25;
  CHECK(f.node("link.fec_enh").number.max == 1.25);
  const MenuNode base = f.node("link.fec_base");
  CHECK(base.number.min == 0.1 && base.number.max == 2.0 && base.number.step == 0.05);
  CHECK((base.steps("0.55") ==
         std::vector<SetStep>{{Owner::kGs, "link.static_overhead_base", "0.55"}}));
  // Lowering base below enh clamps enh first, so base never reads below it.
  CHECK((base.steps("0.2") == std::vector<SetStep>{{Owner::kGs, "link.static_overhead_enh", "0.2"},
                                                    {Owner::kGs, "link.static_overhead_base", "0.2"}}));
  // A rung's enh follows that rung's base.
  f.gs.ladder[1].base = 0.75;
  CHECK(f.node("link.ladder.r1.enh").number.max == 0.75);
}

TEST(mcs_rows_are_capped_at_five) {
  Fx f;
  f.gs.static_mcs = 3;
  const MenuNode m = f.node("link.mcs");
  CHECK(m.number.min == 0 && m.number.max == 5 && m.number.step == 1);
  CHECK(m.number.fmt(3) == "MCS3");
  CHECK((m.steps("4") == std::vector<SetStep>{{Owner::kGs, "link.static_mcs", "4"}}));
  for (size_t i = 0; i < f.gs.ladder.size(); ++i) {
    const MenuNode r = f.node("link.ladder.r" + std::to_string(i) + ".mcs");
    CHECK(r.number.min == 0 && r.number.max == 5);
  }
  // A last rung already at 5 adds another 5, never 6.
  f.gs.ladder.back().mcs = 5;
  std::vector<Rung> more = f.gs.ladder;
  more.push_back(more.back());
  CHECK((f.node("link.ladder.add").action() ==
         std::vector<SetStep>{{Owner::kGs, "link.ladder", maburplay::render_ladder_value(more)}}));
}

// maburgs' setter takes a whole-number FEC value as the bare "1" the model
// hands over: the patch lands and its loader reads it as a real, both for
// the static pair and inside a rung.
TEST(a_whole_number_fec_value_passes_maburgs_setter) {
  Fx f;
  f.gs.static_mcs = 3;
  const SetStep base = f.node("link.fec_base").steps("1").back();
  CHECK(base.value == "1");
  const SetStep rung = f.node("link.ladder.r0.base").steps("1").at(0);
  std::string text = slurp(kGsBundle);
  std::string why;
  REQUIRE(maburgs::patch_live_key(&text, *maburgs::gs_live_key(base.key), base.value, &why));
  REQUIRE(maburgs::patch_live_key(&text, *maburgs::gs_live_key(rung.key), rung.value, &why));
  ScratchFile file("test_menu_tree", ".toml");
  spit(file.path, text);
  const maburgs::Config c = maburgs::load_config(file.path);
  CHECK(c.link.static_overhead_base == 1.0);
  CHECK(c.link.ladder_cfg.ladder.at(0).overhead_base == 1.0);
}

// maburplay's setter takes the quoted strings the rows send.
TEST(the_string_values_pass_maburplays_setter) {
  struct Dir {
    std::string dir, file;
    Dir() {
      ::mkdir(MABUR_TEST_SCRATCH_DIR, 0777);
      dir = std::string(MABUR_TEST_SCRATCH_DIR) + "/test_menu_tree-" + std::to_string(::getpid());
      REQUIRE(::mkdir(dir.c_str(), 0755) == 0);
      file = dir + "/maburplay.toml";
      spit(file, slurp(kPlayBundle));
    }
    ~Dir() {
      for (const char* s : {"", ".new", ".pre-live", ".pre-live.tmp"}) std::remove((file + s).c_str());
      ::rmdir(dir.c_str());
    }
  } d;
  Fx f;
  maburplay::PlayerApplyHooks h;
  h.reacquire_display = [](const maburplay::Config&, std::string*) { return true; };
  h.osd_layers = [](const maburplay::Config&, std::string*) { return true; };
  h.next_start = [](const maburplay::Config&) {};
  h.menu = [](const maburplay::Config&) {};
  h.refresh_sync = [](const maburplay::Config&) {};
  maburplay::PlayerConfigApplier a(d.file, f.cfg, maburplay::BuildCaps{true, false}, h, nullptr);
  auto set = [&](const SetStep& s) {
    std::string why;
    REQUIRE(a.submit(s.key, s.value, &why));
    maburplay::PlayerApplyResult r{};
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (!a.poll(&r)) {
      REQUIRE(std::chrono::steady_clock::now() < deadline);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    if (!r.ok) std::printf("set %s = %s: %s\n", s.key.c_str(), s.value.c_str(), r.reason.c_str());
    return r.ok;
  };
  f.modes = kAvatar;
  f.cam = 100;
  CHECK(set(f.node("video.display").steps("1280x720").at(0)));
  CHECK(set(f.node("rec.burn.size").steps("1280x720").at(0)));
  CHECK(set(f.node("rec.target").steps("both").at(0)));
  CHECK(set(f.node("sys.size").steps("larger").at(0)));
  const maburplay::Config c = maburplay::load_config(d.file);
  CHECK(c.screen_mode == "1280x720");
  CHECK(c.dvr.burned.size_w == 1280 && c.dvr.burned.size_h == 720);
  CHECK(c.dvr.target == "both");
  CHECK(c.menu.scale == "larger");
  CHECK(set(f.node("rec.burn.size").steps("").at(0)));
  CHECK(maburplay::load_config(d.file).dvr.burned.size_w == 0);
}

TEST(menu_size_offers_three_steps_and_reads_the_config) {
  Fx f;
  MenuNode n = f.node(maburplay::kMenuSizeRow);
  CHECK(n.label == "Menu size");
  CHECK(n.owner == Owner::kPlayer);
  std::vector<std::string> values, labels;
  for (const Choice& c : n.choices()) {
    values.push_back(c.value);
    labels.push_back(c.label);
  }
  CHECK((values == std::vector<std::string>{"normal", "large", "larger"}));
  CHECK((labels == std::vector<std::string>{"Normal", "Large", "Larger"}));
  CHECK(n.current() == "normal");
  f.cfg.menu.scale = "large";
  CHECK(f.node("sys.size").current() == "large");
  const std::vector<SetStep> s = n.steps("larger");
  REQUIRE(s.size() == 1);
  CHECK(s[0].key == "menu.scale");
  CHECK(s[0].value == "\"larger\"");
}

// Until maburgs' first `get` lands the view holds GsLiveView's defaults,
// not maburgs' values: every GS row reads "—" rather than present them.
TEST(gs_rows_read_a_dash_until_the_first_get_lands) {
  Fx f;
  f.gs = GsLiveView{};
  REQUIRE(!f.gs.have);
  for (const char* id : {"link.width", "link.rate", "link.nack", "osd.fc", "sys.debug"})
    CHECK(f.node(id).value_text() == "—");
  CHECK(f.node("link.ladder").group_value() == "—");
  // The player's own rows read the player's config regardless.
  CHECK(f.node("video.colour").value_text == nullptr ||
        f.node("video.colour").value_text() != "—");
  CHECK(f.node("sys.adv").value_text == nullptr);
  f.gs = bundle_gs();
  CHECK(f.node("link.width").value_text() == "20 MHz");
  CHECK(f.node("link.rate").value_text() == "Adaptive");
  CHECK(f.node("link.nack").value_text() == (f.gs.nack ? "On" : "Off"));
  CHECK(f.node("sys.debug").value_text() == (f.gs.debug_log ? "On" : "Off"));
  CHECK(f.node("link.ladder").group_value() == "6 rungs");
  f.gs.static_mcs = 3;
  CHECK(f.node("link.mcs").value_text() == "MCS3");
  CHECK(f.node("link.fec_base").value_text() == "0.50");
  f.gs.have = false;
  CHECK(f.node("link.mcs").value_text() == "—");
  CHECK(f.node("link.fec_enh").value_text() == "—");
}

// No ladder to read a rung from, or a row built for a rung the ladder has
// since lost: nothing is sent rather than a guess.
TEST(rate_fixed_and_rung_edits_send_nothing_without_their_rung) {
  Fx f;
  const MenuNode mcs = f.node("link.ladder.r3.mcs");
  const MenuNode bw = f.node("link.ladder.r3.bw");
  const MenuNode base = f.node("link.ladder.r3.base");
  const MenuNode enh = f.node("link.ladder.r3.enh");
  f.gs.ladder.resize(3);   // shrunk under rows built for six rungs
  CHECK(mcs.steps("2").empty());
  CHECK(bw.steps("20").empty());
  CHECK(base.steps("1").empty());
  CHECK(enh.steps("0.3").empty());
  CHECK(!f.node("link.rate").steps("fixed").empty());
  f.gs.ladder.clear();
  CHECK(f.node("link.rate").steps("fixed").empty());
  CHECK((f.node("link.rate").steps("adaptive") ==
         std::vector<SetStep>{{Owner::kGs, "link.static_mcs", "-1"}}));
}

namespace {

// Every node under `nodes`, hidden or not, with each group's children
// read under the current deps.
void walk(const std::vector<MenuNode>& nodes, std::vector<MenuNode>* out) {
  for (const MenuNode& n : nodes) {
    out->push_back(n);
    if (n.kind == NodeKind::kGroup && n.children) walk(n.children(), out);
  }
}

// The values a row can send: each choice, and for a number its min, max and
// a mid-range point on its step grid.
std::vector<std::string> sample_values(const MenuNode& n) {
  std::vector<std::string> v;
  if (n.kind == NodeKind::kChoice && n.choices)
    for (const Choice& c : n.choices()) v.push_back(c.value);
  if (n.kind == NodeKind::kNumber) {
    const double steps = std::floor((n.number.max - n.number.min) / n.number.step / 2);
    for (double x : {n.number.min, n.number.max, n.number.min + steps * n.number.step}) {
      char buf[32];
      std::snprintf(buf, sizeof buf, "%g", x);
      v.push_back(buf);
    }
  }
  return v;
}

}  // namespace

// Every key any row can send is a live key of the daemon its step names: a
// typo in a row is a test failure here, not a refused set on the goggles.
TEST(every_step_key_is_a_live_key_of_its_owner) {
  Fx f;
  f.cfg.menu.show_advanced = true;
  f.cfg.dvr.target = "both";
  f.cfg.dvr.mode = "burned";
  f.modes = kAvatar;
  f.cam = 100;
  f.running = "1920x1080@100.000";
  int checked = 0;
  auto check_steps = [&](const std::vector<SetStep>& steps) {
    for (const SetStep& s : steps) {
      ++checked;
      if (s.owner == Owner::kGs) {
        if (!maburgs::gs_live_key(s.key)) std::printf("not a maburgs live key: %s\n", s.key.c_str());
        CHECK(maburgs::gs_live_key(s.key) != nullptr);
      } else if (s.owner == Owner::kDrone) {
        if (mabur::cfgparam::find(s.key.c_str()) < 0)
          std::printf("not a drone param id: %s\n", s.key.c_str());
        CHECK(mabur::cfgparam::find(s.key.c_str()) >= 0);
      } else {
        CHECK(s.owner == Owner::kPlayer);
        if (!maburplay::player_live_key(s.key))
          std::printf("not a maburplay live key: %s\n", s.key.c_str());
        CHECK(maburplay::player_live_key(s.key) != nullptr);
      }
    }
  };
  // Adaptive (the ladder and its rungs) and Fixed (the fixed sub-rows).
  for (int static_mcs : {-1, 3}) {
    f.gs.static_mcs = static_mcs;
    std::vector<MenuNode> all;
    walk(f.tree(), &all);
    for (const MenuNode& n : all) {
      if (n.steps)
        for (const std::string& v : sample_values(n)) check_steps(n.steps(v));
      if (n.action) check_steps(n.action());
    }
  }
  CHECK(checked > 40);
}

MTEST_MAIN
