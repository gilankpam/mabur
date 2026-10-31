// GsLiveView: parses maburgs' live-key `get` dump (docs/gs-live-config.md)
// into a struct the goggles menu can read, and the link.ladder inline-array
// form both ways. The ladder round trip is also checked against maburgs'
// own patch_live_key (gs_config_apply.h), reachable here as a pure string
// function with no socket or ChannelCore involved, so a render this file
// produces is proven to be what maburgs' set link.ladder would accept.
#include <optional>
#include <string>
#include <vector>

#include "gs_config_apply.h"
#include "gs_live_view.h"
#include "mtest.h"

using maburplay::GsLiveView;
using maburplay::LadderWidth;
using maburplay::Rung;

TEST(a_full_get_dump_fills_every_field) {
  const std::string reply =
      "ok\n"
      "radio.channel = \"auto\"\n"
      "link.static_mcs = -1\n"
      "link.static_bw = 20\n"
      "link.static_overhead_base = 0.5\n"
      "link.static_overhead_enh = 0.25\n"
      "link.ladder = [{mcs = 0, bw = 40, overhead_base = 0.5, overhead_enh = 0.25}, "
      "{mcs = 1, bw = 40, overhead_base = 0.5, overhead_enh = 0.25}, "
      "{mcs = 2, bw = 40, overhead_base = 0.5, overhead_enh = 0.25}, "
      "{mcs = 3, bw = 40, overhead_base = 0.5, overhead_enh = 0.25}, "
      "{mcs = 4, bw = 40, overhead_base = 0.5, overhead_enh = 0.25}]\n"
      "link.nack.enable = true\n"
      "debug_log.enable = false\n"
      "msp.enable = false\n";
  GsLiveView v;
  REQUIRE(maburplay::parse_get_all(reply, &v, 1000));
  CHECK(v.have);
  CHECK(v.read_ms == 1000);
  CHECK(!v.pinned_channel.has_value());
  CHECK(v.static_mcs == -1);
  CHECK(v.static_bw == 20);
  CHECK(v.static_base == 0.5);
  CHECK(v.static_enh == 0.25);
  REQUIRE(v.ladder.size() == 5);
  for (size_t i = 0; i < 5; ++i) {
    CHECK(v.ladder[i].mcs == static_cast<int>(i));
    CHECK(v.ladder[i].bw == 40);
    CHECK(v.ladder[i].base == 0.5);
    CHECK(v.ladder[i].enh == 0.25);
  }
  CHECK(v.nack);
  CHECK(!v.debug_log);
  CHECK(!v.msp);
}

TEST(a_pinned_channel_reads_as_the_number) {
  const std::string reply = "ok\nradio.channel = 64\n";
  GsLiveView v;
  REQUIRE(maburplay::parse_get_all(reply, &v, 0));
  REQUIRE(v.pinned_channel.has_value());
  CHECK(*v.pinned_channel == 64);
}

TEST(unknown_and_malformed_get_lines_are_skipped_not_fatal) {
  const std::string reply =
      "ok\n"
      "foo.bar = 1\n"
      "link.ladder = [{mcs = }]\n"
      "link.nack.enable = maybe\n"
      "link.static_mcs = 2\n";
  GsLiveView v;
  REQUIRE(maburplay::parse_get_all(reply, &v, 42));
  CHECK(v.have);
  CHECK(v.static_mcs == 2);
  CHECK(v.ladder.empty());
  CHECK(!v.nack);  // "maybe" is not true/false: default kept
}

TEST(a_non_ok_reply_is_false) {
  GsLiveView v;
  CHECK(!maburplay::parse_get_all("err busy", &v, 0));
}

TEST(ladder_round_trips_through_render_and_parse) {
  const std::vector<Rung>& l = maburplay::default_ladder(40);
  const std::string text = maburplay::render_ladder_value(l);
  std::vector<Rung> back;
  REQUIRE(maburplay::parse_ladder_value(text, &back));
  REQUIRE(back.size() == l.size());
  for (size_t i = 0; i < l.size(); ++i) {
    CHECK(back[i].mcs == l[i].mcs);
    CHECK(back[i].bw == l[i].bw);
    CHECK(back[i].base == l[i].base);
    CHECK(back[i].enh == l[i].enh);
  }

  // The same text through maburgs' own patcher (gs_config_apply.h), a pure
  // string function reachable without the daemon or ChannelCore: proves
  // this render is what `set link.ladder = ...` would accept.
  std::string file_text =
      "[[link.ladder]]\n"
      "mcs = 0\n"
      "bw  = 40\n"
      "overhead_base = 0.5\n"
      "overhead_enh  = 0.25\n";
  const maburgs::LiveKey* key = maburgs::gs_live_key("link.ladder");
  REQUIRE(key != nullptr);
  std::string why;
  CHECK(maburgs::patch_live_key(&file_text, *key, text, &why));
  CHECK(why.empty());
}

TEST(ladder_width_and_default_detection) {
  const std::vector<Rung>& l40 = maburplay::default_ladder(40);
  CHECK(maburplay::ladder_width(l40) == LadderWidth::k40);
  CHECK(maburplay::is_default_ladder(l40));

  const std::vector<Rung>& l20 = maburplay::default_ladder(20);
  CHECK(maburplay::ladder_width(l20) == LadderWidth::k20);
  CHECK(maburplay::is_default_ladder(l20));

  std::vector<Rung> mixed = l40;
  mixed[0].bw = 20;
  CHECK(maburplay::ladder_width(mixed) == LadderWidth::kMixed);
  CHECK(!maburplay::is_default_ladder(mixed));
}

MTEST_MAIN
