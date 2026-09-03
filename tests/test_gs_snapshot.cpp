#include "mtest.h"
#include "gs_snapshot.h"
#include <cstring>
#include <string>
using namespace maburplay;

static bool parse(const std::string& s, GsSnapshot* out) {
  return parse_gs_snapshot(s.data(), s.size(), out);
}

// A trimmed but structurally faithful sideport datagram.
static const char* kLive = R"({
  "v": 1, "seq": 42, "t_ms": 1000, "session": 7,
  "cards": [
    {"id": 0, "up": true,
     "classes": {"s0": {"rssi": -58.4, "snr": 18.2, "evm": -22.5,
                        "pps": 136.0}}},
    {"id": 1, "up": true,
     "classes": {"s0": {"rssi": -71.0, "snr": 9.0, "pps": 130.0}}}
  ],
  "link": {
    "air_pct": 61.5,
    "residual_loss": 0.0,
    "rtt": {"ms": 12.4, "min_ms": 8.0, "n": 42, "pts_off_us": -123456789,
            "floor_ms": 3.2},
    "ctl": {"rung": {"idx": 3, "mcs": 5, "ov_base": 0.25}, "pre_fec_loss": 0.021},
    "video": {"fps": 60.0, "jitter_ms": 3.1, "mbps": 24.6}
  },
  "drone": null
})";

TEST(parses_a_live_datagram) {
  GsSnapshot s;
  REQUIRE(parse(kLive, &s));
  REQUIRE(s.mcs.has_value());
  CHECK(*s.mcs == 5);
  REQUIRE(s.fec_pct.has_value());
  CHECK(*s.fec_pct > 24.9 && *s.fec_pct < 25.1);      // ov 0.25 -> 25 %
  REQUIRE(s.air_pct.has_value());
  CHECK(*s.air_pct > 61.4 && *s.air_pct < 61.6);
  REQUIRE(s.pre_loss_pct.has_value());
  CHECK(*s.pre_loss_pct > 2.09 && *s.pre_loss_pct < 2.11);  // 0.021 -> 2.1 %
  REQUIRE(s.post_loss_pct.has_value());
  CHECK(*s.post_loss_pct == 0.0);
  REQUIRE(s.cards.size() == 2);
  CHECK(s.cards[0].id == 0);
  CHECK(s.cards[0].heard);
  CHECK(*s.cards[0].rssi_dbm < -58.3 && *s.cards[0].rssi_dbm > -58.5);
  CHECK(*s.cards[0].snr_db > 18.1 && *s.cards[0].snr_db < 18.3);
  REQUIRE(s.cards[0].evm_db.has_value());
  CHECK(*s.cards[0].evm_db > -22.6 && *s.cards[0].evm_db < -22.4);
  CHECK(s.cards[1].id == 1);
  // Card 1's s0 carries no "evm" at all: absent, and it must NOT cost the
  // card its heard status (see the evm_db comment in gs_snapshot.h).
  CHECK(!s.cards[1].evm_db.has_value());
  CHECK(s.cards[1].heard);
  // link.rtt (link-rtt 2026-09-02): control-path RTT + the pts offset the
  // player folds into its own anchor for the absolute LAT floor.
  REQUIRE(s.rtt_ms.has_value());
  CHECK(*s.rtt_ms > 12.3 && *s.rtt_ms < 12.5);
  REQUIRE(s.pts_off_us.has_value());
  CHECK(*s.pts_off_us == -123456789);
}

// link.rtt is null until the estimator's first sample, and pts_off_us stays
// null until the drone ships a usable pts clock — each absence must stay an
// empty optional, never zero (an offset of 0 is a REAL value that would
// silently shift every absolute LAT number).
TEST(rtt_nulls_and_partial_block) {
  const char* j1 = R"({"v":1,"cards":[],"link":{"rtt":null}})";
  GsSnapshot s;
  REQUIRE(parse(j1, &s));
  CHECK(!s.rtt_ms.has_value());
  CHECK(!s.pts_off_us.has_value());
  const char* j2 =
      R"({"v":1,"cards":[],"link":{"rtt":{"ms":9.5,"pts_off_us":null}}})";
  REQUIRE(parse(j2, &s));
  REQUIRE(s.rtt_ms.has_value());
  CHECK(*s.rtt_ms > 9.4 && *s.rtt_ms < 9.6);
  CHECK(!s.pts_off_us.has_value());
  // Wrong-typed members drop individually, same rule as every other field.
  const char* j3 =
      R"({"v":1,"cards":[],"link":{"rtt":{"ms":"slow","pts_off_us":7}}})";
  REQUIRE(parse(j3, &s));
  CHECK(!s.rtt_ms.has_value());
  REQUIRE(s.pts_off_us.has_value());
  CHECK(*s.pts_off_us == 7);
}

// nulls are "never received", NOT zero. Conflating them would paint a dead
// link as a perfect one.
TEST(nulls_stay_empty_and_never_become_zero) {
  const char* j = R"({"v":1,"cards":[],
    "link":{"air_pct":null,"residual_loss":null,
            "ctl":{"rung":{"mcs":0,"ov_base":1.0},"pre_fec_loss":0.0}}})";
  GsSnapshot s;
  REQUIRE(parse(j, &s));
  CHECK(!s.air_pct.has_value());
  CHECK(!s.post_loss_pct.has_value());
  CHECK(s.cards.empty());
  REQUIRE(s.mcs.has_value());
  CHECK(*s.mcs == 0);
}

// A card with no s0 class is present but unheard: the row still renders.
TEST(card_without_s0_is_present_but_unheard) {
  const char* j = R"({"v":1,"cards":[{"id":0,"up":true,"classes":{}}],
    "link":{"ctl":{"rung":{"mcs":1,"ov_base":0.5}}}})";
  GsSnapshot s;
  REQUIRE(parse(j, &s));
  REQUIRE(s.cards.size() == 1);
  CHECK(s.cards[0].id == 0);
  CHECK(!s.cards[0].heard);
  CHECK(!s.cards[0].rssi_dbm.has_value());
  CHECK(!s.cards[0].snr_db.has_value());
  CHECK(!s.cards[0].evm_db.has_value());
}

// A card whose s0 exists but carries nulls is also unheard.
TEST(card_with_null_s0_values_is_unheard) {
  const char* j = R"({"v":1,"cards":[
      {"id":0,"up":true,"classes":{"s0":{"rssi":null,"snr":null}}}],
    "link":{}})";
  GsSnapshot s;
  REQUIRE(parse(j, &s));
  REQUIRE(s.cards.size() == 1);
  CHECK(!s.cards[0].heard);
}

// EVM is the one s0 figure the chip may simply not report on an otherwise
// perfectly healthy card: the aggregator leaves evm_has false until a frame
// arrives with PHY status, and the exporter then writes null. That absence
// must render as a blank EVM field, NEVER as an unheard card -- so it stays
// out of `heard`, which is worst-of(rssi, snr) and nothing else.
TEST(null_evm_leaves_a_heard_card_heard) {
  const char* j = R"({"v":1,"cards":[
      {"id":0,"up":true,"classes":{"s0":{"rssi":-50.0,"snr":15.0,
                                         "evm":null}}}],
    "link":{}})";
  GsSnapshot s;
  REQUIRE(parse(j, &s));
  REQUIRE(s.cards.size() == 1);
  CHECK(s.cards[0].heard);
  CHECK(!s.cards[0].evm_db.has_value());
  // Same for a wrong-typed evm: it drops on its own, like every other field.
  const char* j2 = R"({"v":1,"cards":[
      {"id":0,"up":true,"classes":{"s0":{"rssi":-50.0,"snr":15.0,
                                         "evm":"clean"}}}],
    "link":{}})";
  REQUIRE(parse(j2, &s));
  REQUIRE(s.cards.size() == 1);
  CHECK(s.cards[0].heard);
  CHECK(!s.cards[0].evm_db.has_value());
}

// Schema v1 is additive-only: consumers MUST ignore keys they do not know.
TEST(unknown_keys_are_ignored) {
  const char* j = R"({"v":1,"future_block":{"x":1},"cards":[
      {"id":0,"up":true,"brand_new":7,
       "classes":{"s0":{"rssi":-40.0,"snr":20.0,"tomorrow":1}}}],
    "link":{"air_pct":10.0,"new_thing":[1,2,3],
            "ctl":{"rung":{"mcs":7,"ov_base":0.1},"pre_fec_loss":0.0}}})";
  GsSnapshot s;
  REQUIRE(parse(j, &s));
  CHECK(*s.mcs == 7);
  REQUIRE(s.cards.size() == 1);
  CHECK(s.cards[0].heard);
}

// Whole blocks may be missing entirely on an idle link.
TEST(missing_blocks_yield_empty_optionals) {
  GsSnapshot s;
  REQUIRE(parse(R"({"v":1})", &s));
  CHECK(!s.mcs.has_value());
  CHECK(!s.fec_pct.has_value());
  CHECK(!s.air_pct.has_value());
  CHECK(!s.pre_loss_pct.has_value());
  CHECK(!s.post_loss_pct.has_value());
  CHECK(s.cards.empty());
}

// Malformed input returns false and must not throw -- this runs on the
// main loop, where an exception is a dead player.
TEST(malformed_input_returns_false_without_throwing) {
  GsSnapshot s;
  CHECK(!parse("{not json", &s));
  CHECK(!parse("", &s));
  CHECK(!parse("[]", &s));            // valid JSON, wrong shape
  CHECK(!parse("\"a string\"", &s));
  CHECK(!parse_gs_snapshot(nullptr, 0, &s));
  // Additional adversarial shapes: truncated mid-object, embedded NUL,
  // deeply nested garbage that is technically valid JSON but not an object
  // at the top level, and a bare number.
  CHECK(!parse(R"({"v":1,"cards":[{"id":0,)", &s));      // truncated
  CHECK(!parse(std::string("{\"v\":1,\x00\"x\":1}", 14), &s));  // NUL byte
  CHECK(!parse("[[[[[[[[[[1]]]]]]]]]]", &s));            // nested, but array
  CHECK(!parse("null", &s));
  CHECK(!parse("42", &s));
  CHECK(!parse("{", &s));
}

// Wrong types where a number is expected are dropped field-by-field, not
// treated as a whole-datagram failure: one bad key must not blank the OSD.
TEST(wrong_types_drop_only_their_own_field) {
  const char* j = R"({"v":1,"cards":[
      {"id":0,"up":true,"classes":{"s0":{"rssi":"loud","snr":18.0}}}],
    "link":{"air_pct":"lots","ctl":{"rung":{"mcs":5,"ov_base":0.25}}}})";
  GsSnapshot s;
  REQUIRE(parse(j, &s));
  CHECK(*s.mcs == 5);
  CHECK(!s.air_pct.has_value());
  REQUIRE(s.cards.size() == 1);
  CHECK(!s.cards[0].rssi_dbm.has_value());
  CHECK(s.cards[0].snr_db.has_value());
  CHECK(!s.cards[0].heard);  // needs BOTH to be a heard card
}

// Parsing into a reused struct must not accumulate cards across calls.
TEST(reused_output_is_reset_each_parse) {
  GsSnapshot s;
  REQUIRE(parse(kLive, &s));
  REQUIRE(s.cards.size() == 2);
  REQUIRE(parse(R"({"v":1,"cards":[],"link":{}})", &s));
  CHECK(s.cards.empty());
  CHECK(!s.mcs.has_value());
}

// A card id of 0 must not be mistaken for "missing id" -- 0 is card 0's
// real identity, not a sentinel. A buggy `if (id)` on the parsed int
// (rather than on the optional) would silently leave card 0's id at its
// default-constructed 0 anyway, so pair this with a nonzero id too.
TEST(card_id_zero_is_a_real_id_not_a_missing_one) {
  const char* j = R"({"v":1,"cards":[
      {"id":0,"up":true,"classes":{}},
      {"id":5,"up":true,"classes":{}}],
    "link":{}})";
  GsSnapshot s;
  REQUIRE(parse(j, &s));
  REQUIRE(s.cards.size() == 2);
  CHECK(s.cards[0].id == 0);
  CHECK(s.cards[1].id == 5);
}

// A finite double outside int's range must drop the field, not become a
// garbage int. A raw `(int)` cast on 1e20 is undefined behaviour and
// saturates to INT_MIN unsanitized on x86-64/aarch64 -- silently painting
// a nonsense MCS or card id on screen instead of the field vanishing as
// "never received", which is this parser's contract for every other bad
// field.
TEST(out_of_range_numbers_drop_the_field_not_saturate) {
  const char* j = R"({"v":1,"cards":[
      {"id":1e20,"up":true,"classes":{}}],
    "link":{"ctl":{"rung":{"mcs":1e20,"ov_base":0.1}}}})";
  GsSnapshot s;
  REQUIRE(parse(j, &s));
  CHECK(!s.mcs.has_value());
  REQUIRE(s.cards.size() == 1);
  CHECK(s.cards[0].id == 0);  // default-constructed: the id field dropped

  const char* jneg = R"({"v":1,"cards":[],
    "link":{"ctl":{"rung":{"mcs":-1e20,"ov_base":0.1}}}})";
  GsSnapshot sneg;
  REQUIRE(parse(jneg, &sneg));
  CHECK(!sneg.mcs.has_value());

  // A legitimate, large-but-in-range value must still come through.
  const char* jbig = R"({"v":1,"cards":[{"id":2000000000,"up":true}],
    "link":{"ctl":{"rung":{"mcs":2000000000,"ov_base":0.1}}}})";
  GsSnapshot sbig;
  REQUIRE(parse(jbig, &sbig));
  REQUIRE(sbig.mcs.has_value());
  CHECK(*sbig.mcs == 2000000000);
  REQUIRE(sbig.cards.size() == 1);
  CHECK(sbig.cards[0].id == 2000000000);
}

// "cards" as a JSON object rather than an array must not be mistaken for
// the array shape -- the is_array() guard should simply leave cards empty.
TEST(cards_as_object_is_ignored_not_crashed_on) {
  const char* j = R"({"v":1,"cards":{"0":{"id":0}},"link":{}})";
  GsSnapshot s;
  REQUIRE(parse(j, &s));
  CHECK(s.cards.empty());
}

// A REAL recorded datagram (line 73 of flight-field-20260802.jsonl, an
// active bench link on the mcs0 floor rung) fed through byte-for-byte.
// Values below were independently read out of that same line with
// `python3 -c "json.load(...)"`, not copied from the parser's own output --
// this is the test that catches a field-name typo that hand-written JSON
// would never expose.
TEST(parses_a_real_recorded_datagram) {
  const char* j = R"({"cards":[{"classes":{"ctrl":{"mbps":0.0,"pps":0.0,"rssi":-25.0,"rssi_a":-30.0,"rssi_b":-25.0,"snr":71.0,"snr_a":70.0,"snr_b":71.0},"msp":{"mbps":0.085696,"pps":8.0,"rssi":-26.959999999999994,"rssi_a":-32.0,"rssi_b":-26.959999999999994,"snr":72.215,"snr_a":72.025,"snr_b":70.595},"s0":{"mbps":1.526464,"pps":136.0,"rssi":-24.80416064112279,"rssi_a":-29.04270965212156,"rssi_b":-24.80416064112279,"snr":66.7718252487806,"snr_a":66.20505826232304,"snr_b":66.73435959731553},"s1":{"mbps":2.25664,"pps":208.0,"rssi":-26.231875423676556,"rssi_a":-33.31211913839418,"rssi_b":-26.26379836747526,"snr":69.20633653082075,"snr_a":68.93402960646482,"snr_b":68.67392638851244},"s3":{"mbps":1.123104,"pps":108.0,"rssi":-25.830868445361943,"rssi_a":-32.313639499108575,"rssi_b":-25.96043755334317,"snr":69.0652465508737,"snr_a":68.33334744480135,"snr_b":68.27826176071365}},"crc_fail":0,"foreign_pps":2.0,"frames":231,"id":0,"inj_pps":468.0,"last_frame_age_ms":1,"loss_pct":3.4188034188034178,"pps":460.0,"rx_mbps":4.991904,"self_pps":4.0,"tx_fail":0,"tx_pps":12.0,"up":true},{"classes":{"ctrl":{"mbps":0.0,"pps":0.0,"rssi":-22.0,"rssi_a":-22.0,"rssi_b":-31.0,"snr":70.0,"snr_a":70.0,"snr_b":70.0},"msp":{"mbps":0.085696,"pps":8.0,"rssi":-21.61999999999999,"rssi_a":-21.61999999999999,"rssi_b":-32.480000000000004,"snr":74.421,"snr_a":69.604,"snr_b":74.421},"s0":{"mbps":1.526464,"pps":136.0,"rssi":-19.92727821470703,"rssi_a":-19.92727821470703,"rssi_b":-31.335641757560936,"snr":70.862985602643,"snr_a":69.48530272948422,"snr_b":69.05747372044709},"s1":{"mbps":2.279088,"pps":210.0,"rssi":-10.777735687217572,"rssi_a":-10.777735687217572,"rssi_b":-18.20601179747966,"snr":69.89961261632335,"snr_a":69.15389930872344,"snr_b":68.53036404952576},"s3":{"mbps":1.100656,"pps":106.0,"rssi":-12.33730372302142,"rssi_a":-12.33730372302142,"rssi_b":-22.506309178131218,"snr":69.96435513058267,"snr_a":68.67310948105724,"snr_b":68.61995908372467}},"crc_fail":0,"foreign_pps":4.0,"frames":231,"id":1,"inj_pps":468.0,"last_frame_age_ms":1,"loss_pct":3.4188034188034178,"pps":460.0,"rx_mbps":4.991904,"self_pps":12.0,"tx_fail":0,"tx_pps":8.0,"up":true}],"drone":null,"link":{"air_pct":78.50956460176991,"ctl":{"budget":0.6666666666666666,"counters":{"demotes_residual":0,"demotes_util":0,"probation_fails":0,"promotes":0,"starved_drops":0,"timeout_drops":0},"down_util":0.6,"ladder":[{"mcs":0,"ov":1.0},{"mcs":2,"ov":0.5},{"mcs":4,"ov":0.25},{"mcs":5,"ov":0.25},{"mcs":6,"ov":0.15},{"mcs":7,"ov":0.1}],"last_event":{"from":0,"reason":"none","t_ms":0.0,"to":0,"u":0.0},"penalized":[],"pre_fec_loss":0.1927710843373494,"probation_ms_left":0,"rung":{"idx":0,"mcs":0,"ov":1.0},"up_util":0.15,"util":0.28915662650602414},"deadline_ms":200,"layer_delivery_pct":[100,100,100,100],"op":{"bw":20,"mcs":0,"offset_qdb":0,"overhead":1.0,"sgi":false,"snr_req":0.0,"vht":false},"residual_loss":0.0,"state":"session","streams":[{"abandoned":0,"abandoned_s":0.0,"bad_cfg":0,"in_flight":0,"inj_kbps":1580.4981238938053,"ov":2.0,"phy_mbps":6.5,"recovered":32,"recovered_arrived":0,"recovered_arrived_s":0.0,"recovered_s":64.0,"rung_ldpc":true,"rung_mcs":0,"rung_stbc":true,"stale":91,"stream":0,"sub_fail":0,"syms_in_s":1088.0},{"abandoned":0,"abandoned_s":0.0,"bad_cfg":0,"in_flight":0,"inj_kbps":2359.763681415929,"ov":2.0,"phy_mbps":6.5,"recovered":41,"recovered_arrived":9,"recovered_arrived_s":18.0,"recovered_s":82.0,"rung_ldpc":true,"rung_mcs":0,"rung_stbc":true,"stale":133,"stream":1,"sub_fail":0,"syms_in_s":1616.0},{"abandoned":0,"abandoned_s":0.0,"bad_cfg":0,"in_flight":0,"inj_kbps":1162.8598938053099,"ov":1.0,"phy_mbps":6.5,"recovered":0,"recovered_arrived":0,"recovered_arrived_s":0.0,"recovered_s":0.0,"rung_ldpc":true,"rung_mcs":0,"rung_stbc":true,"stale":100,"stream":3,"sub_fail":0,"syms_in_s":792.0}],"tx_card":1,"video":{"clean":30,"dropped":7,"fps":60.0,"jitter_ms":8.672911674886388,"mbps":1.637824,"q_drop":0,"rtp":{"back":0,"gap":0,"gap_seqs":0,"ok":88},"stall_resets":0,"truncated":0,"udp":{"bytes":102364,"failed":0,"sent":89}},"vtx_id":1},"seq":72,"session":3963394334,"t_ms":51049,"v":1})";
  GsSnapshot s;
  REQUIRE(parse(j, &s));

  REQUIRE(s.mcs.has_value());
  CHECK(*s.mcs == 0);                                  // link.ctl.rung.mcs
  // This real capture predates the Task 5 (same-rate-fixed-pairs) ov ->
  // ov_base rename: it wrote rung.ov, not rung.ov_base, so fec_pct is
  // absent -- the honest "never received" rendering, not a crash or a
  // stale/wrong number.
  CHECK(!s.fec_pct.has_value());
  REQUIRE(s.air_pct.has_value());
  CHECK(*s.air_pct > 78.50 && *s.air_pct < 78.51);      // link.air_pct passthrough
  REQUIRE(s.pre_loss_pct.has_value());
  CHECK(*s.pre_loss_pct > 19.27 && *s.pre_loss_pct < 19.28);  // 0.1927.. -> %
  REQUIRE(s.post_loss_pct.has_value());
  CHECK(*s.post_loss_pct == 0.0);                       // residual_loss 0.0

  REQUIRE(s.cards.size() == 2);
  CHECK(s.cards[0].id == 0);
  CHECK(s.cards[0].heard);
  REQUIRE(s.cards[0].rssi_dbm.has_value());
  CHECK(*s.cards[0].rssi_dbm > -24.81 && *s.cards[0].rssi_dbm < -24.80);
  REQUIRE(s.cards[0].snr_db.has_value());
  CHECK(*s.cards[0].snr_db > 66.77 && *s.cards[0].snr_db < 66.78);

  CHECK(s.cards[1].id == 1);
  CHECK(s.cards[1].heard);
  REQUIRE(s.cards[1].rssi_dbm.has_value());
  CHECK(*s.cards[1].rssi_dbm > -19.93 && *s.cards[1].rssi_dbm < -19.92);
  REQUIRE(s.cards[1].snr_db.has_value());
  CHECK(*s.cards[1].snr_db > 70.86 && *s.cards[1].snr_db < 70.87);
}

// Static-pin mode (link.static_mcs >= 0) never ticks the ladder controller,
// so the exporter emits link.ctl: null on purpose -- but the link is still
// running at a real MCS and a real FEC overhead, both of them in link.op.
// Reading only link.ctl.rung blanked the OSD's rung field for the whole
// flight; link.op is the GS-commanded op point in BOTH modes (it is filled
// from vrx.cur_op(), which the pin branch writes directly), so it is the
// fallback.
TEST(pinned_link_reads_the_rung_from_link_op) {
  const char* j = R"({"v":1,
    "link":{"air_pct":40.0,"ctl":null,
            "op":{"mcs":4,"bw":20,"sgi":false,"vht":false,
                  "overhead_base":0.5,"overhead_enh":0.5,"snr_req":0.0}}})";
  GsSnapshot s;
  REQUIRE(parse(j, &s));
  REQUIRE(s.mcs.has_value());
  CHECK(*s.mcs == 4);
  REQUIRE(s.fec_pct.has_value());
  CHECK(*s.fec_pct > 49.9 && *s.fec_pct < 50.1);       // ov 0.5 -> 50 %
}

// Same null-ctl datagram, the LOSS row: pre-FEC loss reads link.pre_fec_loss
// (the link-level gauge, always exported) when the ladder block is absent.
// Post-FEC already came from link.residual_loss and was never affected.
TEST(pinned_link_reads_pre_fec_loss_from_link_level) {
  const char* j = R"({"v":1,
    "link":{"ctl":null,"pre_fec_loss":0.031,"residual_loss":0.004}})";
  GsSnapshot s;
  REQUIRE(parse(j, &s));
  REQUIRE(s.pre_loss_pct.has_value());
  CHECK(*s.pre_loss_pct > 3.09 && *s.pre_loss_pct < 3.11);   // 0.031 -> 3.1 %
  REQUIRE(s.post_loss_pct.has_value());
  CHECK(*s.post_loss_pct > 0.39 && *s.post_loss_pct < 0.41);
}

// A null link-level gauge (no valid s1 window this poll) must stay an empty
// optional and render as the em-dash, never as a real 0.0% loss.
TEST(null_pre_fec_loss_stays_empty_not_zero) {
  const char* j = R"({"v":1,"link":{"ctl":null,"pre_fec_loss":null}})";
  GsSnapshot s;
  REQUIRE(parse(j, &s));
  CHECK(!s.pre_loss_pct.has_value());
}

// The ctl figure is what the controller acted on and is the right number to
// show while the ladder is running, so it wins over the link-level gauge.
TEST(ctl_pre_fec_loss_wins_over_the_link_level_gauge) {
  const char* j = R"({"v":1,
    "link":{"ctl":{"pre_fec_loss":0.021},"pre_fec_loss":0.031}})";
  GsSnapshot s;
  REQUIRE(parse(j, &s));
  REQUIRE(s.pre_loss_pct.has_value());
  CHECK(*s.pre_loss_pct > 2.09 && *s.pre_loss_pct < 2.11);
}

// The fallback must never override a ticking ladder: when link.ctl.rung is
// present it wins, even mid-probe when link.op has already moved to the
// probe point.
TEST(ctl_rung_wins_over_link_op_when_both_are_present) {
  const char* j = R"({"v":1,
    "link":{"ctl":{"rung":{"idx":3,"mcs":5,"ov_base":0.25}},
            "op":{"mcs":6,"overhead_base":0.15}}})";
  GsSnapshot s;
  REQUIRE(parse(j, &s));
  REQUIRE(s.mcs.has_value());
  CHECK(*s.mcs == 5);
  REQUIRE(s.fec_pct.has_value());
  CHECK(*s.fec_pct > 24.9 && *s.fec_pct < 25.1);
}

MTEST_MAIN
