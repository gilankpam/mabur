#include <string>
#include <utility>

#include "gs_layer.h"
#include "mtest.h"

using maburplay::RecState;
using maburplay::rec_text;
namespace tok = maburplay::tok;

namespace {
const std::string kDot = "\xE2\x97\x8F";
RecState gs(RecState::Kind k, int s = 0) {
  RecState r; r.kind = k; r.elapsed_s = s; return r;
}
}  // namespace

TEST(gs_only_is_byte_identical_to_before_the_vtx_recorder) {
  CHECK(rec_text(gs(RecState::Kind::kArmed)).text.empty());
  auto t = rec_text(gs(RecState::Kind::kRecording, 767));
  CHECK(t.text == kDot + " REC 12:47");
  CHECK(t.rgb == tok::kTextPrimary);
  CHECK(t.aux == 1);
  t = rec_text(gs(RecState::Kind::kFault));
  CHECK(t.text == kDot + " REC FAULT");
  CHECK(t.rgb == tok::kStatusCaution);
  CHECK(t.aux == 0);
}

TEST(both_recording_names_both) {
  auto r = gs(RecState::Kind::kRecording, 83);
  r.vtx = RecState::Vtx::kRecording; r.vtx_elapsed_s = 80;
  auto t = rec_text(r);
  CHECK(t.text == kDot + " REC GS+VTX 01:23");   // the GS clock wins
  CHECK(t.rgb == tok::kTextPrimary);
}

TEST(vtx_only_recording) {
  auto r = gs(RecState::Kind::kArmed);
  r.gs_target = false;
  r.vtx = RecState::Vtx::kRecording; r.vtx_elapsed_s = 5;
  CHECK(rec_text(r).text == kDot + " REC VTX 00:05");
}

TEST(gs_recording_vtx_problem_shows_both) {
  auto r = gs(RecState::Kind::kRecording, 61);
  r.vtx = RecState::Vtx::kNoCard;
  auto t = rec_text(r);
  CHECK(t.text == kDot + " REC GS 01:01 VTX NO CARD");
  CHECK(t.rgb == tok::kStatusCaution);
  CHECK(t.aux == 1);
}

TEST(vtx_problems_alone) {
  auto r = gs(RecState::Kind::kArmed);
  r.gs_target = false;
  const std::pair<RecState::Vtx, const char*> cases[] = {
      {RecState::Vtx::kWait, "VTX WAIT"}, {RecState::Vtx::kNoCard, "VTX NO CARD"},
      {RecState::Vtx::kFull, "VTX FULL"}, {RecState::Vtx::kFault, "VTX FAULT"},
      {RecState::Vtx::kOff, "VTX OFF"}};
  for (auto& c : cases) {
    r.vtx = c.first;
    CHECK(rec_text(r).text == kDot + " REC " + c.second);
    CHECK(rec_text(r).rgb == tok::kStatusCaution);
  }
}

TEST(gs_fault_with_vtx_recording) {
  auto r = gs(RecState::Kind::kFault);
  r.vtx = RecState::Vtx::kRecording; r.vtx_elapsed_s = 9;
  CHECK(rec_text(r).text == kDot + " REC VTX 00:09 GS FAULT");
}

TEST(worst_case_is_at_least_as_long_as_any_state) {
  auto r = gs(RecState::Kind::kFault);
  r.vtx = RecState::Vtx::kNoCard;
  CHECK(rec_text(r).text.size() <= std::string(maburplay::kRecWorst).size());
  r = gs(RecState::Kind::kRecording, 5999);
  r.vtx = RecState::Vtx::kNoCard;
  CHECK(rec_text(r).text.size() <= std::string(maburplay::kRecWorst).size());
}

MTEST_MAIN
