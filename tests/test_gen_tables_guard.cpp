#include <fstream>
#include <sstream>
#include <string>

#include "mtest.h"

// Tripwire: kTxagcGainDb[] was deleted by hand from gs/src/gen/gen_tables.h
// (2026-07-17) because mabur's power model moved to the linear offset-qdB
// gain in gs/src/energy.h. gen_tables.py (devourer-derived) still emits the
// table, so re-running it without re-deleting the array would silently
// reintroduce the abandoned model. This guards against that regression by
// asserting the array DEFINITION (kTxagcGainDb[] = {) is absent — the
// explanatory comment mentioning the deleted name by name is fine and must
// keep matching text, so we specifically look for the definition pattern,
// not just any occurrence of the identifier.
TEST(gen_tables_kTxagcGainDb_stays_deleted) {
  const std::string path =
      std::string(MABUR_SOURCE_DIR) + "/gs/src/gen/gen_tables.h";
  std::ifstream in(path);
  CHECK(in.is_open());
  std::stringstream ss;
  ss << in.rdbuf();
  const std::string contents = ss.str();

  CHECK(contents.find("kTxagcGainDb") != std::string::npos);  // comment stays
  CHECK(contents.find("kTxagcGainDb[] =") == std::string::npos);
}

MTEST_MAIN
