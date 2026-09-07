#include <string>

#include "mabur/toml.h"
#include "mtest.h"

using mabur::toml::Error;
using mabur::toml::Value;
using mabur::toml::parse_toml_string;

namespace {

// Returns the what() of parsing `text`, or "" if it parsed cleanly.
std::string parse_error(const std::string& text) {
  try {
    parse_toml_string(text, "t.toml");
  } catch (const Error& e) {
    return e.what();
  }
  return "";
}

}  // namespace

TEST(toml_scalars_and_tables) {
  Value v = parse_toml_string(
      "# leading comment\n"
      "backend = \"mpp\"\n"
      "\n"
      "[radio]\n"
      "channel = 149          # trailing comment\n"
      "wall_margin_db = 1.0\n"
      "power_mode = \"none\"\n"
      "\n"
      "[venc.roi]\n"
      "enabled = true\n"
      "center = 0.4\n",
      "t.toml");

  CHECK(v.is_object());
  CHECK(v.contains("backend"));
  CHECK(v.at("backend").get<std::string>() == "mpp");
  CHECK(v.at("radio").is_object());
  CHECK(v.at("radio").at("channel").get<int>() == 149);
  CHECK(v.at("radio").at("wall_margin_db").get<double>() == 1.0);
  CHECK(v.at("radio").at("power_mode").get<std::string>() == "none");
  CHECK(v.at("venc").at("roi").at("enabled").get<bool>() == true);
  CHECK(v.at("venc").at("roi").at("center").get<double>() == 0.4);
  CHECK(!v.contains("nope"));
}

TEST(toml_line_numbers_on_the_value) {
  Value v = parse_toml_string("[fec]\nsymbol_size = 332\n", "t.toml");
  // Line 2 is where symbol_size sits; loaders quote this in errors.
  CHECK(v.at("fec").at("symbol_size").line() == 2);
}

TEST(toml_int_widens_to_float_but_not_the_reverse) {
  Value v = parse_toml_string("a = 1\nb = 2.5\n", "t.toml");
  CHECK(v.at("a").get<double>() == 1.0);   // int -> float: allowed
  CHECK(v.at("a").get<int>() == 1);
  CHECK(v.at("a").is_number_integer());
  CHECK(!v.at("b").is_number_integer());
  CHECK(v.at("b").is_number());
  bool threw = false;
  try {
    v.at("b").get<int>();                  // float -> int: rejected
  } catch (const Error&) {
    threw = true;
  }
  CHECK(threw);
}

TEST(toml_type_errors_name_the_wanted_type) {
  Value v = parse_toml_string("a = 1\n", "t.toml");
  std::string msg;
  try {
    v.at("a").get<std::string>();
  } catch (const Error& e) {
    msg = e.what();
  }
  CHECK(msg.find("string") != std::string::npos);
  CHECK(msg.find("int") != std::string::npos);
}

TEST(toml_iteration_yields_values_with_keys) {
  Value v = parse_toml_string("[t]\na = 1\nb = 2\n", "t.toml");
  const Value& t = v.at("t");
  CHECK(t.size() == 2);
  // nlohmann shape: *it is the value, it.key() is the key.
  std::string keys;
  for (auto it = t.begin(); it != t.end(); ++it) keys += it.key();
  CHECK(keys == "ab");
  // items() shape, for structured bindings.
  long sum = 0;
  for (auto& [k, val] : t.items()) {
    CHECK(!k.empty());
    sum += val.get<long>();
  }
  CHECK(sum == 3);
}

TEST(toml_string_escapes) {
  Value v = parse_toml_string("p = \"/etc/a\\\\b\"\nq = \"say \\\"hi\\\"\"\n",
                              "t.toml");
  CHECK(v.at("p").get<std::string>() == "/etc/a\\b");
  CHECK(v.at("q").get<std::string>() == "say \"hi\"");
  CHECK(parse_error("p = \"a\\nb\"\n").find("escape") != std::string::npos);
}

TEST(toml_rejects_unsupported_constructs_with_line_numbers) {
  struct Case { const char* text; const char* needle; int line; };
  const Case cases[] = {
      {"[a]\nx = { y = 1 }\n", "inline table", 2},
      {"[a]\nx = 'lit'\n", "literal string", 2},
      {"[a]\nx = \"\"\"multi\n", "multi-line string", 2},
      {"[a]\nx.y = 1\n", "dotted key", 2},
      {"[a]\nx = 1\nx = 2\n", "duplicate key", 3},
      {"[a]\nx = 1\n[a]\ny = 2\n", "redefined", 3},
      {"[a]\nx = 0x1f\n", "not a valid value", 2},
      {"[a]\nx = 1_000\n", "not a valid value", 2},
      {"[a]\nx = 1979-05-27\n", "not a valid value", 2},
      {"[a\nx = 1\n", "table header", 1},
      {"x = 1\n", "", 0},  // sentinel: top-level scalar is fine
  };
  for (const Case& c : cases) {
    const std::string msg = parse_error(c.text);
    if (c.needle[0] == '\0') {
      CHECK(msg.empty());
      continue;
    }
    CHECK(msg.find(c.needle) != std::string::npos);
    CHECK(msg.find("t.toml:" + std::to_string(c.line) + ":") !=
          std::string::npos);
  }
}

TEST(toml_missing_file_is_an_error) {
  bool threw = false;
  try {
    mabur::toml::parse_toml_file("/nonexistent/nope.toml");
  } catch (const Error&) {
    threw = true;
  }
  CHECK(threw);
}

MTEST_MAIN
