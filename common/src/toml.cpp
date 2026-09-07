#include "mabur/toml.h"

#include <cctype>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <vector>

namespace mabur {
namespace toml {
namespace {

const char* kind_name(Value::Kind k) {
  switch (k) {
    case Value::Kind::Table: return "table";
    case Value::Kind::Array: return "array";
    case Value::Kind::String: return "string";
    case Value::Kind::Int: return "int";
    case Value::Kind::Float: return "float";
    case Value::Kind::Bool: return "bool";
  }
  return "?";
}

std::string trim(const std::string& s) {
  std::size_t a = s.find_first_not_of(" \t\r");
  if (a == std::string::npos) return "";
  std::size_t b = s.find_last_not_of(" \t\r");
  return s.substr(a, b - a + 1);
}

bool is_bare_key(const std::string& s) {
  if (s.empty()) return false;
  for (char c : s)
    if (!(std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '-'))
      return false;
  return true;
}

// -?[0-9]+
bool is_int_lit(const std::string& s) {
  std::size_t i = (!s.empty() && s[0] == '-') ? 1 : 0;
  if (i >= s.size()) return false;
  for (; i < s.size(); ++i)
    if (!std::isdigit(static_cast<unsigned char>(s[i]))) return false;
  return true;
}

// -?[0-9]+\.[0-9]+
bool is_float_lit(const std::string& s) {
  const std::size_t dot = s.find('.');
  if (dot == std::string::npos) return false;
  return is_int_lit(s.substr(0, dot)) && is_int_lit(s.substr(dot + 1)) &&
         s.substr(dot + 1).find('-') == std::string::npos;
}

struct Parser {
  std::string name;
  std::vector<std::string> lines;
  Value root{Value::Kind::Table, 0};
  Value* cur = &root;
  std::vector<std::string> explicit_tables;

  [[noreturn]] void err(std::size_t line1, const std::string& msg) const {
    throw Error(name + ":" + std::to_string(line1) + ": " + msg);
  }

  // Cuts a trailing # comment that is not inside a basic string.
  std::string strip_comment(const std::string& s, std::size_t line1) const {
    bool in_str = false;
    for (std::size_t i = 0; i < s.size(); ++i) {
      const char c = s[i];
      if (in_str) {
        if (c == '\\') {
          ++i;
        } else if (c == '"') {
          in_str = false;
        }
      } else if (c == '"') {
        in_str = true;
      } else if (c == '#') {
        return s.substr(0, i);
      }
    }
    // Deliberately no unterminated-string check here: `x = """multi` must
    // reach parse_string so it can say "multi-line strings are not supported"
    // rather than the less useful "unterminated string".
    (void)line1;
    return s;
  }

  std::vector<std::string> split_path(const std::string& inner,
                                      std::size_t line1) const {
    std::vector<std::string> parts;
    std::size_t start = 0;
    while (true) {
      const std::size_t dot = inner.find('.', start);
      const std::string part =
          trim(inner.substr(start, dot == std::string::npos
                                       ? std::string::npos
                                       : dot - start));
      if (!is_bare_key(part))
        err(line1, "table header: '" + part + "' is not a bare key");
      parts.push_back(part);
      if (dot == std::string::npos) break;
      start = dot + 1;
    }
    return parts;
  }

  std::string parse_string(const std::string& tok, std::size_t line1) const {
    if (tok.size() >= 3 && tok.compare(0, 3, "\"\"\"") == 0)
      err(line1, "multi-line strings are not supported");
    if (tok.size() < 2 || tok.back() != '"')
      err(line1, "unterminated string");
    std::string out;
    for (std::size_t i = 1; i + 1 < tok.size(); ++i) {
      if (tok[i] != '\\') {
        out += tok[i];
        continue;
      }
      if (i + 2 >= tok.size()) err(line1, "trailing backslash in string");
      const char e = tok[++i];
      if (e == '\\' || e == '"') {
        out += e;
      } else {
        err(line1, std::string("unsupported escape \\") + e +
                       " (only \\\\ and \\\" are supported)");
      }
    }
    return out;
  }

  // Net '[' depth of `s`, ignoring brackets inside basic strings.
  int bracket_depth(const std::string& s) const {
    int depth = 0;
    bool in_str = false;
    for (std::size_t i = 0; i < s.size(); ++i) {
      if (in_str) {
        if (s[i] == '\\') ++i;
        else if (s[i] == '"') in_str = false;
      } else if (s[i] == '"') {
        in_str = true;
      } else if (s[i] == '[') {
        ++depth;
      } else if (s[i] == ']') {
        --depth;
      }
    }
    return depth;
  }

  // Splits "[a, b, c]" (outermost brackets included) into its elements.
  // A trailing comma is allowed; nested arrays are rejected by the caller.
  std::vector<std::string> split_elements(const std::string& tok,
                                          std::size_t line1) const {
    std::vector<std::string> out;
    const std::string body = tok.substr(1, tok.size() - 2);
    std::string cur_el;
    int depth = 0;
    bool in_str = false;
    for (std::size_t i = 0; i < body.size(); ++i) {
      const char c = body[i];
      if (in_str) {
        cur_el += c;
        if (c == '\\' && i + 1 < body.size()) cur_el += body[++i];
        else if (c == '"') in_str = false;
        continue;
      }
      if (c == '"') { in_str = true; cur_el += c; continue; }
      if (c == '[') ++depth;
      if (c == ']') --depth;
      if (c == ',' && depth == 0) {
        out.push_back(trim(cur_el));
        cur_el.clear();
        continue;
      }
      cur_el += c;
    }
    if (in_str) err(line1, "unterminated string");
    const std::string last = trim(cur_el);
    if (!last.empty()) out.push_back(last);       // else: trailing comma
    for (const std::string& e : out)
      if (e.empty()) err(line1, "empty array element");
    return out;
  }

  Value parse_scalar(const std::string& tok, std::size_t line1) const {
    if (tok.empty()) err(line1, "missing value");
    if (tok[0] == '{') err(line1, "inline tables are not supported");
    if (tok[0] == '\'') err(line1, "literal strings are not supported; use \"...\"");
    if (tok[0] == '[') {
      if (tok.back() != ']') err(line1, "unterminated array");
      Value v(Value::Kind::Array, static_cast<int>(line1));
      bool first = true;
      Value::Kind elem_kind = Value::Kind::Table;
      for (const std::string& e : split_elements(tok, line1)) {
        if (e[0] == '[') err(line1, "arrays of arrays are not supported");
        Value ev = parse_scalar(e, line1);
        if (first) {
          elem_kind = ev.kind();
          first = false;
        } else if (ev.kind() != elem_kind) {
          err(line1, "array has mixed types");
        }
        v.push(std::move(ev));
      }
      return v;
    }
    if (tok[0] == '"') {
      Value v(Value::Kind::String, static_cast<int>(line1));
      v.set_string(parse_string(tok, line1));
      return v;
    }
    if (tok == "true" || tok == "false") {
      Value v(Value::Kind::Bool, static_cast<int>(line1));
      v.set_bool(tok == "true");
      return v;
    }
    if (is_int_lit(tok)) {
      Value v(Value::Kind::Int, static_cast<int>(line1));
      v.set_int(std::strtoll(tok.c_str(), nullptr, 10));
      return v;
    }
    if (is_float_lit(tok)) {
      Value v(Value::Kind::Float, static_cast<int>(line1));
      v.set_float(std::strtod(tok.c_str(), nullptr));
      return v;
    }
    err(line1, "'" + tok + "' is not a valid value");
  }

  // Walks/creates the parent tables of `parts`, returning the parent.
  Value* parent_of(const std::vector<std::string>& parts, std::size_t line1) {
    Value* node = &root;
    for (std::size_t i = 0; i + 1 < parts.size(); ++i) {
      Value* next = node->find(parts[i]);
      if (next == nullptr) {
        next = &node->set(parts[i], Value(Value::Kind::Table,
                                          static_cast<int>(line1)));
      } else if (next->is_array()) {
        if (next->empty()) err(line1, "'" + parts[i] + "' is an empty array");
        next = &const_cast<Value&>(next->at(next->size() - 1));
      } else if (!next->is_object()) {
        err(line1, "'" + parts[i] + "' is not a table");
      }
      node = next;
    }
    return node;
  }

  void table_header(const std::string& inner, std::size_t line1) {
    const std::vector<std::string> parts = split_path(inner, line1);
    const std::string dotted = inner;
    for (const std::string& d : explicit_tables)
      if (d == dotted) err(line1, "table [" + dotted + "] redefined");
    explicit_tables.push_back(dotted);

    Value* parent = parent_of(parts, line1);
    Value* self = parent->find(parts.back());
    if (self == nullptr)
      self = &parent->set(parts.back(),
                          Value(Value::Kind::Table, static_cast<int>(line1)));
    else if (!self->is_object())
      err(line1, "'" + parts.back() + "' is not a table");
    cur = self;
  }

  void array_table_header(const std::string& inner, std::size_t line1) {
    const std::vector<std::string> parts = split_path(inner, line1);
    Value* parent = parent_of(parts, line1);
    Value* self = parent->find(parts.back());
    if (self == nullptr) {
      self = &parent->set(parts.back(),
                          Value(Value::Kind::Array, static_cast<int>(line1)));
    } else if (!self->is_array()) {
      err(line1, "'" + parts.back() + "' is not an array of tables");
    }
    cur = &self->push(Value(Value::Kind::Table, static_cast<int>(line1)));
  }

  void assignment(const std::string& line, std::size_t line1) {
    bool in_str = false;
    std::size_t eq = std::string::npos;
    for (std::size_t i = 0; i < line.size(); ++i) {
      if (in_str) {
        if (line[i] == '\\') ++i;
        else if (line[i] == '"') in_str = false;
      } else if (line[i] == '"') {
        in_str = true;
      } else if (line[i] == '=') {
        eq = i;
        break;
      }
    }
    if (eq == std::string::npos) err(line1, "expected 'key = value'");
    const std::string key = trim(line.substr(0, eq));
    const std::string rhs = trim(line.substr(eq + 1));
    if (key.find('.') != std::string::npos)
      err(line1, "dotted keys in assignments are not supported; use a [table] header");
    if (!key.empty() && (key.front() == '"' || key.front() == '\''))
      err(line1, "quoted keys are not supported");
    if (!is_bare_key(key)) err(line1, "'" + key + "' is not a bare key");
    if (cur->find(key) != nullptr) err(line1, "duplicate key '" + key + "'");
    cur->set(key, parse_scalar(rhs, line1));
  }

  Value run() {
    for (std::size_t i = 0; i < lines.size(); ++i) {
      const std::size_t line1 = i + 1;
      std::string line = trim(strip_comment(lines[i], line1));
      if (line.empty()) continue;

      if (line.compare(0, 2, "[[") == 0) {
        if (line.size() < 4 || line.compare(line.size() - 2, 2, "]]") != 0)
          err(line1, "table header: expected a closing ']]'");
        array_table_header(trim(line.substr(2, line.size() - 4)), line1);
        continue;
      }
      if (line[0] == '[') {
        if (line.back() != ']')
          err(line1, "table header: expected a closing ']'");
        table_header(trim(line.substr(1, line.size() - 2)), line1);
        continue;
      }

      // An assignment whose value opens an array may span lines. Gather
      // continuation lines (comments stripped) until the brackets balance.
      while (bracket_depth(line) > 0) {
        if (++i >= lines.size()) err(line1, "unterminated array");
        line += " " + trim(strip_comment(lines[i], i + 1));
      }
      assignment(line, line1);
    }
    return std::move(root);
  }
};

std::vector<std::string> split_lines(const std::string& text) {
  std::vector<std::string> out;
  std::istringstream in(text);
  std::string line;
  while (std::getline(in, line)) out.push_back(line);
  return out;
}

}  // namespace

// ---- Value ----

bool Value::contains(const std::string& key) const {
  for (const auto& kv : table_)
    if (kv.first == key) return true;
  return false;
}

const Value& Value::at(const std::string& key) const {
  for (const auto& kv : table_)
    if (kv.first == key) return kv.second;
  throw Error("no such key: '" + key + "'");
}

const Value& Value::at(std::size_t i) const {
  if (kind_ != Kind::Array || i >= array_.size())
    throw Error("array index " + std::to_string(i) + " out of range");
  return array_[i];
}

std::size_t Value::size() const {
  if (kind_ == Kind::Table) return table_.size();
  if (kind_ == Kind::Array) return array_.size();
  return 0;
}

Value& Value::set(const std::string& key, Value v) {
  table_.emplace_back(key, std::move(v));
  return table_.back().second;
}

Value& Value::push(Value v) {
  array_.push_back(std::move(v));
  return array_.back();
}

Value* Value::find(const std::string& key) {
  for (auto& kv : table_)
    if (kv.first == key) return &kv.second;
  return nullptr;
}

void Value::type_error(const char* want) const {
  throw Error("line " + std::to_string(line_) + ": expected " + want +
              ", got " + kind_name(kind_));
}

const std::string& Value::const_iterator::key() const {
  if (tab_ == nullptr) throw Error("key() on an array iterator");
  return (*tab_)[i_].first;
}

Value::const_iterator Value::begin() const {
  return kind_ == Kind::Array ? const_iterator(nullptr, &array_, 0)
                              : const_iterator(&table_, nullptr, 0);
}

Value::const_iterator Value::end() const {
  return kind_ == Kind::Array
             ? const_iterator(nullptr, &array_, array_.size())
             : const_iterator(&table_, nullptr, table_.size());
}

Value::ItemsRange Value::items() const {
  if (kind_ != Kind::Table) throw Error("items() on a non-table");
  return ItemsRange(&table_);
}

// ---- entry points ----

Value parse_toml_string(const std::string& text, const std::string& name) {
  Parser p;
  p.name = name;
  p.lines = split_lines(text);
  return p.run();
}

Value parse_toml_file(const std::string& path) {
  std::ifstream in(path);
  if (!in) throw Error(path + ": cannot open");
  std::ostringstream ss;
  ss << in.rdbuf();
  return parse_toml_string(ss.str(), path);
}

}  // namespace toml
}  // namespace mabur
