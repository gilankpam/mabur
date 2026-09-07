#pragma once

// A deliberately small TOML reader for mabur's three config files.
//
// The accessor surface mirrors nlohmann::json's, because the loaders in
// drone/src/config.cpp, gs/src/config.cpp and gs/player/src/player_config.cpp
// were written against that surface and change type only, not shape.
//
// The accepted subset is documented in docs/deploy.md. Everything outside it
// -- inline tables, literal and multi-line strings, dotted keys, dates, hex
// and underscored ints -- is rejected with a file:line message rather than
// silently reinterpreted.

#include <cstdint>
#include <deque>
#include <limits>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>

namespace mabur {
namespace toml {

// Syntax errors from parse_toml_*(), type errors from Value::get<T>().
// what() is already prefixed with "<name>:<line>: ".
struct Error : std::runtime_error {
  using std::runtime_error::runtime_error;
};

class Value {
 public:
  enum class Kind { Table, Array, String, Int, Float, Bool };

  Value() = default;
  explicit Value(Kind k, int line = 0) : kind_(k), line_(line) {}

  Kind kind() const { return kind_; }
  int line() const { return line_; }

  bool is_object() const { return kind_ == Kind::Table; }
  bool is_array() const { return kind_ == Kind::Array; }
  bool is_string() const { return kind_ == Kind::String; }
  bool is_boolean() const { return kind_ == Kind::Bool; }
  bool is_number() const { return kind_ == Kind::Int || kind_ == Kind::Float; }
  bool is_number_integer() const { return kind_ == Kind::Int; }

  bool contains(const std::string& key) const;
  const Value& at(const std::string& key) const;
  const Value& operator[](const std::string& key) const { return at(key); }
  const Value& at(std::size_t i) const;
  std::size_t size() const;
  bool empty() const { return size() == 0; }

  // Exact kind required, except double, which accepts an Int node too
  // (int widens to float; float never narrows to int).
  template <typename T>
  T get() const {
    if constexpr (std::is_same_v<T, std::string>) {
      if (kind_ != Kind::String) type_error("string");
      return str_;
    } else if constexpr (std::is_same_v<T, bool>) {
      if (kind_ != Kind::Bool) type_error("bool");
      return bool_;
    } else if constexpr (std::is_floating_point_v<T>) {
      if (kind_ == Kind::Float) return static_cast<T>(flt_);
      if (kind_ == Kind::Int) return static_cast<T>(int_);
      type_error("float");
    } else if constexpr (std::is_integral_v<T>) {
      if (kind_ != Kind::Int) type_error("int");
      if constexpr (std::is_signed_v<T>) {
        if (int_ < static_cast<std::int64_t>(std::numeric_limits<T>::min()) ||
            int_ > static_cast<std::int64_t>(std::numeric_limits<T>::max()))
          type_error("int (out of range for this field)");
      } else {
        // numeric_limits<uint64_t>::max() does not survive a cast to int64_t,
        // so compare in the unsigned domain after excluding negatives.
        if (int_ < 0 ||
            static_cast<std::uint64_t>(int_) >
                static_cast<std::uint64_t>(std::numeric_limits<T>::max()))
          type_error("int (out of range for this field)");
      }
      return static_cast<T>(int_);
    } else {
      static_assert(sizeof(T) == 0, "toml::Value::get<T>: unsupported type");
    }
  }

  // --- parser-side mutation ---
  Value& set(const std::string& key, Value v);  // caller checks duplicates
  Value& push(Value v);
  Value* find(const std::string& key);
  void set_string(std::string s) { str_ = std::move(s); }
  void set_int(std::int64_t v) { int_ = v; }
  void set_float(double v) { flt_ = v; }
  void set_bool(bool v) { bool_ = v; }

  using TableStore = std::deque<std::pair<std::string, Value>>;
  using ArrayStore = std::deque<Value>;

  // Iteration is nlohmann-shaped: *it is the VALUE, it.key() is the key.
  class const_iterator {
   public:
    const_iterator(const TableStore* t, const ArrayStore* a, std::size_t i)
        : tab_(t), arr_(a), i_(i) {}
    const Value& operator*() const {
      return tab_ ? (*tab_)[i_].second : (*arr_)[i_];
    }
    const Value* operator->() const { return &**this; }
    const Value& value() const { return **this; }
    const std::string& key() const;
    const_iterator& operator++() {
      ++i_;
      return *this;
    }
    bool operator==(const const_iterator& o) const { return i_ == o.i_; }
    bool operator!=(const const_iterator& o) const { return i_ != o.i_; }

   private:
    const TableStore* tab_;
    const ArrayStore* arr_;
    std::size_t i_;
  };

  const_iterator begin() const;
  const_iterator end() const;

  // `for (auto& [k, v] : val.items())`
  class ItemsRange {
   public:
    explicit ItemsRange(const TableStore* t) : tab_(t) {}
    TableStore::const_iterator begin() const { return tab_->begin(); }
    TableStore::const_iterator end() const { return tab_->end(); }

   private:
    const TableStore* tab_;
  };
  ItemsRange items() const;

 private:
  [[noreturn]] void type_error(const char* want) const;

  Kind kind_ = Kind::Table;
  int line_ = 0;
  // deque, not vector: the parser holds pointers into these while appending
  // to the same container, which vector would invalidate.
  TableStore table_;
  ArrayStore array_;
  std::string str_;
  std::int64_t int_ = 0;
  double flt_ = 0.0;
  bool bool_ = false;
};

Value parse_toml_file(const std::string& path);
Value parse_toml_string(const std::string& text, const std::string& name);

}  // namespace toml
}  // namespace mabur
