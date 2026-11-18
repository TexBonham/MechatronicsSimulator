#pragma once

#include <cstddef>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace mechsim::physics::native::json {

class ParseError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

// The subset of JSON the native translation writes (RFC 8259 values; numbers as double). Keys keep no order: the
// translation addresses every member by name.
class Value {
public:
    enum class Kind { Null, Bool, Number, String, Array, Object };

    Value() = default;

    [[nodiscard]] Kind kind() const noexcept { return kind_; }
    [[nodiscard]] bool is_null() const noexcept { return kind_ == Kind::Null; }
    [[nodiscard]] bool boolean(std::string_view what) const;
    [[nodiscard]] double number(std::string_view what) const;
    [[nodiscard]] int integer(std::string_view what) const;
    [[nodiscard]] const std::string& string(std::string_view what) const;
    [[nodiscard]] const std::vector<Value>& array(std::string_view what) const;
    [[nodiscard]] bool has(std::string_view key) const;
    [[nodiscard]] const Value& at(std::string_view key) const;

    static Value parse(std::string_view text);

private:
    friend class Parser;

    Kind kind_ = Kind::Null;
    bool boolean_ = false;
    double number_ = 0.0;
    std::string string_;
    std::vector<Value> array_;
    std::map<std::string, Value, std::less<>> object_;
};

}  // namespace mechsim::physics::native::json
