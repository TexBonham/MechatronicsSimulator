#include "mechsim/integrity/limits.hpp"

#include <charconv>
#include <cstdint>
#include <functional>
#include <cmath>
#include <cstddef>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

namespace mechsim::integrity {

namespace {

// A strict reader for the one document tier1.py writes (json.dumps output): objects, arrays, finite numbers,
// strings with the escapes json.dumps emits, booleans and null; duplicate keys and trailing text refused.
struct Value {
    enum class Kind : std::uint8_t { null, boolean, number, string, array, object };
    Kind kind = Kind::null;
    bool flag = false;
    double number = 0.0;
    std::string text;
    std::vector<Value> items;
    std::map<std::string, Value, std::less<>> fields;
};

constexpr int max_depth = 32;

class Reader {
public:
    explicit Reader(std::string_view text) : text_(text) {}

    Value document() {
        Value value = parse(0);
        skip();
        if (at_ != text_.size()) {
            fail("trailing text");
        }
        return value;
    }

private:
    [[noreturn]] void fail(const std::string& what) const {
        throw LimitsError("limits table: " + what + " at offset " + std::to_string(at_));
    }

    void skip() {
        while (at_ < text_.size() && (text_[at_] == ' ' || text_[at_] == '\n' || text_[at_] == '\r' ||
                                      text_[at_] == '\t')) {
            ++at_;
        }
    }

    bool literal(std::string_view word) {
        if (text_.substr(at_, word.size()) == word) {
            at_ += word.size();
            return true;
        }
        return false;
    }

    Value parse(int depth) {
        if (depth > max_depth) {
            fail("nesting too deep");
        }
        skip();
        if (at_ >= text_.size()) {
            fail("unexpected end");
        }
        Value value;
        const char c = text_[at_];
        if (c == '{') {
            value.kind = Value::Kind::object;
            ++at_;
            skip();
            if (at_ < text_.size() && text_[at_] == '}') {
                ++at_;
                return value;
            }
            while (true) {
                skip();
                if (at_ >= text_.size() || text_[at_] != '"') {
                    fail("expected a key");
                }
                std::string key = string();
                skip();
                if (at_ >= text_.size() || text_[at_] != ':') {
                    fail("expected ':'");
                }
                ++at_;
                Value item = parse(depth + 1);
                if (value.fields.contains(key)) {
                    fail("duplicate key '" + key + "'");
                }
                value.fields.emplace(std::move(key), std::move(item));
                skip();
                if (at_ < text_.size() && text_[at_] == ',') {
                    ++at_;
                    continue;
                }
                if (at_ < text_.size() && text_[at_] == '}') {
                    ++at_;
                    return value;
                }
                fail("expected ',' or '}'");
            }
        }
        if (c == '[') {
            value.kind = Value::Kind::array;
            ++at_;
            skip();
            if (at_ < text_.size() && text_[at_] == ']') {
                ++at_;
                return value;
            }
            while (true) {
                value.items.push_back(parse(depth + 1));
                skip();
                if (at_ < text_.size() && text_[at_] == ',') {
                    ++at_;
                    continue;
                }
                if (at_ < text_.size() && text_[at_] == ']') {
                    ++at_;
                    return value;
                }
                fail("expected ',' or ']'");
            }
        }
        if (c == '"') {
            value.kind = Value::Kind::string;
            value.text = string();
            return value;
        }
        if (literal("true")) {
            value.kind = Value::Kind::boolean;
            value.flag = true;
            return value;
        }
        if (literal("false")) {
            value.kind = Value::Kind::boolean;
            return value;
        }
        if (literal("null")) {
            return value;
        }
        value.kind = Value::Kind::number;
        value.number = number();
        return value;
    }

    double number() {
        const std::size_t start = at_;
        if (at_ < text_.size() && text_[at_] == '-') {
            ++at_;
        }
        if (at_ >= text_.size() || text_[at_] < '0' || text_[at_] > '9') {
            fail("expected a value");
        }
        if (text_[at_] == '0' && at_ + 1 < text_.size() && text_[at_ + 1] >= '0' && text_[at_ + 1] <= '9') {
            fail("leading zero");
        }
        while (at_ < text_.size() && ((text_[at_] >= '0' && text_[at_] <= '9') || text_[at_] == '.' ||
                                      text_[at_] == 'e' || text_[at_] == 'E' || text_[at_] == '+' ||
                                      text_[at_] == '-')) {
            ++at_;
        }
        double out = 0.0;
        const char* first = text_.data() + start;
        const char* last = text_.data() + at_;
        const std::from_chars_result read = std::from_chars(first, last, out);
        if (read.ec != std::errc{} || read.ptr != last || !std::isfinite(out)) {
            fail("malformed number");
        }
        return out;
    }

    std::string string() {
        ++at_;
        std::string out;
        while (true) {
            if (at_ >= text_.size()) {
                fail("unterminated string");
            }
            const char c = text_[at_];
            ++at_;
            if (c == '"') {
                return out;
            }
            if (static_cast<unsigned char>(c) < 0x20) {
                fail("control character in a string");
            }
            if (c != '\\') {
                out.push_back(c);
                continue;
            }
            if (at_ >= text_.size()) {
                fail("unterminated escape");
            }
            const char e = text_[at_];
            ++at_;
            if (e == '"' || e == '\\' || e == '/') {
                out.push_back(e);
            } else if (e == 'n') {
                out.push_back('\n');
            } else if (e == 't') {
                out.push_back('\t');
            } else {
                fail("unsupported escape");
            }
        }
    }

    std::string_view text_;
    std::size_t at_ = 0;
};

const Value& field(const Value& object, std::string_view key, std::string_view where) {
    if (object.kind != Value::Kind::object) {
        throw LimitsError("limits table: " + std::string(where) + " is not an object");
    }
    const auto found = object.fields.find(key);
    if (found == object.fields.end()) {
        throw LimitsError("limits table: " + std::string(where) + " has no '" + std::string(key) + "'");
    }
    return found->second;
}

double number(const Value& object, std::string_view key, std::string_view where) {
    const Value& value = field(object, key, where);
    if (value.kind != Value::Kind::number) {
        throw LimitsError("limits table: " + std::string(where) + "." + std::string(key) + " is not a number");
    }
    return value.number;
}

double positive(const Value& object, std::string_view key, std::string_view where) {
    const double value = number(object, key, where);
    if (!(value > 0.0)) {
        throw LimitsError("limits table: " + std::string(where) + "." + std::string(key) + " must be > 0");
    }
    return value;
}

std::size_t count(const Value& object, std::string_view key, std::string_view where) {
    const double value = number(object, key, where);
    if (value < 0.0 || std::floor(value) != value || value > 1e15) {
        throw LimitsError("limits table: " + std::string(where) + "." + std::string(key) +
                          " must be a non-negative integer");
    }
    return static_cast<std::size_t>(value);
}

bool flag(const Value& object, std::string_view key, std::string_view where) {
    const Value& value = field(object, key, where);
    if (value.kind != Value::Kind::boolean) {
        throw LimitsError("limits table: " + std::string(where) + "." + std::string(key) + " is not a boolean");
    }
    return value.flag;
}

const std::string& text(const Value& object, std::string_view key, std::string_view where) {
    const Value& value = field(object, key, where);
    if (value.kind != Value::Kind::string) {
        throw LimitsError("limits table: " + std::string(where) + "." + std::string(key) + " is not a string");
    }
    return value.text;
}

const std::vector<Value>& array(const Value& object, std::string_view key, std::string_view where) {
    const Value& value = field(object, key, where);
    if (value.kind != Value::Kind::array) {
        throw LimitsError("limits table: " + std::string(where) + "." + std::string(key) + " is not an array");
    }
    return value.items;
}

Support support(const Value& entry, const std::string& where) {
    Support out;
    out.at = number(entry, "at", where);
    const std::string& condition = text(entry, "condition", where);
    if (condition == "clamped") {
        out.condition = SupportCondition::clamped;
    } else if (condition == "pinned") {
        out.condition = SupportCondition::pinned;
    } else {
        throw LimitsError("limits table: " + where + ".condition '" + condition + "': clamped or pinned");
    }
    return out;
}

MemberLimits member(const Value& entry, std::size_t position) {
    const std::string where = "members[" + std::to_string(position) + "]";
    MemberLimits m;
    m.name = text(entry, "name", where);
    m.index = count(entry, "index", where);
    m.area = positive(entry, "area", where);
    m.i_y = positive(entry, "i_y", where);
    m.i_z = positive(entry, "i_z", where);
    m.c_y = positive(entry, "c_y", where);
    m.c_z = positive(entry, "c_z", where);
    m.length = positive(entry, "length", where);
    m.elastic_modulus = positive(entry, "elastic_modulus", where);
    m.effective_length_factor = positive(entry, "effective_length_factor", where);
    m.allowable_stress = positive(entry, "allowable_stress", where);
    m.yield_strength = positive(entry, "yield_strength", where);
    if (field(entry, "deflection_limit", where).kind != Value::Kind::null) {
        m.deflection_limit = positive(entry, "deflection_limit", where);
    }
    std::size_t k = 0;
    for (const Value& item : array(entry, "supports", where)) {
        m.supports.push_back(support(item, where + ".supports[" + std::to_string(k) + "]"));
        ++k;
    }
    m.first_spring = count(entry, "first_spring", where);
    m.segments = count(entry, "segments", where);
    m.segment_length = positive(entry, "segment_length", where);
    m.node_stiffness = positive(entry, "node_stiffness", where);
    if (m.segments == 0) {
        throw LimitsError("limits table: " + where + ".segments must be >= 1");
    }
    return m;
}

}  // namespace

Limits parse_limits(std::string_view json) {
    const Value root = Reader(json).document();
    if (text(root, "format", "root") != limits_format) {
        throw LimitsError("limits table: format must be '" + std::string(limits_format) + "'");
    }
    Limits limits;
    limits.minimum_safety_factor = positive(root, "minimum_safety_factor", "root");
    const Value& checks = field(root, "checks", "root");
    limits.stress = flag(checks, "stress", "checks");
    limits.buckling = flag(checks, "buckling", "checks");
    limits.deflection = flag(checks, "deflection", "checks");
    limits.fastener = flag(checks, "fastener", "checks");
    limits.yield_offset = flag(checks, "yield_offset", "checks");
    std::size_t k = 0;
    for (const Value& entry : array(root, "members", "root")) {
        limits.members.push_back(member(entry, k));
        ++k;
    }
    k = 0;
    for (const Value& entry : array(root, "fastenings", "root")) {
        const std::string where = "fastenings[" + std::to_string(k) + "]";
        FasteningLimits f;
        f.name = text(entry, "name", where);
        f.joint = count(entry, "joint", where);
        f.load_limit = positive(entry, "load_limit", where);
        limits.fastenings.push_back(std::move(f));
        ++k;
    }
    return limits;
}

}  // namespace mechsim::integrity
