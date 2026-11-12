#include "json.hpp"

#include <charconv>
#include <cmath>
#include <string>
#include <system_error>

namespace mechsim::physics::native::json {

namespace {

[[noreturn]] void wrong(std::string_view what, const char* expected) {
    throw ParseError(std::string(what) + ": expected " + expected);
}

}  // namespace

class Parser {
public:
    explicit Parser(std::string_view text) : text_(text) {}

    Value document() {
        Value value = parse_value(0);
        skip_space();
        if (at_ != text_.size()) {
            fail("trailing characters");
        }
        return value;
    }

private:
    static constexpr int max_depth = 64;

    [[noreturn]] void fail(const std::string& why) const {
        throw ParseError("engine model JSON, offset " + std::to_string(at_) + ": " + why);
    }

    void skip_space() {
        while (at_ < text_.size() &&
               (text_[at_] == ' ' || text_[at_] == '\n' || text_[at_] == '\r' || text_[at_] == '\t')) {
            ++at_;
        }
    }

    char peek() {
        skip_space();
        if (at_ >= text_.size()) {
            fail("unexpected end");
        }
        return text_[at_];
    }

    void expect(char c) {
        if (peek() != c) {
            fail(std::string("expected '") + c + "'");
        }
        ++at_;
    }

    bool literal(std::string_view word) {
        if (text_.substr(at_, word.size()) == word) {
            at_ += word.size();
            return true;
        }
        return false;
    }

    std::string parse_string() {
        expect('"');
        std::string out;
        while (true) {
            if (at_ >= text_.size()) {
                fail("unterminated string");
            }
            const char c = text_[at_++];
            if (c == '"') {
                return out;
            }
            if (c != '\\') {
                out.push_back(c);
                continue;
            }
            if (at_ >= text_.size()) {
                fail("unterminated escape");
            }
            const char e = text_[at_++];
            if (e == 'n') {
                out.push_back('\n');
            } else if (e == 't') {
                out.push_back('\t');
            } else if (e == 'r') {
                out.push_back('\r');
            } else if (e == 'b') {
                out.push_back('\b');
            } else if (e == 'f') {
                out.push_back('\f');
            } else if (e == 'u') {
                if (at_ + 4 > text_.size()) {
                    fail("short \\u escape");
                }
                unsigned code = 0;
                const auto result = std::from_chars(text_.data() + at_, text_.data() + at_ + 4, code, 16);
                if (result.ec != std::errc{} || result.ptr != text_.data() + at_ + 4) {
                    fail("bad \\u escape");
                }
                at_ += 4;
                if (code >= 0x80U) {
                    fail("non-ASCII \\u escape");
                }
                out.push_back(static_cast<char>(code));
            } else {
                out.push_back(e);
            }
        }
    }

    Value parse_value(int depth) {
        if (depth > max_depth) {
            fail("nesting too deep");
        }
        Value value;
        const char c = peek();
        if (c == '{') {
            ++at_;
            value.kind_ = Value::Kind::Object;
            if (peek() == '}') {
                ++at_;
                return value;
            }
            while (true) {
                std::string key = parse_string();
                expect(':');
                value.object_.insert_or_assign(std::move(key), parse_value(depth + 1));
                if (peek() == ',') {
                    ++at_;
                    continue;
                }
                expect('}');
                return value;
            }
        }
        if (c == '[') {
            ++at_;
            value.kind_ = Value::Kind::Array;
            if (peek() == ']') {
                ++at_;
                return value;
            }
            while (true) {
                value.array_.push_back(parse_value(depth + 1));
                if (peek() == ',') {
                    ++at_;
                    continue;
                }
                expect(']');
                return value;
            }
        }
        if (c == '"') {
            value.kind_ = Value::Kind::String;
            value.string_ = parse_string();
            return value;
        }
        if (literal("true")) {
            value.kind_ = Value::Kind::Bool;
            value.boolean_ = true;
            return value;
        }
        if (literal("false")) {
            value.kind_ = Value::Kind::Bool;
            return value;
        }
        if (literal("null")) {
            return value;
        }
        const char* begin = text_.data() + at_;
        const char* end = text_.data() + text_.size();
        double number = 0.0;
        const auto result = std::from_chars(begin, end, number);
        if (result.ec != std::errc{} || result.ptr == begin || !std::isfinite(number)) {
            fail("bad value");
        }
        at_ += static_cast<std::size_t>(result.ptr - begin);
        value.kind_ = Value::Kind::Number;
        value.number_ = number;
        return value;
    }

    std::string_view text_;
    std::size_t at_ = 0;
};

Value Value::parse(std::string_view text) {
    Parser parser(text);
    return parser.document();
}

bool Value::boolean(std::string_view what) const {
    if (kind_ != Kind::Bool) {
        wrong(what, "true or false");
    }
    return boolean_;
}

double Value::number(std::string_view what) const {
    if (kind_ != Kind::Number) {
        wrong(what, "a number");
    }
    return number_;
}

int Value::integer(std::string_view what) const {
    const double n = number(what);
    if (n != std::floor(n) || std::abs(n) > 1e9) {
        wrong(what, "an integer");
    }
    return static_cast<int>(n);
}

const std::string& Value::string(std::string_view what) const {
    if (kind_ != Kind::String) {
        wrong(what, "a string");
    }
    return string_;
}

const std::vector<Value>& Value::array(std::string_view what) const {
    if (kind_ != Kind::Array) {
        wrong(what, "an array");
    }
    return array_;
}

bool Value::has(std::string_view key) const {
    return kind_ == Kind::Object && object_.find(key) != object_.end();
}

const Value& Value::at(std::string_view key) const {
    if (kind_ != Kind::Object) {
        wrong(key, "an object holding it");
    }
    const auto found = object_.find(key);
    if (found == object_.end()) {
        throw ParseError("engine model JSON: missing '" + std::string(key) + "'");
    }
    return found->second;
}

}  // namespace mechsim::physics::native::json
