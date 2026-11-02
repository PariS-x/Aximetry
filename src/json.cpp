#include "aximetry/json.hpp"

#include <cctype>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace aximetry::json {

namespace {
const char* type_name(Value::Type t) {
    switch (t) {
        case Value::Type::Null: return "null";
        case Value::Type::Bool: return "bool";
        case Value::Type::Number: return "number";
        case Value::Type::String: return "string";
        case Value::Type::Array: return "array";
        case Value::Type::Object: return "object";
    }
    return "?";
}
[[noreturn]] void type_error(const char* want, Value::Type got) {
    throw std::runtime_error(std::string("JSON: expected ") + want + ", got " + type_name(got));
}
}  // namespace

double Value::as_number() const { if (type_ != Type::Number) type_error("number", type_); return n_; }
bool Value::as_bool() const { if (type_ != Type::Bool) type_error("bool", type_); return b_; }
const std::string& Value::as_string() const { if (type_ != Type::String) type_error("string", type_); return s_; }
const Value::Array& Value::as_array() const { if (type_ != Type::Array) type_error("array", type_); return arr_; }
const Value::Object& Value::as_object() const { if (type_ != Type::Object) type_error("object", type_); return obj_; }

bool Value::has(const std::string& key) const {
    if (type_ != Type::Object) return false;
    for (const auto& kv : obj_)
        if (kv.first == key) return true;
    return false;
}

const Value& Value::at(const std::string& key) const {
    if (type_ != Type::Object) type_error("object", type_);
    for (const auto& kv : obj_)
        if (kv.first == key) return kv.second;
    throw std::runtime_error("JSON: missing required key \"" + key + "\"");
}

const Value& Value::operator[](size_t i) const {
    if (type_ != Type::Array) type_error("array", type_);
    if (i >= arr_.size()) throw std::runtime_error("JSON: array index out of range");
    return arr_[i];
}

size_t Value::size() const {
    if (type_ == Type::Array) return arr_.size();
    if (type_ == Type::Object) return obj_.size();
    return 0;
}

Value& Value::set(const std::string& key, Value v) {
    if (type_ == Type::Null) type_ = Type::Object;
    if (type_ != Type::Object) type_error("object", type_);
    for (auto& kv : obj_)
        if (kv.first == key) { kv.second = std::move(v); return kv.second; }
    obj_.emplace_back(key, std::move(v));
    return obj_.back().second;
}

void Value::push_back(Value v) {
    if (type_ == Type::Null) type_ = Type::Array;
    if (type_ != Type::Array) type_error("array", type_);
    arr_.push_back(std::move(v));
}

double Value::num(const std::string& key, double def) const {
    if (!has(key)) return def;
    const Value& v = at(key);
    if (v.is_null()) return def;
    if (!v.is_number()) throw std::runtime_error("JSON: key \"" + key + "\" must be a number");
    return v.as_number();
}
int Value::integer(const std::string& key, int def) const { return static_cast<int>(std::lround(num(key, def))); }
std::string Value::str(const std::string& key, const std::string& def) const {
    if (!has(key) || at(key).is_null()) return def;
    const Value& v = at(key);
    if (!v.is_string()) throw std::runtime_error("JSON: key \"" + key + "\" must be a string");
    return v.as_string();
}
bool Value::boolean(const std::string& key, bool def) const {
    if (!has(key) || at(key).is_null()) return def;
    return at(key).as_bool();
}

// ---------------------------------------------------------------- parser
namespace {
class Parser {
public:
    Parser(const std::string& t, std::string name) : s_(t), name_(std::move(name)) {}

    Value parse_document() {
        skip_ws();
        Value v = parse_value();
        skip_ws();
        if (pos_ != s_.size()) fail("unexpected trailing characters");
        return v;
    }

private:
    const std::string& s_;
    std::string name_;
    size_t pos_ = 0;

    [[noreturn]] void fail(const std::string& msg) const {
        int line = 1, col = 1;
        for (size_t i = 0; i < pos_ && i < s_.size(); ++i) {
            if (s_[i] == '\n') { ++line; col = 1; } else { ++col; }
        }
        throw std::runtime_error(name_ + ":" + std::to_string(line) + ":" + std::to_string(col) + ": JSON parse error: " + msg);
    }

    void skip_ws() {
        while (pos_ < s_.size()) {
            const char c = s_[pos_];
            if (c == ' ' || c == '\t' || c == '\n' || c == '\r') { ++pos_; continue; }
            if (c == '/' && pos_ + 1 < s_.size() && s_[pos_ + 1] == '/') {
                while (pos_ < s_.size() && s_[pos_] != '\n') ++pos_;
                continue;
            }
            if (c == '/' && pos_ + 1 < s_.size() && s_[pos_ + 1] == '*') {
                pos_ += 2;
                while (pos_ + 1 < s_.size() && !(s_[pos_] == '*' && s_[pos_ + 1] == '/')) ++pos_;
                if (pos_ + 1 >= s_.size()) fail("unterminated comment");
                pos_ += 2;
                continue;
            }
            break;
        }
    }

    bool consume(const char* lit) {
        size_t n = 0;
        while (lit[n]) ++n;
        if (s_.compare(pos_, n, lit) == 0) { pos_ += n; return true; }
        return false;
    }

    Value parse_value() {
        if (pos_ >= s_.size()) fail("unexpected end of input");
        const char c = s_[pos_];
        if (c == '{') return parse_object();
        if (c == '[') return parse_array();
        if (c == '"') return Value(parse_string());
        if (c == '-' || (c >= '0' && c <= '9')) return parse_number();
        if (consume("true")) return Value(true);
        if (consume("false")) return Value(false);
        if (consume("null")) return Value();
        fail(std::string("unexpected character '") + c + "'");
    }

    Value parse_object() {
        Value obj = Value::object();
        ++pos_;  // {
        skip_ws();
        if (pos_ < s_.size() && s_[pos_] == '}') { ++pos_; return obj; }
        while (true) {
            skip_ws();
            if (pos_ >= s_.size() || s_[pos_] != '"') fail("expected object key string");
            std::string key = parse_string();
            skip_ws();
            if (pos_ >= s_.size() || s_[pos_] != ':') fail("expected ':' after key");
            ++pos_;
            skip_ws();
            obj.set(key, parse_value());
            skip_ws();
            if (pos_ < s_.size() && s_[pos_] == ',') {
                ++pos_;
                skip_ws();
                if (pos_ < s_.size() && s_[pos_] == '}') { ++pos_; return obj; }  // tolerate trailing comma
                continue;
            }
            if (pos_ < s_.size() && s_[pos_] == '}') { ++pos_; return obj; }
            fail("expected ',' or '}' in object");
        }
    }

    Value parse_array() {
        Value arr = Value::array();
        ++pos_;  // [
        skip_ws();
        if (pos_ < s_.size() && s_[pos_] == ']') { ++pos_; return arr; }
        while (true) {
            skip_ws();
            arr.push_back(parse_value());
            skip_ws();
            if (pos_ < s_.size() && s_[pos_] == ',') {
                ++pos_;
                skip_ws();
                if (pos_ < s_.size() && s_[pos_] == ']') { ++pos_; return arr; }
                continue;
            }
            if (pos_ < s_.size() && s_[pos_] == ']') { ++pos_; return arr; }
            fail("expected ',' or ']' in array");
        }
    }

    std::string parse_string() {
        ++pos_;  // opening quote
        std::string out;
        while (pos_ < s_.size()) {
            char c = s_[pos_++];
            if (c == '"') return out;
            if (c == '\\') {
                if (pos_ >= s_.size()) fail("bad escape");
                char e = s_[pos_++];
                switch (e) {
                    case '"': out += '"'; break;
                    case '\\': out += '\\'; break;
                    case '/': out += '/'; break;
                    case 'b': out += '\b'; break;
                    case 'f': out += '\f'; break;
                    case 'n': out += '\n'; break;
                    case 'r': out += '\r'; break;
                    case 't': out += '\t'; break;
                    case 'u': {
                        if (pos_ + 4 > s_.size()) fail("bad \\u escape");
                        unsigned cp = std::stoul(s_.substr(pos_, 4), nullptr, 16);
                        pos_ += 4;
                        // Basic UTF-8 encode (BMP only; enough for config files).
                        if (cp < 0x80) out += static_cast<char>(cp);
                        else if (cp < 0x800) { out += static_cast<char>(0xC0 | (cp >> 6)); out += static_cast<char>(0x80 | (cp & 0x3F)); }
                        else { out += static_cast<char>(0xE0 | (cp >> 12)); out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F)); out += static_cast<char>(0x80 | (cp & 0x3F)); }
                        break;
                    }
                    default: fail("unknown escape");
                }
            } else {
                out += c;
            }
        }
        fail("unterminated string");
    }

    Value parse_number() {
        const size_t start = pos_;
        if (s_[pos_] == '-') ++pos_;
        while (pos_ < s_.size() && (std::isdigit(static_cast<unsigned char>(s_[pos_])) || s_[pos_] == '.' || s_[pos_] == 'e' ||
                                    s_[pos_] == 'E' || s_[pos_] == '+' || s_[pos_] == '-'))
            ++pos_;
        try {
            return Value(std::stod(s_.substr(start, pos_ - start)));
        } catch (...) {
            fail("bad number");
        }
    }
};

void dump_impl(const Value& v, std::ostringstream& os, int indent, int depth) {
    auto nl = [&](int d) {
        if (indent < 0) return;
        os << '\n';
        for (int i = 0; i < d * indent; ++i) os << ' ';
    };
    switch (v.type()) {
        case Value::Type::Null: os << "null"; break;
        case Value::Type::Bool: os << (v.as_bool() ? "true" : "false"); break;
        case Value::Type::Number: os << number_to_string(v.as_number()); break;
        case Value::Type::String: os << '"' << escape_string(v.as_string()) << '"'; break;
        case Value::Type::Array: {
            os << '[';
            const auto& a = v.as_array();
            for (size_t i = 0; i < a.size(); ++i) {
                if (i) os << ',';
                nl(depth + 1);
                dump_impl(a[i], os, indent, depth + 1);
            }
            if (!a.empty()) nl(depth);
            os << ']';
            break;
        }
        case Value::Type::Object: {
            os << '{';
            const auto& o = v.as_object();
            for (size_t i = 0; i < o.size(); ++i) {
                if (i) os << ',';
                nl(depth + 1);
                os << '"' << escape_string(o[i].first) << "\":" << (indent >= 0 ? " " : "");
                dump_impl(o[i].second, os, indent, depth + 1);
            }
            if (!o.empty()) nl(depth);
            os << '}';
            break;
        }
    }
}
}  // namespace

Value parse(const std::string& text, const std::string& source_name) {
    Parser p(text, source_name);
    return p.parse_document();
}

Value parse_file(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) throw std::runtime_error("cannot open file: " + path);
    std::stringstream ss;
    ss << in.rdbuf();
    return parse(ss.str(), path);
}

std::string dump(const Value& v, int indent) {
    std::ostringstream os;
    dump_impl(v, os, indent, 0);
    return os.str();
}

std::string number_to_string(double d, int precision) {
    if (!std::isfinite(d)) return "null";
    if (d == std::floor(d) && std::fabs(d) < 1e15) {
        char buf[32];
        std::snprintf(buf, sizeof(buf), "%.0f", d);
        return buf;
    }
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.*g", precision, d);
    return buf;
}

std::string escape_string(const std::string& s) {
    std::string out;
    out.reserve(s.size() + 2);
    for (char c : s) {
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if (static_cast<unsigned char>(c) < 0x20) {
                    char buf[8];
                    std::snprintf(buf, sizeof(buf), "\\u%04x", c);
                    out += buf;
                } else {
                    out += c;
                }
        }
    }
    return out;
}

}  // namespace aximetry::json
