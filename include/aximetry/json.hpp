// Minimal JSON reader/writer so the project needs no third-party libraries.
// Accepts // and /* */ comments in input, which lets rover and mission files
// carry teaching notes right next to the numbers.
#pragma once

#include <string>
#include <utility>
#include <vector>

namespace aximetry::json {

class Value {
public:
    enum class Type { Null, Bool, Number, String, Array, Object };
    using Object = std::vector<std::pair<std::string, Value>>;
    using Array = std::vector<Value>;

    Value() = default;
    Value(bool b) : type_(Type::Bool), b_(b) {}
    Value(double d) : type_(Type::Number), n_(d) {}
    Value(int i) : type_(Type::Number), n_(static_cast<double>(i)) {}
    Value(const char* s) : type_(Type::String), s_(s) {}
    Value(std::string s) : type_(Type::String), s_(std::move(s)) {}

    static Value array() { Value v; v.type_ = Type::Array; return v; }
    static Value object() { Value v; v.type_ = Type::Object; return v; }

    Type type() const { return type_; }
    bool is_null() const { return type_ == Type::Null; }
    bool is_number() const { return type_ == Type::Number; }
    bool is_string() const { return type_ == Type::String; }
    bool is_array() const { return type_ == Type::Array; }
    bool is_object() const { return type_ == Type::Object; }
    bool is_bool() const { return type_ == Type::Bool; }

    double as_number() const;
    bool as_bool() const;
    const std::string& as_string() const;
    const Array& as_array() const;
    const Object& as_object() const;

    bool has(const std::string& key) const;
    const Value& at(const std::string& key) const;  // throws if missing
    const Value& operator[](size_t i) const;
    size_t size() const;

    // Builders
    Value& set(const std::string& key, Value v);
    void push_back(Value v);

    // Convenience getters with defaults (for optional config fields).
    double num(const std::string& key, double def) const;
    int integer(const std::string& key, int def) const;
    std::string str(const std::string& key, const std::string& def) const;
    bool boolean(const std::string& key, bool def) const;

private:
    Type type_ = Type::Null;
    bool b_ = false;
    double n_ = 0.0;
    std::string s_;
    Array arr_;
    Object obj_;
};

Value parse(const std::string& text, const std::string& source_name = "<string>");
Value parse_file(const std::string& path);
// indent < 0 → compact single line.
std::string dump(const Value& v, int indent = 2);
// Format a double compactly for JSON output (no trailing zeros, finite only).
std::string number_to_string(double d, int precision = 6);
std::string escape_string(const std::string& s);

}  // namespace aximetry::json
