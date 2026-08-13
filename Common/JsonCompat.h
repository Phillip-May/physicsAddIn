#pragma once

#include <nlohmann/json.hpp>

#include <string>


using Json = nlohmann::json;

namespace jsoncompat {

inline const Json& nullValue() {
    static const Json value;
    return value;
}

inline const Json& emptyArray() {
    static const Json value = Json::array();
    return value;
}

inline const Json& emptyObject() {
    static const Json value = Json::object();
    return value;
}

// Looks a key up without inserting it. operator[] on a non-const json would create the key.
inline const Json& member(const Json& object, const char* key) {
    if (!object.is_object()) return nullValue();
    const auto it = object.find(key);
    return it == object.end() ? nullValue() : *it;
}

inline bool contains(const Json& object, const char* key) {
    return object.is_object() && object.find(key) != object.end();
}

// --- value forms ---

inline double toDouble(const Json& value, double fallback = 0.0) {
    return value.is_number() ? value.get<double>() : fallback;
}

inline int toInt(const Json& value, int fallback = 0) {
    if (!value.is_number()) return fallback;
    const double number = value.get<double>();
    if (static_cast<double>(static_cast<int>(number)) != number) return fallback;
    return static_cast<int>(number);
}

inline bool toBool(const Json& value, bool fallback = false) {
    return value.is_boolean() ? value.get<bool>() : fallback;
}

inline std::string toString(const Json& value, const std::string& fallback = std::string()) {
    return value.is_string() ? value.get<std::string>() : fallback;
}

inline const Json& toArray(const Json& value) {
    return value.is_array() ? value : emptyArray();
}

inline const Json& toObject(const Json& value) {
    return value.is_object() ? value : emptyObject();
}

// --- object plus key forms ---

inline double fieldDouble(const Json& object, const char* key, double fallback = 0.0) {
    return toDouble(member(object, key), fallback);
}

inline int fieldInt(const Json& object, const char* key, int fallback = 0) {
    return toInt(member(object, key), fallback);
}

inline bool fieldBool(const Json& object, const char* key, bool fallback = false) {
    return toBool(member(object, key), fallback);
}

inline std::string fieldString(const Json& object, const char* key,
                               const std::string& fallback = std::string()) {
    return toString(member(object, key), fallback);
}

inline const Json& fieldArray(const Json& object, const char* key) {
    return toArray(member(object, key));
}

inline const Json& fieldObject(const Json& object, const char* key) {
    return toObject(member(object, key));
}

} // namespace jsoncompat
