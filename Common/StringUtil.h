#pragma once

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#ifndef CP_UTF8
// OCCT's Standard_Macro.hxx defines NONLS before any windows.h, which strips the NLS APIs —
// and winnls.h honors NONLS even when included directly, precisely to avoid the CompareString
// macro family OCCT collides with. Declare the one API wideToUtf8 needs instead.
#define CP_UTF8 65001
extern "C" __declspec(dllimport) int __stdcall WideCharToMultiByte(
    unsigned int codePage, unsigned long flags, const wchar_t* wide, int wideLen,
    char* out, int outLen, const char* defaultChar, int* usedDefaultChar);
#endif
#endif

// Qt-compatible text formatting, parsing and string operations.
//
// RobotSimulator's CLI output and its .robotprog.txt files are held to byte-identical
// equality across the Qt removal, and every number in them came from QString::number or
// QString::arg. These reproduce those rules exactly; tools/qt_number_format_equivalence.cpp
// checks them against Qt over 828 values and passes on all of 'g',17 and 'f',0/1/2/3/6.
//
// Never call setlocale. Everything here relies on the C locale, and strtod is locale
// sensitive. std::to_chars would avoid that, but this Emscripten ships no <charconv> at all.
namespace strutil {

// --- number formatting ---

// QString::number(value, 'g', 17). Used by programNumber(), so it decides the on-disk
// program format and every column of the trajectory CSV.
inline std::string formatShortest(double value) {
    // Qt drops the sign on negative zero; printf keeps it.
    if (value == 0.0) return "0";
    char buffer[64];
    std::snprintf(buffer, sizeof(buffer), "%.17g", value);
    return buffer;
}

// QString::number(value, 'f', precision) and .arg(value, 0, 'f', precision).
inline std::string formatFixed(double value, int precision) {
    if (std::isnan(value)) return "nan";
    if (std::isinf(value)) return value < 0.0 ? "-inf" : "inf";

    char buffer[2048];
    if (value == 0.0) {
        std::snprintf(buffer, sizeof(buffer), "%.*f", precision, 0.0);
        return buffer;
    }

    // Qt rounds exact halfway cases away from zero; MSVC's printf rounds half to even, so
    // 0.5 at 'f',0 is "1" for Qt and "0" for printf.
    constexpr int kExactDigits = 1080;  // enough for the smallest denormal
    std::snprintf(buffer, sizeof(buffer), "%.*f", kExactDigits, value);

    std::string text(buffer);
    bool negative = false;
    if (!text.empty() && text[0] == '-') {
        negative = true;
        text.erase(text.begin());
    }
    const size_t dot = text.find('.');
    std::string integerPart = text.substr(0, dot);
    const std::string fraction = text.substr(dot + 1);

    std::string kept = fraction.substr(0, static_cast<size_t>(precision));
    const bool roundUp = static_cast<size_t>(precision) < fraction.size() &&
                         fraction[static_cast<size_t>(precision)] >= '5';

    if (roundUp) {
        bool carry = true;
        for (int i = static_cast<int>(kept.size()) - 1; carry && i >= 0; --i) {
            if (kept[static_cast<size_t>(i)] == '9') {
                kept[static_cast<size_t>(i)] = '0';
            } else {
                ++kept[static_cast<size_t>(i)];
                carry = false;
            }
        }
        for (int i = static_cast<int>(integerPart.size()) - 1; carry && i >= 0; --i) {
            if (integerPart[static_cast<size_t>(i)] == '9') {
                integerPart[static_cast<size_t>(i)] = '0';
            } else {
                ++integerPart[static_cast<size_t>(i)];
                carry = false;
            }
        }
        if (carry) integerPart.insert(integerPart.begin(), '1');
    }

    std::string result = integerPart;
    if (precision > 0) result += "." + kept;
    if (negative) result.insert(result.begin(), '-');
    return result;
}

// QString::number(intValue) and QString::number(value, base).
inline std::string formatInt(long long value) { return std::to_string(value); }

inline std::string formatHex(unsigned long long value) {
    char buffer[32];
    std::snprintf(buffer, sizeof(buffer), "%llx", value);
    return buffer;
}

// --- QString::arg-compatible substitution ---

// Mimics QString("...").arg(...).arg(...) so the call sites keep their format string, their
// argument order and their precision specifiers exactly as they were.
class Format {
public:
    explicit Format(std::string pattern) : m_text(std::move(pattern)) {}

    Format& arg(const std::string& value) { return substitute(value); }
    Format& arg(const char* value) { return substitute(value ? value : ""); }
    Format& arg(int value) { return substitute(std::to_string(value)); }
    Format& arg(long value) { return substitute(std::to_string(value)); }
    Format& arg(long long value) { return substitute(std::to_string(value)); }
    Format& arg(unsigned value) { return substitute(std::to_string(value)); }
    Format& arg(unsigned long value) { return substitute(std::to_string(value)); }
    Format& arg(unsigned long long value) { return substitute(std::to_string(value)); }

    // QString::arg(double, fieldWidth, format, precision). Only 'f' and 'g' are used here.
    Format& arg(double value, int fieldWidth = 0, char format = 'g', int precision = -1) {
        std::string text;
        if (format == 'f') {
            text = formatFixed(value, precision < 0 ? 6 : precision);
        } else if (precision == 17) {
            text = formatShortest(value);
        } else {
            char buffer[64];
            std::snprintf(buffer, sizeof(buffer), "%.*g", precision < 0 ? 6 : precision, value);
            text = buffer;
            if (value == 0.0) text = "0";  // Qt drops the sign on negative zero
        }
        return substitute(pad(text, fieldWidth));
    }

    std::string str() const { return m_text; }
    operator std::string() const { return m_text; }

private:
    static std::string pad(const std::string& text, int fieldWidth) {
        const int width = fieldWidth < 0 ? -fieldWidth : fieldWidth;
        if (static_cast<int>(text.size()) >= width) return text;
        const std::string filler(static_cast<size_t>(width) - text.size(), ' ');
        return fieldWidth < 0 ? text + filler : filler + text;
    }

    Format& substitute(const std::string& value) {
        int lowest = -1;
        for (size_t i = 0; i + 1 < m_text.size(); ++i) {
            if (m_text[i] != '%' || m_text[i + 1] < '0' || m_text[i + 1] > '9') continue;
            int number = m_text[i + 1] - '0';
            if (i + 2 < m_text.size() && m_text[i + 2] >= '0' && m_text[i + 2] <= '9') {
                number = number * 10 + (m_text[i + 2] - '0');
            }
            if (lowest < 0 || number < lowest) lowest = number;
        }
        if (lowest < 0) return *this;

        const std::string token = "%" + std::to_string(lowest);
        std::string result;
        size_t at = 0;
        while (true) {
            const size_t hit = m_text.find(token, at);
            if (hit == std::string::npos) {
                result += m_text.substr(at);
                break;
            }
            // "%1" must not match inside "%12".
            const size_t after = hit + token.size();
            if (after < m_text.size() && m_text[after] >= '0' && m_text[after] <= '9') {
                result += m_text.substr(at, after - at);
                at = after;
                continue;
            }
            result += m_text.substr(at, hit - at);
            result += value;
            at = after;
        }
        m_text = result;
        return *this;
    }

    std::string m_text;
};

inline Format format(std::string pattern) { return Format(std::move(pattern)); }

// --- number parsing ---

inline bool parseDouble(const std::string& text, double* out) {
    if (text.empty()) return false;

    // Skip leading space and sign to look for a hex prefix.
    size_t probe = text.find_first_not_of(" \t\n\r\f\v");
    if (probe == std::string::npos) return false;
    if (text[probe] == '+' || text[probe] == '-') ++probe;
    if (probe + 1 < text.size() && text[probe] == '0' &&
        (text[probe + 1] == 'x' || text[probe + 1] == 'X')) {
        return false;
    }

    const char* begin = text.c_str();
    char* end = nullptr;
    errno = 0;
    const double value = std::strtod(begin, &end);
    if (end == begin) return false;
    // Qt rejects out-of-range in both directions: "1e400" and "1e-400" both report failure.
    if (errno == ERANGE) return false;
    while (*end == ' ' || *end == '\t' || *end == '\r' || *end == '\n' || *end == '\f' ||
           *end == '\v') {
        ++end;
    }
    if (*end != '\0') return false;
    if (out) *out = value;
    return true;
}

inline int parseIntOr(const std::string& text, int fallback) {
    if (text.empty()) return fallback;
    const char* begin = text.c_str();
    char* end = nullptr;
    errno = 0;
    const long long value = std::strtoll(begin, &end, 10);
    if (end == begin) return fallback;
    if (errno == ERANGE) return fallback;
    if (value < -2147483648LL || value > 2147483647LL) return fallback;
    while (*end == ' ' || *end == '\t' || *end == '\r' || *end == '\n' || *end == '\f' ||
           *end == '\v') {
        ++end;
    }
    if (*end != '\0') return fallback;
    return static_cast<int>(value);
}

// --- string operations ---

// QString::trimmed(): strips ASCII whitespace from both ends.
inline std::string trimmed(const std::string& text) {
    const size_t first = text.find_first_not_of(" \t\n\r\f\v");
    if (first == std::string::npos) return std::string();
    const size_t last = text.find_last_not_of(" \t\n\r\f\v");
    return text.substr(first, last - first + 1);
}

// QString::split(' ', Qt::SkipEmptyParts): splits on the literal space only. An
// istringstream would also break on tabs, which this must not do.
inline std::vector<std::string> splitSkippingEmpty(const std::string& text, char separator) {
    std::vector<std::string> parts;
    size_t start = 0;
    while (start <= text.size()) {
        const size_t next = text.find(separator, start);
        const size_t end = next == std::string::npos ? text.size() : next;
        if (end > start) parts.push_back(text.substr(start, end - start));
        if (next == std::string::npos) break;
        start = next + 1;
    }
    return parts;
}

// splitSkippingEmpty with double-quoted spans held together, so an instruction can carry free text.
inline std::vector<std::string> splitQuotedTokens(const std::string& text,
                                                  bool* outUnterminated = nullptr) {
    if (outUnterminated) *outUnterminated = false;
    std::vector<std::string> parts;
    std::string current;
    bool inQuotes = false;
    bool haveToken = false;   // distinguishes an empty quoted token, "", from no token at all

    for (size_t i = 0; i < text.size(); ++i) {
        const char c = text[i];
        if (inQuotes && c == '\\' && i + 1 < text.size()) {
            current.push_back(text[i + 1]);
            ++i;
            continue;
        }
        if (c == '"') {
            inQuotes = !inQuotes;
            haveToken = true;
            continue;
        }
        if (!inQuotes && (c == ' ' || c == '\t')) {
            if (haveToken || !current.empty()) {
                parts.push_back(current);
                current.clear();
                haveToken = false;
            }
            continue;
        }
        current.push_back(c);
        haveToken = true;
    }
    if (haveToken || !current.empty()) parts.push_back(current);
    if (inQuotes && outUnterminated) *outUnterminated = true;
    return parts;
}

inline std::string toLower(std::string text) {
    for (char& c : text) {
        if (c >= 'A' && c <= 'Z') c = static_cast<char>(c - 'A' + 'a');
    }
    return text;
}

inline bool startsWith(const std::string& text, const std::string& prefix) {
    return text.size() >= prefix.size() && text.compare(0, prefix.size(), prefix) == 0;
}

inline bool endsWith(const std::string& text, const std::string& suffix) {
    return text.size() >= suffix.size() &&
           text.compare(text.size() - suffix.size(), suffix.size(), suffix) == 0;
}

inline bool contains(const std::string& haystack, const std::string& needle) {
    return haystack.find(needle) != std::string::npos;
}

// QString::contains(..., Qt::CaseInsensitive).
inline bool containsCaseInsensitive(const std::string& haystack, const std::string& needle) {
    return toLower(haystack).find(toLower(needle)) != std::string::npos;
}

inline bool equalsCaseInsensitive(const std::string& a, const std::string& b) {
    return a.size() == b.size() && toLower(a) == toLower(b);
}

// QStringList::removeDuplicates(): keeps the FIRST occurrence of each value and preserves the
// original order.
inline void removeDuplicates(std::vector<std::string>& values) {
    std::vector<std::string> unique;
    unique.reserve(values.size());
    for (const std::string& value : values) {
        bool seen = false;
        for (const std::string& kept : unique) {
            if (kept == value) {
                seen = true;
                break;
            }
        }
        if (!seen) unique.push_back(value);
    }
    values = std::move(unique);
}

inline std::string join(const std::vector<std::string>& parts, const std::string& separator) {
    std::string result;
    for (size_t i = 0; i < parts.size(); ++i) {
        if (i) result += separator;
        result += parts[i];
    }
    return result;
}

// Keeps QStringList-style appends readable where a std::vector<std::string> replaced one.
// Pull into scope with `using strutil::operator<<;`.
inline std::vector<std::string>& operator<<(std::vector<std::string>& lines, std::string line) {
    lines.push_back(std::move(line));
    return lines;
}

#if defined(_WIN32)
// QString::fromWCharArray's replacement: narrows the UTF-16 the Windows common dialogs hand
// back. This is the step that can silently truncate a path or mangle a non-ASCII one, so
// tools/wide_to_utf8_check.cpp exercises this exact definition.
inline std::string wideToUtf8(const wchar_t* text) {
    if (text == nullptr || text[0] == L'\0') return std::string();
    // The returned length includes the terminator, so 1 means an empty string.
    const int needed = WideCharToMultiByte(CP_UTF8, 0, text, -1, nullptr, 0, nullptr, nullptr);
    if (needed <= 1) return std::string();
    std::string result(static_cast<size_t>(needed - 1), '\0');
    WideCharToMultiByte(CP_UTF8, 0, text, -1, result.data(), needed, nullptr, nullptr);
    return result;
}
#endif

} // namespace strutil
