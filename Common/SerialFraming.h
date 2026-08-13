#pragma once


#include <stddef.h>
#include <stdint.h>
#include <string.h>

#if !defined(ARDUINO)
#include <string>
#endif

namespace SerialFraming {

// Separates the JSON from its checksum. '*' cannot appear in the JSON we emit: every string value
// is a fixed identifier and every other token is a number, so the last '*' on a line is always the
// separator.
constexpr char kChecksumSeparator = '*';
constexpr size_t kChecksumTextLength = 4;  // four hex digits

// CRC-16/CCITT-FALSE. Chosen over a sum for catching transposed and burst errors, and over CRC32
// for costing four bytes on the wire instead of eight; the payloads here are short enough that the
// extra detection strength of a 32-bit polynomial buys nothing measurable.
inline uint16_t crc16(const char* data, size_t length) {
    uint16_t crc = 0xFFFF;
    if (!data) return crc;
    for (size_t i = 0; i < length; ++i) {
        crc ^= static_cast<uint16_t>(static_cast<unsigned char>(data[i])) << 8;
        for (uint8_t bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x8000u) ? static_cast<uint16_t>((crc << 1) ^ 0x1021u)
                                  : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

inline char hexDigit(uint8_t value) {
    return value < 10 ? static_cast<char>('0' + value) : static_cast<char>('A' + (value - 10));
}

// Writes "*XXXX" for the payload into out, which must hold kChecksumTextLength + 2 bytes.
// Returns the number of characters written, not counting the terminator.
inline size_t formatChecksumSuffix(const char* payload, size_t payloadLength, char* out, size_t outSize) {
    if (!out || outSize < kChecksumTextLength + 2) return 0;
    const uint16_t crc = crc16(payload, payloadLength);
    out[0] = kChecksumSeparator;
    out[1] = hexDigit(static_cast<uint8_t>((crc >> 12) & 0x0F));
    out[2] = hexDigit(static_cast<uint8_t>((crc >> 8) & 0x0F));
    out[3] = hexDigit(static_cast<uint8_t>((crc >> 4) & 0x0F));
    out[4] = hexDigit(static_cast<uint8_t>(crc & 0x0F));
    out[5] = '\0';
    return kChecksumTextLength + 1;
}

inline bool parseHexDigit(char c, uint8_t* value) {
    if (c >= '0' && c <= '9') { *value = static_cast<uint8_t>(c - '0'); return true; }
    if (c >= 'A' && c <= 'F') { *value = static_cast<uint8_t>(c - 'A' + 10); return true; }
    if (c >= 'a' && c <= 'f') { *value = static_cast<uint8_t>(c - 'a' + 10); return true; }
    return false;
}

enum class CheckResult : uint8_t {
    // No suffix present. Accepted as-is: boot text and older peers have none.
    Unchecked,
    // Suffix present and matching.
    Valid,
    // Suffix present and wrong, or malformed. The only case that means corruption.
    Invalid
};

// Splits a received line. outPayloadLength is set to the JSON length with any suffix removed, so
// the caller parses only the part the checksum covered.
inline CheckResult verifyLine(const char* line, size_t length, size_t* outPayloadLength) {
    if (outPayloadLength) *outPayloadLength = length;
    if (!line || length == 0) return CheckResult::Unchecked;

    // Trailing whitespace is tolerated so a stray '\r' cannot fail an otherwise good line.
    while (length > 0 && (line[length - 1] == '\r' || line[length - 1] == '\n' ||
                          line[length - 1] == ' ' || line[length - 1] == '\t')) {
        --length;
    }
    if (length < kChecksumTextLength + 1) {
        if (outPayloadLength) *outPayloadLength = length;
        return CheckResult::Unchecked;
    }

    const size_t separatorIndex = length - (kChecksumTextLength + 1);
    if (line[separatorIndex] != kChecksumSeparator) {
        if (outPayloadLength) *outPayloadLength = length;
        return CheckResult::Unchecked;
    }

    uint16_t received = 0;
    for (size_t i = 0; i < kChecksumTextLength; ++i) {
        uint8_t nibble = 0;
        if (!parseHexDigit(line[separatorIndex + 1 + i], &nibble)) {
            if (outPayloadLength) *outPayloadLength = length;
            return CheckResult::Unchecked;
        }
        received = static_cast<uint16_t>((received << 4) | nibble);
    }

    if (outPayloadLength) *outPayloadLength = separatorIndex;
    return crc16(line, separatorIndex) == received ? CheckResult::Valid : CheckResult::Invalid;
}

#if !defined(ARDUINO)
// Host-side only: the firmware never adds a sequence number, it only reads one. Guarded the same
// way RobotMotionCore guards its host-only helpers, so the Teensy build does not pull in <string>.

// Inserts "seq":N as the last member of a JSON object. Returns the payload untouched if it does not
// look like an object, so a malformed caller cannot produce a line that fails to parse at the far
// end and looks like a link fault.
inline std::string withSequence(const std::string& payload, int32_t seq) {
    if (seq < 0 || payload.size() < 2) return payload;
    // Trailing whitespace would put the insert in the wrong place.
    size_t end = payload.size();
    while (end > 0 && (payload[end - 1] == ' ' || payload[end - 1] == '\t' ||
                       payload[end - 1] == '\r' || payload[end - 1] == '\n')) {
        --end;
    }
    if (end < 2 || payload[end - 1] != '}') return payload;

    // An empty object takes no leading comma; anything else does.
    size_t firstNonSpace = 1;
    while (firstNonSpace < end - 1 && (payload[firstNonSpace] == ' ' || payload[firstNonSpace] == '\t')) {
        ++firstNonSpace;
    }
    const bool empty = firstNonSpace >= end - 1;

    std::string out = payload.substr(0, end - 1);
    if (!empty) out += ',';
    out += "\"seq\":";
    out += std::to_string(seq);
    out += '}';
    return out;
}
#endif  // !ARDUINO

}  // namespace SerialFraming
