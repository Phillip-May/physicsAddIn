#include "StringUtil.h"

#include <cstdio>
#include <string>

using strutil::wideToUtf8;

namespace {

int g_failures = 0;

void expect(const wchar_t* input, const std::string& expected, const char* what) {
    const std::string actual = wideToUtf8(input);
    if (actual == expected) return;
    std::printf("FAIL %s: got %zu bytes \"%s\", expected %zu bytes \"%s\"\n", what,
                actual.size(), actual.c_str(), expected.size(), expected.c_str());
    ++g_failures;
}

} // namespace

int main() {
    expect(nullptr, "", "null pointer");
    expect(L"", "", "empty string");
    expect(L"C:\\Users\\augme\\robot.zip", "C:\\Users\\augme\\robot.zip", "plain ASCII path");
    expect(L"C:\\path with spaces\\a b.robotprog.txt",
           "C:\\path with spaces\\a b.robotprog.txt", "spaces");

    expect(L"C:\\caf\u00e9\\robot.zip", "C:\\caf\xc3\xa9\\robot.zip", "Latin-1 supplement (2-byte)");
    expect(L"C:\\\u65e5\u672c\u8a9e\\robot.zip",
           "C:\\\xe6\x97\xa5\xe6\x9c\xac\xe8\xaa\x9e\\robot.zip", "CJK (3-byte)");
    // Surrogate pair: one code point, two wchar_t, four UTF-8 bytes.
    expect(L"C:\\\U0001F600\\robot.zip", "C:\\\xf0\x9f\x98\x80\\robot.zip", "emoji (surrogate pair)");

    std::wstring longPath = L"C:\\";
    for (int i = 0; i < 40; ++i) longPath += L"directory_segment\\";
    longPath += L"robot.zip";
    const std::string narrowed = wideToUtf8(longPath.c_str());
    if (narrowed.size() != longPath.size()) {
        std::printf("FAIL long ASCII path: %zu bytes from %zu characters\n", narrowed.size(),
                    longPath.size());
        ++g_failures;
    }

    std::printf("%s\n", g_failures == 0 ? "PASS: wideToUtf8 converts correctly"
                                        : "FAIL: wideToUtf8 is wrong");
    return g_failures == 0 ? 0 : 1;
}
