#include "StringUtil.h"

#include <QString>

#include <cmath>
#include <cstdio>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace {

std::vector<double> buildCorpus() {
    std::vector<double> corpus = {
        0.0, -0.0, 1.0, -1.0, 0.1, 0.5, -0.5, 1.5, 2.5, -1.5, -2.5,
        0.05, 0.15, 0.25, 0.35, 0.45, 0.055, 0.045, 0.125, 0.375,
        2.0 / 3.0, 1.0 / 3.0, 3.14159265358979323846, 2.718281828459045,
        90.0, 180.0, 360.0, -90.0, 0.017453292519943295,
        100.0, 1000.0, 123456789.0, 0.000123456789,
        1e-5, 1e-4, 1e15, 1e16, 1e17, 1e21, 1e22, 1e300, 1e-300, 5e-324,
        std::numeric_limits<double>::min(),
        std::numeric_limits<double>::max(),
        std::numeric_limits<double>::epsilon(),
        std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::quiet_NaN(),
    };
    // Exact and near ties at every precision the codebase formats with.
    for (int i = 0; i <= 40; ++i) {
        corpus.push_back(i + 0.5);
        corpus.push_back(-(i + 0.5));
        corpus.push_back(i + 0.05);
        corpus.push_back(i + 0.005);
        corpus.push_back(i + 0.0005);
        corpus.push_back(i + 0.25);
        corpus.push_back(i + 0.75);
    }
    // Small negatives that round to zero, where the sign is easy to get wrong.
    for (double value : {-0.0001, -0.001, -0.01, -0.4, -0.04, -0.004, -0.0004, -0.49999}) {
        corpus.push_back(value);
    }
    // Joint angles in radians, TCP millimetres, speeds.
    for (int i = -180; i <= 180; i += 3) {
        corpus.push_back(static_cast<double>(i) * 0.017453292519943295);
        corpus.push_back(static_cast<double>(i) * 1.37);
        corpus.push_back(static_cast<double>(i) / 7.0);
        corpus.push_back(static_cast<double>(i) * 0.001);
    }
    return corpus;
}

int checkFormat(const std::vector<double>& corpus, char qtFormat, int precision,
                const char* label) {
    int mismatches = 0;
    for (double value : corpus) {
        const std::string expected =
            QString::number(value, qtFormat, precision).toStdString();
        const std::string actual = (qtFormat == 'g') ? strutil::formatShortest(value)
                                                     : strutil::formatFixed(value, precision);
        if (expected == actual) continue;
        if (mismatches < 8) {
            char raw[64];
            std::snprintf(raw, sizeof(raw), "%.17g", value);
            std::cout << "    " << raw << " -> qt=\"" << expected << "\" ours=\"" << actual
                      << "\"" << std::endl;
        }
        ++mismatches;
    }
    std::cout << label << ": " << mismatches << " / " << corpus.size() << " mismatches"
              << std::endl;
    return mismatches;
}

} // namespace

int main() {
    const std::vector<double> corpus = buildCorpus();
    int total = 0;
    total += checkFormat(corpus, 'g', 17, "'g',17");
    total += checkFormat(corpus, 'f', 0, "'f',0 ");
    total += checkFormat(corpus, 'f', 1, "'f',1 ");
    total += checkFormat(corpus, 'f', 2, "'f',2 ");
    total += checkFormat(corpus, 'f', 3, "'f',3 ");
    total += checkFormat(corpus, 'f', 6, "'f',6 ");

    std::cout << std::endl
              << (total == 0 ? "PASS: formatters match Qt exactly"
                             : "FAIL: formatters diverge from Qt")
              << std::endl;
    return total == 0 ? 0 : 1;
}
