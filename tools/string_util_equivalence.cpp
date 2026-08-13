#include "StringUtil.h"

#include <QString>
#include <QStringList>

#include <cstdio>
#include <string>
#include <vector>

namespace {

int g_failures = 0;

void check(bool ok, const std::string& what) {
    if (ok) return;
    std::printf("FAIL %s\n", what.c_str());
    ++g_failures;
}

} // namespace

int main() {
    const std::vector<std::string> samples = {
        "0", "1", "-1", "3.5", "-2.25", "1e10", "1E-10", "0.000123", ".5", "5.",
        "  7  ", "\t8\t", "9\r\n", "", " ", "abc", "1abc", "abc1", "1.2.3", "--1",
        "+3", "1 2", "0x10", "inf", "-inf", "nan", "2147483647", "-2147483648",
        "4294967296", "1e400", "0.1", "  -4.5  ",
        // Overflow, underflow, hex and unsigned wrap: each of these disagreed on the first run.
        "1e-400", "0X1p3", " 0x10", "18446744073709551616", "-4294967296",
        "2147483648", "-2147483649", "1e309", "-1e309", "0e0",
        // Denormal: rejecting all of ERANGE must not be over-broad.
        "5e-324", "2.2250738585072014e-308", "1.7976931348623157e308",
    };

    for (const std::string& sample : samples) {
        const QString qt = QString::fromStdString(sample);

        // toDouble
        bool qtOk = false;
        const double qtValue = qt.toDouble(&qtOk);
        double ourValue = 0.0;
        const bool ourOk = strutil::parseDouble(sample, &ourValue);
        check(qtOk == ourOk, "parseDouble ok flag for \"" + sample + "\": qt " +
                                 std::to_string(qtOk) + " ours " + std::to_string(ourOk));
        if (qtOk && ourOk) {
            check(qtValue == ourValue || (qtValue != qtValue && ourValue != ourValue),
                  "parseDouble value for \"" + sample + "\"");
        }

        // toInt with no ok pointer
        const int qtInt = qt.toInt();
        const int ourInt = strutil::parseIntOr(sample, 0);
        check(qtInt == ourInt, "parseIntOr for \"" + sample + "\": qt " +
                                   std::to_string(qtInt) + " ours " + std::to_string(ourInt));

        // trimmed
        check(qt.trimmed().toStdString() == strutil::trimmed(sample),
              "trimmed for \"" + sample + "\"");

        // toLower
        check(qt.toLower().toStdString() == strutil::toLower(sample),
              "toLower for \"" + sample + "\"");
    }

    const std::vector<std::string> lines = {
        "MoveJ 0 0 0 0 0 0",
        "  MoveJ   1.5    2.5  ",
        "MoveJ\t1\t2",
        "SetSpeed 30 100",
        "",
        "   ",
        "single",
    };
    for (const std::string& line : lines) {
        const QStringList qtParts =
            QString::fromStdString(line).split(' ', Qt::SkipEmptyParts);
        const std::vector<std::string> ourParts = strutil::splitSkippingEmpty(line, ' ');
        check(static_cast<size_t>(qtParts.size()) == ourParts.size(),
              "split count for \"" + line + "\": qt " + std::to_string(qtParts.size()) +
                  " ours " + std::to_string(ourParts.size()));
        const size_t count = std::min<size_t>(static_cast<size_t>(qtParts.size()), ourParts.size());
        for (size_t i = 0; i < count; ++i) {
            check(qtParts[static_cast<int>(i)].toStdString() == ourParts[i],
                  "split part " + std::to_string(i) + " of \"" + line + "\"");
        }
    }

    // startsWith / endsWith / contains, case sensitive and not.
    struct Pair { const char* haystack; const char* needle; };
    const Pair pairs[] = {
        {"Timed out waiting for serial response.", "Timed out"},
        {"timed out", "Timed out"},
        {"axis_3_link", "axis"},
        {"AXIS_3_LINK", "axis"},
        {"joint_axis_1.meshbin", ".meshbin"},
        {"robot.json", ".JSON"},
        {"", "x"},
        {"x", ""},
    };
    for (const Pair& pair : pairs) {
        const QString haystack = QString::fromUtf8(pair.haystack);
        const QString needle = QString::fromUtf8(pair.needle);
        check(haystack.startsWith(needle) == strutil::startsWith(pair.haystack, pair.needle),
              std::string("startsWith \"") + pair.haystack + "\" / \"" + pair.needle + "\"");
        check(haystack.endsWith(needle) == strutil::endsWith(pair.haystack, pair.needle),
              std::string("endsWith \"") + pair.haystack + "\" / \"" + pair.needle + "\"");
        check(haystack.contains(needle) == strutil::contains(pair.haystack, pair.needle),
              std::string("contains \"") + pair.haystack + "\" / \"" + pair.needle + "\"");
        check(haystack.contains(needle, Qt::CaseInsensitive) ==
                  strutil::containsCaseInsensitive(pair.haystack, pair.needle),
              std::string("containsCaseInsensitive \"") + pair.haystack + "\" / \"" + pair.needle + "\"");
    }

    {
        // Seven placeholders with mixed precision, as describeRobotJointAxes uses.
        check(QString("J%1: origin=(%2, %3, %4), direction=(%5, %6, %7)")
                  .arg(3).arg(1.5, 0, 'f', 3).arg(-2.25, 0, 'f', 3).arg(0.0, 0, 'f', 3)
                  .arg(0.7071, 0, 'f', 6).arg(-0.5, 0, 'f', 6).arg(0.0, 0, 'f', 6).toStdString() ==
              strutil::format("J%1: origin=(%2, %3, %4), direction=(%5, %6, %7)")
                  .arg(3).arg(1.5, 0, 'f', 3).arg(-2.25, 0, 'f', 3).arg(0.0, 0, 'f', 3)
                  .arg(0.7071, 0, 'f', 6).arg(-0.5, 0, 'f', 6).arg(0.0, 0, 'f', 6).str(),
              "arg: seven placeholders with mixed precision");

        // Data-dependent precision, as the blend feasibility warnings use.
        for (double v : {0.125, 2.5, 9.99, 10.0, 12.345, 0.0, -3.5}) {
            const int precision = v < 10.0 ? 2 : 1;
            check(QString("trim %1 mm of %2").arg(v, 0, 'f', precision)
                      .arg(v * 2, 0, 'f', precision).toStdString() ==
                  strutil::format("trim %1 mm of %2").arg(v, 0, 'f', precision)
                      .arg(v * 2, 0, 'f', precision).str(),
                  "arg: data-dependent precision for " + std::to_string(v));
        }

        check(QString("%2 then %1 then %2").arg("a").arg("b").toStdString() ==
                  strutil::format("%2 then %1 then %2").arg("a").arg("b").str(),
              "arg: repeated and out-of-order placeholders");
        check(QString("%1 and %10").arg(1).arg(2).toStdString() ==
                  strutil::format("%1 and %10").arg(1).arg(2).str(),
              "arg: %1 must not match inside %10");
        check(QString("%1").arg("%1 literal").toStdString() ==
                  strutil::format("%1").arg("%1 literal").str(),
              "arg: a replacement is never rescanned for placeholders");
        check(QString("%1 instruction(s)").arg(static_cast<qulonglong>(7)).toStdString() ==
                  strutil::format("%1 instruction(s)").arg(static_cast<unsigned long long>(7)).str(),
              "arg: unsigned");
        check(QString("Joint %1 deg/s, Linear %2 mm/s").arg(30.0, 0, 'f', 1)
                  .arg(100.0, 0, 'f', 1).toStdString() ==
              strutil::format("Joint %1 deg/s, Linear %2 mm/s").arg(30.0, 0, 'f', 1)
                  .arg(100.0, 0, 'f', 1).str(),
              "arg: SetSpeed detail line");
        check(QString("%1/6 valid").arg(4).toStdString() ==
                  strutil::format("%1/6 valid").arg(4).str(),
              "arg: integer followed by a slash");
    }

    std::printf("%s\n", g_failures == 0 ? "PASS: StringUtil matches QString"
                                        : "FAIL: StringUtil diverges from QString");
    return g_failures == 0 ? 0 : 1;
}
