#include "JsonCompat.h"

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonValue>

#include <cstdio>
#include <string>
#include <vector>

namespace {

int g_failures = 0;

void check(bool ok, const char* what, const std::string& detail) {
    if (ok) return;
    std::printf("FAIL %s: %s\n", what, detail.c_str());
    ++g_failures;
}

} // namespace

int main() {
    // One document holding every shape the readers meet: right type, wrong type, absent.
    const char* text = R"({
        "aDouble": 100.5,
        "aWholeDouble": 42.0,
        "anInt": 7,
        "aNegative": -13,
        "aBigNumber": 1e18,
        "aString": "link_1",
        "aNumericString": "3.5",
        "aTrue": true,
        "aFalse": false,
        "anArray": [1, 2, 3],
        "anObject": {"nested": 1},
        "aNull": null
    })";

    const Json mine = Json::parse(text);
    const QJsonObject qt = QJsonDocument::fromJson(QByteArray(text)).object();

    const std::vector<std::string> keys = {
        "aDouble", "aWholeDouble", "anInt", "aNegative", "aBigNumber", "aString",
        "aNumericString", "aTrue", "aFalse", "anArray", "anObject", "aNull", "absentKey"};

    for (const std::string& key : keys) {
        const char* k = key.c_str();
        const QJsonValue qtValue = qt.value(QString::fromStdString(key));

        // Defaults chosen to be distinguishable from any real value in the document.
        const double defaultDouble = -999.25;
        const int defaultInt = -777;
        const bool defaultBool = true;
        const std::string defaultString = "<fallback>";

        check(jsoncompat::fieldDouble(mine, k, defaultDouble) == qtValue.toDouble(defaultDouble),
              "toDouble", key + ": ours " + std::to_string(jsoncompat::fieldDouble(mine, k, defaultDouble)) +
              " qt " + std::to_string(qtValue.toDouble(defaultDouble)));

        check(jsoncompat::fieldInt(mine, k, defaultInt) == qtValue.toInt(defaultInt),
              "toInt", key + ": ours " + std::to_string(jsoncompat::fieldInt(mine, k, defaultInt)) +
              " qt " + std::to_string(qtValue.toInt(defaultInt)));

        check(jsoncompat::fieldBool(mine, k, defaultBool) == qtValue.toBool(defaultBool),
              "toBool", key + ": ours " + std::to_string(jsoncompat::fieldBool(mine, k, defaultBool)) +
              " qt " + std::to_string(qtValue.toBool(defaultBool)));

        check(jsoncompat::fieldString(mine, k, defaultString) ==
                  qtValue.toString(QString::fromStdString(defaultString)).toStdString(),
              "toString", key + ": ours '" + jsoncompat::fieldString(mine, k, defaultString) +
              "' qt '" + qtValue.toString(QString::fromStdString(defaultString)).toStdString() + "'");

        check(static_cast<int>(jsoncompat::fieldArray(mine, k).size()) == qtValue.toArray().size(),
              "toArray size", key);

        check(static_cast<int>(jsoncompat::fieldObject(mine, k).size()) == qtValue.toObject().size(),
              "toObject size", key);

        check(jsoncompat::contains(mine, k) == qt.contains(QString::fromStdString(key)),
              "contains", key);
    }

    // The non-integral rule specifically, since 47 call sites depend on it.
    {
        const Json fractional = Json::parse(R"({"v": 2.5})");
        const QJsonObject qtFractional = QJsonDocument::fromJson(QByteArray(R"({"v": 2.5})")).object();
        const int ours = jsoncompat::fieldInt(fractional, "v", 0);
        const int theirs = qtFractional.value("v").toInt(0);
        check(ours == theirs, "toInt on 2.5",
              "ours " + std::to_string(ours) + " qt " + std::to_string(theirs) +
              " (Qt returns the fallback rather than truncating)");
        std::printf("toInt(2.5, fallback 0): ours %d, qt %d\n", ours, theirs);
    }

    // Array element access, the other form the readers use.
    {
        const Json array = Json::parse(R"([1, "two", 3.5, true, null])");
        const QJsonArray qtArray = QJsonDocument::fromJson(QByteArray(R"([1, "two", 3.5, true, null])")).array();
        for (size_t i = 0; i < array.size(); ++i) {
            const QJsonValue qtValue = qtArray.at(static_cast<int>(i));
            check(jsoncompat::toDouble(array[i], -1.0) == qtValue.toDouble(-1.0),
                  "array toDouble", std::to_string(i));
            check(jsoncompat::toInt(array[i], -1) == qtValue.toInt(-1),
                  "array toInt", std::to_string(i));
            check(jsoncompat::toString(array[i], "<f>") == qtValue.toString("<f>").toStdString(),
                  "array toString", std::to_string(i));
        }
    }

    // Malformed input has to be detectable without throwing, the way fromJson reported errors.
    {
        const Json bad = Json::parse("{not json", nullptr, false);
        check(bad.is_discarded(), "parse failure is discarded, not thrown", "");
        QJsonParseError error{};
        QJsonDocument::fromJson(QByteArray("{not json"), &error);
        check(error.error != QJsonParseError::NoError, "qt also rejects it", "");
    }

    std::printf("%s\n", g_failures == 0 ? "PASS: JsonCompat matches QJsonValue"
                                        : "FAIL: JsonCompat diverges from QJsonValue");
    return g_failures == 0 ? 0 : 1;
}
