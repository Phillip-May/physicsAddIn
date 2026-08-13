#include "../Common/OrientationFormat.h"

#include <cmath>
#include <cstdio>
#include <vector>

namespace {

int g_failures = 0;

double worstDifference(const CadTransform& a, const CadTransform& b) {
    double worst = 0.0;
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            const size_t i = static_cast<size_t>(row * 4 + column);
            worst = std::max(worst, std::abs(a.values[i] - b.values[i]));
        }
    }
    return worst;
}

void checkRotation(const orientation::Format& format, const CadTransform& rotation, const char* what,
                   double tolerance = 1.0e-6) {
    const std::array<double, 4> shown = orientation::valuesFromRotation(format, rotation);
    const CadTransform recomposed = orientation::rotationFromValues(format, shown);
    const double worst = worstDifference(rotation, recomposed);
    if (worst > tolerance) {
        std::printf("  FAIL %-44s %-26s shown (%9.4f %9.4f %9.4f %9.4f) worst %.3e\n",
                    format.label, what, shown[0], shown[1], shown[2], shown[3], worst);
        ++g_failures;
    }
}

std::vector<CadTransform> sweepRotations() {
    static const orientation::Format zyx = {
        "sweep", orientation::Kind::EulerIntrinsic, {2, 1, 0}, false, 3, {"", "", "", ""}, "", 1.0, 1.0};
    const double values[] = {-180.0, -157.0, -90.0, -43.0, -0.5, 0.0, 0.5, 43.0, 90.0, 157.0, 180.0};
    std::vector<CadTransform> rotations;
    for (double a : values) {
        for (double b : values) {
            for (double c : values) {
                rotations.push_back(orientation::rotationFromValues(zyx, {{a, b, c, 0.0}}));
            }
        }
    }
    return rotations;
}

}  // namespace

int main() {
    const orientation::Format* formats = orientation::formats();
    const int count = orientation::formatCount();
    const std::vector<CadTransform> rotations = sweepRotations();

    for (int i = 0; i < count; ++i) {
        for (const CadTransform& rotation : rotations) checkRotation(formats[i], rotation, "sweep");
        std::printf("  %-44s %zu orientations\n", formats[i].label, rotations.size());
    }

    const CadTransform halfTurns[4] = {
        CadTransform(),
        orientation::axisRotation(0, orientation::kPi),
        orientation::axisRotation(1, orientation::kPi),
        orientation::axisRotation(2, orientation::kPi),
    };
    const char* halfTurnNames[4] = {"identity", "half turn about X", "half turn about Y",
                                    "half turn about Z"};
    for (int i = 0; i < count; ++i) {
        for (int h = 0; h < 4; ++h) checkRotation(formats[i], halfTurns[h], halfTurnNames[h]);
    }

    for (int i = 0; i < count; ++i) {
        for (const std::array<double, 3>& axis : {std::array<double, 3>{{1.0, 1.0, 0.0}},
                                                  std::array<double, 3>{{0.0, 1.0, 1.0}},
                                                  std::array<double, 3>{{1.0, 0.0, 1.0}},
                                                  std::array<double, 3>{{1.0, 1.0, 1.0}}}) {
            const double length = std::sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
            static const orientation::Format vector = {
                "", orientation::Kind::RotationVector, {0, 0, 0}, false, 3, {"", "", "", ""}, "", 1.0, 1.0};
            const CadTransform rotation = orientation::rotationFromValues(
                vector, {{axis[0] / length * 180.0, axis[1] / length * 180.0,
                          axis[2] / length * 180.0, 0.0}});
            checkRotation(formats[i], rotation, "half turn, slanted axis");
        }
    }

    // Tiny rotations, where the rotation vector's small-angle guard lives.
    for (int i = 0; i < count; ++i) {
        for (double degrees : {1.0e-7, 1.0e-4, 0.01, 0.5}) {
            checkRotation(formats[i], orientation::axisRotation(1, degrees * orientation::kDegreesToRadians),
                          "tiny rotation");
        }
    }

    {
        const CadTransform quarterTurn = orientation::axisRotation(0, orientation::kPi * 0.5);
        for (int i = 0; i < count; ++i) {
            if (formats[i].kind != orientation::Kind::RotationVector) continue;
            const std::array<double, 4> v = orientation::valuesFromRotation(formats[i], quarterTurn);
            const double expected = formats[i].displayScale == 1.0 ? 90.0 : orientation::kPi * 0.5;
            if (std::abs(v[0] - expected) > 1.0e-9 || std::abs(v[1]) > 1.0e-9 || std::abs(v[2]) > 1.0e-9) {
                std::printf("  FAIL %-44s quarter turn about X -> (%.6f %.6f %.6f), expected (%.6f 0 0)\n",
                            formats[i].label, v[0], v[1], v[2], expected);
                ++g_failures;
            }
        }
    }

    {
        const orientation::Format* rotationVector = nullptr;
        for (int i = 0; i < count; ++i) {
            if (formats[i].kind == orientation::Kind::RotationVector && formats[i].displayScale == 1.0) {
                rotationVector = &formats[i];
                break;
            }
        }
        if (!rotationVector) {
            std::printf("  FAIL no degree rotation-vector format to anchor the reference values on\n");
            ++g_failures;
        } else {
            struct Expected { const char* labelFragment; int fieldCount; double v[4]; };
            struct Sample { const char* what; double ur[3]; const Expected* rows; size_t count; };

            // UR (deg) 30, 40, 50. Nothing degenerate about it, so every row is pinned.
            static const Expected kAsymmetric[] = {
                {"Staubli",    3, { 11.27, 47.29,  48.34, 0.0}},
                {"Fanuc",      3, { 45.14, 19.45,  61.43, 0.0}},
                {"KUKA",       3, { 61.43, 19.45,  45.14, 0.0}},
                {"deg (AR4",   3, { 30.00, 40.00,  50.00, 0.0}},
                {"rad (UR",    3, {  0.52,  0.70,   0.87, 0.0}},
                {"quaternion", 4, {  0.816, 0.245,  0.327, 0.409}},
                {"Adept",      3, {-10.23, 48.30,  63.51, 0.0}},
                {"Epson",      3, { 48.34, 47.29,  11.27, 0.0}},
                {"CATIA",      3, { 79.77, 48.30, -26.49, 0.0}},
            };
            static const Expected kSymmetric[] = {
                {"Staubli",    3, {137.11, 34.24, -76.20, 0.0}},
                {"Fanuc",      3, {137.11, 34.24,  76.20, 0.0}},
                {"KUKA",       3, { 76.20, 34.24, 137.11, 0.0}},
                {"deg (AR4",   3, { 90.00, 90.00,   0.00, 0.0}},
                {"rad (UR",    3, {  1.5708, 1.5708, 0.0, 0.0}},
                {"quaternion", 4, {  0.444, 0.634,  0.634, 0.000}},
                {"Adept",      3, {-45.00, 127.28, 45.00, 0.0}},
                {"Epson",      3, {-76.20, 34.24, 137.11, 0.0}},
            };
            const Sample samples[] = {
                {"asymmetric", {30.0, 40.0, 50.0}, kAsymmetric, sizeof(kAsymmetric) / sizeof(kAsymmetric[0])},
                {"symmetric",  {90.0, 90.0,  0.0}, kSymmetric,  sizeof(kSymmetric) / sizeof(kSymmetric[0])},
            };

            for (const Sample& sample : samples) {
                const CadTransform R = orientation::rotationFromValues(
                    *rotationVector, {{sample.ur[0], sample.ur[1], sample.ur[2], 0.0}});
                for (size_t e = 0; e < sample.count; ++e) {
                    const Expected& want = sample.rows[e];
                    const orientation::Format* found = nullptr;
                    for (int i = 0; i < count; ++i) {
                        if (std::string(formats[i].label).find(want.labelFragment) != std::string::npos) {
                            found = &formats[i];
                            break;
                        }
                    }
                    if (!found) {
                        std::printf("  FAIL reference check: no format matching '%s'\n", want.labelFragment);
                        ++g_failures;
                        continue;
                    }
                    const std::array<double, 4> got = orientation::valuesFromRotation(*found, R);
                    // The reference values are quoted to two decimals, so compare at that resolution.
                    for (int k = 0; k < want.fieldCount; ++k) {
                        if (std::abs(got[static_cast<size_t>(k)] - want.v[k]) > 0.005) {
                            std::printf("  FAIL %-38s %s rotation, field %d: got %.5f, reference %.5f\n",
                                        found->label, sample.what, k,
                                        got[static_cast<size_t>(k)], want.v[k]);
                            ++g_failures;
                        }
                    }
                }
                std::printf("  %-44s %zu rows (%s)\n", "matched against reference values",
                            sample.count, sample.what);
            }
        }
    }

    if (g_failures != 0) {
        std::printf("orientation format check FAILED: %d case(s)\n", g_failures);
        return 1;
    }
    std::printf("orientation format check passed\n");
    return 0;
}
