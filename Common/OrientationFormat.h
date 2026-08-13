#pragma once

#include "CadNode.h"

#include <algorithm>
#include <array>
#include <cmath>

// How a placement orientation is shown and typed. See docs/orientation-formats.md, which
// carries the evidence behind each attribution.

namespace orientation {

constexpr double kDegreesToRadians = 3.14159265358979323846 / 180.0;
constexpr double kRadiansToDegrees = 180.0 / 3.14159265358979323846;
constexpr double kPi = 3.14159265358979323846;

enum class Kind {
    // Three rotations, each about the axis the previous ones left, applied left to right.
    EulerIntrinsic,
    // Axis times angle. Universal Robots reports this in radians; shown here in degrees, so the
    // vector's length is the rotation in degrees and its direction is the axis.
    RotationVector,
    // w, x, y, z. Four numbers with one constraint, so what is typed is normalised on the way in.
    Quaternion,
};

struct Format {
    const char* label;
    Kind kind;
    // EulerIntrinsic only. 0 = X, 1 = Y, 2 = Z.
    int axes[3];
    // EulerIntrinsic only. An extrinsic convention is the same rotation as the reversed
    // intrinsic one, so it is stored that way with the displayed fields run backwards.
    bool reversedFields;
    int fieldCount;
    const char* fieldLabels[4];
    const char* fieldFormat;
    double step;
    double displayScale;
};

inline constexpr Format kFormats[] = {
    {"AR4 tool frame / Staubli / Mecademic", Kind::EulerIntrinsic, {0, 1, 2}, false, 3,
     {"rx", "ry", "rz", ""}, "%.1f deg", 1.0, 1.0},
    {"Rotation vector, deg (AR4 HMI, UR)", Kind::RotationVector, {0, 0, 0}, false, 3,
     {"RX", "RY", "RZ", ""}, "%.2f deg", 1.0, 1.0},
    {"Rotation vector, rad (UR native)", Kind::RotationVector, {0, 0, 0}, false, 3,
     {"RX", "RY", "RZ", ""}, "%.5f rad", 0.01, kDegreesToRadians},
    {"Fanuc / Motoman", Kind::EulerIntrinsic, {2, 1, 0}, true, 3,
     {"W", "P", "R", ""}, "%.1f deg", 1.0, 1.0},
    // One row for three names: ABB's EulerZYX / OrientZYX takes its arguments (z, y, x), which
    // is the same triple KUKA calls A, B, C.
    {"ABB / KUKA / Nachi", Kind::EulerIntrinsic, {2, 1, 0}, false, 3,
     {"A", "B", "C", ""}, "%.1f deg", 1.0, 1.0},
    {"ABB quaternion", Kind::Quaternion, {0, 0, 0}, false, 4,
     {"q1", "q2", "q3", "q4"}, "%.5f", 0.01, 1.0},
    {"Adept / Comau / Kawasaki", Kind::EulerIntrinsic, {2, 1, 2}, false, 3,
     {"O", "A", "T", ""}, "%.1f deg", 1.0, 1.0},
    // The Staubli composition in the opposite order. Fitted on two rotations at once; one alone
    // left this tied with ZXY intrinsic.
    {"Epson / CRS", Kind::EulerIntrinsic, {0, 1, 2}, true, 3,
     {"U", "V", "W", ""}, "%.1f deg", 1.0, 1.0},
    // ZXZ, the sibling of the Adept ZYZ row: (a, b, c) there is (a+90, b, c-90) here.
    {"CATIA / SolidWorks", Kind::EulerIntrinsic, {2, 0, 2}, false, 3,
     {"Z1", "X", "Z2", ""}, "%.1f deg", 1.0, 1.0},
    // Fixed-axis XYZ, the same family as Fanuc. Attributed from Techman's documentation, and the
    // one row with no reference value behind it in tools/orientation_format_check.cpp - the
    // rotations that pin the others do not cover this convention.
    {"Omron TM / Techman", Kind::EulerIntrinsic, {2, 1, 0}, true, 3,
     {"RX", "RY", "RZ", ""}, "%.1f deg", 1.0, 1.0},
};

inline const Format* formats() { return kFormats; }

inline int formatCount() { return static_cast<int>(sizeof(kFormats) / sizeof(kFormats[0])); }

// A rotation about one principal axis. Matches withLocalRotation's elemental matrices, which keeps
// the placement editors and the gimbal describing the same rotations.
inline CadTransform axisRotation(int axis, double radians) {
    const double c = std::cos(radians);
    const double s = std::sin(radians);
    CadTransform rotation;
    if (axis == 0) {
        rotation.values = {{1, 0, 0, 0, 0, c, -s, 0, 0, s, c, 0}};
    } else if (axis == 1) {
        rotation.values = {{c, 0, s, 0, 0, 1, 0, 0, -s, 0, c, 0}};
    } else {
        rotation.values = {{c, -s, 0, 0, s, c, 0, 0, 0, 0, 1, 0}};
    }
    return rotation;
}

namespace detail {

inline double at(const CadTransform& pose, int row, int column) {
    return pose.values[static_cast<size_t>(row * 4 + column)];
}

inline double clampUnit(double value) { return std::max(-1.0, std::min(1.0, value)); }

constexpr double kPoleThreshold = 1.0e-7;

// +1 when the axis order runs the same way round as (X, Y, Z), -1 when it runs the other way.
// Carrying the handedness in a term is what lets one pair of formulas serve every axis order
// instead of a hand-derived pair per convention.
inline double cyclic(int a, int b, int c) {
    return (b == (a + 1) % 3) && (c == (b + 1) % 3) ? 1.0 : -1.0;
}

inline std::array<double, 3> intrinsicEulerFrom(const Format& format, const CadTransform& pose) {
    const int i = format.axes[0];
    const int j = format.axes[1];
    const int k = format.axes[2];

    double first = 0.0;
    double second = 0.0;
    double third = 0.0;

    // At a pole only the sum or difference of the first and third angles is determined, so the
    // third is pinned to zero. One derivation covers all twelve axis orders; a per-convention
    // branch was wrong for ZXZ in a way no ZYZ round-trip could show.
    const auto anglesAtPole = [&](double atSecond) {
        const CadTransform residual = pose * axisRotation(j, -atSecond);
        // For a rotation about axis i, the (q,p) and (p,p) entries are its sine and cosine, where
        // (i, p, q) runs the same way round as (X, Y, Z).
        const int p = (i + 1) % 3;
        const int q = (i + 2) % 3;
        return std::atan2(at(residual, q, p), at(residual, p, p));
    };

    if (i != k) {
        const double epsilon = cyclic(i, j, k);
        second = std::asin(clampUnit(epsilon * at(pose, i, k)));
        if (std::abs(std::cos(second)) > kPoleThreshold) {
            first = std::atan2(-epsilon * at(pose, j, k), at(pose, k, k));
            third = std::atan2(-epsilon * at(pose, i, j), at(pose, i, i));
        } else {
            first = anglesAtPole(second);
            third = 0.0;
        }
    } else {
        const int other = 3 - i - j;
        const double epsilon = cyclic(i, j, other);
        // acos gives [0, pi], so sin(second) is never negative here.
        second = std::acos(clampUnit(at(pose, i, i)));
        if (std::abs(std::sin(second)) > kPoleThreshold) {
            first = std::atan2(at(pose, j, i), -epsilon * at(pose, other, i));
            third = std::atan2(at(pose, i, j), epsilon * at(pose, i, other));
        } else {
            first = anglesAtPole(second);
            third = 0.0;
        }
    }

    return {{first * kRadiansToDegrees, second * kRadiansToDegrees, third * kRadiansToDegrees}};
}

inline CadTransform intrinsicEulerTo(const Format& format, const std::array<double, 3>& degrees) {
    CadTransform pose;
    for (int i = 0; i < 3; ++i) {
        pose = pose * axisRotation(format.axes[i], degrees[static_cast<size_t>(i)] * kDegreesToRadians);
    }
    return pose;
}

inline std::array<double, 3> rotationVectorFrom(const CadTransform& pose) {
    const double trace = at(pose, 0, 0) + at(pose, 1, 1) + at(pose, 2, 2);
    const double angle = std::acos(clampUnit((trace - 1.0) * 0.5));

    std::array<double, 3> axis{{1.0, 0.0, 0.0}};
    if (angle < 1.0e-9) {
        return {{0.0, 0.0, 0.0}};
    }
    if (angle > kPi - 1.0e-6) {
        // Half a turn: the skew part vanishes, so the axis comes from the symmetric part. Averaging
        // the mirrored off-diagonals and halving double-counts, hence the quarter - that mistake
        // reads a half turn about (1, 0, 1) as one about (2, 0, 1).
        const double bxx = (at(pose, 0, 0) + 1.0) * 0.5;
        const double byy = (at(pose, 1, 1) + 1.0) * 0.5;
        const double bzz = (at(pose, 2, 2) + 1.0) * 0.5;
        const double bxy = (at(pose, 0, 1) + at(pose, 1, 0)) * 0.25;
        const double bxz = (at(pose, 0, 2) + at(pose, 2, 0)) * 0.25;
        const double byz = (at(pose, 1, 2) + at(pose, 2, 1)) * 0.25;
        if (bxx >= byy && bxx >= bzz) {
            const double x = std::sqrt(std::max(0.0, bxx));
            axis = {{x, bxy / x, bxz / x}};
        } else if (byy >= bzz) {
            const double y = std::sqrt(std::max(0.0, byy));
            axis = {{bxy / y, y, byz / y}};
        } else {
            const double z = std::sqrt(std::max(0.0, bzz));
            axis = {{bxz / z, byz / z, z}};
        }
    } else {
        const double scale = 1.0 / (2.0 * std::sin(angle));
        axis = {{(at(pose, 2, 1) - at(pose, 1, 2)) * scale,
                 (at(pose, 0, 2) - at(pose, 2, 0)) * scale,
                 (at(pose, 1, 0) - at(pose, 0, 1)) * scale}};
    }

    const double length = std::sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
    if (length < 1.0e-12) return {{0.0, 0.0, 0.0}};
    const double degrees = angle * kRadiansToDegrees;
    return {{axis[0] / length * degrees, axis[1] / length * degrees, axis[2] / length * degrees}};
}

inline CadTransform rotationVectorTo(const std::array<double, 3>& degrees) {
    const double magnitude =
        std::sqrt(degrees[0] * degrees[0] + degrees[1] * degrees[1] + degrees[2] * degrees[2]);
    if (magnitude < 1.0e-12) return CadTransform();

    const double angle = magnitude * kDegreesToRadians;
    const double x = degrees[0] / magnitude;
    const double y = degrees[1] / magnitude;
    const double z = degrees[2] / magnitude;
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    const double t = 1.0 - c;

    // Rodrigues, written out.
    CadTransform pose;
    pose.values[0] = t * x * x + c;
    pose.values[1] = t * x * y - s * z;
    pose.values[2] = t * x * z + s * y;
    pose.values[4] = t * x * y + s * z;
    pose.values[5] = t * y * y + c;
    pose.values[6] = t * y * z - s * x;
    pose.values[8] = t * x * z - s * y;
    pose.values[9] = t * y * z + s * x;
    pose.values[10] = t * z * z + c;
    return pose;
}

inline std::array<double, 4> quaternionFrom(const CadTransform& pose) {
    const double m00 = at(pose, 0, 0), m01 = at(pose, 0, 1), m02 = at(pose, 0, 2);
    const double m10 = at(pose, 1, 0), m11 = at(pose, 1, 1), m12 = at(pose, 1, 2);
    const double m20 = at(pose, 2, 0), m21 = at(pose, 2, 1), m22 = at(pose, 2, 2);
    const double trace = m00 + m11 + m22;

    double w = 1.0, x = 0.0, y = 0.0, z = 0.0;
    if (trace > 0.0) {
        const double s = std::sqrt(trace + 1.0) * 2.0;
        w = 0.25 * s;
        x = (m21 - m12) / s;
        y = (m02 - m20) / s;
        z = (m10 - m01) / s;
    } else if (m00 > m11 && m00 > m22) {
        const double s = std::sqrt(1.0 + m00 - m11 - m22) * 2.0;
        w = (m21 - m12) / s;
        x = 0.25 * s;
        y = (m01 + m10) / s;
        z = (m02 + m20) / s;
    } else if (m11 > m22) {
        const double s = std::sqrt(1.0 + m11 - m00 - m22) * 2.0;
        w = (m02 - m20) / s;
        x = (m01 + m10) / s;
        y = 0.25 * s;
        z = (m12 + m21) / s;
    } else {
        const double s = std::sqrt(1.0 + m22 - m00 - m11) * 2.0;
        w = (m10 - m01) / s;
        x = (m02 + m20) / s;
        y = (m12 + m21) / s;
        z = 0.25 * s;
    }

    // q and -q are the same rotation. Pinning the sign of the scalar keeps the displayed numbers
    // from flipping wholesale as an arm is dragged through a pose.
    if (w < 0.0) { w = -w; x = -x; y = -y; z = -z; }
    return {{w, x, y, z}};
}

inline CadTransform quaternionTo(const std::array<double, 4>& q) {
    double w = q[0], x = q[1], y = q[2], z = q[3];
    const double length = std::sqrt(w * w + x * x + y * y + z * z);
    if (length < 1.0e-12) return CadTransform();
    w /= length; x /= length; y /= length; z /= length;

    CadTransform pose;
    pose.values[0] = 1.0 - 2.0 * (y * y + z * z);
    pose.values[1] = 2.0 * (x * y - w * z);
    pose.values[2] = 2.0 * (x * z + w * y);
    pose.values[4] = 2.0 * (x * y + w * z);
    pose.values[5] = 1.0 - 2.0 * (x * x + z * z);
    pose.values[6] = 2.0 * (y * z - w * x);
    pose.values[8] = 2.0 * (x * z - w * y);
    pose.values[9] = 2.0 * (y * z + w * x);
    pose.values[10] = 1.0 - 2.0 * (x * x + y * y);
    return pose;
}

}  // namespace detail

// The numbers to show for a rotation. Only the first `format.fieldCount` are meaningful.
inline std::array<double, 4> valuesFromRotation(const Format& format, const CadTransform& pose) {
    std::array<double, 4> out{{0.0, 0.0, 0.0, 0.0}};
    switch (format.kind) {
    case Kind::EulerIntrinsic: {
        const std::array<double, 3> angles = detail::intrinsicEulerFrom(format, pose);
        for (int i = 0; i < 3; ++i) {
            out[static_cast<size_t>(i)] = format.reversedFields ? angles[static_cast<size_t>(2 - i)]
                                                                : angles[static_cast<size_t>(i)];
        }
        break;
    }
    case Kind::RotationVector: {
        const std::array<double, 3> vector = detail::rotationVectorFrom(pose);
        for (int i = 0; i < 3; ++i) out[static_cast<size_t>(i)] = vector[static_cast<size_t>(i)];
        break;
    }
    case Kind::Quaternion:
        out = detail::quaternionFrom(pose);
        break;
    }
    if (format.displayScale != 1.0) {
        for (int i = 0; i < format.fieldCount; ++i) out[static_cast<size_t>(i)] *= format.displayScale;
    }
    return out;
}

// The inverse. Rotation only - the translation is the caller's, since none of these formats says
// anything about it.
inline CadTransform rotationFromValues(const Format& format, const std::array<double, 4>& raw) {
    std::array<double, 4> values = raw;
    if (format.displayScale != 1.0) {
        for (int i = 0; i < format.fieldCount; ++i) values[static_cast<size_t>(i)] /= format.displayScale;
    }
    switch (format.kind) {
    case Kind::EulerIntrinsic: {
        std::array<double, 3> angles{{0.0, 0.0, 0.0}};
        for (int i = 0; i < 3; ++i) {
            angles[static_cast<size_t>(i)] = format.reversedFields ? values[static_cast<size_t>(2 - i)]
                                                                   : values[static_cast<size_t>(i)];
        }
        return detail::intrinsicEulerTo(format, angles);
    }
    case Kind::RotationVector:
        return detail::rotationVectorTo({{values[0], values[1], values[2]}});
    case Kind::Quaternion:
        return detail::quaternionTo(values);
    }
    return CadTransform();
}

}  // namespace orientation
