#include "RobotProgramSimulator.h"
#include "StringUtil.h"

#include "RobotMotionCore.h"
#include "RobotRuntime.h"

#include <algorithm>
#include <cmath>
#include <limits>

using strutil::operator<<;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kMotionControlPeriodSeconds = 0.005;
constexpr double kJointSampleRad = 1.0 * kDegToRad;
constexpr double kMinTimeStepSeconds = 1.0e-5;
constexpr double kMinStoredSampleTcpDistanceMm = 0.5;
constexpr double kMinStoredSampleJointDeltaRad = 15.0 * kDegToRad;
constexpr double kSingularityThresholdRad = 5.0 * kDegToRad;
constexpr double kJointFlipThresholdRad = 90.0 * kDegToRad;

double clamp01(double value) {
    return std::max(0.0, std::min(1.0, value));
}

double wrapRadians(double value) {
    while (value > kPi) value -= kTwoPi;
    while (value < -kPi) value += kTwoPi;
    return value;
}

RobotProgramSimulator::Vec3 translationOf(const CadTransform& transform) {
    return {transform.values[3], transform.values[7], transform.values[11]};
}

void setTranslation(CadTransform& transform, const RobotProgramSimulator::Vec3& value) {
    transform.values[3] = value.x;
    transform.values[7] = value.y;
    transform.values[11] = value.z;
}

std::string motionRejectMessage(RobotMotionCore::RejectCode code) {
    return std::string(RobotMotionCore::rejectCodeName(code));
}

double blendCornerCurvature(const RobotMotionCore::MotionSegment& segment) {
    if (segment.kind != RobotMotionCore::MotionSegmentKind::MoveLBlend) return 0.0;
    if (std::isfinite(segment.curvature) && segment.curvature > 1.0e-12) return segment.curvature;
    if (std::isfinite(segment.blendPlan.radiusMm) && segment.blendPlan.radiusMm > 1.0e-9) {
        return 1.0 / segment.blendPlan.radiusMm;
    }
    return 0.0;
}

double blendCornerRadiusMm(const RobotMotionCore::MotionSegment& segment) {
    const double curvature = blendCornerCurvature(segment);
    if (curvature > 1.0e-12) return 1.0 / curvature;
    return 0.0;
}

double blendCornerTrimMm(const RobotMotionCore::MotionSegment& segment) {
    if (segment.kind != RobotMotionCore::MotionSegmentKind::MoveLBlend) return 0.0;
    if (std::isfinite(segment.blendPlan.trimMm) && segment.blendPlan.trimMm > 1.0e-9) {
        return segment.blendPlan.trimMm;
    }
    if (std::isfinite(segment.blendPlan.actualDeviationMm) && segment.blendPlan.actualDeviationMm > 1.0e-9) {
        return segment.blendPlan.actualDeviationMm;
    }
    return 0.0;
}

double positiveFiniteMin(double a, double b) {
    const bool haveA = std::isfinite(a) && a > 1.0e-9;
    const bool haveB = std::isfinite(b) && b > 1.0e-9;
    if (haveA && haveB) return std::min(a, b);
    if (haveA) return a;
    if (haveB) return b;
    return 0.0;
}

double activeBlendCornerSpeedCap(const RobotMotionCore::MotionSegment& segment) {
    const uint32_t speedMask = segment.activeSpeedLimitMask;
    double cap = 0.0;
    if (speedMask & RobotMotionCore::MotionCapJointAcceleration) {
        cap = positiveFiniteMin(cap, segment.speedAfterJointAcceleration);
    }
    if (speedMask & RobotMotionCore::MotionCapPathAcceleration) {
        cap = positiveFiniteMin(cap, segment.speedAfterPathAcceleration);
    }
    if (speedMask & RobotMotionCore::MotionCapJerk) {
        cap = positiveFiniteMin(cap, segment.speedAfterJerk);
    }
    return cap;
}

std::string blendFeasibilityDetails(const RobotMotionCore::MotionSegment& segment,
                                double commandSpeed,
                                double finalSpeed) {
    if (segment.kind != RobotMotionCore::MotionSegmentKind::MoveLBlend) return {};
    const double curvature = blendCornerCurvature(segment);
    const double radiusMm = blendCornerRadiusMm(segment);
    const double trimMm = blendCornerTrimMm(segment);
    if (curvature <= 1.0e-12 || radiusMm <= 1.0e-9 ||
        !std::isfinite(commandSpeed) || commandSpeed <= 1.0e-9) {
        return {};
    }

    const double requestedAccel = commandSpeed * commandSpeed * curvature;
    std::string details = strutil::format("; path blend %1 mm gives actual corner radius %2 mm, needs ~%3 mm/s^2 at %4 mm/s")
                          .arg(trimMm, 0, 'f', trimMm < 10.0 ? 2 : 1)
                          .arg(radiusMm, 0, 'f', radiusMm < 10.0 ? 2 : 1)
                          .arg(requestedAccel, 0, 'f', 0)
                          .arg(commandSpeed, 0, 'f', 1);
    if (segment.blendPlan.maxDeviationMm > trimMm + 1.0e-6) {
        details += strutil::format("; requested path blend %1 mm was clipped by adjacent segment length")
                       .arg(segment.blendPlan.maxDeviationMm, 0, 'f',
                            segment.blendPlan.maxDeviationMm < 10.0 ? 2 : 1);
    }

    const double capSpeed = activeBlendCornerSpeedCap(segment);
    if (capSpeed > 1.0e-9 && capSpeed < commandSpeed - 1.0e-6) {
        const double availableAccel = capSpeed * capSpeed * curvature;
        if (availableAccel > 1.0e-9) {
            const double suggestedRadius = commandSpeed * commandSpeed / availableAccel;
            const double suggestedTrim = trimMm > 1.0e-9 && radiusMm > 1.0e-9
                ? suggestedRadius * (trimMm / radiusMm)
                : 0.0;
            details += strutil::format("; available corner accel ~= %1 mm/s^2, use path blend >= %2 mm for actual radius >= %3 mm to keep %4 mm/s")
                           .arg(availableAccel, 0, 'f', 0)
                           .arg(suggestedTrim, 0, 'f', suggestedTrim < 10.0 ? 2 : 1)
                           .arg(suggestedRadius, 0, 'f', suggestedRadius < 10.0 ? 2 : 1)
                           .arg(commandSpeed, 0, 'f', 1);
        }
    } else if (std::isfinite(finalSpeed) && finalSpeed > 1.0e-9) {
        const double plannedAccel = finalSpeed * finalSpeed * curvature;
        details += strutil::format("; planned corner accel ~= %1 mm/s^2")
                       .arg(plannedAccel, 0, 'f', 0);
    }
    return details;
}

std::string blendFeasibilityWarning(const RobotMotionCore::MotionSegment& segment) {
    if (segment.kind != RobotMotionCore::MotionSegmentKind::MoveLBlend) return {};
    const double commandSpeed = std::isfinite(segment.commandSpeed) ? segment.commandSpeed : segment.nominalSpeed;
    const double capSpeed = activeBlendCornerSpeedCap(segment);
    const double curvature = blendCornerCurvature(segment);
    const double radiusMm = blendCornerRadiusMm(segment);
    const double trimMm = blendCornerTrimMm(segment);
    if (!std::isfinite(commandSpeed) || commandSpeed <= 1.0e-9 ||
        capSpeed <= 1.0e-9 || capSpeed >= commandSpeed * 0.95 ||
        curvature <= 1.0e-12 || radiusMm <= 1.0e-9 ||
        trimMm <= 1.0e-9) {
        return {};
    }

    const double requestedAccel = commandSpeed * commandSpeed * curvature;
    const double availableAccel = capSpeed * capSpeed * curvature;
    if (availableAccel <= 1.0e-9) return {};
    const double suggestedRadius = commandSpeed * commandSpeed / availableAccel;
    const double suggestedTrim = suggestedRadius * (trimMm / radiusMm);
    std::string warning = strutil::format("Small blend corner: %1 mm/s with %2 mm path blend makes only %3 mm actual radius and needs ~%4 mm/s^2; capped near %5 mm/s by ~%6 mm/s^2. Use path blend >= %7 mm for actual radius >= %8 mm, or lower speed.")
        .arg(commandSpeed, 0, 'f', 1)
        .arg(trimMm, 0, 'f', trimMm < 10.0 ? 2 : 1)
        .arg(radiusMm, 0, 'f', radiusMm < 10.0 ? 2 : 1)
        .arg(requestedAccel, 0, 'f', 0)
        .arg(capSpeed, 0, 'f', 1)
        .arg(availableAccel, 0, 'f', 0)
        .arg(suggestedTrim, 0, 'f', suggestedTrim < 10.0 ? 2 : 1)
        .arg(suggestedRadius, 0, 'f', suggestedRadius < 10.0 ? 2 : 1);
    if (segment.blendPlan.maxDeviationMm > trimMm + 1.0e-6) {
        warning += strutil::format(" Requested %1 mm was already clipped by adjacent segment length.")
            .arg(segment.blendPlan.maxDeviationMm, 0, 'f',
                 segment.blendPlan.maxDeviationMm < 10.0 ? 2 : 1);
    }
    return warning;
}

void tallyCapReasons(std::map<std::string, uint32_t>& counts, const RobotMotionCore::MotionSegment& segment) {
    const uint32_t mask = segment.activeSpeedLimitMask | segment.activeAccelerationLimitMask;
    if (mask == RobotMotionCore::MotionCapNone) return;
    struct Entry {
        uint32_t bit;
        const char* name;
    };
    static const Entry kEntries[] = {
        {RobotMotionCore::MotionCapCommandSpeed, "command speed"},
        {RobotMotionCore::MotionCapJointVelocity, "joint velocity"},
        {RobotMotionCore::MotionCapJointAcceleration, "joint acceleration"},
        {RobotMotionCore::MotionCapJerk, "jerk"},
        {RobotMotionCore::MotionCapSingularity, "singularity"},
        {RobotMotionCore::MotionCapBlendGeometry, "blend geometry"},
        {RobotMotionCore::MotionCapQueueBoundary, "queue boundary"},
        {RobotMotionCore::MotionCapFinePoint, "fine point"},
        {RobotMotionCore::MotionCapLookahead, "lookahead"},
        {RobotMotionCore::MotionCapStepRate, "step rate"},
        {RobotMotionCore::MotionCapPathAcceleration, "path acceleration"},
        {RobotMotionCore::MotionCapWeave, "weave"}
    };
    for (const Entry& entry : kEntries) {
        if (mask & entry.bit) counts[std::string(entry.name)] += 1u;
    }
}

std::string capReasonSummary(const RobotMotionCore::MotionSegment& segment) {
    const uint32_t speedMask = segment.activeSpeedLimitMask;
    const uint32_t accelMask = segment.activeAccelerationLimitMask;
    const uint32_t contextMask = segment.contextMask;
    if ((speedMask | accelMask | contextMask | segment.capReasonMask) == RobotMotionCore::MotionCapNone) return {};
    if (speedMask == RobotMotionCore::MotionCapNone &&
        accelMask == RobotMotionCore::MotionCapNone &&
        (contextMask & RobotMotionCore::MotionCapBlendGeometry) == 0) {
        return {};
    }

    const double commandSpeed = std::isfinite(segment.commandSpeed) ? segment.commandSpeed : segment.nominalSpeed;
    const double finalSpeed = segment.finalNominalSpeed > 0.0 ? segment.finalNominalSpeed : segment.nominalSpeed;
    const double commandAccel = std::isfinite(segment.commandAcceleration) ? segment.commandAcceleration : segment.acceleration;
    const double finalAccel = std::isfinite(segment.acceleration) ? segment.acceleration : commandAccel;
    const bool jointSegment = segment.kind == RobotMotionCore::MotionSegmentKind::MoveJ;
    const double speedScale = jointSegment ? (180.0 / kPi) : 1.0;
    const double accelScale = speedScale;
    const std::string speedUnit = jointSegment ? std::string("deg/s") : std::string("mm/s");
    const std::string accelUnit = jointSegment ? std::string("deg/s^2") : std::string("mm/s^2");

    std::vector<std::string> contexts;
    if (contextMask & RobotMotionCore::MotionCapBlendGeometry) contexts << "blend geometry";
    if (contextMask & RobotMotionCore::MotionCapFinePoint) contexts << "fine point";
    if (contextMask & RobotMotionCore::MotionCapQueueBoundary) contexts << "queue boundary";
    if (contextMask & RobotMotionCore::MotionCapLookahead) contexts << "lookahead";

    std::string segmentLabel = jointSegment ? std::string("Joint move") : std::string("Linear move");
    std::string segmentExplanation;
    if (contextMask & RobotMotionCore::MotionCapFinePoint) {
        segmentLabel = std::string("Fine stop at target");
        segmentExplanation = std::string(" so the robot can stop at the instruction");
    } else if (contextMask & RobotMotionCore::MotionCapBlendGeometry) {
        segmentLabel = std::string("Blend corner");
    } else if (contextMask & RobotMotionCore::MotionCapQueueBoundary) {
        segmentLabel = std::string("Queue boundary");
    } else if (contextMask & RobotMotionCore::MotionCapLookahead) {
        segmentLabel = std::string("Lookahead");
    }

    std::vector<std::string> speedLimits;
    const auto addSpeedLimit = [&](uint32_t reason, const std::string& name, double value) {
        if ((speedMask & reason) == 0) return;
        if (std::isfinite(value) && value > 0.0 &&
            std::isfinite(commandSpeed) && value < commandSpeed - 1.0e-6) {
            speedLimits << strutil::format("%1 %2").arg(name).arg(value * speedScale, 0, 'f', 1);
        }
    };
    addSpeedLimit(RobotMotionCore::MotionCapJointVelocity, "joint velocity", segment.speedAfterJointVelocity);
    addSpeedLimit(RobotMotionCore::MotionCapJointAcceleration, "joint acceleration", segment.speedAfterJointAcceleration);
    addSpeedLimit(RobotMotionCore::MotionCapPathAcceleration, "path acceleration", segment.speedAfterPathAcceleration);
    addSpeedLimit(RobotMotionCore::MotionCapSingularity, "singularity", segment.speedAfterSingularity);
    addSpeedLimit(RobotMotionCore::MotionCapStepRate, "step rate", segment.speedAfterStepRate);
    addSpeedLimit(RobotMotionCore::MotionCapJerk, "jerk", segment.speedAfterJerk);
    addSpeedLimit(RobotMotionCore::MotionCapWeave, "weave", segment.speedAfterWeave);

    std::vector<std::string> accelLimits;
    std::vector<std::string> accelCauses;
    const auto addAccelLimit = [&](uint32_t reason, const std::string& name, double value) {
        if ((accelMask & reason) == 0) return;
        if (std::isfinite(value) && value > 0.0 &&
            std::isfinite(commandAccel) && value < commandAccel - 1.0e-6) {
            accelLimits << strutil::format("%1 %2").arg(name).arg(value * accelScale, 0, 'f', 0);
            accelCauses << name;
        }
    };
    addAccelLimit(RobotMotionCore::MotionCapJointAcceleration, "joint acceleration", segment.accelerationAfterJointAcceleration);
    addAccelLimit(RobotMotionCore::MotionCapJerk, "jerk", segment.accelerationAfterJerk);

    const bool speedReduced = std::isfinite(commandSpeed) && std::isfinite(finalSpeed) &&
                              finalSpeed < commandSpeed - 1.0e-6 &&
                              speedMask != RobotMotionCore::MotionCapNone;
    const bool accelReduced = std::isfinite(commandAccel) && std::isfinite(finalAccel) &&
                              finalAccel < commandAccel - 1.0e-6 &&
                              accelMask != RobotMotionCore::MotionCapNone;
    const bool importantContext =
        (contextMask & (RobotMotionCore::MotionCapBlendGeometry |
                        RobotMotionCore::MotionCapFinePoint |
                        RobotMotionCore::MotionCapQueueBoundary |
                        RobotMotionCore::MotionCapLookahead)) != 0;
    if (!speedReduced && speedLimits.empty() && !importantContext) {
        return {};
    }

    std::string summary;
    if (speedReduced || !speedLimits.empty()) {
        summary = strutil::format("%1: speed reduced %2 -> %3 %4")
                      .arg(segmentLabel)
                      .arg(commandSpeed * speedScale, 0, 'f', 1)
                      .arg(finalSpeed * speedScale, 0, 'f', 1)
                      .arg(speedUnit);
        if (!speedLimits.empty()) summary += " by " + strutil::join(speedLimits, ", ");
    } else if (accelReduced || !accelLimits.empty()) {
        if (jointSegment) {
            summary = strutil::format("%1: speed kept at %2 %3, accel limited %4 -> %5 %6")
                          .arg(segmentLabel)
                          .arg(finalSpeed * speedScale, 0, 'f', 1)
                          .arg(speedUnit)
                          .arg(commandAccel * accelScale, 0, 'f', 0)
                          .arg(finalAccel * accelScale, 0, 'f', 0)
                          .arg(accelUnit);
        } else {
            summary = strutil::format("%1: ramp up/down limited to %2 %3")
                          .arg(segmentLabel)
                          .arg(finalAccel * accelScale, 0, 'f', 0)
                          .arg(accelUnit);
            if (!accelCauses.empty()) {
                strutil::removeDuplicates(accelCauses);
                summary += " by " + strutil::join(accelCauses, ", ");
            }
            summary += strutil::format("; cruise speed remains %1 %2")
                           .arg(finalSpeed * speedScale, 0, 'f', 1)
                           .arg(speedUnit);
            if (!segmentExplanation.empty()) summary += segmentExplanation;
        }
        if (jointSegment && !accelCauses.empty()) {
            strutil::removeDuplicates(accelCauses);
            summary += " by " + strutil::join(accelCauses, ", ");
        }
    } else {
        summary = strutil::format("%1: speed kept at %2 %3")
                      .arg(segmentLabel)
                      .arg(finalSpeed * speedScale, 0, 'f', 1)
                      .arg(speedUnit);
    }

    if (!contexts.empty() && segmentLabel == std::string("Linear move")) {
        summary += "; context: " + strutil::join(contexts, ", ");
    }
    summary += blendFeasibilityDetails(segment, commandSpeed, finalSpeed);
    summary += strutil::format("; segment %1 s").arg(segment.durationSec, 0, 'f', 3);
    return summary;
}

RobotProgramSimulator::Vec3 operator+(const RobotProgramSimulator::Vec3& a, const RobotProgramSimulator::Vec3& b) {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

RobotProgramSimulator::Vec3 operator-(const RobotProgramSimulator::Vec3& a, const RobotProgramSimulator::Vec3& b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

RobotProgramSimulator::Vec3 operator*(const RobotProgramSimulator::Vec3& value, double scale) {
    return {value.x * scale, value.y * scale, value.z * scale};
}

double dot(const RobotProgramSimulator::Vec3& a, const RobotProgramSimulator::Vec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

double length(const RobotProgramSimulator::Vec3& value) {
    return std::sqrt(dot(value, value));
}

RobotProgramSimulator::Vec3 normalized(const RobotProgramSimulator::Vec3& value) {
    const double mag = length(value);
    if (mag <= 1.0e-12) return {};
    return value * (1.0 / mag);
}

double distance(const RobotProgramSimulator::Vec3& a, const RobotProgramSimulator::Vec3& b) {
    return length(a - b);
}

double matrixValue(const CadTransform& transform, int row, int col) {
    return transform.values[static_cast<size_t>(row * 4 + col)];
}

double orientationDeltaAngle(const CadTransform& start, const CadTransform& target) {
    double delta[3][3]{};
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            for (int k = 0; k < 3; ++k) {
                delta[row][col] += matrixValue(start, k, row) * matrixValue(target, k, col);
            }
        }
    }
    const double trace = delta[0][0] + delta[1][1] + delta[2][2];
    return std::acos(std::max(-1.0, std::min(1.0, (trace - 1.0) * 0.5)));
}

CadTransform rotationAboutAxis(const RobotProgramSimulator::Vec3& axis, double angle) {
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    const double t = 1.0 - c;
    const double x = axis.x;
    const double y = axis.y;
    const double z = axis.z;
    CadTransform out;
    out.values = {{
        t * x * x + c,     t * x * y - s * z, t * x * z + s * y, 0.0,
        t * x * y + s * z, t * y * y + c,     t * y * z - s * x, 0.0,
        t * x * z - s * y, t * y * z + s * x, t * z * z + c,     0.0
    }};
    return out;
}

RobotProgramSimulator::Vec3 rotationAxisFromDelta(const double delta[3][3], double angle) {
    const double denom = 2.0 * std::sin(angle);
    if (std::abs(denom) > 1.0e-8) {
        return normalized({
            (delta[2][1] - delta[1][2]) / denom,
            (delta[0][2] - delta[2][0]) / denom,
            (delta[1][0] - delta[0][1]) / denom
        });
    }

    RobotProgramSimulator::Vec3 axis{
        std::sqrt(std::max(0.0, (delta[0][0] + 1.0) * 0.5)),
        std::sqrt(std::max(0.0, (delta[1][1] + 1.0) * 0.5)),
        std::sqrt(std::max(0.0, (delta[2][2] + 1.0) * 0.5))
    };
    if (delta[0][1] < 0.0) axis.y = -axis.y;
    if (delta[0][2] < 0.0) axis.z = -axis.z;
    return normalized(axis);
}

CadTransform interpolateOrientation(const CadTransform& start, const CadTransform& target, double t) {
    double delta[3][3]{};
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            for (int k = 0; k < 3; ++k) {
                delta[row][col] += matrixValue(start, k, row) * matrixValue(target, k, col);
            }
        }
    }

    const double trace = delta[0][0] + delta[1][1] + delta[2][2];
    const double angle = std::acos(std::max(-1.0, std::min(1.0, (trace - 1.0) * 0.5)));
    CadTransform out = start;
    if (angle <= 1.0e-12) return out;

    const RobotProgramSimulator::Vec3 axis = rotationAxisFromDelta(delta, angle);
    out = start * rotationAboutAxis(axis, angle * clamp01(t));
    out.values[3] = start.values[3];
    out.values[7] = start.values[7];
    out.values[11] = start.values[11];
    return out;
}

CadTransform interpolatePoseLinear(const CadTransform& start, const CadTransform& target, double t) {
    CadTransform out = interpolateOrientation(start, target, t);
    const RobotProgramSimulator::Vec3 p0 = translationOf(start);
    const RobotProgramSimulator::Vec3 p1 = translationOf(target);
    setTranslation(out, p0 + (p1 - p0) * clamp01(t));
    return out;
}

// Pure forward kinematics; safe to call from trajectory workers.
CadTransform poseForJoints(const RobotKinematicSnapshot& kinematics,
                           const std::array<double, 6>& joints) {
    return toolPoseFromKinematicSnapshot(kinematics, joints);
}

bool finitePositiveArray(const std::array<double, 6>& values) {
    for (double value : values) {
        if (!std::isfinite(value) || value <= 0.0) return false;
    }
    return true;
}

bool nearSingularity(const std::array<double, 6>& joints, double thresholdRad) {
    const double threshold = thresholdRad > 0.0 ? thresholdRad : kSingularityThresholdRad;
    return std::abs(std::sin(joints[4])) <= std::sin(threshold);
}

bool jointFlipBetween(const std::array<double, 6>& previous, const std::array<double, 6>& current) {
    for (int i = 0; i < 6; ++i) {
        const double delta = std::abs(wrapRadians(current[static_cast<size_t>(i)] - previous[static_cast<size_t>(i)]));
        if (delta >= kJointFlipThresholdRad) return true;
    }
    return false;
}

double externalTargetAfter(const std::vector<RobotProgramSimulator::FlatCommand>& commands,
                           int64_t commandId,
                           double initialMm) {
    if (commands.empty() || commandId < 0) return initialMm;
    const int64_t count = static_cast<int64_t>(commands.size());
    const int64_t index = commandId % count;
    for (int64_t i = index; i >= 0; --i) {
        const auto& command = commands[static_cast<size_t>(i)];
        if (command.hasExternalAxis) return command.externalAxisPositionMm;
    }
    // From the second cycle onward, modal J7 begins where the previous cycle ended.
    if (commandId >= count) {
        for (int64_t i = count - 1; i >= 0; --i) {
            const auto& command = commands[static_cast<size_t>(i)];
            if (command.hasExternalAxis) return command.externalAxisPositionMm;
        }
    }
    return initialMm;
}

double externalAxisAtSample(const std::vector<RobotProgramSimulator::FlatCommand>& commands,
                            int64_t commandId,
                            int64_t completedCommandId,
                            double initialExternalMm,
                            double pathProgress) {
    if (commands.empty() || commandId < 0) return initialExternalMm;
    const double startMm = externalTargetAfter(commands, commandId - 1, initialExternalMm);
    const double targetMm = externalTargetAfter(commands, commandId, initialExternalMm);
    if (startMm == targetMm) return targetMm;
    if (completedCommandId >= commandId) return targetMm;

    const double progress = std::max(0.0, std::min(1.0, pathProgress));
    return startMm + (targetMm - startMm) * progress;
}

} // namespace

const std::array<double, 6>* targetJointsForNode(const RobotProgramNode& node) {
    switch (node.type) {
    case RobotProgramNodeType::MoveJ:
        return &std::get<MoveJData>(node.data).targetJoints;
    case RobotProgramNodeType::MoveL:
        return &std::get<MoveLData>(node.data).targetJoints;
    case RobotProgramNodeType::MoveC:
        return &std::get<MoveCData>(node.data).targetJoints;
    case RobotProgramNodeType::Root:
    case RobotProgramNodeType::SetSpeed:
    case RobotProgramNodeType::SetBlending:
    case RobotProgramNodeType::SetWeave:
    case RobotProgramNodeType::WeaveOn:
    case RobotProgramNodeType::WeaveOff:
    case RobotProgramNodeType::SetTool:
    case RobotProgramNodeType::Actuate:
    case RobotProgramNodeType::Stop:
    case RobotProgramNodeType::Trigger:
        break;
    }
    return nullptr;
}

bool externalAxisTargetForNode(const RobotProgramNode& node, double* positionMm) {
    if (node.type == RobotProgramNodeType::MoveJ) {
        const MoveJData& move = std::get<MoveJData>(node.data);
        if (!move.hasExternalAxis) return false;
        if (positionMm) *positionMm = move.externalAxisPositionMm;
        return true;
    }
    if (node.type == RobotProgramNodeType::MoveL) {
        const MoveLData& move = std::get<MoveLData>(node.data);
        if (!move.hasExternalAxis) return false;
        if (positionMm) *positionMm = move.externalAxisPositionMm;
        return true;
    }
    return false;
}

bool applyStateInstruction(RobotProgramSimulator::ProgramState& state,
                           const RobotProgramNode& node) {
    switch (node.type) {
    case RobotProgramNodeType::SetSpeed: {
        const SetSpeedData& speed = std::get<SetSpeedData>(node.data);
        state.jointSpeedRadPerSec = std::max(1.0e-6, speed.jointSpeedRadPerSec);
        state.linearSpeedMmPerSec = std::max(1.0e-6, speed.linearSpeedMmPerSec);
        return true;
    }
    case RobotProgramNodeType::SetBlending:
        state.blendRadiusMm = std::get<SetBlendingData>(node.data).radiusMm;
        return true;
    case RobotProgramNodeType::SetWeave:
        state.weaveInline = std::get<SetWeaveData>(node.data).params;
        return true;
    case RobotProgramNodeType::WeaveOn:
        state.weaveEnabled = true;
        state.weaveScheduleIndex = std::get<WeaveOnData>(node.data).scheduleIndex;
        return true;
    case RobotProgramNodeType::WeaveOff:
        state.weaveEnabled = false;
        state.weaveScheduleIndex = -1;
        return true;
    case RobotProgramNodeType::Root:
    case RobotProgramNodeType::SetTool:
    case RobotProgramNodeType::Actuate:
    case RobotProgramNodeType::Stop:
    case RobotProgramNodeType::Trigger:
    case RobotProgramNodeType::MoveJ:
    case RobotProgramNodeType::MoveL:
    case RobotProgramNodeType::MoveC:
        break;
    }
    return false;
}

RobotMotionCore::Transform toMotionTransform(const CadTransform& transform) {
    RobotMotionCore::Transform out{};
    for (int i = 0; i < 12; ++i) out.values[i] = transform.values[static_cast<size_t>(i)];
    return out;
}

CadTransform fromMotionTransform(const RobotMotionCore::Transform& transform) {
    CadTransform out;
    for (int i = 0; i < 12; ++i) out.values[static_cast<size_t>(i)] = transform.values[i];
    return out;
}

void RobotProgramSimulator::start(const RobotProgramNode* programRoot,
                                  const std::array<double, 6>& startJoints) {
    start(programRoot, startJoints, ProgramState{});
}

void RobotProgramSimulator::start(const RobotProgramNode* programRoot,
                                  const std::array<double, 6>& startJoints,
                                  const ProgramState& initialState) {
    m_root = programRoot;
    m_status = Status::Running;
    m_programState = initialState;
    m_currentJoints = startJoints;
    m_elapsedSeconds = 0.0;
    m_sampleIndex = 0;
    m_planBuilt = false;
    m_samples.clear();
    m_markers.clear();
    m_warnings.clear();
    m_statistics = {};
}

void RobotProgramSimulator::stop() {
    m_root = nullptr;
    m_status = Status::Idle;
    m_elapsedSeconds = 0.0;
    m_sampleIndex = 0;
    m_planBuilt = false;
}

void RobotProgramSimulator::setStepEstimator(const StepEstimator& estimator) {
    m_stepEstimator = estimator;
}

void RobotProgramSimulator::clearStepEstimator() {
    m_stepEstimator = {};
}

void RobotProgramSimulator::setMotionSettingsOverride(const MotionSettingsOverride& override) {
    m_motionSettingsOverride = override;
    m_planBuilt = false;
}

void RobotProgramSimulator::clearMotionSettingsOverride() {
    m_motionSettingsOverride = {};
    m_planBuilt = false;
}

void RobotProgramSimulator::configureExternalAxis(bool enabled, double startPositionMm) {
    m_externalAxisEnabled = enabled;
    m_externalAxisStartMm = startPositionMm;
    m_planBuilt = false;
}

void RobotProgramSimulator::setWeaveSchedules(const RobotMotionCore::WeaveScheduleTable& schedules) {
    m_weaveSchedules = schedules;
    m_planBuilt = false;
}

double RobotProgramSimulator::singularityThresholdRad() const {
    return m_effectiveSingularityThresholdRad > 0.0 ? m_effectiveSingularityThresholdRad
                                                    : kSingularityThresholdRad;
}

double RobotProgramSimulator::durationSeconds() const {
    return m_samples.empty() ? 0.0 : m_samples.back().timeSeconds;
}

void RobotProgramSimulator::setProgressCallback(std::function<bool(size_t)> progress) {
    m_progressCallback = std::move(progress);
}

RobotProgramSimulator::PlanInput RobotProgramSimulator::planInputFromPoseController(
    const RobotPoseController& poseController) {
    PlanInput plan;
    const OPW6RobotData* robot = poseController.robotData();
    if (!poseController.isBound() || !robot) return plan;
    plan.model = poseController.motionModel();
    plan.kinematics = poseController.kinematicSnapshot();
    plan.jointVelocityMaxRadS = robot->jointVelocityMaxRadS;
    plan.jointAccelerationMaxRadS2 = robot->jointAccelerationMaxRadS2;
    plan.jointJerkMaxRadS3 = robot->jointJerkMaxRadS3;
    plan.hasJointVelocityLimits = robot->hasJointVelocityLimits;
    plan.hasJointAccelerationLimits = robot->hasJointAccelerationLimits;
    plan.hasJointJerkLimits = robot->hasJointJerkLimits;
    plan.controlPeriodSec = robot->controlPeriodSec;
    plan.controllerMinTickGapUs = robot->controllerMinTickGapUs;
    plan.singularityThresholdRad = robot->singularityThresholdRad;
    plan.defaultJointSpeedDegPerSec = robot->defaultJointSpeedDegPerSec;
    plan.defaultLinearSpeedMmPerSec = robot->defaultLinearSpeedMmPerSec;
    plan.defaultLinearAccelerationMmSec2 = robot->defaultLinearAccelerationMmSec2;
    plan.defaultLinearJerkMmSec3 = robot->defaultLinearJerkMmSec3;
    plan.defaultToolAngularSpeedRadSec = robot->defaultToolAngularSpeedRadSec;
    plan.defaultToolAngularAccelerationRadSec2 = robot->defaultToolAngularAccelerationRadSec2;
    plan.defaultToolAngularJerkRadSec3 = robot->defaultToolAngularJerkRadSec3;
    plan.valid = true;
    return plan;
}

// The modal walk. Instructions establish speed, blending and weaving and stay in force; moves take a
// copy of whatever was in force when the walk reached them, and a Trigger is claimed by the next move.
std::vector<RobotProgramSimulator::FlatCommand>
RobotProgramSimulator::flattenProgram(const RobotKinematicSnapshot& kinematics) const {
    std::vector<FlatCommand> commands;
    if (!m_root) return commands;

    ProgramState state = m_programState;
    TriggerData pendingTrigger;
    bool havePendingTrigger = false;
    int nextTriggerId = 1;
    int childIndex = -1;
    for (const auto& child : m_root->children) {
        ++childIndex;
        if (!child) continue;
        switch (child->type) {
        case RobotProgramNodeType::SetSpeed:
        case RobotProgramNodeType::SetBlending:
        case RobotProgramNodeType::SetWeave:
        case RobotProgramNodeType::WeaveOn:
        case RobotProgramNodeType::WeaveOff:
            applyStateInstruction(state, *child);
            break;
        case RobotProgramNodeType::SetTool:
            // Tool/TCP selection is applied by the station runtime before the following motion.
            // Joint-space commands remain valid regardless of the selected TCP.
            break;
        case RobotProgramNodeType::Actuate:
            // A mechanism event is applied by the station runtime immediately before the next
            // motion instruction. It intentionally does not become a motion-planner command.
            break;
        case RobotProgramNodeType::Stop:
            // Program flow ends here. Instructions after Stop are intentionally unreachable in both
            // the timeline builder and live execution.
            return commands;
        case RobotProgramNodeType::Trigger:
            // Held for the next move to claim. One-shot, so not part of the modal state.
            pendingTrigger = std::get<TriggerData>(child->data);
            havePendingTrigger = true;
            break;
        case RobotProgramNodeType::MoveJ:
        case RobotProgramNodeType::MoveL:
        case RobotProgramNodeType::MoveC: {
            FlatCommand command;
            command.type = child->type;
            command.node = child.get();
            command.instruction = childIndex;
            command.state = state;
            if (havePendingTrigger) {
                command.trigger = pendingTrigger;
                command.triggerId = nextTriggerId++;
                havePendingTrigger = false;
            }
            if (const std::array<double, 6>* target = targetJointsForNode(*child)) {
                command.targetJoints = *target;
                command.targetPose = poseForJoints(kinematics, command.targetJoints);
            }
            command.hasExternalAxis =
                externalAxisTargetForNode(*child, &command.externalAxisPositionMm);
            if (child->type == RobotProgramNodeType::MoveC) {
                command.viaJoints = std::get<MoveCData>(child->data).viaJoints;
                command.viaPose = poseForJoints(kinematics, command.viaJoints);
            }
            commands.push_back(command);
            break;
        }
        case RobotProgramNodeType::Root:
            break;
        }
    }
    return commands;
}

bool RobotProgramSimulator::toMotionCommand(const FlatCommand& source,
                                            int commandId,
                                            double externalAxisStartMm,
                                            RobotMotionCore::MotionCommand* out,
                                            int* unresolvedSchedule) const {
    if (!out) return false;
    *out = {};
    out->type = source.type == RobotProgramNodeType::MoveJ
        ? RobotMotionCore::MotionCommandType::MoveJ
        : RobotMotionCore::MotionCommandType::MoveL;
    out->id = static_cast<int32_t>(commandId);
    out->targetTcp = toMotionTransform(source.targetPose);
    for (int joint = 0; joint < 6; ++joint) {
        out->targetQ[joint] = source.targetJoints[static_cast<size_t>(joint)];
    }
    out->jointSpeedDegPerSec = source.state.jointSpeedRadPerSec / kDegToRad;
    out->linearSpeedMmPerSec = source.state.linearSpeedMmPerSec;
    out->blendMm = source.state.blendRadiusMm;
    out->sampleMm = 0.0;
    if (m_externalAxisEnabled && source.hasExternalAxis) {
        const double externalDistanceMm =
            std::abs(source.externalAxisPositionMm - externalAxisStartMm);
        if (source.type == RobotProgramNodeType::MoveJ) {
            // MoveJ's path unit is radians. Convert the rail demand so the command's programmed
            // joint and linear speeds describe the same duration, then let the shared profile use
            // whichever of the robot or rail is the limiting axis.
            const double jointSpeedRadSec = std::max(1.0e-9, source.state.jointSpeedRadPerSec);
            const double railSpeedMmSec = std::max(1.0e-9, source.state.linearSpeedMmPerSec);
            out->coordinatedPathLength =
                externalDistanceMm * jointSpeedRadSec / railSpeedMmSec;
        } else {
            // MoveL and a linear rail are already expressed in millimetres.
            out->coordinatedPathLength = externalDistanceMm;
        }
    }
    // Resolved here, through the shared core, so the index in the program means the same pattern on
    // this side as it will on the robot.
    RobotMotionCore::WeaveParams request = source.state.weaveInline;
    request.enabled = source.state.weaveEnabled ? 1 : 0;
    request.scheduleIndex = source.state.weaveScheduleIndex >= 0
        ? static_cast<uint8_t>(source.state.weaveScheduleIndex)
        : RobotMotionCore::kWeaveScheduleInline;
    if (source.state.weaveEnabled && request.shape == RobotMotionCore::WeaveShape::None) {
        // A bare WeaveOn naming a schedule has no inline shape to carry; the schedule supplies one.
        // Marked active so resolveWeaveParams goes and looks it up.
        request.shape = RobotMotionCore::WeaveShape::Sine;
    }
    bool resolved = true;
    if (!RobotMotionCore::resolveWeaveParams(m_weaveSchedules, request, &out->weave)) {
        if (unresolvedSchedule) *unresolvedSchedule = source.state.weaveScheduleIndex;
        resolved = false;
    }

    out->trigger = RobotMotionCore::defaultMotionTrigger();
    if (source.triggerId >= 0) {
        out->trigger.id = source.triggerId;
        out->trigger.reference = source.trigger.referenceStart
            ? RobotMotionCore::MotionTriggerReference::Start
            : RobotMotionCore::MotionTriggerReference::End;
        out->trigger.distanceMm = source.trigger.distanceMm;
        out->trigger.timeMs = source.trigger.timeMs;
    }
    return resolved;
}

RobotMotionCore::MotionProgramSettings
RobotProgramSimulator::motionSettingsFor(const PlanInput& plan) {
    // Aliased as a pointer so the reads below still line up with the package field names.
    const PlanInput* robot = &plan;
    RobotMotionCore::MotionProgramSettings settings = RobotMotionCore::defaultMotionProgramSettings();
    settings.sampleMm = 0.0;
    settings.jointSampleDeg = kJointSampleRad / kDegToRad;
    settings.controlPeriodSec = robot->controlPeriodSec > 0.0 ? robot->controlPeriodSec : kMotionControlPeriodSeconds;
    settings.singularityThresholdRad = robot->singularityThresholdRad > 0.0 ? robot->singularityThresholdRad : kSingularityThresholdRad;
    settings.minBlendSamples = 6;
    // Scale the package arrays before deriving anything from them, so the per-axis caps and the
    // scalar defaults that applyJointDynamicsLimitsToMotionSettings takes as their minimum stay
    // consistent. Scaling afterwards would leave the scalars at the unscaled minimum.
    std::array<double, 6> scaledJointVelocityMaxRadS = robot->jointVelocityMaxRadS;
    std::array<double, 6> scaledJointAccelMaxRadS2 = robot->jointAccelerationMaxRadS2;
    std::array<double, 6> scaledJointJerkMaxRadS3 = robot->jointJerkMaxRadS3;
    for (size_t index = 0; index < scaledJointAccelMaxRadS2.size(); ++index) {
        const double scale = m_motionSettingsOverride.dynamicLimitScale[index];
        if (!(scale > 0.0) || scale == 1.0) continue;
        scaledJointVelocityMaxRadS[index] *= scale;
        scaledJointAccelMaxRadS2[index] *= scale;
        scaledJointJerkMaxRadS3[index] *= scale;
    }
    RobotMotionCore::applyJointDynamicsLimitsToMotionSettings(&settings,
                                                              scaledJointVelocityMaxRadS.data(),
                                                              scaledJointAccelMaxRadS2.data(),
                                                              scaledJointJerkMaxRadS3.data());
    if (robot->defaultJointSpeedDegPerSec > 0.0) settings.defaultJointSpeedDegPerSec = robot->defaultJointSpeedDegPerSec;
    if (robot->defaultLinearSpeedMmPerSec > 0.0) settings.defaultLinearSpeedMmPerSec = robot->defaultLinearSpeedMmPerSec;
    if (robot->defaultLinearAccelerationMmSec2 > 0.0) settings.defaultLinearAccelerationMmSec2 = robot->defaultLinearAccelerationMmSec2;
    if (robot->defaultLinearJerkMmSec3 > 0.0) settings.defaultLinearJerkMmSec3 = robot->defaultLinearJerkMmSec3;
    if (robot->defaultToolAngularSpeedRadSec > 0.0) settings.defaultToolAngularSpeedRadSec = robot->defaultToolAngularSpeedRadSec;
    if (robot->defaultToolAngularAccelerationRadSec2 > 0.0) settings.defaultToolAngularAccelerationRadSec2 = robot->defaultToolAngularAccelerationRadSec2;
    if (robot->defaultToolAngularJerkRadSec3 > 0.0) settings.defaultToolAngularJerkRadSec3 = robot->defaultToolAngularJerkRadSec3;

    // Caller-supplied limits win over the package so the GUI motion planner editors drive the
    // simulated path. Zero means "not overridden", the same convention the package fields use.
    const MotionSettingsOverride& gui = m_motionSettingsOverride;
    if (gui.controlPeriodSec > 0.0) settings.controlPeriodSec = gui.controlPeriodSec;
    if (gui.singularityThresholdRad > 0.0) settings.singularityThresholdRad = gui.singularityThresholdRad;
    if (gui.defaultJointAccelerationRadSec2 > 0.0) settings.defaultJointAccelerationRadSec2 = gui.defaultJointAccelerationRadSec2;
    if (gui.defaultJointJerkRadSec3 > 0.0) settings.defaultJointJerkRadSec3 = gui.defaultJointJerkRadSec3;
    if (gui.defaultLinearAccelerationMmSec2 > 0.0) settings.defaultLinearAccelerationMmSec2 = gui.defaultLinearAccelerationMmSec2;
    if (gui.defaultLinearJerkMmSec3 > 0.0) settings.defaultLinearJerkMmSec3 = gui.defaultLinearJerkMmSec3;
    if (gui.defaultToolAngularSpeedRadSec > 0.0) settings.defaultToolAngularSpeedRadSec = gui.defaultToolAngularSpeedRadSec;
    if (gui.defaultToolAngularAccelerationRadSec2 > 0.0) settings.defaultToolAngularAccelerationRadSec2 = gui.defaultToolAngularAccelerationRadSec2;
    if (gui.defaultToolAngularJerkRadSec3 > 0.0) settings.defaultToolAngularJerkRadSec3 = gui.defaultToolAngularJerkRadSec3;
    m_effectiveSingularityThresholdRad = settings.singularityThresholdRad;
    for (int joint = 0; joint < 6; ++joint) {
        const size_t index = static_cast<size_t>(joint);
        settings.jointVelocityLimitRadSec[joint] = robot->jointVelocityMaxRadS[index];
        settings.jointAccelerationLimitRadSec2[joint] = scaledJointAccelMaxRadS2[index];
        if (m_stepEstimator.enabled &&
            RobotMotionCore::validStepsPerDegree(m_stepEstimator.stepsPerDegree[index])) {
            const double tickGapUs = robot->controllerMinTickGapUs > 0.0 ? robot->controllerMinTickGapUs : 80.0;
            settings.jointStepsPerDegree[joint] = m_stepEstimator.stepsPerDegree[index];
            settings.jointStepRateLimitStepsSec[joint] = 1000000.0 / tickGapUs;
        }
    }
    return settings;
}

bool RobotProgramSimulator::refillLiveRing(std::string* errorMessage) {
    LiveRun& live = m_liveRun;
    if (live.commands.empty()) return true;
    const int64_t count = static_cast<int64_t>(live.commands.size());
    while (RobotMotionCore::motionCommandRingFree(live.ring) > 0) {
        // A run that is not looping stops feeding once the program has been queued once. A looping one
        // never does, which is exactly what keeps the lookahead full and the bead unbroken across the
        // wrap: the join between the last move and the first is planned like any other corner.
        if (!live.state.looping && live.nextCommandId >= count) break;
        const size_t index = static_cast<size_t>(live.nextCommandId % count);
        const FlatCommand& source = live.commands[index];
        RobotMotionCore::MotionCommand target = {};
        int unresolvedSchedule = -1;
        const double externalStartMm = externalTargetAfter(
            live.commands, live.nextCommandId - 1, live.externalAxisStartMm);
        if (!toMotionCommand(source, static_cast<int>(live.nextCommandId), externalStartMm,
                             &target, &unresolvedSchedule)) {
            if (errorMessage) {
                *errorMessage = strutil::format("Run rejected: weave schedule %1 has not been configured.")
                                    .arg(unresolvedSchedule);
            }
            return false;
        }
        // Only a run that ends has a final command. Marking one while looping would let the lookahead
        // release its retained tail and bring the arm to rest at the end of every cycle.
        const bool finalCommand = !live.state.looping && live.nextCommandId + 1 == count;
        if (!RobotMotionCore::motionCommandRingPush(&live.ring, target, finalCommand)) break;
        ++live.nextCommandId;
    }
    return true;
}

bool RobotProgramSimulator::beginLiveRun(const PlanInput& plan, bool loop, std::string* errorMessage) {
    endLiveRun();
    if (!m_root) {
        if (errorMessage) *errorMessage = "No robot program is loaded.";
        return false;
    }
    if (!plan.valid) {
        if (errorMessage) *errorMessage = "Robot pose controller is not bound.";
        return false;
    }
    if (!plan.hasJointVelocityLimits || !plan.hasJointAccelerationLimits || !plan.hasJointJerkLimits) {
        if (errorMessage) *errorMessage = "Robot package missing dynamics: a live run needs the joint limits.";
        return false;
    }
    if (!m_motionScratch) m_motionScratch = std::make_unique<MotionScratch>();

    LiveRun& live = m_liveRun;
    live = LiveRun();
    live.plan = plan;
    live.settings = motionSettingsFor(plan);
    live.commands = flattenProgram(plan.kinematics);
    if (!m_externalAxisEnabled && std::any_of(
            live.commands.begin(), live.commands.end(),
            [](const FlatCommand& command) { return command.hasExternalAxis; })) {
        if (errorMessage) *errorMessage =
            "Program uses J7 but this robot has no station motion_link to a mechanism.";
        return false;
    }
    live.stopAtEnd = std::any_of(
        m_root->children.begin(), m_root->children.end(),
        [](const std::unique_ptr<RobotProgramNode>& child) {
            return child && child->type == RobotProgramNodeType::Stop;
        });
    if (live.commands.empty()) {
        if (errorMessage) *errorMessage = "Program has no moves to run.";
        return false;
    }
    for (const FlatCommand& command : live.commands) {
        if (command.type == RobotProgramNodeType::MoveC) {
            if (errorMessage) *errorMessage = "Run rejected: MoveC is not supported by the shared robot motion planner.";
            return false;
        }
    }
    if (!(live.settings.controlPeriodSec > 0.0)) {
        if (errorMessage) *errorMessage = "Run rejected: the control period is not positive.";
        return false;
    }

    live.joints = m_currentJoints;
    live.initialJoints = m_currentJoints;
    live.externalAxisStartMm = m_externalAxisStartMm;
    double startQ[RobotMotionCore::kAxisCount] = {};
    for (int joint = 0; joint < 6; ++joint) startQ[joint] = live.joints[static_cast<size_t>(joint)];
    RobotMotionCore::clearMotionCommandRing(&live.ring);
    RobotMotionCore::beginMotionWindowRunner(plan.model,
                                             live.settings,
                                             startQ,
                                             &live.ring,
                                             &m_motionScratch->program,
                                             &m_motionScratch->segmentProgram,
                                             &m_motionScratch->sampler,
                                             &m_motionScratch->baseSegmentProgram,
                                             &live.runner);
    live.state.running = true;
    live.state.looping = loop && !live.stopAtEnd;
    live.state.status = "Running";
    live.state.instruction = live.commands.front().instruction;
    live.state.jointSpeedRadPerSec = live.commands.front().state.jointSpeedRadPerSec;
    live.state.linearSpeedMmPerSec = live.commands.front().state.linearSpeedMmPerSec;
    live.state.blendRadiusMm = live.commands.front().state.blendRadiusMm;
    live.state.weaveEnabled = live.commands.front().state.weaveEnabled;
    live.state.weaveScheduleIndex = live.commands.front().state.weaveScheduleIndex;
    live.state.weaveInline = live.commands.front().state.weaveInline;
    live.state.externalAxisValid = m_externalAxisEnabled;
    live.state.externalAxisPositionMm = m_externalAxisStartMm;

    std::string fillError;
    if (!refillLiveRing(&fillError)) {
        live.state.running = false;
        live.state.status = fillError;
        if (errorMessage) *errorMessage = fillError;
        return false;
    }
    return true;
}

void RobotProgramSimulator::endLiveRun() {
    const bool wasRunning = m_liveRun.state.running;
    m_liveRun = LiveRun();
    if (wasRunning) m_liveRun.state.status = "Stopped";
}

RobotProgramSimulator::LiveRunStep RobotProgramSimulator::stepLiveRun(double dtSeconds) {
    LiveRun& live = m_liveRun;
    if (!live.state.running) return LiveRunStep{};
    const double period = live.settings.controlPeriodSec;
    live.pendingSeconds += std::max(0.0, dtSeconds);

    // A backstop on one call, not the pacing policy. LiveRunDriver hands out a slice at a time and
    // decides what to do about a backlog, which is where that decision belongs - it has the clock. This
    // is left because a caller can still ask for any dt it likes, and a run that spent a whole second
    // inside one call would be a stall wherever it was called from.
    const int maxSamplesPerStep = 4096;
    int budget = maxSamplesPerStep;
    bool applied = false;
    while (live.state.running && live.pendingSeconds >= period && budget-- > 0) {
        if (!live.haveWindow) {
            // Each simulator owns its planner scratch.
            const RobotMotionCore::RejectCode planned = RobotMotionCore::planMotionWindow(&live.runner);
            if (planned != RobotMotionCore::RejectCode::Ok) {
                live.state.running = false;
                live.state.status = strutil::format("Run rejected by the planner: %1")
                                       .arg(motionRejectMessage(planned));
                break;
            }
            if (!live.runner.haveWindow) {
                if (live.ring.count > 0) {
                    live.state.running = false;
                    live.state.status = "Run could not fill enough queued lookahead to execute.";
                } else {
                    // The program is out of commands and none are coming, so this is the end of a run
                    // that was not looping. The cycle itself was already counted when the last
                    // command completed - counting it again here read as two passes of a program that
                    // ran once, with a cycle time of zero.
                    live.state.running = false;
                    live.state.status = live.stopAtEnd ? "Stopped by program" : "Complete";
                }
                break;
            }
            live.haveWindow = true;
            live.windowTimeSec = 0.0;
        }

        live.windowTimeSec += period;
        RobotMotionCore::PathSample sample = {};
        bool windowComplete = false;
        const RobotMotionCore::RejectCode sampled =
            RobotMotionCore::sampleMotionWindow(&live.runner, live.windowTimeSec, &sample, &windowComplete);
        if (sampled != RobotMotionCore::RejectCode::Ok) {
            live.state.running = false;
            live.state.status = sampled == RobotMotionCore::RejectCode::WindowSeamDiscontinuity
                ? strutil::format("Run lookahead windows do not meet: %1").arg(motionRejectMessage(sampled))
                : strutil::format("Run aborted by the planner: %1").arg(motionRejectMessage(sampled));
            if (sampled == RobotMotionCore::RejectCode::WindowSeamDiscontinuity) {
                const RobotMotionCore::PathSample& previous = live.runner.previousWindowSample;
                live.state.status += strutil::format(" (commands %1 -> %2; delta deg")
                    .arg(previous.commandId).arg(sample.commandId);
                for (int joint = 0; joint < 6; ++joint) {
                    live.state.status += strutil::format(" %1")
                        .arg(wrapRadians(sample.q[joint] - previous.q[joint]) / kDegToRad,
                             0, 'f', 4);
                }
                live.state.status += ")";
            }
            break;
        }
        live.pendingSeconds -= period;

        if (sample.valid) {
            for (int joint = 0; joint < 6; ++joint) {
                live.joints[static_cast<size_t>(joint)] = sample.q[joint];
            }
            m_currentJoints = live.joints;
            applied = true;
            live.state.runSeconds = live.runner.timeOffsetSec + sample.timeSec;
            // The command ids run on across cycles, so this is both which instruction and which pass.
            const int64_t count = static_cast<int64_t>(live.commands.size());
            if (count > 0 && sample.commandId >= 0) {
                const FlatCommand& active = live.commands[static_cast<size_t>(sample.commandId % count)];
                live.state.instruction = active.instruction;
                live.state.jointSpeedRadPerSec = active.state.jointSpeedRadPerSec;
                live.state.linearSpeedMmPerSec = active.state.linearSpeedMmPerSec;
                live.state.blendRadiusMm = active.state.blendRadiusMm;
                live.state.weaveEnabled = active.state.weaveEnabled;
                live.state.weaveScheduleIndex = active.state.weaveScheduleIndex;
                live.state.weaveInline = active.state.weaveInline;
                live.state.externalAxisPositionMm = externalAxisAtSample(
                    live.commands,
                    sample.commandId,
                    sample.completedCommandId,
                    live.externalAxisStartMm,
                    sample.pathProgress);
            }
            if (count > 0 && sample.completedCommandId >= 0) {
                const int cycles = static_cast<int>((sample.completedCommandId + 1) / count);
                if (cycles > live.state.completedCycles) {
                    live.state.completedCycles = cycles;
                    live.state.cycleSeconds = live.state.runSeconds - live.cycleStartSeconds;
                    live.cycleStartSeconds = live.state.runSeconds;
                }
            }
            live.state.cycleElapsedSeconds = live.state.runSeconds - live.cycleStartSeconds;
        }

        if (windowComplete) {
            if (!live.runner.prefixComplete) {
                live.state.running = false;
                live.state.status = "Run lookahead did not complete its executable prefix.";
                break;
            }
            // m_currentJoints is where this side says the arm is. No step estimator on a live run, so
            // it is the sample's own joints, and only the weave excursion comes off inside.
            RobotMotionCore::retireMotionWindow(&live.runner, sample, m_currentJoints.data());
            live.haveWindow = false;
            std::string fillError;
            if (!refillLiveRing(&fillError)) {
                live.state.running = false;
                live.state.status = fillError;
                break;
            }
        }
    }

    LiveRunStep result;
    result.running = live.state.running;
    result.jointsChanged = applied;
    return result;
}

bool RobotProgramSimulator::buildTrajectory(const PlanInput& plan, std::string* errorMessage) {
    if (m_planBuilt) return !m_samples.empty();
    m_planBuilt = true;
    m_samples.clear();
    m_markers.clear();
    m_warnings.clear();
    m_statistics = {};

    if (!m_root) {
        if (errorMessage) *errorMessage = "No robot program is loaded.";
        return false;
    }
    // Aliased as a pointer so the limit checks below read identically to the package fields.
    const PlanInput* robot = &plan;
    if (!plan.valid) {
        if (errorMessage) *errorMessage = "Robot pose controller is not bound.";
        return false;
    }
    if (!robot->hasJointVelocityLimits || !finitePositiveArray(robot->jointVelocityMaxRadS)) {
        if (errorMessage) *errorMessage = "Robot package missing dynamics: finite jointVelocityMaxRadS[6] is required for planning.";
        return false;
    }
    if (!robot->hasJointAccelerationLimits || !finitePositiveArray(robot->jointAccelerationMaxRadS2)) {
        if (errorMessage) *errorMessage = "Robot package missing dynamics: finite jointAccelerationMaxRadS2[6] is required for planning.";
        return false;
    }
    if (!robot->hasJointJerkLimits || !finitePositiveArray(robot->jointJerkMaxRadS3)) {
        if (errorMessage) *errorMessage = "Robot package missing dynamics: finite jointJerkMaxRadS3[6] is required for planning.";
        return false;
    }

    const std::vector<FlatCommand> commands = flattenProgram(plan.kinematics);
    if (!m_externalAxisEnabled && std::any_of(
            commands.begin(), commands.end(),
            [](const FlatCommand& command) { return command.hasExternalAxis; })) {
        if (errorMessage) *errorMessage =
            "Program uses J7 but this robot has no station motion_link to a mechanism.";
        return false;
    }

    CadTransform currentPose = poseForJoints(plan.kinematics, m_currentJoints);

    auto estimatedJoints = [&](const std::array<double, 6>& joints) {
        if (!m_stepEstimator.enabled) return joints;
        double q[RobotMotionCore::kAxisCount] = {};
        int32_t zeroSteps[RobotMotionCore::kAxisCount] = {};
        double stepsPerDegree[RobotMotionCore::kAxisCount] = {};
        int32_t roundedSteps[RobotMotionCore::kAxisCount] = {};
        double roundedQ[RobotMotionCore::kAxisCount] = {};
        for (int i = 0; i < 6; ++i) {
            const size_t index = static_cast<size_t>(i);
            q[i] = joints[index];
            zeroSteps[i] = m_stepEstimator.zeroSteps[index];
            stepsPerDegree[i] = m_stepEstimator.stepsPerDegree[index];
        }
        if (!RobotMotionCore::quantizeJointRadiansToSteps(q, zeroSteps, stepsPerDegree, roundedSteps, roundedQ)) {
            return joints;
        }
        std::array<double, 6> out = joints;
        for (int i = 0; i < 6; ++i) out[static_cast<size_t>(i)] = roundedQ[i];
        return out;
    };

    auto appendMarker = [&](double seconds, MarkerType type, const std::string& message) {
        // Triggers are never thinned. The window below exists to stop the planner reporting the same
        // cap over and over across neighbouring segments; a trigger is a distinct thing the program
        // asked for, and two of them a tenth of a second apart are two events, not one repeated.
        if (type != MarkerType::Trigger) {
            for (const Marker& marker : m_markers) {
                if (marker.type == type && std::abs(marker.seconds - seconds) < 0.25) return;
            }
        }
        m_markers.push_back({seconds, type, message});
    };

    for (const FlatCommand& command : commands) {
        if (command.type == RobotProgramNodeType::MoveC) {
            if (errorMessage) *errorMessage = "Run program rejected: MoveC is not supported by the shared robot motion planner.";
            return false;
        }
        if (command.type == RobotProgramNodeType::MoveJ && command.state.blendRadiusMm > 0.0) {
            m_warnings << "MoveJ blending is ignored by design; target is planned as FINE.";
        }
    }

    const RobotMotionCore::MotionProgramSettings settings = motionSettingsFor(plan);

    int32_t estimatorZeroSteps[RobotMotionCore::kAxisCount] = {};
    double estimatorStepsPerDegree[RobotMotionCore::kAxisCount] = {};
    bool collectStepExecutorStats = m_stepEstimator.enabled;
    if (collectStepExecutorStats) {
        for (int joint = 0; joint < 6; ++joint) {
            estimatorZeroSteps[joint] = m_stepEstimator.zeroSteps[static_cast<size_t>(joint)];
            estimatorStepsPerDegree[joint] = m_stepEstimator.stepsPerDegree[static_cast<size_t>(joint)];
        }
    }

    RobotMotionCore::StepExecutorStats stepExecutorStats = {};
    RobotMotionCore::MotionKinematicStatsObserver kinematicStats = {};
    RobotMotionCore::resetMotionKinematicStatsObserver(&kinematicStats);
    std::vector<const RobotProgramNode*> nodesById(commands.size(), nullptr);
    std::vector<double> linearSpeedById(commands.size(), std::numeric_limits<double>::quiet_NaN());
    if (!m_motionScratch) m_motionScratch = std::make_unique<MotionScratch>();
    RobotMotionCore::MotionWindowRunner runner = {};

    auto appendTimedSample = [&](const RobotMotionCore::PathSample& sourceSample, bool countStepExecutorSample) {
            RobotMotionCore::PathSample sample = sourceSample;
            sample.timeSec += runner.timeOffsetSec;
            RobotMotionCore::observeMotionKinematicSample(&kinematicStats, sourceSample);
            std::array<double, 6> joints{};
            for (int joint = 0; joint < 6; ++joint) joints[static_cast<size_t>(joint)] = sample.q[joint];
            if (collectStepExecutorStats && countStepExecutorSample) {
                RobotMotionCore::observeStepExecutorSample(sample.q,
                                                           estimatorZeroSteps,
                                                           estimatorStepsPerDegree,
                                                           &stepExecutorStats,
                                                           nullptr,
                                                           nullptr);
            }
            const std::array<double, 6> storedJoints = estimatedJoints(joints);
            CadTransform storedPose = fromMotionTransform(sample.tcp);
            if (m_stepEstimator.enabled) {
                storedPose = poseForJoints(plan.kinematics, storedJoints);
            }
            const CadTransform plannedPose = poseForJoints(plan.kinematics, joints);
            const int id = sample.commandId >= 0 && sample.commandId < static_cast<int32_t>(nodesById.size())
                ? sample.commandId
                : 0;
            const bool cartesian = sample.kind != RobotMotionCore::PathSampleKind::MoveJ;
            const double desiredTcp = cartesian ? linearSpeedById[static_cast<size_t>(id)]
                                                : std::numeric_limits<double>::quiet_NaN();
            const double profileTcp = cartesian
                ? sample.profileSpeed
                : std::numeric_limits<double>::quiet_NaN();
            const int32_t completedCommandId = sample.completedCommandId;
            PlannedSample plannedSample{sample.timeSec,
                                        storedJoints,
                                        joints,
                                        storedPose,
                                        plannedPose,
                                        std::numeric_limits<double>::quiet_NaN(),
                                        desiredTcp,
                                        profileTcp,
                                        nodesById[static_cast<size_t>(id)],
                                        completedCommandId,
                                        completedCommandId >= 0,
                                        sample.kind};
            plannedSample.weavePhaseCycles = sample.weavePhase;
            plannedSample.weaveLateralMm = sample.weaveOffsetMm[0];
            plannedSample.externalAxisValid = m_externalAxisEnabled;
            plannedSample.externalAxisPositionMm = externalAxisAtSample(
                commands,
                sample.commandId,
                sample.completedCommandId,
                m_externalAxisStartMm,
                sample.pathProgress);
            if (!m_samples.empty()) {
                // Thinning anchors on the last sample that was kept, not on the previous tick, so a
                // weave whose amplitude is under the threshold would have whole cycles collapsed
                // into a straight chord and appear not to be weaving at all. While the pattern is
                // running every tick is kept.
                const bool weaving = sample.weaveOffsetMm[0] != 0.0 || sample.weaveOffsetMm[1] != 0.0;
                const bool boundarySample = weaving || plannedSample.segmentEnd ||
                    plannedSample.activeNode != m_samples.back().activeNode ||
                    plannedSample.kind != m_samples.back().kind;
                const double tcpDistanceMm = distance(translationOf(plannedSample.tcpPose),
                                                      translationOf(m_samples.back().tcpPose));
                double jointDeltaRad = 0.0;
                for (int joint = 0; joint < 6; ++joint) {
                    const size_t index = static_cast<size_t>(joint);
                    jointDeltaRad += std::abs(wrapRadians(plannedSample.joints[index] - m_samples.back().joints[index]));
                }
                const double externalDeltaMm =
                    plannedSample.externalAxisValid && m_samples.back().externalAxisValid
                    ? std::abs(plannedSample.externalAxisPositionMm -
                               m_samples.back().externalAxisPositionMm)
                    : 0.0;
                if (!boundarySample &&
                    tcpDistanceMm < kMinStoredSampleTcpDistanceMm &&
                    jointDeltaRad < kMinStoredSampleJointDeltaRad &&
                    externalDeltaMm < 0.5) {
                    // Keep the last retained sample as the accumulation anchor. Replacing it here
                    // would collapse whole smooth spans into long chords because every control tick
                    // is close to the previous tick.
                } else {
                    m_samples.push_back(plannedSample);
                }
            } else {
                m_samples.push_back(plannedSample);
            }
            m_currentJoints = storedJoints;
            currentPose = storedPose;
    };

    double initialQ[RobotMotionCore::kAxisCount] = {};
    for (int joint = 0; joint < 6; ++joint) initialQ[joint] = m_currentJoints[static_cast<size_t>(joint)];
    if (collectStepExecutorStats) {
        int32_t startSteps[RobotMotionCore::kAxisCount] = {};
        if (RobotMotionCore::quantizeJointRadiansToSteps(initialQ, estimatorZeroSteps, estimatorStepsPerDegree, startSteps, nullptr)) {
            RobotMotionCore::beginStepExecutorStats(startSteps, &stepExecutorStats);
        } else {
            collectStepExecutorStats = false;
        }
    }

    RobotMotionCore::PathSample initialSample = {};
    initialSample.valid = 1;
    initialSample.timeSec = 0.0;
    initialSample.tcp = toMotionTransform(currentPose);
    initialSample.commandId = 0;
    initialSample.completedCommandId = -1;
    initialSample.kind = RobotMotionCore::PathSampleKind::MoveJ;
    for (int joint = 0; joint < 6; ++joint) initialSample.q[joint] = m_currentJoints[static_cast<size_t>(joint)];
    appendTimedSample(initialSample, false);

    RobotMotionCore::MotionCommandRing commandRing = {};
    RobotMotionCore::clearMotionCommandRing(&commandRing);
    RobotMotionCore::beginMotionWindowRunner(plan.model,
                                             settings,
                                             initialQ,
                                             &commandRing,
                                             &m_motionScratch->program,
                                             &m_motionScratch->segmentProgram,
                                             &m_motionScratch->sampler,
                                             &m_motionScratch->baseSegmentProgram,
                                             &runner);
    size_t nextCommandToQueue = 0;
    // Set when a program switched on a schedule that was never configured. Reported by index rather
    // than welding a straight bead and leaving the operator to notice.
    int unresolvedWeaveSchedule = -1;
    // The robot never carries trigger text, and neither does the planner; it is looked up here when a
    // trigger fires so the timeline marker reads as the program wrote it.
    std::map<int, std::string> triggerTextById;
    RobotMotionCore::PendingMotionTriggerQueue pendingTriggers = {};
    std::map<int, std::string> pendingTriggerText;
    auto fillCommandRing = [&]() {
        while (nextCommandToQueue < commands.size() && RobotMotionCore::motionCommandRingFree(commandRing) > 0) {
            const size_t commandIndex = nextCommandToQueue++;
            const FlatCommand& source = commands[commandIndex];
            RobotMotionCore::MotionCommand target = {};
            const double externalStartMm = externalTargetAfter(
                commands, static_cast<int64_t>(commandIndex) - 1, m_externalAxisStartMm);
            toMotionCommand(source, static_cast<int>(commandIndex), externalStartMm,
                            &target, &unresolvedWeaveSchedule);
            if (source.triggerId >= 0) triggerTextById[source.triggerId] = source.trigger.message;
            nodesById[commandIndex] = source.node;
            linearSpeedById[commandIndex] = source.state.linearSpeedMmPerSec;
            RobotMotionCore::motionCommandRingPush(&commandRing, target, commandIndex + 1 == commands.size());
        }
    };

    fillCommandRing();
    const auto weaveScheduleError = [&]() {
        if (unresolvedWeaveSchedule < 0) return false;
        if (errorMessage) {
            *errorMessage = strutil::format(
                                "Run program rejected: weave schedule %1 has not been configured.")
                                .arg(unresolvedWeaveSchedule);
        }
        return true;
    };
    if (weaveScheduleError()) return false;
    for (;;) {
        RobotMotionCore::RejectCode result = RobotMotionCore::planMotionWindow(&runner);
        if (result != RobotMotionCore::RejectCode::Ok) {
            if (errorMessage) {
                *errorMessage = strutil::format("Run program queued lookahead rejected by shared timed profile: %1")
                                    .arg(motionRejectMessage(result));
                const RobotMotionCore::MotionVerificationFailure& failure =
                    m_motionScratch->segmentProgram.verificationFailure;
                if (failure.kind != RobotMotionCore::MotionVerificationKind::None) {
                    *errorMessage += strutil::format(" (%1 at %2 s, observed %3 > limit %4, edge %5, segment %6, joint %7)")
                        .arg(std::string(RobotMotionCore::motionVerificationKindName(failure.kind)))
                        .arg(failure.timeSec + runner.timeOffsetSec, 0, 'f', 3)
                        .arg(failure.observed, 0, 'g', 6)
                        .arg(failure.limit, 0, 'g', 6)
                        .arg(failure.trajectoryEdgeIndex)
                        .arg(failure.pathSegmentIndex)
                        .arg(failure.joint < RobotMotionCore::kAxisCount ? failure.joint + 1 : 0);
                }
            }
            return false;
        }
        if (!runner.haveWindow) {
            if (commandRing.count > 0) {
                if (errorMessage) {
                    *errorMessage = "Run program could not fill enough queued lookahead to execute.";
                }
                return false;
            }
            break;
        }

        const RobotMotionCore::MotionSegmentProgram& segmentProgram = m_motionScratch->segmentProgram;
        const int32_t lastExecutedCommandId = runner.lastExecutedCommandId;
        m_statistics.trajectoryEdgeCount += segmentProgram.trajectory.edgeCount;
        m_statistics.septicEdgeCount += segmentProgram.polynomialEdgeCount;
        bool triggerMarked[RobotMotionCore::kMaxMotionCommands] = {};
        for (uint8_t segmentIndex = 0; segmentIndex < segmentProgram.count; ++segmentIndex) {
            const RobotMotionCore::MotionSegment& segment = segmentProgram.segments[segmentIndex];
            if (segment.commandId > lastExecutedCommandId) {
                continue;
            }
            tallyCapReasons(m_statistics.capReasonSegmentCounts, segment);
            const std::string reason = capReasonSummary(segment);
            if (!reason.empty()) {
                appendMarker(segment.startTimeSec + runner.timeOffsetSec, MarkerType::PlannerCap, reason);
            }
            const std::string blendWarning = blendFeasibilityWarning(segment);
            if (!blendWarning.empty() && std::find(m_warnings.begin(), m_warnings.end(), blendWarning) == m_warnings.end()) {
                m_warnings << blendWarning;
            }

            const uint8_t commandIndex = segment.commandIndex;
            if (commandIndex < RobotMotionCore::kMaxMotionCommands &&
                !triggerMarked[commandIndex] &&
                segmentProgram.triggerTimeSec[commandIndex] >= 0.0) {
                triggerMarked[commandIndex] = true;
                const int triggerId = segmentProgram.commandTrigger[commandIndex].id;
                const auto found = triggerTextById.find(triggerId);
                std::string text = found != triggerTextById.end() ? found->second : std::string();
                if (segmentProgram.triggerClamped[commandIndex]) {
                    text += " (clamped: offset reached outside the program)";
                }
                // Absolute run time, so a trigger reaching past this window can be compared against
                // a later one. The robot carries these the same way and for the same reason.
                pendingTriggerText[triggerId] = text;
                RobotMotionCore::pushPendingMotionTrigger(
                    &pendingTriggers, triggerId,
                    segmentProgram.triggerTimeSec[commandIndex] + runner.timeOffsetSec);
            }
        }

        bool windowComplete = false;
        double windowTime = settings.controlPeriodSec;
        RobotMotionCore::PathSample executedPrefixSample = {};
        while (!windowComplete) {
            RobotMotionCore::PathSample sample = {};
            result = RobotMotionCore::sampleMotionWindow(&runner, windowTime, &sample, &windowComplete);
            if (result != RobotMotionCore::RejectCode::Ok) {
                if (errorMessage) {
                    // Only motionCheckWindowSeam ever returns that code, which is what tells the two
                    // failures apart now that one call can report either.
                    *errorMessage = result == RobotMotionCore::RejectCode::WindowSeamDiscontinuity
                        ? strutil::format("Run program lookahead windows do not meet: %1")
                              .arg(motionRejectMessage(result))
                        : strutil::format("Run program aborted by shared timed profile: %1")
                              .arg(motionRejectMessage(result));
                }
                return false;
            }
            if (sample.valid) {
                appendTimedSample(sample, true);
                executedPrefixSample = sample;
            }
            windowTime += settings.controlPeriodSec;
        }
        if (!runner.prefixComplete) {
            if (errorMessage) {
                *errorMessage = "Run program queued lookahead did not complete its executable prefix.";
            }
            return false;
        }
        // Marked once the run has actually reached them. A trigger resolved past this window's
        // samples simply stays queued until a later window covers its time.
        {
            const double reachedSec = runner.timeOffsetSec + executedPrefixSample.timeSec;
            int32_t dueId = -1;
            double dueSec = 0.0;
            while (RobotMotionCore::takeDuePendingMotionTrigger(&pendingTriggers, reachedSec,
                                                               &dueId, &dueSec)) {
                const auto found = pendingTriggerText.find(dueId);
                // Placed at the time it was due, not the moment the drain noticed it.
                appendMarker(dueSec, MarkerType::Trigger,
                             found != pendingTriggerText.end() ? found->second : std::string());
            }
        }

        // m_currentJoints is what this side means by where the arm is: it carries the step estimator's
        // quantisation, and the robot genuinely is at that quantised position, so the seed keeps it.
        RobotMotionCore::retireMotionWindow(&runner, executedPrefixSample, m_currentJoints.data());
        fillCommandRing();
        if (weaveScheduleError()) return false;
        // One lookahead window is done, so the samples appended so far are final and safe to show.
        if (m_progressCallback && !m_progressCallback(m_samples.size())) {
            if (errorMessage) *errorMessage = "Trajectory build cancelled.";
            m_planBuilt = false;
            return false;
        }
    }
        // Accumulated across every window the runner crossed, and copied out once the run is planned.
        // It is what says whether the planner's windows agreed about where the arm was.
        m_statistics.worstWindowSeamSpeedMmPerSec = runner.worstSeamSpeedMmPerSec;

        {
            const double runEndSec = runner.timeOffsetSec;
            int32_t dueId = -1;
            double dueSec = 0.0;
            while (RobotMotionCore::takeDuePendingMotionTrigger(
                       &pendingTriggers, std::numeric_limits<double>::max(), &dueId, &dueSec)) {
                const auto found = pendingTriggerText.find(dueId);
                std::string text = found != pendingTriggerText.end() ? found->second : std::string();
                const double placed = dueSec > runEndSec ? runEndSec : dueSec;
                if (dueSec > runEndSec) text += " (clamped: offset reached past the last move)";
                appendMarker(placed, MarkerType::Trigger, text);
            }
        }

        if (collectStepExecutorStats) {
            m_statistics.stepExecutorStats = stepExecutorStats;
        }
        for (int joint = 0; joint < 6; ++joint) {
            const size_t index = static_cast<size_t>(joint);
            m_statistics.maxJointSpeedRadS[index] = kinematicStats.stats.maxJointSpeedRadS[joint];
            m_statistics.maxJointAccelerationRadS2[index] = kinematicStats.stats.maxJointAccelerationRadS2[joint];
            m_statistics.maxJointJerkRadS3[index] = kinematicStats.stats.maxJointJerkRadS3[joint];
        }
        m_statistics.maxTcpSpeedMmPerSec = kinematicStats.stats.maxTcpSpeedMmPerSec;
        m_statistics.maxTcpAccelerationMmS2 = kinematicStats.stats.maxTcpAccelerationMmS2;
        m_statistics.maxTcpJerkMmS3 = kinematicStats.stats.maxTcpJerkMmS3;
        m_statistics.maxToolAngularSpeedRadS = kinematicStats.stats.maxToolAngularSpeedRadS;
        m_statistics.maxToolAngularAccelerationRadS2 = kinematicStats.stats.maxToolAngularAccelerationRadS2;
        m_statistics.maxToolAngularJerkRadS3 = kinematicStats.stats.maxToolAngularJerkRadS3;

        for (size_t i = 1; i < m_samples.size(); ++i) {
            const double dt = std::max(kMinTimeStepSeconds, m_samples[i].timeSeconds - m_samples[i - 1].timeSeconds);
            // The tool's own motion, off the planned pose. Comparable against the profile and the
            // commanded speed, which the stepped version is not.
            const double plannedStep =
                distance(translationOf(m_samples[i - 1].plannedTcpPose), translationOf(m_samples[i].plannedTcpPose));
            m_samples[i].actualTcpSpeedMmPerSec = plannedStep / dt;
            const double steppedStep =
                distance(translationOf(m_samples[i - 1].tcpPose), translationOf(m_samples[i].tcpPose));
            m_samples[i].steppedTcpSpeedMmPerSec = steppedStep / dt;
            if (!std::isfinite(m_samples[i].profileTcpSpeedMmPerSec)) {
                m_samples[i].profileTcpSpeedMmPerSec = m_samples[i].actualTcpSpeedMmPerSec;
            }
        }
        auto reportsWristSingularity = [](const PlannedSample& sample) {
            return RobotMotionCore::pathSampleKindUsesCartesianSingularity(sample.kind);
        };
        const double singularityThreshold = settings.singularityThresholdRad;
        bool wasNearSingularity = reportsWristSingularity(m_samples.front()) &&
            nearSingularity(m_samples.front().joints, singularityThreshold);
        if (wasNearSingularity) appendMarker(0.0, MarkerType::Singularity, "Wrist singularity proximity");
        for (size_t i = 1; i < m_samples.size(); ++i) {
            const bool isNearSingularity = reportsWristSingularity(m_samples[i]) &&
                nearSingularity(m_samples[i].joints, singularityThreshold);
            if (isNearSingularity && !wasNearSingularity) {
                appendMarker(m_samples[i].timeSeconds, MarkerType::Singularity, "Wrist singularity proximity");
            }
            wasNearSingularity = isNearSingularity;
            if (jointFlipBetween(m_samples[i - 1].joints, m_samples[i].joints)) {
                appendMarker(m_samples[i].timeSeconds, MarkerType::JointFlip, "Forced IK branch flip");
            }
        }
        return true;
}

RobotProgramSimulator::StepResult RobotProgramSimulator::advance(double dtSeconds, RobotPoseController& poseController) {
    if (m_status != Status::Running) {
        return {m_status, m_currentJoints, {}, nullptr, false, std::numeric_limits<double>::quiet_NaN(),
                std::numeric_limits<double>::quiet_NaN(), 0.0, {}};
    }

    std::string errorMessage;
    if (!buildTrajectory(planInputFromPoseController(poseController), &errorMessage)) {
        return finish(Status::Error, errorMessage);
    }
    if (m_samples.empty()) {
        return finish(Status::Complete);
    }

    const double previousSeconds = m_elapsedSeconds;
    m_elapsedSeconds = std::min(durationSeconds(), m_elapsedSeconds + std::max(0.0, dtSeconds));
    StepResult result = sampleAt(m_elapsedSeconds);

    bool crossedSegmentEnd = false;
    while (m_sampleIndex + 1 < m_samples.size() && m_samples[m_sampleIndex + 1].timeSeconds <= m_elapsedSeconds + 1.0e-9) {
        ++m_sampleIndex;
        crossedSegmentEnd = crossedSegmentEnd || m_samples[m_sampleIndex].segmentEnd;
    }
    if (!crossedSegmentEnd) {
        auto begin = std::lower_bound(
            m_samples.begin(),
            m_samples.end(),
            previousSeconds,
            [](const PlannedSample& sample, double seconds) { return sample.timeSeconds < seconds; });
        auto end = std::upper_bound(
            m_samples.begin(),
            m_samples.end(),
            m_elapsedSeconds + 1.0e-9,
            [](double seconds, const PlannedSample& sample) { return seconds < sample.timeSeconds; });
        crossedSegmentEnd = std::any_of(begin, end, [](const PlannedSample& sample) { return sample.segmentEnd; });
    }
    result.segmentComplete = crossedSegmentEnd;

    const StepResult previous = sampleAt(previousSeconds);
    result.tcpDistanceTraveledMm = distance(translationOf(previous.tcpPose), translationOf(result.tcpPose));
    m_currentJoints = result.joints;

    if (m_elapsedSeconds >= durationSeconds() - 1.0e-9) {
        m_status = Status::Complete;
        result.status = Status::Complete;
        result.segmentComplete = true;
    }
    return result;
}

RobotProgramSimulator::StepResult RobotProgramSimulator::sampleAt(double seconds) const {
    if (m_samples.empty()) {
        return {m_status, m_currentJoints, {}, nullptr, false, std::numeric_limits<double>::quiet_NaN(),
                std::numeric_limits<double>::quiet_NaN(),
                std::numeric_limits<double>::quiet_NaN(), 0.0, {}};
    }
    if (seconds <= 0.0 || m_samples.size() == 1) {
        const PlannedSample& sample = m_samples.front();
        return {Status::Running, sample.joints, sample.tcpPose, sample.activeNode, sample.segmentEnd,
                sample.desiredTcpSpeedMmPerSec, sample.profileTcpSpeedMmPerSec, sample.actualTcpSpeedMmPerSec, 0.0, {}};
    }
    if (seconds >= durationSeconds()) {
        const PlannedSample& sample = m_samples.back();
        return {Status::Complete, sample.joints, sample.tcpPose, sample.activeNode, true,
                sample.desiredTcpSpeedMmPerSec, sample.profileTcpSpeedMmPerSec, sample.actualTcpSpeedMmPerSec, 0.0, {}};
    }

    auto it = std::lower_bound(
        m_samples.begin(),
        m_samples.end(),
        seconds,
        [](const PlannedSample& sample, double targetSeconds) { return sample.timeSeconds < targetSeconds; });
    if (it == m_samples.begin()) {
        const PlannedSample& sample = *it;
        return {Status::Running, sample.joints, sample.tcpPose, sample.activeNode, sample.segmentEnd,
                sample.desiredTcpSpeedMmPerSec, sample.profileTcpSpeedMmPerSec, sample.actualTcpSpeedMmPerSec, 0.0, {}};
    }

    const PlannedSample& next = *it;
    const PlannedSample& prev = *std::prev(it);
    const double span = std::max(kMinTimeStepSeconds, next.timeSeconds - prev.timeSeconds);
    const double t = clamp01((seconds - prev.timeSeconds) / span);
    std::array<double, 6> joints = prev.joints;
    for (int joint = 0; joint < 6; ++joint) {
        const size_t index = static_cast<size_t>(joint);
        joints[index] = prev.joints[index] + wrapRadians(next.joints[index] - prev.joints[index]) * t;
    }
    const CadTransform pose = interpolatePoseLinear(prev.tcpPose, next.tcpPose, t);
    return {Status::Running, joints, pose, next.activeNode, false,
            next.desiredTcpSpeedMmPerSec, next.profileTcpSpeedMmPerSec, next.actualTcpSpeedMmPerSec, 0.0, {}};
}

RobotProgramSimulator::StepResult RobotProgramSimulator::finish(Status status, const std::string& message) {
    m_status = status;
    if (!m_samples.empty()) {
        StepResult result = sampleAt(durationSeconds());
        result.status = status;
        result.message = message;
        return result;
    }
    return {m_status, m_currentJoints, {}, nullptr, false, std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(), 0.0, message};
}
