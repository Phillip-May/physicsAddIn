#include "../Common/RobotMotionCore.h"
#include "motion_core_fixtures.h"

#include <cmath>
#include <iostream>
#include <limits>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;

using motion_fixtures::makeAr4Model;

RobotMotionCore::RobotModel makeM800Model() {
    RobotMotionCore::RobotModel model{};
    const double dhm[24] = {
        0.0, -1.5707963267948966, 0.0, 0.0, 0.0, 3.141592653589793,
        0.0, 312.0, 800.0, 225.0, 0.0, 0.0,
        0.0, -1.5707963267948966, 0.0, -1.5707963267948966, 1.5707963267948966, -1.5707963267948966,
        600.0, 0.0, 0.0, 900.0, 0.0, 190.0
    };
    const double qMin[6] = {
        -185.0 * kDegToRad, -90.0 * kDegToRad, -160.0 * kDegToRad,
        -360.0 * kDegToRad, -125.0 * kDegToRad, -360.0 * kDegToRad
    };
    const double qMax[6] = {
        185.0 * kDegToRad, 135.0 * kDegToRad, 180.0 * kDegToRad,
        360.0 * kDegToRad, 125.0 * kDegToRad, 360.0 * kDegToRad
    };
    for (int i = 0; i < 24; ++i) model.dhm[i] = dhm[i];
    for (int i = 0; i < 6; ++i) {
        model.qHome[i] = 0.0;
        model.qMin[i] = qMin[i];
        model.qMax[i] = qMax[i];
        model.dhmSigns[i] = 1.0;
    }
    model.toolBindPose = RobotMotionCore::identityTransform();
    model.valid = 1;
    return model;
}

bool planMoveLWithTarget(const RobotMotionCore::RobotModel& model,
                         const double startQ[RobotMotionCore::kAxisCount],
                         const double targetQ[RobotMotionCore::kAxisCount],
                         RobotMotionCore::RejectCode expected,
                         const char* label) {
    RobotMotionCore::MoveLInput input{};
    input.model = model;
    for (int i = 0; i < RobotMotionCore::kAxisCount; ++i) {
        input.startQ[i] = startQ[i];
        input.targetConfigQ[i] = targetQ[i];
    }
    input.targetTcp = RobotMotionCore::toolPoseForJoints(model, targetQ);
    input.speedMmPerSec = 25.0;
    input.sampleMm = 0.0;
    input.singularityThresholdRad = 5.0 * kDegToRad;
    input.singularityPolicy = RobotMotionCore::SingularityPolicy::Abort;
    input.requireTargetConfig = 1;
    input.targetConfigToleranceRad = 0.5 * kDegToRad;

    RobotMotionCore::MoveLPlan plan{};
    const RobotMotionCore::RejectCode actual = RobotMotionCore::planMoveLEndpoint(input, &plan);
    if (actual != expected) {
        std::cerr << label << ": expected " << RobotMotionCore::rejectCodeName(expected)
                  << ", got " << RobotMotionCore::rejectCodeName(actual) << "\n";
        return false;
    }
    return true;
}

RobotMotionCore::Transform translatedPose(double x, double y, double z) {
    RobotMotionCore::Transform pose = RobotMotionCore::identityTransform();
    pose.values[3] = x;
    pose.values[7] = y;
    pose.values[11] = z;
    return pose;
}

bool near(double actual, double expected, double tolerance) {
    return std::fabs(actual - expected) <= tolerance;
}

}  // namespace

int main() {
    const RobotMotionCore::RobotModel model = makeAr4Model();
    if (!RobotMotionCore::modelIsValid(model)) {
        std::cerr << "AR4 motion model is invalid\n";
        return 1;
    }

    const double startQ[RobotMotionCore::kAxisCount] = {0.0, 0.0, 0.0, 0.0, 15.0 * kDegToRad, 0.0};
    const double targetQ[RobotMotionCore::kAxisCount] = {0.0, 10.0 * kDegToRad, -10.0 * kDegToRad, 0.0, 20.0 * kDegToRad, 0.0};
    double wrongTargetQ[RobotMotionCore::kAxisCount] = {};
    for (int i = 0; i < RobotMotionCore::kAxisCount; ++i) wrongTargetQ[i] = targetQ[i];
    wrongTargetQ[1] += 5.0 * kDegToRad;

    if (!planMoveLWithTarget(model, startQ, targetQ, RobotMotionCore::RejectCode::Ok, "matching target config")) {
        return 1;
    }

    // The taught target identifies the intended IK branch.
    const RobotMotionCore::RobotModel m800 = makeM800Model();
    const double m800StartQ[RobotMotionCore::kAxisCount] = {
        91.3 * kDegToRad, -1.2 * kDegToRad, 47.7 * kDegToRad,
        -189.1 * kDegToRad, 46.9 * kDegToRad, 184.5 * kDegToRad
    };
    const double m800TargetQ[RobotMotionCore::kAxisCount] = {
        92.4 * kDegToRad, -10.5 * kDegToRad, 13.3 * kDegToRad,
        -243.5 * kDegToRad, 6.2 * kDegToRad, 241.6 * kDegToRad
    };
    if (!planMoveLWithTarget(m800, m800StartQ, m800TargetQ, RobotMotionCore::RejectCode::Ok,
                             "M-800 taught wrist branch")) {
        return 1;
    }

    RobotMotionCore::MoveLInput mismatchInput{};
    mismatchInput.model = model;
    for (int i = 0; i < RobotMotionCore::kAxisCount; ++i) {
        mismatchInput.startQ[i] = startQ[i];
        mismatchInput.targetConfigQ[i] = wrongTargetQ[i];
    }
    mismatchInput.targetTcp = RobotMotionCore::toolPoseForJoints(model, targetQ);
    mismatchInput.speedMmPerSec = 25.0;
    mismatchInput.sampleMm = 0.0;
    mismatchInput.singularityThresholdRad = 5.0 * kDegToRad;
    mismatchInput.singularityPolicy = RobotMotionCore::SingularityPolicy::Abort;
    mismatchInput.requireTargetConfig = 1;
    mismatchInput.targetConfigToleranceRad = 0.5 * kDegToRad;

    RobotMotionCore::MoveLPlan mismatchPlan{};
    const RobotMotionCore::RejectCode mismatch =
        RobotMotionCore::planMoveLEndpoint(mismatchInput, &mismatchPlan);
    if (mismatch != RobotMotionCore::RejectCode::TargetConfigMismatch) {
        std::cerr << "mismatched target config: expected target_config_mismatch, got "
                  << RobotMotionCore::rejectCodeName(mismatch) << "\n";
        return 1;
    }

    RobotMotionCore::MoveLBlendPlan blend{};
    if (!RobotMotionCore::planMoveLBlend(translatedPose(0.0, -100.0, 0.0),
                                         translatedPose(0.0, 0.0, 0.0),
                                         translatedPose(100.0, 0.0, 0.0),
                                         2.0,
                                         &blend)) {
        std::cerr << "90 degree blend zone failed to plan\n";
        return 1;
    }

    const double expectedRadius = 2.0 / std::tan(45.0 * kDegToRad);
    if (!near(blend.actualDeviationMm, 2.0, 1.0e-6) ||
        !near(blend.radiusMm, expectedRadius, 1.0e-6) ||
        !near(blend.trimMm, 2.0, 1.0e-6)) {
        std::cerr << "blend_mm is not being treated as an along-path endpoint zone: radius="
                  << blend.radiusMm << " deviation=" << blend.actualDeviationMm << "\n";
        return 1;
    }

    RobotMotionCore::MoveLBlendPlan straightBlend{};
    if (RobotMotionCore::planMoveLBlend(translatedPose(0.0, -100.0, 0.0),
                                        translatedPose(0.0, 0.0, 0.0),
                                        translatedPose(0.0, 100.0, 0.0),
                                        2.0,
                                        &straightBlend)) {
        std::cerr << "straight-line blend should be rejected as degenerate\n";
        return 1;
    }

    RobotMotionCore::MotionProgram badProgram{};
    RobotMotionCore::MotionProgramSettings settings = RobotMotionCore::defaultMotionProgramSettings();
    badProgram.count = 1;
    badProgram.commands[0].type = RobotMotionCore::MotionCommandType::MoveL;
    badProgram.commands[0].targetTcp = translatedPose(10.0, 0.0, 0.0);
    badProgram.commands[0].linearSpeedMmPerSec = 25.0;
    badProgram.commands[0].jointSpeedDegPerSec = 10.0;
    badProgram.commands[0].blendMm = std::numeric_limits<double>::quiet_NaN();
    badProgram.commands[0].sampleMm = 0.0;
    if (RobotMotionCore::validateMotionProgramInput(badProgram, settings) != RobotMotionCore::RejectCode::BadSpeed) {
        std::cerr << "NaN blend input should be rejected before planning\n";
        return 1;
    }

    RobotMotionCore::MotionSegment segment{};
    segment.kind = RobotMotionCore::MotionSegmentKind::MoveLLine;
    segment.startTcp = translatedPose(0.0, 0.0, 0.0);
    segment.endTcp = translatedPose(100.0, 0.0, 0.0);
    segment.length = 100.0;
    segment.nominalSpeed = 100.0;
    segment.commandSpeed = 100.0;
    segment.acceleration = 1000.0;
    segment.commandAcceleration = 1000.0;
    segment.jerk = 10000.0;
    RobotMotionCore::initializeMotionCapDiagnostics(&segment);
    RobotMotionCore::profileMotionSegment(&segment, 0.0, 0.0, 0.0);
    if (RobotMotionCore::validateMotionSegment(segment) != RobotMotionCore::RejectCode::Ok) {
        std::cerr << "valid profiled segment failed invariant validation\n";
        return 1;
    }
    segment.durationSec = -1.0;
    if (RobotMotionCore::validateMotionSegment(segment) != RobotMotionCore::RejectCode::BadDuration) {
        std::cerr << "negative segment duration should be rejected\n";
        return 1;
    }

    std::cout << "motion core endpoint config smoke passed\n";
    return 0;
}
