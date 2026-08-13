// Direct tests for the weave machinery, aimed at the parts that were only reasoned about.
#include "../Common/RobotMotionCore.h"

#include <cmath>
#include <iostream>

namespace {

constexpr double kPi = 3.14159265358979323846;

RobotMotionCore::MotionProgramSettings makeSettings() {
    RobotMotionCore::MotionProgramSettings settings{};
    settings.controlPeriodSec = 0.005;
    settings.defaultLinearAccelerationMmSec2 = 1000.0;
    settings.defaultLinearJerkMmSec3 = 10000.0;
    for (uint8_t joint = 0; joint < RobotMotionCore::kAxisCount; ++joint) {
        // 2 rad/s, close to the AR4's own caps, so the numbers below read against something real.
        settings.jointVelocityLimitRadSec[joint] = 2.0;
        settings.jointAccelerationLimitRadSec2[joint] = 8.0;
    }
    return settings;
}

RobotMotionCore::PathSample makeSample(double timeSec, double q0) {
    RobotMotionCore::PathSample sample{};
    sample.valid = 1;
    sample.timeSec = timeSec;
    sample.tcp = RobotMotionCore::identityTransform();
    sample.tcp.values[3] = q0 * 1000.0;   // a metre of tool travel per radian, so a step is visible
    for (uint8_t joint = 0; joint < RobotMotionCore::kAxisCount; ++joint) {
        sample.q[joint] = joint == 0 ? q0 : 0.0;
        sample.baseQ[joint] = sample.q[joint];
    }
    sample.baseTcp = sample.tcp;
    return sample;
}

RobotMotionCore::WeaveParams makeWeave(double amplitudeLeft, double amplitudeRight) {
    RobotMotionCore::WeaveParams weave = RobotMotionCore::defaultWeaveParams();
    weave.shape = RobotMotionCore::WeaveShape::Sine;
    weave.rateMode = RobotMotionCore::WeaveRateMode::Wavelength;
    weave.wavelengthMm = 8.0;
    weave.amplitudeLeftMm = amplitudeLeft;
    weave.amplitudeRightMm = amplitudeRight;
    weave.enabled = 1;
    return weave;
}

bool nearly(double value, double expected, double tolerance) {
    return std::fabs(value - expected) <= tolerance;
}

}   // namespace

int main() {
    const RobotMotionCore::MotionProgramSettings settings = makeSettings();
    const double dt = settings.controlPeriodSec;

    // A join the windows agree on: one control period of motion well inside the joint limit.
    {
        const RobotMotionCore::PathSample previous = makeSample(1.0, 0.0);
        const RobotMotionCore::PathSample next = makeSample(1.0 + dt, 0.005);   // 1 rad/s
        double worst = 0.0;
        if (RobotMotionCore::motionCheckWindowSeam(previous, next, dt, settings, &worst) !=
            RobotMotionCore::RejectCode::Ok) {
            std::cerr << "a continuous seam was rejected\n";
            return 1;
        }
        if (!nearly(worst, 1000.0, 1.0)) {
            std::cerr << "seam speed should be reported even when the seam passes, got " << worst << "\n";
            return 1;
        }
    }

    {
        const RobotMotionCore::PathSample previous = makeSample(1.0, 0.0);
        const RobotMotionCore::PathSample next = makeSample(1.0 + dt, 0.1);
        double worst = 0.0;
        if (RobotMotionCore::motionCheckWindowSeam(previous, next, dt, settings, &worst) !=
            RobotMotionCore::RejectCode::WindowSeamDiscontinuity) {
            std::cerr << "a discontinuous seam was accepted\n";
            return 1;
        }
    }

    // The first window has no previous seam sample.
    {
        const RobotMotionCore::PathSample none{};
        const RobotMotionCore::PathSample next = makeSample(dt, 0.1);
        double worst = 0.0;
        if (RobotMotionCore::motionCheckWindowSeam(none, next, dt, settings, &worst) !=
            RobotMotionCore::RejectCode::Ok) {
            std::cerr << "the first window was compared against nothing\n";
            return 1;
        }
    }

    // Ramp gain is zero at both run endpoints.
    {
        const RobotMotionCore::WeaveParams weave = makeWeave(1.5, 1.5);
        const double extent = RobotMotionCore::weaveCycleExtent(weave);
        if (!nearly(RobotMotionCore::weaveRampGain(weave, 0.0, RobotMotionCore::kWeaveRunOpenEnded), 0.0, 1.0e-12)) {
            std::cerr << "the ramp does not start from zero\n";
            return 1;
        }
        if (!nearly(RobotMotionCore::weaveRampGain(weave, 1.0, RobotMotionCore::kWeaveRunOpenEnded), 1.0, 1.0e-12)) {
            std::cerr << "the ramp does not reach full amplitude after one cycle\n";
            return 1;
        }
        if (!nearly(RobotMotionCore::weaveRampGain(weave, 5.0, 0.0), 0.0, 1.0e-12)) {
            std::cerr << "the ramp does not return to zero at the end of a run\n";
            return 1;
        }
        if (!nearly(RobotMotionCore::weaveRampGain(weave, 5.0, extent), 1.0, 1.0e-12)) {
            std::cerr << "the ramp tapers a run that is still a full cycle from its end\n";
            return 1;
        }
        // Monotone in, so the taper never doubles back and adds a reversal of its own.
        double last = -1.0;
        for (int step = 0; step <= 20; ++step) {
            const double gain = RobotMotionCore::weaveRampGain(
                weave, static_cast<double>(step) / 20.0, RobotMotionCore::kWeaveRunOpenEnded);
            if (gain < last - 1.0e-12) {
                std::cerr << "the ramp is not monotone\n";
                return 1;
            }
            last = gain;
        }
    }

    {
        const RobotMotionCore::WeaveParams weave = makeWeave(2.0, 1.0);
        double atLeft[2] = {};
        double atRight[2] = {};
        double atCentre[2] = {};
        RobotMotionCore::weaveShapeOffset(weave, 0.25, atLeft);      // sine at +1
        RobotMotionCore::weaveShapeOffset(weave, 0.75, atRight);     // sine at -1
        RobotMotionCore::weaveShapeOffset(weave, 0.5, atCentre);     // sine at 0
        if (!nearly(atLeft[0], 2.0, 1.0e-9) || !nearly(atRight[0], -1.0, 1.0e-9)) {
            std::cerr << "unsymmetrical amplitudes are not exact at the extremes: "
                      << atLeft[0] << " and " << atRight[0] << "\n";
            return 1;
        }
        if (!nearly(atCentre[0], 0.0, 1.0e-9)) {
            std::cerr << "the pattern does not cross the taught line at the centre\n";
            return 1;
        }
        // Symmetric amplitudes must be untouched by the asymmetric path.
        const RobotMotionCore::WeaveParams even = makeWeave(1.5, 1.5);
        double evenLeft[2] = {};
        double evenRight[2] = {};
        RobotMotionCore::weaveShapeOffset(even, 0.25, evenLeft);
        RobotMotionCore::weaveShapeOffset(even, 0.75, evenRight);
        if (!nearly(evenLeft[0], 1.5, 1.0e-9) || !nearly(evenRight[0], -1.5, 1.0e-9)) {
            std::cerr << "symmetric amplitudes changed\n";
            return 1;
        }
        // Check slope continuity across the centre crossing.
        const double h = 1.0e-6;
        double before[2] = {};
        double after[2] = {};
        RobotMotionCore::weaveShapeOffset(weave, 0.5 - h, before);
        RobotMotionCore::weaveShapeOffset(weave, 0.5 + h, after);
        const double slopeBefore = (atCentre[0] - before[0]) / h;
        const double slopeAfter = (after[0] - atCentre[0]) / h;
        if (!nearly(slopeBefore, slopeAfter, 1.0e-3 * std::fabs(slopeBefore))) {
            std::cerr << "the slope still breaks at the centre crossing: "
                      << slopeBefore << " into " << slopeAfter << "\n";
            return 1;
        }
    }

    {
        const RobotMotionCore::RejectCode codes[5] = {
            RobotMotionCore::RejectCode::WeaveInfeasible,
            RobotMotionCore::RejectCode::WeaveNotConfigured,
            RobotMotionCore::RejectCode::WeaveOnJointMove,
            RobotMotionCore::RejectCode::WeaveDerateExhausted,
            RobotMotionCore::RejectCode::WeaveDerateOverflow
        };
        for (int i = 0; i < 5; ++i) {
            for (int j = i + 1; j < 5; ++j) {
                if (codes[i] == codes[j]) {
                    std::cerr << "two weave reject codes collide\n";
                    return 1;
                }
                if (std::string(RobotMotionCore::rejectCodeName(codes[i])) ==
                    RobotMotionCore::rejectCodeName(codes[j])) {
                    std::cerr << "two weave reject codes share a name\n";
                    return 1;
                }
            }
            if (std::string(RobotMotionCore::rejectCodeName(codes[i])) == "unknown") {
                std::cerr << "a weave reject code has no name\n";
                return 1;
            }
        }
    }

    // Replan seeds remove the current weave excursion.
    {
        RobotMotionCore::PathSample seam = makeSample(1.0, 0.5);
        // Pretend the weave had pushed joint 0 a hundredth of a radian off the taught path.
        seam.q[0] = 0.51;
        seam.baseQ[0] = 0.50;
        const double actual[RobotMotionCore::kAxisCount] = {0.51, 0.0, 0.0, 0.0, 0.0, 0.0};
        double seed[RobotMotionCore::kAxisCount] = {};
        RobotMotionCore::motionReplanSeedQ(seam, actual, seed);
        if (!nearly(seed[0], 0.50, 1.0e-12)) {
            std::cerr << "the replan seed keeps the weave excursion, got " << seed[0] << "\n";
            return 1;
        }
        // With no weave the seed must be the caller's own state, untouched, or unwoven paths shift.
        RobotMotionCore::PathSample unwoven = makeSample(1.0, 0.5);
        RobotMotionCore::motionReplanSeedQ(unwoven, actual, seed);
        if (!nearly(seed[0], 0.51, 1.0e-12)) {
            std::cerr << "the replan seed moved on an unwoven path\n";
            return 1;
        }
    }

    std::cout << "motion core weave smoke passed\n";
    return 0;
}
