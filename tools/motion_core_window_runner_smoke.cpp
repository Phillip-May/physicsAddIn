// Does MotionWindowRunner produce exactly the sample stream the hand-written lookahead loop produced?
#include "../Common/RobotMotionCore.h"
#include "motion_core_fixtures.h"

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace RMC = RobotMotionCore;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;

using motion_fixtures::makeAr4Model;

RMC::MotionProgramSettings makeSettings() {
    RMC::MotionProgramSettings settings = RMC::defaultMotionProgramSettings();
    settings.sampleMm = 0.0;
    settings.jointSampleDeg = 1.0;
    settings.controlPeriodSec = 0.005;
    settings.singularityThresholdRad = 5.0 * kDegToRad;
    settings.minBlendSamples = 6;
    settings.verifySampledDynamics = 1;
    for (uint8_t joint = 0; joint < RMC::kAxisCount; ++joint) {
        settings.jointVelocityLimitRadSec[joint] = 2.0;
        settings.jointAccelerationLimitRadSec2[joint] = 8.0;
    }
    settings.defaultLinearAccelerationMmSec2 = 1000.0;
    settings.defaultLinearJerkMmSec3 = 10000.0;
    settings.defaultJointAccelerationRadSec2 = 60.0 * kDegToRad;
    settings.defaultJointJerkRadSec3 = 600.0 * kDegToRad;
    return settings;
}

struct Config {
    const char* name;
    double blendMm;
    bool weave;
    bool joints;      // MoveJ instead of MoveL
    int commandCount;
    double stepScale;
};

// Targets walked in joint space so every one of them is reachable and consistent with its pose.
std::vector<RMC::MotionCommand> makeCommands(const RMC::RobotModel& model,
                                             const double startQ[RMC::kAxisCount],
                                             const Config& config) {
    std::vector<RMC::MotionCommand> commands;
    double q[RMC::kAxisCount];
    for (uint8_t i = 0; i < RMC::kAxisCount; ++i) q[i] = startQ[i];
    for (int index = 0; index < config.commandCount; ++index) {
        q[0] += 0.05 * config.stepScale;
        q[1] += ((index % 2 == 0) ? 0.02 : -0.02) * config.stepScale;
        RMC::MotionCommand command{};
        command.type = config.joints ? RMC::MotionCommandType::MoveJ : RMC::MotionCommandType::MoveL;
        command.id = index;
        command.targetTcp = RMC::toolPoseForJoints(model, q);
        for (uint8_t i = 0; i < RMC::kAxisCount; ++i) command.targetQ[i] = q[i];
        command.jointSpeedDegPerSec = 20.0;
        command.linearSpeedMmPerSec = 60.0;
        command.blendMm = config.blendMm;
        command.sampleMm = 0.0;
        command.weave = RMC::defaultWeaveParams();
        if (config.weave) {
            command.weave.enabled = 1;
            command.weave.shape = RMC::WeaveShape::Sine;
            command.weave.rateMode = RMC::WeaveRateMode::Wavelength;
            command.weave.wavelengthMm = 10.0;
            command.weave.amplitudeLeftMm = 1.5;
            command.weave.amplitudeRightMm = 1.5;
        }
        command.trigger = RMC::defaultMotionTrigger();
        commands.push_back(command);
    }
    return commands;
}

struct Recorded {
    double absoluteTimeSec;
    double windowTimeSec;
    double q[RMC::kAxisCount];
    int32_t commandId;
    int32_t completedCommandId;
    double profileSpeed;
    double weavePhase;
};

struct Result {
    bool ok = false;
    std::string failure;
    std::vector<Recorded> samples;
    double timeOffsetSec = 0.0;
    double worstSeamSpeedMmPerSec = 0.0;
    int windowCount = 0;
};

// One shared ring fill, so a difference between the two runs cannot come from here.
void fillRing(RMC::MotionCommandRing* ring,
              const std::vector<RMC::MotionCommand>& commands,
              size_t* next) {
    while (*next < commands.size() && RMC::motionCommandRingFree(*ring) > 0) {
        const size_t index = (*next)++;
        RMC::motionCommandRingPush(ring, commands[index], index + 1 == commands.size());
    }
}

void record(Result* result, const RMC::PathSample& sample, double timeOffsetSec) {
    Recorded entry{};
    entry.absoluteTimeSec = timeOffsetSec + sample.timeSec;
    entry.windowTimeSec = sample.timeSec;
    for (uint8_t i = 0; i < RMC::kAxisCount; ++i) entry.q[i] = sample.q[i];
    entry.commandId = sample.commandId;
    entry.completedCommandId = sample.completedCommandId;
    entry.profileSpeed = sample.profileSpeed;
    entry.weavePhase = sample.weavePhase;
    result->samples.push_back(entry);
}

// Big enough that they are not going on a stack.
RMC::MotionProgram g_program{};
RMC::MotionSegmentProgram g_segmentProgram{};
RMC::MotionSegmentSampler g_sampler{};
RMC::MotionWindowRunner g_runner{};
RMC::MotionBaseSegmentProgram g_baseSegmentProgram{};

Result runOldLoop(const RMC::RobotModel& model,
                  const RMC::MotionProgramSettings& settings,
                  const double startQ[RMC::kAxisCount],
                  const std::vector<RMC::MotionCommand>& commands) {
    Result result;
    RMC::MotionCommandRing ring{};
    RMC::clearMotionCommandRing(&ring);
    size_t nextCommandToQueue = 0;
    double timeOffset = 0.0;
    double carriedPathSpeed = 0.0;
    bool carriesSpeed = false;
    double carriedWeavePhase = 0.0;
    double replanStartQ[RMC::kAxisCount];
    double currentQ[RMC::kAxisCount];
    for (uint8_t i = 0; i < RMC::kAxisCount; ++i) {
        replanStartQ[i] = startQ[i];
        currentQ[i] = startQ[i];
    }
    RMC::PathSample previousWindowSample{};

    fillRing(&ring, commands, &nextCommandToQueue);
    while (ring.count > 0) {
        const uint8_t executeCount = RMC::motionLookaheadExecutableCount(ring);
        if (executeCount == 0) { result.failure = "no_executable_prefix"; return result; }

        if (!RMC::buildMotionProgramFromRing(ring, executeCount, carriesSpeed, carriedPathSpeed,
                                             carriedWeavePhase, &g_program)) {
            result.failure = "bad_ring";
            return result;
        }
        RMC::RejectCode code = RMC::buildMotionSegmentProgram(model, replanStartQ, g_program,
                                                             settings, &g_baseSegmentProgram,
                                                             &g_segmentProgram);
        if (code != RMC::RejectCode::Ok) {
            result.failure = std::string("plan:") + RMC::rejectCodeName(code);
            return result;
        }
        // Counted where the driver can count it too: once a window has actually been planned.
        ++result.windowCount;
        RMC::beginMotionSegmentSampler(model, replanStartQ, &g_segmentProgram, &g_sampler);
        bool seamChecked = false;
        double nextTime = settings.controlPeriodSec;
        const double totalTime = g_segmentProgram.trajectory.totalDurationSec;
        const RMC::MotionCommand* lastExecuted =
            RMC::motionCommandRingAt(ring, static_cast<uint8_t>(executeCount - 1));
        const int32_t lastExecutedCommandId = lastExecuted ? lastExecuted->id : -1;

        bool executedPrefixComplete = false;
        double executedPrefixTime = 0.0;
        RMC::PathSample executedPrefixSample{};
        while (nextTime < totalTime - 1.0e-9) {
            RMC::PathSample sample{};
            code = RMC::sampleMotionSegmentProgram(&g_sampler, nextTime, &sample);
            if (code != RMC::RejectCode::Ok) {
                result.failure = std::string("sample:") + RMC::rejectCodeName(code);
                return result;
            }
            if (!seamChecked) {
                seamChecked = true;
                const RMC::RejectCode seam = RMC::motionCheckWindowSeam(
                    previousWindowSample, sample, settings.controlPeriodSec, settings,
                    &result.worstSeamSpeedMmPerSec);
                if (seam != RMC::RejectCode::Ok) {
                    result.failure = std::string("seam:") + RMC::rejectCodeName(seam);
                    return result;
                }
            }
            record(&result, sample, timeOffset);
            for (uint8_t i = 0; i < RMC::kAxisCount; ++i) currentQ[i] = sample.q[i];
            previousWindowSample = sample;
            if (sample.completedCommandId >= lastExecutedCommandId) {
                executedPrefixComplete = true;
                executedPrefixTime = sample.timeSec;
                executedPrefixSample = sample;
                break;
            }
            nextTime += settings.controlPeriodSec;
        }
        if (!executedPrefixComplete && totalTime > 1.0e-9) {
            RMC::PathSample sample{};
            code = RMC::sampleMotionSegmentProgram(&g_sampler, totalTime, &sample);
            if (code != RMC::RejectCode::Ok) {
                result.failure = std::string("sample:") + RMC::rejectCodeName(code);
                return result;
            }
            record(&result, sample, timeOffset);
            for (uint8_t i = 0; i < RMC::kAxisCount; ++i) currentQ[i] = sample.q[i];
            previousWindowSample = sample;
            executedPrefixComplete = sample.completedCommandId >= lastExecutedCommandId;
            executedPrefixTime = sample.timeSec;
            executedPrefixSample = sample;
        }
        if (!executedPrefixComplete) { result.failure = "prefix_incomplete"; return result; }

        timeOffset += executedPrefixTime;
        carriesSpeed = executedPrefixSample.profileSpeed > 1.0e-9 && executeCount < ring.count;
        carriedPathSpeed = carriesSpeed ? executedPrefixSample.profileSpeed : 0.0;
        carriedWeavePhase = executeCount < ring.count ? executedPrefixSample.weavePhase : 0.0;
        RMC::motionReplanSeedQExact(model, executedPrefixSample, currentQ, replanStartQ);
        RMC::motionCommandRingPopFront(&ring, executeCount);
        fillRing(&ring, commands, &nextCommandToQueue);
    }
    result.timeOffsetSec = timeOffset;
    result.ok = true;
    return result;
}

Result runDriver(const RMC::RobotModel& model,
                 const RMC::MotionProgramSettings& settings,
                 const double startQ[RMC::kAxisCount],
                 const std::vector<RMC::MotionCommand>& commands) {
    Result result;
    RMC::MotionCommandRing ring{};
    RMC::clearMotionCommandRing(&ring);
    size_t nextCommandToQueue = 0;
    double currentQ[RMC::kAxisCount];
    for (uint8_t i = 0; i < RMC::kAxisCount; ++i) currentQ[i] = startQ[i];

    RMC::beginMotionWindowRunner(model, settings, startQ, &ring, &g_program, &g_segmentProgram,
                                 &g_sampler, &g_baseSegmentProgram, &g_runner);
    fillRing(&ring, commands, &nextCommandToQueue);
    for (;;) {
        const RMC::RejectCode planned = RMC::planMotionWindow(&g_runner);
        if (planned != RMC::RejectCode::Ok) {
            result.failure = std::string("plan:") + RMC::rejectCodeName(planned);
            return result;
        }
        if (!g_runner.haveWindow) {
            if (ring.count > 0) { result.failure = "no_executable_prefix"; return result; }
            break;
        }
        ++result.windowCount;

        double windowTime = settings.controlPeriodSec;
        bool windowComplete = false;
        RMC::PathSample prefixEndSample{};
        while (!windowComplete) {
            RMC::PathSample sample{};
            const RMC::RejectCode code =
                RMC::sampleMotionWindow(&g_runner, windowTime, &sample, &windowComplete);
            if (code != RMC::RejectCode::Ok) {
                result.failure = std::string("sample:") + RMC::rejectCodeName(code);
                return result;
            }
            if (sample.valid) {
                record(&result, sample, g_runner.timeOffsetSec);
                for (uint8_t i = 0; i < RMC::kAxisCount; ++i) currentQ[i] = sample.q[i];
                prefixEndSample = sample;
            }
            windowTime += settings.controlPeriodSec;
        }
        if (!g_runner.prefixComplete) { result.failure = "prefix_incomplete"; return result; }
        RMC::retireMotionWindow(&g_runner, prefixEndSample, currentQ);
        fillRing(&ring, commands, &nextCommandToQueue);
    }
    result.timeOffsetSec = g_runner.timeOffsetSec;
    result.worstSeamSpeedMmPerSec = g_runner.worstSeamSpeedMmPerSec;
    result.ok = true;
    return result;
}

bool compare(const char* label, const Result& old, const Result& driver) {
    bool ok = true;
    if (old.ok != driver.ok || old.failure != driver.failure) {
        std::printf("  FAIL %s: old %s / driver %s\n", label,
                    old.ok ? "ok" : old.failure.c_str(),
                    driver.ok ? "ok" : driver.failure.c_str());
        return false;
    }
    if (old.windowCount != driver.windowCount) {
        std::printf("  FAIL %s: %d windows vs %d\n", label, old.windowCount, driver.windowCount);
        ok = false;
    }
    if (old.samples.size() != driver.samples.size()) {
        std::printf("  FAIL %s: %zu samples vs %zu\n", label, old.samples.size(),
                    driver.samples.size());
        return false;
    }
    for (size_t i = 0; i < old.samples.size(); ++i) {
        const Recorded& a = old.samples[i];
        const Recorded& b = driver.samples[i];
        // Bit-exact, not approximate: the two are meant to be the same arithmetic in the same order.
        bool same = a.absoluteTimeSec == b.absoluteTimeSec && a.windowTimeSec == b.windowTimeSec &&
                    a.commandId == b.commandId && a.completedCommandId == b.completedCommandId &&
                    a.profileSpeed == b.profileSpeed && a.weavePhase == b.weavePhase;
        for (uint8_t joint = 0; joint < RMC::kAxisCount; ++joint) {
            if (a.q[joint] != b.q[joint]) same = false;
        }
        if (!same) {
            std::printf("  FAIL %s: sample %zu differs (t %.17g vs %.17g, cmd %ld vs %ld)\n",
                        label, i, a.absoluteTimeSec, b.absoluteTimeSec,
                        static_cast<long>(a.commandId), static_cast<long>(b.commandId));
            ok = false;
            break;
        }
    }
    if (old.timeOffsetSec != driver.timeOffsetSec) {
        std::printf("  FAIL %s: total time %.17g vs %.17g\n", label, old.timeOffsetSec,
                    driver.timeOffsetSec);
        ok = false;
    }
    if (old.worstSeamSpeedMmPerSec != driver.worstSeamSpeedMmPerSec) {
        std::printf("  FAIL %s: worst seam %.17g vs %.17g\n", label, old.worstSeamSpeedMmPerSec,
                    driver.worstSeamSpeedMmPerSec);
        ok = false;
    }
    if (ok) {
        std::printf("  agree %-22s %3d windows, %6zu samples, %8.4f s, seam %7.3f mm/s -> %s\n",
                    label, old.windowCount, old.samples.size(), old.timeOffsetSec,
                    old.worstSeamSpeedMmPerSec, old.ok ? "planned" : old.failure.c_str());
    }
    return ok;
}

}  // namespace

int main() {
    std::printf("sizeof MotionWindowRunner   %6zu\n", sizeof(RMC::MotionWindowRunner));
    std::printf("sizeof MotionSegmentProgram %6zu\n", sizeof(RMC::MotionSegmentProgram));
    std::printf("sizeof MotionProgram        %6zu\n", sizeof(RMC::MotionProgram));
    std::printf("sizeof MotionSegmentSampler %6zu\n", sizeof(RMC::MotionSegmentSampler));
    std::printf("sizeof MotionCommandRing    %6zu\n\n", sizeof(RMC::MotionCommandRing));

    const RMC::RobotModel model = makeAr4Model();
    const RMC::MotionProgramSettings settings = makeSettings();
    const double startQ[RMC::kAxisCount] = {-1.0, 0.3, -0.2, 0.0, 0.9, 0.0};

    const Config configs[] = {
        {"movel fine",            0.0, false, false, 40, 1.0},
        {"movel blended",         3.0, false, false, 40, 1.0},
        {"movel weave",           0.0, true,  false, 40, 1.0},
        {"movel weave blended",   3.0, true,  false, 40, 1.0},
        {"movej",                 0.0, false, true,  40, 1.0},
        {"movel single window",   2.0, false, false, 5,  1.0},
        {"movel one command",     0.0, false, false, 1,  1.0},
        {"movel exactly 24",      2.0, false, false, 24, 1.0},
        {"movel exactly 25",      2.0, false, false, 25, 1.0},
        {"movel tiny moves",      0.0, false, false, 25, 1.0e-2},
        {"movel sub-tick moves",  0.0, false, false, 25, 1.0e-5},
        {"movej sub-tick moves",  0.0, false, true,  25, 1.0e-5},
    };

    bool allOk = true;
    for (const Config& config : configs) {
        const std::vector<RMC::MotionCommand> commands = makeCommands(model, startQ, config);
        const Result oldResult = runOldLoop(model, settings, startQ, commands);
        const Result driverResult = runDriver(model, settings, startQ, commands);
        if (!compare(config.name, oldResult, driverResult)) allOk = false;
    }

    std::printf("\n%s\n", allOk ? "motion core window runner smoke passed"
                                : "motion core window runner smoke FAILED");
    return allOk ? 0 : 1;
}
