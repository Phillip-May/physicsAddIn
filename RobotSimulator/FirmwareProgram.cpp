#include "FirmwareProgram.h"

#include <algorithm>
#include <cmath>
#include <string>

#include "AppState.h"
#include "MasteringIo.h"
#include "ProgramTextIo.h"
#include "UnitsMath.h"

bool buildHardwareProgramCommands(const RobotInstance& instance,
                                  const std::array<double, 6>& startDeg,
                                  const RobotProgramSimulator::ProgramState& runState,
                                  std::vector<Json>* commands,
                                  std::map<int, std::string>* triggerMessages,
                                  std::string* errorMessage) {
    const auto reject = [&](const std::string& reason) {
        if (errorMessage) *errorMessage = reason;
        return false;
    };
    if (!commands) return reject("Run program rejected: internal command list missing.");
    commands->clear();
    if (triggerMessages) triggerMessages->clear();
    TriggerData pendingTrigger;
    bool havePendingTrigger = false;
    int nextTriggerId = 1;
    if (!instance.poseController.isBound()) return reject("Run program rejected: no robot package loaded.");
    if (!instance.program().root || instance.program().root->children.empty()) {
        return reject("Run program rejected: program is empty.");
    }
    const OPW6RobotData* robot = instance.poseController.robotData();
    if (!robot) return reject("Run program rejected: robot package is missing joint limits.");
    const RobotMotionCore::RobotModel model = currentRobotModelFor(instance);
    if (!RobotMotionCore::modelIsValid(model)) return reject("Run program rejected: invalid robot model.");

    const auto collisionMessage = [](const std::vector<RobotCollisionPair>& collisions) {
        std::string text;
        for (const RobotCollisionPair& pair : collisions) {
            if (!text.empty()) text += ", ";
            text += strutil::format("%1-%2").arg(pair.linkA).arg(pair.linkB).str();
        }
        return text;
    };
    const auto appendTargetDegrees = [](Json* command, const std::array<double, 6>& targetRad) {
        Json targetDeg = Json::array();
        for (double value : targetRad) targetDeg.push_back(value * kRadToDeg);
        (*command)["target_deg"] = targetDeg;
    };

    std::array<double, 6> currentDeg = startDeg;
    std::array<double, 6> currentRad{};
    for (size_t i = 0; i < 6; ++i) currentRad[i] = currentDeg[i] * kDegToRad;
    RobotProgramSimulator::ProgramState state = runState;
    bool programStopped = false;

    int instructionIndex = 0;
    for (const auto& child : instance.program().root->children) {
        if (!child) continue;
        ++instructionIndex;
        switch (child->type) {
        case RobotProgramNodeType::MoveJ: {
            const MoveJData& move = std::get<MoveJData>(child->data);
            if (move.hasExternalAxis) {
                return reject(strutil::format(
                    "Run program rejected at %1 MoveJ: hardware J7 transport is not configured.")
                    .arg(instructionIndex).str());
            }
            std::array<double, 6> targetDeg{};
            for (size_t i = 0; i < 6; ++i) targetDeg[i] = move.targetJoints[i] * kRadToDeg;

            RobotMotionCore::MoveJInput input = {};
            for (int i = 0; i < 6; ++i) {
                const size_t index = static_cast<size_t>(i);
                if (!std::isfinite(currentDeg[index]) || !std::isfinite(targetDeg[index])) {
                    return reject(strutil::format("Run program rejected at %1 MoveJ: J%2 is not finite.")
                                      .arg(instructionIndex)
                                      .arg(i + 1)
                                      .str());
                }
                input.targetDeg[i] = targetDeg[index];
                // The planner works in steps, but this check only cares about degrees, so a
                // scale of 1000 steps per degree stands in for the real mastering numbers and
                // the joints are declared mastered. The firmware uses its own calibration.
                input.currentSteps[i] = static_cast<int32_t>(std::llround(currentDeg[index] * 1000.0));
                input.zeroSteps[i] = 0;
                input.stepsPerDegree[i] = 1000.0;
                input.mastered[i] = 1;
                input.limitActive[i] = 0;
                input.limitDirection[i] = 0;
                input.limits.min[i] = robot->qMin[index] * kRadToDeg;
                input.limits.max[i] = robot->qMax[index] * kRadToDeg;
                if (input.limits.min[i] >= input.limits.max[i]) {
                    input.limits.min[i] = -180.0;
                    input.limits.max[i] = 180.0;
                }
            }
            RobotMotionCore::applyDefaultMoveJPlannerSettings(&input);
            input.speedDegPerSec = state.jointSpeedRadPerSec * kRadToDeg;
            input.estopActive = 0;

            RobotMotionCore::MoveJPlan plan = {};
            const RobotMotionCore::RejectCode planResult = RobotMotionCore::planMoveJ(input, &plan);
            if (planResult != RobotMotionCore::RejectCode::Ok) {
                return reject(strutil::format("Run program rejected at %1 MoveJ: %2")
                                  .arg(instructionIndex)
                                  .arg(RobotMotionCore::rejectCodeName(planResult))
                                  .str());
            }

            // One collision sample per degree of the largest joint move, so the check does not
            // step over a thin obstacle on a long move.
            const int steps = std::max(1, static_cast<int>(std::ceil(plan.maxDeltaDeg)));
            for (int step = 0; step <= steps; ++step) {
                const double t = static_cast<double>(step) / static_cast<double>(steps);
                double sampleDeg[RobotMotionCore::kAxisCount] = {};
                RobotMotionCore::sampleMoveJDegrees(currentDeg.data(), targetDeg.data(), t, sampleDeg);
                std::array<double, 6> sampleRad{};
                for (size_t joint = 0; joint < 6; ++joint) sampleRad[joint] = sampleDeg[joint] * kDegToRad;
                const std::vector<RobotCollisionPair> collisions = instance.collisionModel.selfCollisions(sampleRad);
                if (!collisions.empty()) {
                    return reject(strutil::format("Run program rejected at %1 MoveJ collision %2%: %3")
                                      .arg(instructionIndex)
                                      .arg(t * 100.0, 0, 'f', 1)
                                      .arg(collisionMessage(collisions))
                                      .str());
                }
            }

            Json command = Json::object();
            command["cmd"] = "movej";
            command["program_row"] = instructionIndex - 1;
            appendTargetDegrees(&command, move.targetJoints);
            command["speed_deg_s"] = state.jointSpeedRadPerSec * kRadToDeg;
            command["blend_mm"] = state.blendRadiusMm;
            commands->push_back(command);
            currentRad = move.targetJoints;
            currentDeg = targetDeg;
            break;
        }
        case RobotProgramNodeType::MoveL: {
            const MoveLData& move = std::get<MoveLData>(child->data);
            if (move.hasExternalAxis) {
                return reject(strutil::format(
                    "Run program rejected at %1 MoveL: hardware J7 transport is not configured.")
                    .arg(instructionIndex).str());
            }
            double targetQ[RobotMotionCore::kAxisCount] = {};
            for (size_t i = 0; i < 6; ++i) targetQ[i] = move.targetJoints[i];
            const RobotMotionCore::Transform targetTcp = RobotMotionCore::toolPoseForJoints(model, targetQ);

            RobotMotionCore::MoveLInput input = {};
            input.model = model;
            for (size_t i = 0; i < 6; ++i) input.startQ[i] = currentRad[i];
            input.targetTcp = targetTcp;
            for (size_t i = 0; i < 6; ++i) input.targetConfigQ[i] = move.targetJoints[i];
            input.speedMmPerSec = state.linearSpeedMmPerSec;
            input.sampleMm = timeBasedMoveLSampleMm(state.linearSpeedMmPerSec);
            input.singularityThresholdRad = effectiveSingularityThresholdRadFor(instance);
            input.singularityPolicy = RobotMotionCore::SingularityPolicy::SlowDown;
            // The IK has multiple solutions; requiring the target configuration keeps the arm
            // from arriving in a mirrored one, which would look correct at the tool and be
            // wrong everywhere else.
            input.requireTargetConfig = 1;
            input.targetConfigToleranceRad = 0.5 * kDegToRad;

            RobotMotionCore::MoveLStream stream = {};
            RobotMotionCore::RejectCode result = RobotMotionCore::beginMoveLStream(input, &stream);
            if (result != RobotMotionCore::RejectCode::Ok) {
                return reject(strutil::format("Run program rejected at %1 MoveL: %2")
                                  .arg(instructionIndex)
                                  .arg(RobotMotionCore::rejectCodeName(result))
                                  .str());
            }

            while (stream.nextSample <= stream.plan.sampleCount) {
                double q[RobotMotionCore::kAxisCount] = {};
                uint32_t sampleIndex = 0;
                result = RobotMotionCore::nextMoveLSample(&stream, q, nullptr, &sampleIndex);
                if (result != RobotMotionCore::RejectCode::Ok) {
                    return reject(strutil::format("Run program rejected at %1 MoveL sample %2: %3")
                                      .arg(instructionIndex)
                                      .arg(static_cast<unsigned>(sampleIndex))
                                      .arg(RobotMotionCore::rejectCodeName(result))
                                      .str());
                }
                std::array<double, 6> sampleRad{};
                for (size_t joint = 0; joint < 6; ++joint) sampleRad[joint] = q[joint];
                const std::vector<RobotCollisionPair> collisions = instance.collisionModel.selfCollisions(sampleRad);
                if (!collisions.empty()) {
                    return reject(strutil::format("Run program rejected at %1 MoveL collision sample %2: %3")
                                      .arg(instructionIndex)
                                      .arg(static_cast<unsigned>(sampleIndex))
                                      .arg(collisionMessage(collisions))
                                      .str());
                }
            }

            Json command = Json::object();
            command["cmd"] = "movel";
            command["program_row"] = instructionIndex - 1;
            command["target_tcp"] = jsonArrayFromTransform(targetTcp);
            appendTargetDegrees(&command, move.targetJoints);
            command["speed_mm_s"] = state.linearSpeedMmPerSec;
            command["blend_mm"] = state.blendRadiusMm;
            if (state.weaveEnabled) {
                // The index travels, not the parameters, so the robot resolves it against the same
                // schedule table the Welding tab pushed. Only an inline weave sends its numbers.
                command["weave_index"] = state.weaveScheduleIndex;
                if (state.weaveScheduleIndex < 0) command["weave"] = jsonFromWeaveParams(state.weaveInline);
            }
            if (havePendingTrigger) {
                const int triggerId = nextTriggerId++;
                command["trigger_id"] = triggerId;
                if (pendingTrigger.referenceStart) command["trigger_ref_start"] = 1;
                command["trigger_dist_mm"] = pendingTrigger.distanceMm;
                command["trigger_time_ms"] = pendingTrigger.timeMs;
                if (triggerMessages) (*triggerMessages)[triggerId] = pendingTrigger.message;
                havePendingTrigger = false;
            }
            commands->push_back(command);
            // Continue from where the stream actually ended, not the requested target: the IK
            // endpoint can differ, and carrying the request forward would accumulate error.
            for (size_t i = 0; i < 6; ++i) {
                currentRad[i] = stream.plan.endpointQ[i];
                currentDeg[i] = stream.plan.endpointQ[i] * kRadToDeg;
            }
            break;
        }
        case RobotProgramNodeType::SetSpeed:
        case RobotProgramNodeType::SetBlending:
        case RobotProgramNodeType::SetWeave:
        case RobotProgramNodeType::WeaveOn:
        case RobotProgramNodeType::WeaveOff:
            applyStateInstruction(state, *child);
            break;
        case RobotProgramNodeType::SetTool:
            return reject(strutil::format(
                "Run program rejected at %1 SetTool: hardware tool selection is not configured.")
                .arg(instructionIndex).str());
        case RobotProgramNodeType::Actuate:
            return reject(strutil::format(
                "Run program rejected at %1 Actuate: hardware mechanism IO is not configured.")
                .arg(instructionIndex).str());
        case RobotProgramNodeType::Stop:
            programStopped = true;
            break;
        case RobotProgramNodeType::Trigger:
            // Held until the next move claims it. One-shot, so it is not part of the modal state.
            pendingTrigger = std::get<TriggerData>(child->data);
            havePendingTrigger = true;
            break;
        case RobotProgramNodeType::MoveC:
            return reject(strutil::format("Run program rejected at %1 MoveC: firmware does not support MoveC.")
                              .arg(instructionIndex)
                              .str());
        case RobotProgramNodeType::Root:
            break;
        }
        if (programStopped) break;
    }

    // Blending only means anything between two linear moves; anywhere else the firmware would
    // round a corner the program did not ask for, so it is zeroed.
    for (size_t i = 0; i < commands->size(); ++i) {
        Json& command = (*commands)[i];
        const bool currentMoveL = jsoncompat::fieldString(command, "cmd") == "movel";
        const bool nextMoveL =
            i + 1 < commands->size() && jsoncompat::fieldString((*commands)[i + 1], "cmd") == "movel";
        if (!currentMoveL || !nextMoveL) command["blend_mm"] = 0.0;
    }
    if (errorMessage) errorMessage->clear();
    return true;
}
