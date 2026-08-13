#include "RobotProgramModel.h"
#include "StringUtil.h"
#include <string>
#include <vector>


#include <algorithm>
#include <cmath>
#include <utility>

namespace {

using strutil::operator<<;

constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

std::string jointsSummary(const std::array<double, 6>& joints) {
    std::vector<std::string> parts;
    for (int i = 0; i < 6; ++i) {
        parts << strutil::format("J%1 %2").arg(i + 1).arg(joints[static_cast<size_t>(i)] * kRadToDeg, 0, 'f', 1);
    }
    return strutil::join(parts, "  ");
}

std::string moveSummary(const std::array<double, 6>& joints,
                        bool hasExternalAxis,
                        double externalAxisPositionMm) {
    std::string summary = jointsSummary(joints);
    if (hasExternalAxis) {
        summary += strutil::format("  J7 %1 mm").arg(externalAxisPositionMm, 0, 'f', 1).str();
    }
    return summary;
}

std::string nodeTypeLabel(RobotProgramNodeType type) {
    switch (type) {
    case RobotProgramNodeType::Root:
        return "Program";
    case RobotProgramNodeType::MoveJ:
        return "MoveJ";
    case RobotProgramNodeType::MoveL:
        return "MoveL";
    case RobotProgramNodeType::MoveC:
        return "MoveC";
    case RobotProgramNodeType::SetSpeed:
        return "SetSpeed";
    case RobotProgramNodeType::SetBlending:
        return "SetBlending";
    case RobotProgramNodeType::SetWeave:
        return "SetWeave";
    case RobotProgramNodeType::WeaveOn:
        return "WeaveOn";
    case RobotProgramNodeType::WeaveOff:
        return "WeaveOff";
    case RobotProgramNodeType::SetTool:
        return "SetTool";
    case RobotProgramNodeType::Actuate:
        return "Actuate";
    case RobotProgramNodeType::Stop:
        return "Stop";
    case RobotProgramNodeType::Trigger:
        return "Trigger";
    }
    return "Instruction";
}

std::string nodeDetail(const RobotProgramNode& node) {
    switch (node.type) {
    case RobotProgramNodeType::Root:
        return strutil::format("%1 instruction(s)").arg(node.children.size());
    case RobotProgramNodeType::MoveJ: {
        const MoveJData& move = std::get<MoveJData>(node.data);
        return moveSummary(move.targetJoints, move.hasExternalAxis, move.externalAxisPositionMm);
    }
    case RobotProgramNodeType::MoveL: {
        const MoveLData& move = std::get<MoveLData>(node.data);
        return moveSummary(move.targetJoints, move.hasExternalAxis, move.externalAxisPositionMm);
    }
    case RobotProgramNodeType::MoveC: {
        const MoveCData& move = std::get<MoveCData>(node.data);
        return "Via: " + jointsSummary(move.viaJoints) + "  Target: " + jointsSummary(move.targetJoints);
    }
    case RobotProgramNodeType::SetSpeed: {
        const SetSpeedData& speed = std::get<SetSpeedData>(node.data);
        return strutil::format("Joint %1 deg/s, Linear %2 mm/s")
            .arg(speed.jointSpeedRadPerSec * kRadToDeg, 0, 'f', 1)
            .arg(speed.linearSpeedMmPerSec, 0, 'f', 1);
    }
    case RobotProgramNodeType::SetBlending:
        return strutil::format("%1 mm").arg(std::get<SetBlendingData>(node.data).radiusMm, 0, 'f', 1);
    case RobotProgramNodeType::SetWeave: {
        const RobotMotionCore::WeaveParams& weave = std::get<SetWeaveData>(node.data).params;
        const std::string rate = weave.rateMode == RobotMotionCore::WeaveRateMode::Frequency
            ? strutil::format("%1 Hz").arg(weave.frequencyHz, 0, 'f', 2).str()
            : (weave.rateMode == RobotMotionCore::WeaveRateMode::Wavelength
                   ? strutil::format("%1 mm/cycle").arg(weave.wavelengthMm, 0, 'f', 1).str()
                   : std::string("no rate mode"));
        return strutil::format("%1, %2, +%3/-%4 mm")
            .arg(std::string(RobotMotionCore::weaveShapeName(weave.shape)))
            .arg(rate)
            .arg(weave.amplitudeLeftMm, 0, 'f', 2)
            .arg(weave.amplitudeRightMm, 0, 'f', 2);
    }
    case RobotProgramNodeType::WeaveOn: {
        const int index = std::get<WeaveOnData>(node.data).scheduleIndex;
        return index >= 0 ? strutil::format("schedule %1").arg(index).str()
                          : std::string("inline parameters");
    }
    case RobotProgramNodeType::WeaveOff:
        return {};
    case RobotProgramNodeType::SetTool: {
        const SetToolData& tool = std::get<SetToolData>(node.data);
        return tool.toolId + " / TCP " + std::to_string(tool.tcpIndex + 1);
    }
    case RobotProgramNodeType::Actuate: {
        const ActuateData& actuator = std::get<ActuateData>(node.data);
        return actuator.mechanismId + " / " + actuator.actuatorId + " = " +
            strutil::formatShortest(actuator.position);
    }
    case RobotProgramNodeType::Stop:
        return "End the current run";
    case RobotProgramNodeType::Trigger: {
        const TriggerData& trigger = std::get<TriggerData>(node.data);
        std::string detail = trigger.referenceStart ? "from start" : "from end";
        if (trigger.distanceMm != 0.0) {
            detail += strutil::format(" %1%2 mm")
                          .arg(trigger.distanceMm > 0.0 ? "+" : "")
                          .arg(trigger.distanceMm, 0, 'f', 1)
                          .str();
        }
        if (trigger.timeMs != 0.0) {
            detail += strutil::format(" %1%2 ms")
                          .arg(trigger.timeMs > 0.0 ? "+" : "")
                          .arg(trigger.timeMs, 0, 'f', 0)
                          .str();
        }
        return detail + ": " + trigger.message;
    }
    }
    return {};
}

} // namespace

std::string robotProgramNodeLabel(RobotProgramNodeType type) {
    return nodeTypeLabel(type);
}

std::string robotProgramNodeDetail(const RobotProgramNode& node) {
    return nodeDetail(node);
}

RobotProgram::RobotProgram()
    : root(std::make_unique<RobotProgramNode>()) {
    root->type = RobotProgramNodeType::Root;
}
