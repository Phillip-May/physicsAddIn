#pragma once
#include "RobotMotionCore.h"
#include "StringUtil.h"
#include <string>


#include <array>
#include <memory>
#include <variant>
#include <vector>

enum class RobotProgramNodeType {
    Root,
    MoveJ,
    MoveL,
    MoveC,
    SetSpeed,
    SetBlending,
    SetWeave,
    WeaveOn,
    WeaveOff,
    SetTool,
    Actuate,
    Stop,
    Trigger
};

struct MoveJData {
    std::array<double, 6> targetJoints{};
    bool hasExternalAxis = false;
    double externalAxisPositionMm = 0.0;
};

struct MoveLData {
    std::array<double, 6> targetJoints{};
    bool hasExternalAxis = false;
    double externalAxisPositionMm = 0.0;
};

struct MoveCData {
    std::array<double, 6> viaJoints{};
    std::array<double, 6> targetJoints{};
};

struct SetSpeedData {
    double jointSpeedRadPerSec = 0.5;
    double linearSpeedMmPerSec = 100.0;
};

struct SetBlendingData {
    double radiusMm = 0.0;
};

struct SetWeaveData {
    RobotMotionCore::WeaveParams params{};
};

// Turns weaving on. A schedule index selects one of the stored welding schedules; -1 means use
// whatever the last SetWeave put in place.
struct WeaveOnData {
    int scheduleIndex = -1;
};

// Generic mechanism command. `mechanismId` names a station asset and `actuatorId` addresses the
// shared scalar actuator within it, whether that asset is a tool, rail, or future mechanism.
struct ActuateData {
    std::string mechanismId;
    std::string actuatorId;
    double position = 0.0;
};

struct SetToolData {
    std::string toolId;
    int tcpIndex = 0;
};

// A path-related trigger, attached to the move on the following line. Not modal: it fires once, for
// that move only, unlike SetSpeed and SetBlending which stay in force.
struct TriggerData {
    double distanceMm = 0.0;   // signed, along the seam; negative is before
    double timeMs = 0.0;       // signed, applied after the distance offset
    bool referenceStart = false;
    // The only action for now. When there are others this becomes a discriminated field, and the
    // trigger plumbing does not change.
    std::string message;
};

using RobotProgramNodeData = std::variant<
    std::monostate,
    MoveJData,
    MoveLData,
    MoveCData,
    SetSpeedData,
    SetBlendingData,
    SetWeaveData,
    WeaveOnData,
    SetToolData,
    ActuateData,
    TriggerData>;

struct RobotProgramNode {
    RobotProgramNodeType type = RobotProgramNodeType::Root;
    RobotProgramNodeData data;
    RobotProgramNode* parent = nullptr;
    std::vector<std::unique_ptr<RobotProgramNode>> children;
};

// Row label and detail text for one instruction, shared so every view renders identical strings.
std::string robotProgramNodeLabel(RobotProgramNodeType type);
std::string robotProgramNodeDetail(const RobotProgramNode& node);

struct RobotProgram {
    std::string name = "Program";
    std::unique_ptr<RobotProgramNode> root;

    RobotProgram();
};
