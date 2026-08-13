#pragma once

#include <algorithm>
#include <array>
#include <istream>
#include <limits>
#include <string>
#include <vector>

#include "CadNode.h"
#include "JsonCompat.h"
#include "RobotMotionCore.h"
#include "RobotProgramModel.h"

#include "UnitsMath.h"

class RobotPoseController;

inline double timeBasedMoveLSampleMm(double speedMmPerSec) {
    return std::max(0.001, speedMmPerSec * kMotionControlPeriodSeconds);
}

struct ParsedProgramInstruction {
    RobotProgramNodeType type = RobotProgramNodeType::Root;
    RobotProgramNodeData data;
};

enum class TimelineMarkerType {
    Singularity,
    JointFlip,
    PlannerCap,
    Trigger
};

struct TimelineMarker {
    double seconds = 0.0;
    TimelineMarkerType type = TimelineMarkerType::Singularity;
    std::string message;
};

struct TimelineSample {
    double seconds = 0.0;
    std::array<double, 6> joints{};
    CadTransform tcpPose;
    double tcpSpeedMmPerSec = std::numeric_limits<double>::quiet_NaN();
    double desiredTcpSpeedMmPerSec = std::numeric_limits<double>::quiet_NaN();
    double profileTcpSpeedMmPerSec = std::numeric_limits<double>::quiet_NaN();
    const RobotProgramNode* activeNode = nullptr;
    bool externalAxisValid = false;
    double externalAxisPositionMm = 0.0;
};

namespace progtext {

struct TcpFramePoint {
    CadTransform pose;
    std::array<double, 6> joints{};
};

struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

inline Vec3 tcpPosition(const CadTransform& pose) {
    return {pose.values[3], pose.values[7], pose.values[11]};
}

inline Vec3 vecAdd(Vec3 a, Vec3 b) {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

inline Vec3 vecSub(Vec3 a, Vec3 b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

inline Vec3 vecScale(Vec3 a, double s) {
    return {a.x * s, a.y * s, a.z * s};
}

inline double vecDot(Vec3 a, Vec3 b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline Vec3 vecCross(Vec3 a, Vec3 b) {
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    };
}

inline double vecLength(Vec3 a) {
    return std::sqrt(vecDot(a, a));
}

inline Vec3 vecNormalize(Vec3 a) {
    const double length = vecLength(a);
    if (length <= 1.0e-9) return {1.0, 0.0, 0.0};
    return vecScale(a, 1.0 / length);
}

} // namespace progtext

Json jsonArrayFromTransform(const RobotMotionCore::Transform& transform);
Json jsonFromWeaveParams(const RobotMotionCore::WeaveParams& weave);
bool weaveParamsFromJson(const Json& object, RobotMotionCore::WeaveParams* weave);
std::string programNumber(double value);
bool parseDoubleToken(const std::string& token, double* value);
bool parseJointDegrees(const std::vector<std::string>& tokens, int firstToken, std::array<double, 6>* joints);
bool parseProgramTextLineHeadless(const std::string& line, int lineNumber, ParsedProgramInstruction* instruction, std::string* errorMessage);
void appendInstructionToRoot(RobotProgramNode* root, RobotProgramNodeType type, RobotProgramNodeData data);
bool loadProgramTextFromStream(std::istream& file, RobotProgramNode* root, std::string* errorMessage);
bool loadProgramTextHeadless(const std::string& fileName, RobotProgramNode* root, std::string* errorMessage);
CadTransform withLocalRotation(const CadTransform& pose, int axis, double angleRadians);
CadTransform composeLocalWpr(const CadTransform& basePose, const std::array<double, 3>& wprDegrees);
double tcpDistance(const CadTransform& a, const CadTransform& b);
std::string motionTypeName(RobotProgramNodeType type);
bool collectProgramTcpPoints(const RobotProgramNode& root,
                             RobotPoseController* poseController,
                             std::vector<progtext::TcpFramePoint>* points);
std::string formatMoveLine(const char* command, const std::array<double, 6>& jointsRad);
std::string formatMoveLine(const char* command,
                           const std::array<double, 6>& jointsRad,
                           bool hasExternalAxis,
                           double externalAxisPositionMm);
