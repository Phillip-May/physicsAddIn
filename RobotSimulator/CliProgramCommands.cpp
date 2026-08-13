#include "CliProgramCommands.h"
#include "AppState.h"
#include "StationSceneLoad.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <sstream>
#include <thread>
#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif

#include "CadNodePackage.h"
#include "LiveRunDriver.h"
#include "ProgramTextIo.h"
#include "RobotRuntime.h"
#include "StationPackage.h"
#include "StringUtil.h"

using namespace progtext;

int generateRectProgramCommand(const std::vector<std::string>& args, bool triangularWeave) {
    if (args.size() < 5) {
        if (triangularWeave) {
            std::cerr << "Usage: RobotSimulator.exe --generate-triangle-weave-program <robot.zip|json> "
                         "<reference.robotprog.txt> <output.robotprog.txt> [--points value] [--margin value] "
                         "[--passes value] [--weaves value] [--amplitude value] "
                         "[--wavelength-mm value] [--amplitude-mm value]"
                      << std::endl;
        } else {
            std::cerr << "Usage: RobotSimulator.exe --generate-rect-program <robot.zip|json> "
                         "<reference.robotprog.txt> <output.robotprog.txt> [--points value] [--margin value] [--passes value]"
                      << std::endl;
        }
        return 2;
    }

    const std::string packageFile = args[2];
    const std::string referenceFile = args[3];
    const std::string outputFile = args[4];
    bool pointCountProvided = false;
    int pointCount = triangularWeave ? 0 : 100;
    double marginFraction = 0.12;
    int passCount = 6;
    int weaveCount = 12;
    double weaveAmplitude = 0.25;
    double weaveWavelengthMm = triangularWeave ? 5.0 : 0.0;
    double weaveAmplitudeMm = std::numeric_limits<double>::quiet_NaN();
    bool weaveCountProvided = false;
    bool weaveWavelengthProvided = false;
    for (int i = 5; i < args.size(); i += 2) {
        if (i + 1 >= args.size()) {
            std::cerr << "Missing value for option: " << args[i] << std::endl;
            return 2;
        }
        double value = 0.0;
        if (!strutil::parseDouble(args[i + 1], &value)) {
            std::cerr << "Invalid numeric value for " << args[i] << std::endl;
            return 2;
        }
        if (args[i] == "--points") {
            pointCount = static_cast<int>(std::round(value));
            pointCountProvided = true;
        } else if (args[i] == "--margin") {
            marginFraction = value;
        } else if (args[i] == "--passes") {
            passCount = static_cast<int>(std::round(value));
        } else if (triangularWeave && args[i] == "--weaves") {
            weaveCount = static_cast<int>(std::round(value));
            weaveCountProvided = true;
        } else if (triangularWeave && args[i] == "--amplitude") {
            weaveAmplitude = value;
        } else if (triangularWeave && args[i] == "--wavelength-mm") {
            weaveWavelengthMm = value;
            weaveWavelengthProvided = true;
        } else if (triangularWeave && args[i] == "--amplitude-mm") {
            weaveAmplitudeMm = value;
        } else {
            std::cerr << "Unknown option: " << args[i] << std::endl;
            return 2;
        }
    }
    if (triangularWeave && weaveWavelengthMm <= 0.0) {
        std::cerr << "--wavelength-mm must be positive." << std::endl;
        return 2;
    }
    if (triangularWeave && RobotMotionCore::isFinite(weaveAmplitudeMm) && weaveAmplitudeMm <= 0.0) {
        std::cerr << "--amplitude-mm must be positive." << std::endl;
        return 2;
    }
    if (pointCountProvided && pointCount < 8) {
        std::cerr << "--points must be at least 8." << std::endl;
        return 2;
    }
    if (marginFraction < 0.0 || marginFraction >= 0.45) {
        std::cerr << "--margin must be in [0, 0.45)." << std::endl;
        return 2;
    }
    if (!triangularWeave && (passCount < 2 || passCount > pointCount / 2)) {
        std::cerr << "--passes must be at least 2 and leave enough samples per segment." << std::endl;
        return 2;
    }
    int totalWeaveCycles = triangularWeave ? weaveCount * passCount : 0;
    if (triangularWeave && passCount < 1) {
        std::cerr << "--passes must be at least 1." << std::endl;
        return 2;
    }
    if (triangularWeave && weaveCount < 1) {
        std::cerr << "--weaves must be at least 1." << std::endl;
        return 2;
    }
    if (triangularWeave && (weaveAmplitude <= 0.0 || weaveAmplitude > 1.0)) {
        std::cerr << "--amplitude must be in (0, 1]." << std::endl;
        return 2;
    }

    std::string errorMessage;
    std::shared_ptr<CadNode> loadedNode = loadCadNodePackage(packageFile, &errorMessage);
    if (!loadedNode || !validateRobotPackage(loadedNode.get(), &errorMessage)) {
        std::cerr << "Package validation failed: " << errorMessage << std::endl;
        return 1;
    }

    RobotPoseController poseController;
    if (!poseController.bind(loadedNode.get(), &errorMessage)) {
        std::cerr << "Package pose bind failed: " << errorMessage << std::endl;
        return 1;
    }

    RobotProgramNode root;
    root.type = RobotProgramNodeType::Root;
    if (!loadProgramTextHeadless(referenceFile, &root, &errorMessage)) {
        std::cerr << "Reference program load failed: " << errorMessage << std::endl;
        return 1;
    }

    std::vector<TcpFramePoint> referencePoints;
    if (!collectProgramTcpPoints(root, &poseController, &referencePoints)) {
        std::cerr << "Reference program needs at least three MoveL targets." << std::endl;
        return 1;
    }

    std::vector<double> referenceSegmentLengths;
    referenceSegmentLengths.reserve(referencePoints.size() - 1);
    double referencePathLength = 0.0;
    for (size_t i = 1; i < referencePoints.size(); ++i) {
        const double length = vecLength(vecSub(tcpPosition(referencePoints[i].pose),
                                               tcpPosition(referencePoints[i - 1].pose)));
        referenceSegmentLengths.push_back(length);
        referencePathLength += length;
    }
    if (triangularWeave) {
        if (referencePathLength <= 1.0e-6) {
            std::cerr << "Reference TCP path is too small for a triangular weave." << std::endl;
            return 1;
        }
        if (!weaveWavelengthProvided && weaveCountProvided) {
            weaveWavelengthMm = referencePathLength / static_cast<double>(std::max(1, weaveCount * passCount));
        }
        totalWeaveCycles = std::max(1, static_cast<int>(std::ceil(referencePathLength / weaveWavelengthMm)));
        if (!pointCountProvided) {
            pointCount = totalWeaveCycles * 6;
        }
        if (pointCount < totalWeaveCycles * 5) {
            std::cerr << "--points must provide at least five samples per triangular weave cycle." << std::endl;
            return 2;
        }
    }

    Vec3 center{};
    for (const TcpFramePoint& point : referencePoints) center = vecAdd(center, tcpPosition(point.pose));
    center = vecScale(center, 1.0 / static_cast<double>(referencePoints.size()));

    size_t farA = 0;
    size_t farB = 1;
    double farDistance = -1.0;
    for (size_t a = 0; a < referencePoints.size(); ++a) {
        for (size_t b = a + 1; b < referencePoints.size(); ++b) {
            const double distance = vecLength(vecSub(tcpPosition(referencePoints[a].pose), tcpPosition(referencePoints[b].pose)));
            if (distance > farDistance) {
                farDistance = distance;
                farA = a;
                farB = b;
            }
        }
    }
    const Vec3 u = vecNormalize(vecSub(tcpPosition(referencePoints[farB].pose), tcpPosition(referencePoints[farA].pose)));

    Vec3 v{0.0, 1.0, 0.0};
    double bestPerp = -1.0;
    for (const TcpFramePoint& point : referencePoints) {
        const Vec3 delta = vecSub(tcpPosition(point.pose), center);
        const Vec3 perp = vecSub(delta, vecScale(u, vecDot(delta, u)));
        const double length = vecLength(perp);
        if (length > bestPerp) {
            bestPerp = length;
            v = vecNormalize(perp);
        }
    }
    if (bestPerp <= 1.0e-6) {
        const Vec3 fallback = std::abs(u.x) < 0.8 ? Vec3{1.0, 0.0, 0.0} : Vec3{0.0, 1.0, 0.0};
        v = vecNormalize(vecSub(fallback, vecScale(u, vecDot(fallback, u))));
    }

    double minU = std::numeric_limits<double>::max();
    double maxU = std::numeric_limits<double>::lowest();
    double minV = std::numeric_limits<double>::max();
    double maxV = std::numeric_limits<double>::lowest();
    for (const TcpFramePoint& point : referencePoints) {
        const Vec3 delta = vecSub(tcpPosition(point.pose), center);
        const double pu = vecDot(delta, u);
        const double pv = vecDot(delta, v);
        minU = std::min(minU, pu);
        maxU = std::max(maxU, pu);
        minV = std::min(minV, pv);
        maxV = std::max(maxV, pv);
    }
    const double insetU = (maxU - minU) * marginFraction;
    const double insetV = (maxV - minV) * marginFraction;
    minU += insetU;
    maxU -= insetU;
    minV += insetV;
    maxV -= insetV;
    if (maxU - minU < 1.0 || maxV - minV < 1.0) {
        std::cerr << "Reference TCP bounds are too small for a rectangular pattern." << std::endl;
        return 1;
    }

    CadTransform targetPose = referencePoints.front().pose;
    poseController.setJoints(referencePoints.front().joints);
    std::vector<Vec3> generatedPositions;
    generatedPositions.reserve(static_cast<size_t>(pointCount + 1));
    if (!triangularWeave) {
        std::vector<Vec3> waypoints;
        waypoints.reserve(static_cast<size_t>(passCount * 2));
        for (int pass = 0; pass < passCount; ++pass) {
            const double t = passCount == 1 ? 0.0 : static_cast<double>(pass) / static_cast<double>(passCount - 1);
            const double pv = minV + (maxV - minV) * t;
            const bool leftToRight = (pass % 2) == 0;
            const double startU = leftToRight ? minU : maxU;
            const double endU = leftToRight ? maxU : minU;
            waypoints.push_back(vecAdd(center, vecAdd(vecScale(u, startU), vecScale(v, pv))));
            waypoints.push_back(vecAdd(center, vecAdd(vecScale(u, endU), vecScale(v, pv))));
        }

        std::vector<double> segmentLengths;
        segmentLengths.reserve(waypoints.size() - 1);
        double totalLength = 0.0;
        for (size_t i = 1; i < waypoints.size(); ++i) {
            const double length = vecLength(vecSub(waypoints[i], waypoints[i - 1]));
            segmentLengths.push_back(length);
            totalLength += length;
        }
        if (totalLength <= 1.0e-6 || segmentLengths.empty()) {
            std::cerr << "Generated TCP pattern is too small." << std::endl;
            return 1;
        }

        generatedPositions.push_back(waypoints.front());
        int remainingSamples = pointCount;
        for (size_t segment = 0; segment < segmentLengths.size(); ++segment) {
            const int remainingSegments = static_cast<int>(segmentLengths.size() - segment - 1);
            int samplesThisSegment = remainingSegments == 0
                ? remainingSamples
                : static_cast<int>(std::round((segmentLengths[segment] / totalLength) * static_cast<double>(pointCount)));
            samplesThisSegment = std::max(1, samplesThisSegment);
            samplesThisSegment = std::min(samplesThisSegment, remainingSamples - remainingSegments);
            if (samplesThisSegment <= 0) break;
            const Vec3 a = waypoints[segment];
            const Vec3 b = waypoints[segment + 1];
            for (int sample = 1; sample <= samplesThisSegment; ++sample) {
                const double t = static_cast<double>(sample) / static_cast<double>(samplesThisSegment);
                generatedPositions.push_back(vecAdd(a, vecScale(vecSub(b, a), t)));
            }
            remainingSamples -= samplesThisSegment;
        }
        while (static_cast<int>(generatedPositions.size()) < pointCount + 1) generatedPositions.push_back(waypoints.back());
        if (static_cast<int>(generatedPositions.size()) > pointCount + 1) generatedPositions.resize(static_cast<size_t>(pointCount + 1));
    } else {
        const double amplitudeMm = RobotMotionCore::isFinite(weaveAmplitudeMm)
            ? weaveAmplitudeMm
            : weaveWavelengthMm * weaveAmplitude;
        struct ReferenceFrame {
            Vec3 position;
            Vec3 tangent;
            Vec3 lateral;
        };
        const Vec3 planeNormal = vecNormalize(vecCross(u, v));
        auto referenceFrameAtDistance = [&](double distance) -> ReferenceFrame {
            distance = std::max(0.0, std::min(referencePathLength, distance));
            size_t referenceSegment = 0;
            double segmentStartDistance = 0.0;
            while (referenceSegment + 1 < referenceSegmentLengths.size() &&
                   distance > segmentStartDistance + referenceSegmentLengths[referenceSegment]) {
                segmentStartDistance += referenceSegmentLengths[referenceSegment];
                ++referenceSegment;
            }
            const size_t a = referenceSegment;
            const size_t b = std::min(referencePoints.size() - 1, a + 1);
            const double segmentLength = referenceSegment < referenceSegmentLengths.size()
                ? referenceSegmentLengths[referenceSegment]
                : 0.0;
            const double localT = segmentLength > 1.0e-9
                ? std::max(0.0, std::min(1.0, (distance - segmentStartDistance) / segmentLength))
                : 0.0;
            const Vec3 pa = tcpPosition(referencePoints[a].pose);
            const Vec3 pb = tcpPosition(referencePoints[b].pose);
            Vec3 tangent = vecNormalize(vecSub(pb, pa));
            if (segmentLength <= 1.0e-9) tangent = u;
            Vec3 lateral = vecNormalize(vecCross(planeNormal, tangent));
            if (vecDot(lateral, v) < 0.0) lateral = vecScale(lateral, -1.0);
            return {vecAdd(pa, vecScale(vecSub(pb, pa), localT)), tangent, lateral};
        };

        std::vector<Vec3> toothControlPoints;
        toothControlPoints.reserve(static_cast<size_t>(totalWeaveCycles * 5 + 1));
        toothControlPoints.push_back(referenceFrameAtDistance(0.0).position);
        for (int weave = 0; weave < totalWeaveCycles; ++weave) {
            const double baseDistance = std::min(referencePathLength, static_cast<double>(weave) * weaveWavelengthMm);
            const double nextDistance = std::min(referencePathLength, static_cast<double>(weave + 1) * weaveWavelengthMm);
            const ReferenceFrame side = referenceFrameAtDistance(baseDistance);
            const ReferenceFrame next = referenceFrameAtDistance(nextDistance);
            const Vec3 top = vecAdd(side.position, vecScale(side.lateral, amplitudeMm));
            const Vec3 bottom = vecAdd(side.position, vecScale(side.lateral, -amplitudeMm));
            const Vec3 apex = next.position;
            toothControlPoints.push_back(top);
            toothControlPoints.push_back(bottom);
            toothControlPoints.push_back(apex);
            toothControlPoints.push_back(top);
            toothControlPoints.push_back(apex);
        }

        std::vector<double> toothSegmentLengths;
        toothSegmentLengths.reserve(toothControlPoints.size() - 1);
        double toothLength = 0.0;
        for (size_t i = 1; i < toothControlPoints.size(); ++i) {
            const double length = vecLength(vecSub(toothControlPoints[i], toothControlPoints[i - 1]));
            toothSegmentLengths.push_back(length);
            toothLength += length;
        }
        if (toothLength <= 1.0e-6 || toothSegmentLengths.empty()) {
            std::cerr << "Generated triangular weld weave is too small." << std::endl;
            return 1;
        }

        generatedPositions.push_back(toothControlPoints.front());
        int remainingSamples = pointCount;
        for (size_t segment = 0; segment < toothSegmentLengths.size(); ++segment) {
            const int remainingSegments = static_cast<int>(toothSegmentLengths.size() - segment - 1);
            int samplesThisSegment = remainingSegments == 0
                ? remainingSamples
                : static_cast<int>(std::round((toothSegmentLengths[segment] / toothLength) * static_cast<double>(pointCount)));
            samplesThisSegment = std::max(1, samplesThisSegment);
            samplesThisSegment = std::min(samplesThisSegment, remainingSamples - remainingSegments);
            if (samplesThisSegment <= 0) break;
            const Vec3 a = toothControlPoints[segment];
            const Vec3 b = toothControlPoints[segment + 1];
            for (int sample = 1; sample <= samplesThisSegment; ++sample) {
                const double t = static_cast<double>(sample) / static_cast<double>(samplesThisSegment);
                generatedPositions.push_back(vecAdd(a, vecScale(vecSub(b, a), t)));
            }
            remainingSamples -= samplesThisSegment;
        }
        while (static_cast<int>(generatedPositions.size()) < pointCount + 1) generatedPositions.push_back(toothControlPoints.back());
        if (static_cast<int>(generatedPositions.size()) > pointCount + 1) generatedPositions.resize(static_cast<size_t>(pointCount + 1));
    }

    std::vector<std::array<double, 6>> solvedTargets;
    solvedTargets.reserve(generatedPositions.size());
    for (size_t i = 0; i < generatedPositions.size(); ++i) {
        const Vec3 position = generatedPositions[i];
        targetPose.values[3] = position.x;
        targetPose.values[7] = position.y;
        targetPose.values[11] = position.z;
        if (!poseController.setToolPose(targetPose, &errorMessage)) {
            std::cerr << "IK failed for generated point " << i << " TCP=("
                      << position.x << ", " << position.y << ", " << position.z
                      << "): " << errorMessage << std::endl;
            return 1;
        }
        solvedTargets.push_back(poseController.joints());
    }

    std::ofstream out(outputFile, std::ios::trunc);
    if (!out) {
        std::cerr << "Failed to open output program for writing: " << outputFile << std::endl;
        return 1;
    }
    out << "# RobotSimulatorProgram v1\n";
    out << "# Joints are stored in degrees. Linear speed and blending are millimeters.\n";
    out << "# Generated " << (triangularWeave ? "triangular weave" : "rectangular serpentine")
        << " TCP pattern from " << referenceFile << "\n";
    out << formatMoveLine("MoveJ", solvedTargets.front()) << "\n";
    for (size_t i = 1; i < solvedTargets.size(); ++i) out << formatMoveLine("MoveL", solvedTargets[i]) << "\n";
    out << formatMoveLine("MoveJ", std::array<double, 6>{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}}) << "\n";

    std::cout << (triangularWeave ? "generate-triangle-weave-program" : "generate-rect-program") << std::endl;
    std::cout << "  package=" << packageFile << std::endl;
    std::cout << "  reference=" << referenceFile << std::endl;
    std::cout << "  output=" << outputFile << std::endl;
    std::cout << "  move_l_points=" << (solvedTargets.size() - 1) << std::endl;
    std::cout << "  passes=" << passCount << std::endl;
    if (triangularWeave) std::cout << "  wavelength_mm=" << weaveWavelengthMm << std::endl;
    if (triangularWeave) std::cout << "  total_weaves=" << totalWeaveCycles << std::endl;
    if (triangularWeave) {
        const double reportedAmplitudeMm = RobotMotionCore::isFinite(weaveAmplitudeMm)
            ? weaveAmplitudeMm
            : weaveWavelengthMm * weaveAmplitude;
        std::cout << "  amplitude_mm=" << reportedAmplitudeMm << std::endl;
    }
    std::cout << "  tcp_center=(" << center.x << ", " << center.y << ", " << center.z << ")" << std::endl;
    std::cout << "  tcp_u_span_mm=" << (maxU - minU) << " tcp_v_span_mm=" << (maxV - minV) << std::endl;
    return 0;
}

int jsonArrayIntAt(const Json& values, int index, int fallback) {
    if (index < 0 || static_cast<size_t>(index) >= values.size()) return fallback;
    return jsoncompat::toInt(values[static_cast<size_t>(index)], fallback);
}

double jsonArrayDoubleAt(const Json& values, int index, double fallback) {
    if (index < 0 || static_cast<size_t>(index) >= values.size()) return fallback;
    return jsoncompat::toDouble(values[static_cast<size_t>(index)], fallback);
}

double jsonArrayScaledDoubleAt(const Json& values, int index, double scale, double fallback) {
    if (index < 0 || static_cast<size_t>(index) >= values.size() || scale == 0.0) return fallback;
    return jsoncompat::toDouble(values[static_cast<size_t>(index)], fallback * scale) / scale;
}

Json masteringObjectFromDocument(const Json& root) {
    Json mastering = jsoncompat::fieldObject(root, "mastering");
    if (!mastering.empty()) return mastering;

    if (jsoncompat::member(root, "joints").is_array()) {
        Json mastered = Json::array();
        Json zeroSteps = Json::array();
        Json currentSteps = Json::array();
        Json masterLimitSteps = Json::array();
        Json stepsPerDegree = Json::array();
        Json offsetDeg = Json::array();
        Json masterDir = Json::array();
        const Json joints = jsoncompat::fieldArray(root, "joints");
        for (int i = 0; i < 6; ++i) {
            const Json& joint = static_cast<size_t>(i) < joints.size() ? jsoncompat::toObject(joints[static_cast<size_t>(i)]) : jsoncompat::emptyObject();
            mastered.push_back(jsoncompat::fieldBool(joint, "mastered", false) ? 1 : 0);
            const int current = jsoncompat::fieldInt(joint, "current_steps", 0);
            zeroSteps.push_back(jsoncompat::fieldInt(joint, "zero_steps", current));
            // Restoring a saved zero reference sends current_steps and master_limit_steps, so both
            // must be collected from the per-joint form or Restore reference silently restores
            // nothing but the step scale.
            currentSteps.push_back(current);
            masterLimitSteps.push_back(jsoncompat::fieldInt(joint, "master_limit_steps", 0));
            stepsPerDegree.push_back(jsoncompat::fieldDouble(joint, "steps_per_deg", 0.0));
            offsetDeg.push_back(jsoncompat::fieldDouble(joint, "offset_deg", 0.0));
            if (jsoncompat::contains(joint, "master_dir")) {
                masterDir.push_back(jsoncompat::fieldInt(joint, "master_dir", 1) >= 0 ? 1 : -1);
            }
        }
        mastering["mastered"] = mastered;
        mastering["zero_steps"] = zeroSteps;
        mastering["current_steps"] = currentSteps;
        mastering["master_limit_steps"] = masterLimitSteps;
        mastering["steps_per_deg"] = stepsPerDegree;
        mastering["offset_deg"] = offsetDeg;
        if (masterDir.size() == 6) mastering["master_dir"] = masterDir;
        return mastering;
    }

    mastering["mastered"] = jsoncompat::fieldArray(root, "mastered");
    mastering["zero_steps"] = jsoncompat::fieldArray(root, "zero_steps");
    mastering["steps_per_deg"] = jsoncompat::fieldArray(root, "steps_per_deg");
    mastering["steps_per_deg_x1000"] = jsoncompat::fieldArray(root, "steps_per_deg_x1000");
    mastering["steps_per_deg_x10000000"] = jsoncompat::fieldArray(root, "steps_per_deg_x10000000");
    if (jsoncompat::contains(root, "master_dir")) {
        mastering["master_dir"] = jsoncompat::fieldArray(root, "master_dir");
    }
    return mastering;
}

// Welding schedules for the headless runs. They ride in the same document the Welding tab saves,
// so a program using WeaveOn <index> can be simulated from the command line against exactly the
// schedules the operator configured, instead of being refused for naming an empty one.
bool loadWeaveSchedulesFromMasteringFile(const std::string& path,
                                         RobotMotionCore::WeaveScheduleTable* schedules) {
    if (!schedules) return false;
    std::ifstream file(path, std::ios::binary);
    if (!file) return false;
    const std::string bytes((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    const Json doc = Json::parse(bytes.data(), bytes.data() + bytes.size(), nullptr, false);
    if (doc.is_discarded() || !doc.is_object()) return false;
    const int schema = jsoncompat::fieldInt(doc, "weave_schedule_schema",
                                            static_cast<int>(RobotMotionCore::kWeaveScheduleSchemaVersion));
    if (schema != static_cast<int>(RobotMotionCore::kWeaveScheduleSchemaVersion)) return false;
    const Json& entries = jsoncompat::fieldArray(doc, "weave_schedules");
    for (size_t i = 0; i < entries.size() && i < RobotMotionCore::kMaxWeaveSchedules; ++i) {
        if (!entries[i].is_object()) continue;
        RobotMotionCore::WeaveParams weave = {};
        if (!weaveParamsFromJson(entries[i], &weave)) continue;
        schedules->schedules[i] = weave;
        schedules->valid[i] = jsoncompat::fieldInt(entries[i], "valid", 0) != 0 ? 1 : 0;
    }
    return true;
}

// The planner settings a config file carries, as a settings override, so --simulate-program plans
// against the same limits the GUI does.
bool loadMotionSettingsOverrideFromConfigFile(const std::string& path,
                                              RobotProgramSimulator::MotionSettingsOverride* override,
                                              std::string* errorMessage) {
    if (!override) return false;
    std::ifstream file(path, std::ios::binary);
    if (!file) {
        if (errorMessage) *errorMessage = "Failed to open file: " + path;
        return false;
    }
    const std::string bytes((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    const Json doc = Json::parse(bytes.data(), bytes.data() + bytes.size(), nullptr, false);
    if (doc.is_discarded() || !doc.is_object()) {
        if (errorMessage) *errorMessage = "bad JSON";
        return false;
    }

    const Json& settings = jsoncompat::fieldObject(doc, "motion_settings");
    if (!settings.empty()) {
        const int schema = jsoncompat::fieldInt(settings, "motion_settings_schema",
                                               static_cast<int>(RobotMotionCore::kMotionSettingsSchemaVersion));
        if (schema != static_cast<int>(RobotMotionCore::kMotionSettingsSchemaVersion)) {
            if (errorMessage) {
                *errorMessage = "unsupported motion settings schema " + std::to_string(schema);
            }
            return false;
        }
        const auto assign = [](double* target, double value) {
            if (target && std::isfinite(value) && value > 0.0) *target = value;
        };
        assign(&override->controlPeriodSec, jsoncompat::fieldDouble(settings, "control_period_s"));
        assign(&override->singularityThresholdRad, jsoncompat::fieldDouble(settings, "singularity_threshold_rad"));
        assign(&override->defaultJointAccelerationRadSec2, jsoncompat::fieldDouble(settings, "default_joint_accel_rad_s2"));
        assign(&override->defaultJointJerkRadSec3, jsoncompat::fieldDouble(settings, "default_joint_jerk_rad_s3"));
        assign(&override->defaultLinearAccelerationMmSec2, jsoncompat::fieldDouble(settings, "default_linear_accel_mm_s2"));
        assign(&override->defaultLinearJerkMmSec3, jsoncompat::fieldDouble(settings, "default_linear_jerk_mm_s3"));
        assign(&override->defaultToolAngularSpeedRadSec, jsoncompat::fieldDouble(settings, "default_tool_angular_speed_rad_s"));
        assign(&override->defaultToolAngularAccelerationRadSec2, jsoncompat::fieldDouble(settings, "default_tool_angular_accel_rad_s2"));
        assign(&override->defaultToolAngularJerkRadSec3, jsoncompat::fieldDouble(settings, "default_tool_angular_jerk_rad_s3"));
    }

    const Json& limitScales = jsoncompat::contains(doc, "dynamic_limit_scale")
        ? jsoncompat::fieldArray(doc, "dynamic_limit_scale")
        : jsoncompat::fieldArray(doc, "joint_limit_scale");
    for (size_t i = 0; i < override->dynamicLimitScale.size() && i < limitScales.size(); ++i) {
        const double value = jsoncompat::toDouble(limitScales[i], 1.0);
        override->dynamicLimitScale[i] = value > 0.0 ? value : 1.0;
    }
    return true;
}

bool loadStepEstimatorFromMasteringFile(const std::string& path,
                                        RobotProgramSimulator::StepEstimator* estimator,
                                        std::string* errorMessage) {
    if (!estimator) return false;
    std::ifstream file(path, std::ios::binary);
    if (!file) {
        if (errorMessage) *errorMessage = "Failed to open file: " + path;
        return false;
    }

    const std::string bytes((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    const Json doc = Json::parse(bytes.data(), bytes.data() + bytes.size(),
                                 nullptr, /*allow_exceptions=*/false);
    if (doc.is_discarded() || !doc.is_object()) {
        if (errorMessage) *errorMessage = "bad JSON";
        return false;
    }

    const Json mastering = masteringObjectFromDocument(doc);
    const Json mastered = jsoncompat::fieldArray(mastering, "mastered");
    const Json zeroSteps = jsoncompat::fieldArray(mastering, "zero_steps");
    const Json stepsPerDegreePlain = jsoncompat::fieldArray(mastering, "steps_per_deg");
    const Json stepsPerDegreeX1000 = jsoncompat::fieldArray(mastering, "steps_per_deg_x1000");
    const Json stepsPerDegreeX10000000 = jsoncompat::fieldArray(mastering, "steps_per_deg_x10000000");

    RobotProgramSimulator::StepEstimator parsed;
    for (int i = 0; i < 6; ++i) {
        if (jsonArrayIntAt(mastered, i, 1) != 1) {
            if (errorMessage) *errorMessage = strutil::format("joint %1 is not mastered").arg(i + 1);
            return false;
        }
        const double plain = jsonArrayDoubleAt(stepsPerDegreePlain, i, 0.0);
        const double fallback = jsonArrayIntAt(stepsPerDegreeX1000, i, -1) > 0
                                    ? jsonArrayIntAt(stepsPerDegreeX1000, i, -1) / 1000.0
                                    : plain;
        const double stepsPerDegree = jsonArrayScaledDoubleAt(stepsPerDegreeX10000000, i, 10000000.0, fallback);
        if (!RobotMotionCore::validStepsPerDegree(stepsPerDegree)) {
            if (errorMessage) *errorMessage = strutil::format("invalid steps_per_deg for joint %1").arg(i + 1);
            return false;
        }
        parsed.zeroSteps[static_cast<size_t>(i)] = jsonArrayIntAt(zeroSteps, i, 0);
        parsed.stepsPerDegree[static_cast<size_t>(i)] = stepsPerDegree;
    }

    parsed.enabled = true;
    *estimator = parsed;
    return true;
}

// Runs a program the way an arm in the station view does - fed to the lookahead as commands retire,
// stepped by a clock - and checks it against the same program planned up front. Both routes drive the
// same MotionWindowRunner over the same commands at the same control period, so total run time and
// final pose must agree; a mismatch means the live feeding is wrong.
int liveRunCommand(const std::vector<std::string>& args, bool threaded) {
    if (args.size() < 4) {
        std::cerr << "Usage: RobotSimulator.exe " << (threaded ? "--live-run-threaded" : "--live-run")
                  << " <robot.zip|json> <program.robotprog.txt> "
                     "[--mastering file.json] [--cycles n] [--arms n]"
                  << (threaded ? " [--speed x]" : "")
                  << std::endl;
        return 2;
    }
    const std::string packageFile = args[2];
    const std::string programFile = args[3];
    std::string masteringFile;
    int cycles = 1;
    // More than one arm running at once is the cell case, and the only thing steps 4 and 5 introduce
    // that a single run does not cover: the arms are stepped one after another inside a frame, each
    // through its own planner scratch. Interleaving them here and requiring every one to still match
    // the planned build is what says that is safe.
    int arms = 1;
    // Threaded only, and high by default: the driver paces against the wall clock, so a 72-second
    // program stepped at 1x takes 72 seconds to check. What cannot be delivered is dropped, which delays
    // the finish in wall time and leaves the trajectory alone - which is the invariant being tested.
    double speedFactor = 100.0;
    for (size_t i = 4; i < args.size(); i += 2) {
        if (i + 1 >= args.size()) {
            std::cerr << "Missing value for option: " << args[i] << std::endl;
            return 2;
        }
        if (args[i] == "--mastering") {
            masteringFile = args[i + 1];
        } else if (args[i] == "--cycles") {
            double value = 0.0;
            if (!strutil::parseDouble(args[i + 1], &value) || value < 1.0) {
                std::cerr << "--cycles must be at least 1." << std::endl;
                return 2;
            }
            cycles = static_cast<int>(value);
        } else if (args[i] == "--speed") {
            double value = 0.0;
            if (!strutil::parseDouble(args[i + 1], &value) || value <= 0.0) {
                std::cerr << "--speed must be greater than zero." << std::endl;
                return 2;
            }
            speedFactor = value;
        } else if (args[i] == "--arms") {
            double value = 0.0;
            if (!strutil::parseDouble(args[i + 1], &value) || value < 1.0) {
                std::cerr << "--arms must be at least 1." << std::endl;
                return 2;
            }
            arms = static_cast<int>(value);
        } else {
            std::cerr << "Unknown option: " << args[i] << std::endl;
            return 2;
        }
    }

    std::string errorMessage;
    std::shared_ptr<CadNode> loadedNode = loadCadNodePackage(packageFile, &errorMessage);
    if (!loadedNode || !validateRobotPackage(loadedNode.get(), &errorMessage)) {
        std::cerr << "Package validation failed: " << errorMessage << std::endl;
        return 1;
    }
    RobotPoseController poseController;
    if (!poseController.bind(loadedNode.get(), &errorMessage)) {
        std::cerr << "Package pose bind failed: " << errorMessage << std::endl;
        return 1;
    }
    poseController.resetHome();
    const std::array<double, 6> startJoints = poseController.joints();

    RobotProgramNode root;
    root.type = RobotProgramNodeType::Root;
    if (!loadProgramTextHeadless(programFile, &root, &errorMessage)) {
        std::cerr << "Program load failed: " << errorMessage << std::endl;
        return 1;
    }

    RobotProgramSimulator::MotionSettingsOverride settingsOverride;
    RobotProgramSimulator::StepEstimator estimator;
    RobotMotionCore::WeaveScheduleTable schedules = {};
    bool haveEstimator = false;
    if (!masteringFile.empty()) {
        if (!loadStepEstimatorFromMasteringFile(masteringFile, &estimator, &errorMessage)) {
            std::cerr << "Mastering load failed: " << errorMessage << std::endl;
            return 1;
        }
        haveEstimator = true;
        loadWeaveSchedulesFromMasteringFile(masteringFile, &schedules);
        if (!loadMotionSettingsOverrideFromConfigFile(masteringFile, &settingsOverride, &errorMessage)) {
            std::cerr << "Motion settings load failed: " << errorMessage << std::endl;
            return 1;
        }
    }

    // Configured identically, so a difference in the numbers below can only come from how the program
    // was executed. The step estimator is deliberately left off both: it quantises the stored pose on
    // the planned side only, and comparing a rounded position against an exact one proves nothing.
    auto configure = [&](RobotProgramSimulator* simulator) {
        simulator->setMotionSettingsOverride(settingsOverride);
        if (haveEstimator) simulator->setWeaveSchedules(schedules);
        simulator->start(&root, startJoints);
    };

    const RobotProgramSimulator::PlanInput plan =
        RobotProgramSimulator::planInputFromPoseController(poseController);

    RobotProgramSimulator planned;
    configure(&planned);
    if (!planned.buildTrajectory(plan, &errorMessage)) {
        std::cerr << "Planned build failed: " << errorMessage << std::endl;
        return 1;
    }
    if (planned.trajectory().empty()) {
        std::cerr << "Planned build produced no samples." << std::endl;
        return 1;
    }
    const double plannedSeconds = planned.trajectory().back().timeSeconds;
    const std::array<double, 6> plannedJoints = planned.trajectory().back().plannedJoints;

    // One pose controller per arm, so they cannot write over each other's joints - the same way each
    // RobotInstance owns its own in the station.
    const bool loop = cycles > 1;
    std::vector<std::unique_ptr<RobotProgramSimulator>> liveArms;
    std::vector<RobotPoseController> armControllers(static_cast<size_t>(arms));
    for (int arm = 0; arm < arms; ++arm) {
        RobotPoseController& controller = armControllers[static_cast<size_t>(arm)];
        if (!controller.bind(loadedNode.get(), &errorMessage)) {
            std::cerr << "Arm pose bind failed: " << errorMessage << std::endl;
            return 1;
        }
        controller.setJoints(startJoints);
        auto simulator = std::make_unique<RobotProgramSimulator>();
        configure(simulator.get());
        if (!simulator->beginLiveRun(plan, loop, &errorMessage)) {
            std::cerr << "Live run rejected: " << errorMessage << std::endl;
            return 1;
        }
        liveArms.push_back(std::move(simulator));
    }

    std::vector<RobotProgramSimulator::LiveRunState> finalStates(static_cast<size_t>(arms));

    if (!threaded) {
        int guard = 0;
        bool anyRunning = true;
        while (anyRunning && guard++ < 100000) {
            anyRunning = false;
            for (int arm = 0; arm < arms; ++arm) {
                RobotProgramSimulator& simulator = *liveArms[static_cast<size_t>(arm)];
                const RobotProgramSimulator::LiveRunState& armState = simulator.liveRunState();
                if (!armState.running) continue;
                if (loop && armState.completedCycles >= cycles) continue;
                const RobotProgramSimulator::LiveRunStep step = simulator.stepLiveRun(1.0);
                if (step.jointsChanged) {
                    armControllers[static_cast<size_t>(arm)].setJoints(simulator.liveRunJoints());
                }
                if (step.running) anyRunning = true;
            }
        }
        for (int arm = 0; arm < arms; ++arm) {
            finalStates[static_cast<size_t>(arm)] = liveArms[static_cast<size_t>(arm)]->liveRunState();
        }
    } else {
#ifdef __EMSCRIPTEN__
        std::cerr << "--live-run-threaded needs threads, which the WebAssembly build does not have."
                  << std::endl;
        return 2;
#else
        // The GUI's arrangement, without the GUI: a worker thread looping stepOnce, and this thread
        // taking published joints and applying them to the pose controllers - which is where setJoints
        // belongs, because in the real thing those controllers are writing the scene graph a renderer is
        // walking.
        LiveRunDriver driver;
        std::vector<RobotProgramSimulator*> armPointers;
        for (const std::unique_ptr<RobotProgramSimulator>& simulator : liveArms) {
            armPointers.push_back(simulator.get());
        }
        {
            auto control = driver.lockForControl();
            driver.setArms(armPointers);
        }
        driver.setSpeedFactor(speedFactor);
        driver.resetClock();

        std::atomic<bool> stop{false};
        std::thread worker([&driver, &stop]() {
            while (!stop.load()) {
                const LiveRunDriver::StepResult step = driver.stepOnce();
                if (!step.anyRunning) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                } else if (!step.didWork) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                } else {
                    std::this_thread::yield();
                }
            }
        });

        const std::chrono::steady_clock::time_point deadline =
            std::chrono::steady_clock::now() + std::chrono::seconds(300);
        bool timedOut = false;
        std::vector<bool> stopped(static_cast<size_t>(arms), false);
        // Sampled while the run is going, because the driver unpublishes the rate once the last arm
        // stops - a figure left standing after the run it described would be a claim about nothing.
        double lastRate = 0.0;
        bool haveRate = false;
        for (;;) {
            bool anyRunning = false;
            for (int arm = 0; arm < arms; ++arm) {
                const size_t index = static_cast<size_t>(arm);
                if (stopped[index]) continue;
                LiveRunDriver::Snapshot snapshot;
                if (!driver.snapshot(index, &snapshot)) continue;
                if (snapshot.jointsValid) armControllers[index].setJoints(snapshot.joints);
                finalStates[index] = snapshot.state;
                if (!snapshot.state.running) continue;
                if (loop && snapshot.state.completedCycles >= cycles) {
                    // Captured just above, because endLiveRun resets the run and the cycle count in
                    // finalStates is what says whether it looped as asked.
                    stopped[index] = true;
                    auto control = driver.lockForControl();
                    liveArms[index]->endLiveRun();
                    driver.publishFrom(index);
                    continue;
                }
                anyRunning = true;
            }
            double rateNow = 0.0;
            if (driver.achievedRate(&rateNow)) {
                lastRate = rateNow;
                haveRate = true;
            }
            if (!anyRunning) break;
            if (std::chrono::steady_clock::now() > deadline) {
                timedOut = true;
                break;
            }
            // Roughly a frame, because this is standing in for the frame thread.
            std::this_thread::sleep_for(std::chrono::milliseconds(16));
        }

        stop.store(true);
        worker.join();
        if (timedOut) {
            std::cerr << "Threaded live run did not finish within the deadline." << std::endl;
            return 1;
        }
        std::cout << "driver speed=" << strutil::format("%1").arg(speedFactor, 0, 'g', 6).str()
                  << " achieved=" << (haveRate ? strutil::format("%1").arg(lastRate, 0, 'g', 6).str()
                                               : std::string("-"))
                  << " dropped_s="
                  << strutil::format("%1").arg(driver.droppedSimSeconds(), 0, 'g', 6).str()
                  << std::endl;
#endif
    }

    std::cout << "planned run_s=" << strutil::format("%1").arg(plannedSeconds, 0, 'f', 9).str() << std::endl;
    int failures = 0;
    for (int arm = 0; arm < arms; ++arm) {
        const RobotProgramSimulator::LiveRunState& state = finalStates[static_cast<size_t>(arm)];
        std::cout << "arm " << arm
                  << " status=" << (state.status.empty() ? "-" : state.status)
                  << " cycles=" << state.completedCycles
                  << " instruction=" << state.instruction
                  << " run_s=" << strutil::format("%1").arg(state.runSeconds, 0, 'f', 9).str()
                  << " cycle_s=" << strutil::format("%1").arg(state.cycleSeconds, 0, 'f', 9).str();
        if (loop) {
            std::cout << std::endl;
            if (state.completedCycles < cycles) ++failures;
            continue;
        }
        const double secondsDelta = std::abs(state.runSeconds - plannedSeconds);
        double jointsDelta = 0.0;
        const std::array<double, 6> liveJoints = armControllers[static_cast<size_t>(arm)].joints();
        for (size_t joint = 0; joint < liveJoints.size(); ++joint) {
            jointsDelta = std::max(jointsDelta, std::abs(liveJoints[joint] - plannedJoints[joint]));
        }
        std::cout << " delta_run_s=" << strutil::format("%1").arg(secondsDelta, 0, 'g', 6).str()
                  << " delta_joints_rad=" << strutil::format("%1").arg(jointsDelta, 0, 'g', 6).str()
                  << std::endl;
        if (state.status != "Complete" || secondsDelta > 1.0e-9 || jointsDelta > 1.0e-9) ++failures;
    }
    if (failures > 0) {
        std::cerr << failures << " of " << arms << " live runs disagree with the planned build." << std::endl;
        return 1;
    }
    std::cout << (loop ? "live runs looped as asked" : "live runs match the planned build")
              << " (" << arms << (arms == 1 ? " arm" : " arms")
              << (threaded ? ", threaded)" : ")") << std::endl;
    return 0;
}

int simulateProgramCommand(const std::vector<std::string>& args) {
    if (args.size() < 4) {
        std::cerr << "Usage: RobotSimulator.exe --simulate-program <robot.zip|json> <program.robotprog.txt> "
                     "[--blend-mm value] [--linear-mm-s value] [--joint-deg-s value] [--dt value] "
                     "[--mastering file.json] [--dump-trajectory file.csv]"
                  << std::endl;
        return 2;
    }

    const std::string packageFile = args[2];
    const std::string programFile = args[3];
    double blendOverrideMm = std::numeric_limits<double>::quiet_NaN();
    double linearOverrideMmPerSec = std::numeric_limits<double>::quiet_NaN();
    double jointOverrideDegPerSec = std::numeric_limits<double>::quiet_NaN();
    // Unset by default so the config's control_period_s decides; a value here overrides it and is
    // what the report prints.
    double dtSeconds = std::numeric_limits<double>::quiet_NaN();
    std::string dumpTrajectoryFile;
    std::string masteringFile;

    for (int i = 4; i < args.size(); i += 2) {
        if (i + 1 >= args.size()) {
            std::cerr << "Missing value for option: " << args[i] << std::endl;
            return 2;
        }
        if (args[i] == "--dump-trajectory") {
            dumpTrajectoryFile = args[i + 1];
            continue;
        }
        if (args[i] == "--mastering") {
            masteringFile = args[i + 1];
            continue;
        }
        double value = 0.0;
        if (!strutil::parseDouble(args[i + 1], &value)) {
            std::cerr << "Invalid numeric value for " << args[i] << std::endl;
            return 2;
        }
        if (args[i] == "--blend-mm") {
            blendOverrideMm = value;
        } else if (args[i] == "--linear-mm-s") {
            linearOverrideMmPerSec = value;
        } else if (args[i] == "--joint-deg-s") {
            jointOverrideDegPerSec = value;
        } else if (args[i] == "--dt") {
            dtSeconds = value;
        } else {
            std::cerr << "Unknown option: " << args[i] << std::endl;
            return 2;
        }
    }
    if (std::isfinite(dtSeconds) && dtSeconds <= 0.0) {
        std::cerr << "--dt must be positive." << std::endl;
        return 2;
    }

    std::string errorMessage;
    std::shared_ptr<CadNode> loadedNode = loadCadNodePackage(packageFile, &errorMessage);
    if (!loadedNode || !validateRobotPackage(loadedNode.get(), &errorMessage)) {
        std::cerr << "Package validation failed: " << errorMessage << std::endl;
        return 1;
    }

    RobotPoseController poseController;
    if (!poseController.bind(loadedNode.get(), &errorMessage)) {
        std::cerr << "Package pose bind failed: " << errorMessage << std::endl;
        return 1;
    }
    poseController.resetHome();
    const std::array<double, 6> startJoints = poseController.joints();

    RobotProgramNode root;
    root.type = RobotProgramNodeType::Root;
    if (!loadProgramTextHeadless(programFile, &root, &errorMessage)) {
        std::cerr << "Program load failed: " << errorMessage << std::endl;
        return 1;
    }

    auto prependInstructionToRoot = [&](RobotProgramNodeType type, RobotProgramNodeData data) {
        auto node = std::make_unique<RobotProgramNode>();
        node->type = type;
        node->data = std::move(data);
        node->parent = &root;
        root.children.insert(root.children.begin(), std::move(node));
    };

    if (std::isfinite(jointOverrideDegPerSec) || std::isfinite(linearOverrideMmPerSec)) {
        bool updatedExisting = false;
        for (const auto& child : root.children) {
            if (!child || child->type != RobotProgramNodeType::SetSpeed) continue;
            SetSpeedData& speed = std::get<SetSpeedData>(child->data);
            if (std::isfinite(jointOverrideDegPerSec)) speed.jointSpeedRadPerSec = jointOverrideDegPerSec * kDegToRad;
            if (std::isfinite(linearOverrideMmPerSec)) speed.linearSpeedMmPerSec = linearOverrideMmPerSec;
            updatedExisting = true;
        }
        if (!updatedExisting) {
            prependInstructionToRoot(
                RobotProgramNodeType::SetSpeed,
                SetSpeedData{
                    (std::isfinite(jointOverrideDegPerSec) ? jointOverrideDegPerSec : 30.0) * kDegToRad,
                    std::isfinite(linearOverrideMmPerSec) ? linearOverrideMmPerSec : 100.0});
        }
    }
    if (std::isfinite(blendOverrideMm)) {
        bool updatedExisting = false;
        for (const auto& child : root.children) {
            if (!child || child->type != RobotProgramNodeType::SetBlending) continue;
            std::get<SetBlendingData>(child->data).radiusMm = blendOverrideMm;
            updatedExisting = true;
        }
        if (!updatedExisting) prependInstructionToRoot(RobotProgramNodeType::SetBlending, SetBlendingData{blendOverrideMm});
    }

    struct TargetSummary {
        const RobotProgramNode* node = nullptr;
        std::string type;
        CadTransform pose;
        int32_t commandId = -1;
        double minDistanceMm = std::numeric_limits<double>::max();
        double completionDistanceMm = std::numeric_limits<double>::quiet_NaN();
        bool completionSeen = false;
    };

    std::vector<TargetSummary> targets;
    for (const auto& child : root.children) {
        if (!child) continue;
        const std::array<double, 6>* targetJoints = targetJointsForNode(*child);
        if (!targetJoints) continue;
        poseController.setJoints(*targetJoints);
        targets.push_back({child.get(), motionTypeName(child->type), poseController.toolPose(),
                           static_cast<int32_t>(targets.size())});
    }
    poseController.setJoints(startJoints);

    RobotProgramSimulator simulator;
    RobotProgramSimulator::MotionSettingsOverride settingsOverride;
    if (!masteringFile.empty()) {
        RobotProgramSimulator::StepEstimator estimator;
        if (!loadStepEstimatorFromMasteringFile(masteringFile, &estimator, &errorMessage)) {
            std::cerr << "Mastering load failed: " << errorMessage << std::endl;
            return 1;
        }
        simulator.setStepEstimator(estimator);
        RobotMotionCore::WeaveScheduleTable schedules = {};
        if (loadWeaveSchedulesFromMasteringFile(masteringFile, &schedules)) {
            simulator.setWeaveSchedules(schedules);
        }
        if (!loadMotionSettingsOverrideFromConfigFile(masteringFile, &settingsOverride, &errorMessage)) {
            std::cerr << "Motion settings load failed: " << errorMessage << std::endl;
            return 1;
        }
    }
    // An explicit --dt outranks the file, so a run can be sampled finer than the robot's own period
    // without editing a config.
    if (std::isfinite(dtSeconds)) settingsOverride.controlPeriodSec = dtSeconds;
    simulator.setMotionSettingsOverride(settingsOverride);

    RobotProgramSimulator::ProgramState initialState;
    {
        const RobotProgramSimulator::PlanInput robotDefaults =
            RobotProgramSimulator::planInputFromPoseController(poseController);
        if (robotDefaults.defaultLinearSpeedMmPerSec > 0.0) {
            initialState.linearSpeedMmPerSec = robotDefaults.defaultLinearSpeedMmPerSec;
        }
        if (robotDefaults.defaultJointSpeedDegPerSec > 0.0) {
            initialState.jointSpeedRadPerSec = robotDefaults.defaultJointSpeedDegPerSec * kDegToRad;
        }
    }
    simulator.start(&root, startJoints, initialState);
    double minTcpSpeed = std::numeric_limits<double>::max();
    double maxTcpSpeed = 0.0;
    double sumTcpSpeed = 0.0;
    int cartesianSpeedSamples = 0;
    if (!simulator.buildTrajectory(RobotProgramSimulator::planInputFromPoseController(poseController), &errorMessage)) {
        std::cerr << "Simulation failed: " << errorMessage << std::endl;
        return 1;
    }

    const std::vector<RobotProgramSimulator::PlannedSample>& samples = simulator.trajectory();
    for (const RobotProgramSimulator::PlannedSample& sample : samples) {
        const CadTransform& currentPose = sample.tcpPose;
        const double tcpSpeed = sample.actualTcpSpeedMmPerSec;
        if (std::isfinite(sample.desiredTcpSpeedMmPerSec) && std::isfinite(tcpSpeed)) {
            minTcpSpeed = std::min(minTcpSpeed, tcpSpeed);
            maxTcpSpeed = std::max(maxTcpSpeed, tcpSpeed);
            sumTcpSpeed += tcpSpeed;
            ++cartesianSpeedSamples;
        }

        for (TargetSummary& target : targets) {
            target.minDistanceMm = std::min(target.minDistanceMm, tcpDistance(currentPose, target.pose));
            if (!target.completionSeen &&
                ((sample.completedCommandId >= 0 && sample.completedCommandId == target.commandId) ||
                 (sample.completedCommandId < 0 && sample.segmentEnd && sample.activeNode == target.node))) {
                target.completionDistanceMm = target.minDistanceMm;
                target.completionSeen = true;
            }
        }
    }

    if (!dumpTrajectoryFile.empty()) {
        std::ofstream out(dumpTrajectoryFile);
        if (!out) {
            std::cerr << "Failed to open trajectory CSV for writing: " << dumpTrajectoryFile << std::endl;
            return 1;
        }
        out << "time_s,q1_rad,q2_rad,q3_rad,q4_rad,q5_rad,q6_rad,tcp_x_mm,tcp_y_mm,tcp_z_mm,actual_tcp_mm_s,stepped_tcp_mm_s,profile_tcp_mm_s,command_tcp_mm_s,segment_end,weave_phase_cycles,weave_lateral_mm\n";
        for (const RobotProgramSimulator::PlannedSample& sample : samples) {
            out << strutil::formatShortest(sample.timeSeconds);
            for (double q : sample.joints) out << "," << strutil::formatShortest(q);
            out << "," << strutil::formatShortest(sample.tcpPose.values[3])
                << "," << strutil::formatShortest(sample.tcpPose.values[7])
                << "," << strutil::formatShortest(sample.tcpPose.values[11])
                << "," << strutil::formatShortest(sample.actualTcpSpeedMmPerSec)
                << "," << strutil::formatShortest(sample.steppedTcpSpeedMmPerSec)
                << "," << strutil::formatShortest(sample.profileTcpSpeedMmPerSec)
                << "," << strutil::formatShortest(sample.desiredTcpSpeedMmPerSec)
                << "," << (sample.segmentEnd ? "1" : "0")
                << "," << strutil::formatShortest(sample.weavePhaseCycles)
                << "," << strutil::formatShortest(sample.weaveLateralMm) << "\n";
        }
    }

    std::cout << "simulate-program" << std::endl;
    std::cout << "  package=" << packageFile << std::endl;
    std::cout << "  program=" << programFile << std::endl;
    std::cout << "  blend_mm=" << (std::isfinite(blendOverrideMm) ? strutil::formatFixed(blendOverrideMm, 3) : std::string("program/default")) << std::endl;
    std::cout << "  linear_mm_s=" << (std::isfinite(linearOverrideMmPerSec) ? strutil::formatFixed(linearOverrideMmPerSec, 3) : std::string("program/default")) << std::endl;
    std::cout << "  joint_deg_s=" << (std::isfinite(jointOverrideDegPerSec) ? strutil::formatFixed(jointOverrideDegPerSec, 3) : std::string("program/default")) << std::endl;
    std::cout << "  mastering=" << (!masteringFile.empty() ? masteringFile : std::string("none")) << std::endl;
    std::cout << "  control_period_s="
              << (std::isfinite(settingsOverride.controlPeriodSec) && settingsOverride.controlPeriodSec > 0.0
                      ? strutil::formatShortest(settingsOverride.controlPeriodSec)
                      : std::string("package/default"))
              << (std::isfinite(dtSeconds) ? " (--dt)" : "") << std::endl;
    std::cout << "  program_default_speed=" << strutil::formatFixed(initialState.linearSpeedMmPerSec, 3)
              << " mm/s, " << strutil::formatFixed(initialState.jointSpeedRadPerSec * kRadToDeg, 3)
              << " deg/s, blend " << strutil::formatFixed(initialState.blendRadiusMm, 3) << " mm"
              << std::endl;
    std::cout << "  dynamic_limit_scale=";
    for (size_t i = 0; i < settingsOverride.dynamicLimitScale.size(); ++i) {
        const double scale = settingsOverride.dynamicLimitScale[i];
        std::cout << (i ? "," : "") << (scale > 0.0 ? strutil::formatShortest(scale) : std::string("1"));
    }
    std::cout << std::endl;
    const RobotProgramSimulator::Statistics& stats = simulator.statistics();
    std::cout << "  duration_s=" << simulator.durationSeconds() << " samples=" << samples.size() << std::endl;
    std::cout << "  trajectory_edges=" << stats.trajectoryEdgeCount
              << " septic_edges=" << stats.septicEdgeCount
              << std::endl;
    std::cout << "  worst_window_seam_mm_s=" << strutil::formatFixed(stats.worstWindowSeamSpeedMmPerSec, 2)
              << std::endl;
    if (!dumpTrajectoryFile.empty()) {
        std::cout << "  trajectory_csv=" << dumpTrajectoryFile << std::endl;
    }
    if (!simulator.warnings().empty()) {
        std::cout << "  planner_warnings:" << std::endl;
        for (const std::string& warning : simulator.warnings()) {
            std::cout << "    " << warning << std::endl;
        }
    }
    if (!simulator.markers().empty()) {
        std::cout << "  planner_markers:" << std::endl;
        for (const RobotProgramSimulator::Marker& marker : simulator.markers()) {
            const char* type = "marker";
            switch (marker.type) {
            case RobotProgramSimulator::MarkerType::Singularity:
                type = "singularity";
                break;
            case RobotProgramSimulator::MarkerType::JointFlip:
                type = "joint_flip";
                break;
            case RobotProgramSimulator::MarkerType::PlannerCap:
                type = "planner_cap";
                break;
            case RobotProgramSimulator::MarkerType::Trigger:
                type = "trigger";
                break;
            }
            std::cout << "    t=" << marker.seconds
                      << " " << type
                      << " " << marker.message
                      << std::endl;
        }
    }
    if (cartesianSpeedSamples > 0) {
        std::cout << "  cartesian_tcp_speed_mm_s min=" << minTcpSpeed
                  << " avg=" << (sumTcpSpeed / static_cast<double>(cartesianSpeedSamples))
                  << " max=" << maxTcpSpeed
                  << " samples=" << cartesianSpeedSamples << std::endl;
    }
    std::cout << "  max_tcp_accel_mm_s2=" << stats.maxTcpAccelerationMmS2 << std::endl;
    std::cout << "  max_tcp_jerk_mm_s3=" << stats.maxTcpJerkMmS3 << std::endl;
    std::cout << "  max_tool_angular_speed_deg_s=" << stats.maxToolAngularSpeedRadS * kRadToDeg << std::endl;
    std::cout << "  max_tool_angular_accel_deg_s2=" << stats.maxToolAngularAccelerationRadS2 * kRadToDeg << std::endl;
    std::cout << "  max_tool_angular_jerk_deg_s3=" << stats.maxToolAngularJerkRadS3 * kRadToDeg << std::endl;
    std::cout << "  max_joint_speed_deg_s=";
    for (int i = 0; i < 6; ++i) {
        if (i) std::cout << ",";
        std::cout << stats.maxJointSpeedRadS[static_cast<size_t>(i)] * kRadToDeg;
    }
    std::cout << std::endl;
    std::cout << "  max_joint_accel_deg_s2=";
    for (int i = 0; i < 6; ++i) {
        if (i) std::cout << ",";
        std::cout << stats.maxJointAccelerationRadS2[static_cast<size_t>(i)] * kRadToDeg;
    }
    std::cout << std::endl;
    std::cout << "  max_joint_jerk_deg_s3=";
    for (int i = 0; i < 6; ++i) {
        if (i) std::cout << ",";
        std::cout << stats.maxJointJerkRadS3[static_cast<size_t>(i)] * kRadToDeg;
    }
    std::cout << std::endl;
    if (const OPW6RobotData* robot = poseController.robotData()) {
        if (robot->defaultLinearAccelerationMmSec2 > 0.0) {
            std::cout << "  max_tcp_accel_ratio="
                      << stats.maxTcpAccelerationMmS2 / robot->defaultLinearAccelerationMmSec2
                      << std::endl;
        }
        if (robot->defaultLinearJerkMmSec3 > 0.0) {
            std::cout << "  max_tcp_jerk_ratio="
                      << stats.maxTcpJerkMmS3 / robot->defaultLinearJerkMmSec3
                      << std::endl;
        }
        const RobotMotionCore::MotionProgramSettings defaultMotionSettings = RobotMotionCore::defaultMotionProgramSettings();
        const double toolAngularSpeedLimit = robot->defaultToolAngularSpeedRadSec > 0.0
            ? robot->defaultToolAngularSpeedRadSec
            : defaultMotionSettings.defaultToolAngularSpeedRadSec;
        const double toolAngularAccelLimit = robot->defaultToolAngularAccelerationRadSec2 > 0.0
            ? robot->defaultToolAngularAccelerationRadSec2
            : defaultMotionSettings.defaultToolAngularAccelerationRadSec2;
        const double toolAngularJerkLimit = robot->defaultToolAngularJerkRadSec3 > 0.0
            ? robot->defaultToolAngularJerkRadSec3
            : defaultMotionSettings.defaultToolAngularJerkRadSec3;
        if (toolAngularSpeedLimit > 0.0) {
            std::cout << "  max_tool_angular_speed_ratio="
                      << stats.maxToolAngularSpeedRadS / toolAngularSpeedLimit
                      << std::endl;
        }
        if (toolAngularAccelLimit > 0.0) {
            std::cout << "  max_tool_angular_accel_ratio="
                      << stats.maxToolAngularAccelerationRadS2 / toolAngularAccelLimit
                      << std::endl;
        }
        if (toolAngularJerkLimit > 0.0) {
            std::cout << "  max_tool_angular_jerk_ratio="
                      << stats.maxToolAngularJerkRadS3 / toolAngularJerkLimit
                      << std::endl;
        }
        if (robot->hasJointVelocityLimits) {
            std::cout << "  max_joint_speed_ratio=";
            for (int i = 0; i < 6; ++i) {
                if (i) std::cout << ",";
                const double limit = robot->jointVelocityMaxRadS[static_cast<size_t>(i)];
                std::cout << (limit > 0.0 ? stats.maxJointSpeedRadS[static_cast<size_t>(i)] / limit : 0.0);
            }
            std::cout << std::endl;
        }
        if (robot->hasJointAccelerationLimits) {
            std::cout << "  max_joint_accel_ratio=";
            for (int i = 0; i < 6; ++i) {
                if (i) std::cout << ",";
                const double limit = robot->jointAccelerationMaxRadS2[static_cast<size_t>(i)];
                std::cout << (limit > 0.0 ? stats.maxJointAccelerationRadS2[static_cast<size_t>(i)] / limit : 0.0);
            }
            std::cout << std::endl;
        }
        if (robot->hasJointJerkLimits) {
            std::cout << "  max_joint_jerk_ratio=";
            for (int i = 0; i < 6; ++i) {
                if (i) std::cout << ",";
                const double limit = robot->jointJerkMaxRadS3[static_cast<size_t>(i)];
                std::cout << (limit > 0.0 ? stats.maxJointJerkRadS3[static_cast<size_t>(i)] / limit : 0.0);
            }
            std::cout << std::endl;
        }
    }
    std::cout << "  targets:" << std::endl;
    for (size_t i = 0; i < targets.size(); ++i) {
        const TargetSummary& target = targets[i];
        std::cout << "    " << (i + 1) << " " << target.type
                  << " min_dist_mm=" << target.minDistanceMm
                  << " completion_dist_mm=";
        if (std::isfinite(target.completionDistanceMm)) {
            std::cout << target.completionDistanceMm;
        } else {
            std::cout << "n/a";
        }
        std::cout << std::endl;
    }
    return 0;
}

int collisionPoseCommand(const std::vector<std::string>& args) {
    if (args.size() < 9) {
        std::cerr << "Usage: RobotSimulator --collision-pose <package> j1 j2 j3 j4 j5 j6 [iterations]" << std::endl;
        return 1;
    }

    std::string errorMessage;
    std::shared_ptr<CadNode> loadedNode = loadCadNodePackage(args[2], &errorMessage);
    if (!loadedNode || !validateRobotPackage(loadedNode.get(), &errorMessage)) {
        std::cerr << "Package validation failed: " << errorMessage << std::endl;
        return 1;
    }

    RobotPoseController poseController;
    if (!poseController.bind(loadedNode.get(), &errorMessage)) {
        std::cerr << "Package pose bind failed: " << errorMessage << std::endl;
        return 1;
    }

    RobotCollisionModel collisionModel;
    if (!collisionModel.bind(loadedNode.get(), &errorMessage)) {
        std::cerr << "Package collision bind failed: " << errorMessage << std::endl;
        return 1;
    }

    std::array<double, 6> q{};
    for (int i = 0; i < 6; ++i) {
        double degrees = 0.0;
        if (!strutil::parseDouble(args[3 + i], &degrees) || !std::isfinite(degrees)) {
            std::cerr << "Invalid joint degree value: " << args[3 + i] << std::endl;
            return 1;
        }
        q[static_cast<size_t>(i)] = degrees * kDegToRad;
    }

    int iterations = 1;
    if (args.size() >= 10) {
        iterations = strutil::parseIntOr(args[9], 0);
        if (iterations < 1) iterations = 1;
    }

    long long poseNs = 0;
    long long ikNs = 0;
    long long collisionNs = 0;
    int solutionCount = 0;
    std::vector<RobotCollisionPair> collisions;
    std::array<std::array<double, 6>, 12> solutions{};

    for (int iteration = 0; iteration < iterations; ++iteration) {
        auto mark = std::chrono::steady_clock::now();
        const auto elapsedNs = [&mark]() {
            const auto now = std::chrono::steady_clock::now();
            const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now - mark).count();
            mark = now;
            return static_cast<long long>(ns);
        };
        poseController.setJoints(q);
        poseNs += elapsedNs();

        std::string ikError;
        solutionCount = poseController.toolPoseSolutions(
            poseController.toolPose(),
            solutions.data(),
            static_cast<int>(solutions.size()),
            &ikError);
        ikNs += elapsedNs();

        collisions = collisionModel.selfCollisions(q);
        collisionNs += elapsedNs();
    }

    auto ms = [iterations](long long ns) {
        return static_cast<double>(ns) / 1000000.0 / static_cast<double>(iterations);
    };

    std::cout << "collision-pose"
              << " iterations=" << iterations
              << " pose_ms=" << ms(poseNs)
              << " ik_ms=" << ms(ikNs)
              << " collision_ms=" << ms(collisionNs)
              << " ik_solutions=" << solutionCount
              << " collision_pairs=" << collisions.size()
              << std::endl;
    if (!collisions.empty()) {
        std::cout << "pairs=";
        for (size_t i = 0; i < collisions.size(); ++i) {
            if (i) std::cout << ",";
            std::cout << collisions[i].linkA << "-" << collisions[i].linkB;
        }
        std::cout << std::endl;
    }
    return 0;
}

int dumpPackageProgramsCommand(const std::vector<std::string>& args) {
    if (args.size() < 3) {
        std::cerr << "Usage: RobotSimulator.exe --dump-package-programs <robot.zip>" << std::endl;
        return 2;
    }
    RobotInstance probe;
    const size_t bundled = loadPackageProgramsInto(probe, args[2]);
    if (!probe.statusText.empty()) {
        std::cerr << probe.statusText << std::endl;
        return 1;
    }
    // A package that ships none leaves the instance's own empty program, which is one entry with no
    // instructions; saying so is more use than printing nothing. The count below is what the package
    // supplied, not the length of that list, so zero reads as zero.
    for (size_t i = 0; i < probe.programs.size(); ++i) {
        const RobotProgram& program = probe.programs[i];
        std::cout << "program " << i << " name=" << program.name
                  << " instructions=" << (program.root ? program.root->children.size() : 0)
                  << std::endl;
    }
    std::cout << "bundled programs: " << bundled << std::endl;
    return 0;
}
