#include "ViewerBridge.h"
#include "StringUtil.h"

#include "AccessoryGeometry.h"
#include "ConveyorPhysics.h"
#include "ConveyorRuntime.h"
#include "DragChainPhysics.h"
#include "FirmwareProgram.h"
#include "HardwareIo.h"
#include "JsonCompat.h"
#include "LiveRunDriver.h"
#include "MasteringIo.h"
#include "OrientationFormat.h"
#include "PlacementWindows.h"
#include "ProgramEditing.h"
#include "RobotRuntime.h"
#include "SceneMath.h"
#include "StationParameterLinks.h"
#include "StationSceneLoad.h"
#include "TrajectoryAndLiveRun.h"
#include "UiFormHelpers.h"
#include "WebFiles.h"

#include "imgui.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

CadNode* selectedStationNode() {
    const int index = g_scene.stationSelectionIndex;
    if (index < 0) return nullptr;
    switch (g_scene.stationSelectionKind) {
    case StationSelectionKind::Robot:
        if (index < static_cast<int>(g_scene.robots.size()) && g_scene.robots[index]) {
            return g_scene.robots[index]->poseController.robotNode();
        }
        break;
    case StationSelectionKind::Mechanism:
        if (index < static_cast<int>(g_scene.gantries.size())) {
            return g_scene.gantries[index].gantryNode();
        }
        break;
    case StationSelectionKind::Accessory:
        if (index < static_cast<int>(g_scene.station.accessories.size())) {
            return g_scene.station.accessories[index].node;
        }
        break;
    case StationSelectionKind::None:
        break;
    }
    return nullptr;
}


void selectStationObject(StationSelectionKind kind, int index) {
    g_scene.stationSelectionKind = kind;
    g_scene.stationSelectionIndex = index;
    if (kind == StationSelectionKind::Robot && index >= 0 &&
        index < static_cast<int>(g_scene.robots.size())) {
        g_scene.view.robot = index;
    }
}

// Drains every open connection, once a frame, whichever arm the window is showing. Not left to the
// Hardware IO panel: that panel is drawn for one arm and only in the robot view, so polling inside
// it would leave every other connected arm's status frames and acks unread.
void pollAllHardware() {
    for (const std::unique_ptr<RobotInstance>& robot : g_scene.robots) {
        if (robot && robot->hardware.isConnected()) robot->hardware.poll();
    }
}

// The marker an arm carries when it is driving a real machine, drawn wherever an arm is named.
void drawHardwareMarker(const RobotInstance& robot) {
    if (!robot.hardware.isConnected()) return;
    ImGui::SameLine();
    ImGui::TextColored(ImVec4(0.85f, 0.6f, 0.15f, 1.0f), "[%s]", robot.hardware.connectedPort().c_str());
}

// Camera travel between contexts. The tween owns what the viewer shows while it runs; the two
// ViewStates keep holding where each view actually is, so leaving mid-flight cannot save a halfway
// camera as though it were somewhere the operator chose to stand.
float shortestAngleDelta(float from, float to) {
    float delta = to - from;
    while (delta > 180.0f) delta -= 360.0f;
    while (delta < -180.0f) delta += 360.0f;
    return delta;
}

MeshRobotViewer::CameraState lerpCamera(const MeshRobotViewer::CameraState& a,
                                        const MeshRobotViewer::CameraState& b, float t) {
    const float e = t * t * (3.0f - 2.0f * t);
    MeshRobotViewer::CameraState out;
    out.center = a.center + (b.center - a.center) * e;
    out.distance = a.distance + (b.distance - a.distance) * e;
    // Shortest way round, or turning from -170 to 170 degrees would swing almost all the way back.
    out.yaw = a.yaw + shortestAngleDelta(a.yaw, b.yaw) * e;
    out.pitch = a.pitch + (b.pitch - a.pitch) * e;
    out.framed = true;
    return out;
}

void settleCameraTween() {
    if (!g_scene.cameraTween.active()) return;
    if (g_scene.viewer) g_scene.viewer->setCamera(g_scene.cameraTween.to);
    g_scene.cameraTween = CameraTween();
}

void cancelCameraTween() { g_scene.cameraTween = CameraTween(); }

void beginCameraTween(const MeshRobotViewer::CameraState& from,
                      const MeshRobotViewer::CameraState& to) {
    if (!g_scene.viewer) return;
    g_scene.cameraTween.from = from;
    g_scene.cameraTween.to = to;
    g_scene.cameraTween.elapsed = 0.0f;
    g_scene.cameraTween.duration = 0.28f;
    g_scene.viewer->setCamera(from);
}

void stepCameraTween(float deltaSeconds) {
    CameraTween& tween = g_scene.cameraTween;
    if (!tween.active() || !g_scene.viewer) return;
    tween.elapsed += deltaSeconds;
    if (tween.elapsed >= tween.duration) {
        settleCameraTween();
        return;
    }
    g_scene.viewer->setCamera(lerpCamera(tween.from, tween.to, tween.elapsed / tween.duration));
}


// Refreshes the IK branch list and the collision summary. Recomputed only when the pose moved:
// this runs every frame, and toolPoseSolutions runs IK while selfCollisions walks the hulls.
// Every arm, not just the selected one, because each carries its own tool gimbal at its own target.
void refreshPoseDerivedReadoutsIfMoved() {
    if (!g_scene.viewer) return;
    const size_t activeIndex = static_cast<size_t>(g_scene.activeRobotIndex());
    for (size_t index = 0; index < g_scene.robots.size(); ++index) {
        RobotInstance* robot = g_scene.robots[index].get();
        if (!robot) continue;
        const std::array<double, 6> joints = robot->poseController.joints();
        bool moved = !robot->readoutJointsValid;
        if (robot->readoutJointsValid) {
            for (size_t i = 0; i < joints.size(); ++i) {
                // A tolerance well below display precision, so playback's continuous motion still
                // refreshes but floating-point noise on an unchanged pose does not.
                if (std::abs(joints[i] - robot->readoutJoints[i]) > 1.0e-9) moved = true;
            }
        }
        if (!moved) continue;
        robot->readoutJoints = joints;
        robot->readoutJointsValid = true;

        // The gimbal is tied to the flange, so it follows the robot wherever the pose came from:
        // hardware feedback, a live run, program playback, a timeline scrub, a joint slider or Home.
        if (robot->poseController.isBound()) {
            robot->toolTargetPose = robot->poseController.toolPose();
            // WPR fields are offsets from this base pose.
            robot->toolWprBasePose = robot->toolTargetPose;
            robot->toolWprDegrees = {{0.0, 0.0, 0.0}};
        }

        // The IK branch list, though, is genuinely about the arm whose panel is open: it is a dozen
        // inverse solves and a sort, and computing it for arms nobody is looking at would be work
        // spent on a readout that is not on screen.
        if (index == activeIndex) refreshPoseDerivedReadouts();
    }
}

void refreshPoseDerivedReadouts() {
    activeRobot().toolSolutions.clear();
    activeRobot().toolSolutionLabels.clear();

    std::array<std::array<double, 6>, RobotPoseController::kMaxToolPoseSolutions> solutions{};
    const int count = activeRobot().poseController.toolPoseSolutions(
        activeRobot().toolTargetPose, solutions.data(), RobotPoseController::kMaxToolPoseSolutions, nullptr);
    const std::array<double, 6> current = activeRobot().poseController.joints();
    double bestDistance = std::numeric_limits<double>::max();
    for (int i = 0; i < count; ++i) {
        const std::array<double, 6>& solution = solutions[static_cast<size_t>(i)];
        activeRobot().toolSolutions.push_back(solution);

        const std::string elbow = solution[2] >= 0.0 ? std::string("elbow up") : std::string("elbow down");
        const std::string wrist = solution[4] >= 0.0 ? std::string("wrist +") : std::string("wrist -");
        const std::string base = std::abs(solution[0]) <= (90.0 * kDegToRad) ? std::string("front") : std::string("back");
        std::string label = strutil::format("%1, %2, %3  |").arg(base).arg(elbow).arg(wrist);

        double distance = 0.0;
        for (int joint = 0; joint < 6; ++joint) {
            const double degrees = solution[static_cast<size_t>(joint)] * kRadToDeg;
            label += strutil::format(" %1").arg(degrees, 0, 'f', 1);
            distance += std::abs(degrees - current[static_cast<size_t>(joint)] * kRadToDeg);
        }
        // The pose the robot is actually in, so the list shows where you are, not just where
        // you could go. Threshold is well below what the one-decimal display can show.
        if (distance < 0.05) label += std::string("   (current)");
        activeRobot().toolSolutionLabels.push_back(label);
        if (distance < bestDistance) {
            bestDistance = distance;
            activeRobot().selectedToolSolution = i;
        }
    }

    // The arm these indices belong to travels with them. RobotCollisionModel numbers links within
    // one robot, and the viewer would otherwise resolve them against the whole tree - which in a
    // two-arm cell means painting the other robot's links red.
    CadNode* const collidingRobot = activeRobot().poseController.robotNode();
    const std::vector<RobotCollisionPair> collisions = activeRobot().collisionModel.selfCollisions(current);
    if (collisions.empty()) {
        activeRobot().collisionText = "Collision: clear";
        g_scene.viewer->setCollidingLinkIndices({}, collidingRobot);
    } else {
        std::vector<std::string> pairs;
        std::vector<int> linkIndices;
        for (const RobotCollisionPair& pair : collisions) {
            pairs << strutil::format("%1-%2").arg(pair.linkA).arg(pair.linkB);
            linkIndices.push_back(pair.linkA);
            linkIndices.push_back(pair.linkB);
        }
        activeRobot().collisionText = "Collision: " + strutil::join(pairs, ", ");
        g_scene.viewer->setCollidingLinkIndices(linkIndices, collidingRobot);
    }
}

void refreshProgramPathPreview() {
    if (!g_scene.viewer) return;
    // Every arm, not just the selected one. Where an arm's program takes it is a fact about the cell,
    // and the reason to look at two paths at once is usually to see whether they cross.
    std::vector<std::vector<Vec3f>> paths;
    for (size_t robotIndex = 0; robotIndex < g_scene.robots.size(); ++robotIndex) {
        const std::unique_ptr<RobotInstance>& robot = g_scene.robots[robotIndex];
        const RobotKinematicSnapshot snapshot = robot->poseController.kinematicSnapshot();
        if (!snapshot.valid) continue;
        std::vector<Vec3f> points;
        const int gantryIndex = linkedGantryIndexForRobot(robotIndex);
        const CadVec3 gantryAxis = linkedGantryWorldAxis(gantryIndex);
        const double currentJ7 = gantryIndex >= 0
            ? g_scene.gantries[static_cast<size_t>(gantryIndex)].positionMm() : 0.0;
        double modalJ7 = currentJ7;
        for (const auto& child : robot->program().root->children) {
            if (!child) continue;
            const std::array<double, 6>* target = targetJointsForNode(*child);
            if (!target) continue;
            externalAxisTargetForNode(*child, &modalJ7);
            const CadTransform pose = worldToolPoseFromKinematicSnapshot(snapshot, *target);
            const double offset = modalJ7 - currentJ7;
            points.emplace_back(static_cast<float>(pose.values[3] + gantryAxis.x * offset),
                                static_cast<float>(pose.values[7] + gantryAxis.y * offset),
                                static_cast<float>(pose.values[11] + gantryAxis.z * offset));
        }
        if (points.size() >= 2) paths.push_back(std::move(points));
    }
    g_scene.viewer->setPathPreviewPoints(paths);
}

void refreshSimPathPreview() {
    if (!g_scene.viewer) return;
    std::vector<std::vector<Vec3f>> paths;
    for (size_t robotIndex = 0; robotIndex < g_scene.robots.size(); ++robotIndex) {
        const std::unique_ptr<RobotInstance>& robot = g_scene.robots[robotIndex];
        if (robot->timelineSamples.size() < 2) continue;
        std::vector<Vec3f> points;
        points.reserve(robot->timelineSamples.size());
        const CadTransform baseWorld = robot->poseController.baseWorldTransform();
        const int gantryIndex = linkedGantryIndexForRobot(robotIndex);
        const CadVec3 gantryAxis = linkedGantryWorldAxis(gantryIndex);
        const double currentJ7 = gantryIndex >= 0
            ? g_scene.gantries[static_cast<size_t>(gantryIndex)].positionMm() : 0.0;
        for (const TimelineSample& sample : robot->timelineSamples) {
            const CadTransform pose = baseWorld * sample.tcpPose;
            const double offset = sample.externalAxisValid
                ? sample.externalAxisPositionMm - currentJ7 : 0.0;
            points.emplace_back(static_cast<float>(pose.values[3] + gantryAxis.x * offset),
                                static_cast<float>(pose.values[7] + gantryAxis.y * offset),
                                static_cast<float>(pose.values[11] + gantryAxis.z * offset));
        }
        paths.push_back(std::move(points));
    }
    g_scene.viewer->setSimPathPreviewPoints(paths);
}

std::vector<MeshRobotViewer::ConveyorDirectionGuide> buildConveyorDirectionGuides(
    const StationDocument& station) {
    std::vector<MeshRobotViewer::ConveyorDirectionGuide> guides;
    guides.reserve(station.accessories.size());
    const auto portConnected = [&](const std::string& accessoryId, const char* port) {
        return std::any_of(
            station.parameterLinks.begin(), station.parameterLinks.end(),
            [&](const StationParameterLink& link) {
                if (link.channel != "deckHeight") return false;
                return (link.a.accessoryId == accessoryId && link.a.port == port) ||
                       (link.b.accessoryId == accessoryId && link.b.port == port);
            });
    };
    for (const StationAccessoryInstance& accessory : station.accessories) {
        const TransformNodeData* parameters = conveyorParameters(accessory.node);
        if (!parameters) continue;
        MeshRobotViewer::ConveyorDirectionGuide guide;
        guide.startConnected = portConnected(accessory.id, "start");
        guide.endConnected = portConnected(accessory.id, "end");
        const int segments = std::max(
            1, std::min(160, static_cast<int>(
                std::ceil(conveyorPathLengthMm(*parameters) / 220.0))));
        guide.points.reserve(static_cast<size_t>(segments) + 1);
        const CadTransform world =
            parentWorldTransformOf(accessory.node) * accessory.node->loc;
        const double guideLiftMm = conveyorSurfaceOffsetMm(*parameters) + 55.0;
        for (int segment = 0; segment <= segments; ++segment) {
            ConveyorPathPose pose = conveyorPathPoseAt(
                *parameters, static_cast<double>(segment) / segments);
            pose.y += static_cast<float>(guideLiftMm);
            const CadVec3 point = conveyorTransformPoint(world, pose);
            guide.points.emplace_back(static_cast<float>(point.x),
                                      static_cast<float>(point.y),
                                      static_cast<float>(point.z));
        }
        guides.push_back(std::move(guide));
    }
    return guides;
}

CadNode* placementNodeFor(const RobotInstance& robot);

// Pushes the display toggles and gimbal pose into the pose controller and renderer. Called
// every frame: immediate mode has no change notifications, and these are cheap setters.
void applySceneViewState() {
    if (!g_scene.viewer) return;
    const ViewState& viewState = g_scene.activeViewState();

    for (const std::unique_ptr<RobotInstance>& robot : g_scene.robots) {
        robot->poseController.setVisualMeshesVisible(viewState.showVisualMeshes);
        robot->poseController.setJointAxesVisible(viewState.showJointAxes);
        robot->poseController.setHullOverlaysVisible(viewState.showHullOverlay);
    }
    g_scene.viewer->setPathPreviewVisible(viewState.showPathPreview);
    g_scene.viewer->setSimPathPreviewVisible(viewState.showSimPath);
    g_scene.viewer->setDragChainPivotsVisible(viewState.showDragChainPivots);
    g_scene.viewer->setGantryHullOverlaysVisible(viewState.showHullOverlay);
    g_scene.viewer->setMountingHolesVisible(viewState.showMountingHoles);
    g_scene.viewer->setConveyorDirectionsVisible(viewState.showConveyorDirections);
    if (viewState.showConveyorDirections) {
        g_scene.viewer->setConveyorDirectionGuides(
            buildConveyorDirectionGuides(g_scene.station));
    }
    const CadNode* selectedOutline = nullptr;
    if (g_scene.view.kind == ViewTarget::Kind::Sim) {
        if (g_scene.stationMode == StationMode::Program && !g_scene.robots.empty()) {
            selectedOutline = activeRobot().poseController.robotNode();
        } else if (g_scene.stationMode == StationMode::Layout) {
            selectedOutline = selectedStationNode();
        }
    }
    g_scene.viewer->setSelectionOutlineRoot(selectedOutline);

    // Layout placement gives one transform gimbal to the selected station object. Outside that
    // mode the same renderer handles robot tool gimbals as before. The mapping beside each pose is
    // explicit because an accessory and a robot can both occupy index zero in their own lists.
    std::vector<CadTransform> gimbalPoses;
    g_scene.gimbalTargets.clear();
    const bool movingStationObject = g_scene.view.kind == ViewTarget::Kind::Sim &&
                                     g_scene.stationMode == StationMode::Layout &&
                                     g_scene.moveObjects && !g_libraryPlacement.active();
    if (movingStationObject) {
        CadNode* placementNode = nullptr;
        RobotInstance* robot = nullptr;
        if (g_scene.stationSelectionKind == StationSelectionKind::Robot) {
            const int index = g_scene.stationSelectionIndex;
            if (index >= 0 && index < static_cast<int>(g_scene.robots.size())) {
                robot = g_scene.robots[index].get();
                placementNode = placementNodeFor(*robot);
            }
        } else {
            placementNode = selectedStationNode();
        }
        if (placementNode) {
            gimbalPoses.push_back(parentWorldTransformOf(placementNode) * placementNode->loc);
            g_scene.gimbalTargets.push_back({placementNode, robot, true});
        }
    } else if (viewState.showToolGimbal) {
        for (const std::unique_ptr<RobotInstance>& robot : g_scene.robots) {
            if (!robot || !robot->poseController.isBound()) continue;
            gimbalPoses.push_back(robot->poseController.baseWorldTransform() *
                                  robot->toolTargetPose);
            int programInstruction = -1;
            const bool programWorkspace = g_scene.view.kind == ViewTarget::Kind::Robot ||
                (g_scene.view.kind == ViewTarget::Kind::Sim &&
                 g_scene.stationMode == StationMode::Program);
            if (programWorkspace && robot.get() == &activeRobot() &&
                editableProgramMoveAt(*robot, robot->selectedInstruction)) {
                programInstruction = robot->selectedInstruction;
            }
            g_scene.gimbalTargets.push_back(
                {nullptr, robot.get(), false, programInstruction});
        }
    }
    g_scene.viewer->setGimbalsVisible(!gimbalPoses.empty());
    g_scene.viewer->setGimbalPoses(gimbalPoses);
}


void switchView(ViewTarget::Kind kind, int robotIndex) {
    if (kind == ViewTarget::Kind::Robot &&
        (robotIndex < 0 || robotIndex >= static_cast<int>(g_scene.robots.size()))) {
        return;
    }
    if (!g_scene.viewer) {
        g_scene.view.kind = kind;
        if (kind == ViewTarget::Kind::Robot) g_scene.view.robot = robotIndex;
        return;
    }

    settleCameraTween();
    const MeshRobotViewer::CameraState leavingFrom = g_scene.viewer->camera();
    g_scene.activeViewState().camera = leavingFrom;

    g_scene.view.kind = kind;
    if (kind == ViewTarget::Kind::Robot) g_scene.view.robot = robotIndex;

    ViewState& incoming = g_scene.activeViewState();
    const int wanted = kind == ViewTarget::Kind::Sim ? -1 : robotIndex;
    if (!incoming.camera.framed || incoming.framedRobot != wanted) {
        // reframe writes straight to the viewer, so the result is read back as the destination and
        // the viewer is then put back where it was for the move to start from.
        if (kind == ViewTarget::Kind::Sim) {
            g_scene.viewer->reframeCamera();
        } else {
            g_scene.viewer->reframeCameraOn(activeRobot().poseController.robotNode());
        }
        incoming.framedRobot = wanted;
        incoming.camera = g_scene.viewer->camera();
    }
    beginCameraTween(leavingFrom, incoming.camera);
}

// Enters the robot view for one arm. The pose readouts are keyed to the instance they were computed
// for, so switching has to invalidate them or the new arm's solution list and collision line would
// be the previous arm's until something happened to move a joint.
void enterRobotView(int robotIndex) {
    if (robotIndex < 0 || robotIndex >= static_cast<int>(g_scene.robots.size())) return;
    switchView(ViewTarget::Kind::Robot, robotIndex);
    activeRobot().readoutJointsValid = false;
    refreshPoseDerivedReadouts();
    refreshProgramPathPreview();
    refreshSimPathPreview();
}

void exitToStationView() {
    switchView(ViewTarget::Kind::Sim, g_scene.view.robot);
}

StationDocument stationDocumentFromScene() {
    StationDocument document = g_scene.station;
    document.conveyorSimulationMode =
        g_scene.defaultConveyorMode == ConveyorSimulationMode::PhysX ? "physx" : "logical";
    document.mechanisms.resize(g_scene.gantries.size());
    document.robots.resize(g_scene.robots.size());
    if (document.name.empty()) {
        document.name = g_scene.root && !g_scene.root->name.empty() ? g_scene.root->name : "Station";
    }
    // The ids the loaded station already had are claimed before any is derived below, so an arm that
    // arrived without one cannot be handed an id another arm is already using - two arms of the same
    // model share a name, and a shared id is a shared instances/<id>/config.json.
    StationInstanceIds instanceIds;
    for (const StationRobotInstance& entry : document.robots) instanceIds.reserve(entry.id);

    StationInstanceIds mechanismIds;
    for (const StationMechanismInstance& entry : document.mechanisms) mechanismIds.reserve(entry.id);
    for (size_t i = 0; i < g_scene.gantries.size(); ++i) {
        const GantryPoseController& gantry = g_scene.gantries[i];
        StationMechanismInstance& entry = document.mechanisms[i];
        const CadNode* node = gantry.gantryNode();
        if (node) {
            if (!node->name.empty()) entry.name = node->name;
            entry.placement = node->loc;
        }
        entry.positionMm = gantry.positionMm();
        if (entry.id.empty()) entry.id = mechanismIds.derive(entry.name, static_cast<int>(i));
    }

    StationInstanceIds accessoryIds;
    for (const StationAccessoryInstance& entry : document.accessories) {
        accessoryIds.reserve(entry.id);
    }
    for (size_t i = 0; i < document.accessories.size(); ++i) {
        StationAccessoryInstance& entry = document.accessories[i];
        if (entry.node) {
            if (!entry.node->name.empty()) entry.name = entry.node->name;
            entry.placement = entry.node->loc;
            if (const TransformNodeData* transform = entry.node->asTransform()) {
                if (transform->hasParametricAccessory()) {
                    entry.parameters = transform->accessoryParametersJson();
                }
            }
        }
        if (entry.id.empty()) {
            entry.id = accessoryIds.derive(entry.name, static_cast<int>(i));
        }
    }

    for (size_t i = 0; i < g_scene.robots.size(); ++i) {
        const RobotInstance& robot = *g_scene.robots[i];
        StationRobotInstance& entry = document.robots[i];
        const CadNode* node = robot.poseController.robotNode();
        if (node) {
            if (!node->name.empty()) entry.name = node->name;
            // Off the base frame, which is where the placement lives once the arm hangs under one.
            // Reading the arm's own loc would write out an identity and flatten the cell on reload.
            entry.placement = robotBaseFrameNode(node)->loc;
        }
        if (entry.id.empty()) entry.id = instanceIds.derive(entry.name, static_cast<int>(i));
        if (entry.packageRef.empty()) {
            // A bare robot package was opened, so there is no reference yet. The station is written
            // beside the package it came from and names it, which is the only thing that can be
            // true without copying geometry around.
            entry.packageRef = std::filesystem::path(g_scene.packageLabel).filename().string();
            entry.resolvedPackagePath = g_scene.packageLabel;
        }
        entry.config = stationConfigForInstance(robot);
    }
    return document;
}

void saveStationToFile() {
#ifdef __EMSCRIPTEN__
    activeRobot().hardwareStatus = "Saving a station needs a filesystem; not available in the browser.";
#else
    const std::string path = runFileDialog("Save station", true, kStationFilter, L"zip");
    if (path.empty()) return;
    std::string errorMessage;
    StationDocument document = stationDocumentFromScene();

    std::vector<std::string> promoted;
    if (g_scene.saveStationUsingBuiltins) promoteBuiltinPackageRefs(&document, &promoted);

    if (!saveStationPackage(path, document, g_scene.stationSource, &errorMessage)) {
        activeRobot().hardwareStatus = "Save station failed: " + errorMessage;
        return;
    }
    // The saved file becomes the one a later save copies embedded packages out of, so saving twice
    // in a row does not need the original still to be there.
    g_scene.stationSource = path;
    g_scene.packageLabel = path;
    g_scene.station = document;
    activeRobot().hardwareStatus = "Station saved: " + path;
    if (!promoted.empty()) {
        activeRobot().hardwareStatus += strutil::format(" (%1 asset(s) now reference built-ins: %2)")
                                      .arg(promoted.size())
                                      .arg(strutil::join(promoted, ", "));
    }
#endif
}

// The node an arm's placement actually lives on: its base frame where it has one, and the arm
// itself where it does not - a bare package whose robot node is the root of its tree has no frame
// to put one on.
CadNode* placementNodeFor(const RobotInstance& robot) {
    if (CadNode* frame = baseFrameNodeFor(robot)) return frame;
    return const_cast<CadNode*>(robot.poseController.robotNode());
}


// The conveyor runtime's one renderer touchpoint, kept out of ConveyorRuntime.cpp so that TU
// stays free of MeshRobotViewer calls.
static uint64_t g_conveyorSceneDirtyMarks = 0;

void conveyorMarkSceneDirty() {
    ++g_conveyorSceneDirtyMarks;
    if (g_scene.viewer) g_scene.viewer->markCacheDirty();
}

uint64_t conveyorSceneDirtyMarkCount() { return g_conveyorSceneDirtyMarks; }
