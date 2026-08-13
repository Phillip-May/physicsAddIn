#include "StationSceneLoad.h"
#include "StringUtil.h"

#include "AccessoryBuilders.h"
#include "AccessoryGeometry.h"
#include "CadNodePackage.h"
#include "ConveyorPhysics.h"
#include "ConveyorRuntime.h"
#include "DragChainPhysics.h"
#include "FirmwareProgram.h"
#include "HardwareIo.h"
#include "JsonCompat.h"
#include "LibraryPlacement.h"
#include "LiveRunDriver.h"
#include "ProgramEditing.h"
#include "MasteringIo.h"
#include "MeshRobotViewer.h"
#include "ProgramTextIo.h"
#include "RobotRuntime.h"
#include "SceneMath.h"
#include "TrajectoryAndLiveRun.h"
#include "ViewerBridge.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#if defined(_WIN32) && !defined(__EMSCRIPTEN__)
#include <commdlg.h>
#include <windows.h>
#endif

std::string runPackageDialog();

// Programs shipped inside the robot package, under programs/, loaded into the arm as it binds. They
// arrive as ordinary programs in the arm's list: editable, renameable, and saved out with the station
// like any other. The default empty program the instance starts with is dropped only if the package
// actually supplied one.
size_t loadPackageProgramsInto(RobotInstance& robot, const std::string& packageFile) {
    if (!cadPackageIsZip(packageFile)) return 0;
    std::vector<std::string> entries = listCadPackageEntries(packageFile);
    std::sort(entries.begin(), entries.end());

    std::vector<RobotProgram> loaded;
    for (const std::string& entry : entries) {
        if (entry.rfind("programs/", 0) != 0) continue;
        if (!strutil::endsWith(entry, ".txt")) continue;
        std::string bytes;
        if (!readCadPackageEntry(packageFile, entry, &bytes)) continue;

        RobotProgram program;
        program.root = std::make_unique<RobotProgramNode>();
        program.root->type = RobotProgramNodeType::Root;
        std::istringstream stream(bytes);
        std::string parseError;
        if (!loadProgramTextFromStream(stream, program.root.get(), &parseError)) {
            robot.statusText = strutil::format("%1: %2").arg(entry).arg(parseError).str();
            continue;
        }
        std::string name = std::filesystem::path(entry).filename().string();
        for (const std::string& suffix : {std::string(".robotprog.txt"), std::string(".txt")}) {
            if (strutil::endsWith(name, suffix)) {
                name = name.substr(0, name.size() - suffix.size());
                break;
            }
        }
        program.name = name;
        loaded.push_back(std::move(program));
    }
    if (loaded.empty()) return 0;

    const size_t count = loaded.size();
    robot.programs = std::move(loaded);
    robot.selectedProgram = 0;
    robot.selectedInstruction = -1;
    return count;
}


int linkedGantryIndexForRobot(size_t robotIndex) {
    if (robotIndex >= g_scene.station.robots.size()) return -1;
    const std::string& link = g_scene.station.robots[robotIndex].motionLink;
    if (link.empty()) return -1;
    for (size_t i = 0; i < g_scene.station.mechanisms.size() &&
                       i < g_scene.gantries.size(); ++i) {
        if (g_scene.station.mechanisms[i].motionLink == link) return static_cast<int>(i);
    }
    return -1;
}

CadVec3 linkedGantryWorldAxis(int gantryIndex) {
    if (gantryIndex < 0 || gantryIndex >= static_cast<int>(g_scene.gantries.size())) {
        return CadVec3();
    }
    const GantryPoseController& controller = g_scene.gantries[static_cast<size_t>(gantryIndex)];
    const GantryMechanismData* data = controller.gantryData();
    CadNode* node = controller.gantryNode();
    if (!data || !node) return CadVec3();
    const CadTransform world = parentWorldTransformOf(node) * node->loc;
    const CadVec3 axis = rotate(world, CadVec3(data->axisOfTravel.x, data->axisOfTravel.y,
                                               data->axisOfTravel.z));
    const double length = std::sqrt(axis.x * axis.x + axis.y * axis.y + axis.z * axis.z);
    return length > 1.0e-12
        ? CadVec3(axis.x / length, axis.y / length, axis.z / length) : CadVec3();
}

void configureRobotExternalAxis(RobotInstance& robot) {
    const size_t robotIndex = liveRunArmIndexOf(robot);
    const int gantryIndex = linkedGantryIndexForRobot(robotIndex);
    robot.simulator.configureExternalAxis(
        gantryIndex >= 0,
        gantryIndex >= 0 ? g_scene.gantries[static_cast<size_t>(gantryIndex)].positionMm() : 0.0);
}

size_t loadStationProgramsInto(RobotInstance& robot,
                               const std::vector<StationRobotInstance::ProgramText>& sources) {
    std::vector<RobotProgram> loaded;
    for (const StationRobotInstance::ProgramText& source : sources) {
        RobotProgram program;
        program.name = source.name.empty() ? std::string("Program") : source.name;
        program.root = std::make_unique<RobotProgramNode>();
        program.root->type = RobotProgramNodeType::Root;
        std::istringstream stream(source.text);
        std::string parseError;
        if (!loadProgramTextFromStream(stream, program.root.get(), &parseError)) {
            robot.statusText = program.name + ": " + parseError;
            continue;
        }
        loaded.push_back(std::move(program));
    }
    if (loaded.empty()) return 0;
    robot.programs = std::move(loaded);
    robot.selectedProgram = 0;
    robot.selectedInstruction = -1;
    return robot.programs.size();
}

size_t probeStationInstancePrograms(const std::string& resolvedPackagePath,
                                    const std::vector<StationRobotInstance::ProgramText>& programs) {
    RobotInstance probe;
    const size_t bundled = resolvedPackagePath.empty()
        ? 0 : loadPackageProgramsInto(probe, resolvedPackagePath);
    const size_t station = loadStationProgramsInto(probe, programs);
    return station ? station : bundled;
}


void loadPackageIntoScene(const std::string& packageFile) {
    // Loading replaces the entire scene tree.  Do not leave a library preview pointing into the
    // old tree if the operator opens another package while a drag is active.
    cancelLibraryRobotPlacement();
    // Runtime products belong to the station being replaced and are deliberately not serialized.
    g_scene.conveyorPhysics.stopAsync();
    clearConveyorWorkpieces();
    g_scene.conveyorTimeAccumulatorSeconds = 0.0;
    g_scene.liveRunDriver.setPaused(false);
    g_scene.stationRunState = StationRunState::Stopped;
    g_scene.stationSimulationStatus.clear();
    std::string errorMessage;
    std::shared_ptr<CadNode> loaded;
    StationDocument station;
    const bool emptyScene = packageFile.empty();
    const bool isStation = emptyScene || fileIsStationPackage(packageFile);
    if (emptyScene) {
        loaded = std::make_shared<CadNode>();
        loaded->name = "Untitled station";
        loaded->type = CadNodeType::Custom;
        station.name = loaded->name;
    } else if (isStation) {
        loaded = loadStationPackage(packageFile, &station, &errorMessage);
        if (!loaded) {
            g_scene.error = errorMessage.empty() ? std::string("Failed to load station.") : errorMessage;
            return;
        }
        // Every arm is validated, including arms nested beneath a mechanism carriage.
        for (CadNode* robotNode : collectRobotNodes(loaded.get())) {
            if (!validateRobotPackage(robotNode, &errorMessage)) {
                g_scene.error = "Station robot '" + robotNode->name + "': " + errorMessage;
                return;
            }
        }
    } else {
        loaded = loadCadNodePackage(packageFile, &errorMessage);
        if (!loaded || !validateRobotPackage(loaded.get(), &errorMessage)) {
            g_scene.error = errorMessage.empty() ? std::string("Failed to load package.") : errorMessage;
            return;
        }
    }

    // Each arm gets its base frame before anything binds to it or reads a placement off it, so
    // there is no window in which half the code sees the placement on the arm and half on the
    // frame. The arms themselves do not move, so the pointers collected below are the ones the
    // pose controllers bind to either way.
    ensureRobotBaseFrames(loaded.get());

    const std::vector<CadNode*> robotNodes = collectRobotNodes(loaded.get());
    if (robotNodes.empty() && !isStation) {
        g_scene.error = "Package does not contain an OPW6Robot node.";
        return;
    }
    const std::vector<CadNode*> gantryNodes = collectGantryMechanismNodes(loaded.get());
    if (isStation && gantryNodes.size() != station.mechanisms.size()) {
        g_scene.error = "Station mechanism count does not match the loaded gantry tree.";
        return;
    }
    std::vector<GantryPoseController> gantryControllers(gantryNodes.size());
    for (size_t i = 0; i < gantryNodes.size(); ++i) {
        if (!gantryControllers[i].bindToGantry(gantryNodes[i], &errorMessage)) {
            g_scene.error = errorMessage;
            return;
        }
    }
    const std::vector<CadNode*> dragChainNodes = collectDragChainMechanismNodes(loaded.get());
    std::vector<DragChainPoseController> dragChainControllers(dragChainNodes.size());
    for (size_t i = 0; i < dragChainNodes.size(); ++i) {
        if (!dragChainControllers[i].bindToDragChain(dragChainNodes[i], &errorMessage)) {
            g_scene.error = errorMessage;
            return;
        }
    }
    std::vector<DragChainPhysics> dragChainPhysics;
    dragChainPhysics.reserve(dragChainNodes.size());
    for (CadNode* chainNode : dragChainNodes) {
        DragChainPhysics physics;
        if (!physics.bind(chainNode, &errorMessage)) {
            g_scene.error = errorMessage;
            return;
        }
        dragChainPhysics.push_back(std::move(physics));
    }

    {
        auto control = g_scene.liveRunDriver.lockForControl();
        for (const std::unique_ptr<RobotInstance>& robot : g_scene.robots) {
            if (robot) robot->simulator.endLiveRun();
        }
        g_scene.liveRunDriver.setArms({});
    }

    // Every arm's build, not only the arms about to be popped. A kept instance is rebound below and
    // has loadPackageProgramsInto move a new program list over its old one; the worker holds the root
    // of a tree in that list and walks its children from flattenProgram, so a build still running
    // across the load reads freed nodes.
    for (const std::unique_ptr<RobotInstance>& robot : g_scene.robots) {
        if (robot) cancelTrajectoryBuild(*robot);
    }
    while (g_scene.robots.size() > robotNodes.size()) {
        g_scene.robots.pop_back();
    }
    while (g_scene.robots.size() < robotNodes.size()) {
        g_scene.robots.push_back(std::make_unique<RobotInstance>());
    }
    g_scene.view.robot = 0;
    for (const std::unique_ptr<RobotInstance>& robot : g_scene.robots) {
        if (robot) robot->liveRun = RobotProgramSimulator::LiveRunState();
    }

    for (size_t i = 0; i < robotNodes.size(); ++i) {
        RobotInstance& robot = *g_scene.robots[i];
        if (!robot.poseController.bindToRobot(robotNodes[i], &errorMessage) ||
            !robot.collisionModel.bindToRobot(robotNodes[i], &errorMessage)) {
            g_scene.error = errorMessage;
            return;
        }
    }

    // Reloading replaces the renderer, because its static mesh buffers belong to the old tree.
    g_scene.viewer.reset();
    for (const std::unique_ptr<RobotInstance>& robot : g_scene.robots) {
        robot->timelineSamples.clear();
        robot->timelineMarkers.clear();
        robot->limitReasonCounts.clear();
        robot->durationSeconds = 0.0;
        robot->elapsedSeconds = 0.0;
        robot->appliedTimelineValid = false;
    }
    g_scene.packageLabel = emptyScene ? std::string("Untitled station") : packageFile;
    g_scene.gantries = std::move(gantryControllers);
    g_scene.dragChains = std::move(dragChainControllers);
    g_scene.dragChainPhysics = std::move(dragChainPhysics);
    // Parametric spawner visuals resolve their source object while rebuilding, so the station's
    // stable-id map must already describe the freshly loaded tree at this point.
    g_scene.station = station;
    rebuildParametricAccessories(loaded.get(), g_scene.station);
    g_defaultFloorNode = appendDefaultFloor(loaded);
    g_scene.root = loaded;
    for (const std::unique_ptr<RobotInstance>& robot : g_scene.robots) robot->poseController.resetHome();
    g_scene.viewer = std::make_unique<MeshRobotViewer>(g_scene.root.get());
    g_scene.viewer->reframeCamera();

    g_scene.view.kind = ViewTarget::Kind::Sim;
    g_scene.view.robot = 0;
    g_scene.stationSelectionKind = StationSelectionKind::None;
    g_scene.stationSelectionIndex = -1;
    g_scene.objectDragCandidateKind = StationSelectionKind::None;
    g_scene.objectDragCandidateIndex = -1;
    cancelCameraTween();
    g_scene.simView.framedRobot = -1;
    g_scene.simView.camera = g_scene.viewer->camera();
    g_scene.robotView.camera = MeshRobotViewer::CameraState();
    g_scene.robotView.framedRobot = -1;
    // Bundled programs are loaded further down, per arm, once the station's own per-instance data is
    // being applied.
    g_scene.viewer->setGimbalMoveCallback([](int gimbalIndex, const CadTransform& pose) {
        if (gimbalIndex < 0 ||
            gimbalIndex >= static_cast<int>(g_scene.gimbalTargets.size())) return;
        const SceneGimbalTarget target =
            g_scene.gimbalTargets[static_cast<size_t>(gimbalIndex)];

        if (target.movesObject) {
            if (target.placementNode) {
                target.placementNode->loc =
                    parentWorldTransformOf(target.placementNode).rigidInverse() * pose;
                target.placementNode->needsGlobalLocUpdate = true;
                for (const std::unique_ptr<RobotInstance>& robot : g_scene.robots) {
                    if (robot) robot->readoutJointsValid = false;
                }
                refreshProgramPathPreview();
                refreshSimPathPreview();
            }
            return;
        }

        if (!target.robot) return;
        if (target.programInstruction >= 0) {
            std::string ikError;
            if (!editProgramMoveTargetFromWorldPose(
                    *target.robot, target.programInstruction, pose, &ikError)) {
                target.robot->statusText = ikError;
            }
            return;
        }
        const CadTransform localPose =
            target.robot->poseController.baseWorldTransform().rigidInverse() * pose;
        std::string ikError;
        if (target.robot->poseController.setToolPose(localPose, &ikError)) {
            target.robot->toolTargetPose = target.robot->poseController.toolPose();
            target.robot->statusText.clear();
        } else {
            target.robot->statusText = ikError;
        }
    });
    if (!g_scene.robots.empty()) {
        activeRobot().toolTargetPose = activeRobot().poseController.toolPose();
        activeRobot().toolWprBasePose = activeRobot().toolTargetPose;
    }
    g_scene.error.clear();

    for (const std::unique_ptr<RobotInstance>& robot : g_scene.robots) {
        const OPW6RobotData* robotData = robot->poseController.robotData();
        robot->packageMotionSettings = motionSettingsFromRobotData(robotData);
        robot->packageJointLimitsValid = robotData != nullptr;
        if (robotData) {
            robot->packageJointVelocityMaxRadS = robotData->jointVelocityMaxRadS;
            robot->packageJointAccelMaxRadS2 = robotData->jointAccelerationMaxRadS2;
            robot->packageJointJerkMaxRadS3 = robotData->jointJerkMaxRadS3;
        }
        robot->motionDynamicLimitScale.fill(1.0);
        applyMotionSettingsToEditors(*robot, robot->packageMotionSettings, "Package settings loaded.");
    }

    // Then the station's own answer for each arm, over the top of the package defaults. This is the
    // point of per-instance folders: the package says what the machine is, the station says how
    // this installation of it is calibrated, and the second wins where it has an opinion.
    for (size_t i = 0; i < station.robots.size() && i < g_scene.robots.size(); ++i) {
        if (station.robots[i].config.empty()) continue;
        applyMasteringDocumentToEditors(*g_scene.robots[i], station.robots[i].config);
    }

    if (isStation) {
        for (size_t i = 0; i < station.robots.size() && i < g_scene.robots.size(); ++i) {
            // Empty for an embedded copy, whose temporary is gone by now - see resolvedPackagePath.
            // Such an arm keeps its own empty program, which is what it did before any of this.
            const std::string& resolved = station.robots[i].resolvedPackagePath;
            if (!resolved.empty()) loadPackageProgramsInto(*g_scene.robots[i], resolved);
            // Station programs intentionally win over package samples: they reference this cell's
            // mechanism ids and should not be copied into every use of the robot model.
            loadStationProgramsInto(*g_scene.robots[i], station.robots[i].programs);
        }
    } else if (!emptyScene) {
        loadPackageProgramsInto(activeRobot(), packageFile);
    }

    g_scene.stationSource = (isStation && !emptyScene) ? packageFile : std::string();
    g_scene.station = station;
    g_scene.defaultConveyorMode = station.conveyorSimulationMode == "physx"
        ? ConveyorSimulationMode::PhysX : ConveyorSimulationMode::Logical;

    // The instance list is final, so the driver can hold the arms again.
    rebindLiveRunDriverArms();

    // Hand each arm's firmware the kinematics of the arm it is. Nothing did this before, so the
    // firmware never had a model and answered every movej and movel with model_not_loaded - Run
    // program could not work at all. HardwareIo sends it now if that arm is already connected, and
    // again on its next connect otherwise.
    for (const std::unique_ptr<RobotInstance>& robot : g_scene.robots) {
        if (!robot) continue;
        robot->hardware.setRobotModelCommand(
            robotModelCommandForPackage(robot->poseController, currentRobotModelFor(*robot)));
    }

    refreshPoseDerivedReadouts();
    refreshProgramPathPreview();
}

void ensureSceneLoaded(const std::string& packageFile) {
    if (g_scene.loadAttempted) return;
    g_scene.loadAttempted = true;
    loadPackageIntoScene(packageFile);
}


#if defined(_WIN32) && !defined(__EMSCRIPTEN__)
std::string runPackageDialog() {
    wchar_t buffer[MAX_PATH] = {0};
    OPENFILENAMEW dialog = {};
    dialog.lStructSize = sizeof(dialog);
    // One entry for both, because loadPackageIntoScene decides which it is by reading the JSON. A
    // separate "Open Station" item would only be a way to pick the wrong one.
    dialog.lpstrFilter = L"Robot Packages and Stations\0*.zip;*.json\0All Files\0*.*\0";
    dialog.lpstrFile = buffer;
    dialog.nMaxFile = MAX_PATH;
    dialog.lpstrTitle = L"Open Robot Package or Station";
    dialog.Flags = OFN_EXPLORER | OFN_NOCHANGEDIR | OFN_FILEMUSTEXIST;
    if (!GetOpenFileNameW(&dialog)) return std::string();
    return strutil::wideToUtf8(buffer);
}
#else
std::string runPackageDialog() { return std::string(); }
#endif

// Loads the runtime scene used by CLI station validators.
bool loadStationIntoScene(const std::string& path, LoadedStation* out, std::string* error,
                          bool bindDragChains) {
    LoadedStation loaded;
    loaded.root = loadStationPackage(path, &loaded.document, error);
    if (!loaded.root) return false;
    g_scene.root = loaded.root;
    g_scene.station = loaded.document;
    rebuildParametricAccessories(loaded.root.get(), g_scene.station);
    g_defaultFloorNode = appendDefaultFloor(loaded.root);
    ensureRobotBaseFrames(loaded.root.get());
    g_scene.gantries.clear();
    for (CadNode* node : collectGantryMechanismNodes(loaded.root.get())) {
        GantryPoseController gantry;
        if (!gantry.bindToGantry(node, error)) return false;
        g_scene.gantries.push_back(std::move(gantry));
    }
    if (bindDragChains) {
        for (CadNode* node : collectDragChainMechanismNodes(loaded.root.get())) {
            loaded.chains.emplace_back();
            if (!loaded.chains.back().bindToDragChain(node, error)) return false;
            loaded.chainPhysics.emplace_back();
            if (!loaded.chainPhysics.back().bind(node, error)) return false;
        }
    }
    if (out) *out = std::move(loaded);
    return true;
}

