#include "CliStationCommands.h"
#include "StringUtil.h"

#include "AccessoryBuilders.h"
#include "AccessoryGeometry.h"
#include "AppState.h"
#include "CliHardwareCommands.h"
#include "CliInspectCommands.h"
#include "CliProgramCommands.h"
#include "ConveyorPhysics.h"
#include "ConveyorRuntime.h"
#include "DragChainPhysics.h"
#include "ImGuiApp.h"
#include "JsonCompat.h"
#include "LiveRunDriver.h"
#include "MasteringIo.h"
#include "ProgramEditing.h"
#include "MeshRobotViewer.h"
#include "RobotMotionCore.h"
#include "RobotProgramModel.h"
#include "RobotProgramSimulator.h"
#include "RobotRuntime.h"
#include "SceneMath.h"
#include "StationPackage.h"
#include "StationSceneLoad.h"
#include "TrajectoryAndLiveRun.h"
#include "ViewerBridge.h"

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

int renderStationCommand(const std::vector<std::string>& args) {
#ifdef __EMSCRIPTEN__
    (void)args;
    std::cerr << "Offscreen station rendering is available in native builds only." << std::endl;
    return 2;
#else
    if (args.size() < 10) {
        std::cerr << "Usage: RobotSimulator --render-station <station> <output.ppm> "
                     "<center-x> <center-y> <center-z> <distance> <yaw-deg> <pitch-deg> "
                     "[width height [physics-steps [show-pivots] [show-hulls] "
                     "[show-mounting-holes] [outline-selected-robot] "
                     "[show-conveyor-directions] "
                     "[speed=<0.5|1|2|5|10|100>] "
                     "[async-conveyors] "
                     "[no-conveyor-spawn] "
                     "[no-conveyor-workpieces] "
                     "[physics-accessories=<start>:<count>] "
                     "[frame-selected-robot] [outline-first-accessory] "
                     "[outline-first-mechanism] [program-row=<index>] "
                     "[joints=<j1,j2,j3,j4,j5,j6>]]]" << std::endl;
        return 2;
    }

    std::string errorMessage;
    LoadedStation loaded;
    if (!loadStationIntoScene(args[2], &loaded, &errorMessage, /*bindDragChains=*/true)) {
        std::cerr << errorMessage << std::endl;
        return 1;
    }
    std::shared_ptr<CadNode> root = loaded.root;
    StationDocument& station = loaded.document;
    std::vector<DragChainPoseController>& chains = loaded.chains;
    std::vector<DragChainPhysics>& chainPhysics = loaded.chainPhysics;
    const std::vector<CadNode*> gantryNodes = collectGantryMechanismNodes(root.get());

    const int width = args.size() >= 11 ? std::max(64, std::atoi(args[10].c_str())) : 1600;
    const int height = args.size() >= 12 ? std::max(64, std::atoi(args[11].c_str())) : 1000;
    const int physicsSteps = args.size() >= 13 ? std::max(0, std::atoi(args[12].c_str())) : 0;
    const auto hasRenderFlag = [&](const char* flag) {
        return std::find(args.begin() + std::min<size_t>(13, args.size()), args.end(), flag) !=
               args.end();
    };
    for (size_t index = 13; index < args.size(); ++index) {
        constexpr const char* kSpeedPrefix = "speed=";
        if (args[index].rfind(kSpeedPrefix, 0) != 0) continue;
        const double requested = std::max(0.0, std::atof(args[index].c_str() + 6));
        int closest = 0;
        for (int candidate = 1; candidate < kLiveSpeedCount; ++candidate) {
            if (std::abs(kLiveSpeedFactors[candidate] - requested) <
                std::abs(kLiveSpeedFactors[closest] - requested)) {
                closest = candidate;
            }
        }
        g_scene.liveSpeedIndex = closest;
    }
    g_scene.root = root;
    g_scene.station = station;
    g_scene.defaultConveyorMode = station.conveyorSimulationMode == "physx"
        ? ConveyorSimulationMode::PhysX : ConveyorSimulationMode::Logical;
    g_scene.stationRunState = StationRunState::Running;
    int physicsAccessoryStart = -1;
    int physicsAccessoryCount = 0;
    for (size_t index = 13; index < args.size(); ++index) {
        constexpr const char* kRangePrefix = "physics-accessories=";
        if (args[index].rfind(kRangePrefix, 0) != 0) continue;
        const std::string range = args[index].substr(std::strlen(kRangePrefix));
        const size_t separator = range.find(':');
        if (separator == std::string::npos) continue;
        physicsAccessoryStart = std::max(0, std::atoi(range.substr(0, separator).c_str()));
        physicsAccessoryCount = std::max(0, std::atoi(range.substr(separator + 1).c_str()));
    }
    if (physicsAccessoryStart >= 0) {
        for (size_t index = 0; index < g_scene.station.accessories.size(); ++index) {
            const bool selected = static_cast<int>(index) >= physicsAccessoryStart &&
                static_cast<int>(index) < physicsAccessoryStart + physicsAccessoryCount;
            TransformNodeData* parameters =
                conveyorParameters(g_scene.station.accessories[index].node);
            if (!parameters || selected) continue;
            parameters->accessoryConveyorMode = "logical";
            parameters->accessoryConveyorRole = "normal";
        }
    }
    if (hasRenderFlag("no-conveyor-spawn")) {
        for (const StationAccessoryInstance& entry : g_scene.station.accessories) {
            TransformNodeData* parameters = conveyorParameters(entry.node);
            if (parameters) parameters->accessoryConveyorRole = "normal";
        }
    }
    if (hasRenderFlag("no-conveyor-workpieces")) {
        for (const StationAccessoryInstance& entry : g_scene.station.accessories) {
            TransformNodeData* parameters = conveyorParameters(entry.node);
            if (parameters) parameters->accessoryInitialWorkpieceCount = 0;
        }
    }
    rebuildConveyorRuntime();
    double asyncAchievedRate = 0.0;
    double asyncDroppedSeconds = 0.0;
    ConveyorPhysics::RuntimeStats conveyorRuntimeStats;
    if (hasRenderFlag("async-conveyors")) {
        g_scene.conveyorPhysics.startAsync(
            kLiveSpeedFactors[std::max(0, std::min(g_scene.liveSpeedIndex,
                                                   kLiveSpeedCount - 1))]);
        auto nextFrame = std::chrono::steady_clock::now();
        for (int frame = 0; frame < physicsSteps; ++frame) {
            nextFrame += std::chrono::microseconds(8333);
            std::this_thread::sleep_until(nextFrame);
            for (DragChainPoseController& chain : chains) chain.update();
            for (DragChainPhysics& physics : chainPhysics) physics.step(1.0 / 120.0);
            stepConveyors(1.0 / 120.0);
        }
        const uint64_t finalSteps = g_scene.conveyorPhysics.consumeCompletedAsyncSteps();
        if (finalSteps > 0) {
            stepConveyorTick(finalSteps * ConveyorPhysics::fixedStepSeconds(), false);
        }
        g_scene.conveyorPhysics.asyncAchievedRate(&asyncAchievedRate);
        asyncDroppedSeconds = g_scene.conveyorPhysics.asyncDroppedSimSeconds();
        conveyorRuntimeStats = g_scene.conveyorPhysics.runtimeStats();
        g_scene.conveyorPhysics.stopAsync();
    } else {
        for (int step = 0; step < physicsSteps; ++step) {
            for (DragChainPoseController& chain : chains) chain.update();
            for (DragChainPhysics& physics : chainPhysics) physics.step(1.0 / 120.0);
            stepConveyors(1.0 / 120.0);
        }
    }
    ImGuiApp context;
    if (!context.initialize("RobotSimulator offscreen render", width, height, &errorMessage, false)) {
        std::cerr << "OpenGL context creation failed: " << errorMessage << std::endl;
        return 1;
    }

    MeshRobotViewer viewer(root.get());
    viewer.setDragChainPivotsVisible(hasRenderFlag("show-pivots"));
    viewer.setGantryHullOverlaysVisible(hasRenderFlag("show-hulls"));
    viewer.setMountingHolesVisible(hasRenderFlag("show-mounting-holes"));
    viewer.setConveyorDirectionsVisible(hasRenderFlag("show-conveyor-directions"));
    if (hasRenderFlag("show-conveyor-directions")) {
        viewer.setConveyorDirectionGuides(buildConveyorDirectionGuides(station));
    }
    const std::vector<CadNode*> robotNodes = collectRobotNodes(root.get());
    int programRow = -1;
    std::array<double, 6> overrideJoints{};
    bool haveOverrideJoints = false;
    for (size_t index = 13; index < args.size(); ++index) {
        constexpr const char* kProgramRowPrefix = "program-row=";
        if (args[index].rfind(kProgramRowPrefix, 0) == 0) {
            programRow = std::max(0, std::atoi(args[index].c_str() +
                                               std::strlen(kProgramRowPrefix)));
        }
        constexpr const char* kJointsPrefix = "joints=";
        if (args[index].rfind(kJointsPrefix, 0) == 0) {
            const std::vector<std::string> values = strutil::splitSkippingEmpty(
                args[index].substr(std::strlen(kJointsPrefix)), ',');
            if (values.size() == overrideJoints.size()) {
                haveOverrideJoints = true;
                for (size_t joint = 0; joint < overrideJoints.size(); ++joint) {
                    double degrees = 0.0;
                    if (!parseDoubleToken(values[joint], &degrees)) {
                        haveOverrideJoints = false;
                        break;
                    }
                    overrideJoints[joint] = degrees * kDegToRad;
                }
            }
        }
    }
    if (programRow >= 0 && !robotNodes.empty() && !station.robots.empty()) {
        g_scene.robots.clear();
        auto instance = std::make_unique<RobotInstance>();
        if (!instance->poseController.bindToRobot(robotNodes.front(), &errorMessage)) {
            std::cerr << "Program-row pose bind failed: " << errorMessage << std::endl;
            return 1;
        }
        loadStationProgramsInto(*instance, station.robots.front().programs);
        g_scene.robots.push_back(std::move(instance));
        RobotInstance& robot = *g_scene.robots.front();
        const int last = std::min(
            programRow, static_cast<int>(robot.program().root->children.size()) - 1);
        for (int row = 0; row <= last; ++row) {
            const auto& child = robot.program().root->children[static_cast<size_t>(row)];
            if (!child) continue;
            if (child->type == RobotProgramNodeType::SetTool) {
                if (!commandProgramTool(robot, std::get<SetToolData>(child->data), &errorMessage)) {
                    std::cerr << "Program-row SetTool failed: " << errorMessage << std::endl;
                    return 1;
                }
            } else if (const std::array<double, 6>* target = targetJointsForNode(*child)) {
                double externalMm = 0.0;
                if (externalAxisTargetForNode(*child, &externalMm) &&
                    !station.robots.front().motionLink.empty()) {
                    for (size_t mechanism = 0; mechanism < station.mechanisms.size() &&
                                               mechanism < gantryNodes.size(); ++mechanism) {
                        if (station.mechanisms[mechanism].motionLink !=
                            station.robots.front().motionLink) continue;
                        GantryPoseController gantry;
                        if (gantry.bindToGantry(gantryNodes[mechanism], &errorMessage)) {
                            gantry.setPositionMm(externalMm);
                        }
                    }
                }
                robot.poseController.setJoints(*target);
            }
        }
        if (haveOverrideJoints) robot.poseController.setJoints(overrideJoints);
    }
    for (size_t robotIndex = 0; robotIndex < robotNodes.size(); ++robotIndex) {
        CadNode* robotNode = robotNodes[robotIndex];
        if (programRow >= 0 && robotIndex < g_scene.robots.size() &&
            g_scene.robots[robotIndex]) {
            RobotPoseController& pose = g_scene.robots[robotIndex]->poseController;
            pose.setVisualMeshesVisible(true);
            pose.setJointAxesVisible(false);
            pose.setHullOverlaysVisible(false);
            continue;
        }
        RobotPoseController pose;
        if (pose.bindToRobot(robotNode, &errorMessage)) {
            pose.setVisualMeshesVisible(true);
            pose.setJointAxesVisible(false);
            pose.setHullOverlaysVisible(false);
        }
    }
    if (hasRenderFlag("outline-selected-robot") && !robotNodes.empty()) {
        viewer.setSelectionOutlineRoot(robotNodes.front());
    } else if (hasRenderFlag("outline-first-accessory") && !station.accessories.empty()) {
        viewer.setSelectionOutlineRoot(station.accessories.front().node);
    } else if (hasRenderFlag("outline-first-mechanism") && !gantryNodes.empty()) {
        viewer.setSelectionOutlineRoot(gantryNodes.front());
    }
    MeshRobotViewer::CameraState camera;
    camera.center = Vec3f(static_cast<float>(std::atof(args[4].c_str())),
                          static_cast<float>(std::atof(args[5].c_str())),
                          static_cast<float>(std::atof(args[6].c_str())));
    camera.distance = static_cast<float>(std::atof(args[7].c_str()));
    camera.yaw = static_cast<float>(std::atof(args[8].c_str()));
    camera.pitch = static_cast<float>(std::atof(args[9].c_str()));
    camera.framed = true;
    viewer.setCamera(camera);
    if (hasRenderFlag("frame-selected-robot") && !robotNodes.empty()) {
        viewer.reframeCameraOn(robotNodes.front());
    }
    if (!viewer.renderToPpm(args[3], width, height)) {
        std::cerr << "Failed to write render: " << args[3] << std::endl;
        return 1;
    }
    std::cout << "Station render written: " << args[3] << " camera=("
              << camera.center.x() << ", " << camera.center.y() << ", " << camera.center.z()
              << ") distance=" << camera.distance << " yaw=" << camera.yaw
              << " pitch=" << camera.pitch << " physics_steps=" << physicsSteps
              << " speed=" << kLiveSpeedLabels[std::max(
                     0, std::min(g_scene.liveSpeedIndex, kLiveSpeedCount - 1))]
              << (hasRenderFlag("async-conveyors")
                      ? strutil::format(" async_achieved=%1x async_dropped=%2s static=%3 "
                                        "bodies=%4 active=%5 contacts=%6 box=%7 tri=%8 body=%9 "
                                        "threads=%10 batch=%11 wake_ms=%12 simulate_ms=%13 "
                                        "snapshot_ms=%14 min_y_mm=%15")
                            .arg(asyncAchievedRate, 0, 'f', 2)
                            .arg(asyncDroppedSeconds, 0, 'f', 3)
                            .arg(conveyorRuntimeStats.staticActors)
                            .arg(conveyorRuntimeStats.dynamicBodies)
                            .arg(conveyorRuntimeStats.activeDynamicBodies)
                            .arg(conveyorRuntimeStats.contactPairs)
                            .arg(conveyorRuntimeStats.convexBoxPairs)
                            .arg(conveyorRuntimeStats.convexTrianglePairs)
                            .arg(conveyorRuntimeStats.convexConvexPairs)
                            .arg(conveyorRuntimeStats.dispatcherThreads)
                            .arg(conveyorRuntimeStats.solverBatchSize)
                            .arg(conveyorRuntimeStats.averageWakeMs, 0, 'f', 3)
                            .arg(conveyorRuntimeStats.averageSimulateMs, 0, 'f', 3)
                            .arg(conveyorRuntimeStats.averageSnapshotMs, 0, 'f', 3)
                            .arg(conveyorRuntimeStats.minimumBodyYmm, 0, 'f', 1).str()
                      : std::string())
              << std::endl;
    return 0;
#endif
}

int validateGripperDemoCommand(const std::vector<std::string>& args) {
    if (args.size() < 3) {
        std::cerr << "Usage: RobotSimulator.exe --validate-gripper-demo <station.json> [cycles]" << std::endl;
        return 2;
    }
    int requiredLoopCycles = 1;
    if (args.size() >= 4) {
        try {
            requiredLoopCycles = std::stoi(args[3]);
        } catch (...) {
            requiredLoopCycles = 0;
        }
        if (requiredLoopCycles < 1) {
            std::cerr << "Gripper demo validation cycles must be at least 1." << std::endl;
            return 2;
        }
    }
    std::string error;
    LoadedStation loaded;
    if (!loadStationIntoScene(args[2], &loaded, &error)) {
        std::cerr << error << std::endl;
        return 1;
    }
    std::shared_ptr<CadNode> root = loaded.root;
    StationDocument& document = loaded.document;
    const std::vector<CadNode*> robotNodes = collectRobotNodes(root.get());
    if (robotNodes.empty() || document.robots.empty()) {
        std::cerr << "Gripper demo station has no robot." << std::endl;
        return 1;
    }
    g_scene.defaultConveyorMode = document.conveyorSimulationMode == "physx"
        ? ConveyorSimulationMode::PhysX : ConveyorSimulationMode::Logical;
    g_scene.robots.clear();
    auto instance = std::make_unique<RobotInstance>();
    if (!instance->poseController.bindToRobot(robotNodes.front(), &error) ||
        !instance->collisionModel.bindToRobot(robotNodes.front(), &error)) {
        std::cerr << error << std::endl;
        return 1;
    }
    loadStationProgramsInto(*instance, document.robots.front().programs);
    if (!instance->program().root || instance->program().root->children.empty()) {
        std::cerr << "Gripper demo station has no inline program." << std::endl;
        return 1;
    }
    instance->poseController.resetHome();
    g_scene.robots.push_back(std::move(instance));
    rebuildConveyorRuntime();

    RobotInstance& robot = *g_scene.robots.front();
    bool allMoveTargetsElbowUp = true;
    bool allMoveTargetsSelfClear = true;
    const OPW6RobotData* validationRobotData = robot.poseController.robotData();
    for (const auto& child : robot.program().root->children) {
        if (!child) continue;
        const std::array<double, 6>* target = targetJointsForNode(*child);
        if (!target) continue;
        // The feeder approach/pick targets must stay on the elbow-up branch. The final command is
        // allowed to be the robot package's explicit home posture, even when that manufacturer's
        // home posture lives on the other branch.
        bool isHomeTarget = validationRobotData != nullptr;
        if (validationRobotData) {
            for (size_t joint = 0; joint < target->size(); ++joint) {
                if (std::abs((*target)[joint] - validationRobotData->qHome[joint]) > 1e-5) {
                    isHomeTarget = false;
                    break;
                }
            }
        }
        allMoveTargetsElbowUp = allMoveTargetsElbowUp &&
            (isHomeTarget || (*target)[2] > 0.0);
        allMoveTargetsSelfClear = allMoveTargetsSelfClear &&
            robot.collisionModel.selfCollisions(*target).empty();
    }
    configureRobotExternalAxis(robot);
    robot.simulator.start(robot.program().root.get(), robot.poseController.joints(),
                          programStateFor(robot));
    const RobotProgramSimulator::PlanInput plan =
        RobotProgramSimulator::planInputFromPoseController(robot.poseController);
    // Station Start requests a loop for each robot. This demo deliberately has no Stop: after one
    // unload pass its first coordinated home command must carry J7 smoothly from 6400 back to zero
    // before the next pickup begins.
    if (!robot.simulator.beginLiveRun(plan, true, &error)) {
        std::cerr << error << std::endl;
        return 1;
    }
    bool logicalSeen = false;
    bool physicsSeen = false;
    bool physicsReleased = false;
    bool leftTcpCorrect = false;
    bool rightTcpCorrect = false;
    double maximumPhysicsOffsetMm = 0.0;
    size_t maximumPhysicsOffsetInstruction = 0;
    CadTransform maximumPhysicsOffsetTcp;
    CadTransform maximumPhysicsOffsetBody;
    ConveyorPhysics::BodyHandle observedPhysicsBody = 0;
    bool reportedPhysicsLoss = false;
    bool reportedFeederDisplacement = false;
    bool feederStayedSingleFile = true;
    double maximumAbreastLateralSeparationMm = 0.0;
    ConveyorPhysics::BodyHandle reportedPhysicsCapture = 0;
    std::string lastActuatorStatus;
    CadNode* outputConveyor = nullptr;
    for (const StationAccessoryInstance& accessory : g_scene.station.accessories) {
        if (accessory.id == "output-conveyor") outputConveyor = accessory.node;
    }
    std::vector<const CadNode*> depositedProducts;
    bool linkedAxisReachedEnd = false;
    bool linkedAxisReturning = false;
    bool linkedAxisReturnedHome = false;
    int linkedAxisReturnedCycle = 0;
    double previousLinkedAxisMm = g_scene.gantries.empty()
        ? 0.0 : g_scene.gantries.front().positionMm();
    double maximumLinkedAxisStepMm = 0.0;
    double previousReturnStepMm = 0.0;
    double peakReturnStepMm = 0.0;
    bool haveReturnStep = false;
    bool linkedAxisAccelerated = false;
    bool linkedAxisDecelerated = false;
    constexpr double dt = ConveyorPhysics::fixedStepSeconds();
    for (int step = 0; step < 60 * 360 * requiredLoopCycles; ++step) {
        const RobotProgramSimulator::LiveRunStep run = robot.simulator.stepLiveRun(dt);
        const RobotProgramSimulator::LiveRunState& runState = robot.simulator.liveRunState();
        const int linkedGantry = linkedGantryIndexForRobot(0);
        if (linkedGantry >= 0 && runState.externalAxisValid) {
            const double linkedAxisMm = runState.externalAxisPositionMm;
            const double linkedAxisStepMm = std::abs(linkedAxisMm - previousLinkedAxisMm);
            maximumLinkedAxisStepMm = std::max(
                maximumLinkedAxisStepMm, linkedAxisStepMm);
            linkedAxisReachedEnd = linkedAxisReachedEnd || linkedAxisMm >= 6399.0;
            if (runState.completedCycles >= 1 && linkedAxisMm > 1.0 && linkedAxisMm < 6399.0) {
                linkedAxisReturning = true;
            }
            if (runState.completedCycles >= 1 && linkedAxisReturning &&
                linkedAxisMm < previousLinkedAxisMm && linkedAxisStepMm > 1.0e-4) {
                if (haveReturnStep && linkedAxisStepMm > previousReturnStepMm + 0.01) {
                    linkedAxisAccelerated = true;
                }
                peakReturnStepMm = std::max(peakReturnStepMm, linkedAxisStepMm);
                if (linkedAxisAccelerated && linkedAxisStepMm < peakReturnStepMm - 0.01) {
                    linkedAxisDecelerated = true;
                }
                previousReturnStepMm = linkedAxisStepMm;
                haveReturnStep = true;
            }
            if (runState.completedCycles >= 1 && linkedAxisReturning && linkedAxisMm <= 1.0e-3) {
                linkedAxisReturnedHome = true;
                linkedAxisReturnedCycle = runState.completedCycles;
                linkedAxisReturning = false;
            }
            previousLinkedAxisMm = linkedAxisMm;
            g_scene.gantries[static_cast<size_t>(linkedGantry)].setPositionMm(linkedAxisMm);
        }
        if (run.jointsChanged) robot.poseController.setJoints(robot.simulator.liveRunJoints());
        robot.liveRun = robot.simulator.liveRunState();
        applyActuatorInstructionsForState(robot, robot.liveRun);
        for (const ToolActuatorRuntime& runtime : g_scene.toolActuators) {
            if (!runtime.physicsGraspBody || runtime.physicsGraspBody == reportedPhysicsCapture) {
                continue;
            }
            reportedPhysicsCapture = runtime.physicsGraspBody;
            const CadTransform tcp = actuatorTcpWorld(runtime);
            CadTransform captured;
            g_scene.conveyorPhysics.bodyPose(runtime.physicsGraspBody, &captured);
            std::cout << "physics_grasp_acquired instruction=" << robot.liveRun.instruction
                      << " tcp=(" << tcp.values[3] << ',' << tcp.values[7] << ','
                      << tcp.values[11] << ") body=(" << captured.values[3] << ','
                      << captured.values[7] << ',' << captured.values[11] << ')';
            for (size_t pieceIndex = 0; pieceIndex < g_scene.conveyorWorkpieces.size();
                 ++pieceIndex) {
                const ConveyorWorkpiece& piece = g_scene.conveyorWorkpieces[pieceIndex];
                CadTransform world;
                if (!piece.physicsBody ||
                    !g_scene.conveyorPhysics.bodyPose(piece.physicsBody, &world)) continue;
                std::cout << " piece[" << pieceIndex << "]=(" << world.values[3] << ','
                          << world.values[7] << ',' << world.values[11] << ')';
            }
            std::cout << std::endl;
        }
        stepConveyorTick(dt, true);
        for (const ConveyorWorkpiece& piece : g_scene.conveyorWorkpieces) {
            if (piece.conveyor != outputConveyor || !piece.node) continue;
            if (std::find(depositedProducts.begin(), depositedProducts.end(), piece.node.get()) ==
                depositedProducts.end()) {
                depositedProducts.push_back(piece.node.get());
            }
        }
        struct FeederPiecePose {
            CadTransform local;
            CadVec3 halfExtents;
        };
        std::vector<FeederPiecePose> feederPieces;
        for (const ConveyorWorkpiece& piece : g_scene.conveyorWorkpieces) {
            if (!piece.node || piece.logicalAttachment || piece.graspTool) continue;
            const TransformNodeData* parameters = conveyorParameters(piece.conveyor);
            if (!parameters || parameters->accessoryConveyorRole != "pick_feeder") continue;
            CadTransform world;
            if (!piece.physicsBody || !g_scene.conveyorPhysics.bodyPose(piece.physicsBody, &world)) {
                world = nodeWorldTransform(piece.node.get());
            }
            const CadTransform local = nodeWorldTransform(piece.conveyor).rigidInverse() * world;
            const double halfLength = parameters->accessoryLengthMm * 0.5;
            if (local.values[3] < -halfLength - piece.halfExtentsMm.x ||
                local.values[3] > halfLength + piece.halfExtentsMm.x) {
                continue;
            }
            const double t = (local.values[3] + halfLength) /
                             std::max(1.0, parameters->accessoryLengthMm);
            const double allowedSide = robotPickFeederSurfaceHalfWidthAt(*parameters, t) +
                                       piece.halfExtentsMm.z + 8.0;
            if (std::abs(local.values[11]) > allowedSide) continue;
            feederPieces.push_back({local, piece.halfExtentsMm});
        }
        for (size_t a = 0; a < feederPieces.size(); ++a) {
            for (size_t b = a + 1; b < feederPieces.size(); ++b) {
                const double longitudinalSeparation = std::abs(
                    feederPieces[a].local.values[3] - feederPieces[b].local.values[3]);
                const double lateralSeparation = std::abs(
                    feederPieces[a].local.values[11] - feederPieces[b].local.values[11]);
                const double halfBoxLength = std::min(
                    feederPieces[a].halfExtents.x, feederPieces[b].halfExtents.x);
                const double halfBoxWidth = std::min(
                    feederPieces[a].halfExtents.z, feederPieces[b].halfExtents.z);
                const bool abreast = longitudinalSeparation < halfBoxLength &&
                    lateralSeparation > halfBoxWidth;
                if (abreast) {
                    feederStayedSingleFile = false;
                    maximumAbreastLateralSeparationMm = std::max(
                        maximumAbreastLateralSeparationMm, lateralSeparation);
                }
            }
        }
        if (g_scene.stationSimulationStatus != lastActuatorStatus &&
            g_scene.stationSimulationStatus.find("closed without") != std::string::npos) {
            lastActuatorStatus = g_scene.stationSimulationStatus;
            std::cout << "gripper_capture_miss instruction=" << robot.liveRun.instruction
                      << " status=\"" << lastActuatorStatus << '"';
            for (const ToolActuatorRuntime& runtime : g_scene.toolActuators) {
                if (!runtime.actuator) continue;
                const CadTransform tcp = actuatorTcpWorld(runtime);
                std::cout << " tcp[" << runtime.actuator->id << "]=("
                          << tcp.values[3] << ',' << tcp.values[7] << ',' << tcp.values[11]
                          << ')';
            }
            for (size_t pieceIndex = 0; pieceIndex < g_scene.conveyorWorkpieces.size();
                 ++pieceIndex) {
                const ConveyorWorkpiece& piece = g_scene.conveyorWorkpieces[pieceIndex];
                CadTransform world;
                if (!piece.physicsBody ||
                    !g_scene.conveyorPhysics.bodyPose(piece.physicsBody, &world)) {
                    world = nodeWorldTransform(piece.node.get());
                }
                std::cout << " piece[" << pieceIndex << "]=("
                          << world.values[3] << ',' << world.values[7] << ','
                          << world.values[11] << ')';
            }
            std::cout << std::endl;
        }
        for (const ConveyorWorkpiece& piece : g_scene.conveyorWorkpieces) {
            if (piece.logicalAttachment && !logicalSeen) {
                const OPW6RobotData* robotData = robot.poseController.robotData();
                const RobotToolData* activeTool = robotData && robotData->activeTool
                    ? robotData->activeTool->asRobotTool() : nullptr;
                leftTcpCorrect = activeTool && activeTool->activeTcpIndex == 0;
                logicalSeen = true;
            }
            if (!reportedFeederDisplacement && !physicsSeen && !piece.graspTool &&
                piece.physicsBody) {
                CadTransform body;
                if (g_scene.conveyorPhysics.bodyPose(piece.physicsBody, &body) &&
                    body.values[7] < 650.0) {
                    reportedFeederDisplacement = true;
                    std::cout << "feeder_product_displaced instruction="
                              << robot.liveRun.instruction << " body=("
                              << body.values[3] << ',' << body.values[7] << ','
                              << body.values[11] << ')';
                    for (const ToolActuatorRuntime& runtime : g_scene.toolActuators) {
                        if (!runtime.actuator) continue;
                        const CadTransform tcp = actuatorTcpWorld(runtime);
                        std::cout << " tcp[" << runtime.actuator->id << "]=("
                                  << tcp.values[3] << ',' << tcp.values[7] << ','
                                  << tcp.values[11] << ')';
                        for (size_t bindingIndex = 0;
                             bindingIndex < runtime.actuator->bindings.size(); ++bindingIndex) {
                            const MechanismActuatorBinding& binding =
                                runtime.actuator->bindings[bindingIndex];
                            const MeshGeometryData* mesh =
                                binding.node ? binding.node->asMeshGeometry() : nullptr;
                            if (!mesh) continue;
                            CadTransform center;
                            center.values[3] = (mesh->bounds[0] + mesh->bounds[3]) * 0.5;
                            center.values[7] = (mesh->bounds[1] + mesh->bounds[4]) * 0.5;
                            center.values[11] = (mesh->bounds[2] + mesh->bounds[5]) * 0.5;
                            const CadTransform jaw = nodeWorldTransform(binding.node) * center;
                            std::cout << " jaw[" << runtime.actuator->id << ':'
                                      << bindingIndex << "]=(" << jaw.values[3] << ','
                                      << jaw.values[7] << ',' << jaw.values[11] << ')';
                        }
                    }
                    std::cout << std::endl;
                }
            }
        }
        for (ToolActuatorRuntime& runtime : g_scene.toolActuators) {
            if (!runtime.actuator) continue;
            if (runtime.actuator->interaction != "physics-grasp") continue;
            if (runtime.physicsGraspBody) {
                if (!physicsSeen) {
                    const OPW6RobotData* robotData = robot.poseController.robotData();
                    const RobotToolData* activeTool = robotData && robotData->activeTool
                        ? robotData->activeTool->asRobotTool() : nullptr;
                    rightTcpCorrect = activeTool && activeTool->activeTcpIndex == 1;
                    physicsSeen = true;
                }
                observedPhysicsBody = runtime.physicsGraspBody;
            } else if (physicsSeen && !runtime.wasClosed) {
                physicsReleased = true;
            }
            if (observedPhysicsBody && runtime.wasClosed) {
                CadTransform body;
                if (!g_scene.conveyorPhysics.bodyPose(observedPhysicsBody, &body)) continue;
                const CadTransform tcp = actuatorTcpWorld(runtime);
                const double dx = body.values[3] - tcp.values[3];
                const double dy = body.values[7] - tcp.values[7];
                const double dz = body.values[11] - tcp.values[11];
                const double offset = std::sqrt(dx * dx + dy * dy + dz * dz);
                if (offset > maximumPhysicsOffsetMm) {
                    maximumPhysicsOffsetMm = offset;
                    maximumPhysicsOffsetInstruction = robot.liveRun.instruction;
                    maximumPhysicsOffsetTcp = tcp;
                    maximumPhysicsOffsetBody = body;
                }
                if (offset > 35.0 && !reportedPhysicsLoss) {
                    reportedPhysicsLoss = true;
                    std::cout << "physics_grasp_lost instruction=" << robot.liveRun.instruction
                              << " tcp=(" << tcp.values[3] << ',' << tcp.values[7] << ','
                              << tcp.values[11] << ") body=(" << body.values[3] << ','
                              << body.values[7] << ',' << body.values[11] << ") offset="
                              << offset << std::endl;
                }
            }
        }
        if (!run.running ||
            linkedAxisReturnedCycle >= requiredLoopCycles) break;
    }
    const bool logicalHeldAtStop = std::any_of(
        g_scene.conveyorWorkpieces.begin(), g_scene.conveyorWorkpieces.end(),
        [](const ConveyorWorkpiece& piece) { return piece.logicalAttachment; });
    const bool physicsHeldAtStop = std::any_of(
        g_scene.toolActuators.begin(), g_scene.toolActuators.end(),
        [](const ToolActuatorRuntime& runtime) {
            return runtime.actuator && runtime.actuator->interaction == "physics-grasp" &&
                runtime.wasClosed && runtime.physicsGraspBody != 0;
        });
    bool feederContained = true;
    for (size_t pieceIndex = 0; pieceIndex < g_scene.conveyorWorkpieces.size(); ++pieceIndex) {
        const ConveyorWorkpiece& piece = g_scene.conveyorWorkpieces[pieceIndex];
        const bool heldByPhysicsGripper = piece.physicsBody && std::any_of(
            g_scene.toolActuators.begin(), g_scene.toolActuators.end(),
            [&](const ToolActuatorRuntime& runtime) {
                return runtime.physicsGraspBody == piece.physicsBody;
            });
        if (piece.logicalAttachment || heldByPhysicsGripper) continue;
        const TransformNodeData* parameters = conveyorParameters(piece.conveyor);
        if (!parameters || parameters->accessoryConveyorRole != "pick_feeder" || !piece.node) {
            continue;
        }
        CadTransform world;
        if (!piece.physicsBody || !g_scene.conveyorPhysics.bodyPose(piece.physicsBody, &world)) {
            world = nodeWorldTransform(piece.node.get());
        }
        const CadTransform local = nodeWorldTransform(piece.conveyor).rigidInverse() * world;
        const double halfLength = parameters->accessoryLengthMm * 0.5;
        const double t = (local.values[3] + halfLength) /
                         std::max(1.0, parameters->accessoryLengthMm);
        const double allowedSide = robotPickFeederSurfaceHalfWidthAt(*parameters, t) +
                                   piece.halfExtentsMm.z + 8.0;
        const bool pastStop = local.values[3] > halfLength + piece.halfExtentsMm.x + 5.0;
        const bool escapedSide = std::abs(local.values[11]) > allowedSide;
        const bool fellBelowDeck = local.values[7] <
            conveyorPathPoseAt(*parameters, std::max(0.0, std::min(1.0, t))).y - 80.0;
        if (pastStop || escapedSide || fellBelowDeck) {
            std::cout << "feeder_escape piece=" << pieceIndex << " local=("
                      << local.values[3] << ','
                      << local.values[7] << ',' << local.values[11] << ") t=" << t
                      << " allowed_side=" << allowedSide
                      << " logical=" << (piece.logicalAttachment ? "yes" : "no")
                      << " grasp_tool=" << (piece.graspTool ? piece.graspTool->name : "none")
                      << " past_stop=" << (pastStop ? "yes" : "no")
                      << " escaped_side=" << (escapedSide ? "yes" : "no")
                      << " fell_below=" << (fellBelowDeck ? "yes" : "no") << std::endl;
        }
        feederContained = feederContained && !pastStop && !escapedSide && !fellBelowDeck;
    }
    const bool retained = maximumPhysicsOffsetMm <= 35.0;
    const bool programLooping = robot.simulator.liveRunState().running &&
        robot.simulator.liveRunState().completedCycles >= requiredLoopCycles &&
        linkedAxisReturnedCycle >= requiredLoopCycles;
    // At 300 mm/s (the fastest J7 command in this program), one 1/60 s validation tick advances at
    // most 5 mm once cruising. Ten leaves room for the control-period boundary without allowing a
    // command-boundary teleport to hide as an ordinary sample.
    const bool linkedAxisMotionSmooth = maximumLinkedAxisStepMm <= 10.0;
    const bool linkedAxisMotionRamped = linkedAxisAccelerated && linkedAxisDecelerated;
    const bool bothProductsDeposited = depositedProducts.size() >= 2;
    bool spawnerCapsRespected = true;
    bool deletionReleasedSpawnerCapacity = true;
    size_t maximumActiveSpawnCount = 0;
    for (const ConveyorSpawnerRuntime& spawner : g_scene.conveyorSpawners) {
        TransformNodeData* parameters = conveyorParameters(spawner.conveyor);
        if (!parameters || parameters->accessoryMaxActiveSpawns <= 0) continue;
        const size_t limit = static_cast<size_t>(parameters->accessoryMaxActiveSpawns);
        const size_t beforeDelete = activeSpawnCount(spawner.conveyor);
        maximumActiveSpawnCount = std::max(maximumActiveSpawnCount, beforeDelete);
        spawnerCapsRespected = spawnerCapsRespected && beforeDelete <= limit;
        if (beforeDelete != limit) continue;

        const auto removable = std::find_if(
            g_scene.conveyorWorkpieces.begin(), g_scene.conveyorWorkpieces.end(),
            [&](const ConveyorWorkpiece& piece) {
                return piece.originSpawner == spawner.conveyor && !piece.logicalAttachment &&
                    !piece.graspTool;
            });
        if (removable == g_scene.conveyorWorkpieces.end()) {
            deletionReleasedSpawnerCapacity = false;
            continue;
        }
        if (removable->physicsBody) {
            g_scene.conveyorPhysics.removeBody(removable->physicsBody);
        }
        detachConveyorWorkpiece(*removable);
        g_scene.conveyorWorkpieces.erase(removable);
        const size_t afterDelete = activeSpawnCount(spawner.conveyor);
        spawnConveyorWorkpiece(spawner.conveyor, 0.25);
        const size_t afterReplacement = activeSpawnCount(spawner.conveyor);
        deletionReleasedSpawnerCapacity = deletionReleasedSpawnerCapacity &&
            afterDelete + 1 == beforeDelete && afterReplacement == beforeDelete;
    }
    std::cout << "gripper_demo logical_grasp=" << (logicalSeen ? "yes" : "no")
              << " logical_held_at_stop=" << (logicalHeldAtStop ? "yes" : "no")
              << " left_tcp_correct=" << (leftTcpCorrect ? "yes" : "no")
              << " physics_grasp=" << (physicsSeen ? "yes" : "no")
              << " physics_held_at_stop=" << (physicsHeldAtStop ? "yes" : "no")
              << " right_tcp_correct=" << (rightTcpCorrect ? "yes" : "no")
              << " products_released="
              << ((physicsReleased && !logicalHeldAtStop) ? "yes" : "no")
              << " elbow_up_targets=" << (allMoveTargetsElbowUp ? "yes" : "no")
              << " target_self_collision=" << (allMoveTargetsSelfClear ? "no" : "yes")
              << " feeder_contained=" << (feederContained ? "yes" : "no")
              << " feeder_single_file=" << (feederStayedSingleFile ? "yes" : "no")
              << " max_abreast_lateral_mm=" << maximumAbreastLateralSeparationMm
              << " spawner_cap=" << (spawnerCapsRespected ? "yes" : "no")
              << " max_active_spawns=" << maximumActiveSpawnCount
              << " deletion_releases_capacity="
              << (deletionReleasedSpawnerCapacity ? "yes" : "no")
              << " program_loop=" << (programLooping ? "yes" : "no")
              << " completed_cycles=" << robot.simulator.liveRunState().completedCycles
              << " linked_axis_end=" << (linkedAxisReachedEnd ? "yes" : "no")
              << " linked_axis_return=" << (linkedAxisReturnedHome ? "yes" : "no")
              << " linked_axis_smooth=" << (linkedAxisMotionSmooth ? "yes" : "no")
              << " linked_axis_ramped=" << (linkedAxisMotionRamped ? "yes" : "no")
              << " linked_axis_max_step_mm=" << maximumLinkedAxisStepMm
              << " linked_axis_final_mm="
              << (g_scene.gantries.empty() ? 0.0 : g_scene.gantries.front().positionMm())
              << " output_deposits=" << depositedProducts.size()
              << " physics_max_tcp_offset_mm=" << maximumPhysicsOffsetMm
              << " at_instruction=" << maximumPhysicsOffsetInstruction
              << " tcp=(" << maximumPhysicsOffsetTcp.values[3] << ','
              << maximumPhysicsOffsetTcp.values[7] << ',' << maximumPhysicsOffsetTcp.values[11]
              << ") body=(" << maximumPhysicsOffsetBody.values[3] << ','
              << maximumPhysicsOffsetBody.values[7] << ','
              << maximumPhysicsOffsetBody.values[11] << ") status=\""
              << robot.simulator.liveRunState().status << '"' << std::endl;
    clearConveyorWorkpieces();
    return logicalSeen && !logicalHeldAtStop && leftTcpCorrect && physicsSeen &&
        !physicsHeldAtStop && rightTcpCorrect && physicsReleased && retained &&
        allMoveTargetsElbowUp && allMoveTargetsSelfClear && feederContained &&
        feederStayedSingleFile && spawnerCapsRespected && deletionReleasedSpawnerCapacity &&
        programLooping && linkedAxisReachedEnd && linkedAxisReturnedHome &&
        linkedAxisMotionSmooth && linkedAxisMotionRamped && bothProductsDeposited
        ? 0 : 1;
}

int validateStationProgramLoopCommand(const std::vector<std::string>& args) {
    if (args.size() < 3) {
        std::cerr << "Usage: RobotSimulator.exe --validate-station-program-loop "
                     "<station.json> [cycles]" << std::endl;
        return 2;
    }
    int requiredCycles = 5;
    if (args.size() >= 4) {
        try {
            requiredCycles = std::stoi(args[3]);
        } catch (...) {
            requiredCycles = 0;
        }
        if (requiredCycles < 1) {
            std::cerr << "Station loop validation cycles must be at least 1." << std::endl;
            return 2;
        }
    }

    std::string error;
    LoadedStation loaded;
    if (!loadStationIntoScene(args[2], &loaded, &error)) {
        std::cerr << error << std::endl;
        return 1;
    }
    const std::vector<CadNode*> robotNodes = collectRobotNodes(loaded.root.get());
    if (robotNodes.empty() || loaded.document.robots.empty()) {
        std::cerr << "Station loop validation needs a robot." << std::endl;
        return 1;
    }
    g_scene.robots.clear();
    auto instance = std::make_unique<RobotInstance>();
    if (!instance->poseController.bindToRobot(robotNodes.front(), &error)) {
        std::cerr << error << std::endl;
        return 1;
    }
    loadStationProgramsInto(*instance, loaded.document.robots.front().programs);
    if (!instance->program().root || instance->program().root->children.empty()) {
        std::cerr << "Station loop validation needs an inline robot program." << std::endl;
        return 1;
    }
    instance->poseController.resetHome();
    g_scene.robots.push_back(std::move(instance));

    RobotInstance& robot = *g_scene.robots.front();
    configureRobotExternalAxis(robot);
    robot.simulator.start(robot.program().root.get(), robot.poseController.joints(),
                          programStateFor(robot));
    const RobotProgramSimulator::PlanInput plan =
        RobotProgramSimulator::planInputFromPoseController(robot.poseController);
    if (!robot.simulator.beginLiveRun(plan, true, &error)) {
        std::cerr << error << std::endl;
        return 1;
    }

    const int linkedGantry = linkedGantryIndexForRobot(0);
    double previousAxisMm = linkedGantry >= 0
        ? g_scene.gantries[static_cast<size_t>(linkedGantry)].positionMm() : 0.0;
    double maximumAxisStepMm = 0.0;
    bool reachedFarEnd = false;
    bool returning = false;
    int returnedCycle = 0;
    constexpr double dt = 1.0 / 60.0;
    const int guardSteps = 60 * 600 * requiredCycles;
    for (int stepIndex = 0; stepIndex < guardSteps; ++stepIndex) {
        const RobotProgramSimulator::LiveRunStep step = robot.simulator.stepLiveRun(dt);
        const RobotProgramSimulator::LiveRunState& state = robot.simulator.liveRunState();
        if (step.jointsChanged) robot.poseController.setJoints(robot.simulator.liveRunJoints());
        if (linkedGantry >= 0 && state.externalAxisValid) {
            const double positionMm = state.externalAxisPositionMm;
            maximumAxisStepMm = std::max(maximumAxisStepMm,
                                         std::abs(positionMm - previousAxisMm));
            reachedFarEnd = reachedFarEnd || positionMm >= 6399.0;
            if (state.completedCycles > returnedCycle && positionMm > 1.0 &&
                positionMm < previousAxisMm) {
                returning = true;
            }
            if (returning && positionMm <= 1.0e-3) {
                returnedCycle = state.completedCycles;
                returning = false;
            }
            previousAxisMm = positionMm;
            g_scene.gantries[static_cast<size_t>(linkedGantry)].setPositionMm(positionMm);
        }
        if (!step.running || returnedCycle >= requiredCycles) break;
    }

    const RobotProgramSimulator::LiveRunState& state = robot.simulator.liveRunState();
    const bool passed = state.running && state.completedCycles >= requiredCycles &&
        returnedCycle >= requiredCycles && reachedFarEnd && maximumAxisStepMm <= 10.0;
    std::cout << "station_program_loop requested_cycles=" << requiredCycles
              << " completed_cycles=" << state.completedCycles
              << " returned_cycles=" << returnedCycle
              << " reached_far_end=" << (reachedFarEnd ? "yes" : "no")
              << " max_axis_step_mm=" << maximumAxisStepMm
              << " final_axis_mm=" << previousAxisMm
              << " running=" << (state.running ? "yes" : "no")
              << " status=\"" << state.status << "\"" << std::endl;
    return passed ? 0 : 1;
}

int dumpStationProgramIkCommand(const std::vector<std::string>& args) {
    if (args.size() < 3) {
        std::cerr << "Usage: RobotSimulator.exe --dump-station-program-ik <station.json> "
                     "[target-row [vertical-lift-mm [robot-facing-offset-mm "
                     "[world-x-offset-mm [world-z-offset-mm]]]]]" << std::endl;
        return 2;
    }
    std::string error;
    LoadedStation loaded;
    if (!loadStationIntoScene(args[2], &loaded, &error)) {
        std::cerr << error << std::endl;
        return 1;
    }
    const std::vector<CadNode*> robotNodes = collectRobotNodes(loaded.root.get());
    if (robotNodes.empty() || loaded.document.robots.empty()) return 1;
    g_scene.robots.clear();
    auto instance = std::make_unique<RobotInstance>();
    if (!instance->poseController.bindToRobot(robotNodes.front(), &error) ||
        !instance->collisionModel.bindToRobot(robotNodes.front(), &error)) {
        std::cerr << error << std::endl;
        return 1;
    }
    loadStationProgramsInto(*instance, loaded.document.robots.front().programs);
    g_scene.robots.push_back(std::move(instance));
    RobotInstance& robot = *g_scene.robots.front();
    auto printDistalLinkBounds = [&](const std::array<double, 6>& joints) {
        robot.poseController.setJoints(joints);
        for (const auto& child : robotNodes.front()->children) {
            if (!child) continue;
            const bool distalGeometry =
                child->name.rfind("Link 4 geometry", 0) == 0 ||
                child->name.rfind("Link 5 geometry", 0) == 0 ||
                child->name.rfind("Link 6 geometry", 0) == 0;
            if (!distalGeometry) continue;
            Vec3f minimum;
            Vec3f maximum;
            if (!MeshRobotViewer::computeBoundsOf(child.get(), minimum, maximum)) continue;
            std::cout << " [" << child->name << " xyz="
                      << minimum.x() << ',' << minimum.y() << ',' << minimum.z() << ".."
                      << maximum.x() << ',' << maximum.y() << ',' << maximum.z() << ']';
        }
        for (const StationAccessoryInstance& accessory : g_scene.station.accessories) {
            if (!accessory.node || !accessory.node->asRobotTool()) continue;
            Vec3f minimum;
            Vec3f maximum;
            if (!MeshRobotViewer::computeBoundsOf(accessory.node, minimum, maximum)) continue;
            std::cout << " [tool " << accessory.id << " xyz="
                      << minimum.x() << ',' << minimum.y() << ',' << minimum.z() << ".."
                      << maximum.x() << ',' << maximum.y() << ',' << maximum.z() << ']';
        }
    };
    const int topRow = args.size() >= 4 ? std::atoi(args[3].c_str()) : -1;
    const double verticalLiftMm = args.size() >= 5 ? std::atof(args[4].c_str()) : 0.0;
    const double robotFacingOffsetMm = args.size() >= 6 ? std::atof(args[5].c_str()) : 0.0;
    const double worldXOffsetMm = args.size() >= 7 ? std::atof(args[6].c_str()) : 0.0;
    const double worldZOffsetMm = args.size() >= 8 ? std::atof(args[7].c_str()) : 0.0;
    for (int row = 0; row < static_cast<int>(robot.program().root->children.size()); ++row) {
        const auto& child = robot.program().root->children[static_cast<size_t>(row)];
        if (!child) continue;
        if (child->type == RobotProgramNodeType::SetTool) {
            if (!commandProgramTool(robot, std::get<SetToolData>(child->data), &error)) {
                std::cerr << "row=" << row << ' ' << error << std::endl;
                return 1;
            }
            continue;
        }
        const std::array<double, 6>* target = targetJointsForNode(*child);
        if (!target) continue;
        double externalMm = 0.0;
        if (externalAxisTargetForNode(*child, &externalMm)) {
            const int gantryIndex = linkedGantryIndexForRobot(0);
            if (gantryIndex >= 0) {
                g_scene.gantries[static_cast<size_t>(gantryIndex)].setPositionMm(externalMm);
            }
        }
        robot.poseController.setJoints(*target);
        const CadTransform targetPose = robot.poseController.toolPose();
        const CadTransform targetWorld = robot.poseController.baseWorldTransform() * targetPose;
        std::array<std::array<double, 6>, RobotPoseController::kMaxToolPoseSolutions> solutions{};
        const int count = robot.poseController.toolPoseSolutions(
            targetPose, solutions.data(), RobotPoseController::kMaxToolPoseSolutions, &error);
        std::cout << "row=" << row << " type=" << robotProgramNodeLabel(child->type)
                  << " target=";
        for (double q : *target) std::cout << ' ' << q * kRadToDeg;
        std::cout << " world_tcp=(" << targetWorld.values[3] << ',' << targetWorld.values[7]
                  << ',' << targetWorld.values[11] << ") solutions=" << count << std::endl;
        for (int solution = 0; solution < count; ++solution) {
            const auto& q = solutions[static_cast<size_t>(solution)];
            const auto collisions = robot.collisionModel.selfCollisions(q);
            std::cout << "  " << (q[2] >= 0.0 ? "elbow_up" : "elbow_down")
                      << (collisions.empty() ? " clear" : " self_collision") << ':';
            for (double value : q) std::cout << ' ' << value * kRadToDeg;
            std::cout << std::endl;
        }
        if (row == topRow) {
            const CadTransform baseWorld = robot.poseController.baseWorldTransform();
            const CadTransform currentWorld = baseWorld * targetPose;
            if (std::abs(robotFacingOffsetMm) > 1e-9) {
                CadTransform sideWorld = currentWorld;
                sideWorld.values[11] -= robotFacingOffsetMm;
                const CadTransform sideLocal = baseWorld.rigidInverse() * sideWorld;
                std::array<std::array<double, 6>, RobotPoseController::kMaxToolPoseSolutions>
                    sideSolutions{};
                const int sideCount = robot.poseController.toolPoseSolutions(
                    sideLocal, sideSolutions.data(), RobotPoseController::kMaxToolPoseSolutions,
                    &error);
                std::cout << "  side offset=" << robotFacingOffsetMm
                          << " solutions=" << sideCount << " world_xyz="
                          << sideWorld.values[3] << ',' << sideWorld.values[7] << ','
                          << sideWorld.values[11] << std::endl;
                for (int solution = 0; solution < sideCount; ++solution) {
                    const auto& q = sideSolutions[static_cast<size_t>(solution)];
                    const auto collisions = robot.collisionModel.selfCollisions(q);
                    std::cout << "    " << (q[2] >= 0.0 ? "elbow_up" : "elbow_down")
                              << (collisions.empty() ? " clear" : " self_collision") << ':';
                    for (double value : q) std::cout << ' ' << value * kRadToDeg;
                    std::cout << std::endl;
                }
                for (int rollDegrees : {0, 90, 180, 270}) {
                    const double roll = rollDegrees * kDegToRad;
                    const double c = std::cos(roll);
                    const double s = std::sin(roll);
                    CadTransform horizontalPickWorld;
                    // TCP local Z enters the robot-facing window along +world Z. Roll selects
                    // where the unused half of a dual gripper sits around that approach axis.
                    horizontalPickWorld.values = {{
                        c, -s, 0.0, currentWorld.values[3],
                        s, c, 0.0, currentWorld.values[7] + verticalLiftMm,
                        0.0, 0.0, 1.0, currentWorld.values[11]
                    }};
                    for (const auto& variant : {
                             std::pair<const char*, double>{"pick", 0.0},
                             std::pair<const char*, double>{"approach", -robotFacingOffsetMm}}) {
                        CadTransform desiredWorld = horizontalPickWorld;
                        desiredWorld.values[11] += variant.second;
                        const CadTransform desiredLocal =
                            baseWorld.rigidInverse() * desiredWorld;
                        std::array<std::array<double, 6>,
                                   RobotPoseController::kMaxToolPoseSolutions> rollSolutions{};
                        const int rollCount = robot.poseController.toolPoseSolutions(
                            desiredLocal, rollSolutions.data(),
                            RobotPoseController::kMaxToolPoseSolutions, &error);
                        std::cout << "  horizontal roll=" << rollDegrees << ' '
                                  << variant.first << " solutions=" << rollCount
                                  << " world_xyz=" << desiredWorld.values[3] << ','
                                  << desiredWorld.values[7] << ','
                                  << desiredWorld.values[11] << std::endl;
                        for (int solution = 0; solution < rollCount; ++solution) {
                            const auto& q = rollSolutions[static_cast<size_t>(solution)];
                            const auto collisions = robot.collisionModel.selfCollisions(q);
                            std::cout << "    "
                                      << (q[2] >= 0.0 ? "elbow_up" : "elbow_down")
                                      << (collisions.empty() ? " clear" : " self_collision")
                                      << ':';
                            for (double value : q) std::cout << ' ' << value * kRadToDeg;
                            std::cout << std::endl;
                        }
                    }
                }
            }
            for (int yawDegrees : {0, 90, 180, 270}) {
                const double yaw = yawDegrees * kDegToRad;
                const double c = std::cos(yaw);
                const double s = std::sin(yaw);
                CadTransform desiredWorld;
                desiredWorld.values = {{
                    c, -s, 0.0, currentWorld.values[3] + worldXOffsetMm,
                    0.0, 0.0, -1.0, currentWorld.values[7] + verticalLiftMm,
                    s, c, 0.0, currentWorld.values[11] + worldZOffsetMm
                }};
                const CadTransform desiredLocal = baseWorld.rigidInverse() * desiredWorld;
                std::array<std::array<double, 6>, RobotPoseController::kMaxToolPoseSolutions>
                    topSolutions{};
                const int topCount = robot.poseController.toolPoseSolutions(
                    desiredLocal, topSolutions.data(), RobotPoseController::kMaxToolPoseSolutions,
                    &error);
                std::cout << "  top yaw=" << yawDegrees << " solutions=" << topCount
                           << " world_xyz=" << currentWorld.values[3] + worldXOffsetMm << ','
                           << currentWorld.values[7] + verticalLiftMm << ','
                           << currentWorld.values[11] + worldZOffsetMm << std::endl;
                for (int solution = 0; solution < topCount; ++solution) {
                    const auto& q = topSolutions[static_cast<size_t>(solution)];
                    const auto collisions = robot.collisionModel.selfCollisions(q);
                    std::cout << "    " << (q[2] >= 0.0 ? "elbow_up" : "elbow_down")
                              << (collisions.empty() ? " clear" : " self_collision") << ':';
                    for (double value : q) std::cout << ' ' << value * kRadToDeg;
                    printDistalLinkBounds(q);
                    std::cout << std::endl;
                }
            }
        }
    }
    return 0;
}


namespace {

// One product's observable state after a tick. Deliberately not a ConveyorWorkpiece: everything a
// conveyor rule can be wrong about is here, and nothing a rule cannot reach - no node pointers, no
// physics handles, nothing whose value is an allocator's choice.
struct ConveyorTraceProduct {
    uint64_t id = 0;
    std::string conveyor;
    double progress = 0.0;
    bool forward = true;
    bool physical = false;
    bool grasped = false;
    CadVec3 world;
};

std::string conveyorTraceAccessoryId(const CadNode* node) {
    if (!node) return "-";
    for (const StationAccessoryInstance& accessory : g_scene.station.accessories) {
        if (accessory.node == node) return accessory.id;
    }
    return "?";
}

std::string conveyorTraceNumber(double value, int decimals) {
    if (value == 0.0) value = 0.0;
    char buffer[64];
    std::snprintf(buffer, sizeof(buffer), "%.*f", decimals, value);
    return std::string(buffer);
}

std::vector<ConveyorTraceProduct> captureConveyorTraceProducts() {
    std::vector<ConveyorTraceProduct> snapshot;
    snapshot.reserve(g_scene.conveyorWorkpieces.size());
    for (const ConveyorWorkpiece& piece : g_scene.conveyorWorkpieces) {
        ConveyorTraceProduct product;
        product.id = piece.id;
        product.conveyor = conveyorTraceAccessoryId(piece.conveyor);
        product.progress = piece.progress;
        product.forward = piece.forward;
        product.physical = piece.physicsBody != 0;
        product.grasped = piece.graspTool != nullptr;
        if (piece.node) {
            const CadTransform world = parentWorldTransformOf(piece.node.get()) * piece.node->loc;
            product.world = CadVec3(world.values[3], world.values[7], world.values[11]);
        }
        snapshot.push_back(product);
    }
    // By id, not by storage order. Which slot a product occupies in the scene's vector is not one of
    // the rules, and a port that keeps every rule while packing the vector differently is correct.
    std::sort(snapshot.begin(), snapshot.end(),
              [](const ConveyorTraceProduct& a, const ConveyorTraceProduct& b) {
                  return a.id < b.id;
              });
    return snapshot;
}

std::string conveyorTraceProductLine(const ConveyorTraceProduct& product) {
    return "p" + std::to_string(product.id) + " conv=" + product.conveyor +
        " prog=" + conveyorTraceNumber(product.progress, 9) +
        " fwd=" + (product.forward ? "1" : "0") +
        " phys=" + (product.physical ? "1" : "0") +
        " grasp=" + (product.grasped ? "1" : "0") +
        " world=" + conveyorTraceNumber(product.world.x, 4) + "," +
        conveyorTraceNumber(product.world.y, 4) + "," +
        conveyorTraceNumber(product.world.z, 4);
}

} // namespace

int dumpConveyorTraceCommand(const std::vector<std::string>& args) {
    if (args.size() < 4) {
        std::cerr << "Usage: RobotSimulator.exe --dump-conveyor-trace <station.json> <seconds> "
                     "[logical|physx|station [sample-every-ticks]]" << std::endl;
        return 2;
    }
    double seconds = 0.0;
    try {
        seconds = std::stod(args[3]);
    } catch (...) {
        seconds = 0.0;
    }
    if (!(seconds > 0.0)) {
        std::cerr << "Trace duration must be positive." << std::endl;
        return 2;
    }
    const std::string mode = args.size() >= 5 ? args[4] : std::string("logical");
    if (mode != "logical" && mode != "physx" && mode != "station") {
        std::cerr << "Trace mode must be logical, physx or station." << std::endl;
        return 2;
    }
    int sampleEvery = 6;
    if (args.size() >= 6) {
        try {
            sampleEvery = std::stoi(args[5]);
        } catch (...) {
            sampleEvery = 0;
        }
        if (sampleEvery < 1) {
            std::cerr << "Sample interval must be at least one tick." << std::endl;
            return 2;
        }
    }

    std::string error;
    LoadedStation loaded;
    if (!loadStationIntoScene(args[2], &loaded, &error)) {
        std::cerr << error << std::endl;
        return 1;
    }
    if (mode != "station") {
        g_scene.defaultConveyorMode = mode == "physx"
            ? ConveyorSimulationMode::PhysX : ConveyorSimulationMode::Logical;
        for (const StationAccessoryInstance& accessory : g_scene.station.accessories) {
            if (TransformNodeData* parameters = conveyorParameters(accessory.node)) {
                parameters->accessoryConveyorMode = mode;
            }
        }
    }
    rebuildConveyorRuntime();

    std::vector<const StationAccessoryInstance*> conveyors;
    for (const StationAccessoryInstance& accessory : g_scene.station.accessories) {
        if (conveyorParameters(accessory.node)) conveyors.push_back(&accessory);
    }
    if (conveyors.empty()) {
        std::cerr << "Station has no roller conveyor to trace." << std::endl;
        return 1;
    }

    constexpr double kTickSeconds = ConveyorPhysics::fixedStepSeconds();
    const int ticks = static_cast<int>(std::llround(seconds / kTickSeconds));
    std::cout << "conveyor_trace mode=" << mode << " hz=60 ticks=" << ticks
              << " sample_every=" << sampleEvery << std::endl;
    for (const StationAccessoryInstance* accessory : conveyors) {
        const TransformNodeData* parameters = conveyorParameters(accessory->node);
        std::cout << "c " << accessory->id
                  << " role=" << parameters->accessoryConveyorRole
                  << " mode=" << (resolvedConveyorMode(*parameters) ==
                                  ConveyorSimulationMode::PhysX ? "physx" : "logical")
                  << " speed_mm_s=" << conveyorTraceNumber(parameters->accessoryConveyorSpeedMmS, 3)
                  << " length_mm=" << conveyorTraceNumber(conveyorPathLengthMm(*parameters), 3)
                  << " interval_s="
                  << conveyorTraceNumber(parameters->accessorySpawnIntervalSeconds, 3)
                  << " max_active=" << parameters->accessoryMaxActiveSpawns
                  << " initial=" << parameters->accessoryInitialWorkpieceCount << std::endl;
    }

    std::vector<ConveyorTraceProduct> previous = captureConveyorTraceProducts();
    for (const ConveyorTraceProduct& product : previous) {
        std::cout << "init " << conveyorTraceProductLine(product) << std::endl;
    }

    uint64_t spawned = 0;
    uint64_t destroyed = 0;
    uint64_t transferred = 0;
    uint64_t previousDirtyMarks = conveyorSceneDirtyMarkCount();
    std::vector<std::string> invariantFailures;
    for (int tick = 1; tick <= ticks; ++tick) {
        stepConveyorTick(kTickSeconds);
        const std::vector<ConveyorTraceProduct> current = captureConveyorTraceProducts();
        const uint64_t dirtyMarks = conveyorSceneDirtyMarkCount();

        // Events on every tick, state on sampled ticks. A cadence that drifts by one tick moves a
        // spawn event, which sampling would have hidden; a rule that moves a product wrongly shows
        // up in the state lines, which do not need every tick to say so.
        std::vector<std::string> lines;
        size_t previousIndex = 0;
        size_t currentIndex = 0;
        while (previousIndex < previous.size() || currentIndex < current.size()) {
            const bool havePrevious = previousIndex < previous.size();
            const bool haveCurrent = currentIndex < current.size();
            if (haveCurrent && (!havePrevious ||
                                current[currentIndex].id < previous[previousIndex].id)) {
                lines.push_back("  + " + conveyorTraceProductLine(current[currentIndex]));
                ++spawned;
                ++currentIndex;
                continue;
            }
            if (havePrevious && (!haveCurrent ||
                                 previous[previousIndex].id < current[currentIndex].id)) {
                lines.push_back("  - " + conveyorTraceProductLine(previous[previousIndex]));
                ++destroyed;
                ++previousIndex;
                continue;
            }
            if (previous[previousIndex].conveyor != current[currentIndex].conveyor) {
                lines.push_back("  > p" + std::to_string(current[currentIndex].id) + " " +
                                previous[previousIndex].conveyor + "->" +
                                current[currentIndex].conveyor + " fwd=" +
                                (current[currentIndex].forward ? "1" : "0") + " prog=" +
                                conveyorTraceNumber(current[currentIndex].progress, 9));
                ++transferred;
            }
            ++previousIndex;
            ++currentIndex;
        }
        if (tick % sampleEvery == 0) {
            for (const ConveyorTraceProduct& product : current) {
                lines.push_back("  = " + conveyorTraceProductLine(product));
            }
        }
        if (!lines.empty() || dirtyMarks != previousDirtyMarks) {
            std::cout << "tick " << tick << " products=" << current.size()
                      << " dirty=" << (dirtyMarks - previousDirtyMarks) << std::endl;
            for (const std::string& line : lines) std::cout << line << std::endl;
        }

        // Held to in every mode, including the physical one that cannot be goldened. A product is
        // on its conveyor's path, or it has been transferred, or it has been deleted; there is no
        // fourth outcome, whatever the backend does between ticks.
        for (const ConveyorTraceProduct& product : current) {
            if (product.progress < 0.0 || product.progress > 1.0) {
                invariantFailures.push_back(
                    "p" + std::to_string(product.id) + " left 0..1 with prog=" +
                    conveyorTraceNumber(product.progress, 9));
            }
            if (product.conveyor == "?" || product.conveyor == "-") {
                invariantFailures.push_back(
                    "p" + std::to_string(product.id) + " is on no station conveyor");
            }
        }
        for (const StationAccessoryInstance* accessory : conveyors) {
            const TransformNodeData* parameters = conveyorParameters(accessory->node);
            if (parameters->accessoryConveyorRole != "spawner" ||
                parameters->accessoryMaxActiveSpawns <= 0) continue;
            const size_t active = activeSpawnCount(accessory->node);
            if (active > static_cast<size_t>(parameters->accessoryMaxActiveSpawns)) {
                invariantFailures.push_back(
                    accessory->id + " exceeded maxActiveSpawns with " +
                    std::to_string(active) + " active");
            }
        }

        previous = current;
        previousDirtyMarks = dirtyMarks;
    }

    std::cout << "end products=" << previous.size() << " spawned=" << spawned
              << " destroyed=" << destroyed << " transferred=" << transferred << std::endl;
    for (const ConveyorTraceProduct& product : previous) {
        std::cout << "final " << conveyorTraceProductLine(product) << std::endl;
    }
    // Deduplicated: one broken rule otherwise reports once per tick for the rest of the run, and a
    // thousand copies of one line is not a better report than one.
    std::sort(invariantFailures.begin(), invariantFailures.end());
    invariantFailures.erase(std::unique(invariantFailures.begin(), invariantFailures.end()),
                            invariantFailures.end());
    for (const std::string& failure : invariantFailures) {
        std::cout << "INVARIANT " << failure << std::endl;
    }
    std::cout << "invariant_failures=" << invariantFailures.size() << std::endl;
    return invariantFailures.empty() ? 0 : 1;
}
