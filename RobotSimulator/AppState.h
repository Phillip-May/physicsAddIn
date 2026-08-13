#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "CadNode.h"
#include "ConveyorPhysics.h"
#include "DragChainPhysics.h"
#include "HardwareIo.h"
#include "JsonCompat.h"
#include "LiveRunDriver.h"
#include "MeshRobotViewer.h"
#include "PlacementSession.h"
#include "ProgramTextIo.h"
#include "RobotLibraryPanel.h"
#include "RobotMotionCore.h"
#include "RobotProgramModel.h"
#include "RobotProgramSimulator.h"
#include "RobotRuntime.h"
#include "StationPackage.h"
#include "StationParameterLinks.h"

// Shared station clock notches. Kept near the other global timing constants so conveyor runtime
// and robot live-run controls use exactly the same speed selection.
inline const double kLiveSpeedFactors[] = {0.5, 1.0, 2.0, 5.0, 10.0, 100.0};
inline const char* kLiveSpeedLabels[] = {"0.5x", "1x", "2x", "5x", "10x", "100x"};
inline constexpr int kLiveSpeedCount =
    static_cast<int>(sizeof(kLiveSpeedFactors) / sizeof(kLiveSpeedFactors[0]));


struct HardwareTraceSample {
    double seconds = 0.0;
    std::array<double, 6> joints{};
    RobotMotionCore::Transform tcpPose{};
};

struct RobotInstance {
    RobotPoseController poseController;

    // How this arm's base orientation is shown - Euler angles in some order, a rotation vector, a
    // quaternion. Per arm because a cell can hold arms whose numbers were copied off different
    // controllers, and those do not agree. Display only: the stored placement is a matrix whatever
    // this says, so nothing here reaches the station file.
    int placementOrientationFormat = 0;
    bool placementWindowOpen = false;

    // Tool target. The WPR fields are relative to the pose captured when the target was last reset.
    CadTransform toolTargetPose;
    CadTransform toolWprBasePose;
    std::array<double, 3> toolWprDegrees{{0.0, 0.0, 0.0}};
    std::string statusText;

    std::vector<RobotProgram> programs;
    int selectedProgram = 0;
    int selectedInstruction = -1;

    std::unique_ptr<RobotProgramNode> pendingLoadRoot;
    std::string pendingLoadName;
    bool pendingLoadRaise = false;

    RobotInstance() { programs.emplace_back(); }

    int selectedProgramIndex() const {
        if (programs.empty()) return 0;
        return std::max(0, std::min(selectedProgram, static_cast<int>(programs.size()) - 1));
    }
    RobotProgram& program() { return programs[static_cast<size_t>(selectedProgramIndex())]; }
    const RobotProgram& program() const { return programs[static_cast<size_t>(selectedProgramIndex())]; }
    double programJointSpeedDegPerSec = 30.0;
    double programLinearSpeedMmPerSec = 100.0;
    double programBlendMm = 0.0;

    RobotProgramSimulator simulator;
    std::vector<TimelineSample> timelineSamples;
    std::vector<TimelineMarker> timelineMarkers;
    std::map<std::string, uint32_t> limitReasonCounts;
    double durationSeconds = 0.0;
    double elapsedSeconds = 0.0;
    // The timeline position the robot pose currently reflects, so a scrub is applied once rather
    // than re-applied every frame - which would otherwise fight the gimbal and the joint sliders.
    double appliedTimelineSeconds = 0.0;
    bool appliedTimelineValid = false;
    bool playing = false;

    RobotProgramSimulator::LiveRunState liveRun;
    int appliedActuatorInstruction = -1;
    int appliedActuatorCycle = -1;

    // Async build. The worker only reads the program tree and its own PlanInput copy, so the
    // invariant is that editing is blocked while it runs.
    std::thread buildThread;
    std::atomic<bool> buildCancel{false};
    bool buildRunning = false;
    size_t buildSentSamples = 0;
    bool startPlaybackAfterBuild = false;
    std::string buildStatus;

    RobotCollisionModel collisionModel;
    std::string collisionText = "Collision: clear";
    std::vector<std::array<double, 6>> toolSolutions;
    std::vector<std::string> toolSolutionLabels;
    int selectedToolSolution = 0;
    // Joints used for the last readout refresh.
    std::array<double, 6> readoutJoints{};
    bool readoutJointsValid = false;

    // Motion Planner editors. Read fresh on each Simulate so the values drive the simulated path
    // without pushing state on every keystroke.
    //
    // Units here are the ones shown in the UI - degrees and milliseconds - and converted to the
    // planner's radians and seconds in currentMotionSettingsOverride().
    double motionControlPeriodMs = 1.0;
    double motionJointAccelDegS2 = 0.0;
    double motionJointJerkDegS3 = 0.0;
    double motionLinearAccelMmS2 = 0.0;
    double motionLinearJerkMmS3 = 0.0;
    double motionToolAngularSpeedDegS = 0.0;
    double motionToolAngularAccelDegS2 = 0.0;
    double motionToolAngularJerkDegS3 = 0.0;
    double motionSingularityDeg = 5.0;
    // Per joint, multiplying the package acceleration and jerk limits. One means the package value
    // stands. The wrist axes carry almost nothing next to the shoulder, so a single figure for the
    // arm had to be set for the worst joint and left the rest slower than they need to be.
    std::array<double, 6> motionDynamicLimitScale{{1.0, 1.0, 1.0, 1.0, 1.0, 1.0}};
    std::string motionSettingsStatus = "-";
    RobotMotionCore::WeaveScheduleTable weaveSchedules{};
    std::array<std::string, RobotMotionCore::kMaxWeaveSchedules> weaveScheduleNames{};
    std::string weaveScheduleStatus = "-";

    std::array<double, 6> packageJointVelocityMaxRadS{};
    std::array<double, 6> packageJointAccelMaxRadS2{};
    std::array<double, 6> packageJointJerkMaxRadS3{};
    bool packageJointLimitsValid = false;
    Json packageMotionSettings = Json::object();

    // Mastering editors. The steps-per-degree defaults are the AR4 nominal values, so a fresh
    // session can master without loading a file first.
    int masterBackoffSteps = 700;
    std::array<double, 6> masterStepsPerDegree{{88.888, 111.111, 111.111, 99.555, 43.720, 44.444}};
    std::array<double, 6> masterOffsetDeg{};
    // Sweep direction per joint, +1 or -1, mirroring the firmware's starting values so the
    // checkboxes read correctly before the first status frame.
    std::array<int, 6> masterDirection{{1, -1, 1, -1, -1, 1}};
    // Set once a box is ticked by hand, so a status frame cannot revert it before the firmware has
    // been told. Cleared when the direction is sent.
    std::array<bool, 6> masterDirectionEdited{};
    // Pulse polarity per joint, mirroring the firmware's starting values, with the same
    // follow-the-robot-until-edited flag the other calibration editors use.
    std::array<bool, 6> jogDirInvert{{false, true, true, false, true, true}};
    std::array<bool, 6> jogDirInvertEdited{};
    // Set while an editor has keyboard focus, so an arriving robot_status does not overwrite the
    // number being typed. Read a frame late, which is harmless: the value only changes on a
    // status frame, and one frame of a stale flag cannot land mid-keystroke.
    std::array<bool, 6> masterStepsPerDegreeEditing{};
    std::array<bool, 6> masterOffsetEditing{};
    RobotMotionCore::RobotModel robotModelOverride{};
    bool robotModelOverrideValid = false;
    std::string robotModelOverrideSource;

    HardwareIo hardware;
    std::string hardwareStatus = "-";
    int selectedPort = -1;
    // Firmware jog magnitude in steps, 1..500 (kJogMaxSteps).
    double hardwareJogSteps = 50.0;
    bool hardwareTrackDuringMove = true;

    std::vector<HardwareTraceSample> hardwareTrace;
    // Firmware time of the moment the robot began to move, which is the trace's zero. Not the time
    // the command was sent: the firmware fills its lookahead queue before it steps anything, so
    // between the button press and the first step the arm is standing still, and starting the clock
    // there would shift the whole trace right of the simulated one by that dead time.
    double hardwareTraceStartSeconds = std::numeric_limits<double>::quiet_NaN();
    // The status frame already sampled, so a frame that redraws without new data adds nothing.
    uint32_t hardwareTraceLastStatusCount = 0;
    // The last status frame seen while the robot was still stationary, held back until motion
    // proves it was the departure point. It becomes the trace's first sample, so the first
    // computed speed has a baseline to difference against and the trace opens at zero.
    HardwareTraceSample hardwareTracePendingSample;
    std::array<int, 6> hardwareTracePendingSteps{};
    bool hardwareTracePendingValid = false;
    // A loaded calibration waiting on the operator's answer to "is the arm at the saved zero?"
    // before any of it is sent to this arm's firmware.
    Json pendingMasteringDocument;
    std::string pendingMasteringSource;
    bool masteringConfirmPending = false;
    // Get status asked for the view to be moved to the robot, and the status frame it is waiting
    // for has not arrived yet. The count is the one on screen when the button was pressed, so a
    // frame already held does not satisfy the request.
    bool poseSyncPending = false;
    uint32_t poseSyncAwaitCount = 0;
};

struct ViewTarget {
    enum class Kind { Sim, Robot };
    Kind kind = Kind::Sim;
    int robot = 0;
};

enum class StationMode { Layout, Program };

enum class StationRunState { Stopped, Running, Paused };
enum class ConveyorSimulationMode { Logical, PhysX };

struct ConveyorWorkpiece {
    uint64_t id = 0;
    std::shared_ptr<CadNode> node;
    CadNode* conveyor = nullptr;
    double progress = 0.0;
    bool forward = true;
    double radiusMm = 60.0;
    double heightMm = 100.0;
    CadVec3 halfExtentsMm{60.0, 50.0, 60.0};
    ConveyorPhysics::BodyHandle physicsBody = 0;
    CadVec3 previousPhysicalCenter;
    bool hasPreviousPhysicalCenter = false;
    CadNode* graspTool = nullptr;
    std::string graspActuatorId;
    CadNode* sourceConveyor = nullptr;
    CadNode* originSpawner = nullptr;
    bool logicalAttachment = false;
};

struct ToolActuatorRuntime {
    CadNode* tool = nullptr;
    MechanismActuatorData* actuator = nullptr;
    std::vector<CadTransform> bindingBaseLocs;
    std::vector<ConveyorPhysics::BodyHandle> physicsJawBodies;
    ConveyorPhysics::BodyHandle physicsGraspBody = 0; // observation only; never constrains the body
    CadTransform physicsGraspTcpRelative;
    bool hasPhysicsGraspTcpRelative = false;
    CadTransform previousForceTarget;
    bool hasPreviousForceTarget = false;
    bool wasClosed = false;
};

struct ConveyorSpawnerRuntime {
    CadNode* conveyor = nullptr;
};

// Disambiguates indices shared by different station object collections.
enum class StationSelectionKind { None, Robot, Mechanism, Accessory };

// Concrete side-panel tab within the broader station mode.
enum class StationPanelTab { Layout, Program, Library };

// Display and camera state stored independently for each view.
struct ViewState {
    bool showVisualMeshes = true;
    bool showJointAxes = true;
    bool showHullOverlay = false;
    bool showToolGimbal = true;
    bool showPathPreview = true;
    bool showSimPath = true;
    bool showDragChainPivots = false;
    bool showMountingHoles = false;
    bool showConveyorDirections = false;
    bool showPlannerCaps = true;
    bool showSingularities = true;
    bool showJointFlips = true;
    MeshRobotViewer::CameraState camera;
    int framedRobot = -1;
};

struct StationGuideState {
    bool showJointAxes = false;
    bool showToolGimbal = false;
    bool showPathPreview = true;
    bool showSimPath = true;
    bool showConveyorDirections = false;
};

struct CameraTween {
    MeshRobotViewer::CameraState from;
    MeshRobotViewer::CameraState to;
    float elapsed = 0.0f;
    float duration = 0.0f;
    bool active() const { return duration > 0.0f; }
};

struct SceneGimbalTarget {
    CadNode* placementNode = nullptr;
    RobotInstance* robot = nullptr;
    bool movesObject = false;
    int programInstruction = -1;
};

// Scene state owned by the ImGui UI: the CAD tree, the renderer that walks it, the robots found
// in it, and the single hardware connection.
struct SceneState {
    std::shared_ptr<CadNode> root;
    std::unique_ptr<MeshRobotViewer> viewer;
    std::string error;
    bool loadAttempted = false;
    std::string packageLabel = "No package";

    ViewState simView;
    ViewState robotView;
    CameraTween cameraTween;

    // Stable addresses are required by trajectory workers; RobotInstance is non-movable.
    std::vector<std::unique_ptr<RobotInstance>> robots;
    // UI-only state for robotless stations; never simulated or serialized.
    std::unique_ptr<RobotInstance> robotlessUi = std::make_unique<RobotInstance>();

    // Station-level axes stay separate from the six-joint arm runtimes. Each controller writes the
    // carriage transform; a robot mounted below it follows through the scene hierarchy.
    std::vector<GantryPoseController> gantries;
    std::vector<DragChainPoseController> dragChains;
    std::vector<DragChainPhysics> dragChainPhysics;

    StationDocument station;
    std::string stationSource;

    // Which view is up, and which arm the robot view is pointed at. `robot` stays meaningful in the
    // sim view: playback, the readout refresh and the build poll all run every frame regardless of
    // what is on screen, and they all need an instance to run against.
    ViewTarget view;
    StationMode stationMode = StationMode::Layout;
    StationPanelTab stationPanelTab = StationPanelTab::Layout;
    bool stationPanelTabSelectionPending = false;
    StationGuideState layoutGuides;
    StationGuideState programGuides{false, true, true, true, false};
    StationSelectionKind stationSelectionKind = StationSelectionKind::None;
    int stationSelectionIndex = -1;
    // Retained across frames to distinguish selection from a threshold-crossing drag.
    StationSelectionKind objectDragCandidateKind = StationSelectionKind::None;
    int objectDragCandidateIndex = -1;

    // Requested and measured live speed are tracked separately.
    int liveSpeedIndex = 1;
    double measuredLiveSpeed = 0.0;
    bool measuredLiveSpeedValid = false;

    StationRunState stationRunState = StationRunState::Stopped;
    ConveyorSimulationMode defaultConveyorMode = ConveyorSimulationMode::Logical;
    std::vector<ConveyorWorkpiece> conveyorWorkpieces;
    std::vector<ConveyorSpawnerRuntime> conveyorSpawners;
    std::vector<ToolActuatorRuntime> toolActuators;
    ConveyorPhysics conveyorPhysics;
    // One fixed-step clock advances all conveyor subsystems.
    double conveyorTimeAccumulatorSeconds = 0.0;
    uint64_t nextWorkpieceId = 1;
    std::string stationSimulationStatus;

    // The live-run worker publishes joint updates to the main thread.
    LiveRunDriver liveRunDriver;
#ifndef __EMSCRIPTEN__
    std::thread liveRunThread;
    std::atomic<bool> liveRunThreadStop{false};
#endif

    // Layout placement mode. The selected station object owns the transform gimbal, whether that
    // object is a robot base frame, gantry package root, or static accessory.
    bool moveObjects = false;

    // Rebuilt every frame so gimbals cannot retain stale selection targets.
    std::vector<SceneGimbalTarget> gimbalTargets;

    bool saveStationUsingBuiltins = false;

    // Screen-space mounting-hole tolerance used while a model is dragged out of the Library.
    // Relative to the shorter 3D viewport edge, so resizing the dock does not change how precise
    // the gesture feels.
    float mountingSnapScreenPercent = 2.5f;

    SceneState() {
        simView.showJointAxes = false;
        simView.showToolGimbal = false;
        simView.showPathPreview = true;
        simView.showSimPath = true;
    }

    int activeRobotIndex() const {
        if (robots.empty()) return 0;
        return std::max(0, std::min(view.robot, static_cast<int>(robots.size()) - 1));
    }
    RobotInstance& activeRobot() {
        return robots.empty() ? *robotlessUi : *robots[static_cast<size_t>(activeRobotIndex())];
    }
    const RobotInstance& activeRobot() const {
        return robots.empty() ? *robotlessUi : *robots[static_cast<size_t>(activeRobotIndex())];
    }

    ViewState& activeViewState() {
        return view.kind == ViewTarget::Kind::Sim ? simView : robotView;
    }

    void setStationMode(StationMode nextMode) {
        if (stationMode == nextMode) return;

        StationGuideState& outgoing = stationMode == StationMode::Layout
            ? layoutGuides : programGuides;
        outgoing.showJointAxes = simView.showJointAxes;
        outgoing.showToolGimbal = simView.showToolGimbal;
        outgoing.showPathPreview = simView.showPathPreview;
        outgoing.showSimPath = simView.showSimPath;
        outgoing.showConveyorDirections = simView.showConveyorDirections;

        stationMode = nextMode;
        const StationGuideState& incoming = stationMode == StationMode::Layout
            ? layoutGuides : programGuides;
        simView.showJointAxes = incoming.showJointAxes;
        simView.showToolGimbal = incoming.showToolGimbal;
        simView.showPathPreview = incoming.showPathPreview;
        simView.showSimPath = incoming.showSimPath;
        simView.showConveyorDirections = incoming.showConveyorDirections;
    }

};

extern SceneState g_scene;
extern RobotLibraryPanel g_robotLibrary;
extern CadNode* g_defaultFloorNode;

struct LibraryRobotPlacement {
    std::string packagePath;
    std::string variantKey;
    std::string displayName;
    std::shared_ptr<CadNode> frame;
    std::shared_ptr<CadNode> packageRoot;
    CadNode* robotNode = nullptr;
    CadNode* gantryNode = nullptr;
    CadNode* accessoryNode = nullptr;
    placementsession::Session session;
    // Library assets are temporary trees until delivery. Existing station objects stay in their
    // tree while they use the same preview/snap solver, and cancellation restores this transform.
    bool movingExisting = false;
    StationSelectionKind existingKind = StationSelectionKind::None;
    int existingIndex = -1;
    CadTransform originalLocalPose;

    bool active() const { return frame != nullptr; }
    bool snapped() const { return session.last().snapped; }
    const std::string& status() const { return session.last().status; }
    const CadTransform& worldPose() const { return session.last().worldPose; }
};

extern LibraryRobotPlacement g_libraryPlacement;

inline RobotInstance& activeRobot() { return g_scene.activeRobot(); }
