#pragma once
#include "StringUtil.h"
#include <string>

#include "CadNode.h"
#include "RobotMotionCore.h"
#include "RobotProgramModel.h"
#include "RobotRuntime.h"

#include <array>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <vector>

#include <map>

class RobotPoseController;

class RobotProgramSimulator {
public:
    enum class Status {
        Idle,
        Running,
        Complete,
        Error
    };

    struct ProgramState {
        double jointSpeedRadPerSec = 30.0 * 3.14159265358979323846 / 180.0;
        double linearSpeedMmPerSec = 100.0;
        double blendRadiusMm = 0.0;
        bool weaveEnabled = false;
        int weaveScheduleIndex = -1;
        RobotMotionCore::WeaveParams weaveInline{};
    };

    struct StepEstimator {
        bool enabled = false;
        std::array<int32_t, 6> zeroSteps{};
        std::array<double, 6> stepsPerDegree{};
    };

    // Everything planning needs from the robot, held by value. buildTrajectory runs on a worker
    // thread, so it must not reach into the scene graph or the package: RobotPoseController::
    // setJoints mutates CadNode transforms that the renderer reads, and the GUI can load a new
    // package at any moment. Field names match OPW6RobotData so the planner body reads the same.
    struct PlanInput {
        RobotMotionCore::RobotModel model{};
        // Geometric FK, used for tool poses so the planner keeps the kinematics it had before
        // planning moved off the GUI thread. The DH FK in model is close but not identical.
        RobotKinematicSnapshot kinematics;
        std::array<double, 6> jointVelocityMaxRadS{};
        std::array<double, 6> jointAccelerationMaxRadS2{};
        std::array<double, 6> jointJerkMaxRadS3{};
        bool hasJointVelocityLimits = false;
        bool hasJointAccelerationLimits = false;
        bool hasJointJerkLimits = false;
        double controlPeriodSec = 0.0;
        double controllerMinTickGapUs = 0.0;
        double singularityThresholdRad = 0.0;
        double defaultJointSpeedDegPerSec = 0.0;
        double defaultLinearSpeedMmPerSec = 0.0;
        double defaultLinearAccelerationMmSec2 = 0.0;
        double defaultLinearJerkMmSec3 = 0.0;
        double defaultToolAngularSpeedRadSec = 0.0;
        double defaultToolAngularAccelerationRadSec2 = 0.0;
        double defaultToolAngularJerkRadSec3 = 0.0;
        bool valid = false;
    };

    struct MotionSettingsOverride {
        double controlPeriodSec = 0.0;
        double singularityThresholdRad = 0.0;
        double defaultJointAccelerationRadSec2 = 0.0;
        double defaultJointJerkRadSec3 = 0.0;
        double defaultLinearAccelerationMmSec2 = 0.0;
        double defaultLinearJerkMmSec3 = 0.0;
        // Multiplies the package per-axis velocity, acceleration and jerk limits, per joint. Zero
        // or one means no scaling for that joint, so a default-constructed override changes
        // nothing.
        std::array<double, 6> dynamicLimitScale{};
        double defaultToolAngularSpeedRadSec = 0.0;
        double defaultToolAngularAccelerationRadSec2 = 0.0;
        double defaultToolAngularJerkRadSec3 = 0.0;
    };

    struct FlatCommand {
        RobotProgramNodeType type = RobotProgramNodeType::Root;
        const RobotProgramNode* node = nullptr;
        int instruction = -1;
        ProgramState state;
        std::array<double, 6> targetJoints{};
        std::array<double, 6> viaJoints{};
        bool hasExternalAxis = false;
        double externalAxisPositionMm = 0.0;
        CadTransform targetPose;
        CadTransform viaPose;
        TriggerData trigger;
        int triggerId = -1;
    };

    struct StepResult {
        Status status = Status::Idle;
        std::array<double, 6> joints{};
        CadTransform tcpPose;
        const RobotProgramNode* activeNode = nullptr;
        bool segmentComplete = false;
        double desiredTcpSpeedMmPerSec = std::numeric_limits<double>::quiet_NaN();
        double profileTcpSpeedMmPerSec = std::numeric_limits<double>::quiet_NaN();
        double actualTcpSpeedMmPerSec = std::numeric_limits<double>::quiet_NaN();
        double tcpDistanceTraveledMm = 0.0;
        std::string message;
    };

    enum class MarkerType {
        Singularity,
        JointFlip,
        PlannerCap,
        // A program trigger, placed at the time it resolved to. Reusing markers means the timeline
        // shows triggers alongside the planner's own events with no new display machinery.
        Trigger
    };

    struct Marker {
        double seconds = 0.0;
        MarkerType type = MarkerType::Singularity;
        std::string message;
    };

    struct PlannedSample {
        double timeSeconds = 0.0;
        std::array<double, 6> joints{};
        std::array<double, 6> plannedJoints{};
        CadTransform tcpPose;
        CadTransform plannedTcpPose;
        // Differentiated from plannedTcpPose, not from the stepped one. A first difference of a
        // quantised position over one control period is not a speed: at 15 mm/s a 5 ms sample advances
        // the tool 0.075 mm, which is less than one motor step at the flange, so most samples read zero
        // and the ones where a step lands read several times the truth. Measured that way a weave whose
        // real envelope is 15 to 23.18 mm/s reported a median of 24.35 and a peak of 59.75, with 598
        // samples at exactly zero.
        double actualTcpSpeedMmPerSec = std::numeric_limits<double>::quiet_NaN();
        double desiredTcpSpeedMmPerSec = std::numeric_limits<double>::quiet_NaN();
        double profileTcpSpeedMmPerSec = std::numeric_limits<double>::quiet_NaN();
        const RobotProgramNode* activeNode = nullptr;
        int32_t completedCommandId = -1;
        bool segmentEnd = false;
        RobotMotionCore::PathSampleKind kind = RobotMotionCore::PathSampleKind::MoveJ;
        // Station-linked external axis, kept outside RobotMotionCore's six-axis robot model.
        bool externalAxisValid = false;
        double externalAxisPositionMm = 0.0;
        double weavePhaseCycles = std::numeric_limits<double>::quiet_NaN();
        double weaveLateralMm = std::numeric_limits<double>::quiet_NaN();
        double steppedTcpSpeedMmPerSec = std::numeric_limits<double>::quiet_NaN();
    };

    struct Statistics {
        std::array<double, 6> maxJointSpeedRadS{};
        std::array<double, 6> maxJointAccelerationRadS2{};
        std::array<double, 6> maxJointJerkRadS3{};
        double maxTcpSpeedMmPerSec = 0.0;
        double maxTcpAccelerationMmS2 = 0.0;
        double maxTcpJerkMmS3 = 0.0;
        double maxToolAngularSpeedRadS = 0.0;
        double maxToolAngularAccelerationRadS2 = 0.0;
        double maxToolAngularJerkRadS3 = 0.0;
        uint16_t trajectoryEdgeCount = 0;
        uint16_t septicEdgeCount = 0;
        double worstWindowSeamSpeedMmPerSec = 0.0;
        std::map<std::string, uint32_t> capReasonSegmentCounts;
        RobotMotionCore::StepExecutorStats stepExecutorStats{};
    };

    struct LiveRunState {
        bool running = false;
        bool looping = false;
        // Instruction the arm is executing, as an index into the program's children, and how many
        // times the program has been round.
        int instruction = -1;
        int completedCycles = 0;
        double runSeconds = 0.0;
        double cycleElapsedSeconds = 0.0;
        double cycleSeconds = 0.0;
        double jointSpeedRadPerSec = 0.0;
        double linearSpeedMmPerSec = 0.0;
        double blendRadiusMm = 0.0;
        bool weaveEnabled = false;
        int weaveScheduleIndex = -1;
        RobotMotionCore::WeaveParams weaveInline{};
        bool externalAxisValid = false;
        double externalAxisPositionMm = 0.0;
        std::string status;
    };

    bool beginLiveRun(const PlanInput& plan, bool loop, std::string* errorMessage);
    void endLiveRun();
    struct LiveRunStep {
        bool running = false;
        bool jointsChanged = false;
    };

    LiveRunStep stepLiveRun(double dtSeconds);
    const LiveRunState& liveRunState() const { return m_liveRun.state; }
    const std::array<double, 6>& liveRunJoints() const { return m_liveRun.joints; }

    void start(const RobotProgramNode* programRoot,
               const std::array<double, 6>& startJoints,
               const ProgramState& initialState);
    void start(const RobotProgramNode* programRoot,
               const std::array<double, 6>& startJoints);
    void stop();
    void setStepEstimator(const StepEstimator& estimator);
    void clearStepEstimator();
    void setMotionSettingsOverride(const MotionSettingsOverride& override);
    void clearMotionSettingsOverride();
    // Configures the station-installed axis paired with this arm. It deliberately is not part of
    // PlanInput or the robot package: the same robot can be installed with or without a rail.
    void configureExternalAxis(bool enabled, double startPositionMm);
    void setWeaveSchedules(const RobotMotionCore::WeaveScheduleTable& schedules);
    double singularityThresholdRad() const;
    bool isRunning() const { return m_status == Status::Running; }
    Status status() const { return m_status; }

    StepResult advance(double dtSeconds, RobotPoseController& poseController);
    const std::vector<PlannedSample>& trajectory() const { return m_samples; }
    const std::vector<Marker>& markers() const { return m_markers; }
    const std::vector<std::string>& warnings() const { return m_warnings; }
    const Statistics& statistics() const { return m_statistics; }
    double durationSeconds() const;
    // Thread-safe: touches only the PlanInput copy and this object's own members.
    bool buildTrajectory(const PlanInput& plan, std::string* errorMessage = nullptr);
    void setProgressCallback(std::function<bool(size_t)> progress);

    static PlanInput planInputFromPoseController(const RobotPoseController& poseController);

    struct Vec3 {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

private:
    const RobotProgramNode* m_root = nullptr;
    Status m_status = Status::Idle;
    ProgramState m_programState;
    std::array<double, 6> m_currentJoints{};
    double m_elapsedSeconds = 0.0;
    size_t m_sampleIndex = 0;
    bool m_planBuilt = false;
    std::vector<PlannedSample> m_samples;
    std::vector<Marker> m_markers;
    std::vector<std::string> m_warnings;
    Statistics m_statistics;
    StepEstimator m_stepEstimator;
    MotionSettingsOverride m_motionSettingsOverride;
    RobotMotionCore::WeaveScheduleTable m_weaveSchedules{};
    double m_effectiveSingularityThresholdRad = 0.0;
    bool m_externalAxisEnabled = false;
    double m_externalAxisStartMm = 0.0;
    std::function<bool(size_t)> m_progressCallback;

    // The shared window runner's scratch. On the heap and reused across windows: MotionSegmentProgram
    // alone is 441 KB, too large for a one-megabyte thread stack. Cleared once, not per window.
    struct MotionScratch {
        RobotMotionCore::MotionProgram program{};
        RobotMotionCore::MotionSegmentProgram segmentProgram{};
        RobotMotionCore::MotionSegmentSampler sampler{};
        RobotMotionCore::MotionBaseSegmentProgram baseSegmentProgram{};
    };
    std::unique_ptr<MotionScratch> m_motionScratch;

    struct LiveRun {
        LiveRunState state;
        PlanInput plan;
        RobotMotionCore::MotionProgramSettings settings{};
        std::vector<FlatCommand> commands;
        RobotMotionCore::MotionCommandRing ring{};
        RobotMotionCore::MotionWindowRunner runner{};
        std::array<double, 6> joints{};
        std::array<double, 6> initialJoints{};
        double externalAxisStartMm = 0.0;
        int64_t nextCommandId = 0;
        // Wall-clock time taken in but not yet turned into samples. A frame is many control periods
        // long and rarely a whole number of them.
        double pendingSeconds = 0.0;
        double windowTimeSec = 0.0;
        double cycleStartSeconds = 0.0;
        bool haveWindow = false;
        // A Stop instruction converts even a station-requested looping run into one finite pass.
        bool stopAtEnd = false;
    };
    LiveRun m_liveRun;

    bool refillLiveRing(std::string* errorMessage);

    StepResult sampleAt(double seconds) const;
    StepResult finish(Status status, const std::string& message = std::string());

    std::vector<FlatCommand> flattenProgram(const RobotKinematicSnapshot& kinematics) const;
    bool toMotionCommand(const FlatCommand& source,
                         int commandId,
                         double externalAxisStartMm,
                         RobotMotionCore::MotionCommand* out,
                         int* unresolvedSchedule) const;
    // Package limits, caller overrides and the step estimator, in the planner's units. Shared so a
    // live run cannot be planned against different limits from the ones a timeline was built with.
    RobotMotionCore::MotionProgramSettings motionSettingsFor(const PlanInput& plan);
};

const std::array<double, 6>* targetJointsForNode(const RobotProgramNode& node);
bool externalAxisTargetForNode(const RobotProgramNode& node, double* positionMm);
bool applyStateInstruction(RobotProgramSimulator::ProgramState& state, const RobotProgramNode& node);

// CadTransform and RobotMotionCore::Transform are both 12 row-major doubles.
RobotMotionCore::Transform toMotionTransform(const CadTransform& transform);
CadTransform fromMotionTransform(const RobotMotionCore::Transform& transform);
