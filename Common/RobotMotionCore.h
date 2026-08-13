#pragma once

#include <math.h>
#include <stdint.h>
#include <string.h>

#ifndef ROBOT_MOTION_CORE_COLD
#if defined(FLASHMEM)
#define ROBOT_MOTION_CORE_STRINGIFY_DETAIL(x) #x
#define ROBOT_MOTION_CORE_STRINGIFY(x) ROBOT_MOTION_CORE_STRINGIFY_DETAIL(x)
#define ROBOT_MOTION_CORE_COLD __attribute__((section(".flashmem.rmc." ROBOT_MOTION_CORE_STRINGIFY(__COUNTER__))))
#else
#define ROBOT_MOTION_CORE_COLD
#endif
#endif

#ifndef ROBOT_MOTION_CORE_RAM2
#if defined(DMAMEM)
#define ROBOT_MOTION_CORE_RAM2 DMAMEM
#else
#define ROBOT_MOTION_CORE_RAM2
#endif
#endif

namespace RobotMotionCore {

constexpr uint8_t kAxisCount = 6;
constexpr uint32_t kMotionSettingsSchemaVersion = 1;
constexpr uint8_t kMaxWeaveSchedules = 8;
constexpr uint32_t kWeaveScheduleSchemaVersion = 1;
constexpr double kDefaultMoveJSpeedDegPerSec = 10.0;
constexpr double kDefaultMoveJMaxSpeedDegPerSec = 180.0;
constexpr double kDefaultMoveJSampleDeg = 1.0;
constexpr uint32_t kDefaultMoveJMinTickGapUs = 80;
constexpr uint32_t kDefaultMoveJPlanMaxTicks = 250000;

enum class RejectCode {
    Ok,
    BadTarget,
    JointLimit,
    NotMastered,
    BadStepsPerDeg,
    CurrentJointLimit,
    RoundedJointLimit,
    LimitSwitch,
    TooManyTicks,
    BadSpeed,
    BadDuration,
    SampleJointLimit,
    Estop,
    ModelNotLoaded,
    BadPose,
    NoIkSolution,
    Singularity,
    SegmentFailed,
    TargetConfigMismatch,
    // Genuine infeasibility: the pattern cannot be traced there, or its joint budget cannot be
    // reserved. Slowing down will not help and the program has to change.
    WeaveInfeasible,
    WeaveNotConfigured,
    // A weave asked for on a joint move, which has no direction of travel to weave across.
    WeaveOnJointMove,
    WeaveDerateExhausted,
    // The two samples either side of a lookahead boundary do not meet: the planner's windows disagree
    // about where the arm is. Distinct from BadSpeed, which is a limit exceeded inside one window.
    WindowSeamDiscontinuity,
    // Derating for the weave grew the plan past the node budget. A capacity limit reached as a
    // consequence of slowing the pattern down, not because the program was too long.
    WeaveDerateOverflow
};

inline const char* rejectCodeName(RejectCode code) {
    switch (code) {
    case RejectCode::Ok: return "ok";
    case RejectCode::BadTarget: return "bad_target";
    case RejectCode::JointLimit: return "joint_limit";
    case RejectCode::NotMastered: return "not_mastered";
    case RejectCode::BadStepsPerDeg: return "bad_steps_per_deg";
    case RejectCode::CurrentJointLimit: return "current_joint_limit";
    case RejectCode::RoundedJointLimit: return "rounded_joint_limit";
    case RejectCode::LimitSwitch: return "limit_switch";
    case RejectCode::TooManyTicks: return "too_many_ticks";
    case RejectCode::BadSpeed: return "bad_speed";
    case RejectCode::BadDuration: return "bad_duration";
    case RejectCode::SampleJointLimit: return "sample_joint_limit";
    case RejectCode::Estop: return "estop";
    case RejectCode::ModelNotLoaded: return "model_not_loaded";
    case RejectCode::BadPose: return "bad_pose";
    case RejectCode::NoIkSolution: return "no_ik_solution";
    case RejectCode::Singularity: return "singularity";
    case RejectCode::SegmentFailed: return "segment_failed";
    case RejectCode::TargetConfigMismatch: return "target_config_mismatch";
    case RejectCode::WeaveInfeasible: return "weave_infeasible";
    case RejectCode::WeaveNotConfigured: return "weave_not_configured";
    case RejectCode::WeaveOnJointMove: return "weave_on_joint_move";
    case RejectCode::WeaveDerateExhausted: return "weave_derate_exhausted";
    case RejectCode::WindowSeamDiscontinuity: return "window_seam_discontinuity";
    case RejectCode::WeaveDerateOverflow: return "weave_derate_overflow";
    }
    return "unknown";
}

struct JointLimitsDeg {
    double min[kAxisCount];
    double max[kAxisCount];
};

struct MoveJInput {
    double targetDeg[kAxisCount];
    int32_t currentSteps[kAxisCount];
    int32_t zeroSteps[kAxisCount];
    double stepsPerDegree[kAxisCount];
    uint8_t mastered[kAxisCount];
    uint8_t limitActive[kAxisCount];
    int8_t limitDirection[kAxisCount];
    JointLimitsDeg limits;
    double speedDegPerSec;
    double defaultSpeedDegPerSec;
    double maxSpeedDegPerSec;
    double sampleDeg;
    uint32_t minTickGapUs;
    uint32_t maxTicks;
    uint8_t estopActive;
};

inline void applyDefaultMoveJPlannerSettings(MoveJInput* input) {
    if (!input) return;
    input->defaultSpeedDegPerSec = kDefaultMoveJSpeedDegPerSec;
    input->maxSpeedDegPerSec = kDefaultMoveJMaxSpeedDegPerSec;
    input->sampleDeg = kDefaultMoveJSampleDeg;
    input->minTickGapUs = kDefaultMoveJMinTickGapUs;
    input->maxTicks = kDefaultMoveJPlanMaxTicks;
}

struct MoveJPlan {
    double targetDeg[kAxisCount];
    int32_t startSteps[kAxisCount];
    int32_t targetSteps[kAxisCount];
    int32_t deltaSteps[kAxisCount];
    uint32_t absSteps[kAxisCount];
    int32_t direction[kAxisCount];
    uint32_t maxStepCount;
    uint32_t tickGapUs;
    double speedDegPerSec;
    double durationSec;
    double maxDeltaDeg;
};

struct Transform {
    double values[12];
};

struct RobotModel {
    double dhm[24];
    double qHome[kAxisCount];
    double qMin[kAxisCount];
    double qMax[kAxisCount];
    double dhmSigns[kAxisCount];
    Transform toolBindPose;
    uint8_t valid;
};

enum class SingularityPolicy : uint8_t {
    SlowDown = 0,
    Abort = 1
};

inline bool singularityPolicyAborts(SingularityPolicy policy) {
    return policy == SingularityPolicy::Abort;
}

struct MoveLInput {
    RobotModel model;
    double startQ[kAxisCount];
    Transform targetTcp;
    double targetConfigQ[kAxisCount];
    double speedMmPerSec;
    double sampleMm;
    double singularityThresholdRad;
    SingularityPolicy singularityPolicy;
    uint8_t requireTargetConfig;
    double targetConfigToleranceRad;
};

struct MoveLPlan {
    double endpointQ[kAxisCount];
    Transform startTcp;
    double lineLengthMm;
    double durationSec;
    double sampleDurationSec;
    uint32_t sampleCount;
};

struct MoveLStream {
    MoveLInput input;
    MoveLPlan plan;
    double previousQ[kAxisCount];
    uint32_t nextSample;
};

struct MoveLBlendPlan {
    uint8_t enabled;
    uint8_t useQuintic;
    Transform cornerTcp;
    Transform entryTcp;
    Transform exitTcp;
    double center[3];
    double startUnit[3];
    double planeUnit[3];
    double entryPoint[3];
    double exitPoint[3];
    double incomingUnit[3];
    double outgoingUnit[3];
    double tangentScaleMm;
    double radiusMm;
    double sweepRad;
    double trimMm;
    double maxDeviationMm;
    double actualDeviationMm;
    double contourDeviationMm;
    double translationLengthMm;
    double arcLengthSamples[9];
};

// Weaving as the arc welding vendors model it: a lateral oscillation superimposed on the taught
// path, parameterised once into a numbered schedule and then switched on and off by index from the
// program.
enum class WeaveShape : uint8_t {
    None = 0,
    Sine,
    Zigzag,
    Trapezoid,
    Circular,
    FigureEight,
    LShape
};

// Frequency mode reserves joint budget; wavelength mode scales demand with path speed.
enum class WeaveRateMode : uint8_t {
    Unset = 0,
    Frequency,
    Wavelength
};

// Stands in for a schedule index when the program carried its own inline parameters instead of
// naming a stored schedule, the way FANUC's inline Weave Sine[] stands beside Weave Sched[n].
constexpr uint8_t kWeaveScheduleInline = 0xFF;

// MotionSegment::commandIndex when a segment belongs to no command in the program.
constexpr uint8_t kNoMotionCommandIndex = 0xFF;

struct WeaveParams {
    WeaveShape shape;
    WeaveRateMode rateMode;
    uint8_t enabled;
    uint8_t scheduleIndex;
    double frequencyHz;
    double wavelengthMm;
    double amplitudeLeftMm;
    double amplitudeRightMm;
    // Out of the weave plane, which is what separates a flat zigzag from a circle, a figure of
    // eight or a spiral.
    double elevationMm;
    // Rotates the weave plane about the direction of travel.
    double planeAngleDeg;
    // Shifts the centre of the oscillation off the taught line.
    double biasMm;
    // Seconds when rateMode is Frequency, millimetres when it is Wavelength.
    double dwellLeft;
    double dwellCenter;
    double dwellRight;
};

struct WeaveScheduleTable {
    WeaveParams schedules[kMaxWeaveSchedules];
    uint8_t valid[kMaxWeaveSchedules];
};

inline const char* weaveShapeName(WeaveShape shape) {
    switch (shape) {
    case WeaveShape::None: return "none";
    case WeaveShape::Sine: return "sine";
    case WeaveShape::Zigzag: return "zigzag";
    case WeaveShape::Trapezoid: return "trapezoid";
    case WeaveShape::Circular: return "circular";
    case WeaveShape::FigureEight: return "figure_eight";
    case WeaveShape::LShape: return "l_shape";
    }
    return "unknown";
}

inline const char* weaveRateModeName(WeaveRateMode mode) {
    switch (mode) {
    case WeaveRateMode::Unset: return "unset";
    case WeaveRateMode::Frequency: return "frequency";
    case WeaveRateMode::Wavelength: return "wavelength";
    }
    return "unknown";
}

enum class MotionTriggerReference : uint8_t {
    // The taught point. Default, matching KUKA's and FANUC's reference for a signed offset, and
    // matching how a program reads: a MoveL is identified by where it ends up.
    End = 0,
    Start = 1
};

struct MotionTrigger {
    int32_t id;                          // negative means this move carries no trigger
    MotionTriggerReference reference;
    // Along the seam, not along the tool's actual travel. With a weave running the tool covers
    // substantially more ground than the seam does, and weave amplitude is editable independently of
    // the program, so measuring on the woven path would move every trigger whenever someone changed
    // the weave.
    double distanceMm;
    double timeMs;
};

inline MotionTrigger defaultMotionTrigger() {
    MotionTrigger trigger = {};
    trigger.id = -1;
    trigger.reference = MotionTriggerReference::End;
    return trigger;
}

inline bool motionTriggerIsActive(const MotionTrigger& trigger) {
    return trigger.id >= 0;
}

// Declared with the other per-command tables further down, where kMaxMotionCommands and isFinite
// both exist.
struct PendingMotionTriggerQueue;

enum class MotionCommandType : uint8_t {
    MoveJ,
    MoveL
};

enum class PathSampleKind : uint8_t {
    MoveJ,
    MoveLLine,
    MoveLBlend
};

inline bool pathSampleKindUsesCartesianSingularity(PathSampleKind kind) {
    return kind == PathSampleKind::MoveLLine || kind == PathSampleKind::MoveLBlend;
}

enum class MotionSegmentKind : uint8_t {
    MoveJ,
    MoveLLine,
    MoveLBlend
};

enum MotionCapReason : uint32_t {
    MotionCapNone = 0,
    MotionCapCommandSpeed = 1u << 0,
    MotionCapJointVelocity = 1u << 1,
    MotionCapJointAcceleration = 1u << 2,
    MotionCapJerk = 1u << 3,
    MotionCapSingularity = 1u << 4,
    MotionCapBlendGeometry = 1u << 5,
    MotionCapQueueBoundary = 1u << 6,
    MotionCapFinePoint = 1u << 7,
    MotionCapLookahead = 1u << 8,
    MotionCapStepRate = 1u << 9,
    MotionCapPathAcceleration = 1u << 10,
    MotionCapWeave = 1u << 11
};

// Seams get their own tolerance, looser than kVerifierTolerance's half a percent. A join legitimately
// carries one control period of ordinary motion plus whatever the two windows round differently, and
// the failure this exists to catch is an axis moving many times its limit, not a fraction over.
constexpr double kMotionSeamTolerance = 1.05;

constexpr uint8_t kMotionLookaheadQueuedCommands = 24;
constexpr uint8_t kMotionLookaheadRetainedCommands = 16;
constexpr uint8_t kMotionLookaheadExecuteCommands =
    kMotionLookaheadQueuedCommands - kMotionLookaheadRetainedCommands;
constexpr uint8_t kMaxMotionCommands = kMotionLookaheadQueuedCommands;
constexpr uint8_t kMaxBaseMotionSegments = 64;
constexpr uint16_t kMaxMotionSegments = 240;
constexpr uint16_t kMaxMotionPathNodes = static_cast<uint16_t>(kMaxMotionSegments) + 1;

struct MotionCommand {
    MotionCommandType type;
    int32_t id;
    Transform targetTcp;
    double targetQ[kAxisCount];
    double jointSpeedDegPerSec;
    double linearSpeedMmPerSec;
    double blendMm;
    double sampleMm;
    // Optional extra coordinated path demand in the command's native path units: radians for
    // MoveJ and millimetres for MoveL. The station simulator uses this for a linked linear axis
    // without pretending that the robot model itself has seven joints. A demand larger than the
    // six-axis path stretches the shared timed profile, so every participating axis accelerates,
    // cruises and decelerates together. Zero preserves the ordinary six-axis command exactly.
    double coordinatedPathLength;
    // Either names a stored schedule or carries the program's inline block, resolved once by
    // resolveWeaveParams so the simulator and the firmware cannot read the same index differently.
    WeaveParams weave;
    MotionTrigger trigger;
};

struct MotionProgram {
    MotionCommand commands[kMaxMotionCommands];
    uint8_t count;
    uint8_t continuesBeforeStart;
    // Separate from continuesBeforeStart, because speed continuity and pattern continuity are
    // different questions and only one of them is about the bead. The profile may legitimately come
    // to rest at a window boundary while the seam and the weave run straight on through it.
    uint8_t continuesWeaveBeforeStart;
    uint8_t continuesAfterEnd;
    double initialPathSpeed;
    // Weave phase, in cycles, at the first sample of this window. The planner replans every
    // motionLookaheadExecutableCount commands, so a window that restarted the phase at zero would
    // put a visible step in the bead at every window boundary. Carried in alongside the path speed,
    // which crosses the same boundary for the same reason.
    double initialWeavePhase;
};

struct MotionCommandRing {
    MotionCommand commands[kMotionLookaheadQueuedCommands];
    uint8_t head;
    uint8_t count;
    uint8_t finalCommandQueued;
};

inline void clearMotionCommandRing(MotionCommandRing* ring) {
    if (!ring) return;
    ring->head = 0;
    ring->count = 0;
    ring->finalCommandQueued = 0;
}

inline uint8_t motionCommandRingFree(const MotionCommandRing& ring) {
    return ring.count < kMotionLookaheadQueuedCommands
        ? static_cast<uint8_t>(kMotionLookaheadQueuedCommands - ring.count)
        : 0;
}

inline bool motionCommandRingPush(MotionCommandRing* ring, const MotionCommand& command, bool finalCommand) {
    if (!ring || ring->count >= kMotionLookaheadQueuedCommands) return false;
    const uint8_t index = static_cast<uint8_t>((ring->head + ring->count) % kMotionLookaheadQueuedCommands);
    ring->commands[index] = command;
    ++ring->count;
    if (finalCommand) ring->finalCommandQueued = 1;
    return true;
}

inline const MotionCommand* motionCommandRingAt(const MotionCommandRing& ring, uint8_t index) {
    if (index >= ring.count) return nullptr;
    return &ring.commands[(ring.head + index) % kMotionLookaheadQueuedCommands];
}

inline void motionCommandRingPopFront(MotionCommandRing* ring, uint8_t count) {
    if (!ring || count == 0) return;
    if (count >= ring->count) {
        clearMotionCommandRing(ring);
        return;
    }
    ring->head = static_cast<uint8_t>((ring->head + count) % kMotionLookaheadQueuedCommands);
    ring->count = static_cast<uint8_t>(ring->count - count);
}

inline bool buildMotionProgramFromRing(const MotionCommandRing& ring,
                                       uint8_t executeCount,
                                       bool carriesSpeed,
                                       double carriedPathSpeed,
                                       double carriedWeavePhase,
                                       MotionProgram* program) {
    if (!program || executeCount == 0 || executeCount > ring.count) return false;
    memset(program, 0, sizeof(*program));
    program->count = ring.count;
    program->continuesBeforeStart = carriesSpeed ? 1 : 0;
    program->initialPathSpeed = carriedPathSpeed;
    program->initialWeavePhase = carriedWeavePhase;
    // A carried phase is its own evidence that the run continues: both callers pass zero exactly
    // when the executed prefix ended with the weave off, and a live run's phase has advanced past
    // zero. Gating this on carriesSpeed instead dropped the phase back to zero every
    // kMotionLookaheadExecuteCommands commands, stepping the torch sideways by up to a full
    // amplitude - and because each window is verified on its own, nothing ever saw it.
    program->continuesWeaveBeforeStart = carriedWeavePhase > 0.0 ? 1 : 0;
    program->continuesAfterEnd = ring.finalCommandQueued && ring.count <= executeCount ? 0 : 1;
    for (uint8_t i = 0; i < ring.count; ++i) {
        const MotionCommand* command = motionCommandRingAt(ring, i);
        if (!command) return false;
        program->commands[i] = *command;
    }
    return true;
}

inline uint8_t motionLookaheadExecutableCount(const MotionCommandRing& ring) {
    if (ring.count == 0) return 0;
    if (ring.finalCommandQueued) return ring.count;
    return ring.count > kMotionLookaheadRetainedCommands
        ? static_cast<uint8_t>(ring.count - kMotionLookaheadRetainedCommands)
        : 0;
}

struct MotionProgramSettings {
    double defaultJointSpeedDegPerSec;
    double defaultLinearSpeedMmPerSec;
    double defaultJointAccelerationRadSec2;
    double defaultLinearAccelerationMmSec2;
    double defaultJointJerkRadSec3;
    double defaultLinearJerkMmSec3;
    double defaultToolAngularSpeedRadSec;
    double defaultToolAngularAccelerationRadSec2;
    double defaultToolAngularJerkRadSec3;
    double jointVelocityLimitRadSec[kAxisCount];
    double jointAccelerationLimitRadSec2[kAxisCount];
    double jointStepsPerDegree[kAxisCount];
    double jointStepRateLimitStepsSec[kAxisCount];
    double sampleMm;
    double jointSampleDeg;
    double controlPeriodSec;
    double singularityThresholdRad;
    SingularityPolicy singularityPolicy;
    uint8_t verifySampledDynamics;
    uint32_t minBlendSamples;
};

struct MotionTimeProfile {
    double phaseDuration[7];
    double startSpeed;
    double endSpeed;
    double peakSpeed;
    double length;
    double accelTimeSec;
    double cruiseTimeSec;
    double decelTimeSec;
    double accelDistance;
    double cruiseDistance;
};

struct MotionSegment {
    MotionSegmentKind kind;
    int32_t commandId;
    int32_t completedCommandId;
    Transform startTcp;
    Transform endTcp;
    MoveLBlendPlan blendPlan;
    double startQ[kAxisCount];
    double endQ[kAxisCount];
    double length;
    double curvature;
    double pathParamStart;
    double pathParamEnd;
    double globalPathStart;
    double globalPathEnd;
    double orientationLengthRad;
    double orientationRatioRadPerPathUnit;
    double nominalSpeed;
    double commandSpeed;
    double acceleration;
    double commandAcceleration;
    double jerk;
    double startSpeed;
    double endSpeed;
    double peakSpeed;
    double startTimeSec;
    double durationSec;
    double accelTimeSec;
    double cruiseTimeSec;
    double decelTimeSec;
    double accelDistance;
    double cruiseDistance;
    double singularityMarginRad;
    double speedAfterCommand;
    double speedAfterJointVelocity;
    double speedAfterJointAcceleration;
    double speedAfterPathAcceleration;
    double speedAfterSingularity;
    double speedAfterStepRate;
    double speedAfterJerk;
    double speedAfterWeave;
    double finalNominalSpeed;
    double accelerationAfterCommand;
    double accelerationAfterJointAcceleration;
    double accelerationAfterJerk;
    uint32_t capReasonMask;
    uint32_t activeSpeedLimitMask;
    uint32_t activeAccelerationLimitMask;
    uint32_t contextMask;
    MotionTimeProfile timeProfile;
    uint8_t finePoint;
    uint8_t blended;
    uint8_t commandIndex;
};

struct MotionPathNode {
    Transform tcp;
    double q[kAxisCount];
    double s;
    double localT;
    double speedLimit;
    double solvedSpeed;
    double accelLimit;
    double jerkLimit;
    double curvature;
    double orientationRatioRadPerPathUnit;
    double singularityMarginRad;
    uint32_t capReasonMask;
    uint8_t finePoint;
    uint16_t segmentIndex;
    int32_t commandId;
    int32_t completedCommandId;
};

struct MotionPathGrid {
    MotionPathNode nodes[kMaxMotionPathNodes];
    double edgeSpeedLimit[kMaxMotionPathNodes - 1];
    uint32_t edgeCapReasonMask[kMaxMotionPathNodes - 1];
    uint16_t count;
    double totalLength;
};

struct MotionTrajectoryNode {
    double timeSec;
    double pathS;
    double speed;
    double acceleration;
    double jerk;
    uint16_t pathNodeIndex;
};

struct MotionTrajectoryEdge {
    double startTimeSec;
    double durationSec;
    double startS;
    double endS;
    double length;
    double startSpeed;
    double endSpeed;
    double peakSpeed;
    double speedLimit;
    double acceleration;
    double jerk;
    double startAcceleration;
    double endAcceleration;
    double startJerk;
    double endJerk;
    double quinticC3;
    double quinticC4;
    double quinticC5;
    double quinticC6;
    double quinticC7;
    uint16_t startNodeIndex;
    uint16_t endNodeIndex;
    uint16_t pathSegmentIndex;
    int32_t commandId;
    int32_t completedCommandId;
    uint32_t capReasonMask;
    uint8_t finePoint;
    uint8_t blended;
    uint8_t useQuintic;
};

struct MotionTrajectory {
    MotionTrajectoryNode nodes[kMaxMotionPathNodes];
    MotionTrajectoryEdge edges[kMaxMotionPathNodes - 1];
    uint16_t count;
    uint16_t edgeCount;
    double totalDurationSec;
};

enum class MotionVerificationKind : uint8_t {
    None,
    JointVelocity,
    JointStepRate,
    TcpSpeed,
    ToolAngularSpeed,
    TcpAcceleration,
    ToolAngularAcceleration,
    TcpJerk,
    ToolAngularJerk,
    JointAcceleration,
    JointJerk
};

struct MotionVerificationFailure {
    MotionVerificationKind kind;
    double timeSec;
    double observed;
    double limit;
    uint16_t trajectoryEdgeIndex;
    uint16_t pathSegmentIndex;
    uint8_t joint;
};

inline const char* motionVerificationKindName(MotionVerificationKind kind) {
    switch (kind) {
    case MotionVerificationKind::None: return "none";
    case MotionVerificationKind::JointVelocity: return "joint_velocity";
    case MotionVerificationKind::JointStepRate: return "joint_step_rate";
    case MotionVerificationKind::TcpSpeed: return "tcp_speed";
    case MotionVerificationKind::ToolAngularSpeed: return "tool_angular_speed";
    case MotionVerificationKind::TcpAcceleration: return "tcp_acceleration";
    case MotionVerificationKind::ToolAngularAcceleration: return "tool_angular_acceleration";
    case MotionVerificationKind::TcpJerk: return "tcp_jerk";
    case MotionVerificationKind::ToolAngularJerk: return "tool_angular_jerk";
    case MotionVerificationKind::JointAcceleration: return "joint_acceleration";
    case MotionVerificationKind::JointJerk: return "joint_jerk";
    }
    return "unknown";
}

struct MotionSegmentProgram {
    MotionSegment segments[kMaxMotionSegments];
    MotionPathGrid pathGrid;
    MotionTrajectory trajectory;
    MotionVerificationFailure verificationFailure;
    uint8_t count;
    uint8_t continuesBeforeStart;
    uint8_t continuesWeaveBeforeStart;
    uint8_t continuesAfterEnd;
    double initialPathSpeed;
    uint16_t polynomialEdgeCount;
    double totalDurationSec;
    WeaveParams commandWeave[kMaxMotionCommands];
    // Where the contiguous run of weaving that covers this command began, so the phase at any
    // sample is a pure function of the sample and never depends on how the sampler got there. The
    // verifier restarts from zero and the executor seeks forward, and both must agree.
    double weaveAnchorS[kMaxMotionCommands];
    double weaveAnchorTimeSec[kMaxMotionCommands];
    double weaveAnchorPhase[kMaxMotionCommands];
    // Where the run ends, so the amplitude can be taken back to zero over the last cycle instead of
    // being dropped. kWeaveRunOpenEnded when the run reaches the end of this window and the next one
    // will carry it on, because a lookahead boundary is not somewhere the bead should taper.
    double weaveRunEndS[kMaxMotionCommands];
    double weaveRunEndTimeSec[kMaxMotionCommands];
    double initialWeavePhase;

    // Triggers, per command, alongside the time each one resolved to. Negative means this command
    // carries no trigger or its trigger could not be placed. Resolved once the trajectory is solved,
    // so the sampler only has to compare a time.
    MotionTrigger commandTrigger[kMaxMotionCommands];
    double triggerTimeSec[kMaxMotionCommands];
    uint8_t triggerClamped[kMaxMotionCommands];
};

struct MotionBaseSegmentProgram {
    MotionSegment segments[kMaxBaseMotionSegments];
    uint8_t count;
};

struct MotionSegmentSampler {
    const MotionSegmentProgram* program;
    RobotModel model;
    double previousQ[kAxisCount];
    int32_t pendingCompletedCommandId;
    uint16_t trajectoryEdgeIndex;
    uint8_t pendingFinePoint;
};

struct PathSample {
    uint8_t valid;
    double timeSec;
    Transform tcp;
    double q[kAxisCount];
    int32_t commandId;
    int32_t completedCommandId;
    double curvature;
    double profileSpeed;
    uint32_t capReasonMask;
    uint16_t pathSegmentIndex;
    uint16_t trajectoryEdgeIndex;
    // Normalized progress through the active path segment after the timed profile has been sampled.
    // This is distinct from wall-clock fraction: acceleration and deceleration make those differ.
    double pathProgress;
    PathSampleKind kind;
    uint8_t finePoint;
    uint8_t blended;
    // Cycles, monotonic across segments and across lookahead windows. Reported so the caller can
    // carry it into the next window and so the simulator can plot the weave against the seam.
    double weavePhase;
    // Lateral then out-of-plane, in millimetres, as applied to this sample's pose. Zero when the
    // weave is off, which also makes it the signal the sample decimator uses to stop thinning.
    double weaveOffsetMm[2];
    // The joints on the taught path, before the weave displaced the pose. Equal to q when the weave
    // is off.
    double baseQ[kAxisCount];
    Transform baseTcp;
};

// The joint state the next lookahead window should plan from, given where the arm is at the seam.
inline void motionReplanSeedQ(const PathSample& boundary,
                              const double actualQ[kAxisCount],
                              double outQ[kAxisCount]) {
    if (!actualQ || !outQ) return;
    for (uint8_t joint = 0; joint < kAxisCount; ++joint) {
        outQ[joint] = actualQ[joint] + (boundary.baseQ[joint] - boundary.q[joint]);
    }
}

// motionReplanSeedQExact, which needs the solver, is defined below it.

#if !defined(ARDUINO)
struct MotionKinematicStats {
    double maxJointSpeedRadS[kAxisCount];
    double maxJointAccelerationRadS2[kAxisCount];
    double maxJointJerkRadS3[kAxisCount];
    double maxTcpSpeedMmPerSec;
    double maxTcpAccelerationMmS2;
    double maxTcpJerkMmS3;
    double maxToolAngularSpeedRadS;
    double maxToolAngularAccelerationRadS2;
    double maxToolAngularJerkRadS3;
};

struct MotionKinematicStatsObserver {
    MotionKinematicStats stats;
    PathSample lastSample;
    double lastJointVelocity[kAxisCount];
    double lastJointAcceleration[kAxisCount];
    double lastLinearVelocity[3];
    double lastLinearAcceleration[3];
    double lastAngularVelocity[3];
    double lastAngularAcceleration[3];
    double lastVelocityDt;
    double lastAccelerationDt;
    double lastTcpVelocityDt;
    double lastTcpAccelerationDt;
    uint16_t lastVelocitySegment;
    uint16_t lastAccelerationSegment;
    uint16_t lastTcpVelocitySegment;
    uint16_t lastTcpAccelerationSegment;
    uint8_t haveSample;
    uint8_t haveVelocity;
    uint8_t haveAcceleration;
    uint8_t haveTcpVelocity;
    uint8_t haveTcpAcceleration;
};
#endif

struct StepExecutorStats {
    uint32_t samples;
    uint32_t zeroStepSamples;
    uint32_t reversalSamples;
    uint32_t maxStepDelta;
    uint32_t totalAbsStepDelta;
    uint32_t maxJointStepDelta[kAxisCount];
    uint32_t totalJointAbsStepDelta[kAxisCount];
    int32_t previousTargetSteps[kAxisCount];
    int8_t previousDirection[kAxisCount];
    uint8_t initialized;
};

inline MotionProgramSettings defaultMotionProgramSettings() {
    MotionProgramSettings settings = {};
    settings.defaultJointSpeedDegPerSec = 10.0;
    settings.defaultLinearSpeedMmPerSec = 25.0;
    settings.defaultJointAccelerationRadSec2 = 60.0 * 3.14159265358979323846 / 180.0;
    settings.defaultLinearAccelerationMmSec2 = 1000.0;
    settings.defaultJointJerkRadSec3 = 600.0 * 3.14159265358979323846 / 180.0;
    settings.defaultLinearJerkMmSec3 = 10000.0;
    settings.defaultToolAngularSpeedRadSec = 180.0 * 3.14159265358979323846 / 180.0;
    settings.defaultToolAngularAccelerationRadSec2 = 720.0 * 3.14159265358979323846 / 180.0;
    settings.defaultToolAngularJerkRadSec3 = 7200.0 * 3.14159265358979323846 / 180.0;
    for (uint8_t i = 0; i < kAxisCount; ++i) {
        settings.jointVelocityLimitRadSec[i] = 0.0;
        settings.jointAccelerationLimitRadSec2[i] = 0.0;
        settings.jointStepsPerDegree[i] = 0.0;
        settings.jointStepRateLimitStepsSec[i] = 0.0;
    }
    settings.sampleMm = 0.0;
    settings.jointSampleDeg = 1.0;
    settings.controlPeriodSec = 0.005;
    settings.singularityThresholdRad = 5.0 * 3.14159265358979323846 / 180.0;
    settings.singularityPolicy = SingularityPolicy::SlowDown;
    settings.verifySampledDynamics = 1;
    settings.minBlendSamples = 6;
    return settings;
}

inline bool isFinite(double value) {
    return isfinite(value);
}

inline double absDouble(double value) {
    return value < 0.0 ? -value : value;
}

inline double radiansToDegrees(double radians) {
    return radians * (180.0 / 3.14159265358979323846);
}

inline double degreesToRadians(double degrees) {
    return degrees * (3.14159265358979323846 / 180.0);
}

// A trigger whose resolved time landed past the end of the window that planned it.
struct PendingMotionTrigger {
    int32_t id;
    double dueTimeSec;
};

struct PendingMotionTriggerQueue {
    PendingMotionTrigger entries[kMaxMotionCommands];
    uint8_t count;
};

inline void clearPendingMotionTriggers(PendingMotionTriggerQueue* queue) {
    if (queue) queue->count = 0;
}

inline bool pushPendingMotionTrigger(PendingMotionTriggerQueue* queue, int32_t id, double dueTimeSec) {
    if (!queue || id < 0 || !isFinite(dueTimeSec)) return false;
    if (queue->count >= kMaxMotionCommands) return false;
    queue->entries[queue->count].id = id;
    queue->entries[queue->count].dueTimeSec = dueTimeSec;
    ++queue->count;
    return true;
}

inline bool takeDuePendingMotionTrigger(PendingMotionTriggerQueue* queue,
                                        double nowSec,
                                        int32_t* outId,
                                        double* outDueTimeSec = nullptr) {
    if (!queue || queue->count == 0 || !outId) return false;
    uint8_t best = kMaxMotionCommands;
    for (uint8_t i = 0; i < queue->count; ++i) {
        if (queue->entries[i].dueTimeSec > nowSec) continue;
        if (best >= queue->count || queue->entries[i].dueTimeSec < queue->entries[best].dueTimeSec) {
            best = i;
        }
    }
    if (best >= queue->count) return false;
    *outId = queue->entries[best].id;
    if (outDueTimeSec) *outDueTimeSec = queue->entries[best].dueTimeSec;
    for (uint8_t i = static_cast<uint8_t>(best + 1); i < queue->count; ++i) {
        queue->entries[i - 1] = queue->entries[i];
    }
    --queue->count;
    return true;
}

inline WeaveParams defaultWeaveParams() {
    WeaveParams params = {};
    params.shape = WeaveShape::None;
    params.rateMode = WeaveRateMode::Unset;
    params.enabled = 0;
    params.scheduleIndex = kWeaveScheduleInline;
    return params;
}

// Whether the program asked for a weave at all, kept separate from whether its numbers are usable.
// Validation needs both: an enabled schedule with an Unset rate mode is a configuration mistake to
// report by index, where an enabled schedule with zero amplitude is merely a no-op.
inline bool weaveIsActive(const WeaveParams& params) {
    return params.enabled != 0 && params.shape != WeaveShape::None;
}

inline double weaveAmplitudePeakMm(const WeaveParams& params) {
    double peak = 0.0;
    const double candidates[3] = {params.amplitudeLeftMm, params.amplitudeRightMm, params.elevationMm};
    for (uint8_t i = 0; i < 3; ++i) {
        if (isFinite(candidates[i]) && candidates[i] > peak) peak = candidates[i];
    }
    return peak;
}

// How far from the taught line the tool is actually taken, laterally: the oscillation plus the bias
// that weaveShapeOffset adds to every lateral sample. Deliberately a separate quantity from the
// amplitude above, because the two answer different questions and only one of them is the pattern.
inline double weaveLateralReachMm(const WeaveParams& params) {
    double reach = weaveAmplitudePeakMm(params);
    if (isFinite(params.biasMm)) reach += absDouble(params.biasMm);
    return reach;
}

inline bool weaveParamsAreUsable(const WeaveParams& params) {
    if (!weaveIsActive(params)) return false;
    if (params.rateMode == WeaveRateMode::Frequency) {
        if (!isFinite(params.frequencyHz) || params.frequencyHz <= 0.0) return false;
    } else if (params.rateMode == WeaveRateMode::Wavelength) {
        if (!isFinite(params.wavelengthMm) || params.wavelengthMm <= 1.0e-6) return false;
    } else {
        return false;
    }
    const double lengths[6] = {params.amplitudeLeftMm, params.amplitudeRightMm, params.elevationMm,
                               params.dwellLeft, params.dwellCenter, params.dwellRight};
    for (uint8_t i = 0; i < 6; ++i) {
        if (!isFinite(lengths[i]) || lengths[i] < 0.0) return false;
    }
    if (!isFinite(params.planeAngleDeg) || !isFinite(params.biasMm)) return false;
    return weaveAmplitudePeakMm(params) > 1.0e-9;
}

inline bool weaveParamsEquivalent(const WeaveParams& a, const WeaveParams& b) {
    if (a.enabled != b.enabled || a.shape != b.shape || a.rateMode != b.rateMode) return false;
    const double left[10] = {a.frequencyHz, a.wavelengthMm, a.amplitudeLeftMm, a.amplitudeRightMm,
                             a.elevationMm, a.planeAngleDeg, a.biasMm,
                             a.dwellLeft, a.dwellCenter, a.dwellRight};
    const double right[10] = {b.frequencyHz, b.wavelengthMm, b.amplitudeLeftMm, b.amplitudeRightMm,
                              b.elevationMm, b.planeAngleDeg, b.biasMm,
                              b.dwellLeft, b.dwellCenter, b.dwellRight};
    for (uint8_t i = 0; i < 10; ++i) {
        if (absDouble(left[i] - right[i]) > 1.0e-12) return false;
    }
    return true;
}

// The single place a schedule index becomes parameters. The simulator and the firmware both call
// it, so an index cannot come to mean two different welds. Returns false only when the request
// names a schedule that was never configured; field-level sanity is validateMotionProgramInput's
// job, because that is where there is somewhere to report it.
inline bool resolveWeaveParams(const WeaveScheduleTable& table,
                               const WeaveParams& request,
                               WeaveParams* out) {
    if (!out) return false;
    *out = defaultWeaveParams();
    if (!weaveIsActive(request)) return true;
    if (request.scheduleIndex >= kMaxWeaveSchedules) {
        *out = request;
        out->scheduleIndex = kWeaveScheduleInline;
        return true;
    }
    if (!table.valid[request.scheduleIndex]) return false;
    *out = table.schedules[request.scheduleIndex];
    out->enabled = 1;
    out->scheduleIndex = request.scheduleIndex;
    return true;
}

inline void applyJointDynamicsLimitsToMotionSettings(MotionProgramSettings* settings,
                                                     const double* velocityLimitRadSec,
                                                     const double* accelerationLimitRadSec2,
                                                     const double* jerkLimitRadSec3) {
    if (!settings) return;
    double defaultAcceleration = 0.0;
    double defaultJerk = 0.0;
    for (uint8_t joint = 0; joint < kAxisCount; ++joint) {
        if (velocityLimitRadSec && isFinite(velocityLimitRadSec[joint]) && velocityLimitRadSec[joint] > 0.0) {
            settings->jointVelocityLimitRadSec[joint] = velocityLimitRadSec[joint];
        }
        if (accelerationLimitRadSec2 && isFinite(accelerationLimitRadSec2[joint]) && accelerationLimitRadSec2[joint] > 0.0) {
            settings->jointAccelerationLimitRadSec2[joint] = accelerationLimitRadSec2[joint];
            if (defaultAcceleration <= 0.0 || accelerationLimitRadSec2[joint] < defaultAcceleration) {
                defaultAcceleration = accelerationLimitRadSec2[joint];
            }
        }
        if (jerkLimitRadSec3 && isFinite(jerkLimitRadSec3[joint]) && jerkLimitRadSec3[joint] > 0.0) {
            if (defaultJerk <= 0.0 || jerkLimitRadSec3[joint] < defaultJerk) {
                defaultJerk = jerkLimitRadSec3[joint];
            }
        }
    }
    if (defaultAcceleration > 0.0) settings->defaultJointAccelerationRadSec2 = defaultAcceleration;
    if (defaultJerk > 0.0) settings->defaultJointJerkRadSec3 = defaultJerk;
}

inline void applyControllerStepLimitsToMotionSettings(MotionProgramSettings* settings,
                                                      const double* stepsPerDegree,
                                                      double minTickGapUs) {
    if (!settings || !stepsPerDegree || !isFinite(minTickGapUs) || minTickGapUs <= 0.0) return;
    const double stepRate = 1000000.0 / minTickGapUs;
    for (uint8_t joint = 0; joint < kAxisCount; ++joint) {
        if (isFinite(stepsPerDegree[joint]) && stepsPerDegree[joint] > 0.0) {
            settings->jointStepsPerDegree[joint] = stepsPerDegree[joint];
            settings->jointStepRateLimitStepsSec[joint] = stepRate;
        }
    }
}

inline uint32_t absSteps(int32_t value) {
    return value < 0 ? static_cast<uint32_t>(-static_cast<int64_t>(value)) : static_cast<uint32_t>(value);
}

inline bool validStepsPerDegree(double value) {
    return isFinite(value) && value >= 1.0 && value <= 10000.0;
}

inline int32_t jointRadiansToSteps(double radians, int32_t zeroSteps, double stepsPerDegree) {
    return zeroSteps + static_cast<int32_t>(llround(radiansToDegrees(radians) * stepsPerDegree));
}

inline double jointStepsToRadians(int32_t steps, int32_t zeroSteps, double stepsPerDegree) {
    if (!validStepsPerDegree(stepsPerDegree)) return 0.0;
    return degreesToRadians(static_cast<double>(steps - zeroSteps) / stepsPerDegree);
}

inline bool quantizeJointRadiansToSteps(const double q[kAxisCount],
                                        const int32_t zeroSteps[kAxisCount],
                                        const double stepsPerDegree[kAxisCount],
                                        int32_t outSteps[kAxisCount],
                                        double outQ[kAxisCount]) {
    if (!q || !zeroSteps || !stepsPerDegree || !outSteps) return false;
    for (uint8_t i = 0; i < kAxisCount; ++i) {
        if (!isFinite(q[i]) || !validStepsPerDegree(stepsPerDegree[i])) return false;
        outSteps[i] = jointRadiansToSteps(q[i], zeroSteps[i], stepsPerDegree[i]);
        if (outQ) outQ[i] = jointStepsToRadians(outSteps[i], zeroSteps[i], stepsPerDegree[i]);
    }
    return true;
}

inline void beginStepExecutorStats(const int32_t startSteps[kAxisCount],
                                   StepExecutorStats* stats) {
    if (!stats) return;
    memset(stats, 0, sizeof(*stats));
    if (!startSteps) return;
    for (uint8_t i = 0; i < kAxisCount; ++i) {
        stats->previousTargetSteps[i] = startSteps[i];
    }
    stats->initialized = 1;
}

inline void resetStepExecutorStats(StepExecutorStats* stats) {
    if (!stats) return;
    int32_t previous[kAxisCount] = {};
    int8_t direction[kAxisCount] = {};
    for (uint8_t i = 0; i < kAxisCount; ++i) {
        previous[i] = stats->previousTargetSteps[i];
        direction[i] = stats->previousDirection[i];
    }
    const uint8_t initialized = stats->initialized;
    memset(stats, 0, sizeof(*stats));
    for (uint8_t i = 0; i < kAxisCount; ++i) {
        stats->previousTargetSteps[i] = previous[i];
        stats->previousDirection[i] = direction[i];
    }
    stats->initialized = initialized;
}

inline void observeStepExecutorTarget(const int32_t targetSteps[kAxisCount],
                                      StepExecutorStats* stats) {
    if (!targetSteps || !stats || !stats->initialized) return;

    bool zeroStepSample = true;
    bool reversalSample = false;
    uint32_t sampleAbsDelta = 0;
    uint32_t sampleMaxDelta = 0;
    for (uint8_t i = 0; i < kAxisCount; ++i) {
        const int32_t delta = targetSteps[i] - stats->previousTargetSteps[i];
        const uint32_t absDelta = absSteps(delta);
        sampleAbsDelta += absDelta;
        if (absDelta > sampleMaxDelta) sampleMaxDelta = absDelta;
        if (absDelta > stats->maxJointStepDelta[i]) stats->maxJointStepDelta[i] = absDelta;
        stats->totalJointAbsStepDelta[i] += absDelta;
        if (delta != 0) {
            zeroStepSample = false;
            const int8_t direction = delta > 0 ? 1 : -1;
            if (stats->previousDirection[i] != 0 && stats->previousDirection[i] != direction) {
                reversalSample = true;
            }
            stats->previousDirection[i] = direction;
        }
        stats->previousTargetSteps[i] = targetSteps[i];
    }

    stats->samples++;
    if (zeroStepSample) stats->zeroStepSamples++;
    if (reversalSample) stats->reversalSamples++;
    if (sampleMaxDelta > stats->maxStepDelta) stats->maxStepDelta = sampleMaxDelta;
    stats->totalAbsStepDelta += sampleAbsDelta;
}

inline bool observeStepExecutorSample(const double q[kAxisCount],
                                      const int32_t zeroSteps[kAxisCount],
                                      const double stepsPerDegree[kAxisCount],
                                      StepExecutorStats* stats,
                                      int32_t outSteps[kAxisCount],
                                      double outQ[kAxisCount]) {
    int32_t targetSteps[kAxisCount] = {};
    if (!quantizeJointRadiansToSteps(q, zeroSteps, stepsPerDegree, targetSteps, outQ)) {
        return false;
    }
    observeStepExecutorTarget(targetSteps, stats);
    if (outSteps) {
        for (uint8_t i = 0; i < kAxisCount; ++i) outSteps[i] = targetSteps[i];
    }
    return true;
}

inline Transform identityTransform() {
    Transform out{{1.0, 0.0, 0.0, 0.0,
                   0.0, 1.0, 0.0, 0.0,
                   0.0, 0.0, 1.0, 0.0}};
    return out;
}

inline bool transformIsFinite(const Transform& transform) {
    for (int i = 0; i < 12; ++i) {
        if (!isFinite(transform.values[i])) return false;
    }
    return true;
}

inline Transform multiply(const Transform& a, const Transform& b) {
    Transform out{};
    out.values[0] = a.values[0] * b.values[0] + a.values[1] * b.values[4] + a.values[2] * b.values[8];
    out.values[1] = a.values[0] * b.values[1] + a.values[1] * b.values[5] + a.values[2] * b.values[9];
    out.values[2] = a.values[0] * b.values[2] + a.values[1] * b.values[6] + a.values[2] * b.values[10];
    out.values[3] = a.values[0] * b.values[3] + a.values[1] * b.values[7] + a.values[2] * b.values[11] + a.values[3];
    out.values[4] = a.values[4] * b.values[0] + a.values[5] * b.values[4] + a.values[6] * b.values[8];
    out.values[5] = a.values[4] * b.values[1] + a.values[5] * b.values[5] + a.values[6] * b.values[9];
    out.values[6] = a.values[4] * b.values[2] + a.values[5] * b.values[6] + a.values[6] * b.values[10];
    out.values[7] = a.values[4] * b.values[3] + a.values[5] * b.values[7] + a.values[6] * b.values[11] + a.values[7];
    out.values[8] = a.values[8] * b.values[0] + a.values[9] * b.values[4] + a.values[10] * b.values[8];
    out.values[9] = a.values[8] * b.values[1] + a.values[9] * b.values[5] + a.values[10] * b.values[9];
    out.values[10] = a.values[8] * b.values[2] + a.values[9] * b.values[6] + a.values[10] * b.values[10];
    out.values[11] = a.values[8] * b.values[3] + a.values[9] * b.values[7] + a.values[10] * b.values[11] + a.values[11];
    return out;
}

inline Transform inverseRigidTransform(const Transform& transform) {
    Transform out{};
    out.values[0] = transform.values[0];
    out.values[1] = transform.values[4];
    out.values[2] = transform.values[8];
    out.values[3] = -(transform.values[0] * transform.values[3] + transform.values[4] * transform.values[7] + transform.values[8] * transform.values[11]);
    out.values[4] = transform.values[1];
    out.values[5] = transform.values[5];
    out.values[6] = transform.values[9];
    out.values[7] = -(transform.values[1] * transform.values[3] + transform.values[5] * transform.values[7] + transform.values[9] * transform.values[11]);
    out.values[8] = transform.values[2];
    out.values[9] = transform.values[6];
    out.values[10] = transform.values[10];
    out.values[11] = -(transform.values[2] * transform.values[3] + transform.values[6] * transform.values[7] + transform.values[10] * transform.values[11]);
    return out;
}

inline double matrixValue(const Transform& transform, int row, int col) {
    return transform.values[row * 4 + col];
}

inline double distanceTranslation(const Transform& a, const Transform& b) {
    const double dx = a.values[3] - b.values[3];
    const double dy = a.values[7] - b.values[7];
    const double dz = a.values[11] - b.values[11];
    return sqrt(dx * dx + dy * dy + dz * dz);
}

inline void translationVector(const Transform& transform, double out[3]) {
    out[0] = transform.values[3];
    out[1] = transform.values[7];
    out[2] = transform.values[11];
}

inline double dot3(const double a[3], const double b[3]) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

inline void cross3(const double a[3], const double b[3], double out[3]) {
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}

inline double length3(const double value[3]) {
    return sqrt(dot3(value, value));
}

inline bool normalize3(double value[3]) {
    const double mag = length3(value);
    if (mag <= 1.0e-12) return false;
    value[0] /= mag;
    value[1] /= mag;
    value[2] /= mag;
    return true;
}

inline void subtract3(const double a[3], const double b[3], double out[3]) {
    out[0] = a[0] - b[0];
    out[1] = a[1] - b[1];
    out[2] = a[2] - b[2];
}

inline double clamp01(double value) {
    if (value < 0.0) return 0.0;
    if (value > 1.0) return 1.0;
    return value;
}

inline Transform rotationAboutAxis(double x, double y, double z, double angle) {
    const double mag = sqrt(x * x + y * y + z * z);
    if (mag <= 1.0e-12) return identityTransform();
    x /= mag;
    y /= mag;
    z /= mag;
    const double c = cos(angle);
    const double s = sin(angle);
    const double t = 1.0 - c;
    Transform out{};
    out.values[0] = t * x * x + c;
    out.values[1] = t * x * y - s * z;
    out.values[2] = t * x * z + s * y;
    out.values[3] = 0.0;
    out.values[4] = t * x * y + s * z;
    out.values[5] = t * y * y + c;
    out.values[6] = t * y * z - s * x;
    out.values[7] = 0.0;
    out.values[8] = t * x * z - s * y;
    out.values[9] = t * y * z + s * x;
    out.values[10] = t * z * z + c;
    out.values[11] = 0.0;
    return out;
}

inline Transform interpolateOrientation(const Transform& start, const Transform& target, double t) {
    double delta[3][3]{};
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            for (int k = 0; k < 3; ++k) {
                delta[row][col] += matrixValue(start, k, row) * matrixValue(target, k, col);
            }
        }
    }

    const double trace = delta[0][0] + delta[1][1] + delta[2][2];
    const double angle = acos((trace - 1.0) * 0.5 < -1.0 ? -1.0 : ((trace - 1.0) * 0.5 > 1.0 ? 1.0 : (trace - 1.0) * 0.5));
    Transform out = start;
    if (angle <= 1.0e-12) return out;

    const double denom = 2.0 * sin(angle);
    double x = 0.0;
    double y = 0.0;
    double z = 1.0;
    if (absDouble(denom) > 1.0e-8) {
        x = (delta[2][1] - delta[1][2]) / denom;
        y = (delta[0][2] - delta[2][0]) / denom;
        z = (delta[1][0] - delta[0][1]) / denom;
    }
    out = multiply(start, rotationAboutAxis(x, y, z, angle * clamp01(t)));
    out.values[3] = start.values[3];
    out.values[7] = start.values[7];
    out.values[11] = start.values[11];
    return out;
}

inline double orientationDeltaAngleRad(const Transform& start, const Transform& target) {
    double delta[3][3]{};
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            for (int k = 0; k < 3; ++k) {
                delta[row][col] += matrixValue(start, k, row) * matrixValue(target, k, col);
            }
        }
    }
    const double trace = delta[0][0] + delta[1][1] + delta[2][2];
    double c = (trace - 1.0) * 0.5;
    if (c < -1.0) c = -1.0;
    if (c > 1.0) c = 1.0;
    return acos(c);
}

inline void orientationDeltaVectorRad(const Transform& start, const Transform& target, double out[3]) {
    out[0] = 0.0;
    out[1] = 0.0;
    out[2] = 0.0;
    double delta[3][3]{};
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            for (int k = 0; k < 3; ++k) {
                delta[row][col] += matrixValue(start, k, row) * matrixValue(target, k, col);
            }
        }
    }
    double c = (delta[0][0] + delta[1][1] + delta[2][2] - 1.0) * 0.5;
    if (c < -1.0) c = -1.0;
    if (c > 1.0) c = 1.0;
    const double angle = acos(c);
    if (angle <= 1.0e-12) return;
    const double s = sin(angle);
    double axis[3] = {
        delta[2][1] - delta[1][2],
        delta[0][2] - delta[2][0],
        delta[1][0] - delta[0][1]
    };
    if (absDouble(s) > 1.0e-8) {
        const double scale = angle / (2.0 * s);
        out[0] = axis[0] * scale;
        out[1] = axis[1] * scale;
        out[2] = axis[2] * scale;
        return;
    }
    if (!normalize3(axis)) {
        axis[0] = sqrt(delta[0][0] > 0.0 ? delta[0][0] : 0.0);
        axis[1] = sqrt(delta[1][1] > 0.0 ? delta[1][1] : 0.0);
        axis[2] = sqrt(delta[2][2] > 0.0 ? delta[2][2] : 0.0);
        if (!normalize3(axis)) return;
    }
    out[0] = axis[0] * angle;
    out[1] = axis[1] * angle;
    out[2] = axis[2] * angle;
}

inline Transform interpolatePoseLinear(const Transform& start, const Transform& target, double t) {
    const double clamped = clamp01(t);
    Transform out = interpolateOrientation(start, target, clamped);
    out.values[3] = start.values[3] + (target.values[3] - start.values[3]) * clamped;
    out.values[7] = start.values[7] + (target.values[7] - start.values[7]) * clamped;
    out.values[11] = start.values[11] + (target.values[11] - start.values[11]) * clamped;
    return out;
}

inline void sampleQuinticCornerPoint(const MoveLBlendPlan& plan, double t, double out[3]) {
    const double u = clamp01(t);
    const double u2 = u * u;
    const double u3 = u2 * u;
    const double u4 = u3 * u;
    const double u5 = u4 * u;
    for (uint8_t i = 0; i < 3; ++i) {
        const double p0 = plan.entryPoint[i];
        const double p1 = plan.exitPoint[i];
        const double v0 = plan.incomingUnit[i] * plan.tangentScaleMm;
        const double v1 = plan.outgoingUnit[i] * plan.tangentScaleMm;
        const double a = p1 - p0 - v0;
        const double b = v1 - v0;
        const double c3 = 10.0 * a - 4.0 * b;
        const double c4 = 7.0 * b - 15.0 * a;
        const double c5 = 6.0 * a - 3.0 * b;
        out[i] = p0 + v0 * u + c3 * u3 + c4 * u4 + c5 * u5;
    }
}

inline double estimateQuinticCornerLength(const MoveLBlendPlan& plan) {
    constexpr uint8_t kSamples = 64;
    double previous[3] = {};
    double current[3] = {};
    sampleQuinticCornerPoint(plan, 0.0, previous);
    double length = 0.0;
    for (uint8_t i = 1; i <= kSamples; ++i) {
        sampleQuinticCornerPoint(plan, static_cast<double>(i) / static_cast<double>(kSamples), current);
        double delta[3] = {};
        subtract3(current, previous, delta);
        length += length3(delta);
        for (uint8_t axis = 0; axis < 3; ++axis) previous[axis] = current[axis];
    }
    return length;
}

inline void updateMoveLBlendArcLengthTable(MoveLBlendPlan* plan) {
    if (!plan) return;
    constexpr uint8_t kTableSamples = 8;
    plan->arcLengthSamples[0] = 0.0;
    if (!plan->useQuintic) {
        const double length = absDouble(plan->radiusMm * plan->sweepRad);
        plan->translationLengthMm = isFinite(length) && length > 0.0 ? length : 0.0;
        for (uint8_t i = 1; i <= kTableSamples; ++i) {
            plan->arcLengthSamples[i] = plan->translationLengthMm * (static_cast<double>(i) / static_cast<double>(kTableSamples));
        }
        return;
    }

    double previous[3] = {};
    double current[3] = {};
    sampleQuinticCornerPoint(*plan, 0.0, previous);
    double length = 0.0;
    constexpr uint8_t kSubSamplesPerTableCell = 8;
    for (uint8_t cell = 1; cell <= kTableSamples; ++cell) {
        const double cellStart = static_cast<double>(cell - 1) / static_cast<double>(kTableSamples);
        const double cellEnd = static_cast<double>(cell) / static_cast<double>(kTableSamples);
        for (uint8_t sub = 1; sub <= kSubSamplesPerTableCell; ++sub) {
            const double u = cellStart + (cellEnd - cellStart) * (static_cast<double>(sub) / static_cast<double>(kSubSamplesPerTableCell));
            sampleQuinticCornerPoint(*plan, u, current);
            double delta[3] = {};
            subtract3(current, previous, delta);
            length += length3(delta);
            for (uint8_t axis = 0; axis < 3; ++axis) previous[axis] = current[axis];
        }
        plan->arcLengthSamples[cell] = length;
    }
    plan->translationLengthMm = isFinite(length) && length > 0.0 ? length : 0.0;
}

inline double estimateQuinticCornerDeviation(const MoveLBlendPlan& plan) {
    constexpr uint8_t kSamples = 64;
    double corner[3] = {};
    translationVector(plan.cornerTcp, corner);
    double maxDeviation = 0.0;
    for (uint8_t i = 0; i <= kSamples; ++i) {
        double point[3] = {};
        sampleQuinticCornerPoint(plan, static_cast<double>(i) / static_cast<double>(kSamples), point);
        double delta[3] = {};
        subtract3(point, corner, delta);
        const double deviation = length3(delta);
        if (deviation > maxDeviation) maxDeviation = deviation;
    }
    return maxDeviation;
}

inline Transform sampleBlendArcPose(const MoveLBlendPlan& plan, double t) {
    const double clamped = clamp01(t);
    const Transform entryToCorner = interpolateOrientation(plan.entryTcp, plan.cornerTcp, clamped);
    const Transform cornerToExit = interpolateOrientation(plan.cornerTcp, plan.exitTcp, clamped);
    Transform pose = interpolateOrientation(entryToCorner, cornerToExit, clamped);
    if (plan.useQuintic) {
        double point[3] = {};
        sampleQuinticCornerPoint(plan, clamped, point);
        pose.values[3] = point[0];
        pose.values[7] = point[1];
        pose.values[11] = point[2];
        return pose;
    }
    const double angle = plan.sweepRad * clamped;
    const double c = cos(angle);
    const double s = sin(angle);
    pose.values[3] = plan.center[0] + (plan.startUnit[0] * c + plan.planeUnit[0] * s) * plan.radiusMm;
    pose.values[7] = plan.center[1] + (plan.startUnit[1] * c + plan.planeUnit[1] * s) * plan.radiusMm;
    pose.values[11] = plan.center[2] + (plan.startUnit[2] * c + plan.planeUnit[2] * s) * plan.radiusMm;
    return pose;
}

inline bool planMoveLBlend(const Transform& startTcp,
                           const Transform& cornerTcp,
                           const Transform& nextTcp,
                           double maxDeviationMm,
                           MoveLBlendPlan* plan) {
    if (!plan) return false;
    memset(plan, 0, sizeof(*plan));
    if (!isFinite(maxDeviationMm) ||
        maxDeviationMm <= 0.0 ||
        !transformIsFinite(startTcp) ||
        !transformIsFinite(cornerTcp) ||
        !transformIsFinite(nextTcp)) {
        return false;
    }

    double startPoint[3] = {};
    double cornerPoint[3] = {};
    double nextPoint[3] = {};
    translationVector(startTcp, startPoint);
    translationVector(cornerTcp, cornerPoint);
    translationVector(nextTcp, nextPoint);

    double inVector[3] = {};
    double outVector[3] = {};
    subtract3(cornerPoint, startPoint, inVector);
    subtract3(nextPoint, cornerPoint, outVector);
    const double incomingLength = length3(inVector);
    const double outgoingLength = length3(outVector);
    if (incomingLength <= 1.0e-9 || outgoingLength <= 1.0e-9) return false;
    if (!normalize3(inVector) || !normalize3(outVector)) return false;

    double alignment = dot3(inVector, outVector);
    if (alignment < -1.0) alignment = -1.0;
    if (alignment > 1.0) alignment = 1.0;
    const double deflection = acos(alignment);
    constexpr double kPiLocal = 3.14159265358979323846;
    if (deflection <= 1.0e-4 || deflection >= kPiLocal - 1.0e-4) return false;

    const double half = deflection * 0.5;
    const double tanHalf = tan(half);
    const double maxTrim = (incomingLength < outgoingLength ? incomingLength : outgoingLength) * 0.5;
    if (tanHalf <= 1.0e-9 || maxTrim <= 1.0e-6) return false;

    // Treat blend_mm as the maximum distance from the programmed corner to the
    // blend entry/exit points measured along the original polyline segments.
    double trim = maxDeviationMm < maxTrim ? maxDeviationMm : maxTrim;
    if (trim <= 1.0e-6 || !isFinite(trim)) return false;
    const double radius = trim / tanHalf;
    if (radius <= 1.0e-6 || !isFinite(radius)) return false;
    const double actualDeviation = trim;
    const double contourDeviation = radius * (1.0 - cos(half));
    if (!isFinite(contourDeviation) || actualDeviation > maxDeviationMm + 1.0e-6) return false;

    plan->entryTcp = interpolatePoseLinear(startTcp, cornerTcp, (incomingLength - trim) / incomingLength);
    plan->exitTcp = interpolatePoseLinear(cornerTcp, nextTcp, trim / outgoingLength);
    if (!transformIsFinite(plan->entryTcp) || !transformIsFinite(plan->exitTcp)) return false;
    double entryPoint[3] = {};
    double exitPoint[3] = {};
    translationVector(plan->entryTcp, entryPoint);
    translationVector(plan->exitTcp, exitPoint);
    plan->cornerTcp = cornerTcp;
    for (uint8_t i = 0; i < 3; ++i) {
        plan->entryPoint[i] = entryPoint[i];
        plan->exitPoint[i] = exitPoint[i];
        plan->incomingUnit[i] = inVector[i];
        plan->outgoingUnit[i] = outVector[i];
    }
    plan->tangentScaleMm = trim;

    double normal[3] = {};
    cross3(inVector, outVector, normal);
    if (!normalize3(normal)) return false;

    double centerOffsetUnit[3] = {};
    cross3(normal, inVector, centerOffsetUnit);
    if (!normalize3(centerOffsetUnit)) return false;
    for (uint8_t i = 0; i < 3; ++i) {
        plan->center[i] = entryPoint[i] + centerOffsetUnit[i] * radius;
        plan->startUnit[i] = entryPoint[i] - plan->center[i];
    }
    if (!normalize3(plan->startUnit)) return false;

    double endUnit[3] = {};
    for (uint8_t i = 0; i < 3; ++i) endUnit[i] = exitPoint[i] - plan->center[i];
    if (!normalize3(endUnit)) return false;
    cross3(normal, plan->startUnit, plan->planeUnit);
    if (!normalize3(plan->planeUnit)) return false;

    double sweep = atan2(dot3(endUnit, plan->planeUnit), dot3(endUnit, plan->startUnit));
    if (sweep < 0.0) sweep += 2.0 * kPiLocal;
    if (sweep > kPiLocal) sweep -= 2.0 * kPiLocal;
    if (absDouble(sweep) <= 1.0e-9) return false;
    const double arcLength = absDouble(radius * sweep);
    if (!isFinite(arcLength) || arcLength <= 1.0e-6) return false;
    plan->tangentScaleMm = arcLength * 1.5;

    double cornerToEntry[3] = {};
    double cornerToExit[3] = {};
    subtract3(entryPoint, cornerPoint, cornerToEntry);
    subtract3(exitPoint, cornerPoint, cornerToExit);
    if (length3(cornerToEntry) > maxDeviationMm + 1.0e-5 ||
        length3(cornerToExit) > maxDeviationMm + 1.0e-5) {
        return false;
    }
    const double entryRadiusError = absDouble(length3(plan->startUnit) - 1.0);
    const double exitRadiusError = absDouble(length3(endUnit) - 1.0);
    if (entryRadiusError > 1.0e-6 || exitRadiusError > 1.0e-6) return false;

    plan->radiusMm = radius;
    plan->sweepRad = sweep;
    plan->trimMm = trim;
    plan->maxDeviationMm = maxDeviationMm;
    plan->actualDeviationMm = actualDeviation;
    plan->contourDeviationMm = contourDeviation;
    plan->useQuintic = 1;
    plan->enabled = 1;
    updateMoveLBlendArcLengthTable(plan);
    return true;
}

inline double wrapRadians(double value) {
    constexpr double kPi = 3.14159265358979323846;
    while (value > kPi) value -= 2.0 * kPi;
    while (value < -kPi) value += 2.0 * kPi;
    return value;
}

inline double nearestEquivalentRadians(double value, double reference) {
    constexpr double kPi = 3.14159265358979323846;
    while (value - reference > kPi) value -= 2.0 * kPi;
    while (value - reference < -kPi) value += 2.0 * kPi;
    return value;
}

#if !defined(ARDUINO)
inline void resetMotionKinematicStatsObserver(MotionKinematicStatsObserver* observer) {
    if (observer) memset(observer, 0, sizeof(*observer));
}

inline void observeMotionKinematicSample(MotionKinematicStatsObserver* observer, const PathSample& sample) {
    if (!observer || !sample.valid) return;
    if (!observer->haveSample) {
        observer->lastSample = sample;
        observer->haveSample = 1;
        return;
    }

    const PathSample& last = observer->lastSample;
    const double dt = sample.timeSec - last.timeSec;
    if (dt <= 1.0e-9) {
        observer->lastSample = sample;
        return;
    }

    const bool sameSegment = sample.pathSegmentIndex == last.pathSegmentIndex;
    if (!sameSegment) {
        observer->haveVelocity = 0;
        observer->haveAcceleration = 0;
        observer->haveTcpVelocity = 0;
        observer->haveTcpAcceleration = 0;
        observer->lastSample = sample;
        return;
    }

    double velocity[kAxisCount] = {};
    for (uint8_t joint = 0; joint < kAxisCount; ++joint) {
        velocity[joint] = wrapRadians(sample.q[joint] - last.q[joint]) / dt;
        const double speed = absDouble(velocity[joint]);
        if (speed > observer->stats.maxJointSpeedRadS[joint]) {
            observer->stats.maxJointSpeedRadS[joint] = speed;
        }
    }

    if (observer->haveVelocity && observer->lastVelocitySegment == sample.pathSegmentIndex) {
        const double accelDt = 0.5 * (observer->lastVelocityDt + dt);
        if (accelDt > 1.0e-9) {
            double acceleration[kAxisCount] = {};
            for (uint8_t joint = 0; joint < kAxisCount; ++joint) {
                acceleration[joint] = (velocity[joint] - observer->lastJointVelocity[joint]) / accelDt;
                const double accel = absDouble(acceleration[joint]);
                if (accel > observer->stats.maxJointAccelerationRadS2[joint]) {
                    observer->stats.maxJointAccelerationRadS2[joint] = accel;
                }
            }
            if (observer->haveAcceleration &&
                observer->lastAccelerationSegment == sample.pathSegmentIndex &&
                sample.kind == PathSampleKind::MoveJ &&
                last.kind == PathSampleKind::MoveJ) {
                const double jerkDt = 0.5 * (observer->lastAccelerationDt + accelDt);
                if (jerkDt > 1.0e-9) {
                    for (uint8_t joint = 0; joint < kAxisCount; ++joint) {
                        const double jerk = absDouble(acceleration[joint] - observer->lastJointAcceleration[joint]) / jerkDt;
                        if (jerk > observer->stats.maxJointJerkRadS3[joint]) {
                            observer->stats.maxJointJerkRadS3[joint] = jerk;
                        }
                    }
                }
            }
            for (uint8_t joint = 0; joint < kAxisCount; ++joint) {
                observer->lastJointAcceleration[joint] = acceleration[joint];
            }
            observer->lastAccelerationDt = accelDt;
            observer->lastAccelerationSegment = sample.pathSegmentIndex;
            observer->haveAcceleration = 1;
        }
    }
    for (uint8_t joint = 0; joint < kAxisCount; ++joint) {
        observer->lastJointVelocity[joint] = velocity[joint];
    }
    observer->lastVelocityDt = dt;
    observer->lastVelocitySegment = sample.pathSegmentIndex;
    observer->haveVelocity = 1;

    if (sample.kind != PathSampleKind::MoveJ && last.kind != PathSampleKind::MoveJ) {
        double currentPoint[3] = {};
        double lastPoint[3] = {};
        double deltaPoint[3] = {};
        translationVector(sample.tcp, currentPoint);
        translationVector(last.tcp, lastPoint);
        subtract3(currentPoint, lastPoint, deltaPoint);
        double linearVelocity[3] = {
            deltaPoint[0] / dt,
            deltaPoint[1] / dt,
            deltaPoint[2] / dt
        };
        const double tcpSpeed = length3(linearVelocity);
        if (tcpSpeed > observer->stats.maxTcpSpeedMmPerSec) {
            observer->stats.maxTcpSpeedMmPerSec = tcpSpeed;
        }

        double rotationDelta[3] = {};
        orientationDeltaVectorRad(last.tcp, sample.tcp, rotationDelta);
        double angularVelocity[3] = {
            rotationDelta[0] / dt,
            rotationDelta[1] / dt,
            rotationDelta[2] / dt
        };
        const double angularSpeed = length3(angularVelocity);
        if (angularSpeed > observer->stats.maxToolAngularSpeedRadS) {
            observer->stats.maxToolAngularSpeedRadS = angularSpeed;
        }

        if (observer->haveTcpVelocity && observer->lastTcpVelocitySegment == sample.pathSegmentIndex) {
            const double accelDt = 0.5 * (observer->lastTcpVelocityDt + dt);
            if (accelDt > 1.0e-9) {
                double linearAcceleration[3] = {
                    (linearVelocity[0] - observer->lastLinearVelocity[0]) / accelDt,
                    (linearVelocity[1] - observer->lastLinearVelocity[1]) / accelDt,
                    (linearVelocity[2] - observer->lastLinearVelocity[2]) / accelDt
                };
                double angularAcceleration[3] = {
                    (angularVelocity[0] - observer->lastAngularVelocity[0]) / accelDt,
                    (angularVelocity[1] - observer->lastAngularVelocity[1]) / accelDt,
                    (angularVelocity[2] - observer->lastAngularVelocity[2]) / accelDt
                };
                const double tcpAccel = length3(linearAcceleration);
                const double angularAccel = length3(angularAcceleration);
                if (tcpAccel > observer->stats.maxTcpAccelerationMmS2) {
                    observer->stats.maxTcpAccelerationMmS2 = tcpAccel;
                }
                if (angularAccel > observer->stats.maxToolAngularAccelerationRadS2) {
                    observer->stats.maxToolAngularAccelerationRadS2 = angularAccel;
                }
                if (observer->haveTcpAcceleration &&
                    observer->lastTcpAccelerationSegment == sample.pathSegmentIndex) {
                    const double jerkDt = 0.5 * (observer->lastTcpAccelerationDt + accelDt);
                    if (jerkDt > 1.0e-9) {
                        double linearJerk[3] = {
                            (linearAcceleration[0] - observer->lastLinearAcceleration[0]) / jerkDt,
                            (linearAcceleration[1] - observer->lastLinearAcceleration[1]) / jerkDt,
                            (linearAcceleration[2] - observer->lastLinearAcceleration[2]) / jerkDt
                        };
                        double angularJerkVector[3] = {
                            (angularAcceleration[0] - observer->lastAngularAcceleration[0]) / jerkDt,
                            (angularAcceleration[1] - observer->lastAngularAcceleration[1]) / jerkDt,
                            (angularAcceleration[2] - observer->lastAngularAcceleration[2]) / jerkDt
                        };
                        const double tcpJerk = length3(linearJerk);
                        const double angularJerk = length3(angularJerkVector);
                        if (tcpJerk > observer->stats.maxTcpJerkMmS3) {
                            observer->stats.maxTcpJerkMmS3 = tcpJerk;
                        }
                        if (angularJerk > observer->stats.maxToolAngularJerkRadS3) {
                            observer->stats.maxToolAngularJerkRadS3 = angularJerk;
                        }
                    }
                }
                for (uint8_t axis = 0; axis < 3; ++axis) {
                    observer->lastLinearAcceleration[axis] = linearAcceleration[axis];
                    observer->lastAngularAcceleration[axis] = angularAcceleration[axis];
                }
                observer->lastTcpAccelerationDt = accelDt;
                observer->lastTcpAccelerationSegment = sample.pathSegmentIndex;
                observer->haveTcpAcceleration = 1;
            }
        }
        for (uint8_t axis = 0; axis < 3; ++axis) {
            observer->lastLinearVelocity[axis] = linearVelocity[axis];
            observer->lastAngularVelocity[axis] = angularVelocity[axis];
        }
        observer->lastTcpVelocityDt = dt;
        observer->lastTcpVelocitySegment = sample.pathSegmentIndex;
        observer->haveTcpVelocity = 1;
    } else {
        observer->haveTcpVelocity = 0;
        observer->haveTcpAcceleration = 0;
    }

    observer->lastSample = sample;
}
#endif

inline double dhmThetaOffset(const RobotModel& model, uint8_t joint) { return model.dhm[joint]; }
inline double dhmA(const RobotModel& model, uint8_t joint) { return model.dhm[kAxisCount + joint]; }
inline double dhmAlpha(const RobotModel& model, uint8_t joint) { return model.dhm[2 * kAxisCount + joint]; }
inline double dhmD(const RobotModel& model, uint8_t joint) { return model.dhm[3 * kAxisCount + joint]; }

inline bool modelIsValid(const RobotModel& model) {
    if (model.valid == 0 || !transformIsFinite(model.toolBindPose)) return false;
    bool anyDhm = false;
    for (uint8_t i = 0; i < 24; ++i) {
        if (!isFinite(model.dhm[i])) return false;
        anyDhm = anyDhm || absDouble(model.dhm[i]) > 1.0e-12;
    }
    if (!anyDhm) return false;
    for (uint8_t i = 0; i < kAxisCount; ++i) {
        if (!isFinite(model.qHome[i]) || !isFinite(model.qMin[i]) || !isFinite(model.qMax[i]) ||
            !isFinite(model.dhmSigns[i]) || absDouble(model.dhmSigns[i]) < 0.5) {
            return false;
        }
    }
    return true;
}

inline Transform dhmLinkTransform(double alpha, double a, double theta, double d) {
    const double crx = cos(alpha);
    const double srx = sin(alpha);
    const double crz = cos(theta);
    const double srz = sin(theta);
    Transform pose{};
    pose.values[0] = crz;
    pose.values[1] = -srz;
    pose.values[2] = 0.0;
    pose.values[3] = a;
    pose.values[4] = crx * srz;
    pose.values[5] = crx * crz;
    pose.values[6] = -srx;
    pose.values[7] = -d * srx;
    pose.values[8] = srx * srz;
    pose.values[9] = crz * srx;
    pose.values[10] = crx;
    pose.values[11] = d * crx;
    return pose;
}

inline Transform ar4ForwardDhm(const RobotModel& model, const double qDhm[kAxisCount]) {
    Transform pose = identityTransform();
    for (uint8_t i = 0; i < kAxisCount; ++i) {
        pose = multiply(pose, dhmLinkTransform(dhmAlpha(model, i), dhmA(model, i), dhmThetaOffset(model, i) + qDhm[i], dhmD(model, i)));
    }
    return pose;
}

inline Transform viewerAr4Basis() {
    Transform s{};
    s.values[0] = 1.0; s.values[1] = 0.0; s.values[2] = 0.0; s.values[3] = 0.0;
    s.values[4] = 0.0; s.values[5] = 0.0; s.values[6] = 1.0; s.values[7] = 0.0;
    s.values[8] = 0.0; s.values[9] = -1.0; s.values[10] = 0.0; s.values[11] = 0.0;
    return s;
}

inline Transform viewerFromAr4Transform(const Transform& ar4) {
    const Transform s = viewerAr4Basis();
    return multiply(multiply(s, ar4), inverseRigidTransform(s));
}

inline Transform ar4FromViewerTransform(const Transform& viewer) {
    const Transform s = viewerAr4Basis();
    return multiply(multiply(inverseRigidTransform(s), viewer), s);
}

inline void transformToAr4Matrix(const Transform& transform, double out[16]) {
    out[0] = transform.values[0];  out[4] = transform.values[1];  out[8] = transform.values[2];   out[12] = transform.values[3];
    out[1] = transform.values[4];  out[5] = transform.values[5];  out[9] = transform.values[6];   out[13] = transform.values[7];
    out[2] = transform.values[8];  out[6] = transform.values[9];  out[10] = transform.values[10]; out[14] = transform.values[11];
    out[3] = 0.0;                  out[7] = 0.0;                  out[11] = 0.0;                  out[15] = 1.0;
}

inline void matrixMultiplyAr4(double out[16], const double a[16], const double b[16]) {
    double temp[16]{};
    temp[0] = a[0] * b[0] + a[4] * b[1] + a[8] * b[2];
    temp[1] = a[1] * b[0] + a[5] * b[1] + a[9] * b[2];
    temp[2] = a[2] * b[0] + a[6] * b[1] + a[10] * b[2];
    temp[3] = 0.0;
    temp[4] = a[0] * b[4] + a[4] * b[5] + a[8] * b[6];
    temp[5] = a[1] * b[4] + a[5] * b[5] + a[9] * b[6];
    temp[6] = a[2] * b[4] + a[6] * b[5] + a[10] * b[6];
    temp[7] = 0.0;
    temp[8] = a[0] * b[8] + a[4] * b[9] + a[8] * b[10];
    temp[9] = a[1] * b[8] + a[5] * b[9] + a[9] * b[10];
    temp[10] = a[2] * b[8] + a[6] * b[9] + a[10] * b[10];
    temp[11] = 0.0;
    temp[12] = a[0] * b[12] + a[4] * b[13] + a[8] * b[14] + a[12];
    temp[13] = a[1] * b[12] + a[5] * b[13] + a[9] * b[14] + a[13];
    temp[14] = a[2] * b[12] + a[6] * b[13] + a[10] * b[14] + a[14];
    temp[15] = 1.0;
    for (int i = 0; i < 16; ++i) out[i] = temp[i];
}

inline void robotDhmToAr4Dk(const RobotModel& model, double dk[66]) {
    for (int i = 0; i < 66; ++i) dk[i] = 0.0;
    for (int i = 0; i < 6; ++i) {
        const int offset = i * 6;
        dk[offset + 0] = dhmAlpha(model, static_cast<uint8_t>(i));
        dk[offset + 1] = dhmA(model, static_cast<uint8_t>(i));
        dk[offset + 2] = dhmThetaOffset(model, static_cast<uint8_t>(i));
        dk[offset + 3] = dhmD(model, static_cast<uint8_t>(i));
        dk[60 + i] = 1.0;
    }
}

inline bool ar4InverseRawNoFrames(const double pose[16], const double dk[66], const double jointsApproxDeg[6], int elbowBranch, double jointsDeg[6]) {
    constexpr double kPi = 3.14159265358979323846;
    constexpr double kRadToDeg = 180.0 / kPi;
    constexpr double kDegToRad = kPi / 180.0;
    double jointsApprox[6]{};
    for (int i = 0; i < 6; ++i) jointsApprox[i] = dk[60 + i] * jointsApproxDeg[i];

    double tool[16]{};
    for (int i = 0; i < 16; ++i) tool[i] = pose[i];

    const double p04[4]{pose[12] - pose[8] * dk[33], pose[13] - pose[9] * dk[33], pose[14] - pose[10] * dk[33], 1.0};
    double q1 = 0.0;
    if (absDouble(dk[9]) <= 1.0e-12) {
        q1 = atan2(p04[1], p04[0]);
    } else {
        const double rootValue = (p04[0] * p04[0] + p04[1] * p04[1]) - dk[9] * dk[9];
        if (rootValue < 0.0) return false;
        q1 = atan2(p04[1], p04[0]) - atan2(dk[9], sqrt(rootValue));
    }

    const double k2Initial = p04[2] - dk[3];
    const double k1 = (cos(q1) * p04[0] + sin(q1) * p04[1]) - dk[7];
    const double aiInitial = (((k1 * k1 + k2Initial * k2Initial) - dk[13] * dk[13]) - dk[21] * dk[21]) - dk[19] * dk[19];
    const double bInitial = 2.0 * dk[21] * dk[13];
    const double cInitial = 2.0 * dk[19] * dk[13];
    if (absDouble(bInitial) <= 1.0e-12 && absDouble(cInitial) <= 1.0e-12) return false;

    double s31 = 0.0;
    double c31 = 0.0;
    double rootValue = 0.0;
    if (absDouble(cInitial) <= 1.0e-12) {
        s31 = -aiInitial / bInitial;
        rootValue = 1.0 - s31 * s31;
        if (rootValue >= 0.0) c31 = static_cast<double>(elbowBranch) * sqrt(rootValue);
    } else {
        const double cSquared = cInitial * cInitial;
        const double bbDivCc = bInitial * bInitial / cSquared;
        const double linearTerm = 2.0 * aiInitial * bInitial / cSquared;
        rootValue = linearTerm * linearTerm - 4.0 * ((1.0 + bbDivCc) * (aiInitial * aiInitial / cSquared - 1.0));
        if (rootValue >= 0.0) {
            s31 = (-linearTerm + static_cast<double>(elbowBranch) * sqrt(rootValue)) / (2.0 * (1.0 + bbDivCc));
            c31 = (aiInitial + bInitial * s31) / cInitial;
        }
    }
    if (rootValue < 0.0 || absDouble(s31) > 1.0) return false;

    double b = atan2(s31, c31);
    const double cosB = cos(b);
    const double sinB = sin(b);
    double c = (dk[13] - dk[21] * sinB) + dk[19] * cosB;
    const double shoulder = dk[21] * cosB + dk[19] * sinB;
    const double q13Idx0 = q1 - dk[2];
    const double q2Raw = atan2(c * k1 - shoulder * k2Initial, c * k2Initial + shoulder * k1) + (-dk[8] - kPi * 0.5);
    const double q13Idx2 = b - dk[14];
    const double q4Estimate = jointsApprox[3] * kDegToRad + dk[20];

    q1 = q13Idx0 + dk[2];
    b = q2Raw + dk[8];
    c = q13Idx2 + dk[14];
    const double bPlusC = b + c;
    double h03Inv[16]{};
    const double cosBC = cos(bPlusC);
    const double sinBC = sin(bPlusC);
    const double cosQ1 = cos(q1);
    const double sinQ1 = sin(q1);
    h03Inv[0] = cosBC * cosQ1;
    h03Inv[4] = cosBC * sinQ1;
    h03Inv[8] = -sinBC;
    h03Inv[12] = (dk[3] * sinBC - dk[7] * cosBC) - dk[13] * cos(c);
    h03Inv[1] = -sinBC * cosQ1;
    h03Inv[5] = -sinBC * sinQ1;
    h03Inv[9] = -cosBC;
    h03Inv[13] = (dk[3] * cosBC + dk[7] * sinBC) + dk[13] * sin(c);
    h03Inv[2] = -sinQ1;
    h03Inv[6] = cosQ1;
    h03Inv[10] = 0.0;
    h03Inv[14] = 0.0;
    h03Inv[3] = 0.0;
    h03Inv[7] = 0.0;
    h03Inv[11] = 0.0;
    h03Inv[15] = 1.0;

    double wrist[16]{};
    matrixMultiplyAr4(wrist, h03Inv, tool);

    double wristRoot = 1.0 - wrist[9] * wrist[9];
    wristRoot = wristRoot <= 0.0 ? 0.0 : sqrt(wristRoot);
    double q4Raw = 0.0;
    double q5Raw = 0.0;
    double q6Raw = 0.0;
    if (wristRoot < 1.0e-6) {
        q5Raw = atan2(wristRoot, wrist[9]);
        const double sinQ4Estimate = sin(q4Estimate);
        const double cosQ4Estimate = cos(q4Estimate);
        q6Raw = atan2(sinQ4Estimate * wrist[0] + cosQ4Estimate * wrist[2],
                      sinQ4Estimate * wrist[2] - cosQ4Estimate * wrist[0]);
    } else if (jointsApprox[4] >= 0.0) {
        q4Raw = atan2(wrist[10] / wristRoot, -wrist[8] / wristRoot);
        q5Raw = atan2(wristRoot, wrist[9]);
        const double sinQ5 = sin(q5Raw);
        q6Raw = atan2(wrist[5] / sinQ5, -wrist[1] / sinQ5);
    } else {
        q4Raw = atan2(-wrist[10] / wristRoot, wrist[8] / wristRoot);
        q5Raw = atan2(-wristRoot, wrist[9]);
        const double sinQ5 = sin(q5Raw);
        q6Raw = atan2(wrist[5] / sinQ5, -wrist[1] / sinQ5);
    }

    double jointsRad[6]{q13Idx0, q2Raw, q13Idx2, q4Raw - dk[20], q5Raw - dk[26], q6Raw + (-dk[32] + kPi)};
    jointsRad[5] = wrapRadians(jointsRad[5]);
    for (int i = 0; i < 6; ++i) jointsDeg[i] = dk[60 + i] * jointsRad[i] * kRadToDeg;
    return true;
}

inline int ar4AnalyticInverseDhm(const RobotModel& model,
                                 const Transform& targetAr4Pose,
                                 const double referenceDhmQ[kAxisCount],
                                 double solutions[][kAxisCount],
                                 int solutionCapacity) {
    if (!modelIsValid(model) || !solutions || solutionCapacity <= 0) return 0;
    constexpr double kPi = 3.14159265358979323846;
    constexpr double kRadToDeg = 180.0 / kPi;
    constexpr double kDegToRad = kPi / 180.0;

    double dk[66]{};
    robotDhmToAr4Dk(model, dk);
    double pose[16]{};
    transformToAr4Matrix(targetAr4Pose, pose);

    double estimateDeg[6]{};
    for (int i = 0; i < 6; ++i) estimateDeg[i] = referenceDhmQ[i] * kRadToDeg;

    int count = 0;
    for (int wristBranch = -3; wristBranch <= 3 && count < solutionCapacity; ++wristBranch) {
        double branchEstimate[6]{};
        for (int i = 0; i < 6; ++i) branchEstimate[i] = estimateDeg[i];
        branchEstimate[4] = static_cast<double>(wristBranch * 30);

        for (int elbowBranchIndex = 0; elbowBranchIndex < 2 && count < solutionCapacity; ++elbowBranchIndex) {
            const int elbowBranch = elbowBranchIndex == 0 ? 1 : -1;
            double solvedDeg[6]{};
            if (!ar4InverseRawNoFrames(pose, dk, branchEstimate, elbowBranch, solvedDeg)) continue;

            double q[kAxisCount]{};
            for (int i = 0; i < 6; ++i) {
                q[i] = nearestEquivalentRadians(solvedDeg[i] * kDegToRad, referenceDhmQ[i]);
            }

            bool duplicate = false;
            for (int existing = 0; existing < count && !duplicate; ++existing) {
                double maxDelta = 0.0;
                for (int i = 0; i < 6; ++i) {
                    const double delta = absDouble(wrapRadians(q[i] - solutions[existing][i]));
                    if (delta > maxDelta) maxDelta = delta;
                }
                duplicate = maxDelta < 1.0e-6;
            }
            if (!duplicate) {
                for (int i = 0; i < 6; ++i) solutions[count][i] = q[i];
                ++count;
            }
        }
    }
    return count;
}

// UR-family arms do not have the spherical wrist required by OPW: d5 separates the
// fourth and sixth axes.  Their modified-DH table nevertheless has a very distinctive
// form, and admits the eight-solution closed form used by Universal Robots.  FAIRINO's
// FR20 uses this geometry (with larger link dimensions).
inline bool isUrStyleDhm(const RobotModel& model) {
    if (!modelIsValid(model)) return false;
    constexpr double kPi = 3.14159265358979323846;
    constexpr double kAngleTolerance = 1.0e-8;
    constexpr double kLengthTolerance = 1.0e-8;
    const auto nearlyEqual = [](double a, double b, double tolerance) {
        return absDouble(a - b) <= tolerance;
    };
    return nearlyEqual(dhmThetaOffset(model, 0), 0.0, kAngleTolerance) &&
           nearlyEqual(dhmThetaOffset(model, 1), kPi, kAngleTolerance) &&
           nearlyEqual(dhmThetaOffset(model, 2), 0.0, kAngleTolerance) &&
           nearlyEqual(dhmThetaOffset(model, 3), 0.0, kAngleTolerance) &&
           nearlyEqual(dhmThetaOffset(model, 4), 0.0, kAngleTolerance) &&
           nearlyEqual(dhmThetaOffset(model, 5), kPi, kAngleTolerance) &&
           nearlyEqual(dhmAlpha(model, 0), 0.0, kAngleTolerance) &&
           nearlyEqual(dhmAlpha(model, 1), kPi * 0.5, kAngleTolerance) &&
           nearlyEqual(dhmAlpha(model, 2), 0.0, kAngleTolerance) &&
           nearlyEqual(dhmAlpha(model, 3), 0.0, kAngleTolerance) &&
           nearlyEqual(dhmAlpha(model, 4), -kPi * 0.5, kAngleTolerance) &&
           nearlyEqual(dhmAlpha(model, 5), kPi * 0.5, kAngleTolerance) &&
           nearlyEqual(dhmA(model, 0), 0.0, kLengthTolerance) &&
           nearlyEqual(dhmA(model, 1), 0.0, kLengthTolerance) &&
           dhmA(model, 2) > kLengthTolerance &&
           dhmA(model, 3) > kLengthTolerance &&
           nearlyEqual(dhmA(model, 4), 0.0, kLengthTolerance) &&
           nearlyEqual(dhmA(model, 5), 0.0, kLengthTolerance) &&
           nearlyEqual(dhmD(model, 1), 0.0, kLengthTolerance) &&
           nearlyEqual(dhmD(model, 2), 0.0, kLengthTolerance) &&
           absDouble(dhmD(model, 4)) > kLengthTolerance &&
           absDouble(dhmD(model, 5)) > kLengthTolerance;
}

// Closed-form UR inverse, generalized to dimensions read from RobotModel.  The algebra
// follows the ROS-Industrial ur_kinematics solver, but the input/output frame conversion
// below is derived from our modified-DH convention:
//
//   q_UR = {q_DHM[0] + pi, q_DHM[1..5]}
//   T_UR = T_DHM * R, R = [[0,-1,0],[0,0,-1],[1,0,0]]
//
// Source/reference implementation (BSD-3-Clause):
// https://github.com/ros-industrial/universal_robot/blob/noetic-devel/ur_kinematics/src/ur_kin.cpp
inline int urStyleAnalyticInverseDhm(const RobotModel& model,
                                     const Transform& targetDhmPose,
                                     const double referenceDhmQ[kAxisCount],
                                     double solutions[][kAxisCount],
                                     int solutionCapacity) {
    if (!isUrStyleDhm(model) || !solutions || solutionCapacity <= 0) return 0;
    constexpr double kPi = 3.14159265358979323846;
    constexpr double kTwoPi = 2.0 * kPi;
    constexpr double kZeroThreshold = 1.0e-8;
    const double d1 = dhmD(model, 0);
    const double a2 = -dhmA(model, 2);
    const double a3 = -dhmA(model, 3);
    const double d4 = dhmD(model, 3);
    const double d5 = dhmD(model, 4);
    const double d6 = dhmD(model, 5);
    if (absDouble(a2) <= kZeroThreshold || absDouble(a3) <= kZeroThreshold ||
        absDouble(d6) <= kZeroThreshold) {
        return 0;
    }

    Transform endFrame{};
    endFrame.values[0] = 0.0;  endFrame.values[1] = -1.0; endFrame.values[2] = 0.0;  endFrame.values[3] = 0.0;
    endFrame.values[4] = 0.0;  endFrame.values[5] = 0.0;  endFrame.values[6] = -1.0; endFrame.values[7] = 0.0;
    endFrame.values[8] = 1.0;  endFrame.values[9] = 0.0;  endFrame.values[10] = 0.0; endFrame.values[11] = 0.0;
    const Transform urTarget = multiply(targetDhmPose, endFrame);

    // ur_kinematics' inverse routine first changes its public flange convention to
    // the internal matrix used by the closed-form equations.
    const double T02 = -urTarget.values[0];
    const double T00 = urTarget.values[1];
    const double T01 = urTarget.values[2];
    const double T03 = -urTarget.values[3];
    const double T12 = -urTarget.values[4];
    const double T10 = urTarget.values[5];
    const double T11 = urTarget.values[6];
    const double T13 = -urTarget.values[7];
    const double T22 = urTarget.values[8];
    const double T20 = -urTarget.values[9];
    const double T21 = -urTarget.values[10];
    const double T23 = urTarget.values[11];

    const auto sign = [](double value) -> double {
        return value > 0.0 ? 1.0 : (value < 0.0 ? -1.0 : 0.0);
    };
    const auto clampUnit = [](double value) {
        return value < -1.0 ? -1.0 : (value > 1.0 ? 1.0 : value);
    };
    const auto positiveAngle = [=](double value) {
        if (absDouble(value) < kZeroThreshold) return 0.0;
        while (value < 0.0) value += kTwoPi;
        while (value >= kTwoPi) value -= kTwoPi;
        return value;
    };

    double q1[2]{};
    const double shoulderA = d6 * T12 - T13;
    const double shoulderB = d6 * T02 - T03;
    const double shoulderR = shoulderA * shoulderA + shoulderB * shoulderB;
    if (absDouble(shoulderA) < kZeroThreshold) {
        if (absDouble(shoulderB) < kZeroThreshold) return 0;
        const double divisor = absDouble(absDouble(d4) - absDouble(shoulderB)) < kZeroThreshold
            ? -sign(d4) * sign(shoulderB)
            : -d4 / shoulderB;
        const double angle = asin(clampUnit(divisor));
        q1[0] = positiveAngle(angle);
        q1[1] = positiveAngle(kPi - angle);
    } else if (absDouble(shoulderB) < kZeroThreshold) {
        const double divisor = absDouble(absDouble(d4) - absDouble(shoulderA)) < kZeroThreshold
            ? sign(d4) * sign(shoulderA)
            : d4 / shoulderA;
        const double angle = acos(clampUnit(divisor));
        q1[0] = positiveAngle(angle);
        q1[1] = positiveAngle(kTwoPi - angle);
    } else {
        if (d4 * d4 > shoulderR + kZeroThreshold) return 0;
        const double angle = acos(clampUnit(d4 / sqrt(shoulderR)));
        const double bearing = atan2(-shoulderB, shoulderA);
        q1[0] = positiveAngle(angle + bearing);
        q1[1] = positiveAngle(-angle + bearing);
    }

    int count = 0;
    for (int shoulder = 0; shoulder < 2 && count < solutionCapacity; ++shoulder) {
        const double c1 = cos(q1[shoulder]);
        const double s1 = sin(q1[shoulder]);
        const double wristNumerator = T03 * s1 - T13 * c1 - d4;
        double wristDivisor = wristNumerator / d6;
        if (absDouble(absDouble(wristNumerator) - absDouble(d6)) < kZeroThreshold) {
            wristDivisor = sign(wristNumerator) * sign(d6);
        }
        if (wristDivisor < -1.0 - kZeroThreshold || wristDivisor > 1.0 + kZeroThreshold) continue;
        const double wristAngle = acos(clampUnit(wristDivisor));
        const double q5[2]{positiveAngle(wristAngle), positiveAngle(kTwoPi - wristAngle)};

        for (int wrist = 0; wrist < 2 && count < solutionCapacity; ++wrist) {
            const double c5 = cos(q5[wrist]);
            const double s5 = sin(q5[wrist]);
            double q6 = referenceDhmQ[5];
            if (absDouble(s5) >= kZeroThreshold) {
                q6 = atan2(sign(s5) * -(T01 * s1 - T11 * c1),
                           sign(s5) * (T00 * s1 - T10 * c1));
            }
            q6 = positiveAngle(q6);
            const double c6 = cos(q6);
            const double s6 = sin(q6);
            const double x04x = -s5 * (T02 * c1 + T12 * s1) -
                c5 * (s6 * (T01 * c1 + T11 * s1) - c6 * (T00 * c1 + T10 * s1));
            const double x04y = c5 * (T20 * c6 - T21 * s6) - T22 * s5;
            const double p13x = d5 * (s6 * (T00 * c1 + T10 * s1) + c6 * (T01 * c1 + T11 * s1)) -
                d6 * (T02 * c1 + T12 * s1) + T03 * c1 + T13 * s1;
            const double p13y = T23 - d1 - d6 * T22 + d5 * (T21 * c6 + T20 * s6);
            double c3 = (p13x * p13x + p13y * p13y - a2 * a2 - a3 * a3) / (2.0 * a2 * a3);
            if (c3 < -1.0 - kZeroThreshold || c3 > 1.0 + kZeroThreshold) continue;
            c3 = clampUnit(c3);
            const double q3Base = acos(c3);
            const double q3[2]{positiveAngle(q3Base), positiveAngle(kTwoPi - q3Base)};
            const double denominator = a2 * a2 + a3 * a3 + 2.0 * a2 * a3 * c3;
            if (absDouble(denominator) < kZeroThreshold) continue;
            const double s3 = sin(q3Base);
            const double armA = a2 + a3 * c3;
            const double armB = a3 * s3;
            const double q2[2]{
                positiveAngle(atan2((armA * p13y - armB * p13x) / denominator,
                                    (armA * p13x + armB * p13y) / denominator)),
                positiveAngle(atan2((armA * p13y + armB * p13x) / denominator,
                                    (armA * p13x - armB * p13y) / denominator))
            };

            for (int elbow = 0; elbow < 2 && count < solutionCapacity; ++elbow) {
                const double q23 = q2[elbow] + q3[elbow];
                const double q4 = positiveAngle(atan2(cos(q23) * x04y - sin(q23) * x04x,
                                                      x04x * cos(q23) + x04y * sin(q23)));
                double raw[kAxisCount]{q1[shoulder] - kPi, q2[elbow], q3[elbow], q4, q5[wrist], q6};
                for (uint8_t joint = 0; joint < kAxisCount; ++joint) {
                    solutions[count][joint] = nearestEquivalentRadians(raw[joint], referenceDhmQ[joint]);
                }
                ++count;
            }
        }
    }
    return count;
}

inline Transform toolPoseForJoints(const RobotModel& model, const double q[kAxisCount]) {
    double dhmQ[kAxisCount]{};
    double homeDhmQ[kAxisCount]{};
    for (uint8_t i = 0; i < kAxisCount; ++i) {
        dhmQ[i] = model.dhmSigns[i] * q[i];
        homeDhmQ[i] = model.dhmSigns[i] * model.qHome[i];
    }
    const Transform flange = viewerFromAr4Transform(ar4ForwardDhm(model, dhmQ));
    const Transform homeFlange = viewerFromAr4Transform(ar4ForwardDhm(model, homeDhmQ));
    const Transform homeInv = inverseRigidTransform(homeFlange);
    return multiply(multiply(flange, homeInv), model.toolBindPose);
}

inline bool solveToolPoseNearest(const RobotModel& model,
                                 const double referenceQ[kAxisCount],
                                 const Transform& targetPose,
                                 double outQ[kAxisCount]) {
    if (!modelIsValid(model) || !transformIsFinite(targetPose) || !outQ) return false;

    double homeDhmQ[kAxisCount]{};
    double referenceDhmQ[kAxisCount]{};
    for (uint8_t i = 0; i < kAxisCount; ++i) {
        homeDhmQ[i] = model.dhmSigns[i] * model.qHome[i];
        referenceDhmQ[i] = model.dhmSigns[i] * referenceQ[i];
    }

    const Transform ar4Home = ar4ForwardDhm(model, homeDhmQ);
    const Transform ar4HomeInViewerFrame = viewerFromAr4Transform(ar4Home);
    const Transform targetAr4ViewerFrame = multiply(multiply(targetPose, inverseRigidTransform(model.toolBindPose)), ar4HomeInViewerFrame);
    const Transform targetAr4Pose = ar4FromViewerTransform(targetAr4ViewerFrame);

    double candidateDhmQs[12][kAxisCount]{};
    const int candidateCount = isUrStyleDhm(model)
        ? urStyleAnalyticInverseDhm(model, targetAr4Pose, referenceDhmQ, candidateDhmQs, 12)
        : ar4AnalyticInverseDhm(model, targetAr4Pose, referenceDhmQ, candidateDhmQs, 12);
    if (candidateCount <= 0) return false;

    int bestIndex = -1;
    double bestDistance = 0.0;
    for (int candidateIndex = 0; candidateIndex < candidateCount; ++candidateIndex) {
        double q[kAxisCount]{};
        bool withinLimits = true;
        double jointDistance = 0.0;
        for (uint8_t i = 0; i < kAxisCount; ++i) {
            q[i] = nearestEquivalentRadians(model.dhmSigns[i] * candidateDhmQs[candidateIndex][i], referenceQ[i]);
            if (model.qMin[i] < model.qMax[i] && (q[i] < model.qMin[i] - 1.0e-6 || q[i] > model.qMax[i] + 1.0e-6)) {
                withinLimits = false;
                break;
            }
            const double delta = wrapRadians(q[i] - referenceQ[i]);
            jointDistance += delta * delta;
        }
        if (!withinLimits) continue;
        if (bestIndex < 0 || jointDistance < bestDistance) {
            bestIndex = candidateIndex;
            bestDistance = jointDistance;
        }
    }

    if (bestIndex < 0) return false;
    for (uint8_t i = 0; i < kAxisCount; ++i) {
        outQ[i] = nearestEquivalentRadians(model.dhmSigns[i] * candidateDhmQs[bestIndex][i], referenceQ[i]);
    }
    return true;
}

// Continuity across a lookahead seam: the last sample of one window against the first of the next.
inline RejectCode motionCheckWindowSeam(const PathSample& previous,
                                        const PathSample& next,
                                        double dtSec,
                                        const MotionProgramSettings& settings,
                                        double* outWorstTcpSpeedMmPerSec) {
    if (!previous.valid || !next.valid || !(dtSec > 1.0e-9)) return RejectCode::Ok;

    double previousPoint[3] = {};
    double nextPoint[3] = {};
    double delta[3] = {};
    translationVector(previous.tcp, previousPoint);
    translationVector(next.tcp, nextPoint);
    subtract3(nextPoint, previousPoint, delta);
    // Reported as a speed, not a distance. Crossing a join covers one control period of perfectly
    // ordinary travel - half a millimetre at 100 mm/s - so a raw step reads as a discontinuity when it
    // is nothing of the kind. A speed can be compared against the programmed one at a glance.
    const double impliedSpeed = length3(delta) / dtSec;
    if (outWorstTcpSpeedMmPerSec && impliedSpeed > *outWorstTcpSpeedMmPerSec) {
        *outWorstTcpSpeedMmPerSec = impliedSpeed;
    }

    // Joints first, because that is where a seam error is unambiguous - no weave allowance, no
    // curvature term, just how far an axis was asked to move in one control period.
    for (uint8_t joint = 0; joint < kAxisCount; ++joint) {
        const double limit = settings.jointVelocityLimitRadSec[joint];
        if (!(limit > 0.0)) continue;
        const double speed = absDouble(wrapRadians(next.q[joint] - previous.q[joint])) / dtSec;
        if (speed > limit * kMotionSeamTolerance) return RejectCode::WindowSeamDiscontinuity;
    }
    return RejectCode::Ok;
}

ROBOT_MOTION_CORE_COLD inline void motionReplanSeedQExact(const RobotModel& model,
                                                          const PathSample& boundary,
                                                          const double actualQ[kAxisCount],
                                                          double outQ[kAxisCount]) {
    if (!actualQ || !outQ) return;
    // With no weave, the actual joints already are the exact state the next window must start from.
    if (absDouble(boundary.weaveOffsetMm[0]) <= 1.0e-12 &&
        absDouble(boundary.weaveOffsetMm[1]) <= 1.0e-12) {
        for (uint8_t joint = 0; joint < kAxisCount; ++joint) outQ[joint] = actualQ[joint];
        return;
    }
    double exactBaseQ[kAxisCount] = {};
    if (solveToolPoseNearest(model, boundary.q, boundary.baseTcp, exactBaseQ)) {
        for (uint8_t joint = 0; joint < kAxisCount; ++joint) {
            outQ[joint] = actualQ[joint] + (exactBaseQ[joint] - boundary.q[joint]);
        }
        return;
    }
    motionReplanSeedQ(boundary, actualQ, outQ);
}

inline bool nearSingularity(const double q[kAxisCount], double thresholdRad) {
    if (!q) return true;
    const double threshold = thresholdRad > 0.0 ? thresholdRad : (5.0 * 3.14159265358979323846 / 180.0);
    return absDouble(sin(q[4])) <= sin(threshold);
}

inline double wristSingularityMarginRad(const double q[kAxisCount]) {
    return q ? absDouble(q[4]) : 0.0;
}

// singularityThresholdRad affects Abort policy and reporting only. Physical limits bound speed.

inline bool jointConfigMatches(const double actualQ[kAxisCount],
                               const double expectedQ[kAxisCount],
                               double toleranceRad) {
    if (!actualQ || !expectedQ) return false;
    const double tolerance = toleranceRad > 0.0 ? toleranceRad : (0.5 * 3.14159265358979323846 / 180.0);
    for (uint8_t i = 0; i < kAxisCount; ++i) {
        if (!isFinite(actualQ[i]) || !isFinite(expectedQ[i])) return false;
        if (absDouble(wrapRadians(actualQ[i] - expectedQ[i])) > tolerance) return false;
    }
    return true;
}

inline RejectCode planMoveLEndpoint(const MoveLInput& input, MoveLPlan* plan) {
    if (!plan) return RejectCode::BadPose;
    memset(plan, 0, sizeof(*plan));
    if (!modelIsValid(input.model)) return RejectCode::ModelNotLoaded;
    if (!transformIsFinite(input.targetTcp)) return RejectCode::BadPose;
    for (uint8_t i = 0; i < kAxisCount; ++i) {
        if (!isFinite(input.startQ[i])) return RejectCode::BadTarget;
        if (input.model.qMin[i] < input.model.qMax[i] &&
            (input.startQ[i] < input.model.qMin[i] - 1.0e-6 || input.startQ[i] > input.model.qMax[i] + 1.0e-6)) {
            return RejectCode::CurrentJointLimit;
        }
    }

    plan->startTcp = toolPoseForJoints(input.model, input.startQ);
    // A taught MoveL carries both a TCP pose and the joint configuration intended at that pose.
    const double* endpointReference = input.requireTargetConfig ? input.targetConfigQ : input.startQ;
    if (!solveToolPoseNearest(input.model, endpointReference, input.targetTcp, plan->endpointQ)) {
        return RejectCode::NoIkSolution;
    }
    if (input.requireTargetConfig &&
        !jointConfigMatches(plan->endpointQ, input.targetConfigQ, input.targetConfigToleranceRad)) {
        return RejectCode::TargetConfigMismatch;
    }
    plan->lineLengthMm = distanceTranslation(plan->startTcp, input.targetTcp);
    const double speedMmPerSec = input.speedMmPerSec > 0.0 ? input.speedMmPerSec : 25.0;
    plan->durationSec = plan->lineLengthMm > 0.0 ? plan->lineLengthMm / speedMmPerSec : 0.0;
    if (input.sampleMm > 0.0) {
        plan->sampleCount = static_cast<uint32_t>(ceil(plan->lineLengthMm / input.sampleMm));
        if (plan->sampleCount < 1) plan->sampleCount = 1;
    } else {
        plan->sampleCount = 1;
    }
    plan->sampleDurationSec = plan->sampleCount > 0
        ? plan->durationSec / static_cast<double>(plan->sampleCount)
        : 0.0;
    return RejectCode::Ok;
}

inline double moveLSampleTimeSeconds(const MoveLPlan& plan, uint32_t sampleIndex) {
    if (sampleIndex == 0) return 0.0;
    if (sampleIndex >= plan.sampleCount) return plan.durationSec;
    return plan.sampleDurationSec * static_cast<double>(sampleIndex);
}

inline RejectCode beginMoveLStream(const MoveLInput& input, MoveLStream* stream) {
    if (!stream) return RejectCode::BadPose;
    memset(stream, 0, sizeof(*stream));
    stream->input = input;
    const RejectCode endpoint = planMoveLEndpoint(input, &stream->plan);
    if (endpoint != RejectCode::Ok) return endpoint;
    for (uint8_t i = 0; i < kAxisCount; ++i) stream->previousQ[i] = input.startQ[i];
    stream->nextSample = 1;
    return RejectCode::Ok;
}

inline RejectCode nextMoveLSample(MoveLStream* stream, double outQ[kAxisCount], Transform* outTcp, uint32_t* sampleIndex) {
    if (!stream || !outQ) return RejectCode::BadPose;
    if (stream->nextSample > stream->plan.sampleCount) return RejectCode::Ok;
    const uint32_t currentSample = stream->nextSample;
    const double t = static_cast<double>(currentSample) / static_cast<double>(stream->plan.sampleCount);
    const Transform pose = interpolatePoseLinear(stream->plan.startTcp, stream->input.targetTcp, t);
    if (!solveToolPoseNearest(stream->input.model, stream->previousQ, pose, outQ)) {
        if (sampleIndex) *sampleIndex = currentSample;
        return RejectCode::NoIkSolution;
    }
    if (singularityPolicyAborts(stream->input.singularityPolicy) &&
        nearSingularity(outQ, stream->input.singularityThresholdRad)) {
        if (sampleIndex) *sampleIndex = currentSample;
        return RejectCode::Singularity;
    }
    for (uint8_t i = 0; i < kAxisCount; ++i) stream->previousQ[i] = outQ[i];
    if (outTcp) *outTcp = pose;
    if (sampleIndex) *sampleIndex = currentSample;
    ++stream->nextSample;
    return RejectCode::Ok;
}

inline double motionCommandJointSpeedDegPerSec(const MotionCommand& command,
                                               const MotionProgramSettings& settings) {
    if (isFinite(command.jointSpeedDegPerSec) && command.jointSpeedDegPerSec > 0.0) {
        return command.jointSpeedDegPerSec;
    }
    return settings.defaultJointSpeedDegPerSec > 0.0 ? settings.defaultJointSpeedDegPerSec : 10.0;
}

inline double motionCommandLinearSpeedMmPerSec(const MotionCommand& command,
                                               const MotionProgramSettings& settings) {
    if (isFinite(command.linearSpeedMmPerSec) && command.linearSpeedMmPerSec > 0.0) {
        return command.linearSpeedMmPerSec;
    }
    return settings.defaultLinearSpeedMmPerSec > 0.0 ? settings.defaultLinearSpeedMmPerSec : 25.0;
}

struct JerkSpeedChangeProfile {
    double phaseDuration[3];
    double jerkSign;
    double durationSec;
    double distance;
};

inline JerkSpeedChangeProfile makeJerkSpeedChangeProfile(double startSpeed,
                                                         double endSpeed,
                                                         double accelLimit,
                                                         double jerkLimit) {
    JerkSpeedChangeProfile profile = {};
    if (!isFinite(startSpeed) || startSpeed < 0.0) startSpeed = 0.0;
    if (!isFinite(endSpeed) || endSpeed < 0.0) endSpeed = 0.0;
    const double dvSigned = endSpeed - startSpeed;
    const double dv = absDouble(dvSigned);
    if (dv <= 1.0e-12) {
        return profile;
    }

    double accel = isFinite(accelLimit) && accelLimit > 1.0e-9 ? accelLimit : 0.0;
    double jerk = isFinite(jerkLimit) && jerkLimit > 1.0e-9 ? jerkLimit : 0.0;
    if (accel <= 0.0) accel = 1.0;
    if (jerk <= 0.0) jerk = accel * 1000.0;
    profile.jerkSign = dvSigned >= 0.0 ? 1.0 : -1.0;

    const double triangularDv = accel * accel / jerk;
    if (dv <= triangularDv) {
        const double tj = sqrt(dv / jerk);
        profile.phaseDuration[0] = tj;
        profile.phaseDuration[1] = 0.0;
        profile.phaseDuration[2] = tj;
    } else {
        const double tj = accel / jerk;
        const double ta = (dv - triangularDv) / accel;
        profile.phaseDuration[0] = tj;
        profile.phaseDuration[1] = ta;
        profile.phaseDuration[2] = tj;
    }
    profile.durationSec = profile.phaseDuration[0] + profile.phaseDuration[1] + profile.phaseDuration[2];

    double velocity = startSpeed;
    double acceleration = 0.0;
    double distance = 0.0;
    const double jerks[3] = {
        profile.jerkSign * jerk,
        0.0,
        -profile.jerkSign * jerk
    };
    for (uint8_t i = 0; i < 3; ++i) {
        const double dt = profile.phaseDuration[i];
        const double j = jerks[i];
        distance += velocity * dt + 0.5 * acceleration * dt * dt + (j * dt * dt * dt) / 6.0;
        velocity += acceleration * dt + 0.5 * j * dt * dt;
        acceleration += j * dt;
    }
    profile.distance = distance > 0.0 ? distance : 0.0;
    return profile;
}

inline double jerkSpeedChangeDistanceAtTime(double startSpeed,
                                            double endSpeed,
                                            double accelLimit,
                                            double jerkLimit,
                                            double timeSec,
                                            double* outSpeed = nullptr) {
    const JerkSpeedChangeProfile profile = makeJerkSpeedChangeProfile(startSpeed, endSpeed, accelLimit, jerkLimit);
    double remaining = timeSec;
    if (remaining <= 0.0) {
        if (outSpeed) *outSpeed = startSpeed;
        return 0.0;
    }
    if (remaining >= profile.durationSec) {
        if (outSpeed) *outSpeed = endSpeed;
        return profile.distance;
    }

    double velocity = startSpeed;
    double acceleration = 0.0;
    double distance = 0.0;
    const double positiveJerk = isFinite(jerkLimit) && jerkLimit > 1.0e-9 ? jerkLimit : 0.0;
    const double positiveAccel = isFinite(accelLimit) && accelLimit > 1.0e-9 ? accelLimit : 0.0;
    const double jerk = positiveJerk > 0.0 ? positiveJerk : (positiveAccel > 0.0 ? positiveAccel * 1000.0 : 1000.0);
    const double jerks[3] = {
        profile.jerkSign * jerk,
        0.0,
        -profile.jerkSign * jerk
    };
    for (uint8_t i = 0; i < 3; ++i) {
        const double dt = remaining < profile.phaseDuration[i] ? remaining : profile.phaseDuration[i];
        const double j = jerks[i];
        distance += velocity * dt + 0.5 * acceleration * dt * dt + (j * dt * dt * dt) / 6.0;
        velocity += acceleration * dt + 0.5 * j * dt * dt;
        acceleration += j * dt;
        remaining -= dt;
        if (remaining <= 1.0e-12) break;
    }
    if (outSpeed) *outSpeed = velocity > 0.0 ? velocity : 0.0;
    return distance > 0.0 ? distance : 0.0;
}

inline double jerkLimitedReachableSpeed(double startSpeed,
                                        double distance,
                                        double maxSpeed,
                                        double accelLimit,
                                        double jerkLimit) {
    if (!isFinite(distance) || distance <= 1.0e-12) return startSpeed;
    double low = startSpeed > 0.0 ? startSpeed : 0.0;
    double high = maxSpeed > low ? maxSpeed : low;
    for (uint8_t i = 0; i < 32; ++i) {
        const double mid = 0.5 * (low + high);
        const double needed = makeJerkSpeedChangeProfile(startSpeed, mid, accelLimit, jerkLimit).distance;
        if (needed <= distance) {
            low = mid;
        } else {
            high = mid;
        }
    }
    return low;
}

inline double motionSegmentDistanceAtTime(const MotionSegment& segment, double localTimeSec) {
    if (localTimeSec <= 0.0 || segment.length <= 1.0e-12) return 0.0;
    if (localTimeSec >= segment.durationSec) return segment.length;
    const MotionTimeProfile& profile = segment.timeProfile;
    if (localTimeSec <= profile.accelTimeSec) {
        return jerkSpeedChangeDistanceAtTime(profile.startSpeed,
                                             profile.peakSpeed,
                                             segment.acceleration,
                                             segment.jerk,
                                             localTimeSec);
    }
    double remainingTime = localTimeSec - profile.accelTimeSec;
    double distance = profile.accelDistance;
    if (remainingTime <= profile.cruiseTimeSec) {
        return distance + profile.peakSpeed * remainingTime;
    }
    distance += profile.cruiseDistance;
    remainingTime -= profile.cruiseTimeSec;
    return distance + jerkSpeedChangeDistanceAtTime(profile.peakSpeed,
                                                    profile.endSpeed,
                                                    segment.acceleration,
                                                    segment.jerk,
                                                    remainingTime);
}

inline double motionSegmentSpeedAtTime(const MotionSegment& segment, double localTimeSec) {
    if (localTimeSec <= 0.0 || segment.length <= 1.0e-12) return segment.startSpeed;
    if (localTimeSec >= segment.durationSec) return segment.endSpeed;
    const MotionTimeProfile& profile = segment.timeProfile;
    double speed = profile.startSpeed;
    if (localTimeSec <= profile.accelTimeSec) {
        jerkSpeedChangeDistanceAtTime(profile.startSpeed,
                                      profile.peakSpeed,
                                      segment.acceleration,
                                      segment.jerk,
                                      localTimeSec,
                                      &speed);
        return speed;
    }
    double remainingTime = localTimeSec - profile.accelTimeSec;
    if (remainingTime <= profile.cruiseTimeSec) return profile.peakSpeed;
    remainingTime -= profile.cruiseTimeSec;
    jerkSpeedChangeDistanceAtTime(profile.peakSpeed,
                                  profile.endSpeed,
                                  segment.acceleration,
                                  segment.jerk,
                                  remainingTime,
                                  &speed);
    return speed;
}

inline double clampAccelerationForQuinticEdge(double acceleration,
                                              double length,
                                              double duration,
                                              double startSpeed,
                                              double endSpeed) {
    if (!isFinite(acceleration) || duration <= 1.0e-9 || length <= 1.0e-12) return 0.0;
    const double meanSpeed = length / duration;
    double bound = (absDouble(startSpeed) + absDouble(endSpeed) + absDouble(meanSpeed)) * 6.0 / duration;
    if (!isFinite(bound) || bound <= 1.0e-9) bound = 0.0;
    if (bound > 0.0) {
        if (acceleration > bound) return bound;
        if (acceleration < -bound) return -bound;
    }
    return acceleration;
}

inline void configureMotionTrajectoryEdgePolynomial(MotionTrajectoryEdge* edge) {
    if (edge) edge->useQuintic = 0;
    if (!edge || edge->durationSec <= 1.0e-9 || edge->length <= 1.0e-12) {
        if (edge) {
            edge->startAcceleration = 0.0;
            edge->endAcceleration = 0.0;
            edge->startJerk = 0.0;
            edge->endJerk = 0.0;
            edge->quinticC3 = 0.0;
            edge->quinticC4 = 0.0;
            edge->quinticC5 = 0.0;
            edge->quinticC6 = 0.0;
            edge->quinticC7 = 0.0;
        }
        return;
    }

    const double t = edge->durationSec;
    const double t2 = t * t;
    const double t3 = t2 * t;
    const double t4 = t3 * t;
    const double t5 = t4 * t;
    const double t6 = t5 * t;
    const double t7 = t6 * t;
    const double baseStartAcceleration = clampAccelerationForQuinticEdge(edge->startAcceleration,
                                                                         edge->length,
                                                                         t,
                                                                         edge->startSpeed,
                                                                         edge->endSpeed);
    const double baseEndAcceleration = clampAccelerationForQuinticEdge(edge->endAcceleration,
                                                                       edge->length,
                                                                       t,
                                                                       edge->startSpeed,
                                                                       edge->endSpeed);
    const double baseStartJerk = isFinite(edge->startJerk) ? edge->startJerk : 0.0;
    const double baseEndJerk = isFinite(edge->endJerk) ? edge->endJerk : 0.0;
    const double x = edge->length;
    const double v0 = edge->startSpeed;
    const double v1 = edge->endSpeed;
    const double speedLimit = edge->speedLimit > 1.0e-9 ? edge->speedLimit : edge->peakSpeed;
    const double negativeSpeedTolerance = speedLimit > 1.0e-9 ? speedLimit * 1.0e-4 + 1.0e-7 : 1.0e-7;
    const double derivativeScales[] = {1.0, 0.5, 0.25, 0.125, 0.0};
    for (double derivativeScale : derivativeScales) {
        const double a0 = baseStartAcceleration * derivativeScale;
        const double a1 = baseEndAcceleration * derivativeScale;
        const double j0 = baseStartJerk * derivativeScale;
        const double j1 = baseEndJerk * derivativeScale;
        const double p = x - (v0 * t + 0.5 * a0 * t2 + (j0 * t3) / 6.0);
        const double v = v1 * t - (v0 * t + a0 * t2 + 0.5 * j0 * t3);
        const double a = a1 * t2 - (a0 * t2 + j0 * t3);
        const double j = j1 * t3 - j0 * t3;
        const double c3 = j0 / 6.0;
        const double c4 = (35.0 * p - 15.0 * v + 2.5 * a - j / 6.0) / t4;
        const double c5 = (-84.0 * p + 39.0 * v - 7.0 * a + 0.5 * j) / t5;
        const double c6 = (70.0 * p - 34.0 * v + 6.5 * a - 0.5 * j) / t6;
        const double c7 = (-20.0 * p + 10.0 * v - 2.0 * a + j / 6.0) / t7;
        if (!isFinite(c3) || !isFinite(c4) || !isFinite(c5) || !isFinite(c6) || !isFinite(c7)) {
            continue;
        }

        double maxSpeed = edge->startSpeed > edge->endSpeed ? edge->startSpeed : edge->endSpeed;
        bool monotonic = true;
        for (uint8_t sample = 0; sample <= 64; ++sample) {
            const double tt = t * (static_cast<double>(sample) / 64.0);
            const double tt2 = tt * tt;
            const double tt3 = tt2 * tt;
            const double tt4 = tt2 * tt2;
            const double tt5 = tt4 * tt;
            const double tt6 = tt5 * tt;
            const double speed = v0 +
                a0 * tt +
                3.0 * c3 * tt2 +
                4.0 * c4 * tt3 +
                5.0 * c5 * tt4 +
                6.0 * c6 * tt5 +
                7.0 * c7 * tt6;
            if (!isFinite(speed) || speed < -negativeSpeedTolerance) {
                monotonic = false;
                break;
            }
            if (speed > maxSpeed) maxSpeed = speed;
        }
        if (!monotonic || (speedLimit > 0.0 && maxSpeed > speedLimit + 1.0e-6)) {
            continue;
        }

        edge->startAcceleration = a0;
        edge->endAcceleration = a1;
        edge->startJerk = j0;
        edge->endJerk = j1;
        edge->quinticC3 = c3;
        edge->quinticC4 = c4;
        edge->quinticC5 = c5;
        edge->quinticC6 = c6;
        edge->quinticC7 = c7;
        edge->useQuintic = 1;
        return;
    }

    edge->startAcceleration = 0.0;
    edge->endAcceleration = 0.0;
    edge->startJerk = 0.0;
    edge->endJerk = 0.0;
    edge->quinticC3 = 0.0;
    edge->quinticC4 = 0.0;
    edge->quinticC5 = 0.0;
    edge->quinticC6 = 0.0;
    edge->quinticC7 = 0.0;
}

inline bool configureMotionTrajectoryEdgePolynomialAtDuration(MotionTrajectoryEdge* edge,
                                                              const MotionTrajectoryEdge& base,
                                                              double durationSec) {
    if (!edge || !isFinite(durationSec) || durationSec <= 1.0e-9) return false;
    *edge = base;
    edge->durationSec = durationSec;
    configureMotionTrajectoryEdgePolynomial(edge);
    return edge->useQuintic != 0;
}

inline bool fitMotionTrajectoryEdgePolynomial(MotionTrajectoryEdge* edge) {
    if (!edge) return false;
    if (edge->durationSec <= 1.0e-9 || edge->length <= 1.0e-12) {
        configureMotionTrajectoryEdgePolynomial(edge);
        return true;
    }

    const MotionTrajectoryEdge base = *edge;
    if (configureMotionTrajectoryEdgePolynomialAtDuration(edge, base, base.durationSec)) return true;

    const double maxExtension = base.durationSec > 0.25 ? base.durationSec : 0.25;
    const double maxDuration = base.durationSec + maxExtension;
    double low = base.durationSec;
    double high = base.durationSec;
    bool found = false;
    for (uint8_t attempt = 0; attempt < 48; ++attempt) {
        high *= 1.125;
        if (high > maxDuration) high = maxDuration;
        if (configureMotionTrajectoryEdgePolynomialAtDuration(edge, base, high)) {
            found = true;
            break;
        }
        if (high >= maxDuration - 1.0e-12) break;
        low = high;
    }
    if (!found) {
        *edge = base;
        configureMotionTrajectoryEdgePolynomial(edge);
        return edge->useQuintic != 0;
    }

    for (uint8_t iteration = 0; iteration < 24; ++iteration) {
        const double mid = 0.5 * (low + high);
        MotionTrajectoryEdge candidate = base;
        if (configureMotionTrajectoryEdgePolynomialAtDuration(&candidate, base, mid)) {
            high = mid;
        } else {
            low = mid;
        }
    }
    const bool fitted = configureMotionTrajectoryEdgePolynomialAtDuration(edge, base, high);
    if (fitted && high > base.durationSec * 1.001 + 1.0e-6) {
        edge->capReasonMask |= static_cast<uint32_t>(MotionCapJerk);
    }
    return fitted;
}

inline double motionTrajectoryEdgeDistanceAtTime(const MotionTrajectoryEdge& edge, double localTimeSec) {
    if (localTimeSec <= 0.0 || edge.length <= 1.0e-12) return 0.0;
    if (localTimeSec >= edge.durationSec) return edge.length;
    if (!edge.useQuintic || edge.durationSec <= 1.0e-9) return 0.0;
    const double t = localTimeSec;
    const double t2 = t * t;
    const double t3 = t2 * t;
    const double t4 = t2 * t2;
    const double t5 = t4 * t;
    const double t6 = t5 * t;
    const double t7 = t6 * t;
    const double distance = edge.startSpeed * t +
        0.5 * edge.startAcceleration * t2 +
        edge.quinticC3 * t3 +
        edge.quinticC4 * t4 +
        edge.quinticC5 * t5 +
        edge.quinticC6 * t6 +
        edge.quinticC7 * t7;
    if (distance <= 0.0) return 0.0;
    if (distance >= edge.length) return edge.length;
    return distance;
}

inline double motionTrajectoryEdgeSpeedAtTime(const MotionTrajectoryEdge& edge, double localTimeSec) {
    if (localTimeSec <= 0.0 || edge.length <= 1.0e-12) return edge.startSpeed;
    if (localTimeSec >= edge.durationSec) return edge.endSpeed;
    if (!edge.useQuintic || edge.durationSec <= 1.0e-9) return 0.0;
    const double t = localTimeSec;
    const double t2 = t * t;
    const double t3 = t2 * t;
    const double t4 = t2 * t2;
    const double t5 = t4 * t;
    const double t6 = t5 * t;
    const double speed = edge.startSpeed +
        edge.startAcceleration * t +
        3.0 * edge.quinticC3 * t2 +
        4.0 * edge.quinticC4 * t3 +
        5.0 * edge.quinticC5 * t4 +
        6.0 * edge.quinticC6 * t5 +
        7.0 * edge.quinticC7 * t6;
    return speed > 0.0 ? speed : 0.0;
}

inline double motionTrajectoryEdgeAccelerationAtTime(const MotionTrajectoryEdge& edge, double localTimeSec) {
    if (edge.length <= 1.0e-12 || edge.durationSec <= 1.0e-9) return 0.0;
    if (!edge.useQuintic) return 0.0;
    const double t = localTimeSec <= 0.0 ? 0.0 : (localTimeSec >= edge.durationSec ? edge.durationSec : localTimeSec);
    const double t2 = t * t;
    const double t3 = t2 * t;
    const double t4 = t2 * t2;
    const double t5 = t4 * t;
    return edge.startAcceleration +
        6.0 * edge.quinticC3 * t +
        12.0 * edge.quinticC4 * t2 +
        20.0 * edge.quinticC5 * t3 +
        30.0 * edge.quinticC6 * t4 +
        42.0 * edge.quinticC7 * t5;
}

inline double motionTrajectoryEdgeJerkAtTime(const MotionTrajectoryEdge& edge, double localTimeSec) {
    if (edge.length <= 1.0e-12 || edge.durationSec <= 1.0e-9) return 0.0;
    if (!edge.useQuintic) return 0.0;
    const double t = localTimeSec <= 0.0 ? 0.0 : (localTimeSec >= edge.durationSec ? edge.durationSec : localTimeSec);
    const double t2 = t * t;
    const double t3 = t2 * t;
    const double t4 = t2 * t2;
    return 6.0 * edge.quinticC3 +
        24.0 * edge.quinticC4 * t +
        60.0 * edge.quinticC5 * t2 +
        120.0 * edge.quinticC6 * t3 +
        210.0 * edge.quinticC7 * t4;
}

inline void motionTrajectoryEdgeDerivativeBounds(const MotionTrajectoryEdge& edge,
                                                 double* outPeakSpeed,
                                                 double* outPeakAcceleration,
                                                 double* outPeakJerk) {
    double peakSpeed = edge.startSpeed > edge.endSpeed ? edge.startSpeed : edge.endSpeed;
    if (edge.peakSpeed > peakSpeed) peakSpeed = edge.peakSpeed;
    double peakAcceleration = 0.0;
    double peakJerk = 0.0;
    if (edge.durationSec > 1.0e-9 && edge.length > 1.0e-12) {
        for (uint8_t sample = 0; sample <= 16; ++sample) {
            const double localTime = edge.durationSec * (static_cast<double>(sample) / 16.0);
            const double speed = motionTrajectoryEdgeSpeedAtTime(edge, localTime);
            const double accel = absDouble(motionTrajectoryEdgeAccelerationAtTime(edge, localTime));
            const double jerk = absDouble(motionTrajectoryEdgeJerkAtTime(edge, localTime));
            if (isFinite(speed) && speed > peakSpeed) peakSpeed = speed;
            if (isFinite(accel) && accel > peakAcceleration) peakAcceleration = accel;
            if (isFinite(jerk) && jerk > peakJerk) peakJerk = jerk;
        }
    }
    if (outPeakSpeed) *outPeakSpeed = peakSpeed;
    if (outPeakAcceleration) *outPeakAcceleration = peakAcceleration;
    if (outPeakJerk) *outPeakJerk = peakJerk;
}

inline double motionTrajectoryEdgeNominalSpeed(const MotionTrajectoryEdge& edge, const MotionSegment& pathSegment) {
    double limit = pathSegment.nominalSpeed > 1.0e-9 ? pathSegment.nominalSpeed : 1.0;
    if (edge.speedLimit > 1.0e-9 && edge.speedLimit < limit) limit = edge.speedLimit;
    return limit;
}

inline double motionEffectiveAcceleration(double acceleration, double jerk, double length) {
    double out = acceleration > 1.0e-9 ? acceleration : 1.0;
    if (jerk > 1.0e-9 && length > 1.0e-9) {
        const double jerkLimited = pow(jerk * jerk * length, 1.0 / 3.0);
        if (isFinite(jerkLimited) && jerkLimited > 1.0e-9 && jerkLimited < out) {
            out = jerkLimited;
        }
    }
    return out;
}

inline void markMotionCap(MotionSegment* segment, uint32_t reason) {
    if (!segment) return;
    segment->capReasonMask |= reason;
}

inline void markMotionContext(MotionSegment* segment, uint32_t reason) {
    if (!segment) return;
    segment->contextMask |= reason;
    markMotionCap(segment, reason);
}

inline void markMotionSpeedLimit(MotionSegment* segment, uint32_t reason) {
    if (!segment) return;
    segment->activeSpeedLimitMask |= reason;
    markMotionCap(segment, reason);
}

inline void markMotionAccelerationLimit(MotionSegment* segment, uint32_t reason) {
    if (!segment) return;
    segment->activeAccelerationLimitMask |= reason;
    markMotionCap(segment, reason);
}

inline void initializeMotionCapDiagnostics(MotionSegment* segment) {
    if (!segment) return;
    segment->speedAfterCommand = segment->commandSpeed > 0.0 ? segment->commandSpeed : segment->nominalSpeed;
    segment->speedAfterJointVelocity = segment->speedAfterCommand;
    segment->speedAfterJointAcceleration = segment->speedAfterCommand;
    segment->speedAfterPathAcceleration = segment->speedAfterCommand;
    segment->speedAfterSingularity = segment->speedAfterCommand;
    segment->speedAfterStepRate = segment->speedAfterCommand;
    segment->speedAfterJerk = segment->speedAfterCommand;
    segment->speedAfterWeave = segment->speedAfterCommand;
    segment->finalNominalSpeed = segment->nominalSpeed;
    segment->accelerationAfterCommand = segment->commandAcceleration > 0.0 ? segment->commandAcceleration : segment->acceleration;
    segment->accelerationAfterJointAcceleration = segment->accelerationAfterCommand;
    segment->accelerationAfterJerk = segment->accelerationAfterCommand;
    if (segment->nominalSpeed > 0.0 && segment->commandSpeed > 0.0 && segment->nominalSpeed < segment->commandSpeed - 1.0e-9) {
        markMotionSpeedLimit(segment, MotionCapCommandSpeed);
    }
}

inline void recordMotionSpeedCap(MotionSegment* segment, double cap, uint32_t reason) {
    if (!segment || !isFinite(cap) || cap <= 1.0e-9) return;
    if (reason & MotionCapJointVelocity) {
        if (segment->speedAfterJointVelocity <= 0.0 || cap < segment->speedAfterJointVelocity) segment->speedAfterJointVelocity = cap;
    }
    if (reason & MotionCapJointAcceleration) {
        if (segment->speedAfterJointAcceleration <= 0.0 || cap < segment->speedAfterJointAcceleration) segment->speedAfterJointAcceleration = cap;
    }
    if (reason & MotionCapPathAcceleration) {
        if (segment->speedAfterPathAcceleration <= 0.0 || cap < segment->speedAfterPathAcceleration) segment->speedAfterPathAcceleration = cap;
    }
    if (reason & MotionCapSingularity) {
        if (segment->speedAfterSingularity <= 0.0 || cap < segment->speedAfterSingularity) segment->speedAfterSingularity = cap;
    }
    if (reason & MotionCapStepRate) {
        if (segment->speedAfterStepRate <= 0.0 || cap < segment->speedAfterStepRate) segment->speedAfterStepRate = cap;
    }
    if (reason & MotionCapJerk) {
        if (segment->speedAfterJerk <= 0.0 || cap < segment->speedAfterJerk) segment->speedAfterJerk = cap;
    }
    if (reason & MotionCapWeave) {
        if (segment->speedAfterWeave <= 0.0 || cap < segment->speedAfterWeave) segment->speedAfterWeave = cap;
    }
}

inline void recordMotionAccelerationCap(MotionSegment* segment, double cap, uint32_t reason) {
    if (!segment || !isFinite(cap) || cap <= 1.0e-9) return;
    if (reason & MotionCapJointAcceleration) {
        if (segment->accelerationAfterJointAcceleration <= 0.0 || cap < segment->accelerationAfterJointAcceleration) {
            segment->accelerationAfterJointAcceleration = cap;
        }
    }
    if (reason & MotionCapJerk) {
        if (segment->accelerationAfterJerk <= 0.0 || cap < segment->accelerationAfterJerk) {
            segment->accelerationAfterJerk = cap;
        }
    }
}

inline void capMotionSegmentSpeed(MotionSegment* segment, double cap, uint32_t reason) {
    if (!segment || !isFinite(cap) || cap <= 1.0e-9) return;
    recordMotionSpeedCap(segment, cap, reason);
    if (cap < segment->nominalSpeed) {
        segment->nominalSpeed = cap;
        markMotionSpeedLimit(segment, reason);
    }
}

inline void capMotionSegmentAcceleration(MotionSegment* segment, double cap, uint32_t reason) {
    if (!segment || !isFinite(cap) || cap <= 1.0e-9) return;
    recordMotionAccelerationCap(segment, cap, reason);
    if (cap < segment->acceleration) {
        segment->acceleration = cap;
        markMotionAccelerationLimit(segment, reason);
    }
}

inline void updateMotionSegmentOrientationDemand(const MotionProgramSettings& settings,
                                                 MotionSegment* segment) {
    if (!segment || segment->kind == MotionSegmentKind::MoveJ) return;
    segment->orientationLengthRad = segment->kind == MotionSegmentKind::MoveLBlend
        ? orientationDeltaAngleRad(segment->startTcp, segment->blendPlan.cornerTcp) +
              orientationDeltaAngleRad(segment->blendPlan.cornerTcp, segment->endTcp)
        : orientationDeltaAngleRad(segment->startTcp, segment->endTcp);
    segment->orientationRatioRadPerPathUnit = segment->length > 1.0e-9
        ? segment->orientationLengthRad / segment->length
        : 0.0;
    const double ratio = segment->orientationRatioRadPerPathUnit;
    if (ratio <= 1.0e-12) return;

    const double angularSpeed = isFinite(settings.defaultToolAngularSpeedRadSec) && settings.defaultToolAngularSpeedRadSec > 1.0e-9
        ? settings.defaultToolAngularSpeedRadSec
        : 0.0;
    if (angularSpeed > 0.0) {
        capMotionSegmentSpeed(segment, angularSpeed / ratio, MotionCapJointVelocity);
    }
    const double angularAccel = isFinite(settings.defaultToolAngularAccelerationRadSec2) && settings.defaultToolAngularAccelerationRadSec2 > 1.0e-9
        ? settings.defaultToolAngularAccelerationRadSec2
        : 0.0;
    if (angularAccel > 0.0) {
        capMotionSegmentAcceleration(segment, angularAccel / ratio, MotionCapJointAcceleration);
    }
    const double angularJerk = isFinite(settings.defaultToolAngularJerkRadSec3) && settings.defaultToolAngularJerkRadSec3 > 1.0e-9
        ? settings.defaultToolAngularJerkRadSec3
        : 0.0;
    if (angularJerk > 0.0) {
        const double pathJerkCap = angularJerk / ratio;
        if (pathJerkCap > 1.0e-9 && pathJerkCap < segment->jerk) {
            segment->jerk = pathJerkCap;
            markMotionAccelerationLimit(segment, MotionCapJerk);
            recordMotionAccelerationCap(segment, pathJerkCap, MotionCapJerk);
        }
    }
}

inline void profileMotionSegment(MotionSegment* segment, double startSpeed, double endSpeed, double startTimeSec) {
    if (!segment) return;
    segment->startSpeed = isFinite(startSpeed) && startSpeed > 0.0 ? startSpeed : 0.0;
    segment->endSpeed = isFinite(endSpeed) && endSpeed > 0.0 ? endSpeed : 0.0;
    segment->startTimeSec = isFinite(startTimeSec) && startTimeSec > 0.0 ? startTimeSec : 0.0;
    segment->durationSec = 0.0;
    segment->accelTimeSec = 0.0;
    segment->cruiseTimeSec = 0.0;
    segment->decelTimeSec = 0.0;
    segment->accelDistance = 0.0;
    segment->cruiseDistance = 0.0;
    memset(&segment->timeProfile, 0, sizeof(segment->timeProfile));
    if (!isFinite(segment->length) || segment->length <= 1.0e-12) {
        segment->length = 0.0;
        return;
    }

    const double accel = isFinite(segment->acceleration) && segment->acceleration > 1.0e-9 ? segment->acceleration : 0.0;
    if (accel <= 1.0e-9) {
        segment->acceleration = 1.0;
    }
    segment->acceleration = isFinite(segment->acceleration) && segment->acceleration > 1.0e-9 ? segment->acceleration : 1.0;
    segment->jerk = isFinite(segment->jerk) && segment->jerk > 1.0e-9 ? segment->jerk : segment->acceleration * 1000.0;
    const double vmax = segment->nominalSpeed > 1.0e-9 ? segment->nominalSpeed : 1.0;
    double peakLow = segment->startSpeed > segment->endSpeed ? segment->startSpeed : segment->endSpeed;
    double peakHigh = vmax > peakLow ? vmax : peakLow;
    double peak = peakLow;
    for (uint8_t i = 0; i < 36; ++i) {
        const double mid = 0.5 * (peakLow + peakHigh);
        const double accelDistance = makeJerkSpeedChangeProfile(segment->startSpeed, mid, segment->acceleration, segment->jerk).distance;
        const double decelDistance = makeJerkSpeedChangeProfile(mid, segment->endSpeed, segment->acceleration, segment->jerk).distance;
        if (accelDistance + decelDistance <= segment->length + 1.0e-9) {
            peak = mid;
            peakLow = mid;
        } else {
            peakHigh = mid;
        }
    }

    segment->peakSpeed = peak;
    const JerkSpeedChangeProfile accelProfile = makeJerkSpeedChangeProfile(segment->startSpeed, peak, segment->acceleration, segment->jerk);
    const JerkSpeedChangeProfile decelProfile = makeJerkSpeedChangeProfile(peak, segment->endSpeed, segment->acceleration, segment->jerk);
    segment->accelTimeSec = accelProfile.durationSec;
    segment->decelTimeSec = decelProfile.durationSec;
    segment->accelDistance = accelProfile.distance;
    const double decelDistance = decelProfile.distance;
    segment->cruiseDistance = segment->length - segment->accelDistance - decelDistance;
    if (segment->cruiseDistance < 0.0) segment->cruiseDistance = 0.0;
    segment->cruiseTimeSec = peak > 1.0e-9 ? segment->cruiseDistance / peak : 0.0;
    segment->durationSec = segment->accelTimeSec + segment->cruiseTimeSec + segment->decelTimeSec;
    segment->finalNominalSpeed = segment->nominalSpeed;

    if (peak < vmax - 1.0e-9) markMotionContext(segment, MotionCapLookahead);

    segment->timeProfile.startSpeed = segment->startSpeed;
    segment->timeProfile.endSpeed = segment->endSpeed;
    segment->timeProfile.peakSpeed = segment->peakSpeed;
    segment->timeProfile.length = segment->length;
    segment->timeProfile.accelTimeSec = segment->accelTimeSec;
    segment->timeProfile.cruiseTimeSec = segment->cruiseTimeSec;
    segment->timeProfile.decelTimeSec = segment->decelTimeSec;
    segment->timeProfile.accelDistance = segment->accelDistance;
    segment->timeProfile.cruiseDistance = segment->cruiseDistance;

    segment->timeProfile.phaseDuration[0] = accelProfile.phaseDuration[0];
    segment->timeProfile.phaseDuration[1] = accelProfile.phaseDuration[1];
    segment->timeProfile.phaseDuration[2] = accelProfile.phaseDuration[2];
    segment->timeProfile.phaseDuration[3] = segment->cruiseTimeSec;
    segment->timeProfile.phaseDuration[4] = decelProfile.phaseDuration[0];
    segment->timeProfile.phaseDuration[5] = decelProfile.phaseDuration[1];
    segment->timeProfile.phaseDuration[6] = decelProfile.phaseDuration[2];
    for (uint8_t i = 0; i < 7; ++i) {
        if (segment->timeProfile.phaseDuration[i] < 0.0) segment->timeProfile.phaseDuration[i] = 0.0;
    }
}

inline void profileMotionTrajectoryEdge(MotionTrajectoryEdge* edge,
                                        const MotionSegment& pathSegment,
                                        double startSpeed,
                                        double endSpeed,
                                        double startTimeSec) {
    if (!edge) return;
    MotionSegment temp = pathSegment;
    temp.length = edge->length;
    temp.nominalSpeed = motionTrajectoryEdgeNominalSpeed(*edge, pathSegment);
    temp.acceleration = edge->acceleration;
    temp.jerk = edge->jerk;
    profileMotionSegment(&temp, startSpeed, endSpeed, startTimeSec);
    edge->startTimeSec = temp.startTimeSec;
    edge->durationSec = temp.durationSec;
    edge->startSpeed = temp.startSpeed;
    edge->endSpeed = temp.endSpeed;
    edge->peakSpeed = temp.peakSpeed;
    edge->acceleration = temp.acceleration;
    edge->jerk = temp.jerk;
}

inline bool finiteNonnegative(double value, double epsilon = 1.0e-9) {
    return isFinite(value) && value >= -epsilon;
}

inline double blendPlanTranslationLength(const MoveLBlendPlan& plan);

inline double finitePositiveOrZero(double value) {
    return isFinite(value) && value > 1.0e-9 ? value : 0.0;
}

inline bool jointArrayIsFinite(const double q[kAxisCount]) {
    if (!q) return false;
    for (uint8_t i = 0; i < kAxisCount; ++i) {
        if (!isFinite(q[i])) return false;
    }
    return true;
}

inline bool blendPlanIsFinite(const MoveLBlendPlan& plan) {
    if (!transformIsFinite(plan.cornerTcp) ||
        !transformIsFinite(plan.entryTcp) ||
        !transformIsFinite(plan.exitTcp)) return false;
    if (!isFinite(plan.radiusMm) || !isFinite(plan.sweepRad) ||
        !isFinite(plan.trimMm) || !isFinite(plan.maxDeviationMm) ||
        !isFinite(plan.actualDeviationMm) || !isFinite(plan.contourDeviationMm)) {
        return false;
    }
    for (uint8_t i = 0; i < 3; ++i) {
        if (!isFinite(plan.center[i]) ||
            !isFinite(plan.startUnit[i]) ||
            !isFinite(plan.planeUnit[i]) ||
            !isFinite(plan.entryPoint[i]) ||
            !isFinite(plan.exitPoint[i]) ||
            !isFinite(plan.incomingUnit[i]) ||
            !isFinite(plan.outgoingUnit[i])) return false;
    }
    if (!isFinite(plan.tangentScaleMm) || plan.tangentScaleMm < 0.0) return false;
    return true;
}

inline RejectCode validateMotionSegment(const MotionSegment& segment) {
    if (!jointArrayIsFinite(segment.startQ) || !jointArrayIsFinite(segment.endQ)) return RejectCode::BadTarget;
    if (!transformIsFinite(segment.startTcp) || !transformIsFinite(segment.endTcp)) return RejectCode::BadPose;
    if (!finiteNonnegative(segment.length) ||
        !finiteNonnegative(segment.nominalSpeed) ||
        !finiteNonnegative(segment.commandSpeed) ||
        !finiteNonnegative(segment.acceleration) ||
        !finiteNonnegative(segment.commandAcceleration) ||
        !finiteNonnegative(segment.jerk) ||
        !finiteNonnegative(segment.pathParamStart) ||
        !finiteNonnegative(segment.pathParamEnd) ||
        !finiteNonnegative(segment.globalPathStart) ||
        !finiteNonnegative(segment.globalPathEnd) ||
        !finiteNonnegative(segment.orientationLengthRad) ||
        !finiteNonnegative(segment.orientationRatioRadPerPathUnit) ||
        !finiteNonnegative(segment.startSpeed) ||
        !finiteNonnegative(segment.endSpeed) ||
        !finiteNonnegative(segment.peakSpeed)) {
        return RejectCode::BadSpeed;
    }
    if (segment.pathParamStart > segment.pathParamEnd + 1.0e-9 ||
        segment.pathParamEnd > 1.0 + 1.0e-9) {
        return RejectCode::BadPose;
    }
    if (segment.globalPathStart > segment.globalPathEnd + 1.0e-9) {
        return RejectCode::BadDuration;
    }
    if (!finiteNonnegative(segment.startTimeSec) ||
        !finiteNonnegative(segment.durationSec) ||
        !finiteNonnegative(segment.accelTimeSec) ||
        !finiteNonnegative(segment.cruiseTimeSec) ||
        !finiteNonnegative(segment.decelTimeSec) ||
        !finiteNonnegative(segment.accelDistance) ||
        !finiteNonnegative(segment.cruiseDistance)) {
        return RejectCode::BadDuration;
    }
    if (segment.nominalSpeed > 1.0e-9 &&
        (segment.startSpeed > segment.nominalSpeed + 1.0e-6 ||
         segment.endSpeed > segment.nominalSpeed + 1.0e-6 ||
         segment.peakSpeed > segment.nominalSpeed + 1.0e-6)) {
        return RejectCode::BadSpeed;
    }
    if (segment.kind == MotionSegmentKind::MoveLBlend) {
        if (!segment.blendPlan.enabled || !blendPlanIsFinite(segment.blendPlan)) return RejectCode::BadPose;
        if (segment.blendPlan.radiusMm <= 1.0e-9 || absDouble(segment.blendPlan.sweepRad) <= 1.0e-9) return RejectCode::BadPose;
        if (segment.blendPlan.maxDeviationMm > 0.0 &&
            segment.blendPlan.actualDeviationMm > segment.blendPlan.maxDeviationMm + 1.0e-6) {
            return RejectCode::BadPose;
        }
        const double fullArcLength = blendPlanTranslationLength(segment.blendPlan);
        const double arcFraction = segment.pathParamEnd > segment.pathParamStart
            ? segment.pathParamEnd - segment.pathParamStart
            : 1.0;
        const double arcLength = fullArcLength * arcFraction;
        const double tolerance = 1.0e-5 + arcLength * 1.0e-6;
        if (absDouble(segment.length - arcLength) > tolerance) return RejectCode::BadDuration;
    }
    const MotionTimeProfile& profile = segment.timeProfile;
    if (!finiteNonnegative(profile.startSpeed) ||
        !finiteNonnegative(profile.endSpeed) ||
        !finiteNonnegative(profile.peakSpeed) ||
        !finiteNonnegative(profile.length) ||
        !finiteNonnegative(profile.accelTimeSec) ||
        !finiteNonnegative(profile.cruiseTimeSec) ||
        !finiteNonnegative(profile.decelTimeSec) ||
        !finiteNonnegative(profile.accelDistance) ||
        !finiteNonnegative(profile.cruiseDistance)) {
        return RejectCode::BadDuration;
    }
    double phaseSum = 0.0;
    for (uint8_t i = 0; i < 7; ++i) {
        if (!finiteNonnegative(profile.phaseDuration[i])) return RejectCode::BadDuration;
        phaseSum += profile.phaseDuration[i];
    }
    if (profile.length > 1.0e-9 &&
        segment.durationSec > 1.0e-9 &&
        absDouble(phaseSum - segment.durationSec) > 1.0e-5 + segment.durationSec * 1.0e-6) {
        return RejectCode::BadDuration;
    }
    return RejectCode::Ok;
}

inline RejectCode validateMotionTrajectoryEdgePolynomial(const MotionTrajectoryEdge& edge) {
    if (edge.useQuintic == 0) return RejectCode::Ok;
    if (!isFinite(edge.startAcceleration) ||
        !isFinite(edge.endAcceleration) ||
        !isFinite(edge.startJerk) ||
        !isFinite(edge.endJerk) ||
        !isFinite(edge.quinticC3) ||
        !isFinite(edge.quinticC4) ||
        !isFinite(edge.quinticC5) ||
        !isFinite(edge.quinticC6) ||
        !isFinite(edge.quinticC7)) {
        return RejectCode::BadSpeed;
    }
    if (edge.durationSec <= 1.0e-9 || edge.length <= 1.0e-12) return RejectCode::Ok;

    const double t = edge.durationSec;
    const double t2 = t * t;
    const double t3 = t2 * t;
    const double t4 = t2 * t2;
    const double t5 = t4 * t;
    const double t6 = t5 * t;
    const double t7 = t6 * t;
    const double endDistance = edge.startSpeed * t +
        0.5 * edge.startAcceleration * t2 +
        edge.quinticC3 * t3 +
        edge.quinticC4 * t4 +
        edge.quinticC5 * t5 +
        edge.quinticC6 * t6 +
        edge.quinticC7 * t7;
    const double endSpeed = edge.startSpeed +
        edge.startAcceleration * t +
        3.0 * edge.quinticC3 * t2 +
        4.0 * edge.quinticC4 * t3 +
        5.0 * edge.quinticC5 * t4 +
        6.0 * edge.quinticC6 * t5 +
        7.0 * edge.quinticC7 * t6;
    const double startAccel = edge.startAcceleration;
    const double endAccel = edge.startAcceleration +
        6.0 * edge.quinticC3 * t +
        12.0 * edge.quinticC4 * t2 +
        20.0 * edge.quinticC5 * t3 +
        30.0 * edge.quinticC6 * t4 +
        42.0 * edge.quinticC7 * t5;
    const double startJerk = 6.0 * edge.quinticC3;
    const double endJerk = 6.0 * edge.quinticC3 +
        24.0 * edge.quinticC4 * t +
        60.0 * edge.quinticC5 * t2 +
        120.0 * edge.quinticC6 * t3 +
        210.0 * edge.quinticC7 * t4;
    const double distanceTolerance = 1.0e-5 + edge.length * 1.0e-6;
    double speedScale = edge.speedLimit > edge.peakSpeed ? edge.speedLimit : edge.peakSpeed;
    if (speedScale < edge.startSpeed) speedScale = edge.startSpeed;
    if (speedScale < edge.endSpeed) speedScale = edge.endSpeed;
    const double speedTolerance = 1.0e-5 + speedScale * 1.0e-5;
    double accelScale = edge.acceleration > absDouble(edge.startAcceleration)
        ? edge.acceleration
        : absDouble(edge.startAcceleration);
    if (accelScale < absDouble(edge.endAcceleration)) accelScale = absDouble(edge.endAcceleration);
    const double accelTolerance = 1.0e-4 + accelScale * 1.0e-4;
    double jerkScale = edge.jerk > absDouble(edge.startJerk) ? edge.jerk : absDouble(edge.startJerk);
    if (jerkScale < absDouble(edge.endJerk)) jerkScale = absDouble(edge.endJerk);
    const double jerkTolerance = 1.0e-3 + jerkScale * 1.0e-4;

    if (absDouble(endDistance - edge.length) > distanceTolerance) return RejectCode::BadDuration;
    if (absDouble(endSpeed - edge.endSpeed) > speedTolerance) return RejectCode::BadSpeed;
    if (absDouble(startAccel - edge.startAcceleration) > accelTolerance ||
        absDouble(endAccel - edge.endAcceleration) > accelTolerance) {
        return RejectCode::BadSpeed;
    }
    if (absDouble(startJerk - edge.startJerk) > jerkTolerance ||
        absDouble(endJerk - edge.endJerk) > jerkTolerance) {
        return RejectCode::BadSpeed;
    }
    return RejectCode::Ok;
}

inline RejectCode validateMotionSegmentProgram(const MotionSegmentProgram& program) {
    if (program.count > kMaxMotionSegments) return RejectCode::TooManyTicks;
    for (uint8_t i = 0; i < program.count; ++i) {
        const MotionSegment& segment = program.segments[i];
        const RejectCode segmentResult = validateMotionSegment(segment);
        if (segmentResult != RejectCode::Ok) return segmentResult;
        if (i > 0) {
            const MotionSegment& previous = program.segments[i - 1];
            for (uint8_t joint = 0; joint < kAxisCount; ++joint) {
                if (absDouble(wrapRadians(segment.startQ[joint] - previous.endQ[joint])) > 1.0e-4) {
                    return RejectCode::SegmentFailed;
                }
            }
        }
    }
    if (!finiteNonnegative(program.totalDurationSec)) return RejectCode::BadDuration;
    if (program.trajectory.count > kMaxMotionPathNodes ||
        program.trajectory.edgeCount + 1 != program.trajectory.count ||
        program.trajectory.count != program.pathGrid.count) {
        return RejectCode::BadDuration;
    }
    for (uint16_t i = 0; i < program.trajectory.count; ++i) {
        const MotionTrajectoryNode& node = program.trajectory.nodes[i];
        if (!finiteNonnegative(node.timeSec) ||
            !finiteNonnegative(node.pathS) ||
            !finiteNonnegative(node.speed) ||
            !isFinite(node.acceleration) ||
            !isFinite(node.jerk) ||
            node.pathNodeIndex >= program.pathGrid.count) {
            return RejectCode::BadDuration;
        }
        if (i > 0) {
            const MotionTrajectoryNode& previousNode = program.trajectory.nodes[i - 1];
            if (node.timeSec + 1.0e-9 < previousNode.timeSec ||
                node.pathS + 1.0e-9 < previousNode.pathS) {
                return RejectCode::BadDuration;
            }
        }
    }
    double expectedStartTime = 0.0;
    uint16_t polynomialEdgeCount = 0;
    for (uint16_t i = 0; i < program.trajectory.edgeCount; ++i) {
        const MotionTrajectoryEdge& edge = program.trajectory.edges[i];
        if (edge.pathSegmentIndex >= program.count ||
            edge.startNodeIndex >= program.pathGrid.count ||
            edge.endNodeIndex >= program.pathGrid.count ||
            edge.endNodeIndex != edge.startNodeIndex + 1 ||
            !finiteNonnegative(edge.startTimeSec) ||
            !finiteNonnegative(edge.durationSec) ||
            !finiteNonnegative(edge.startS) ||
            !finiteNonnegative(edge.endS) ||
            !finiteNonnegative(edge.length) ||
            !finiteNonnegative(edge.startSpeed) ||
            !finiteNonnegative(edge.endSpeed) ||
            !finiteNonnegative(edge.peakSpeed) ||
            !finiteNonnegative(edge.speedLimit) ||
            !finiteNonnegative(edge.acceleration) ||
            !finiteNonnegative(edge.jerk) ||
            !isFinite(edge.startAcceleration) ||
            !isFinite(edge.endAcceleration) ||
            !isFinite(edge.startJerk) ||
            !isFinite(edge.endJerk)) {
            return RejectCode::BadDuration;
        }
        if (edge.startNodeIndex != i || edge.endNodeIndex != i + 1) return RejectCode::BadDuration;
        const MotionTrajectoryNode& startNode = program.trajectory.nodes[edge.startNodeIndex];
        const MotionTrajectoryNode& endNode = program.trajectory.nodes[edge.endNodeIndex];
        if (absDouble(edge.startTimeSec - startNode.timeSec) > 1.0e-5 + startNode.timeSec * 1.0e-6 ||
            absDouble((edge.startTimeSec + edge.durationSec) - endNode.timeSec) > 1.0e-5 + endNode.timeSec * 1.0e-6 ||
            absDouble(edge.startS - startNode.pathS) > 1.0e-5 + edge.startS * 1.0e-6 ||
            absDouble(edge.endS - endNode.pathS) > 1.0e-5 + edge.endS * 1.0e-6 ||
            absDouble(edge.startSpeed - startNode.speed) > 1.0e-5 + edge.startSpeed * 1.0e-6 ||
            absDouble(edge.endSpeed - endNode.speed) > 1.0e-5 + edge.endSpeed * 1.0e-6) {
            return RejectCode::BadDuration;
        }
        if (absDouble(edge.startTimeSec - expectedStartTime) > 1.0e-5 + expectedStartTime * 1.0e-6) {
            return RejectCode::BadDuration;
        }
        if (edge.endS + 1.0e-9 < edge.startS) return RejectCode::BadDuration;
        if (edge.length > 1.0e-9 && edge.durationSec <= 1.0e-12) return RejectCode::BadDuration;
        if (edge.length > 1.0e-12 && edge.durationSec > 1.0e-9) {
            if (edge.useQuintic != 0) {
                ++polynomialEdgeCount;
            } else {
                return RejectCode::BadDuration;
            }
        }
        const RejectCode polynomialResult = validateMotionTrajectoryEdgePolynomial(edge);
        if (polynomialResult != RejectCode::Ok) return polynomialResult;
        expectedStartTime += edge.durationSec;
    }
    if (program.polynomialEdgeCount != polynomialEdgeCount) {
        return RejectCode::BadDuration;
    }
    if (absDouble(program.totalDurationSec - expectedStartTime) > 1.0e-5 + expectedStartTime * 1.0e-6) {
        return RejectCode::BadDuration;
    }
    return RejectCode::Ok;
}

inline RejectCode validateMotionProgramInput(const MotionProgram& program, const MotionProgramSettings& settings) {
    if (program.count > kMaxMotionCommands) return RejectCode::TooManyTicks;
    if (!isFinite(settings.defaultJointSpeedDegPerSec) ||
        !isFinite(settings.defaultLinearSpeedMmPerSec) ||
        !isFinite(settings.defaultJointAccelerationRadSec2) ||
        !isFinite(settings.defaultLinearAccelerationMmSec2) ||
        !isFinite(settings.defaultJointJerkRadSec3) ||
        !isFinite(settings.defaultLinearJerkMmSec3) ||
        !isFinite(settings.defaultToolAngularSpeedRadSec) ||
        !isFinite(settings.defaultToolAngularAccelerationRadSec2) ||
        !isFinite(settings.defaultToolAngularJerkRadSec3) ||
        !isFinite(settings.sampleMm) ||
        !isFinite(settings.jointSampleDeg) ||
        !isFinite(settings.controlPeriodSec) ||
        !isFinite(settings.singularityThresholdRad) ||
        !isFinite(program.initialPathSpeed)) {
        return RejectCode::BadSpeed;
    }
    if (settings.defaultJointSpeedDegPerSec < 0.0 ||
        settings.defaultLinearSpeedMmPerSec < 0.0 ||
        settings.defaultJointAccelerationRadSec2 < 0.0 ||
        settings.defaultLinearAccelerationMmSec2 < 0.0 ||
        settings.defaultJointJerkRadSec3 < 0.0 ||
        settings.defaultLinearJerkMmSec3 < 0.0 ||
        settings.defaultToolAngularSpeedRadSec < 0.0 ||
        settings.defaultToolAngularAccelerationRadSec2 < 0.0 ||
        settings.defaultToolAngularJerkRadSec3 < 0.0 ||
        settings.sampleMm < 0.0 ||
        settings.jointSampleDeg < 0.0 ||
        settings.controlPeriodSec < 0.0 ||
        settings.singularityThresholdRad < 0.0 ||
        program.initialPathSpeed < 0.0) {
        return RejectCode::BadSpeed;
    }
    for (uint8_t joint = 0; joint < kAxisCount; ++joint) {
        if (!isFinite(settings.jointVelocityLimitRadSec[joint]) ||
            !isFinite(settings.jointAccelerationLimitRadSec2[joint]) ||
            !isFinite(settings.jointStepsPerDegree[joint]) ||
            !isFinite(settings.jointStepRateLimitStepsSec[joint])) {
            return RejectCode::BadSpeed;
        }
        if (settings.jointVelocityLimitRadSec[joint] < 0.0 ||
            settings.jointAccelerationLimitRadSec2[joint] < 0.0 ||
            settings.jointStepsPerDegree[joint] < 0.0 ||
            settings.jointStepRateLimitStepsSec[joint] < 0.0) {
            return RejectCode::BadSpeed;
        }
    }
    for (uint8_t i = 0; i < program.count; ++i) {
        const MotionCommand& command = program.commands[i];
        if (!jointArrayIsFinite(command.targetQ)) return RejectCode::BadTarget;
        if (!isFinite(command.jointSpeedDegPerSec) ||
            !isFinite(command.linearSpeedMmPerSec) ||
            !isFinite(command.blendMm) ||
            !isFinite(command.sampleMm) ||
            !isFinite(command.coordinatedPathLength)) {
            return RejectCode::BadSpeed;
        }
        if (command.jointSpeedDegPerSec < 0.0 ||
            command.linearSpeedMmPerSec < 0.0 ||
            command.blendMm < -1.0 ||
            command.sampleMm < 0.0 ||
            command.coordinatedPathLength < 0.0) {
            return RejectCode::BadSpeed;
        }
        if (command.type == MotionCommandType::MoveL && !transformIsFinite(command.targetTcp)) {
            return RejectCode::BadPose;
        }
        // A weave the program switched on but whose numbers do not describe a pattern is a mistake
        // to report, not something to quietly not do. The commonest case is a schedule saved with
        // no rate mode chosen, which would otherwise weld a straight bead and look like the weave
        // silently failed. A weave left off, or on with zero amplitude, is a no-op and allowed.
        if (weaveIsActive(command.weave) && !weaveParamsAreUsable(command.weave)) {
            if (weaveAmplitudePeakMm(command.weave) > 1.0e-9 ||
                command.weave.rateMode == WeaveRateMode::Unset) {
                return RejectCode::WeaveNotConfigured;
            }
        }
        if (command.type == MotionCommandType::MoveJ && weaveParamsAreUsable(command.weave)) {
            return RejectCode::WeaveOnJointMove;
        }
        if (motionTriggerIsActive(command.trigger)) {
            if (!isFinite(command.trigger.distanceMm) || !isFinite(command.trigger.timeMs)) {
                return RejectCode::BadTarget;
            }
            if (command.type == MotionCommandType::MoveJ &&
                absDouble(command.trigger.distanceMm) > 1.0e-9) {
                return RejectCode::BadTarget;
            }
        }
    }
    return RejectCode::Ok;
}

inline bool appendMotionSegment(MotionSegmentProgram* out, const MotionSegment& segment) {
    if (!out || out->count >= kMaxMotionSegments) return false;
    out->segments[out->count++] = segment;
    return true;
}

inline bool appendBaseMotionSegment(MotionBaseSegmentProgram* out, const MotionSegment& segment) {
    if (!out || out->count >= kMaxBaseMotionSegments) return false;
    out->segments[out->count++] = segment;
    return true;
}

inline void initializeMotionPathNode(MotionPathNode* node) {
    if (!node) return;
    memset(node, 0, sizeof(*node));
    node->commandId = -1;
    node->completedCommandId = -1;
    node->segmentIndex = 0;
    node->localT = 0.0;
    node->speedLimit = 0.0;
    node->solvedSpeed = 0.0;
    node->accelLimit = 1.0;
    node->jerkLimit = 1000.0;
    node->singularityMarginRad = 1.0e9;
}

inline void assignMotionPathNodePose(MotionPathNode* node,
                                     const Transform& tcp,
                                     const double q[kAxisCount]) {
    if (!node || !q) return;
    node->tcp = tcp;
    for (uint8_t i = 0; i < kAxisCount; ++i) node->q[i] = q[i];
}

inline bool anyJointLimitConfigured(const double values[kAxisCount]) {
    for (uint8_t i = 0; i < kAxisCount; ++i) {
        if (finitePositiveOrZero(values[i]) > 0.0) return true;
    }
    return false;
}

inline double blendPlanTranslationLength(const MoveLBlendPlan& plan) {
    if (isFinite(plan.translationLengthMm) && plan.translationLengthMm > 1.0e-9) {
        return plan.translationLengthMm;
    }
    if (plan.useQuintic) {
        const double deviation = estimateQuinticCornerDeviation(plan);
        if (!isFinite(deviation) ||
            (plan.maxDeviationMm > 0.0 && deviation > plan.maxDeviationMm + 1.0e-6)) {
            return 0.0;
        }
        const double length = estimateQuinticCornerLength(plan);
        if (isFinite(length) && length > 1.0e-9) return length;
    }
    return absDouble(plan.radiusMm * plan.sweepRad);
}

inline double blendPlanTranslationLengthToT(const MoveLBlendPlan& plan, double t) {
    const double clamped = clamp01(t);
    if (clamped <= 0.0) return 0.0;
    const double totalLength = blendPlanTranslationLength(plan);
    if (!plan.useQuintic) return totalLength * clamped;
    if (totalLength > 1.0e-9 && plan.arcLengthSamples[8] > 1.0e-9) {
        const double scaled = clamped * 8.0;
        uint8_t index = static_cast<uint8_t>(scaled);
        if (index >= 8) return plan.arcLengthSamples[8];
        const double local = scaled - static_cast<double>(index);
        const double a = plan.arcLengthSamples[index];
        const double b = plan.arcLengthSamples[index + 1];
        return a + (b - a) * local;
    }

    constexpr uint8_t kSamples = 24;
    double previous[3] = {};
    double current[3] = {};
    sampleQuinticCornerPoint(plan, 0.0, previous);
    double length = 0.0;
    for (uint8_t i = 1; i <= kSamples; ++i) {
        const double sampleT = clamped * (static_cast<double>(i) / static_cast<double>(kSamples));
        sampleQuinticCornerPoint(plan, sampleT, current);
        double delta[3] = {};
        subtract3(current, previous, delta);
        length += length3(delta);
        for (uint8_t axis = 0; axis < 3; ++axis) previous[axis] = current[axis];
    }
    return length;
}

inline double blendPlanParameterAtTranslationDistance(const MoveLBlendPlan& plan, double distanceMm) {
    const double totalLength = blendPlanTranslationLength(plan);
    if (!isFinite(distanceMm) || distanceMm <= 0.0 || totalLength <= 1.0e-9) return 0.0;
    if (distanceMm >= totalLength) return 1.0;
    if (!plan.useQuintic) return clamp01(distanceMm / totalLength);
    if (plan.arcLengthSamples[8] > 1.0e-9) {
        constexpr uint8_t kTableSamples = 8;
        uint8_t cell = 0;
        while (cell + 1 < kTableSamples && plan.arcLengthSamples[cell + 1] < distanceMm) {
            ++cell;
        }
        const double a = plan.arcLengthSamples[cell];
        const double b = plan.arcLengthSamples[cell + 1];
        const double span = b - a;
        const double local = span > 1.0e-12 ? (distanceMm - a) / span : 0.0;
        return clamp01((static_cast<double>(cell) + clamp01(local)) / static_cast<double>(kTableSamples));
    }

    double lo = 0.0;
    double hi = 1.0;
    for (uint8_t iteration = 0; iteration < 18; ++iteration) {
        const double mid = 0.5 * (lo + hi);
        const double length = blendPlanTranslationLengthToT(plan, mid);
        if (length < distanceMm) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    return 0.5 * (lo + hi);
}

inline Transform sampleMotionSegmentPoseOnly(const MotionSegment& segment, double t) {
    const double local = clamp01(t);
    const double pathT = segment.pathParamEnd > segment.pathParamStart
        ? segment.pathParamStart + (segment.pathParamEnd - segment.pathParamStart) * local
        : local;
    if (segment.kind == MotionSegmentKind::MoveLBlend) {
        return sampleBlendArcPose(segment.blendPlan, pathT);
    }
    return interpolatePoseLinear(segment.startTcp, segment.endTcp, local);
}

inline double weaveCycleExtent(const WeaveParams& params) {
    double base = 0.0;
    if (params.rateMode == WeaveRateMode::Frequency) {
        if (!isFinite(params.frequencyHz) || params.frequencyHz <= 0.0) return 0.0;
        base = 1.0 / params.frequencyHz;
    } else if (params.rateMode == WeaveRateMode::Wavelength) {
        if (!isFinite(params.wavelengthMm) || params.wavelengthMm <= 1.0e-6) return 0.0;
        base = params.wavelengthMm;
    } else {
        return 0.0;
    }
    const double left = isFinite(params.dwellLeft) && params.dwellLeft > 0.0 ? params.dwellLeft : 0.0;
    const double right = isFinite(params.dwellRight) && params.dwellRight > 0.0 ? params.dwellRight : 0.0;
    const double center = isFinite(params.dwellCenter) && params.dwellCenter > 0.0 ? params.dwellCenter : 0.0;
    return base + left + right + 2.0 * center;
}

// Phase, in cycles, advanced by an elapsed time and an elapsed path distance. Only the one the rate
// mode cares about is read, so callers can hand over both and stay out of the distinction.
inline double weavePhaseAdvance(const WeaveParams& params, double deltaTimeSec, double deltaDistanceMm) {
    const double extent = weaveCycleExtent(params);
    if (extent <= 1.0e-12) return 0.0;
    const double delta = params.rateMode == WeaveRateMode::Frequency ? deltaTimeSec : deltaDistanceMm;
    if (!isFinite(delta) || delta <= 0.0) return 0.0;
    return delta / extent;
}

// Maps a phase in cycles onto the pattern parameter u in [0,1), where u = 0 is the centre moving
// toward the left, 0.25 the left extreme, 0.5 the centre again and 0.75 the right extreme. Dwell is
// applied here, by freezing u while the cycle's cursor crosses the dwell's share of the extent, so
// every shape below inherits dwell without knowing about it.
inline double weaveShapeParamAtPhase(const WeaveParams& params, double phaseCycles) {
    const double extent = weaveCycleExtent(params);
    if (extent <= 1.0e-12) return 0.0;
    double fraction = phaseCycles - floor(phaseCycles);
    if (!isFinite(fraction) || fraction < 0.0) fraction = 0.0;

    double base = params.rateMode == WeaveRateMode::Frequency ? 1.0 / params.frequencyHz : params.wavelengthMm;
    const double quarter = base * 0.25;
    const double left = isFinite(params.dwellLeft) && params.dwellLeft > 0.0 ? params.dwellLeft : 0.0;
    const double right = isFinite(params.dwellRight) && params.dwellRight > 0.0 ? params.dwellRight : 0.0;
    const double center = isFinite(params.dwellCenter) && params.dwellCenter > 0.0 ? params.dwellCenter : 0.0;

    // Alternating travel and dwell around the cycle. A zero-length dwell simply contributes nothing.
    const double spans[8] = {quarter, left, quarter, center, quarter, right, quarter, center};
    const double startU[8] = {0.0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1.0};
    const double endU[8] = {0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1.0, 1.0};

    double cursor = fraction * extent;
    for (uint8_t i = 0; i < 8; ++i) {
        if (spans[i] <= 1.0e-12) continue;
        if (cursor <= spans[i]) {
            const double local = cursor / spans[i];
            return startU[i] + (endU[i] - startU[i]) * local;
        }
        cursor -= spans[i];
    }
    return 0.0;
}

// A triangle wave built from its first three harmonics instead of straight ramps.
inline double weaveSmoothTriangle(double u) {
    const double kTwoPi = 2.0 * 3.14159265358979323846;
    const double sum = sin(kTwoPi * u) - sin(3.0 * kTwoPi * u) / 9.0 + sin(5.0 * kTwoPi * u) / 25.0;
    // Normalised so the peak at u = 0.25 is exactly the requested amplitude.
    return sum * 0.868727;
}

// Lateral and out-of-plane excursion in millimetres for a pattern parameter. Lateral is signed:
// positive is the left side, so the two amplitudes can differ.
inline void weaveShapeOffset(const WeaveParams& params, double u, double out[2]) {
    out[0] = 0.0;
    out[1] = 0.0;
    const double kTwoPi = 2.0 * 3.14159265358979323846;
    const double left = isFinite(params.amplitudeLeftMm) && params.amplitudeLeftMm > 0.0 ? params.amplitudeLeftMm : 0.0;
    const double right = isFinite(params.amplitudeRightMm) && params.amplitudeRightMm > 0.0 ? params.amplitudeRightMm : 0.0;
    const double elevation = isFinite(params.elevationMm) && params.elevationMm > 0.0 ? params.elevationMm : 0.0;

    const double triangle = weaveSmoothTriangle(u);
    const double sine = sin(kTwoPi * u);

    // Unsymmetrical amplitudes, without a corner where the pattern crosses the centre.
    const double amplitudeMean = 0.5 * (left + right);
    const double amplitudeHalfDifference = 0.5 * (left - right);
    const auto scaleBySide = [&](double unit) {
        return amplitudeMean * unit + amplitudeHalfDifference * unit * unit;
    };

    switch (params.shape) {
    case WeaveShape::None:
        break;
    case WeaveShape::Sine:
        out[0] = scaleBySide(sine);
        break;
    case WeaveShape::Zigzag:
        out[0] = scaleBySide(triangle);
        break;
    case WeaveShape::Trapezoid: {
        // A steepened triangle pushed through a smooth saturation, which flattens the extremes
        // without needing a second parameter for the flat's width and without the corners a hard
        // clip would put back. Distinct from dwell: this flattens within the pattern, where dwell
        // holds the extreme while the torch keeps travelling.
        constexpr double kSteepness = 1.8;
        constexpr double kNormalise = 1.0 / 0.946806;   // tanh(kSteepness)
        out[0] = scaleBySide(tanh(kSteepness * triangle) * kNormalise);
        break;
    }
    case WeaveShape::Circular:
        // Lateral and out-of-plane a quarter cycle apart. Combined with the travel along the seam
        // this is the helix that FANUC calls a circle weave and KUKA calls a spiral.
        out[0] = scaleBySide(sine);
        out[1] = elevation * cos(kTwoPi * u);
        break;
    case WeaveShape::FigureEight:
        out[0] = scaleBySide(sine);
        out[1] = elevation * sin(2.0 * kTwoPi * u);
        break;
    case WeaveShape::LShape: {
        // An approximation of the vendors' L pattern for fillets: one leg of the cycle works the
        // lateral direction, the other the out-of-plane direction.
        const double leg = u < 0.5 ? (u / 0.5) : ((u - 0.5) / 0.5);
        const double raised = 0.5 * (1.0 - cos(kTwoPi * leg));
        const double rise = raised * raised;
        if (u < 0.5) {
            out[0] = scaleBySide(rise);
        } else {
            out[1] = elevation * rise;
        }
        break;
    }
    }

    if (isFinite(params.biasMm)) out[0] += params.biasMm;
}

inline void weaveOffsetAtPhase(const WeaveParams& params, double phaseCycles, double out[2]) {
    out[0] = 0.0;
    out[1] = 0.0;
    if (!weaveParamsAreUsable(params)) return;
    weaveShapeOffset(params, weaveShapeParamAtPhase(params, phaseCycles), out);
}

// Direction of travel at a point on a segment. Lines are exact from their endpoints; blends are
// differenced off the pose sampler so the circular and quintic corner geometries are both covered
// by one path, and so the result stays continuous with the lines either side of the corner.
inline bool weaveTangentForSegment(const MotionSegment& segment, double localT, double out[3]) {
    if (!out) return false;
    if (segment.kind == MotionSegmentKind::MoveLLine) {
        double start[3] = {};
        double end[3] = {};
        translationVector(segment.startTcp, start);
        translationVector(segment.endTcp, end);
        subtract3(end, start, out);
        return normalize3(out);
    }
    if (segment.kind != MotionSegmentKind::MoveLBlend) return false;
    constexpr double kDelta = 1.0e-4;
    double before = localT - kDelta;
    double after = localT + kDelta;
    if (before < 0.0) before = 0.0;
    if (after > 1.0) after = 1.0;
    if (after - before < 1.0e-9) return false;
    double a[3] = {};
    double b[3] = {};
    translationVector(sampleMotionSegmentPoseOnly(segment, before), a);
    translationVector(sampleMotionSegmentPoseOnly(segment, after), b);
    subtract3(b, a, out);
    return normalize3(out);
}

// The plane the weave oscillates in. Tool Z is the reference because it is the only frame this
// planner has: there is no work object or user frame anywhere in the model, only the tool bind pose.
inline bool weaveFrameVectors(const Transform& tcp,
                              const double tangent[3],
                              double planeAngleDeg,
                              double outLateral[3],
                              double outNormal[3]) {
    if (!tangent || !outLateral || !outNormal) return false;
    const double toolZ[3] = {tcp.values[2], tcp.values[6], tcp.values[10]};
    const double toolX[3] = {tcp.values[0], tcp.values[4], tcp.values[8]};

    double lateral[3] = {};
    cross3(tangent, toolZ, lateral);
    if (!normalize3(lateral)) {
        cross3(tangent, toolX, lateral);
        if (!normalize3(lateral)) return false;
    }
    double normal[3] = {};
    cross3(tangent, lateral, normal);
    if (!normalize3(normal)) return false;

    if (isFinite(planeAngleDeg) && absDouble(planeAngleDeg) > 1.0e-9) {
        const double angle = degreesToRadians(planeAngleDeg);
        const double c = cos(angle);
        const double s = sin(angle);
        for (uint8_t i = 0; i < 3; ++i) {
            const double l = lateral[i];
            const double n = normal[i];
            outLateral[i] = l * c + n * s;
            outNormal[i] = -l * s + n * c;
        }
        return true;
    }
    for (uint8_t i = 0; i < 3; ++i) {
        outLateral[i] = lateral[i];
        outNormal[i] = normal[i];
    }
    return true;
}

// Stands in for "this run has no end inside this window", so nothing ramps out at a lookahead
// boundary the pattern is going to carry straight through.
constexpr double kWeaveRunOpenEnded = 1.0e30;

// Steepest slope of the quintic below, at the middle of a ramp. The taper is itself a movement across
// the seam - d(gain * offset) carries a gain' * offset term - so a ramping run briefly asks for more
// transverse speed than the pattern alone, and anything bounding that speed has to allow for it.
constexpr double kWeaveRampPeakSlope = 1.875;

// How much of the pattern is in effect, over the cycle at each end of a weaving run.
inline double weaveRampGain(const WeaveParams& params, double phaseCycles, double remaining) {
    const double extent = weaveCycleExtent(params);
    if (extent <= 1.0e-12) return 0.0;
    const auto smooth = [](double u) {
        if (!(u > 0.0)) return 0.0;
        if (u >= 1.0) return 1.0;
        return u * u * u * (u * (u * 6.0 - 15.0) + 10.0);
    };
    // One cycle to come in. phaseCycles counts from where the run began and keeps counting across
    // lookahead boundaries, so a run that spans several windows ramps once, at its true start.
    double gain = smooth(phaseCycles);
    if (isFinite(remaining) && remaining < kWeaveRunOpenEnded) {
        gain *= smooth(remaining / extent);
    }
    return gain;
}

// Displaces a pose without rotating it: the torch keeps the orientation the path gave it and only
// its position moves, which is what every vendor's weave does and what keeps the work angle honest.
inline Transform weaveDisplacePose(const Transform& pose,
                                   const double lateral[3],
                                   const double normal[3],
                                   const double offsetMm[2]) {
    Transform out = pose;
    out.values[3] += lateral[0] * offsetMm[0] + normal[0] * offsetMm[1];
    out.values[7] += lateral[1] * offsetMm[0] + normal[1] * offsetMm[1];
    out.values[11] += lateral[2] * offsetMm[0] + normal[2] * offsetMm[1];
    return out;
}

// Everything above, for one sample. Returns false when the weave is off or the frame collapsed, in
// which case the pose is handed back untouched and the offsets read zero.
inline bool weaveApplyToPose(const MotionSegment& segment,
                             const WeaveParams& params,
                             double localT,
                             double phaseCycles,
                             double remainingToRunEnd,
                             Transform* pose,
                             double outOffsetMm[2]) {
    if (outOffsetMm) {
        outOffsetMm[0] = 0.0;
        outOffsetMm[1] = 0.0;
    }
    if (!pose || !weaveParamsAreUsable(params)) return false;
    if (segment.kind == MotionSegmentKind::MoveJ) return false;

    double offset[2] = {};
    weaveOffsetAtPhase(params, phaseCycles, offset);
    const double gain = weaveRampGain(params, phaseCycles, remainingToRunEnd);
    offset[0] *= gain;
    offset[1] *= gain;
    if (absDouble(offset[0]) <= 1.0e-12 && absDouble(offset[1]) <= 1.0e-12) return false;

    double tangent[3] = {};
    if (!weaveTangentForSegment(segment, localT, tangent)) return false;
    double lateral[3] = {};
    double normal[3] = {};
    if (!weaveFrameVectors(*pose, tangent, params.planeAngleDeg, lateral, normal)) return false;

    *pose = weaveDisplacePose(*pose, lateral, normal, offset);
    if (outOffsetMm) {
        outOffsetMm[0] = offset[0];
        outOffsetMm[1] = offset[1];
    }
    return true;
}

inline bool locateMotionPathSegmentByS(const MotionSegmentProgram* program,
                                       double pathS,
                                       uint16_t* outIndex,
                                       double* outT) {
    if (!program || !outIndex || !outT || program->count == 0) return false;
    if (pathS <= 0.0) {
        *outIndex = 0;
        *outT = 0.0;
        return true;
    }
    for (uint16_t i = 0; i < program->count; ++i) {
        const MotionSegment& segment = program->segments[i];
        if (pathS <= segment.globalPathEnd + 1.0e-9 || i + 1 == program->count) {
            *outIndex = i;
            if (segment.length > 1.0e-12) {
                const double localDistance = pathS - segment.globalPathStart;
                *outT = segment.kind == MotionSegmentKind::MoveLBlend
                    ? blendPlanParameterAtTranslationDistance(segment.blendPlan, localDistance)
                    : clamp01(localDistance / segment.length);
            } else {
                *outT = 1.0;
            }
            return true;
        }
    }
    *outIndex = static_cast<uint16_t>(program->count - 1);
    *outT = 1.0;
    return true;
}

inline double motionPathGridJointTangent(const MotionPathGrid& grid, uint16_t node, uint8_t joint) {
    if (grid.count < 2 || joint >= kAxisCount) return 0.0;
    if (grid.nodes[node].finePoint) return 0.0;
    bool havePrev = false;
    bool haveNext = false;
    double prevSlope = 0.0;
    double nextSlope = 0.0;
    if (node > 0 && !grid.nodes[node - 1].finePoint) {
        const double ds = grid.nodes[node].s - grid.nodes[node - 1].s;
        if (ds > 1.0e-12) {
            prevSlope = wrapRadians(grid.nodes[node].q[joint] - grid.nodes[node - 1].q[joint]) / ds;
            havePrev = true;
        }
    }
    if (node + 1 < grid.count && !grid.nodes[node + 1].finePoint) {
        const double ds = grid.nodes[node + 1].s - grid.nodes[node].s;
        if (ds > 1.0e-12) {
            nextSlope = wrapRadians(grid.nodes[node + 1].q[joint] - grid.nodes[node].q[joint]) / ds;
            haveNext = true;
        }
    }
    if (havePrev && haveNext) {
        if (prevSlope * nextSlope <= 0.0) return 0.0;
        double tangent = 0.5 * (prevSlope + nextSlope);
        const double limit = 3.0 * (absDouble(prevSlope) < absDouble(nextSlope) ? absDouble(prevSlope) : absDouble(nextSlope));
        if (absDouble(tangent) > limit) tangent = tangent < 0.0 ? -limit : limit;
        return tangent;
    }
    if (havePrev) return prevSlope;
    if (haveNext) return nextSlope;
    return 0.0;
}

inline bool interpolateMotionPathGridJointsOnEdge(const MotionPathGrid& grid,
                                                  uint16_t edge,
                                                  double pathS,
                                                  double q[kAxisCount]) {
    if (!q || grid.count == 0 || edge + 1 >= grid.count) return false;
    const MotionPathNode& a = grid.nodes[edge];
    const MotionPathNode& b = grid.nodes[edge + 1];
    const double ds = b.s - a.s;
    const double t = ds > 1.0e-12 ? clamp01((pathS - a.s) / ds) : 1.0;
    const double t2 = t * t;
    const double t3 = t2 * t;
    const double t4 = t3 * t;
    const double t5 = t4 * t;
    for (uint8_t joint = 0; joint < kAxisCount; ++joint) {
        const double q0 = a.q[joint];
        const double q1 = q0 + wrapRadians(b.q[joint] - a.q[joint]);
        const double m0 = motionPathGridJointTangent(grid, edge, joint);
        const double m1 = motionPathGridJointTangent(grid, static_cast<uint16_t>(edge + 1), joint);
        const double c0 = q0;
        const double c1 = ds * m0;
        const double c2 = 0.0;
        const double d0 = q1 - (c0 + c1 + c2);
        const double d1 = ds * m1 - (c1 + 2.0 * c2);
        const double d2 = -2.0 * c2;
        const double c3 = 10.0 * d0 - 4.0 * d1 + 0.5 * d2;
        const double c4 = -15.0 * d0 + 7.0 * d1 - d2;
        const double c5 = 6.0 * d0 - 3.0 * d1 + 0.5 * d2;
        q[joint] = c0 + c1 * t + c2 * t2 + c3 * t3 + c4 * t4 + c5 * t5;
    }
    return true;
}

inline bool interpolateMotionPathGridJointsAtS(const MotionSegmentProgram* program,
                                               double pathS,
                                               double q[kAxisCount]) {
    if (!program || !q || program->pathGrid.count == 0) return false;
    const MotionPathGrid& grid = program->pathGrid;
    if (pathS <= grid.nodes[0].s || grid.count == 1) {
        for (uint8_t joint = 0; joint < kAxisCount; ++joint) q[joint] = grid.nodes[0].q[joint];
        return true;
    }
    for (uint16_t edge = 0; edge + 1 < grid.count; ++edge) {
        const MotionPathNode& b = grid.nodes[edge + 1];
        if (pathS <= b.s + 1.0e-9 || edge + 2 == grid.count) {
            return interpolateMotionPathGridJointsOnEdge(grid, edge, pathS, q);
        }
    }
    const MotionPathNode& last = grid.nodes[grid.count - 1];
    for (uint8_t joint = 0; joint < kAxisCount; ++joint) q[joint] = last.q[joint];
    return true;
}

inline bool motionPathGridJointDerivativeBoundsOnEdge(const MotionPathGrid& grid,
                                                      uint16_t edge,
                                                      uint8_t joint,
                                                      double* outMaxDqDs,
                                                      double* outMaxD2qDs2,
                                                      double* outMaxD3qDs3) {
    if (outMaxDqDs) *outMaxDqDs = 0.0;
    if (outMaxD2qDs2) *outMaxD2qDs2 = 0.0;
    if (outMaxD3qDs3) *outMaxD3qDs3 = 0.0;
    if (joint >= kAxisCount || grid.count == 0 || edge + 1 >= grid.count) return false;
    const MotionPathNode& a = grid.nodes[edge];
    const MotionPathNode& b = grid.nodes[edge + 1];
    const double ds = b.s - a.s;
    if (ds <= 1.0e-12) return false;
    const double q0 = a.q[joint];
    const double q1 = q0 + wrapRadians(b.q[joint] - a.q[joint]);
    const double m0 = motionPathGridJointTangent(grid, edge, joint);
    const double m1 = motionPathGridJointTangent(grid, static_cast<uint16_t>(edge + 1), joint);
    const double c0 = q0;
    const double c1 = ds * m0;
    const double c2 = 0.0;
    const double d0 = q1 - (c0 + c1 + c2);
    const double d1 = ds * m1 - (c1 + 2.0 * c2);
    const double d2 = -2.0 * c2;
    const double c3 = 10.0 * d0 - 4.0 * d1 + 0.5 * d2;
    const double c4 = -15.0 * d0 + 7.0 * d1 - d2;
    const double c5 = 6.0 * d0 - 3.0 * d1 + 0.5 * d2;
    const double invDs = 1.0 / ds;
    const double invDs2 = invDs * invDs;
    const double invDs3 = invDs2 * invDs;
    double maxDqDs = 0.0;
    double maxD2qDs2 = 0.0;
    double maxD3qDs3 = 0.0;
    for (uint8_t sample = 0; sample <= 16; ++sample) {
        const double t = static_cast<double>(sample) / 16.0;
        const double t2 = t * t;
        const double t3 = t2 * t;
        const double t4 = t2 * t2;
        const double dqDt = c1 +
            3.0 * c3 * t2 +
            4.0 * c4 * t3 +
            5.0 * c5 * t4;
        const double d2qDt2 =
            6.0 * c3 * t +
            12.0 * c4 * t2 +
            20.0 * c5 * t3;
        const double d3qDt3 =
            6.0 * c3 +
            24.0 * c4 * t +
            60.0 * c5 * t2;
        const double dqDs = absDouble(dqDt * invDs);
        const double d2qDs2 = absDouble(d2qDt2 * invDs2);
        const double d3qDs3 = absDouble(d3qDt3 * invDs3);
        if (dqDs > maxDqDs) maxDqDs = dqDs;
        if (d2qDs2 > maxD2qDs2) maxD2qDs2 = d2qDs2;
        if (d3qDs3 > maxD3qDs3) maxD3qDs3 = d3qDs3;
    }
    if (outMaxDqDs) *outMaxDqDs = maxDqDs;
    if (outMaxD2qDs2) *outMaxD2qDs2 = maxD2qDs2;
    if (outMaxD3qDs3) *outMaxD3qDs3 = maxD3qDs3;
    return true;
}

inline RejectCode sampleMotionPathAtS(const RobotModel& model,
                                      const MotionSegmentProgram* program,
                                      const double previousQ[kAxisCount],
                                      double pathS,
                                      PathSample* sample,
                                      uint16_t* outSegmentIndex = nullptr,
                                      double* outLocalT = nullptr) {
    if (!program || !previousQ || !sample) return RejectCode::BadTarget;
    uint16_t index = 0;
    double t = 0.0;
    if (!locateMotionPathSegmentByS(program, pathS, &index, &t)) return RejectCode::BadTarget;
    const MotionSegment& segment = program->segments[index];
    sample->pathSegmentIndex = index;
    sample->commandId = segment.commandId;
    sample->curvature = segment.curvature;
    sample->capReasonMask = segment.capReasonMask;
    sample->blended = segment.blended;
    sample->kind = segment.kind == MotionSegmentKind::MoveJ
        ? PathSampleKind::MoveJ
        : (segment.kind == MotionSegmentKind::MoveLBlend ? PathSampleKind::MoveLBlend : PathSampleKind::MoveLLine);

    if (segment.kind == MotionSegmentKind::MoveJ) {
        for (uint8_t joint = 0; joint < kAxisCount; ++joint) {
            sample->q[joint] = segment.startQ[joint] + (segment.endQ[joint] - segment.startQ[joint]) * t;
            sample->baseQ[joint] = sample->q[joint];
        }
        sample->tcp = toolPoseForJoints(model, sample->q);
        sample->baseTcp = sample->tcp;
    } else {
        // The grid joints are the taught path's own solution at this arc length, which is exactly the
        // unwoven state baseQ is for. They were already being computed and then discarded by the
        // solve below.
        const bool havePlannedQ = interpolateMotionPathGridJointsAtS(program, pathS, sample->q);
        if (havePlannedQ) {
            for (uint8_t joint = 0; joint < kAxisCount; ++joint) sample->baseQ[joint] = sample->q[joint];
        }
        sample->tcp = sampleMotionSegmentPoseOnly(segment, t);
        // Kept before the weave gets to displace it: this is the taught path's own pose here.
        sample->baseTcp = sample->tcp;
        bool weaveApplied = false;
        if (segment.commandIndex < kMaxMotionCommands) {
            const WeaveParams& weave = program->commandWeave[segment.commandIndex];
            if (weaveParamsAreUsable(weave)) {
                weaveApplied = true;
                const double phase = program->weaveAnchorPhase[segment.commandIndex] +
                    weavePhaseAdvance(weave,
                                      sample->timeSec - program->weaveAnchorTimeSec[segment.commandIndex],
                                      pathS - program->weaveAnchorS[segment.commandIndex]);
                sample->weavePhase = phase;
                const double runEnd = weave.rateMode == WeaveRateMode::Frequency
                    ? program->weaveRunEndTimeSec[segment.commandIndex]
                    : program->weaveRunEndS[segment.commandIndex];
                const double here = weave.rateMode == WeaveRateMode::Frequency ? sample->timeSec : pathS;
                const double remaining = runEnd >= kWeaveRunOpenEnded ? kWeaveRunOpenEnded : runEnd - here;
                weaveApplyToPose(segment, weave, t, phase, remaining, &sample->tcp, sample->weaveOffsetMm);
            }
        }
        if (!solveToolPoseNearest(model, previousQ, sample->tcp, sample->q)) {
            return RejectCode::NoIkSolution;
        }
        if (!weaveApplied || !havePlannedQ) {
            for (uint8_t joint = 0; joint < kAxisCount; ++joint) sample->baseQ[joint] = sample->q[joint];
        }
    }
    if (outSegmentIndex) *outSegmentIndex = index;
    if (outLocalT) *outLocalT = t;
    return RejectCode::Ok;
}

inline bool estimateBlendJointPathDemand(const RobotModel& model,
                                         const double startQ[kAxisCount],
                                         const MoveLBlendPlan& plan,
                                         double* outDemand) {
    if (!outDemand || !startQ || !plan.enabled) return false;
    const double length = blendPlanTranslationLength(plan);
    if (!isFinite(length) || length <= 1.0e-9) return false;
    constexpr uint8_t kSamples = 64;
    const double ds = length / static_cast<double>(kSamples);
    if (ds <= 1.0e-12) return false;
    double previousQ[kAxisCount] = {};
    for (uint8_t joint = 0; joint < kAxisCount; ++joint) previousQ[joint] = startQ[joint];
    double maxDemand = 0.0;
    for (uint8_t sample = 1; sample <= kSamples; ++sample) {
        const Transform pose = sampleBlendArcPose(plan, static_cast<double>(sample) / static_cast<double>(kSamples));
        double q[kAxisCount] = {};
        if (!solveToolPoseNearest(model, previousQ, pose, q)) return false;
        for (uint8_t joint = 0; joint < kAxisCount; ++joint) {
            const double demand = absDouble(wrapRadians(q[joint] - previousQ[joint])) / ds;
            if (demand > maxDemand) maxDemand = demand;
            previousQ[joint] = q[joint];
        }
    }
    *outDemand = maxDemand;
    return true;
}

inline void selectBlendGeometryForJointDemand(const RobotModel& model,
                                              const double startQ[kAxisCount],
                                              MoveLBlendPlan* plan) {
    if (!plan || !plan->enabled || !startQ) return;
    MoveLBlendPlan circular = *plan;
    MoveLBlendPlan quintic = *plan;
    circular.useQuintic = 0;
    quintic.useQuintic = 1;
    double circularDemand = 0.0;
    double quinticDemand = 0.0;
    const bool haveCircular = estimateBlendJointPathDemand(model, startQ, circular, &circularDemand);
    const bool haveQuintic = estimateBlendJointPathDemand(model, startQ, quintic, &quinticDemand);
    if (haveQuintic && (!haveCircular || quinticDemand < circularDemand * 0.95)) {
        *plan = quintic;
    } else if (haveCircular) {
        *plan = circular;
    }
}

inline RejectCode applyMoveJJointLimits(const MotionProgramSettings& settings, MotionSegment* segment) {
    if (!segment || segment->kind != MotionSegmentKind::MoveJ) return RejectCode::Ok;
    double jointVelocitySpeedLimit = segment->nominalSpeed;
    double stepRateSpeedLimit = segment->nominalSpeed;
    double accelLimit = segment->acceleration;
    for (uint8_t i = 0; i < kAxisCount; ++i) {
        const double delta = absDouble(wrapRadians(segment->endQ[i] - segment->startQ[i]));
        if (delta <= 1.0e-12 || segment->length <= 1.0e-12) continue;
        const double scale = delta / segment->length;
        const double vLimit = finitePositiveOrZero(settings.jointVelocityLimitRadSec[i]);
        const double aLimit = finitePositiveOrZero(settings.jointAccelerationLimitRadSec2[i]);
        const double stepRateLimit = finitePositiveOrZero(settings.jointStepRateLimitStepsSec[i]);
        const double stepsPerDegree = finitePositiveOrZero(settings.jointStepsPerDegree[i]);
        if (vLimit > 0.0) {
            const double cap = vLimit / scale;
            if (cap > 1.0e-9 && cap < jointVelocitySpeedLimit) jointVelocitySpeedLimit = cap;
        }
        if (aLimit > 0.0) accelLimit = accelLimit < aLimit / scale ? accelLimit : aLimit / scale;
        if (stepRateLimit > 0.0 && stepsPerDegree > 0.0) {
            const double stepsPerPathUnit = radiansToDegrees(scale) * stepsPerDegree;
            if (stepsPerPathUnit > 1.0e-12) {
                const double cap = stepRateLimit / stepsPerPathUnit;
                if (cap > 1.0e-9 && cap < stepRateSpeedLimit) stepRateSpeedLimit = cap;
            }
        }
    }
    capMotionSegmentSpeed(segment, jointVelocitySpeedLimit, MotionCapJointVelocity);
    capMotionSegmentSpeed(segment, stepRateSpeedLimit, MotionCapStepRate);
    capMotionSegmentAcceleration(segment, accelLimit, MotionCapJointAcceleration);
    return RejectCode::Ok;
}

inline RejectCode applyCartesianJointDemandLimits(const RobotModel& model,
                                                  const MotionProgramSettings& settings,
                                                  MotionSegment* segment) {
    if (!segment || segment->kind == MotionSegmentKind::MoveJ || segment->length <= 1.0e-9) {
        return RejectCode::Ok;
    }
    const bool hasVelocityLimits = anyJointLimitConfigured(settings.jointVelocityLimitRadSec);
    const bool hasAccelerationLimits = anyJointLimitConfigured(settings.jointAccelerationLimitRadSec2);
    if (!hasVelocityLimits && !hasAccelerationLimits) return RejectCode::Ok;

    constexpr uint8_t kMaxSamples = 32;
    const uint8_t sampleCount = segment->kind == MotionSegmentKind::MoveLBlend
        ? kMaxSamples
        : (segment->length > 240.0 ? 24 : (segment->length > 120.0 ? 16 : 10));
    double qSamples[kMaxSamples + 1][kAxisCount] = {};
    for (uint8_t joint = 0; joint < kAxisCount; ++joint) qSamples[0][joint] = segment->startQ[joint];

    double previousQ[kAxisCount] = {};
    for (uint8_t joint = 0; joint < kAxisCount; ++joint) previousQ[joint] = segment->startQ[joint];

    const double ds = segment->length / static_cast<double>(sampleCount);
    double jointVelocitySpeedLimit = segment->nominalSpeed;
    double jointAccelerationSpeedLimit = segment->nominalSpeed;
    double pathCurvatureSpeedLimit = segment->nominalSpeed;
    double stepRateSpeedLimit = segment->nominalSpeed;
    double accelLimit = segment->acceleration;
    double minSingularityMargin = 1.0e9;

    if (segment->curvature > 1.0e-12) {
        const double linearAccel = finitePositiveOrZero(settings.defaultLinearAccelerationMmSec2);
        if (linearAccel > 0.0) {
            const double cap = sqrt(linearAccel / segment->curvature);
            if (isFinite(cap) && cap > 1.0e-9 && cap < pathCurvatureSpeedLimit) {
                pathCurvatureSpeedLimit = cap;
            }
        }
    }

    for (uint8_t sampleIndex = 1; sampleIndex <= sampleCount; ++sampleIndex) {
        const double t = static_cast<double>(sampleIndex) / static_cast<double>(sampleCount);
        const Transform pose = sampleMotionSegmentPoseOnly(*segment, t);
        if (!solveToolPoseNearest(model, previousQ, pose, qSamples[sampleIndex])) {
            return RejectCode::NoIkSolution;
        }
        const double singularityMargin = wristSingularityMarginRad(qSamples[sampleIndex]);
        if (singularityMargin < minSingularityMargin) minSingularityMargin = singularityMargin;
        for (uint8_t joint = 0; joint < kAxisCount; ++joint) {
            const double dq = absDouble(wrapRadians(qSamples[sampleIndex][joint] - previousQ[joint]));
            if (dq <= 1.0e-12 || ds <= 1.0e-12) continue;
            const double dqds = dq / ds;
            const double vLimit = finitePositiveOrZero(settings.jointVelocityLimitRadSec[joint]);
            const double aLimit = finitePositiveOrZero(settings.jointAccelerationLimitRadSec2[joint]);
            const double stepRateLimit = finitePositiveOrZero(settings.jointStepRateLimitStepsSec[joint]);
            const double stepsPerDegree = finitePositiveOrZero(settings.jointStepsPerDegree[joint]);
            if (vLimit > 0.0) {
                const double cap = vLimit / dqds;
                if (cap > 1.0e-9 && cap < jointVelocitySpeedLimit) jointVelocitySpeedLimit = cap;
            }
            if (aLimit > 0.0) {
                const double cap = aLimit / dqds;
                if (cap > 1.0e-9 && cap < accelLimit) accelLimit = cap;
            }
            if (stepRateLimit > 0.0 && stepsPerDegree > 0.0) {
                const double stepsPerMm = dqds * radiansToDegrees(1.0) * stepsPerDegree;
                if (stepsPerMm > 1.0e-12) {
                    const double cap = stepRateLimit / stepsPerMm;
                    if (cap > 1.0e-9 && cap < stepRateSpeedLimit) stepRateSpeedLimit = cap;
                }
            }
        }
        for (uint8_t joint = 0; joint < kAxisCount; ++joint) previousQ[joint] = qSamples[sampleIndex][joint];
    }

    if (hasAccelerationLimits && ds > 1.0e-12) {
        for (uint8_t sampleIndex = 1; sampleIndex < sampleCount; ++sampleIndex) {
            for (uint8_t joint = 0; joint < kAxisCount; ++joint) {
                const double dq0 = wrapRadians(qSamples[sampleIndex][joint] - qSamples[sampleIndex - 1][joint]);
                const double dq1 = wrapRadians(qSamples[sampleIndex + 1][joint] - qSamples[sampleIndex][joint]);
                const double d2qds2 = absDouble(dq1 - dq0) / (ds * ds);
                const double aLimit = finitePositiveOrZero(settings.jointAccelerationLimitRadSec2[joint]);
                if (aLimit > 0.0 && d2qds2 > 1.0e-12) {
                    const double cap = sqrt(aLimit / d2qds2);
                    if (cap > 1.0e-9 && cap < jointAccelerationSpeedLimit) jointAccelerationSpeedLimit = cap;
                }
            }
        }
    }

    if (minSingularityMargin < 1.0e8) segment->singularityMarginRad = minSingularityMargin;
    capMotionSegmentSpeed(segment, jointVelocitySpeedLimit, MotionCapJointVelocity);
    capMotionSegmentSpeed(segment, jointAccelerationSpeedLimit, MotionCapJointAcceleration);
    capMotionSegmentSpeed(segment, pathCurvatureSpeedLimit, MotionCapPathAcceleration);
    capMotionSegmentSpeed(segment, stepRateSpeedLimit, MotionCapStepRate);
    capMotionSegmentAcceleration(segment, accelLimit, MotionCapJointAcceleration);
    return RejectCode::Ok;
}

// Peak first, second and third derivative of one cycle of a unit-amplitude shape, with respect to
// cycle count, plus the largest excursion the two axes reach together.
struct WeaveShapeHarmonics {
    double d1;
    double d2;
    double d3;
    double excursion;
};

inline WeaveShapeHarmonics weaveShapeHarmonics(WeaveShape shape) {
    switch (shape) {
    case WeaveShape::Sine:
    case WeaveShape::Circular:    return {6.284, 39.48, 248.1, 1.0};
    case WeaveShape::Zigzag:      return {5.094, 102.89, 1695.5, 1.0};
    case WeaveShape::Trapezoid:   return {8.993, 66.92, 2533.2, 1.0};
    case WeaveShape::FigureEight: return {14.050, 160.38, 1999.8, 1.25};
    case WeaveShape::LShape:      return {8.162, 157.91, 2714.5, 1.0};
    case WeaveShape::None:        break;
    }
    return {6.284, 39.48, 248.1, 1.0};
}

inline const WeaveParams* motionSegmentWeave(const MotionSegmentProgram& program, uint16_t segmentIndex) {
    if (segmentIndex >= program.count) return nullptr;
    const uint8_t index = program.segments[segmentIndex].commandIndex;
    if (index >= kMaxMotionCommands) return nullptr;
    return &program.commandWeave[index];
}

// Peak speed the pattern adds across the seam. A commanded speed is a travel speed along the taught
// line, which is how all four vendors define it, so while the weave runs the torch itself covers
// more ground than the seam does and genuinely moves faster than the number the operator typed.
inline double weaveTransverseSpeedMmPerSec(const WeaveParams& weave, double pathSpeedMmPerSec) {
    if (!weaveParamsAreUsable(weave)) return 0.0;
    const double amplitude = weaveAmplitudePeakMm(weave);
    if (amplitude <= 1.0e-9) return 0.0;
    double rate = 0.0;
    if (weave.rateMode == WeaveRateMode::Frequency) {
        rate = weave.frequencyHz;
    } else if (weave.wavelengthMm > 1.0e-9) {
        rate = pathSpeedMmPerSec / weave.wavelengthMm;
    }
    if (rate <= 1.0e-9) return 0.0;
    return amplitude * (weaveShapeHarmonics(weave.shape).d1 + kWeaveRampPeakSlope) * rate;
}

// Radians of joint travel per millimetre of weave displacement, measured in the middle of the
// segment by displacing the pose and re-solving. Two extra solves per weaving segment, against the
// ten to thirty applyCartesianJointDemandLimits already performs.
ROBOT_MOTION_CORE_COLD inline bool weaveJointSensitivity(const RobotModel& model,
                                                         const MotionSegment& segment,
                                                         const WeaveParams& weave,
                                                         double outPerMm[kAxisCount]) {
    if (!outPerMm) return false;
    for (uint8_t joint = 0; joint < kAxisCount; ++joint) outPerMm[joint] = 0.0;
    const double amplitude = weaveAmplitudePeakMm(weave);
    if (amplitude <= 1.0e-9) return true;

    const Transform pose = sampleMotionSegmentPoseOnly(segment, 0.5);
    double tangent[3] = {};
    if (!weaveTangentForSegment(segment, 0.5, tangent)) return true;
    double lateral[3] = {};
    double normal[3] = {};
    if (!weaveFrameVectors(pose, tangent, weave.planeAngleDeg, lateral, normal)) return true;

    double baseQ[kAxisCount] = {};
    if (!solveToolPoseNearest(model, segment.startQ, pose, baseQ)) return false;

    // Probe the full biased lateral excursion because joint sensitivity varies across the workspace.
    // Bias applies only to the lateral component.
    const double lateralReach =
        isFinite(weave.biasMm) && weave.biasMm < 0.0 ? -weaveLateralReachMm(weave)
                                                     : weaveLateralReachMm(weave);
    const double probes[2][2] = {{lateralReach, 0.0}, {0.0, amplitude}};
    for (uint8_t probe = 0; probe < 2; ++probe) {
        if (probe == 1 && (!isFinite(weave.elevationMm) || weave.elevationMm <= 1.0e-9)) continue;
    // Report positive joint travel per millimetre for each signed probe displacement.
        const double distance = absDouble(probes[probe][0]) + absDouble(probes[probe][1]);
        if (distance <= 1.0e-9) continue;
        const Transform displaced = weaveDisplacePose(pose, lateral, normal, probes[probe]);
        double probeQ[kAxisCount] = {};
        if (!solveToolPoseNearest(model, baseQ, displaced, probeQ)) return false;
        for (uint8_t joint = 0; joint < kAxisCount; ++joint) {
            const double perMm = absDouble(wrapRadians(probeQ[joint] - baseQ[joint])) / distance;
            if (perMm > outPerMm[joint]) outPerMm[joint] = perMm;
        }
    }
    return true;
}

// Makes room for the weave before the base path is profiled.
ROBOT_MOTION_CORE_COLD inline RejectCode applyWeaveDynamicLimits(const RobotModel& model,
                                                                 const WeaveParams& weave,
                                                                 MotionSegment* segmentOut,
                                                                 MotionProgramSettings* settings) {
    if (!settings || !segmentOut || !weaveParamsAreUsable(weave)) return RejectCode::Ok;
    const MotionSegment& segment = *segmentOut;
    if (segment.kind == MotionSegmentKind::MoveJ) return RejectCode::Ok;

    const WeaveShapeHarmonics harmonics = weaveShapeHarmonics(weave.shape);
    const double amplitude = weaveAmplitudePeakMm(weave) * harmonics.excursion;
    if (amplitude <= 1.0e-9) return RejectCode::Ok;

    double perMm[kAxisCount] = {};
    if (!weaveJointSensitivity(model, segment, weave, perMm)) return RejectCode::WeaveInfeasible;

    constexpr double kPathShare = 0.1;
    // A hair under the share the reservation will then check against. The cap solves for the speed at
    // which the demand exactly equals its ceiling, and the reservation below arrives at the same
    // quantity by a different route - sqrt on the way down, a squared rate on the way back - so it
    // landed a few ULPs over and refused a plan it had just finished making fit. Capping to a
    // millionth inside leaves room for that without giving up anything physical: on the AR4 this was
    // the whole difference between a weave planning at 21.9 mm/s and being rejected outright.
    constexpr double kWeaveCapHeadroom = 1.0 - 1.0e-6;
    const double weaveShare = (1.0 - kPathShare) * kWeaveCapHeadroom;

    double rate = 0.0;
    if (weave.rateMode == WeaveRateMode::Frequency) {
        rate = weave.frequencyHz;
    } else {
        const double k = 1.0 / weave.wavelengthMm;
        // Capped against the weave's share, not the whole ceiling. Capping against the whole one
        // and then reserving a share below is self-contradictory: the cap picks the speed at which
        // the weave uses everything, and the reservation then refuses it for doing so.
        const double linearAccelCap = finitePositiveOrZero(settings->defaultLinearAccelerationMmSec2) * weaveShare;
        if (linearAccelCap > 0.0) {
            const double denominator = amplitude * harmonics.d2 * k * k;
            if (denominator > 1.0e-12) {
                capMotionSegmentSpeed(segmentOut, sqrt(linearAccelCap / denominator), MotionCapWeave);
            }
        }
        const double linearJerkCap = finitePositiveOrZero(settings->defaultLinearJerkMmSec3) * weaveShare;
        if (linearJerkCap > 0.0) {
            const double denominator = amplitude * harmonics.d3 * k * k * k;
            if (denominator > 1.0e-12) {
                capMotionSegmentSpeed(segmentOut, pow(linearJerkCap / denominator, 1.0 / 3.0), MotionCapWeave);
            }
        }
        for (uint8_t joint = 0; joint < kAxisCount; ++joint) {
            if (perMm[joint] <= 1.0e-12) continue;
            const double velocityLimit = finitePositiveOrZero(settings->jointVelocityLimitRadSec[joint]) * weaveShare;
            if (velocityLimit > 0.0) {
                const double denominator = amplitude * harmonics.d1 * k * perMm[joint];
                if (denominator > 1.0e-12) {
                    capMotionSegmentSpeed(segmentOut, velocityLimit / denominator, MotionCapWeave);
                }
            }
            const double accelerationLimit =
                finitePositiveOrZero(settings->jointAccelerationLimitRadSec2[joint]) * weaveShare;
            if (accelerationLimit > 0.0) {
                const double denominator = amplitude * harmonics.d2 * k * k * perMm[joint];
                if (denominator > 1.0e-12) {
                    capMotionSegmentSpeed(segmentOut, sqrt(accelerationLimit / denominator), MotionCapWeave);
                }
            }
        }
        // Evaluated at the capped speed. Anything downstream can only slow the segment further,
        // which makes the weave cheaper than assumed here, so the reservation stays on the safe side.
        rate = segmentOut->nominalSpeed * k;
    }
    if (rate <= 1.0e-9) return RejectCode::Ok;

    const double weaveSpeed = amplitude * harmonics.d1 * rate;
    const double weaveAccel = amplitude * harmonics.d2 * rate * rate;
    const double weaveJerk = amplitude * harmonics.d3 * rate * rate * rate;

    const double linearAccel = finitePositiveOrZero(settings->defaultLinearAccelerationMmSec2);
    const double linearJerk = finitePositiveOrZero(settings->defaultLinearJerkMmSec3);

    // The direct terms only. In wavelength mode the excursion is a function of position, so
    // differentiating through the path adds cross terms in the path's own acceleration and jerk,
    // but bounding each of those by its independent maximum is wildly pessimistic: they peak a
    // quarter cycle apart and the path is only accelerating hard while it is still slow. Rather
    // than reserve for a worst case that never occurs, the estimate below stays deliberately
    // optimistic and profileContinuousMotionProgram derates against the sampled verifier, which
    // measures the real trajectory instead of bounding it.
    if (linearAccel > 0.0) {
        if (weaveAccel > linearAccel * (1.0 - kPathShare)) return RejectCode::WeaveInfeasible;
        settings->defaultLinearAccelerationMmSec2 = linearAccel - weaveAccel;
    }
    if (linearJerk > 0.0) {
        if (weaveJerk > linearJerk * (1.0 - kPathShare)) return RejectCode::WeaveInfeasible;
        settings->defaultLinearJerkMmSec3 = linearJerk - weaveJerk;
    }
        // Keep the segment profile limits consistent with the reduced settings.
    capMotionSegmentAcceleration(segmentOut, settings->defaultLinearAccelerationMmSec2, MotionCapWeave);
    if (settings->defaultLinearJerkMmSec3 > 0.0 && segmentOut->jerk > settings->defaultLinearJerkMmSec3) {
        segmentOut->jerk = settings->defaultLinearJerkMmSec3;
        markMotionCap(segmentOut, MotionCapWeave);
    }

    for (uint8_t joint = 0; joint < kAxisCount; ++joint) {
        if (perMm[joint] <= 1.0e-12) continue;
        const double velocityLimit = finitePositiveOrZero(settings->jointVelocityLimitRadSec[joint]);
        if (velocityLimit > 0.0) {
            const double used = weaveSpeed * perMm[joint];
            if (used > velocityLimit * (1.0 - kPathShare)) return RejectCode::WeaveInfeasible;
            settings->jointVelocityLimitRadSec[joint] = velocityLimit - used;
        }
        const double accelerationLimit = finitePositiveOrZero(settings->jointAccelerationLimitRadSec2[joint]);
        if (accelerationLimit > 0.0) {
            const double used = weaveAccel * perMm[joint];
            if (used > accelerationLimit * (1.0 - kPathShare)) return RejectCode::WeaveInfeasible;
            settings->jointAccelerationLimitRadSec2[joint] = accelerationLimit - used;
        }
    }
    return RejectCode::Ok;
}


inline uint8_t motionSegmentSplitCount(const MotionSegment& segment, const MotionProgramSettings& settings) {
    if (segment.length <= 1.0e-9) return 1;
    const double controlPeriod = settings.controlPeriodSec > 1.0e-6 ? settings.controlPeriodSec : 0.005;
    double period = controlPeriod * 0.5;
    if (period < 0.001) period = 0.001;
    const double speed = segment.nominalSpeed > 1.0e-9 ? segment.nominalSpeed : 1.0;
    double spacing = speed * period * 4.0;
    if (segment.kind == MotionSegmentKind::MoveJ) {
        const double jointSample = settings.jointSampleDeg > 0.0 ? degreesToRadians(settings.jointSampleDeg) : degreesToRadians(2.0);
        spacing = jointSample * 4.0;
    } else if (segment.kind == MotionSegmentKind::MoveLBlend) {
        spacing = speed * period * 2.0;
    }
    if (!isFinite(spacing) || spacing <= 1.0e-9) spacing = segment.length;
    uint32_t count = static_cast<uint32_t>(ceil(segment.length / spacing));
    if (segment.kind == MotionSegmentKind::MoveLBlend && count < 4) {
        count = 4;
    }
    if (count < 1) count = 1;
    if (count > 32) count = 32;
    return static_cast<uint8_t>(count);
}

inline uint8_t minimumMotionSegmentSplitCount(const MotionSegment& segment) {
    return segment.kind == MotionSegmentKind::MoveLBlend ? 4 : 1;
}

ROBOT_MOTION_CORE_COLD inline RejectCode buildMotionPathGridFromCurve(const RobotModel& model,
                                                                      const MotionProgramSettings& settings,
                                                                      MotionSegmentProgram* program) {
    if (!program) return RejectCode::BadTarget;
    MotionPathGrid& grid = program->pathGrid;
    memset(&grid, 0, sizeof(grid));
    if (program->count == 0) return RejectCode::Ok;

    double s = 0.0;
    for (uint8_t i = 0; i < program->count; ++i) {
        MotionSegment& segment = program->segments[i];
        segment.globalPathStart = s;
        s += segment.length > 0.0 ? segment.length : 0.0;
        segment.globalPathEnd = s;
    }
    grid.totalLength = s;

    uint8_t splitCounts[kMaxMotionSegments] = {};
    uint16_t totalNodeCount = 1;
    for (uint8_t i = 0; i < program->count; ++i) {
        splitCounts[i] = motionSegmentSplitCount(program->segments[i], settings);
        totalNodeCount = static_cast<uint16_t>(totalNodeCount + splitCounts[i]);
    }
    while (totalNodeCount > kMaxMotionPathNodes) {
        uint8_t candidate = kMaxMotionSegments;
        uint8_t candidateCount = 0;
        for (uint8_t i = 0; i < program->count; ++i) {
            const uint8_t minimum = minimumMotionSegmentSplitCount(program->segments[i]);
            if (splitCounts[i] > minimum && splitCounts[i] > candidateCount) {
                candidate = i;
                candidateCount = splitCounts[i];
            }
        }
        if (candidate >= program->count) return RejectCode::TooManyTicks;
        --splitCounts[candidate];
        --totalNodeCount;
    }

    MotionSegment& first = program->segments[0];
    initializeMotionPathNode(&grid.nodes[0]);
    grid.nodes[0].s = 0.0;
    if (program->continuesBeforeStart) {
        double initialLimit = first.nominalSpeed > 0.0 ? first.nominalSpeed : 1.0;
        if (isFinite(program->initialPathSpeed) && program->initialPathSpeed > 0.0) {
            initialLimit = program->initialPathSpeed < initialLimit ? program->initialPathSpeed : initialLimit;
        }
        grid.nodes[0].speedLimit = initialLimit;
        grid.nodes[0].solvedSpeed = initialLimit;
    } else {
        grid.nodes[0].speedLimit = 0.0;
        grid.nodes[0].solvedSpeed = 0.0;
    }
    grid.nodes[0].accelLimit = finitePositiveOrZero(first.acceleration) > 0.0 ? first.acceleration : 1.0;
    grid.nodes[0].jerkLimit = finitePositiveOrZero(first.jerk) > 0.0 ? first.jerk : grid.nodes[0].accelLimit * 1000.0;
    grid.nodes[0].curvature = first.curvature;
    grid.nodes[0].orientationRatioRadPerPathUnit = first.orientationRatioRadPerPathUnit;
    grid.nodes[0].singularityMarginRad = first.singularityMarginRad > 0.0 ? first.singularityMarginRad : 1.0e9;
    grid.nodes[0].commandId = first.commandId;
    grid.nodes[0].segmentIndex = 0;
    grid.nodes[0].localT = 0.0;
    assignMotionPathNodePose(&grid.nodes[0], first.startTcp, first.startQ);
    grid.count = 1;

    for (uint8_t i = 0; i < program->count; ++i) {
        MotionSegment& segment = program->segments[i];
        const uint8_t sampleCount = splitCounts[i] > 0 ? splitCounts[i] : 1;
        for (uint8_t sample = 1; sample <= sampleCount; ++sample) {
            if (grid.count >= kMaxMotionPathNodes) return RejectCode::TooManyTicks;
            const double localT = static_cast<double>(sample) / static_cast<double>(sampleCount);
            MotionPathNode& node = grid.nodes[grid.count];
            initializeMotionPathNode(&node);
            node.s = segment.globalPathStart + segment.length * localT;
            node.segmentIndex = i;
            node.localT = localT;
            node.commandId = segment.commandId;
            node.completedCommandId = sample == sampleCount ? segment.completedCommandId : -1;
            node.finePoint = sample == sampleCount ? segment.finePoint : 0;
            node.capReasonMask = segment.capReasonMask;
            node.curvature = segment.curvature;
            node.orientationRatioRadPerPathUnit = segment.orientationRatioRadPerPathUnit;
            node.singularityMarginRad = segment.singularityMarginRad > 0.0 ? segment.singularityMarginRad : 1.0e9;
            node.accelLimit = finitePositiveOrZero(segment.acceleration) > 0.0 ? segment.acceleration : 1.0;
            node.jerkLimit = finitePositiveOrZero(segment.jerk) > 0.0 ? segment.jerk : node.accelLimit * 1000.0;

            if (segment.kind == MotionSegmentKind::MoveJ) {
                for (uint8_t joint = 0; joint < kAxisCount; ++joint) {
                    node.q[joint] = segment.startQ[joint] + wrapRadians(segment.endQ[joint] - segment.startQ[joint]) * localT;
                }
                node.tcp = toolPoseForJoints(model, node.q);
            } else {
                node.tcp = sampleMotionSegmentPoseOnly(segment, localT);
                const MotionPathNode& previousNode = grid.nodes[grid.count - 1];
                if (!solveToolPoseNearest(model, previousNode.q, node.tcp, node.q)) {
                    return RejectCode::NoIkSolution;
                }
            }

            double speedLimit = segment.nominalSpeed > 0.0 ? segment.nominalSpeed : 1.0;
            if (sample == sampleCount && i + 1 < program->count) {
                const MotionSegment& next = program->segments[i + 1];
                if (next.nominalSpeed > 0.0 && next.nominalSpeed < speedLimit) speedLimit = next.nominalSpeed;
                if (next.acceleration > 0.0 && next.acceleration < node.accelLimit) node.accelLimit = next.acceleration;
                if (next.jerk > 0.0 && next.jerk < node.jerkLimit) node.jerkLimit = next.jerk;
                if (next.curvature > node.curvature) node.curvature = next.curvature;
                if (next.orientationRatioRadPerPathUnit > node.orientationRatioRadPerPathUnit) {
                    node.orientationRatioRadPerPathUnit = next.orientationRatioRadPerPathUnit;
                }
            }
            if ((sample == sampleCount && segment.finePoint) ||
                (i + 1 == program->count && !program->continuesAfterEnd)) {
                speedLimit = 0.0;
            }
            if (segment.kind != MotionSegmentKind::MoveJ) {
            // The singularity margin is diagnostic and does not cap speed.
                node.singularityMarginRad = wristSingularityMarginRad(node.q);
            }
            node.speedLimit = speedLimit;
            node.solvedSpeed = speedLimit;
            ++grid.count;
        }
    }

    if (!program->continuesBeforeStart) {
        grid.nodes[0].speedLimit = 0.0;
        grid.nodes[0].solvedSpeed = 0.0;
    }
    if (!program->continuesAfterEnd) {
        grid.nodes[grid.count - 1].speedLimit = 0.0;
        grid.nodes[grid.count - 1].solvedSpeed = 0.0;
    }
    for (uint16_t edge = 0; edge + 1 < grid.count; ++edge) {
        const uint16_t pathSegmentIndex = grid.nodes[edge + 1].segmentIndex;
        const MotionSegment& segment = program->segments[pathSegmentIndex];
        grid.edgeSpeedLimit[edge] = segment.nominalSpeed > 1.0e-9 ? segment.nominalSpeed : 1.0;
        grid.edgeCapReasonMask[edge] = segment.capReasonMask;
    }

    return RejectCode::Ok;
}

inline void capMotionPathNodeSpeed(MotionPathNode* node, double cap, uint32_t reason) {
    if (!node || !isFinite(cap) || cap <= 1.0e-9) return;
    if (node->speedLimit <= 0.0 || cap < node->speedLimit) {
        node->speedLimit = cap;
        if (node->solvedSpeed <= 0.0 || node->solvedSpeed > cap) node->solvedSpeed = cap;
        node->capReasonMask |= reason;
    }
}

ROBOT_MOTION_CORE_COLD inline void applyMotionPathGridDerivativeConstraints(const MotionProgramSettings& settings,
                                                                            MotionSegmentProgram* program) {
    if (!program || program->pathGrid.count < 2) return;
    MotionPathGrid& grid = program->pathGrid;

    for (uint16_t edge = 0; edge + 1 < grid.count; ++edge) {
        MotionPathNode& currentNode = grid.nodes[edge];
        MotionPathNode& nextNode = grid.nodes[edge + 1];
        const double ds = nextNode.s - currentNode.s;
        if (ds <= 1.0e-12) continue;
        for (uint8_t joint = 0; joint < kAxisCount; ++joint) {
            const double dqds = absDouble(wrapRadians(nextNode.q[joint] - currentNode.q[joint])) / ds;
            if (dqds <= 1.0e-12) continue;
            const double vLimit = finitePositiveOrZero(settings.jointVelocityLimitRadSec[joint]);
            if (vLimit > 0.0) {
                capMotionPathNodeSpeed(&nextNode, vLimit / dqds, MotionCapJointVelocity);
            }
            const double stepRateLimit = finitePositiveOrZero(settings.jointStepRateLimitStepsSec[joint]);
            const double stepsPerDegree = finitePositiveOrZero(settings.jointStepsPerDegree[joint]);
            if (stepRateLimit > 0.0 && stepsPerDegree > 0.0) {
                const double stepsPerPathUnit = dqds * radiansToDegrees(1.0) * stepsPerDegree;
                if (stepsPerPathUnit > 1.0e-12) {
                    capMotionPathNodeSpeed(&nextNode, stepRateLimit / stepsPerPathUnit, MotionCapStepRate);
                }
            }
            const double jLimit = finitePositiveOrZero(settings.defaultJointJerkRadSec3);
            if (jLimit > 0.0) {
                constexpr double kJointPathJerkSafety = 0.50;
                const double pathJerkCap = (jLimit / dqds) * kJointPathJerkSafety;
                if (pathJerkCap > 1.0e-9) {
                    if (pathJerkCap < currentNode.jerkLimit) currentNode.jerkLimit = pathJerkCap;
                    if (pathJerkCap < nextNode.jerkLimit) nextNode.jerkLimit = pathJerkCap;
                    currentNode.capReasonMask |= static_cast<uint32_t>(MotionCapJerk);
                    nextNode.capReasonMask |= static_cast<uint32_t>(MotionCapJerk);
                }
            }
        }
        const double orientationRatio = currentNode.orientationRatioRadPerPathUnit > nextNode.orientationRatioRadPerPathUnit
            ? currentNode.orientationRatioRadPerPathUnit
            : nextNode.orientationRatioRadPerPathUnit;
        if (orientationRatio > 1.0e-12) {
            const double angularSpeed = isFinite(settings.defaultToolAngularSpeedRadSec) && settings.defaultToolAngularSpeedRadSec > 0.0
                ? settings.defaultToolAngularSpeedRadSec
                : 0.0;
            if (angularSpeed > 0.0) {
                capMotionPathNodeSpeed(&nextNode, angularSpeed / orientationRatio, MotionCapJointVelocity);
            }
        }
    }

    for (uint16_t node = 1; node + 1 < grid.count; ++node) {
        MotionPathNode& previous = grid.nodes[node - 1];
        MotionPathNode& current = grid.nodes[node];
        MotionPathNode& next = grid.nodes[node + 1];
        if (previous.finePoint || current.finePoint) continue;
        const double ds0 = current.s - previous.s;
        const double ds1 = next.s - current.s;
        const double ds = 0.5 * (ds0 + ds1);
        if (ds <= 1.0e-12 || ds0 <= 1.0e-12 || ds1 <= 1.0e-12) continue;
        for (uint8_t joint = 0; joint < kAxisCount; ++joint) {
            const double aLimit = finitePositiveOrZero(settings.jointAccelerationLimitRadSec2[joint]);
            if (aLimit <= 0.0) continue;
            const double dq0 = wrapRadians(current.q[joint] - previous.q[joint]) / ds0;
            const double dq1 = wrapRadians(next.q[joint] - current.q[joint]) / ds1;
            const double d2qds2 = absDouble(dq1 - dq0) / ds;
            if (d2qds2 > 1.0e-12) {
                capMotionPathNodeSpeed(&current, sqrt(aLimit / d2qds2), MotionCapJointAcceleration);
            }
        }
        const double linearAccel = finitePositiveOrZero(settings.defaultLinearAccelerationMmSec2);
        if (linearAccel > 0.0 && current.curvature > 1.0e-12) {
            capMotionPathNodeSpeed(&current, sqrt(linearAccel / current.curvature), MotionCapPathAcceleration);
        }
        const double angularAccel = isFinite(settings.defaultToolAngularAccelerationRadSec2) && settings.defaultToolAngularAccelerationRadSec2 > 0.0
            ? settings.defaultToolAngularAccelerationRadSec2
            : 0.0;
        if (angularAccel > 0.0) {
            const double ratio0 = previous.orientationRatioRadPerPathUnit;
            const double ratio1 = next.orientationRatioRadPerPathUnit;
            const double dRatioDs = absDouble(ratio1 - ratio0) / ds;
            if (dRatioDs > 1.0e-12) {
                capMotionPathNodeSpeed(&current, sqrt(angularAccel / dRatioDs), MotionCapJointAcceleration);
            }
        }
    }

    for (uint16_t node = 1; node + 2 < grid.count; ++node) {
        const MotionPathNode& a = grid.nodes[node - 1];
        const MotionPathNode& b = grid.nodes[node];
        const MotionPathNode& c = grid.nodes[node + 1];
        const MotionPathNode& d = grid.nodes[node + 2];
        if (a.finePoint || b.finePoint || c.finePoint) continue;
        const double ds0 = b.s - a.s;
        const double ds1 = c.s - b.s;
        const double ds2 = d.s - c.s;
        const double ds = (ds0 + ds1 + ds2) / 3.0;
        if (ds <= 1.0e-12 || ds0 <= 1.0e-12 || ds1 <= 1.0e-12 || ds2 <= 1.0e-12) continue;
        for (uint8_t joint = 0; joint < kAxisCount; ++joint) {
            const double jLimit = finitePositiveOrZero(settings.defaultJointJerkRadSec3);
            if (jLimit <= 0.0) continue;
            const double dq0 = wrapRadians(b.q[joint] - a.q[joint]) / ds0;
            const double dq1 = wrapRadians(c.q[joint] - b.q[joint]) / ds1;
            const double dq2 = wrapRadians(d.q[joint] - c.q[joint]) / ds2;
            const double d3qds3 = absDouble(dq2 - 2.0 * dq1 + dq0) / (ds * ds);
            if (d3qds3 > 1.0e-12) {
                const double cap = pow(jLimit / d3qds3, 1.0 / 3.0);
                capMotionPathNodeSpeed(&grid.nodes[node + 1], cap, MotionCapJerk);
            }
        }
        const double angularJerk = isFinite(settings.defaultToolAngularJerkRadSec3) && settings.defaultToolAngularJerkRadSec3 > 0.0
            ? settings.defaultToolAngularJerkRadSec3
            : 0.0;
        if (angularJerk > 0.0) {
            const double d3ods3 = absDouble(d.orientationRatioRadPerPathUnit -
                                           3.0 * c.orientationRatioRadPerPathUnit +
                                           3.0 * b.orientationRatioRadPerPathUnit -
                                           a.orientationRatioRadPerPathUnit) / (ds * ds);
            if (d3ods3 > 1.0e-12) {
                const double cap = pow(angularJerk / d3ods3, 1.0 / 3.0);
                capMotionPathNodeSpeed(&grid.nodes[node + 1], cap, MotionCapJerk);
            }
        }
    }
}

ROBOT_MOTION_CORE_COLD inline void estimateMotionTrajectoryNodeAccelerations(const MotionPathGrid& grid,
                                                                             MotionTrajectory* trajectory) {
    if (!trajectory) return;
    for (uint16_t node = 0; node < trajectory->count; ++node) {
        trajectory->nodes[node].acceleration = 0.0;
        trajectory->nodes[node].jerk = 0.0;
    }
    if (trajectory->count < 3 || trajectory->edgeCount + 1 != trajectory->count) return;

    for (uint16_t node = 1; node + 1 < trajectory->count; ++node) {
        if (node >= grid.count) return;
        if (grid.nodes[node].finePoint || trajectory->nodes[node].speed <= 1.0e-9) {
            trajectory->nodes[node].acceleration = 0.0;
            trajectory->nodes[node].jerk = 0.0;
            continue;
        }

        const MotionTrajectoryEdge& previousEdge = trajectory->edges[node - 1];
        const MotionTrajectoryEdge& nextEdge = trajectory->edges[node];
        if (!previousEdge.blended && !nextEdge.blended) {
            trajectory->nodes[node].acceleration = 0.0;
            trajectory->nodes[node].jerk = 0.0;
            continue;
        }
        if (previousEdge.durationSec <= 1.0e-9 || nextEdge.durationSec <= 1.0e-9) {
            trajectory->nodes[node].acceleration = 0.0;
            trajectory->nodes[node].jerk = 0.0;
            continue;
        }

        const double previousSlope = (trajectory->nodes[node].speed - trajectory->nodes[node - 1].speed) /
                                     previousEdge.durationSec;
        const double nextSlope = (trajectory->nodes[node + 1].speed - trajectory->nodes[node].speed) /
                                 nextEdge.durationSec;
        if (!isFinite(previousSlope) || !isFinite(nextSlope) || previousSlope * nextSlope <= 0.0) {
            trajectory->nodes[node].acceleration = 0.0;
            trajectory->nodes[node].jerk = 0.0;
            continue;
        }

        double magnitude = absDouble(previousSlope) < absDouble(nextSlope)
            ? absDouble(previousSlope)
            : absDouble(nextSlope);
        double localLimit = grid.nodes[node].accelLimit;
        if (previousEdge.acceleration > 1.0e-9 && previousEdge.acceleration < localLimit) {
            localLimit = previousEdge.acceleration;
        }
        if (nextEdge.acceleration > 1.0e-9 && nextEdge.acceleration < localLimit) {
            localLimit = nextEdge.acceleration;
        }
        if (isFinite(localLimit) && localLimit > 1.0e-9 && magnitude > localLimit) {
            magnitude = localLimit;
        }
        magnitude *= 0.5;
        trajectory->nodes[node].acceleration = previousSlope < 0.0 ? -magnitude : magnitude;
        trajectory->nodes[node].jerk = 0.0;
    }

    for (uint8_t pass = 0; pass < 4; ++pass) {
        bool changed = false;
        for (uint16_t edge = 0; edge < trajectory->edgeCount; ++edge) {
            const MotionTrajectoryEdge& trajectoryEdge = trajectory->edges[edge];
            if (trajectoryEdge.startNodeIndex >= trajectory->count ||
                trajectoryEdge.endNodeIndex >= trajectory->count ||
                trajectoryEdge.durationSec <= 1.0e-9) {
                continue;
            }
            const double jerkLimit = trajectoryEdge.jerk > 1.0e-9 ? trajectoryEdge.jerk : 0.0;
            if (jerkLimit <= 0.0) continue;
            const double maxDelta = jerkLimit * trajectoryEdge.durationSec * 0.5;
            MotionTrajectoryNode& start = trajectory->nodes[trajectoryEdge.startNodeIndex];
            MotionTrajectoryNode& end = trajectory->nodes[trajectoryEdge.endNodeIndex];
            const double high = start.acceleration + maxDelta;
            const double low = start.acceleration - maxDelta;
            if (end.acceleration > high) {
                end.acceleration = high;
                changed = true;
            } else if (end.acceleration < low) {
                end.acceleration = low;
                changed = true;
            }
        }
        for (uint16_t edge = trajectory->edgeCount; edge-- > 0;) {
            const MotionTrajectoryEdge& trajectoryEdge = trajectory->edges[edge];
            if (trajectoryEdge.startNodeIndex >= trajectory->count ||
                trajectoryEdge.endNodeIndex >= trajectory->count ||
                trajectoryEdge.durationSec <= 1.0e-9) {
                continue;
            }
            const double jerkLimit = trajectoryEdge.jerk > 1.0e-9 ? trajectoryEdge.jerk : 0.0;
            if (jerkLimit <= 0.0) {
                if (edge == 0) break;
                continue;
            }
            const double maxDelta = jerkLimit * trajectoryEdge.durationSec * 0.5;
            MotionTrajectoryNode& start = trajectory->nodes[trajectoryEdge.startNodeIndex];
            const MotionTrajectoryNode& end = trajectory->nodes[trajectoryEdge.endNodeIndex];
            const double high = end.acceleration + maxDelta;
            const double low = end.acceleration - maxDelta;
            if (start.acceleration > high) {
                start.acceleration = high;
                changed = true;
            } else if (start.acceleration < low) {
                start.acceleration = low;
                changed = true;
            }
            if (edge == 0) break;
        }
        trajectory->nodes[0].acceleration = 0.0;
        trajectory->nodes[trajectory->count - 1].acceleration = 0.0;
        if (!changed) break;
    }
}

ROBOT_MOTION_CORE_COLD inline void recomputeMotionTrajectoryTiming(MotionSegmentProgram* program) {
    if (!program) return;
    MotionTrajectory& trajectory = program->trajectory;
    for (uint8_t i = 0; i < program->count; ++i) {
        MotionSegment& segment = program->segments[i];
        segment.startTimeSec = 0.0;
        segment.durationSec = 0.0;
        segment.startSpeed = 0.0;
        segment.endSpeed = 0.0;
        segment.peakSpeed = 0.0;
        segment.accelTimeSec = 0.0;
        segment.cruiseTimeSec = 0.0;
        segment.decelTimeSec = 0.0;
        segment.accelDistance = 0.0;
        segment.cruiseDistance = 0.0;
        memset(&segment.timeProfile, 0, sizeof(segment.timeProfile));
    }

    double time = 0.0;
    for (uint16_t edgeIndex = 0; edgeIndex < trajectory.edgeCount; ++edgeIndex) {
        MotionTrajectoryEdge& edge = trajectory.edges[edgeIndex];
        edge.startTimeSec = time;
        if (edge.startNodeIndex < trajectory.count) {
            MotionTrajectoryNode& startNode = trajectory.nodes[edge.startNodeIndex];
            startNode.timeSec = time;
            startNode.pathS = edge.startS;
            startNode.speed = edge.startSpeed;
        }

        if (edge.pathSegmentIndex < program->count) {
            MotionSegment& segment = program->segments[edge.pathSegmentIndex];
            if (segment.durationSec <= 1.0e-12) {
                segment.startTimeSec = edge.startTimeSec;
                segment.startSpeed = edge.startSpeed;
            }
            segment.durationSec += edge.durationSec;
            segment.endSpeed = edge.endSpeed;
            if (edge.peakSpeed > segment.peakSpeed) segment.peakSpeed = edge.peakSpeed;
            if ((edge.capReasonMask & static_cast<uint32_t>(MotionCapJerk)) != 0) {
                segment.capReasonMask |= static_cast<uint32_t>(MotionCapJerk);
                segment.activeAccelerationLimitMask |= static_cast<uint32_t>(MotionCapJerk);
                recordMotionAccelerationCap(&segment, edge.jerk, MotionCapJerk);
            }
        }
        time += edge.durationSec;
    }

    if (trajectory.count > 0) {
        const uint16_t last = static_cast<uint16_t>(trajectory.count - 1);
        MotionTrajectoryNode& lastNode = trajectory.nodes[last];
        lastNode.timeSec = time;
        if (lastNode.pathNodeIndex < program->pathGrid.count) {
            const MotionPathNode& pathNode = program->pathGrid.nodes[lastNode.pathNodeIndex];
            lastNode.pathS = pathNode.s;
            lastNode.speed = pathNode.solvedSpeed;
        }
    }
    trajectory.totalDurationSec = time;
    program->totalDurationSec = time;
}

ROBOT_MOTION_CORE_COLD inline RejectCode solveMotionPathGridTrajectory(MotionSegmentProgram* program) {
    if (!program) return RejectCode::BadTarget;
    MotionPathGrid& grid = program->pathGrid;
    MotionTrajectory& trajectory = program->trajectory;
    memset(&trajectory, 0, sizeof(trajectory));
    if (grid.count == 0) return RejectCode::Ok;
    trajectory.count = grid.count;
    trajectory.edgeCount = grid.count > 0 ? static_cast<uint16_t>(grid.count - 1) : 0;
    for (uint16_t node = 0; node < grid.count; ++node) {
        grid.nodes[node].solvedSpeed = grid.nodes[node].speedLimit;
    }
    if (!program->continuesBeforeStart) {
        grid.nodes[0].solvedSpeed = 0.0;
    }
    if (!program->continuesAfterEnd) {
        grid.nodes[grid.count - 1].solvedSpeed = 0.0;
    }

    for (uint8_t iteration = 0; iteration < 10; ++iteration) {
        bool changed = false;
        for (uint16_t edge = 0; edge + 1 < grid.count; ++edge) {
            const MotionPathNode& a = grid.nodes[edge];
            const MotionPathNode& b = grid.nodes[edge + 1];
            const uint16_t pathSegmentIndex = a.segmentIndex != b.segmentIndex ? b.segmentIndex : a.segmentIndex;
            if (pathSegmentIndex >= program->count) return RejectCode::BadTarget;
            MotionSegment& segment = program->segments[pathSegmentIndex];
            const double edgeLength = b.s > a.s ? b.s - a.s : 0.0;
            const double accel = a.accelLimit < b.accelLimit ? a.accelLimit : b.accelLimit;
            const double jerk = a.jerkLimit < b.jerkLimit ? a.jerkLimit : b.jerkLimit;
            const double edgeSpeedLimit = grid.edgeSpeedLimit[edge] > 1.0e-9 && grid.edgeSpeedLimit[edge] < segment.nominalSpeed
                ? grid.edgeSpeedLimit[edge]
                : segment.nominalSpeed;
            const double reachable = jerkLimitedReachableSpeed(grid.nodes[edge].solvedSpeed,
                                                               edgeLength,
                                                               edgeSpeedLimit,
                                                               accel,
                                                               jerk);
            double limit = grid.nodes[edge + 1].speedLimit;
            if (edgeSpeedLimit > 1.0e-9 && edgeSpeedLimit < limit) limit = edgeSpeedLimit;
            if (b.finePoint) limit = 0.0;
            if (reachable < limit) limit = reachable;
            if (limit < grid.nodes[edge + 1].solvedSpeed - 1.0e-9) {
                grid.nodes[edge + 1].solvedSpeed = limit;
                markMotionContext(&segment, MotionCapLookahead);
                changed = true;
            }
        }
        for (uint16_t edge = grid.count - 1; edge-- > 0;) {
            const MotionPathNode& a = grid.nodes[edge];
            const MotionPathNode& b = grid.nodes[edge + 1];
            const uint16_t pathSegmentIndex = a.segmentIndex != b.segmentIndex ? b.segmentIndex : a.segmentIndex;
            if (pathSegmentIndex >= program->count) return RejectCode::BadTarget;
            MotionSegment& segment = program->segments[pathSegmentIndex];
            const double edgeLength = b.s > a.s ? b.s - a.s : 0.0;
            const double accel = a.accelLimit < b.accelLimit ? a.accelLimit : b.accelLimit;
            const double jerk = a.jerkLimit < b.jerkLimit ? a.jerkLimit : b.jerkLimit;
            const double edgeSpeedLimit = grid.edgeSpeedLimit[edge] > 1.0e-9 && grid.edgeSpeedLimit[edge] < segment.nominalSpeed
                ? grid.edgeSpeedLimit[edge]
                : segment.nominalSpeed;
            const double reachable = jerkLimitedReachableSpeed(grid.nodes[edge + 1].solvedSpeed,
                                                               edgeLength,
                                                               edgeSpeedLimit,
                                                               accel,
                                                               jerk);
            double limit = grid.nodes[edge].speedLimit;
            if (edgeSpeedLimit > 1.0e-9 && edgeSpeedLimit < limit) limit = edgeSpeedLimit;
            if (edge > 0 && grid.nodes[edge].finePoint) limit = 0.0;
            if (reachable < limit) limit = reachable;
            if (limit < grid.nodes[edge].solvedSpeed - 1.0e-9) {
                grid.nodes[edge].solvedSpeed = limit;
                markMotionContext(&segment, MotionCapLookahead);
                changed = true;
            }
        }
        if (!program->continuesBeforeStart) {
            grid.nodes[0].solvedSpeed = 0.0;
        }
        if (!program->continuesAfterEnd) {
            grid.nodes[grid.count - 1].solvedSpeed = 0.0;
        }
        if (!changed) break;
    }

    double time = 0.0;
    for (uint8_t i = 0; i < program->count; ++i) {
        MotionSegment& segment = program->segments[i];
        segment.startTimeSec = 0.0;
        segment.durationSec = 0.0;
        segment.startSpeed = 0.0;
        segment.endSpeed = 0.0;
        segment.peakSpeed = 0.0;
        segment.accelTimeSec = 0.0;
        segment.cruiseTimeSec = 0.0;
        segment.decelTimeSec = 0.0;
        segment.accelDistance = 0.0;
        segment.cruiseDistance = 0.0;
        memset(&segment.timeProfile, 0, sizeof(segment.timeProfile));
    }
    for (uint16_t edge = 0; edge + 1 < grid.count; ++edge) {
        const MotionPathNode& a = grid.nodes[edge];
        const MotionPathNode& b = grid.nodes[edge + 1];
        const uint16_t pathSegmentIndex = a.segmentIndex != b.segmentIndex ? b.segmentIndex : a.segmentIndex;
        if (pathSegmentIndex >= program->count) return RejectCode::BadTarget;
        MotionSegment& segment = program->segments[pathSegmentIndex];
        const double edgeLength = b.s > a.s ? b.s - a.s : 0.0;

        MotionTrajectoryEdge& trajectoryEdge = trajectory.edges[edge];
        memset(&trajectoryEdge, 0, sizeof(trajectoryEdge));
        trajectoryEdge.startNodeIndex = edge;
        trajectoryEdge.endNodeIndex = static_cast<uint16_t>(edge + 1);
        trajectoryEdge.pathSegmentIndex = pathSegmentIndex;
        trajectoryEdge.startS = a.s;
        trajectoryEdge.endS = b.s;
        trajectoryEdge.length = edgeLength;
        trajectoryEdge.speedLimit = grid.edgeSpeedLimit[edge] > 1.0e-9 && grid.edgeSpeedLimit[edge] < segment.nominalSpeed
            ? grid.edgeSpeedLimit[edge]
            : segment.nominalSpeed;
        trajectoryEdge.acceleration = a.accelLimit < b.accelLimit ? a.accelLimit : b.accelLimit;
        trajectoryEdge.jerk = a.jerkLimit < b.jerkLimit ? a.jerkLimit : b.jerkLimit;
        trajectoryEdge.commandId = b.commandId >= 0 ? b.commandId : segment.commandId;
        trajectoryEdge.completedCommandId = b.completedCommandId;
        trajectoryEdge.capReasonMask = segment.capReasonMask | a.capReasonMask | b.capReasonMask | grid.edgeCapReasonMask[edge];
        trajectoryEdge.finePoint = b.finePoint;
        trajectoryEdge.blended = segment.blended;

        segment.capReasonMask |= a.capReasonMask | b.capReasonMask | grid.edgeCapReasonMask[edge];
        if (grid.edgeCapReasonMask[edge] & (static_cast<uint32_t>(MotionCapJointVelocity) |
                                            static_cast<uint32_t>(MotionCapPathAcceleration) |
                                            static_cast<uint32_t>(MotionCapSingularity) |
                                            static_cast<uint32_t>(MotionCapStepRate))) {
            segment.activeSpeedLimitMask |= grid.edgeCapReasonMask[edge] &
                (static_cast<uint32_t>(MotionCapJointVelocity) |
                 static_cast<uint32_t>(MotionCapPathAcceleration) |
                 static_cast<uint32_t>(MotionCapSingularity) |
                 static_cast<uint32_t>(MotionCapStepRate));
        }
        if (grid.edgeCapReasonMask[edge] & (static_cast<uint32_t>(MotionCapJointAcceleration) |
                                            static_cast<uint32_t>(MotionCapJerk))) {
            segment.activeAccelerationLimitMask |= grid.edgeCapReasonMask[edge] &
                (static_cast<uint32_t>(MotionCapJointAcceleration) |
                 static_cast<uint32_t>(MotionCapJerk));
        }
        if (a.solvedSpeed < segment.nominalSpeed - 1.0e-9 ||
            b.solvedSpeed < segment.nominalSpeed - 1.0e-9) {
            markMotionContext(&segment, MotionCapLookahead);
            trajectoryEdge.capReasonMask |= static_cast<uint32_t>(MotionCapLookahead);
        }
        profileMotionTrajectoryEdge(&trajectoryEdge, segment, a.solvedSpeed, b.solvedSpeed, time);
        trajectory.nodes[edge].timeSec = time;
        trajectory.nodes[edge].pathS = a.s;
        trajectory.nodes[edge].speed = a.solvedSpeed;
        trajectory.nodes[edge].acceleration = 0.0;
        trajectory.nodes[edge].jerk = 0.0;
        trajectory.nodes[edge].pathNodeIndex = edge;
        if (segment.durationSec <= 1.0e-12) {
            segment.startTimeSec = trajectoryEdge.startTimeSec;
            segment.startSpeed = trajectoryEdge.startSpeed;
        }
        segment.durationSec += trajectoryEdge.durationSec;
        segment.endSpeed = trajectoryEdge.endSpeed;
        if (trajectoryEdge.peakSpeed > segment.peakSpeed) segment.peakSpeed = trajectoryEdge.peakSpeed;
        time += trajectoryEdge.durationSec;
    }
    const uint16_t last = grid.count - 1;
    trajectory.nodes[last].timeSec = time;
    trajectory.nodes[last].pathS = grid.nodes[last].s;
    trajectory.nodes[last].speed = grid.nodes[last].solvedSpeed;
    trajectory.nodes[last].acceleration = 0.0;
    trajectory.nodes[last].jerk = 0.0;
    trajectory.nodes[last].pathNodeIndex = last;
    trajectory.totalDurationSec = time;
    program->totalDurationSec = time;
    program->polynomialEdgeCount = 0;

    if (trajectory.count > 1) {
        estimateMotionTrajectoryNodeAccelerations(grid, &trajectory);
        for (uint16_t edge = 0; edge < trajectory.edgeCount; ++edge) {
            MotionTrajectoryEdge& trajectoryEdge = trajectory.edges[edge];
            trajectoryEdge.startAcceleration = trajectory.nodes[trajectoryEdge.startNodeIndex].acceleration;
            trajectoryEdge.endAcceleration = trajectory.nodes[trajectoryEdge.endNodeIndex].acceleration;
            trajectoryEdge.startJerk = trajectory.nodes[trajectoryEdge.startNodeIndex].jerk;
            trajectoryEdge.endJerk = trajectory.nodes[trajectoryEdge.endNodeIndex].jerk;
            if (!fitMotionTrajectoryEdgePolynomial(&trajectoryEdge)) {
                return RejectCode::BadDuration;
            }
            trajectoryEdge.peakSpeed = trajectoryEdge.startSpeed > trajectoryEdge.endSpeed
                ? trajectoryEdge.startSpeed
                : trajectoryEdge.endSpeed;
            for (uint8_t sample = 1; sample < 16; ++sample) {
                const double sampleTime = trajectoryEdge.durationSec * (static_cast<double>(sample) / 16.0);
                const double sampleSpeed = motionTrajectoryEdgeSpeedAtTime(trajectoryEdge, sampleTime);
                if (sampleSpeed > trajectoryEdge.peakSpeed) trajectoryEdge.peakSpeed = sampleSpeed;
            }
        }
        recomputeMotionTrajectoryTiming(program);
        program->polynomialEdgeCount = 0;
        for (uint16_t edge = 0; edge < trajectory.edgeCount; ++edge) {
            const MotionTrajectoryEdge& trajectoryEdge = trajectory.edges[edge];
            if (trajectoryEdge.length <= 1.0e-12 || trajectoryEdge.durationSec <= 1.0e-9) continue;
            if (trajectoryEdge.useQuintic != 0) {
                ++program->polynomialEdgeCount;
            } else {
                return RejectCode::BadDuration;
            }
        }
    }
    return RejectCode::Ok;
}

inline double motionTrajectoryEdgeJointVelocity(const MotionPathGrid& grid,
                                                const MotionTrajectory& trajectory,
                                                uint16_t edge,
                                                uint8_t joint) {
    if (edge >= trajectory.edgeCount || joint >= kAxisCount) return 0.0;
    const MotionTrajectoryEdge& trajectoryEdge = trajectory.edges[edge];
    if (trajectoryEdge.durationSec <= 1.0e-9 ||
        trajectoryEdge.startNodeIndex >= grid.count ||
        trajectoryEdge.endNodeIndex >= grid.count) {
        return 0.0;
    }
    const MotionPathNode& a = grid.nodes[trajectoryEdge.startNodeIndex];
    const MotionPathNode& b = grid.nodes[trajectoryEdge.endNodeIndex];
    return wrapRadians(b.q[joint] - a.q[joint]) / trajectoryEdge.durationSec;
}

inline double motionTrajectoryEdgeAngularVelocity(const MotionPathGrid& grid,
                                                  const MotionTrajectory& trajectory,
                                                  uint16_t edge) {
    if (edge >= trajectory.edgeCount) return 0.0;
    const MotionTrajectoryEdge& trajectoryEdge = trajectory.edges[edge];
    if (trajectoryEdge.durationSec <= 1.0e-9 ||
        trajectoryEdge.startNodeIndex >= grid.count ||
        trajectoryEdge.endNodeIndex >= grid.count) {
        return 0.0;
    }
    const MotionPathNode& a = grid.nodes[trajectoryEdge.startNodeIndex];
    const MotionPathNode& b = grid.nodes[trajectoryEdge.endNodeIndex];
    return orientationDeltaAngleRad(a.tcp, b.tcp) / trajectoryEdge.durationSec;
}

inline bool capMotionPathEdgeSpeed(MotionPathGrid* grid,
                                   uint16_t edge,
                                   double cap,
                                   uint32_t reason) {
    if (!grid || edge + 1 >= grid->count || !isFinite(cap) || cap <= 1.0e-9) return false;
    double& current = grid->edgeSpeedLimit[edge];
    if (current <= 1.0e-9 || cap < current - 1.0e-9) {
        current = cap;
        grid->edgeCapReasonMask[edge] |= reason;
        return true;
    }
    return false;
}

inline bool scaleMotionPathEdgeSpeed(MotionPathGrid* grid,
                                     uint16_t edge,
                                     double scale,
                                     uint32_t reason) {
    if (!grid || edge + 1 >= grid->count || !isFinite(scale) || scale <= 1.0e-9) return false;
    if (scale >= 1.0) return false;
    const double current = grid->edgeSpeedLimit[edge] > 1.0e-9 ? grid->edgeSpeedLimit[edge] : 1.0;
    return capMotionPathEdgeSpeed(grid, edge, current * scale, reason);
}

ROBOT_MOTION_CORE_COLD inline bool enforceSolvedTrajectoryDynamicLimits(const RobotModel& model,
                                                                        const MotionProgramSettings& settings,
                                                                        MotionSegmentProgram* program) {
    (void)model;
    if (!program || program->pathGrid.count < 2 || program->trajectory.edgeCount == 0) return false;
    MotionPathGrid& grid = program->pathGrid;
    const MotionTrajectory& trajectory = program->trajectory;
    bool changed = false;
    constexpr double kTightenSafety = 0.92;

    for (uint16_t edge = 0; edge < trajectory.edgeCount; ++edge) {
        const MotionTrajectoryEdge& trajectoryEdge = trajectory.edges[edge];
        if (trajectoryEdge.durationSec <= 1.0e-9) continue;
        double peakPathSpeed = 0.0;
        double peakPathAcceleration = 0.0;
        double peakPathJerk = 0.0;
        motionTrajectoryEdgeDerivativeBounds(trajectoryEdge, &peakPathSpeed, &peakPathAcceleration, &peakPathJerk);
        const uint16_t pathSegmentIndex = trajectoryEdge.pathSegmentIndex;
        const MotionSegment* pathSegment = pathSegmentIndex < program->count ? &program->segments[pathSegmentIndex] : nullptr;
        const double invLength = trajectoryEdge.length > 1.0e-12 ? 1.0 / trajectoryEdge.length : 0.0;
        for (uint8_t joint = 0; joint < kAxisCount; ++joint) {
            double dqDs = 0.0;
            double d2qDs2 = 0.0;
            double d3qDs3 = 0.0;
            if (pathSegment && pathSegment->kind == MotionSegmentKind::MoveJ) {
                if (invLength > 0.0 &&
                    trajectoryEdge.startNodeIndex < grid.count &&
                    trajectoryEdge.endNodeIndex < grid.count) {
                    const MotionPathNode& a = grid.nodes[trajectoryEdge.startNodeIndex];
                    const MotionPathNode& b = grid.nodes[trajectoryEdge.endNodeIndex];
                    dqDs = absDouble(b.q[joint] - a.q[joint]) * invLength;
                }
                d2qDs2 = 0.0;
                d3qDs3 = 0.0;
            } else if (motionPathGridJointDerivativeBoundsOnEdge(grid,
                                                                 edge,
                                                                 joint,
                                                                 &dqDs,
                                                                 &d2qDs2,
                                                                 &d3qDs3)) {
                d3qDs3 = 0.0;
            } else if (invLength > 0.0 &&
                       trajectoryEdge.startNodeIndex < grid.count &&
                       trajectoryEdge.endNodeIndex < grid.count) {
                const MotionPathNode& a = grid.nodes[trajectoryEdge.startNodeIndex];
                const MotionPathNode& b = grid.nodes[trajectoryEdge.endNodeIndex];
                dqDs = absDouble(wrapRadians(b.q[joint] - a.q[joint])) * invLength;
            }
            const double v = dqDs * peakPathSpeed;
            const double vLimit = finitePositiveOrZero(settings.jointVelocityLimitRadSec[joint]);
            if (vLimit > 0.0 && v > vLimit * 1.002) {
                changed |= capMotionPathEdgeSpeed(&grid,
                                                  edge,
                                                  grid.edgeSpeedLimit[edge] * (vLimit / v) * kTightenSafety,
                                                  MotionCapJointVelocity);
            }
            const double stepRateLimit = finitePositiveOrZero(settings.jointStepRateLimitStepsSec[joint]);
            const double stepsPerDegree = finitePositiveOrZero(settings.jointStepsPerDegree[joint]);
            if (stepRateLimit > 0.0 && stepsPerDegree > 0.0) {
                const double stepsPerSec = radiansToDegrees(v) * stepsPerDegree;
                if (stepsPerSec > stepRateLimit * 1.002) {
                    changed |= capMotionPathEdgeSpeed(&grid,
                                                      edge,
                                                      grid.edgeSpeedLimit[edge] * (stepRateLimit / stepsPerSec) * kTightenSafety,
                                                      MotionCapStepRate);
                }
            }
            const double aLimit = finitePositiveOrZero(settings.jointAccelerationLimitRadSec2[joint]);
            const double jointAcceleration = d2qDs2 * peakPathSpeed * peakPathSpeed +
                dqDs * peakPathAcceleration;
            if (aLimit > 0.0 && jointAcceleration > aLimit * 1.002) {
                const double scale = sqrt(aLimit / jointAcceleration) * kTightenSafety;
                changed |= scaleMotionPathEdgeSpeed(&grid, edge, scale, MotionCapJointAcceleration);
            }
            const double jLimit = finitePositiveOrZero(settings.defaultJointJerkRadSec3);
            const double jointJerk = d3qDs3 * peakPathSpeed * peakPathSpeed * peakPathSpeed +
                3.0 * d2qDs2 * peakPathSpeed * peakPathAcceleration +
                dqDs * peakPathJerk;
            if (jLimit > 0.0 && jointJerk > jLimit * 1.002) {
                const double scale = pow(jLimit / jointJerk, 1.0 / 3.0) * kTightenSafety;
                changed |= scaleMotionPathEdgeSpeed(&grid, edge, scale, MotionCapJerk);
            }
        }

        if (pathSegment && pathSegment->kind != MotionSegmentKind::MoveJ) {
            const double commandedSpeed = trajectoryEdge.speedLimit > 1.0e-9 ? trajectoryEdge.speedLimit : pathSegment->nominalSpeed;
            if (commandedSpeed > 0.0 && peakPathSpeed > commandedSpeed * 1.002) {
                changed |= capMotionPathEdgeSpeed(&grid,
                                                  edge,
                                                  grid.edgeSpeedLimit[edge] * (commandedSpeed / peakPathSpeed) * kTightenSafety,
                                                  MotionCapCommandSpeed);
            }
            const double linearAccelLimit = finitePositiveOrZero(settings.defaultLinearAccelerationMmSec2);
            if (linearAccelLimit > 0.0) {
                if (peakPathAcceleration > linearAccelLimit * 1.002) {
                    const double scale = sqrt(linearAccelLimit / peakPathAcceleration) * kTightenSafety;
                    changed |= scaleMotionPathEdgeSpeed(&grid, edge, scale, MotionCapPathAcceleration);
                }
                const double curvature = pathSegment->curvature > 0.0 ? pathSegment->curvature : 0.0;
                const double centripetal = curvature * peakPathSpeed * peakPathSpeed;
                if (centripetal > linearAccelLimit * 1.002) {
                    const double scale = sqrt(linearAccelLimit / centripetal) * kTightenSafety;
                    changed |= scaleMotionPathEdgeSpeed(&grid, edge, scale, MotionCapPathAcceleration);
                }
            }
            const double linearJerkLimit = finitePositiveOrZero(settings.defaultLinearJerkMmSec3);
            if (linearJerkLimit > 0.0 && peakPathJerk > linearJerkLimit * 1.002) {
                const double scale = pow(linearJerkLimit / peakPathJerk, 1.0 / 3.0) * kTightenSafety;
                changed |= scaleMotionPathEdgeSpeed(&grid, edge, scale, MotionCapJerk);
            }
        }

        double orientationRatio = 0.0;
        if (trajectoryEdge.startNodeIndex < grid.count && trajectoryEdge.endNodeIndex < grid.count) {
            const MotionPathNode& a = grid.nodes[trajectoryEdge.startNodeIndex];
            const MotionPathNode& b = grid.nodes[trajectoryEdge.endNodeIndex];
            orientationRatio = a.orientationRatioRadPerPathUnit > b.orientationRatioRadPerPathUnit
                ? a.orientationRatioRadPerPathUnit
                : b.orientationRatioRadPerPathUnit;
        }
        const double angularVelocity = orientationRatio * peakPathSpeed;
        const double angularLimit = finitePositiveOrZero(settings.defaultToolAngularSpeedRadSec);
        if (angularLimit > 0.0 && angularVelocity > angularLimit * 1.002) {
            changed |= capMotionPathEdgeSpeed(&grid,
                                              edge,
                                              grid.edgeSpeedLimit[edge] * (angularLimit / angularVelocity) * kTightenSafety,
                                              MotionCapJointVelocity);
        }
        const double angularAccelLimit = finitePositiveOrZero(settings.defaultToolAngularAccelerationRadSec2);
        const double angularAcceleration = orientationRatio * peakPathAcceleration;
        if (angularAccelLimit > 0.0 && angularAcceleration > angularAccelLimit * 1.002) {
            const double scale = sqrt(angularAccelLimit / angularAcceleration) * kTightenSafety;
            changed |= scaleMotionPathEdgeSpeed(&grid, edge, scale, MotionCapJointAcceleration);
        }
        const double angularJerkLimit = finitePositiveOrZero(settings.defaultToolAngularJerkRadSec3);
        const double angularJerk = orientationRatio * peakPathJerk;
        if (angularJerkLimit > 0.0 && angularJerk > angularJerkLimit * 1.002) {
            const double scale = pow(angularJerkLimit / angularJerk, 1.0 / 3.0) * kTightenSafety;
            changed |= scaleMotionPathEdgeSpeed(&grid, edge, scale, MotionCapJerk);
        }
    }

    for (uint16_t node = 1; node < trajectory.edgeCount; ++node) {
        const MotionTrajectoryEdge& prevEdge = trajectory.edges[node - 1];
        const MotionTrajectoryEdge& nextEdge = trajectory.edges[node];
        const double dt = 0.5 * (prevEdge.durationSec + nextEdge.durationSec);
        if (dt <= 1.0e-9) continue;
        for (uint8_t joint = 0; joint < kAxisCount; ++joint) {
            const double aLimit = finitePositiveOrZero(settings.jointAccelerationLimitRadSec2[joint]);
            if (aLimit <= 0.0) continue;
            const double v0 = motionTrajectoryEdgeJointVelocity(grid, trajectory, static_cast<uint16_t>(node - 1), joint);
            const double v1 = motionTrajectoryEdgeJointVelocity(grid, trajectory, node, joint);
            const double accel = absDouble(v1 - v0) / dt;
            if (accel > aLimit * 1.002) {
                const double scale = sqrt(aLimit / accel) * kTightenSafety;
                changed |= capMotionPathEdgeSpeed(&grid, static_cast<uint16_t>(node - 1), grid.edgeSpeedLimit[node - 1] * scale, MotionCapJointAcceleration);
                changed |= capMotionPathEdgeSpeed(&grid, node, grid.edgeSpeedLimit[node] * scale, MotionCapJointAcceleration);
            }
        }

        const double angularAccelLimit = finitePositiveOrZero(settings.defaultToolAngularAccelerationRadSec2);
        if (angularAccelLimit > 0.0) {
            const double w0 = motionTrajectoryEdgeAngularVelocity(grid, trajectory, static_cast<uint16_t>(node - 1));
            const double w1 = motionTrajectoryEdgeAngularVelocity(grid, trajectory, node);
            const double accel = absDouble(w1 - w0) / dt;
            if (accel > angularAccelLimit * 1.002) {
                const double scale = sqrt(angularAccelLimit / accel) * kTightenSafety;
                changed |= capMotionPathEdgeSpeed(&grid, static_cast<uint16_t>(node - 1), grid.edgeSpeedLimit[node - 1] * scale, MotionCapJointAcceleration);
                changed |= capMotionPathEdgeSpeed(&grid, node, grid.edgeSpeedLimit[node] * scale, MotionCapJointAcceleration);
            }
        }
    }

    for (uint16_t node = 1; node + 1 < trajectory.edgeCount; ++node) {
        const MotionTrajectoryEdge& aEdge = trajectory.edges[node - 1];
        const MotionTrajectoryEdge& bEdge = trajectory.edges[node];
        const MotionTrajectoryEdge& cEdge = trajectory.edges[node + 1];
        const double dt0 = 0.5 * (aEdge.durationSec + bEdge.durationSec);
        const double dt1 = 0.5 * (bEdge.durationSec + cEdge.durationSec);
        const double jerkDt = 0.5 * (dt0 + dt1);
        if (dt0 <= 1.0e-9 || dt1 <= 1.0e-9 || jerkDt <= 1.0e-9) continue;
        for (uint8_t joint = 0; joint < kAxisCount; ++joint) {
            const double jLimit = finitePositiveOrZero(settings.defaultJointJerkRadSec3);
            if (jLimit <= 0.0) continue;
            const double v0 = motionTrajectoryEdgeJointVelocity(grid, trajectory, static_cast<uint16_t>(node - 1), joint);
            const double v1 = motionTrajectoryEdgeJointVelocity(grid, trajectory, node, joint);
            const double v2 = motionTrajectoryEdgeJointVelocity(grid, trajectory, static_cast<uint16_t>(node + 1), joint);
            const double accel0 = (v1 - v0) / dt0;
            const double accel1 = (v2 - v1) / dt1;
            const double jerk = absDouble(accel1 - accel0) / jerkDt;
            if (jerk > jLimit * 1.002) {
                const double scale = pow(jLimit / jerk, 1.0 / 3.0) * kTightenSafety;
                changed |= capMotionPathEdgeSpeed(&grid, static_cast<uint16_t>(node - 1), grid.edgeSpeedLimit[node - 1] * scale, MotionCapJerk);
                changed |= capMotionPathEdgeSpeed(&grid, node, grid.edgeSpeedLimit[node] * scale, MotionCapJerk);
                changed |= capMotionPathEdgeSpeed(&grid, static_cast<uint16_t>(node + 1), grid.edgeSpeedLimit[node + 1] * scale, MotionCapJerk);
            }
        }

        const double angularJerkLimit = finitePositiveOrZero(settings.defaultToolAngularJerkRadSec3);
        if (angularJerkLimit > 0.0) {
            const double w0 = motionTrajectoryEdgeAngularVelocity(grid, trajectory, static_cast<uint16_t>(node - 1));
            const double w1 = motionTrajectoryEdgeAngularVelocity(grid, trajectory, node);
            const double w2 = motionTrajectoryEdgeAngularVelocity(grid, trajectory, static_cast<uint16_t>(node + 1));
            const double accel0 = (w1 - w0) / dt0;
            const double accel1 = (w2 - w1) / dt1;
            const double jerk = absDouble(accel1 - accel0) / jerkDt;
            if (jerk > angularJerkLimit * 1.002) {
                const double scale = pow(angularJerkLimit / jerk, 1.0 / 3.0) * kTightenSafety;
                changed |= capMotionPathEdgeSpeed(&grid, static_cast<uint16_t>(node - 1), grid.edgeSpeedLimit[node - 1] * scale, MotionCapJerk);
                changed |= capMotionPathEdgeSpeed(&grid, node, grid.edgeSpeedLimit[node] * scale, MotionCapJerk);
                changed |= capMotionPathEdgeSpeed(&grid, static_cast<uint16_t>(node + 1), grid.edgeSpeedLimit[node + 1] * scale, MotionCapJerk);
            }
        }
    }

    return changed;
}

inline void beginMotionSegmentSampler(const RobotModel& model,
                                      const double startQ[kAxisCount],
                                      const MotionSegmentProgram* program,
                                      MotionSegmentSampler* sampler);
inline RejectCode sampleMotionSegmentProgram(MotionSegmentSampler* sampler,
                                             double timeSec,
                                             PathSample* sample);

ROBOT_MOTION_CORE_COLD inline RejectCode verifySampledTrajectoryDynamicLimits(const RobotModel& model,
                                                                              const MotionProgramSettings& settings,
                                                                              const MotionSegmentProgram& program,
                                                                              MotionVerificationFailure* failure = nullptr) {
    if (failure) memset(failure, 0, sizeof(*failure));
    auto fail = [&](MotionVerificationKind kind,
                    const PathSample& sample,
                    double observed,
                    double limit,
                    uint8_t joint) {
        if (failure && failure->kind == MotionVerificationKind::None) {
            failure->kind = kind;
            failure->timeSec = sample.timeSec;
            failure->observed = observed;
            failure->limit = limit;
            failure->trajectoryEdgeIndex = sample.trajectoryEdgeIndex;
            failure->pathSegmentIndex = sample.pathSegmentIndex;
            failure->joint = joint;
        }
        return RejectCode::BadSpeed;
    };
    const MotionTrajectory& trajectory = program.trajectory;
    if (trajectory.edgeCount == 0 || trajectory.totalDurationSec <= 1.0e-9) return RejectCode::Ok;

    const double verifyControlPeriod = settings.controlPeriodSec > 1.0e-6 ? settings.controlPeriodSec : 0.005;
    double period = verifyControlPeriod * 0.5;
    if (period < 0.001) period = 0.001;
    MotionSegmentSampler sampler = {};
    beginMotionSegmentSampler(model, program.segments[0].startQ, &program, &sampler);

    PathSample last = {};
    PathSample current = {};
    RejectCode result = sampleMotionSegmentProgram(&sampler, 0.0, &last);
    if (result != RejectCode::Ok) return result;

    double lastVelocity[kAxisCount] = {};
    double lastAcceleration[kAxisCount] = {};
    double lastLinearVelocity[3] = {};
    double lastLinearAcceleration[3] = {};
    double lastAngularVelocity[3] = {};
    double lastAngularAcceleration[3] = {};
    double lastVelocityTime = 0.0;
    double lastAccelerationTime = 0.0;
    double lastTcpVelocityTime = 0.0;
    double lastTcpAccelerationTime = 0.0;
    bool haveVelocity = false;
    bool haveAcceleration = false;
    bool haveTcpVelocity = false;
    bool haveTcpAcceleration = false;

    constexpr double kVerifierTolerance = 1.005;
    constexpr uint32_t kMaxVerifierSamples = 200000;
    for (uint32_t sampleIndex = 1; sampleIndex < kMaxVerifierSamples; ++sampleIndex) {
        double sampleTime = static_cast<double>(sampleIndex) * period;
        if (sampleTime > trajectory.totalDurationSec) sampleTime = trajectory.totalDurationSec;
        result = sampleMotionSegmentProgram(&sampler, sampleTime, &current);
        if (result != RejectCode::Ok) return result;

        const double dt = current.timeSec - last.timeSec;
        const bool sameTrajectoryEdge = current.trajectoryEdgeIndex == last.trajectoryEdgeIndex;
        if (!sameTrajectoryEdge) {
            haveVelocity = false;
            haveAcceleration = false;
            haveTcpVelocity = false;
            haveTcpAcceleration = false;
        }
        if (dt > 1.0e-9) {
            double velocity[kAxisCount] = {};
            for (uint8_t joint = 0; joint < kAxisCount; ++joint) {
                velocity[joint] = wrapRadians(current.q[joint] - last.q[joint]) / dt;
                const double vLimit = finitePositiveOrZero(settings.jointVelocityLimitRadSec[joint]);
                const double jointVelocity = absDouble(velocity[joint]);
                if (vLimit > 0.0 && jointVelocity > vLimit * kVerifierTolerance) {
                    return fail(MotionVerificationKind::JointVelocity, current, jointVelocity, vLimit, joint);
                }
                const double stepRateLimit = finitePositiveOrZero(settings.jointStepRateLimitStepsSec[joint]);
                const double stepsPerDegree = finitePositiveOrZero(settings.jointStepsPerDegree[joint]);
                if (stepRateLimit > 0.0 && stepsPerDegree > 0.0) {
                    const double stepsPerSec = radiansToDegrees(jointVelocity) * stepsPerDegree;
                    if (stepsPerSec > stepRateLimit * kVerifierTolerance) {
                        return fail(MotionVerificationKind::JointStepRate, current, stepsPerSec, stepRateLimit, joint);
                    }
                }
            }

            if (current.kind != PathSampleKind::MoveJ && last.kind != PathSampleKind::MoveJ) {
                double currentPoint[3] = {};
                double lastPoint[3] = {};
                double deltaPoint[3] = {};
                translationVector(current.tcp, currentPoint);
                translationVector(last.tcp, lastPoint);
                subtract3(currentPoint, lastPoint, deltaPoint);
                double linearVelocity[3] = {
                    deltaPoint[0] / dt,
                    deltaPoint[1] / dt,
                    deltaPoint[2] / dt
                };
                double rotationDelta[3] = {};
                orientationDeltaVectorRad(last.tcp, current.tcp, rotationDelta);
                double angularVelocity[3] = {
                    rotationDelta[0] / dt,
                    rotationDelta[1] / dt,
                    rotationDelta[2] / dt
                };
                const double tcpSpeed = length3(linearVelocity);
                double commandedSpeed = current.trajectoryEdgeIndex < trajectory.edgeCount
                    ? trajectory.edges[current.trajectoryEdgeIndex].speedLimit
                    : 0.0;
                if (current.profileSpeed > commandedSpeed) commandedSpeed = current.profileSpeed;
                if (last.profileSpeed > commandedSpeed) commandedSpeed = last.profileSpeed;
                double allowedSpeed = commandedSpeed;
                if (const WeaveParams* weave = motionSegmentWeave(program, current.pathSegmentIndex)) {
                    const double transverse = weaveTransverseSpeedMmPerSec(*weave, commandedSpeed);
                    if (weaveParamsAreUsable(*weave)) {
                        double excursion =
                            weaveAmplitudePeakMm(*weave) * weaveShapeHarmonics(weave->shape).excursion;
                        if (isFinite(weave->biasMm)) excursion += absDouble(weave->biasMm);
                        const double alongTrack =
                            commandedSpeed * (1.0 + excursion * finitePositiveOrZero(current.curvature));
                        allowedSpeed = sqrt(alongTrack * alongTrack + transverse * transverse);
                    }
                }
                const double tcpSpeedTolerance = allowedSpeed * kVerifierTolerance + 0.05;
                if (commandedSpeed > 1.0e-9 && tcpSpeed > tcpSpeedTolerance) {
                    return fail(MotionVerificationKind::TcpSpeed, current, tcpSpeed, allowedSpeed, kAxisCount);
                }
                const double angularSpeedLimit = finitePositiveOrZero(settings.defaultToolAngularSpeedRadSec);
                const double angularSpeed = length3(angularVelocity);
                if (angularSpeedLimit > 0.0 && angularSpeed > angularSpeedLimit * kVerifierTolerance) {
                    return fail(MotionVerificationKind::ToolAngularSpeed, current, angularSpeed, angularSpeedLimit, kAxisCount);
                }

                if (haveTcpVelocity) {
                    const double accelDt = 0.5 * (last.timeSec - lastTcpVelocityTime + dt);
                    if (accelDt > 1.0e-9) {
                        double linearAcceleration[3] = {
                            (linearVelocity[0] - lastLinearVelocity[0]) / accelDt,
                            (linearVelocity[1] - lastLinearVelocity[1]) / accelDt,
                            (linearVelocity[2] - lastLinearVelocity[2]) / accelDt
                        };
                        double angularAcceleration[3] = {
                            (angularVelocity[0] - lastAngularVelocity[0]) / accelDt,
                            (angularVelocity[1] - lastAngularVelocity[1]) / accelDt,
                            (angularVelocity[2] - lastAngularVelocity[2]) / accelDt
                        };
                        const double linearAccelLimit = finitePositiveOrZero(settings.defaultLinearAccelerationMmSec2);
                        const double linearAccel = length3(linearAcceleration);
                        if (linearAccelLimit > 0.0 && linearAccel > linearAccelLimit * kVerifierTolerance) {
                            return fail(MotionVerificationKind::TcpAcceleration, current, linearAccel, linearAccelLimit, kAxisCount);
                        }
                        const double angularAccelLimit = finitePositiveOrZero(settings.defaultToolAngularAccelerationRadSec2);
                        const double angularAccel = length3(angularAcceleration);
                        if (angularAccelLimit > 0.0 && angularAccel > angularAccelLimit * kVerifierTolerance) {
                            return fail(MotionVerificationKind::ToolAngularAcceleration, current, angularAccel, angularAccelLimit, kAxisCount);
                        }
                        if (haveTcpAcceleration) {
                            const double jerkDt = 0.5 * (last.timeSec - lastTcpAccelerationTime + accelDt);
                            if (jerkDt > 1.0e-9) {
                                double linearJerk[3] = {
                                    (linearAcceleration[0] - lastLinearAcceleration[0]) / jerkDt,
                                    (linearAcceleration[1] - lastLinearAcceleration[1]) / jerkDt,
                                    (linearAcceleration[2] - lastLinearAcceleration[2]) / jerkDt
                                };
                                double angularJerk[3] = {
                                    (angularAcceleration[0] - lastAngularAcceleration[0]) / jerkDt,
                                    (angularAcceleration[1] - lastAngularAcceleration[1]) / jerkDt,
                                    (angularAcceleration[2] - lastAngularAcceleration[2]) / jerkDt
                                };
                                const double linearJerkLimit = finitePositiveOrZero(settings.defaultLinearJerkMmSec3);
                                const double linearJerkMag = length3(linearJerk);
                                if (linearJerkLimit > 0.0 && linearJerkMag > linearJerkLimit * kVerifierTolerance) {
                                    return fail(MotionVerificationKind::TcpJerk, current, linearJerkMag, linearJerkLimit, kAxisCount);
                                }
                                const double angularJerkLimit = finitePositiveOrZero(settings.defaultToolAngularJerkRadSec3);
                                const double angularJerkMag = length3(angularJerk);
                                if (angularJerkLimit > 0.0 && angularJerkMag > angularJerkLimit * kVerifierTolerance) {
                                    return fail(MotionVerificationKind::ToolAngularJerk, current, angularJerkMag, angularJerkLimit, kAxisCount);
                                }
                            }
                        }
                        for (uint8_t axis = 0; axis < 3; ++axis) {
                            lastLinearAcceleration[axis] = linearAcceleration[axis];
                            lastAngularAcceleration[axis] = angularAcceleration[axis];
                        }
                        lastTcpAccelerationTime = last.timeSec;
                        haveTcpAcceleration = true;
                    }
                }
                // Recorded only from inside an edge. A velocity measured across a join is a sound
                // velocity and is checked as one above, but differencing the next one against it
                // would report the join's own change of curvature as an acceleration - which is how
                // an ordinary unwoven MoveL path came to read 8707 mm/s^2 against a 1000 limit.
                if (sameTrajectoryEdge) {
                    for (uint8_t axis = 0; axis < 3; ++axis) {
                        lastLinearVelocity[axis] = linearVelocity[axis];
                        lastAngularVelocity[axis] = angularVelocity[axis];
                    }
                    lastTcpVelocityTime = last.timeSec;
                    haveTcpVelocity = true;
                }
            }

            if (haveVelocity) {
                const double accelDt = 0.5 * (last.timeSec - lastVelocityTime + dt);
                if (accelDt > 1.0e-9) {
                    double acceleration[kAxisCount] = {};
                    for (uint8_t joint = 0; joint < kAxisCount; ++joint) {
                        acceleration[joint] = (velocity[joint] - lastVelocity[joint]) / accelDt;
                        const double aLimit = finitePositiveOrZero(settings.jointAccelerationLimitRadSec2[joint]);
                        const double jointAcceleration = absDouble(acceleration[joint]);
                        if (aLimit > 0.0 && jointAcceleration > aLimit * kVerifierTolerance) {
                            return fail(MotionVerificationKind::JointAcceleration, current, jointAcceleration, aLimit, joint);
                        }
                    }
                    if (haveAcceleration) {
                        const double jerkDt = 0.5 * (last.timeSec - lastAccelerationTime + accelDt);
                        if (jerkDt > 1.0e-9) {
                            if (current.kind == PathSampleKind::MoveJ && last.kind == PathSampleKind::MoveJ) {
                                for (uint8_t joint = 0; joint < kAxisCount; ++joint) {
                                    const double jLimit = finitePositiveOrZero(settings.defaultJointJerkRadSec3);
                                    if (jLimit <= 0.0) continue;
                                    const double jerk = absDouble(acceleration[joint] - lastAcceleration[joint]) / jerkDt;
                                    if (jerk > jLimit * kVerifierTolerance) {
                                        return fail(MotionVerificationKind::JointJerk, current, jerk, jLimit, joint);
                                    }
                                }
                            }
                        }
                    }
                    for (uint8_t joint = 0; joint < kAxisCount; ++joint) lastAcceleration[joint] = acceleration[joint];
                    lastAccelerationTime = last.timeSec;
                    haveAcceleration = true;
                }
            }
            if (sameTrajectoryEdge) {
                for (uint8_t joint = 0; joint < kAxisCount; ++joint) lastVelocity[joint] = velocity[joint];
                lastVelocityTime = last.timeSec;
                haveVelocity = true;
            }
        }

        last = current;
        if (sampleTime >= trajectory.totalDurationSec - 1.0e-9) return RejectCode::Ok;
    }
    return RejectCode::TooManyTicks;
}

// True when two consecutive segments meet at a corner the planner did not round off.
inline bool motionSegmentsMeetSharply(const MotionSegment& before, const MotionSegment& after) {
    if (before.kind == MotionSegmentKind::MoveLBlend || after.kind == MotionSegmentKind::MoveLBlend) {
        return false;
    }
    double tangentBefore[3] = {};
    double tangentAfter[3] = {};
    if (!weaveTangentForSegment(before, 1.0, tangentBefore)) return false;
    if (!weaveTangentForSegment(after, 0.0, tangentAfter)) return false;
    const double alignment = tangentBefore[0] * tangentAfter[0] + tangentBefore[1] * tangentAfter[1] +
                             tangentBefore[2] * tangentAfter[2];
    // About five degrees. Below that the wrist can carry the pattern round without the offset having
    // to jump, and breaking the run would taper the bead for no reason.
    constexpr double kSharpTurnAlignment = 0.9962;
    return alignment < kSharpTurnAlignment;
}

ROBOT_MOTION_CORE_COLD inline void assignMotionProgramWeaveAnchors(MotionSegmentProgram* program) {
    if (!program) return;
    for (uint8_t i = 0; i < kMaxMotionCommands; ++i) {
        program->weaveAnchorS[i] = 0.0;
        program->weaveAnchorTimeSec[i] = 0.0;
        program->weaveAnchorPhase[i] = 0.0;
        program->weaveRunEndS[i] = kWeaveRunOpenEnded;
        program->weaveRunEndTimeSec[i] = kWeaveRunOpenEnded;
    }
    if (program->count == 0) return;

    bool haveRun = false;
    uint8_t runcommandIndex = kWeaveScheduleInline;
    uint16_t previousRunSegment = kMaxMotionSegments;
    double anchorS = 0.0;
    double anchorTime = 0.0;
    double anchorPhase = 0.0;

    for (uint8_t i = 0; i < program->count; ++i) {
        const MotionSegment& segment = program->segments[i];
        const uint8_t index = segment.commandIndex;
        if (index >= kMaxMotionCommands || !weaveParamsAreUsable(program->commandWeave[index])) {
            haveRun = false;
            continue;
        }
        const bool continues = haveRun &&
            weaveParamsEquivalent(program->commandWeave[index], program->commandWeave[runcommandIndex]) &&
            !(previousRunSegment < program->count &&
              motionSegmentsMeetSharply(program->segments[previousRunSegment], segment));
        if (!continues) {
            anchorS = segment.globalPathStart;
            anchorTime = isFinite(segment.startTimeSec) && segment.startTimeSec > 0.0
                ? segment.startTimeSec
                : 0.0;
            // Only a run already live at the first segment can be a continuation of the previous
            // lookahead window. Anything starting later began inside this one, at phase zero.
            anchorPhase = (i == 0 && program->continuesWeaveBeforeStart) ? program->initialWeavePhase : 0.0;
            haveRun = true;
        }
        runcommandIndex = index;
        previousRunSegment = i;
        program->weaveAnchorS[index] = anchorS;
        program->weaveAnchorTimeSec[index] = anchorTime;
        program->weaveAnchorPhase[index] = anchorPhase;
    }

    // Backwards for the far end of each run, which the forward pass cannot know: a run's end is only
    // visible once the segment after it turns out to weave differently, or not at all.
    bool haveEnd = false;
    uint8_t endRunIndex = kWeaveScheduleInline;
    uint16_t laterRunSegment = kMaxMotionSegments;
    double runEndS = kWeaveRunOpenEnded;
    double runEndTime = kWeaveRunOpenEnded;
    for (uint16_t back = program->count; back > 0; --back) {
        const uint16_t i = static_cast<uint16_t>(back - 1);
        const MotionSegment& segment = program->segments[i];
        const uint8_t index = segment.commandIndex;
        if (index >= kMaxMotionCommands || !weaveParamsAreUsable(program->commandWeave[index])) {
            haveEnd = false;
            continue;
        }
        const bool continuesLater = haveEnd &&
            weaveParamsEquivalent(program->commandWeave[index], program->commandWeave[endRunIndex]) &&
            !(laterRunSegment < program->count &&
              motionSegmentsMeetSharply(segment, program->segments[laterRunSegment]));
        if (!continuesLater) {
            // Last segment of its run. Left open when the run runs off the end of a window that has
            // more commands behind it, so the pattern does not taper at a planning seam.
            const bool openEnded = (i + 1 == program->count) && program->continuesAfterEnd;
            runEndS = openEnded ? kWeaveRunOpenEnded : segment.globalPathStart + segment.length;
            runEndTime = openEnded ? kWeaveRunOpenEnded : segment.startTimeSec + segment.durationSec;
            haveEnd = true;
        }
        endRunIndex = index;
        laterRunSegment = i;
        program->weaveRunEndS[index] = runEndS;
        program->weaveRunEndTimeSec[index] = runEndTime;
    }
}

// How far to slow a weaving segment to clear the limit the verifier just reported.
inline double weaveDerateFactorFor(const MotionVerificationFailure& failure, bool linearFalloff) {
    if (failure.kind == MotionVerificationKind::None) return 0.0;
    if (!(failure.observed > 0.0) || !(failure.limit > 0.0)) return 0.0;
    if (failure.observed <= failure.limit) return 0.0;
    double exponent = 1.0;
    if (linearFalloff) {
        const double factor = (failure.limit / failure.observed) * 0.9;
        return factor < 0.5 ? 0.5 : factor;
    }
    switch (failure.kind) {
    case MotionVerificationKind::TcpJerk:
    case MotionVerificationKind::ToolAngularJerk:
    case MotionVerificationKind::JointJerk:
        exponent = 1.0 / 3.0;
        break;
    case MotionVerificationKind::TcpAcceleration:
    case MotionVerificationKind::ToolAngularAcceleration:
    case MotionVerificationKind::JointAcceleration:
        exponent = 0.5;
        break;
    default:
        exponent = 1.0;
        break;
    }
    // Slightly past the mark, so a limit that is only just cleared does not come straight back.
    double factor = pow(failure.limit / failure.observed, exponent) * 0.95;
    if (factor > 0.95) factor = 0.95;   // always make progress
    if (factor < 0.15) factor = 0.15;   // but never collapse to a standstill in one step
    return factor;
}

// Feeds a sampled linear-jerk refusal back into the local path profile.
ROBOT_MOTION_CORE_COLD inline bool tightenNonWeavePathJerkFromSampledFailure(
    const MotionVerificationFailure& failure,
    MotionSegmentProgram* program) {
    if (!program || failure.kind != MotionVerificationKind::TcpJerk ||
        !(failure.observed > failure.limit) || !(failure.limit > 0.0) ||
        failure.trajectoryEdgeIndex >= program->trajectory.edgeCount) {
        return false;
    }

    const MotionTrajectoryEdge& edge = program->trajectory.edges[failure.trajectoryEdgeIndex];
    if (edge.pathSegmentIndex >= program->count) return false;
    const MotionSegment& segment = program->segments[edge.pathSegmentIndex];
    if (segment.kind == MotionSegmentKind::MoveJ) return false;
    if (segment.commandIndex < kMaxMotionCommands &&
        weaveParamsAreUsable(program->commandWeave[segment.commandIndex])) {
        return false;
    }
    if (edge.startNodeIndex >= program->pathGrid.count ||
        edge.endNodeIndex >= program->pathGrid.count) {
        return false;
    }

    double scale = (failure.limit / failure.observed) * 0.98;
    if (scale > 0.98) scale = 0.98;
    if (scale < 0.80) scale = 0.80;
    double speedScale = pow(failure.limit / failure.observed, 1.0 / 3.0) * 0.99;
    if (speedScale > 0.99) speedScale = 0.99;
    if (speedScale < 0.75) speedScale = 0.75;

    double reportedEdgeCap = edge.jerk;
    if (!(reportedEdgeCap > 1.0e-9)) {
        const MotionPathNode& start = program->pathGrid.nodes[edge.startNodeIndex];
        const MotionPathNode& end = program->pathGrid.nodes[edge.endNodeIndex];
        reportedEdgeCap = start.jerkLimit < end.jerkLimit ? start.jerkLimit : end.jerkLimit;
    }
    const double correctedCap = reportedEdgeCap * scale;
    if (!(correctedCap > 1.0e-9)) return false;

    bool changed = false;
    for (uint16_t nodeIndex = 0; nodeIndex < program->pathGrid.count; ++nodeIndex) {
        MotionPathNode& node = program->pathGrid.nodes[nodeIndex];
        if (node.segmentIndex >= program->count) continue;
        const MotionSegment& nodeSegment = program->segments[node.segmentIndex];
        if (nodeSegment.kind == MotionSegmentKind::MoveJ) continue;
        if (nodeSegment.commandIndex < kMaxMotionCommands &&
            weaveParamsAreUsable(program->commandWeave[nodeSegment.commandIndex])) {
            continue;
        }
        if (correctedCap < node.jerkLimit - 1.0e-9) {
            node.jerkLimit = correctedCap;
            node.capReasonMask |= static_cast<uint32_t>(MotionCapJerk);
            changed = true;
        }
        const double speedCap = node.solvedSpeed * speedScale;
        if (speedCap > 1.0e-9 &&
            (node.speedLimit <= 0.0 || speedCap < node.speedLimit - 1.0e-9)) {
            node.speedLimit = speedCap;
            node.solvedSpeed = speedCap;
            node.capReasonMask |= static_cast<uint32_t>(MotionCapJerk);
            changed = true;
        }
    }
    for (uint16_t edgeIndex = 0; edgeIndex < program->trajectory.edgeCount; ++edgeIndex) {
        const MotionTrajectoryEdge& candidate = program->trajectory.edges[edgeIndex];
        if (candidate.pathSegmentIndex >= program->count) continue;
        const MotionSegment& candidateSegment = program->segments[candidate.pathSegmentIndex];
        if (candidateSegment.kind == MotionSegmentKind::MoveJ) continue;
        if (candidateSegment.commandIndex < kMaxMotionCommands &&
            weaveParamsAreUsable(program->commandWeave[candidateSegment.commandIndex])) {
            continue;
        }
        const double edgeSpeedCap = candidate.peakSpeed * speedScale;
        if (edgeSpeedCap > 1.0e-9) {
            changed |= capMotionPathEdgeSpeed(&program->pathGrid, edgeIndex, edgeSpeedCap, MotionCapJerk);
        }
    }
    return changed;
}

ROBOT_MOTION_CORE_COLD inline bool derateWeavingSegments(MotionSegmentProgram* program, double factor) {
    if (!program || !(factor > 0.0) || factor >= 1.0) return false;
    bool derated = false;
    for (uint8_t i = 0; i < program->count; ++i) {
        MotionSegment& segment = program->segments[i];
        if (segment.commandIndex >= kMaxMotionCommands) continue;
        if (!weaveParamsAreUsable(program->commandWeave[segment.commandIndex])) continue;
        if (segment.nominalSpeed > 0.0) segment.nominalSpeed *= factor;
        if (segment.acceleration > 0.0) segment.acceleration *= factor;
        if (segment.jerk > 0.0) segment.jerk *= factor;
        segment.finalNominalSpeed = segment.nominalSpeed;
        markMotionCap(&segment, MotionCapWeave);
        derated = true;
    }
    return derated;
}

// Interpolates trajectory time at a path position.
ROBOT_MOTION_CORE_COLD inline double motionTrajectoryTimeAtS(const MotionSegmentProgram& program, double pathS) {
    const MotionTrajectory& trajectory = program.trajectory;
    if (trajectory.count == 0) return 0.0;
    if (pathS <= trajectory.nodes[0].pathS) return trajectory.nodes[0].timeSec;
    for (uint16_t i = 1; i < trajectory.count; ++i) {
        const MotionTrajectoryNode& previous = trajectory.nodes[i - 1];
        const MotionTrajectoryNode& node = trajectory.nodes[i];
        if (pathS <= node.pathS) {
            const double span = node.pathS - previous.pathS;
            if (span <= 1.0e-12) return node.timeSec;
            const double local = (pathS - previous.pathS) / span;
            return previous.timeSec + (node.timeSec - previous.timeSec) * local;
        }
    }
    return trajectory.nodes[trajectory.count - 1].timeSec;
}

// Places each command's trigger on the solved trajectory.
ROBOT_MOTION_CORE_COLD inline void assignMotionProgramTriggers(MotionSegmentProgram* program) {
    if (!program) return;
    for (uint8_t i = 0; i < kMaxMotionCommands; ++i) {
        program->triggerTimeSec[i] = -1.0;
        program->triggerClamped[i] = 0;
    }
    if (program->count == 0 || program->trajectory.count == 0) return;

    const double totalDuration = program->trajectory.totalDurationSec;

    for (uint8_t command = 0; command < kMaxMotionCommands; ++command) {
        const MotionTrigger& trigger = program->commandTrigger[command];
        if (!motionTriggerIsActive(trigger)) continue;

        // The command's own span, gathered from the segments that came from it. A MoveL that got a
        // corner blend contributes two segments, so the end is the last of them.
        bool found = false;
        double startS = 0.0;
        double endS = 0.0;
        double startTime = 0.0;
        double endTime = 0.0;
        for (uint8_t i = 0; i < program->count; ++i) {
            const MotionSegment& segment = program->segments[i];
            if (segment.commandIndex != command) continue;
            if (!found) {
                startS = segment.globalPathStart;
                startTime = segment.startTimeSec;
                found = true;
            }
            endS = segment.globalPathEnd;
            endTime = segment.startTimeSec + segment.durationSec;
        }
        if (!found) continue;

        const bool fromEnd = trigger.reference == MotionTriggerReference::End;
        double referenceS = fromEnd ? endS : startS;
        double timeSec = fromEnd ? endTime : startTime;

        // Distance first, converted through the trajectory, then the time offset on top. Applied in
        // that order so a combined offset means "this far along, then this much later".
        if (isFinite(trigger.distanceMm) && absDouble(trigger.distanceMm) > 1.0e-9) {
            referenceS += trigger.distanceMm;
            if (referenceS < 0.0) referenceS = 0.0;
            timeSec = motionTrajectoryTimeAtS(*program, referenceS);
        }
        if (isFinite(trigger.timeMs)) timeSec += trigger.timeMs * 0.001;

        if (!isFinite(timeSec)) continue;
        if (timeSec < 0.0) {
            timeSec = 0.0;
            program->triggerClamped[command] = 1;
        } else if (timeSec > totalDuration && !program->continuesAfterEnd) {
            // The program genuinely ends here, so there is nothing further to wait for.
            timeSec = totalDuration;
            program->triggerClamped[command] = 1;
        }
        program->triggerTimeSec[command] = timeSec;
    }
}

ROBOT_MOTION_CORE_COLD inline RejectCode profileContinuousMotionProgramOnce(const RobotModel& model,
                                                                            const MotionProgramSettings& settings,
                                                                            MotionSegmentProgram* program) {
    if (!program) return RejectCode::BadTarget;
    if (program->count == 0) return RejectCode::Ok;
    RejectCode result = buildMotionPathGridFromCurve(model, settings, program);
    if (result != RejectCode::Ok) return result;
    applyMotionPathGridDerivativeConstraints(settings, program);
    constexpr uint8_t kMaxProfileConvergenceIterations = 64;
    for (uint8_t iteration = 0; iteration < kMaxProfileConvergenceIterations; ++iteration) {
        result = solveMotionPathGridTrajectory(program);
        if (result != RejectCode::Ok) return result;
        // Before anything samples. The sampled verifier below reads the anchors, and stale ones
        // would have it check a bead the executor is never going to lay down.
        assignMotionProgramWeaveAnchors(program);
        assignMotionProgramTriggers(program);
        if (enforceSolvedTrajectoryDynamicLimits(model, settings, program)) continue;
        if (settings.verifySampledDynamics) {
            result = verifySampledTrajectoryDynamicLimits(model, settings, *program, &program->verificationFailure);
            if (result != RejectCode::Ok) {
                if (result == RejectCode::BadSpeed &&
                    tightenNonWeavePathJerkFromSampledFailure(program->verificationFailure, program)) {
                    continue;
                }
                return result;
            }
        }
        return validateMotionSegmentProgram(*program);
    }
    // Reserve retries for sampled jerk feedback after analytical tightening converges.
    constexpr uint8_t kMaxSampledJerkFeedbackAttempts = 8;
    for (uint8_t attempt = 0; attempt < kMaxSampledJerkFeedbackAttempts; ++attempt) {
        result = solveMotionPathGridTrajectory(program);
        if (result != RejectCode::Ok) return result;
        assignMotionProgramWeaveAnchors(program);
        assignMotionProgramTriggers(program);
        if (settings.verifySampledDynamics) {
            result = verifySampledTrajectoryDynamicLimits(model, settings, *program, &program->verificationFailure);
            if (result != RejectCode::Ok) {
                if (result == RejectCode::BadSpeed && attempt + 1 < kMaxSampledJerkFeedbackAttempts &&
                    tightenNonWeavePathJerkFromSampledFailure(program->verificationFailure, program)) {
                    continue;
                }
                return result;
            }
        }
        return validateMotionSegmentProgram(*program);
    }
    return result;
}

// Profiles the program, and when the sampled verifier refuses a weaving program, slows the weaving
// segments and tries again.
ROBOT_MOTION_CORE_COLD inline RejectCode profileContinuousMotionProgram(const RobotModel& model,
                                                                        const MotionProgramSettings& settings,
                                                                        MotionSegmentProgram* program) {
    if (!program) return RejectCode::BadTarget;
    if (program->count == 0) return RejectCode::Ok;

    constexpr uint8_t kMaxWeaveDerateAttempts = 8;
    double previousObserved = 0.0;
    bool linearFalloff = false;
    for (uint8_t attempt = 0;; ++attempt) {
        const RejectCode result = profileContinuousMotionProgramOnce(model, settings, program);
        if (result == RejectCode::Ok) return result;
        const MotionVerificationFailure& failure = program->verificationFailure;
        const bool sampledRefusal =
            result == RejectCode::BadSpeed && failure.kind != MotionVerificationKind::None;
        if (!sampledRefusal) {
            // A capacity refusal after we have already slowed the weave down is not the same story as
            // a program that was simply too long to begin with: derating splits the path grid finer
            // every pass, so it is the derating that ran out of nodes. Reported apart because the two
            // want opposite remedies - soften the pattern, or break the program up - and reading
            // too_many_ticks for the first sends you to shorten a program that was never too long.
            if (attempt > 0 && result == RejectCode::TooManyTicks) {
                return RejectCode::WeaveDerateOverflow;
            }
            return result;
        }
        if (attempt >= kMaxWeaveDerateAttempts) return RejectCode::WeaveDerateExhausted;

        // Barely moved despite a derate, so the assumption behind the step size is wrong for this
        // pattern. Say so, and the next step stops treating the excursion as smooth.
        if (previousObserved > 0.0 && failure.observed > previousObserved * 0.9) linearFalloff = true;
        previousObserved = failure.observed;

        const double factor = weaveDerateFactorFor(failure, linearFalloff);
        if (factor <= 0.0) return result;
        if (!derateWeavingSegments(program, factor)) return result;
    }
}

// baseSegmentProgram is caller-owned 86 KB scratch, cleared on entry and not retained.
ROBOT_MOTION_CORE_COLD inline RejectCode buildMotionSegmentProgram(const RobotModel& model,
                                                                   const double startQ[kAxisCount],
                                                                   const MotionProgram& program,
                                                                   const MotionProgramSettings& settings,
                                                                   MotionBaseSegmentProgram* baseSegmentProgram,
                                                                   MotionSegmentProgram* out) {
    if (!out || !startQ || !baseSegmentProgram) return RejectCode::BadTarget;
    memset(out, 0, sizeof(*out));
    out->continuesBeforeStart = program.continuesBeforeStart;
    out->continuesWeaveBeforeStart = program.continuesWeaveBeforeStart;
    out->continuesAfterEnd = program.continuesAfterEnd;
    out->initialPathSpeed = program.initialPathSpeed;
    out->initialWeavePhase = isFinite(program.initialWeavePhase) && program.initialWeavePhase > 0.0
        ? program.initialWeavePhase
        : 0.0;
    for (uint8_t i = 0; i < kMaxMotionCommands; ++i) {
        out->commandTrigger[i] = defaultMotionTrigger();
    }
    for (uint8_t i = 0; i < kMaxMotionCommands && i < program.count; ++i) {
        out->commandWeave[i] = program.commands[i].weave;
        out->commandTrigger[i] = program.commands[i].trigger;
    }
    if (!modelIsValid(model)) return RejectCode::ModelNotLoaded;
    if (!jointArrayIsFinite(startQ)) return RejectCode::BadTarget;
    const RejectCode inputValidation = validateMotionProgramInput(program, settings);
    if (inputValidation != RejectCode::Ok) return inputValidation;

    MotionBaseSegmentProgram& baseProgram = *baseSegmentProgram;
    memset(&baseProgram, 0, sizeof(baseProgram));
    double currentQ[kAxisCount] = {};
    for (uint8_t i = 0; i < kAxisCount; ++i) currentQ[i] = startQ[i];
    Transform currentTcp = toolPoseForJoints(model, currentQ);

    for (uint8_t commandIndex = 0; commandIndex < program.count; ++commandIndex) {
        const MotionCommand& command = program.commands[commandIndex];
        if (command.type == MotionCommandType::MoveJ) {
            MotionSegment segment = {};
            segment.kind = MotionSegmentKind::MoveJ;
            segment.commandId = command.id;
            segment.completedCommandId = command.id;
            segment.startTcp = currentTcp;
            segment.finePoint = 1;
            // Set explicitly, because a zeroed struct would otherwise claim to be command zero.
            // A joint move carries its real ordinal: it cannot weave, but it can carry a trigger.
            segment.commandIndex = commandIndex;
            for (uint8_t i = 0; i < kAxisCount; ++i) {
                if (!isFinite(command.targetQ[i])) return RejectCode::BadTarget;
                if (model.qMin[i] < model.qMax[i] &&
                    (command.targetQ[i] < model.qMin[i] - 1.0e-6 ||
                     command.targetQ[i] > model.qMax[i] + 1.0e-6)) {
                    return RejectCode::JointLimit;
                }
                segment.startQ[i] = currentQ[i];
                segment.endQ[i] = nearestEquivalentRadians(command.targetQ[i], currentQ[i]);
                const double delta = absDouble(segment.endQ[i] - segment.startQ[i]);
                if (delta > segment.length) segment.length = delta;
            }
            if (command.coordinatedPathLength > segment.length) {
                segment.length = command.coordinatedPathLength;
            }
            segment.endTcp = toolPoseForJoints(model, command.targetQ);
            segment.nominalSpeed = degreesToRadians(motionCommandJointSpeedDegPerSec(command, settings));
            segment.acceleration = settings.defaultJointAccelerationRadSec2 > 0.0
                ? settings.defaultJointAccelerationRadSec2
                : degreesToRadians(60.0);
            segment.commandSpeed = segment.nominalSpeed;
            segment.commandAcceleration = segment.acceleration;
            segment.jerk = settings.defaultJointJerkRadSec3 > 0.0
                ? settings.defaultJointJerkRadSec3
                : degreesToRadians(600.0);
            segment.pathParamStart = 0.0;
            segment.pathParamEnd = 1.0;
            markMotionContext(&segment, MotionCapFinePoint);
            if (!appendBaseMotionSegment(&baseProgram, segment)) return RejectCode::TooManyTicks;
            for (uint8_t i = 0; i < kAxisCount; ++i) currentQ[i] = command.targetQ[i];
            currentTcp = segment.endTcp;
            continue;
        }

        if (command.type != MotionCommandType::MoveL) return RejectCode::BadTarget;

        MoveLBlendPlan blendPlan = {};
        const bool explicitFinePoint = command.blendMm < 0.0;
        const bool hasMoveLSuccessor =
            commandIndex + 1 < program.count &&
            program.commands[commandIndex + 1].type == MotionCommandType::MoveL;
        const bool canBlend = command.coordinatedPathLength <= 1.0e-12 && !explicitFinePoint &&
            command.blendMm > 0.0 &&
            hasMoveLSuccessor &&
            planMoveLBlend(currentTcp,
                           command.targetTcp,
                           program.commands[commandIndex + 1].targetTcp,
                           command.blendMm,
                           &blendPlan);

        MoveLInput input = {};
        input.model = model;
        for (uint8_t i = 0; i < kAxisCount; ++i) {
            input.startQ[i] = currentQ[i];
            input.targetConfigQ[i] = command.targetQ[i];
        }
        input.targetTcp = canBlend ? blendPlan.entryTcp : command.targetTcp;
        input.speedMmPerSec = motionCommandLinearSpeedMmPerSec(command, settings);
        input.sampleMm = 0.0;
        input.singularityThresholdRad = settings.singularityThresholdRad;
        input.singularityPolicy = settings.singularityPolicy;
        input.requireTargetConfig = canBlend ? 0 : 1;
        input.targetConfigToleranceRad = 0.5 * 3.14159265358979323846 / 180.0;

        MoveLPlan linePlan = {};
        RejectCode result = planMoveLEndpoint(input, &linePlan);
        if (result != RejectCode::Ok) return result;

        MotionSegment line = {};
        line.kind = MotionSegmentKind::MoveLLine;
        line.commandId = command.id;
        line.completedCommandId = canBlend ? -1 : command.id;
        line.startTcp = currentTcp;
        line.endTcp = input.targetTcp;
        line.length = command.coordinatedPathLength > linePlan.lineLengthMm
            ? command.coordinatedPathLength : linePlan.lineLengthMm;
        line.nominalSpeed = input.speedMmPerSec;
        line.acceleration = settings.defaultLinearAccelerationMmSec2 > 0.0
            ? settings.defaultLinearAccelerationMmSec2
            : 1000.0;
        line.commandSpeed = line.nominalSpeed;
        line.commandAcceleration = line.acceleration;
        line.jerk = settings.defaultLinearJerkMmSec3 > 0.0
            ? settings.defaultLinearJerkMmSec3
            : 10000.0;
        line.pathParamStart = 0.0;
        line.pathParamEnd = 1.0;
        line.finePoint = explicitFinePoint ? 1 : 0;
        line.commandIndex = commandIndex;
        if (line.finePoint) markMotionContext(&line, MotionCapFinePoint);
        for (uint8_t i = 0; i < kAxisCount; ++i) {
            line.startQ[i] = currentQ[i];
            line.endQ[i] = linePlan.endpointQ[i];
        }
        if (!appendBaseMotionSegment(&baseProgram, line)) return RejectCode::TooManyTicks;
        for (uint8_t i = 0; i < kAxisCount; ++i) currentQ[i] = linePlan.endpointQ[i];
        currentTcp = input.targetTcp;

        if (canBlend) {
            selectBlendGeometryForJointDemand(model, currentQ, &blendPlan);
            MotionSegment blend = {};
            blend.kind = MotionSegmentKind::MoveLBlend;
            blend.commandId = command.id;
            blend.completedCommandId = command.id;
            blend.startTcp = blendPlan.entryTcp;
            blend.endTcp = blendPlan.exitTcp;
            blend.blendPlan = blendPlan;
            blend.length = blendPlanTranslationLength(blendPlan);
            blend.curvature = blendPlan.radiusMm > 1.0e-9 ? 1.0 / blendPlan.radiusMm : 0.0;
            const double nextSpeed = motionCommandLinearSpeedMmPerSec(program.commands[commandIndex + 1], settings);
            blend.nominalSpeed = nextSpeed < input.speedMmPerSec ? nextSpeed : input.speedMmPerSec;
            blend.acceleration = settings.defaultLinearAccelerationMmSec2 > 0.0
                ? settings.defaultLinearAccelerationMmSec2
                : 1000.0;
            blend.commandSpeed = blend.nominalSpeed;
            blend.commandAcceleration = blend.acceleration;
            blend.jerk = settings.defaultLinearJerkMmSec3 > 0.0
                ? settings.defaultLinearJerkMmSec3
                : 10000.0;
            blend.pathParamStart = 0.0;
            blend.pathParamEnd = 1.0;
            blend.blended = 1;
            // The corner keeps the weave of the move it rounds off, so the pattern carries through
            // the turn instead of dropping out and restarting on the far side.
            blend.commandIndex = commandIndex;
            markMotionContext(&blend, MotionCapBlendGeometry);
            for (uint8_t i = 0; i < kAxisCount; ++i) blend.startQ[i] = currentQ[i];
            if (!solveToolPoseNearest(model, currentQ, blendPlan.exitTcp, blend.endQ)) {
                return RejectCode::NoIkSolution;
            }
            if (!appendBaseMotionSegment(&baseProgram, blend)) return RejectCode::TooManyTicks;
            for (uint8_t i = 0; i < kAxisCount; ++i) currentQ[i] = blend.endQ[i];
            currentTcp = blendPlan.exitTcp;
        }
    }

    for (uint8_t i = 0; i < baseProgram.count; ++i) {
        MotionSegment segment = baseProgram.segments[i];
        initializeMotionCapDiagnostics(&segment);
        updateMotionSegmentOrientationDemand(settings, &segment);

        WeaveParams segmentWeave = defaultWeaveParams();
        if (segment.commandIndex < kMaxMotionCommands) {
            segmentWeave = out->commandWeave[segment.commandIndex];
        }

        MotionProgramSettings segmentSettings = settings;
        const RejectCode weaveLimits =
            applyWeaveDynamicLimits(model, segmentWeave, &segment, &segmentSettings);
        if (weaveLimits != RejectCode::Ok) return weaveLimits;

        const RejectCode limits = segment.kind == MotionSegmentKind::MoveJ
            ? applyMoveJJointLimits(segmentSettings, &segment)
            : applyCartesianJointDemandLimits(model, segmentSettings, &segment);
        if (limits != RejectCode::Ok) return limits;

        segment.finalNominalSpeed = segment.nominalSpeed;
        if (!appendMotionSegment(out, segment)) return RejectCode::TooManyTicks;
    }
    return profileContinuousMotionProgram(model, settings, out);
}

inline void beginMotionSegmentSampler(const RobotModel& model,
                                      const double startQ[kAxisCount],
                                      const MotionSegmentProgram* program,
                                      MotionSegmentSampler* sampler) {
    if (!sampler) return;
    memset(sampler, 0, sizeof(*sampler));
    sampler->model = model;
    sampler->program = program;
    sampler->pendingCompletedCommandId = -1;
    for (uint8_t i = 0; i < kAxisCount; ++i) sampler->previousQ[i] = startQ[i];
}

inline RejectCode sampleMotionSegmentProgram(MotionSegmentSampler* sampler,
                                             double timeSec,
                                             PathSample* sample) {
    if (!sampler || !sampler->program || !sample) return RejectCode::BadTarget;
    memset(sample, 0, sizeof(*sample));
    sample->completedCommandId = -1;
    if (sampler->program->count == 0) return RejectCode::Ok;
    const MotionTrajectory& trajectory = sampler->program->trajectory;
    if (trajectory.count < 2 || trajectory.edgeCount == 0) return RejectCode::BadDuration;

    while (sampler->trajectoryEdgeIndex + 1 < trajectory.edgeCount &&
           timeSec > trajectory.nodes[sampler->trajectoryEdgeIndex + 1].timeSec + 1.0e-9) {
        const MotionTrajectoryEdge& finished = trajectory.edges[sampler->trajectoryEdgeIndex];
        if (finished.completedCommandId >= 0) {
            sampler->pendingCompletedCommandId = finished.completedCommandId;
            sampler->pendingFinePoint = finished.finePoint;
        }
        const MotionPathNode& node = sampler->program->pathGrid.nodes[finished.endNodeIndex];
        for (uint8_t i = 0; i < kAxisCount; ++i) sampler->previousQ[i] = node.q[i];
        ++sampler->trajectoryEdgeIndex;
    }

    const MotionTrajectoryEdge& edge = trajectory.edges[sampler->trajectoryEdgeIndex];
    if (edge.pathSegmentIndex >= sampler->program->count) return RejectCode::BadDuration;
    const MotionSegment& segment = sampler->program->segments[edge.pathSegmentIndex];
    const double localTime = timeSec - edge.startTimeSec;
    const double edgeDistance = motionTrajectoryEdgeDistanceAtTime(edge, localTime);
    const double pathS = edge.startS + edgeDistance;
    uint16_t pathSegmentIndex = edge.pathSegmentIndex;
    double t = 0.0;
    sample->valid = 1;
    sample->timeSec = timeSec;
    sample->profileSpeed = segment.kind == MotionSegmentKind::MoveJ
        ? 0.0
        : motionTrajectoryEdgeSpeedAtTime(edge, localTime);
    sample->trajectoryEdgeIndex = sampler->trajectoryEdgeIndex;
    RejectCode result = sampleMotionPathAtS(sampler->model,
                                            sampler->program,
                                            sampler->previousQ,
                                            pathS,
                                            sample,
                                            &pathSegmentIndex,
                                            &t);
    if (result != RejectCode::Ok) return result;
    sample->pathProgress = t;

    if (timeSec >= trajectory.nodes[sampler->trajectoryEdgeIndex + 1].timeSec - 1.0e-9) {
        if (edge.completedCommandId >= 0) sample->completedCommandId = edge.completedCommandId;
        sample->finePoint = edge.finePoint;
        const MotionPathNode& node = sampler->program->pathGrid.nodes[edge.endNodeIndex];
        for (uint8_t i = 0; i < kAxisCount; ++i) sampler->previousQ[i] = node.q[i];
    } else {
        for (uint8_t i = 0; i < kAxisCount; ++i) sampler->previousQ[i] = sample->q[i];
    }
    if (sampler->pendingCompletedCommandId >= 0 &&
        (sample->completedCommandId < 0 || sampler->pendingCompletedCommandId < sample->completedCommandId)) {
        sample->completedCommandId = sampler->pendingCompletedCommandId;
        sample->finePoint = sampler->pendingFinePoint;
        sampler->pendingCompletedCommandId = -1;
        sampler->pendingFinePoint = 0;
    }
    return RejectCode::Ok;
}

// The lookahead window loop itself, resumable, for everything that executes a program.
struct MotionWindowRunner {
    RobotModel model;
    MotionProgramSettings settings;

    // Caller-owned scratch, referenced and never owned. Between them these are over half a megabyte -
    // kMaxMotionSegments is 240 - and where they live is the caller's business: the firmware keeps
    // the segment program in DMAMEM and the rest in RAM1, and a runner holding them by value would
    // both blow RAM2 and quietly move them.
    MotionCommandRing* ring;
    // Needed after planning, not only during it: a caller that reports which commands it just ran,
    // or carries their triggers into the next window, reads them back out of here.
    MotionProgram* program;
    MotionSegmentProgram* segmentProgram;
    MotionSegmentSampler* sampler;
    // Scratch used only during planMotionWindow.
    MotionBaseSegmentProgram* baseSegmentProgram;

    double timeOffsetSec;
    double carriedPathSpeed;
    double carriedWeavePhase;
    // Where the next window plans from: the taught path, not the woven pose the arm is standing on.
    // Written by retireMotionWindow through motionReplanSeedQExact, off the joints the caller says
    // it is really at.
    double replanStartQ[kAxisCount];
    // The last sample handed out, so the first sample of the next window can be checked against it.
    // Zeroed is invalid, which is how the first window of a run opts out of the seam check.
    PathSample previousWindowSample;
    double worstSeamSpeedMmPerSec;
    uint8_t carriesSpeed;
    // Cleared by the window that empties the ring, because after it there is nothing to plan from a
    // seam and the next run has to say where the arm actually is. Callers that resume a stopped run
    // check this and call setMotionWindowRunnerSeedQ.
    uint8_t replanSeedValid;

    // This window only. planMotionWindow resets all of it.
    double windowDurationSec;
    // The time the most recent sample was taken at, which is not always the time that was asked for:
    // see the clamp in sampleMotionWindow.
    double windowTimeSec;
    int32_t lastExecutedCommandId;
    uint8_t executeCount;
    uint8_t haveWindow;
    uint8_t seamChecked;
    uint8_t prefixComplete;
};

// Seeds from where the arm is. Does not plan, and does not touch the ring: the caller fills that,
// before this or after it, from wherever its commands come from.
inline void beginMotionWindowRunner(const RobotModel& model,
                                   const MotionProgramSettings& settings,
                                   const double startQ[kAxisCount],
                                   MotionCommandRing* ring,
                                   MotionProgram* program,
                                   MotionSegmentProgram* segmentProgram,
                                   MotionSegmentSampler* sampler,
                                   MotionBaseSegmentProgram* baseSegmentProgram,
                                   MotionWindowRunner* runner) {
    if (!runner) return;
    memset(runner, 0, sizeof(*runner));
    runner->model = model;
    runner->settings = settings;
    runner->ring = ring;
    runner->program = program;
    runner->segmentProgram = segmentProgram;
    runner->sampler = sampler;
    runner->baseSegmentProgram = baseSegmentProgram;
    runner->lastExecutedCommandId = -1;
    if (startQ) {
        for (uint8_t joint = 0; joint < kAxisCount; ++joint) runner->replanStartQ[joint] = startQ[joint];
        runner->replanSeedValid = 1;
    }
}

    // Explicit start state for a new or resumed run.
inline void setMotionWindowRunnerSeedQ(MotionWindowRunner* runner, const double q[kAxisCount]) {
    if (!runner || !q) return;
    for (uint8_t joint = 0; joint < kAxisCount; ++joint) runner->replanStartQ[joint] = q[joint];
    runner->replanSeedValid = 1;
}

// Plans the next window off whatever is queued in the ring.
ROBOT_MOTION_CORE_COLD inline RejectCode planMotionWindow(MotionWindowRunner* runner) {
    if (!runner || !runner->ring || !runner->program || !runner->segmentProgram || !runner->sampler ||
        !runner->baseSegmentProgram) {
        return RejectCode::BadTarget;
    }
    runner->haveWindow = 0;
    runner->seamChecked = 0;
    runner->prefixComplete = 0;
    runner->windowDurationSec = 0.0;
    runner->windowTimeSec = 0.0;
    runner->executeCount = 0;
    runner->lastExecutedCommandId = -1;

    const uint8_t executeCount = motionLookaheadExecutableCount(*runner->ring);
    if (executeCount == 0) return RejectCode::Ok;

    // Unreachable with an executeCount that came from the ring it is being checked against, so the
    // callers lose nothing by not being able to tell this apart from a bad seed.
    if (!buildMotionProgramFromRing(*runner->ring,
                                    executeCount,
                                    runner->carriesSpeed != 0,
                                    runner->carriedPathSpeed,
                                    runner->carriedWeavePhase,
                                    runner->program)) {
        return RejectCode::BadTarget;
    }

    const RejectCode built = buildMotionSegmentProgram(runner->model,
                                                      runner->replanStartQ,
                                                      *runner->program,
                                                      runner->settings,
                                                      runner->baseSegmentProgram,
                                                      runner->segmentProgram);
    if (built != RejectCode::Ok) return built;

    beginMotionSegmentSampler(runner->model, runner->replanStartQ, runner->segmentProgram, runner->sampler);
    const MotionCommand* lastExecuted =
        motionCommandRingAt(*runner->ring, static_cast<uint8_t>(executeCount - 1));
    runner->executeCount = executeCount;
    runner->lastExecutedCommandId = lastExecuted ? lastExecuted->id : -1;
    runner->windowDurationSec = runner->segmentProgram->trajectory.totalDurationSec;
    runner->haveWindow = 1;
    return RejectCode::Ok;
}

// One sample, at the time within this window the caller asks for. Sets *windowComplete when there is
// nothing further to take from this window, at which point the caller checks prefixComplete, calls
// retireMotionWindow with this sample, and plans again.
inline RejectCode sampleMotionWindow(MotionWindowRunner* runner,
                                    double windowTimeSec,
                                    PathSample* outSample,
                                    bool* windowComplete) {
    if (windowComplete) *windowComplete = false;
    if (!runner || !outSample) return RejectCode::BadTarget;
    memset(outSample, 0, sizeof(*outSample));
    outSample->completedCommandId = -1;
    if (!runner->haveWindow) return RejectCode::BadTarget;

    if (!(runner->windowDurationSec > 1.0e-9)) {
        if (windowComplete) *windowComplete = true;
        return RejectCode::Ok;
    }

    bool lastSampleOfWindow = false;
    double sampleTimeSec = windowTimeSec;
    if (!(sampleTimeSec < runner->windowDurationSec - 1.0e-9)) {
        sampleTimeSec = runner->windowDurationSec;
        lastSampleOfWindow = true;
    }

    const RejectCode sampled = sampleMotionSegmentProgram(runner->sampler, sampleTimeSec, outSample);
    if (sampled != RejectCode::Ok) return sampled;
    runner->windowTimeSec = sampleTimeSec;

    // The join with the previous window, checked once, on the first sample this one produces.
    if (!runner->seamChecked && !lastSampleOfWindow) {
        runner->seamChecked = 1;
        const RejectCode seam = motionCheckWindowSeam(runner->previousWindowSample,
                                                     *outSample,
                                                     runner->settings.controlPeriodSec,
                                                     runner->settings,
                                                     &runner->worstSeamSpeedMmPerSec);
        if (seam != RejectCode::Ok) return seam;
    }
    runner->previousWindowSample = *outSample;

    // The executable prefix, not the whole window: a window plans the retained commands too, and
    // they are planned again next time against whatever has been queued behind them. Sampling past
    // here would execute motion that is about to be replanned.
    if (outSample->completedCommandId >= runner->lastExecutedCommandId) runner->prefixComplete = 1;
    if (windowComplete) *windowComplete = runner->prefixComplete != 0 || lastSampleOfWindow;
    return RejectCode::Ok;
}

// Closes the window whose executable prefix has just been covered, and leaves the runner ready to
// plan the next one off the same ring.
ROBOT_MOTION_CORE_COLD inline void retireMotionWindow(MotionWindowRunner* runner,
                                                     const PathSample& prefixEndSample,
                                                     const double currentQ[kAxisCount]) {
    if (!runner || !runner->ring || !currentQ) return;
    // Read before the pop, because it is asking whether anything is queued behind the prefix.
    const bool hasRemaining = runner->executeCount < runner->ring->count;
    runner->timeOffsetSec += prefixEndSample.timeSec;
    runner->carriesSpeed = (hasRemaining && prefixEndSample.profileSpeed > 1.0e-9) ? 1 : 0;
    runner->carriedPathSpeed = runner->carriesSpeed ? prefixEndSample.profileSpeed : 0.0;
    // A stopped run restarts weave at phase zero.
    runner->carriedWeavePhase = hasRemaining ? prefixEndSample.weavePhase : 0.0;
    motionReplanSeedQExact(runner->model, prefixEndSample, currentQ, runner->replanStartQ);
    runner->replanSeedValid = hasRemaining ? 1 : 0;
    motionCommandRingPopFront(runner->ring, runner->executeCount);
    runner->haveWindow = 0;
}

inline bool positionWithinJointLimit(const JointLimitsDeg& limits, uint8_t index, double positionDeg) {
    if (index >= kAxisCount || !isFinite(positionDeg)) {
        return false;
    }
    constexpr double kLimitEpsilonDeg = 0.0001;
    return positionDeg >= limits.min[index] - kLimitEpsilonDeg &&
           positionDeg <= limits.max[index] + kLimitEpsilonDeg;
}

inline double normalizedSpeed(const MoveJInput& input) {
    double speed = input.speedDegPerSec;
    if (!isFinite(speed) || speed <= 0.0) {
        speed = input.defaultSpeedDegPerSec;
    }
    if (isFinite(input.maxSpeedDegPerSec) && input.maxSpeedDegPerSec > 0.0 && speed > input.maxSpeedDegPerSec) {
        speed = input.maxSpeedDegPerSec;
    }
    return speed;
}

inline RejectCode planMoveJ(const MoveJInput& input, MoveJPlan* plan) {
    if (!plan) {
        return RejectCode::BadTarget;
    }
    memset(plan, 0, sizeof(*plan));

    if (input.estopActive) {
        return RejectCode::Estop;
    }

    for (uint8_t i = 0; i < kAxisCount; ++i) {
        plan->targetDeg[i] = input.targetDeg[i];
        if (!isFinite(plan->targetDeg[i])) {
            return RejectCode::BadTarget;
        }
        if (!positionWithinJointLimit(input.limits, i, plan->targetDeg[i])) {
            return RejectCode::JointLimit;
        }
    }

    for (uint8_t i = 0; i < kAxisCount; ++i) {
        plan->startSteps[i] = input.currentSteps[i];
        if (input.mastered[i] == 0) {
            return RejectCode::NotMastered;
        }
        if (!isFinite(input.stepsPerDegree[i]) || input.stepsPerDegree[i] < 1.0 || input.stepsPerDegree[i] > 10000.0) {
            return RejectCode::BadStepsPerDeg;
        }

        const double currentDeg = static_cast<double>(input.currentSteps[i] - input.zeroSteps[i]) / input.stepsPerDegree[i];
        if (!positionWithinJointLimit(input.limits, i, currentDeg)) {
            return RejectCode::CurrentJointLimit;
        }
    }

    for (uint8_t i = 0; i < kAxisCount; ++i) {
        plan->targetSteps[i] = input.zeroSteps[i] + static_cast<int32_t>(llround(input.targetDeg[i] * input.stepsPerDegree[i]));
        const double roundedTargetDeg = static_cast<double>(plan->targetSteps[i] - input.zeroSteps[i]) / input.stepsPerDegree[i];
        if (!positionWithinJointLimit(input.limits, i, roundedTargetDeg)) {
            return RejectCode::RoundedJointLimit;
        }

        plan->deltaSteps[i] = plan->targetSteps[i] - input.currentSteps[i];
        plan->absSteps[i] = absSteps(plan->deltaSteps[i]);
        plan->direction[i] = plan->deltaSteps[i] > 0 ? 1 : (plan->deltaSteps[i] < 0 ? -1 : 0);
        if (plan->absSteps[i] > plan->maxStepCount) {
            plan->maxStepCount = plan->absSteps[i];
        }

        const double currentDeg = static_cast<double>(input.currentSteps[i] - input.zeroSteps[i]) / input.stepsPerDegree[i];
        const double deltaDeg = absDouble(input.targetDeg[i] - currentDeg);
        if (deltaDeg > plan->maxDeltaDeg) {
            plan->maxDeltaDeg = deltaDeg;
        }

        if (plan->direction[i] != 0 &&
            plan->direction[i] == input.limitDirection[i] &&
            input.limitActive[i] != 0) {
            return RejectCode::LimitSwitch;
        }
    }

    if (input.maxTicks > 0 && plan->maxStepCount > input.maxTicks) {
        return RejectCode::TooManyTicks;
    }

    plan->speedDegPerSec = normalizedSpeed(input);
    if (!isFinite(plan->speedDegPerSec) || plan->speedDegPerSec <= 0.0) {
        return RejectCode::BadSpeed;
    }

    if (plan->maxStepCount == 0) {
        plan->durationSec = 0.0;
        plan->tickGapUs = input.minTickGapUs;
        return RejectCode::Ok;
    }

    plan->durationSec = plan->maxDeltaDeg / plan->speedDegPerSec;
    if (!isFinite(plan->durationSec) || plan->durationSec < 0.0) {
        return RejectCode::BadDuration;
    }
    plan->tickGapUs = static_cast<uint32_t>(llround((plan->durationSec * 1000000.0) / static_cast<double>(plan->maxStepCount)));
    if (plan->tickGapUs < input.minTickGapUs) {
        plan->tickGapUs = input.minTickGapUs;
    }

    const double sampleDeg = input.sampleDeg > 0.0 ? input.sampleDeg : 1.0;
    const uint32_t sampleCount = static_cast<uint32_t>(ceil(plan->maxDeltaDeg / sampleDeg));
    for (uint32_t sample = 1; sample <= sampleCount; ++sample) {
        const double t = static_cast<double>(sample) / static_cast<double>(sampleCount);
        for (uint8_t i = 0; i < kAxisCount; ++i) {
            const double currentDeg = static_cast<double>(input.currentSteps[i] - input.zeroSteps[i]) / input.stepsPerDegree[i];
            const double sampledDeg = currentDeg + (input.targetDeg[i] - currentDeg) * t;
            if (!positionWithinJointLimit(input.limits, i, sampledDeg)) {
                return RejectCode::SampleJointLimit;
            }
        }
    }

    return RejectCode::Ok;
}

inline void sampleMoveJDegrees(const double startDeg[kAxisCount],
                               const double targetDeg[kAxisCount],
                               double t,
                               double outDeg[kAxisCount]) {
    if (t < 0.0) t = 0.0;
    if (t > 1.0) t = 1.0;
    for (uint8_t i = 0; i < kAxisCount; ++i) {
        outDeg[i] = startDeg[i] + (targetDeg[i] - startDeg[i]) * t;
    }
}

}  // namespace RobotMotionCore
