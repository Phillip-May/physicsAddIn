#include "TrajectoryAndLiveRun.h"
#include "StringUtil.h"

#include "ConveyorPhysics.h"
#include "ConveyorRuntime.h"
#include "LiveRunDriver.h"
#include "MasteringIo.h"
#include "ProgramEditing.h"
#include "RobotRuntime.h"
#include "StationSceneLoad.h"
#include "ViewerBridge.h"

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

bool timelineMarkerVisible(TimelineMarkerType type) {
    switch (type) {
    case TimelineMarkerType::PlannerCap: return g_scene.activeViewState().showPlannerCaps;
    // Always shown: a trigger is something the program asked for, not a diagnostic to filter out.
    case TimelineMarkerType::Trigger: return true;
    case TimelineMarkerType::Singularity: return g_scene.activeViewState().showSingularities;
    case TimelineMarkerType::JointFlip: return g_scene.activeViewState().showJointFlips;
    }
    return true;
}

void cancelTrajectoryBuild(RobotInstance& robot) {
    if (!robot.buildThread.joinable()) {
        robot.buildRunning = false;
        return;
    }
    robot.buildCancel = true;
    robot.buildThread.join();
    robot.buildRunning = false;
    robot.simulator.setProgressCallback(nullptr);
    robot.simulator.stop();
}

void cancelTrajectoryBuild() { cancelTrajectoryBuild(activeRobot()); }

void collectFinishedTrajectory(RobotInstance& robot) {
    robot.timelineSamples.clear();
    for (const RobotProgramSimulator::PlannedSample& sample : robot.simulator.trajectory()) {
        robot.timelineSamples.push_back({sample.timeSeconds,
                                          sample.joints,
                                          sample.tcpPose,
                                          sample.actualTcpSpeedMmPerSec,
                                          sample.desiredTcpSpeedMmPerSec,
                                          sample.profileTcpSpeedMmPerSec,
                                          sample.activeNode,
                                          sample.externalAxisValid,
                                          sample.externalAxisPositionMm});
    }
    robot.timelineMarkers.clear();
    for (const RobotProgramSimulator::Marker& marker : robot.simulator.markers()) {
        TimelineMarkerType type = TimelineMarkerType::JointFlip;
        switch (marker.type) {
        case RobotProgramSimulator::MarkerType::Singularity: type = TimelineMarkerType::Singularity; break;
        case RobotProgramSimulator::MarkerType::PlannerCap:  type = TimelineMarkerType::PlannerCap;  break;
        case RobotProgramSimulator::MarkerType::Trigger:     type = TimelineMarkerType::Trigger;     break;
        case RobotProgramSimulator::MarkerType::JointFlip:   type = TimelineMarkerType::JointFlip;   break;
        }
        robot.timelineMarkers.push_back({marker.seconds, type, marker.message});
    }
    robot.limitReasonCounts = robot.simulator.statistics().capReasonSegmentCounts;
    robot.durationSeconds = robot.simulator.durationSeconds();
    // Draws every arm's samples, so an off-screen arm's path appears with the rest of the cell.
    refreshSimPathPreview();
}

void beginTrajectoryBuild(bool startPlayback) {
    cancelTrajectoryBuild();
    // Capture a stable RobotInstance pointer because selection may change during the build.
    RobotInstance& instance = activeRobot();
    if (!g_scene.viewer || instance.program().root->children.empty()) {
        instance.buildStatus = "Program is empty.";
        return;
    }
    const RobotProgramSimulator::PlanInput plan =
        RobotProgramSimulator::planInputFromPoseController(instance.poseController);
    if (!plan.valid) {
        instance.buildStatus = "Robot is not bound.";
        return;
    }

    // Simulate takes the arm over, so a live run on it stops here. Not only because two things cannot
    // drive one arm: buildTrajectory runs on a worker and plans through m_motionScratch, which holds
    // the very program, segment program and sampler the live run's window runner points at.
    {
        const size_t armIndex = liveRunArmIndexOf(instance);
        auto control = g_scene.liveRunDriver.lockForControl();
        instance.simulator.endLiveRun();
        if (armIndex < g_scene.robots.size()) g_scene.liveRunDriver.publishFrom(armIndex);
    }

    instance.timelineSamples.clear();
    instance.timelineMarkers.clear();
    instance.limitReasonCounts.clear();
    instance.durationSeconds = 0.0;
    instance.elapsedSeconds = 0.0;
    instance.appliedTimelineValid = false;
    instance.buildSentSamples = 0;
    instance.buildCancel = false;
    instance.buildRunning = true;
    instance.startPlaybackAfterBuild = startPlayback;
    instance.buildStatus = "Planning trajectory...";

    // Read editor overrides fresh for each run.
    instance.simulator.setMotionSettingsOverride(currentMotionSettingsOverride());
    // Same reason: a WeaveOn index means nothing to the planner until the table it indexes into is
    // handed over, and the Welding tab can be edited between runs.
    instance.simulator.setWeaveSchedules(instance.weaveSchedules);
    configureRobotExternalAxis(instance);
    instance.simulator.start(instance.program().root.get(), instance.poseController.joints(),
                             currentProgramState());
    instance.simulator.setProgressCallback([target = &instance](size_t total) {
        // Runs on the worker. Only the count is published; the samples are copied once at the
        // end, so nothing here touches a vector the UI thread might be reading.
        target->buildSentSamples = total;
        return !target->buildCancel.load();
    });

#ifdef __EMSCRIPTEN__
    std::string error;
    const bool ok = instance.simulator.buildTrajectory(plan, &error);
    instance.buildRunning = false;
    if (ok) collectFinishedTrajectory(instance);
    instance.buildStatus = ok ? std::string() : error;
    instance.playing = ok && startPlayback;
#else
    instance.buildThread = std::thread([plan, target = &instance]() {
        std::string error;
        const bool ok = target->simulator.buildTrajectory(plan, &error);
        target->buildStatus = ok ? std::string() : error;
        target->buildRunning = false;
    });
#endif
}

int clampedLiveSpeedIndex() {
    return std::max(0, std::min(g_scene.liveSpeedIndex, kLiveSpeedCount - 1));
}

bool achievedRateKeepingUp(double achieved, double requested) {
    return achieved >= requested * 0.9;
}

ImVec4 achievedRateCpuBoundColor() {
    return ImVec4(0.85f, 0.6f, 0.15f, 1.0f);
}

bool stationAchievedRate(double* outRate) {
    bool valid = false;
    double slowestRate = std::numeric_limits<double>::max();
    if (g_scene.measuredLiveSpeedValid) {
        slowestRate = g_scene.measuredLiveSpeed;
        valid = true;
    }
    double conveyorRate = 0.0;
    if (g_scene.conveyorPhysics.asyncAchievedRate(&conveyorRate)) {
        slowestRate = valid ? std::min(slowestRate, conveyorRate) : conveyorRate;
        valid = true;
    }
    if (valid && outRate) *outRate = slowestRate;
    return valid;
}

// Everything the frame thread still has to do about live runs: take what the driver published and put
// it in the scene.
void applyActuatorInstructionsForState(
    RobotInstance& robot, const RobotProgramSimulator::LiveRunState& state) {
    if (!robot.program().root || state.instruction < 0) return;
    // A worker can advance from the last visible motion through actuator releases and Stop between
    // two UI frames. The terminal snapshot still owns those reached instructions; apply them even
    // though running is already false so a fast run cannot leave its workpieces attached.
    if (state.instruction < robot.appliedActuatorInstruction) {
        robot.appliedActuatorCycle = state.completedCycles;
        robot.appliedActuatorInstruction = -1;
    }
    robot.appliedActuatorCycle = state.completedCycles;
    const int current = std::min(
        state.instruction, static_cast<int>(robot.program().root->children.size()) - 1);
    for (int instruction = robot.appliedActuatorInstruction + 1;
         instruction <= current; ++instruction) {
        const auto& node = robot.program().root->children[static_cast<size_t>(instruction)];
        if (node && node->type == RobotProgramNodeType::SetTool) {
            std::string error;
            if (!commandProgramTool(robot, std::get<SetToolData>(node->data), &error)) {
                robot.statusText = error;
            }
        } else if (node && node->type == RobotProgramNodeType::Actuate) {
            commandToolActuator(std::get<ActuateData>(node->data));
        }
    }
    robot.appliedActuatorInstruction = current;
}

void applyLiveRunResults() {
    for (size_t i = 0; i < g_scene.robots.size(); ++i) {
        RobotInstance* robot = g_scene.robots[i].get();
        if (!robot) continue;
        LiveRunDriver::Snapshot snapshot;
        if (!g_scene.liveRunDriver.snapshot(i, &snapshot)) continue;
        robot->liveRun = snapshot.state;
        const int gantryIndex = linkedGantryIndexForRobot(i);
        if (gantryIndex >= 0 && snapshot.state.externalAxisValid) {
            g_scene.gantries[static_cast<size_t>(gantryIndex)].setPositionMm(
                snapshot.state.externalAxisPositionMm);
        }
        // Only while it is running. A finished run leaves the arm where it stopped, and re-applying
        // those joints every frame afterwards would fight the gimbal and the joint sliders.
        if (snapshot.jointsValid && snapshot.state.running) {
            robot->poseController.setJoints(snapshot.joints);
        }
        applyActuatorInstructionsForState(*robot, snapshot.state);
    }

    // Tool poses follow the joints above. Updating afterwards gives PhysX the exact rendered jaw
    // pose and current bounded pad-force target for every worker substep.
    updatePhysicsToolJaws();
    if (g_scene.conveyorPhysics.asyncActive()) {
        const double simulatedSeconds = std::max(
            1.0e-6, static_cast<double>(ImGui::GetIO().DeltaTime) *
                kLiveSpeedFactors[clampedLiveSpeedIndex()]);
        applyPhysicsToolForces(simulatedSeconds);
    }

    double rate = 0.0;
    g_scene.measuredLiveSpeedValid = g_scene.liveRunDriver.achievedRate(&rate);
    g_scene.measuredLiveSpeed = rate;
}

// One slice of live-run work, and the only part of this that differs between hosts.
#ifdef __EMSCRIPTEN__
void pumpLiveRunsForFrame() {
    // Bounded by wall time because the browser has to get its thread back. A slice is short, so this is
    // several of them, and what it cannot deliver the driver drops and reports.
    constexpr double kBudgetSeconds = 0.006;
    const std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    for (;;) {
        const LiveRunDriver::StepResult step = g_scene.liveRunDriver.stepOnce();
        if (!step.anyRunning || !step.didWork) break;
        const double spent = std::chrono::duration_cast<std::chrono::duration<double>>(
            std::chrono::steady_clock::now() - start).count();
        if (spent >= kBudgetSeconds) break;
        // Returns control to the page and resumes here. Legal because no ImGui frame is open and the
        // slice has finished, so nothing is half-written across the unwind.
        emscripten_sleep(0);
    }
}
#else
void liveRunThreadMain() {
    while (!g_scene.liveRunThreadStop.load()) {
        const LiveRunDriver::StepResult step = g_scene.liveRunDriver.stepOnce();
        if (!step.anyRunning) {
            // Idle. Long enough not to matter, short enough that a run starts without a visible delay.
            std::this_thread::sleep_for(std::chrono::milliseconds(4));
        } else if (!step.didWork) {
            // Running, but ahead of the clock - the ordinary case below about 10x. Waiting here is what
            // makes the arm follow the wall clock instead of running as fast as the CPU allows.
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        } else {
            // Behind, so straight back in. The control lock was released on the way out of stepOnce,
            // which is what lets the frame thread start or stop a run without waiting for the backlog.
            std::this_thread::yield();
        }
    }
}
#endif

// Starts an arm running its selected program for real, off the shared lookahead.
bool startLiveRunFor(RobotInstance& robot, bool loop) {
    if (!robot.poseController.isBound()) {
        robot.statusText = "Robot is not bound.";
        return false;
    }
    const int linkedGantry = linkedGantryIndexForRobot(liveRunArmIndexOf(robot));
    if (linkedGantry >= 0 && robot.program().root) {
        const GantryMechanismData* gantry =
            g_scene.gantries[static_cast<size_t>(linkedGantry)].gantryData();
        for (const auto& child : robot.program().root->children) {
            if (!child) continue;
            bool hasExternal = false;
            double targetMm = 0.0;
            if (child->type == RobotProgramNodeType::MoveJ) {
                const MoveJData& move = std::get<MoveJData>(child->data);
                hasExternal = move.hasExternalAxis;
                targetMm = move.externalAxisPositionMm;
            } else if (child->type == RobotProgramNodeType::MoveL) {
                const MoveLData& move = std::get<MoveLData>(child->data);
                hasExternal = move.hasExternalAxis;
                targetMm = move.externalAxisPositionMm;
            }
            if (hasExternal && gantry &&
                (targetMm < gantry->lowerLimitMm || targetMm > gantry->upperLimitMm)) {
                robot.statusText = strutil::format(
                    "J7 target %1 mm is outside linked mechanism travel [%2, %3] mm.")
                    .arg(targetMm).arg(gantry->lowerLimitMm).arg(gantry->upperLimitMm).str();
                return false;
            }
        }
    }
    cancelTrajectoryBuild(robot);
    robot.playing = false;
    robot.appliedTimelineValid = false;
    robot.appliedActuatorInstruction = -1;
    robot.appliedActuatorCycle = -1;

    const size_t armIndex = liveRunArmIndexOf(robot);
    std::string errorMessage;
    bool started = false;
    {
        auto control = g_scene.liveRunDriver.lockForControl();
        robot.simulator.setMotionSettingsOverride(motionSettingsOverrideFor(robot));
        robot.simulator.setWeaveSchedules(robot.weaveSchedules);
        // No step estimator: a live run is the planned path, not a guess at what the steppers resolve it
        // to. The mastering readouts belong to the timeline, where the comparison is the point.
        robot.simulator.clearStepEstimator();
        configureRobotExternalAxis(robot);
        robot.simulator.start(robot.program().root.get(), robot.poseController.joints(),
                              programStateFor(robot));
        const RobotProgramSimulator::PlanInput plan =
            RobotProgramSimulator::planInputFromPoseController(robot.poseController);
        started = robot.simulator.beginLiveRun(plan, loop, &errorMessage);
        if (started && armIndex < g_scene.robots.size()) {
            g_scene.liveRunDriver.publishFrom(armIndex);
        }
    }
    if (!started) {
        robot.statusText = errorMessage;
        return false;
    }
    // From now, not from whenever the clock last ticked: the time the operator spent deciding is not
    // simulated time the arm owes.
    g_scene.liveRunDriver.setSpeedFactor(kLiveSpeedFactors[clampedLiveSpeedIndex()]);
    g_scene.liveRunDriver.resetClock();
    robot.statusText.clear();
    return true;
}

void stopLiveRunFor(RobotInstance& robot) {
    const size_t armIndex = liveRunArmIndexOf(robot);
    auto control = g_scene.liveRunDriver.lockForControl();
    robot.simulator.endLiveRun();
    if (armIndex < g_scene.robots.size()) g_scene.liveRunDriver.publishFrom(armIndex);
}

void startStationSimulation() {
    if (g_scene.stationRunState == StationRunState::Paused) {
        g_scene.liveRunDriver.setPaused(false);
        g_scene.conveyorPhysics.setAsyncPaused(false);
        g_scene.stationRunState = StationRunState::Running;
        g_scene.stationSimulationStatus = "Station resumed.";
        return;
    }
    if (g_scene.stationRunState == StationRunState::Running) return;

    g_scene.liveRunDriver.setPaused(false);
    rebuildConveyorRuntime();
    int startedRobots = 0;
    int loopingRobots = 0;
    int programStopRobots = 0;
    int skippedRobots = 0;
    for (const std::unique_ptr<RobotInstance>& robot : g_scene.robots) {
        if (!robot) continue;
        cancelTrajectoryBuild(*robot);
        robot->playing = false;
        const bool hasProgram = robot->program().root && !robot->program().root->children.empty();
        if (!hasProgram) {
            ++skippedRobots;
            robot->statusText = "Station Start skipped this robot: selected program is empty.";
            continue;
        }
        if (startLiveRunFor(*robot, true)) {
            ++startedRobots;
            if (robot->simulator.liveRunState().looping) ++loopingRobots;
            else ++programStopRobots;
        } else {
            ++skippedRobots;
        }
    }
    g_scene.conveyorPhysics.startAsync(
        kLiveSpeedFactors[clampedLiveSpeedIndex()]);
    g_scene.stationRunState = StationRunState::Running;
    const std::string conveyorWarning = g_scene.stationSimulationStatus;
    g_scene.stationSimulationStatus =
        strutil::format("Station started: %1 robot program(s), %2 looping, %3 ending at Stop, "
                        "%4 skipped, %5 spawner(s).")
            .arg(startedRobots)
            .arg(loopingRobots)
            .arg(programStopRobots)
            .arg(skippedRobots)
            .arg(g_scene.conveyorSpawners.size())
            .str();
    if (!conveyorWarning.empty()) g_scene.stationSimulationStatus += " " + conveyorWarning;
}

void pauseStationSimulation() {
    if (g_scene.stationRunState != StationRunState::Running) return;
    g_scene.liveRunDriver.setPaused(true);
    g_scene.conveyorPhysics.setAsyncPaused(true);
    g_scene.stationRunState = StationRunState::Paused;
    g_scene.stationSimulationStatus = "Station paused; robot and conveyor state preserved.";
}

void stopStationSimulation() {
    g_scene.liveRunDriver.setPaused(false);
    g_scene.conveyorPhysics.stopAsync();
    for (const std::unique_ptr<RobotInstance>& robot : g_scene.robots) {
        if (!robot) continue;
        cancelTrajectoryBuild(*robot);
        robot->playing = false;
        stopLiveRunFor(*robot);
    }
    clearConveyorWorkpieces();
    g_scene.conveyorTimeAccumulatorSeconds = 0.0;
    g_scene.stationRunState = StationRunState::Stopped;
    g_scene.stationSimulationStatus = "Station stopped; runtime workpieces cleared.";
}

void pollTrajectoryBuild() {
    // Join completed workers for every arm, not only the selected arm.
    for (const std::unique_ptr<RobotInstance>& robot : g_scene.robots) {
        if (!robot || !robot->buildThread.joinable() || robot->buildRunning) continue;
        robot->buildThread.join();
        robot->simulator.setProgressCallback(nullptr);
        if (robot->buildStatus.empty()) {
            collectFinishedTrajectory(*robot);
            robot->playing = robot->startPlaybackAfterBuild;
        }
    }

    // The timeline below is the selected arm's: one set of playback controls, driving whichever arm
    // is on screen.
    if (activeRobot().timelineSamples.empty()) return;

    if (activeRobot().playing) {
        activeRobot().elapsedSeconds += static_cast<double>(ImGui::GetIO().DeltaTime);
        if (activeRobot().elapsedSeconds >= activeRobot().durationSeconds) {
            activeRobot().elapsedSeconds = activeRobot().durationSeconds;
            activeRobot().playing = false;
        }
    }

    // Apply timeline scrubs as well as playback updates.
    if (activeRobot().appliedTimelineValid && activeRobot().appliedTimelineSeconds == activeRobot().elapsedSeconds) {
        return;
    }
    activeRobot().appliedTimelineSeconds = activeRobot().elapsedSeconds;
    activeRobot().appliedTimelineValid = true;

    // Nearest stored sample at or before the scrub time.
    const TimelineSample* best = &activeRobot().timelineSamples.front();
    for (const TimelineSample& sample : activeRobot().timelineSamples) {
        if (sample.seconds <= activeRobot().elapsedSeconds) best = &sample; else break;
    }
    activeRobot().poseController.setJoints(best->joints);
    const size_t robotIndex = liveRunArmIndexOf(activeRobot());
    const int gantryIndex = linkedGantryIndexForRobot(robotIndex);
    if (gantryIndex >= 0 && best->externalAxisValid) {
        g_scene.gantries[static_cast<size_t>(gantryIndex)].setPositionMm(
            best->externalAxisPositionMm);
    }
    // The gimbal follows from refreshPoseDerivedReadoutsIfMoved, which runs straight after this.
}

