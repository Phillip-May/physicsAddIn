#pragma once

#include "AppState.h"
#include "ProgramTextIo.h"
#include "RobotProgramModel.h"

#include "imgui.h"

#include <vector>

bool timelineMarkerVisible(TimelineMarkerType type);
void cancelTrajectoryBuild(RobotInstance& robot);
void cancelTrajectoryBuild();
void collectFinishedTrajectory(RobotInstance& robot);
void beginTrajectoryBuild(bool startPlayback);

int clampedLiveSpeedIndex();
bool achievedRateKeepingUp(double achieved, double requested);
ImVec4 achievedRateCpuBoundColor();
bool stationAchievedRate(double* outRate);
void applyActuatorInstructionsForState(
    RobotInstance& robot, const RobotProgramSimulator::LiveRunState& state);
void applyLiveRunResults();
#ifdef __EMSCRIPTEN__
void pumpLiveRunsForFrame();
#else
void liveRunThreadMain();
#endif
bool startLiveRunFor(RobotInstance& robot, bool loop);
void stopLiveRunFor(RobotInstance& robot);
void startStationSimulation();
void pauseStationSimulation();
void stopStationSimulation();
void pollTrajectoryBuild();
