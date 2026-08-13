#pragma once

#include <string>
#include <vector>

#include "JsonCompat.h"
#include "RobotMotionCore.h"
#include "RobotProgramSimulator.h"

int generateRectProgramCommand(const std::vector<std::string>& args, bool triangularWeave = false);
int jsonArrayIntAt(const Json& values, int index, int fallback);
double jsonArrayDoubleAt(const Json& values, int index, double fallback);
double jsonArrayScaledDoubleAt(const Json& values, int index, double scale, double fallback);
Json masteringObjectFromDocument(const Json& root);
bool loadWeaveSchedulesFromMasteringFile(const std::string& path,
                                         RobotMotionCore::WeaveScheduleTable* schedules);
bool loadMotionSettingsOverrideFromConfigFile(const std::string& path,
                                              RobotProgramSimulator::MotionSettingsOverride* override,
                                              std::string* errorMessage);
bool loadStepEstimatorFromMasteringFile(const std::string& path,
                                        RobotProgramSimulator::StepEstimator* estimator,
                                        std::string* errorMessage);
int liveRunCommand(const std::vector<std::string>& args, bool threaded);
int simulateProgramCommand(const std::vector<std::string>& args);
int collisionPoseCommand(const std::vector<std::string>& args);
int dumpPackageProgramsCommand(const std::vector<std::string>& args);
