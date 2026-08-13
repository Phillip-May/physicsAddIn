#pragma once

#include <string>
#include <vector>

#include "JsonCompat.h"
#include "RobotMotionCore.h"
#include "StationPackage.h"

// robotModelCommandForPackage lives in MasteringIo.h; probeStationInstancePrograms in
// StationSceneLoad.h.
#include "MasteringIo.h"
#include "StationSceneLoad.h"

int validatePackageCommand(const std::vector<std::string>& args);
int validateStationCommand(const std::vector<std::string>& args);
int orientationCommand(const std::vector<std::string>& args);
int listRobotsCommand(const std::vector<std::string>& args);
int resaveStationCommand(const std::vector<std::string>& args);
int dumpAxesCommand(const std::vector<std::string>& args);
int toolPoseCommand(const std::vector<std::string>& args);
int dumpRobotModelCommand(const std::vector<std::string>& args);
int stationIkPositionCommand(const std::vector<std::string>& args);
int ikSmokeCommand(const std::vector<std::string>& args);
int bakeHullsCommand(const std::vector<std::string>& args);
