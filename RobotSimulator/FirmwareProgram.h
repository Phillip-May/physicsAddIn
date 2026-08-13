#pragma once

#include <array>
#include <map>
#include <string>
#include <vector>

#include "JsonCompat.h"
#include "RobotProgramSimulator.h"

struct RobotInstance;

// The preflight gate before Run program; see the definition for what it refuses and why.
bool buildHardwareProgramCommands(const RobotInstance& robot,
                                  const std::array<double, 6>& startDeg,
                                  const RobotProgramSimulator::ProgramState& runState,
                                  std::vector<Json>* commands,
                                  std::map<int, std::string>* triggerMessages,
                                  std::string* errorMessage);
