#pragma once

#include <string>
#include <vector>

int renderStationCommand(const std::vector<std::string>& args);
int validateGripperDemoCommand(const std::vector<std::string>& args);
int validateStationProgramLoopCommand(const std::vector<std::string>& args);
int dumpStationProgramIkCommand(const std::vector<std::string>& args);
int dumpConveyorTraceCommand(const std::vector<std::string>& args);
