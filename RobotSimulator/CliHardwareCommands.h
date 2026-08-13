#pragma once

#include <string>
#include <vector>

#ifndef ROBOTSIM_NO_SERIAL
int hardwareIoListCommand();
int hardwareIoStatusCommand(const std::vector<std::string>& args);
int hardwareIoLoadMasteringCommand(const std::vector<std::string>& args);
int hardwareIoSendCommand(const std::vector<std::string>& args);
int hardwareIoMonitorCommand(const std::vector<std::string>& args);
#endif
