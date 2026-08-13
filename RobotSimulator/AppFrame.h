#pragma once

#include <string>

#ifdef __EMSCRIPTEN__
void pollWebFileRequests();
#endif
void drawRobotSimulatorUi(const std::string& initialPackage);
