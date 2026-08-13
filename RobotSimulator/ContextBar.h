#pragma once

// The robot context draws an inset frame around the dockspace; AppFrame sizes the dockspace by it.
inline constexpr float kRobotContextInset = 6.0f;

float persistentContextBarHeight();
bool stationExecutionActive();
void drawPersistentContextBar();
