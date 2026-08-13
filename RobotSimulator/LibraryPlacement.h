#pragma once

#include "AppState.h"
#include "RobotLibraryPanel.h"
#include "StationParameterLinks.h"

#include <string>
#include <vector>

// The search itself is `Common/MountingSnap`, over explicit inputs and an abstract camera; what is
// here is the lifecycle around it, because what a placement *delivers into* is this cell's own -
// a StationDocument entry, a pose controller and a robot runtime.
bool beginLibraryRobotPlacement(const RobotLibraryPanel::AssetRequest& request);
bool beginExistingStationObjectPlacement(StationSelectionKind kind, int index);
void cancelLibraryRobotPlacement();
void rotateLibraryRobotPlacement(float wheelDelta);
void updateLibraryRobotPlacement(const PointI& cursor, int viewportWidth, int viewportHeight);
bool finishLibraryPlacement(const std::string& statusText, bool refreshPose);
bool commitLibraryRobotPlacement();
