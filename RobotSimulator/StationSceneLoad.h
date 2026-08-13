#pragma once

#include "AppState.h"
#include "DragChainPhysics.h"
#include "StationPackage.h"

#include <memory>
#include <string>
#include <vector>

size_t loadPackageProgramsInto(RobotInstance& robot, const std::string& packageFile);
size_t loadStationProgramsInto(RobotInstance& robot,
                               const std::vector<StationRobotInstance::ProgramText>& sources);
size_t probeStationInstancePrograms(const std::string& resolvedPackagePath,
                                    const std::vector<StationRobotInstance::ProgramText>& programs);
int linkedGantryIndexForRobot(size_t robotIndex);
CadVec3 linkedGantryWorldAxis(int gantryIndex);
void configureRobotExternalAxis(RobotInstance& robot);

void loadPackageIntoScene(const std::string& packageFile);
void ensureSceneLoaded(const std::string& packageFile);
std::string runPackageDialog();

struct LoadedStation {
    std::shared_ptr<CadNode> root;
    StationDocument document;
    // Bound only when the caller asked for drag chains; per-command state, not scene state.
    std::vector<DragChainPoseController> chains;
    std::vector<DragChainPhysics> chainPhysics;
};
bool loadStationIntoScene(const std::string& path, LoadedStation* out, std::string* error,
                          bool bindDragChains = false);
