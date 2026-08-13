#pragma once

#include "AppState.h"
#include "MeshRobotViewer.h"
#include "StationPackage.h"

#include <string>
#include <vector>

CadNode* selectedStationNode();
void selectStationObject(StationSelectionKind kind, int index);
void pollAllHardware();
void drawHardwareMarker(const RobotInstance& robot);

// Which arm this is, for publishing against. LiveRunDriver indexes by position in g_scene.robots, so
// anything that starts or stops a run has to be able to say where that arm sat. Returns the list size
// for an instance that is not in it, which is the same "no such arm" the driver's own bounds check uses.
inline size_t liveRunArmIndexOf(const RobotInstance& robot) {
    for (size_t i = 0; i < g_scene.robots.size(); ++i) {
        if (g_scene.robots[i].get() == &robot) return i;
    }
    return g_scene.robots.size();
}

// The arms the driver steps. Rebuilt whenever the scene's instance list changes, because it holds a raw
// pointer to each simulator and a reload destroys them.
inline void rebindLiveRunDriverArms() {
    std::vector<RobotProgramSimulator*> arms;
    arms.reserve(g_scene.robots.size());
    for (const std::unique_ptr<RobotInstance>& robot : g_scene.robots) {
        arms.push_back(robot ? &robot->simulator : nullptr);
    }
    auto control = g_scene.liveRunDriver.lockForControl();
    g_scene.liveRunDriver.setArms(std::move(arms));
}

float shortestAngleDelta(float from, float to);
MeshRobotViewer::CameraState lerpCamera(const MeshRobotViewer::CameraState& a,
                                        const MeshRobotViewer::CameraState& b, float t);
void settleCameraTween();
void cancelCameraTween();
void beginCameraTween(const MeshRobotViewer::CameraState& from,
                      const MeshRobotViewer::CameraState& to);
void stepCameraTween(float deltaSeconds);

void refreshPoseDerivedReadouts();
void refreshPoseDerivedReadoutsIfMoved();
void refreshProgramPathPreview();
void refreshSimPathPreview();
std::vector<MeshRobotViewer::ConveyorDirectionGuide> buildConveyorDirectionGuides(
    const StationDocument& station);
void applySceneViewState();

void switchView(ViewTarget::Kind kind, int robotIndex);
void enterRobotView(int robotIndex);
void exitToStationView();
StationDocument stationDocumentFromScene();
void saveStationToFile();

CadNode* placementNodeFor(const RobotInstance& robot);
