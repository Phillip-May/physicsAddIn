#include "ContextBar.h"

#include "AppState.h"
#include "HardwareIoPanel.h"
#include "LibraryPlacement.h"
#include "StringUtil.h"
#include "TrajectoryAndLiveRun.h"
#include "UiFormHelpers.h"
#include "ViewerBridge.h"

#include "imgui.h"

#include <algorithm>
#include <memory>
#include <string>
float persistentContextBarHeight() {
    return 94.0f;
}


bool stationExecutionActive() {
    if (g_scene.stationRunState != StationRunState::Stopped) return true;
    for (const std::unique_ptr<RobotInstance>& robot : g_scene.robots) {
        if (!robot) continue;
        if (robot->buildRunning || robot->playing || robot->liveRun.running) return true;
    }
    return false;
}

bool contextToggleButton(const char* label, bool* value) {
    if (!value) return false;
    if (*value) {
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.22f, 0.42f, 0.67f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.28f, 0.50f, 0.76f, 1.0f));
    }
    const bool pressed = ImGui::Button(label);
    if (*value) ImGui::PopStyleColor(2);
    if (pressed) *value = !*value;
    return pressed;
}

bool stationModeButton(const char* label, StationMode mode, bool disabled) {
    const bool selected = g_scene.stationMode == mode;
    if (selected) {
        const ImVec4 accent = mode == StationMode::Layout
            ? ImVec4(0.18f, 0.43f, 0.72f, 1.0f)
            : ImVec4(0.48f, 0.30f, 0.72f, 1.0f);
        ImGui::PushStyleColor(ImGuiCol_Button, accent);
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, accent);
    }
    ImGui::BeginDisabled(disabled);
    const bool pressed = ImGui::Button(label, ImVec2(104.0f, 0.0f));
    ImGui::EndDisabled();
    if (selected) ImGui::PopStyleColor(2);
    if (pressed) {
        g_scene.setStationMode(mode);
        g_scene.stationPanelTab = mode == StationMode::Layout
            ? StationPanelTab::Layout : StationPanelTab::Program;
        g_scene.stationPanelTabSelectionPending = true;
    }
    return pressed;
}

std::string activeRobotDisplayName() {
    const CadNode* node = activeRobot().poseController.robotNode();
    return node && !node->name.empty() ? node->name : "Robot";
}

std::string stationContextStatus() {
    if (g_libraryPlacement.active()) {
        return std::string(g_libraryPlacement.movingExisting ? "Moving " : "Placing ") +
               g_libraryPlacement.displayName + " - " + g_libraryPlacement.status();
    }
    if (g_scene.stationRunState == StationRunState::Paused) return "Station paused";
    if (g_scene.stationRunState == StationRunState::Running) return "Station running";
    const RobotInstance& robot = activeRobot();
    if (robot.buildRunning) {
        return strutil::format("Planning %1 - %2 samples")
            .arg(activeRobotDisplayName()).arg(robot.buildSentSamples).str();
    }
    if (robot.playing) {
        return strutil::format("Simulating %1 - %2 / %3 s")
            .arg(activeRobotDisplayName())
            .arg(robot.elapsedSeconds, 0, 'f', 2)
            .arg(robot.durationSeconds, 0, 'f', 2).str();
    }
    if (robot.liveRun.running) {
        return strutil::format("Running %1 - instruction %2")
            .arg(activeRobotDisplayName()).arg(robot.liveRun.instruction + 1).str();
    }
    return g_scene.stationMode == StationMode::Layout
        ? "Placement and station geometry"
        : "Selected robot: " + activeRobotDisplayName();
}

void drawStationExecutionCluster() {
    const bool stopped = g_scene.stationRunState == StationRunState::Stopped;
    const bool running = g_scene.stationRunState == StationRunState::Running;

    ImGui::BeginDisabled(running);
    if (ImGui::Button("Start")) startStationSimulation();
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip(
            "Start or resume the station. Programs loop unless they execute a Stop instruction.");
    }

    ImGui::SameLine();
    ImGui::BeginDisabled(!running);
    if (ImGui::Button("Pause")) pauseStationSimulation();
    ImGui::EndDisabled();

    ImGui::SameLine();
    ImGui::BeginDisabled(stopped);
    if (ImGui::Button("Stop")) stopStationSimulation();
    ImGui::EndDisabled();

    ImGui::SameLine();
    const int speedIndex = clampedLiveSpeedIndex();
    if (ImGui::Button(kLiveSpeedLabels[speedIndex], ImVec2(54.0f, 0.0f))) {
        g_scene.liveSpeedIndex = (speedIndex + 1) % kLiveSpeedCount;
        g_scene.liveRunDriver.setSpeedFactor(
            kLiveSpeedFactors[clampedLiveSpeedIndex()]);
        g_scene.conveyorPhysics.setAsyncSpeedFactor(
            kLiveSpeedFactors[clampedLiveSpeedIndex()]);
    }
    if (ImGui::IsItemHovered()) {
        double achievedRate = 0.0;
        if (g_scene.conveyorPhysics.asyncAchievedRate(&achievedRate)) {
            ImGui::SetTooltip(
                "Requested station speed; click to cycle.\n"
                "Conveyor worker achieved %.1fx; dropped %.2f s of excess simulation debt.\n"
                "PhysX remains at 60 Hz and throttles instead of blocking the GUI.",
                achievedRate, g_scene.conveyorPhysics.asyncDroppedSimSeconds());
        } else {
            ImGui::SetTooltip(
                "Requested station speed; click to cycle.\n"
                "Conveyor physics runs at 60 Hz on a worker and throttles if CPU capped.");
        }
    }

    if (running) {
        ImGui::SameLine();
        double actualRate = 0.0;
        if (stationAchievedRate(&actualRate)) {
            const double requestedRate = kLiveSpeedFactors[clampedLiveSpeedIndex()];
            if (achievedRateKeepingUp(actualRate, requestedRate)) {
                ImGui::TextDisabled("%.2fx actual", actualRate);
            } else {
                ImGui::TextColored(achievedRateCpuBoundColor(), "%.2fx actual", actualRate);
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip(
                    "Slowest achieved station rate over the shared 4-second rolling window.\n"
                    "This readout does not limit robot or conveyor execution.");
            }
        } else {
            ImGui::TextDisabled("-.--x actual");
        }
    }

    ImGui::SameLine();
    ImGui::AlignTextToFramePadding();
    ImGui::TextDisabled("CONVEYORS");
    ImGui::SameLine(0.0f, 6.0f);
    const char* modeLabel = g_scene.defaultConveyorMode == ConveyorSimulationMode::Logical
        ? "Logical" : "PhysX";
    ImGui::BeginDisabled(!stopped);
    ImGui::SetNextItemWidth(96.0f);
    if (ImGui::BeginCombo("##globalConveyorMode", modeLabel)) {
        if (ImGui::Selectable("Logical", g_scene.defaultConveyorMode ==
                                         ConveyorSimulationMode::Logical)) {
            g_scene.defaultConveyorMode = ConveyorSimulationMode::Logical;
        }
        if (ImGui::Selectable("PhysX", g_scene.defaultConveyorMode ==
                                       ConveyorSimulationMode::PhysX)) {
            g_scene.defaultConveyorMode = ConveyorSimulationMode::PhysX;
        }
        ImGui::EndCombo();
    }
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Station default used by conveyors whose mode is Global default.");
    }
}

void drawStationContextShelf() {
    ViewState& view = g_scene.simView;
    if (g_libraryPlacement.active()) {
        ImGui::AlignTextToFramePadding();
        ImGui::TextDisabled("PLACEMENT");
        ImGui::SameLine(0.0f, 12.0f);
        ImGui::TextUnformatted(
            ((g_libraryPlacement.movingExisting ? "Moving " : "Placing ") +
             g_libraryPlacement.displayName).c_str());
        ImGui::SameLine(0.0f, 18.0f);
        ImGui::TextColored(g_libraryPlacement.snapped()
                               ? ImVec4(0.30f, 0.95f, 0.48f, 1.0f)
                               : ImVec4(1.0f, 0.62f, 0.20f, 1.0f),
                           "%s", g_libraryPlacement.status().c_str());
        ImGui::SameLine();
        ImGui::SetCursorPosX(std::max(ImGui::GetCursorPosX(),
                                      ImGui::GetContentRegionMax().x - 82.0f));
        if (ImGui::Button("Cancel")) cancelLibraryRobotPlacement();
        return;
    }

    if (g_scene.stationMode == StationMode::Layout) {
        ImGui::AlignTextToFramePadding();
        ImGui::TextDisabled("PLACEMENT");
        ImGui::SameLine(0.0f, 12.0f);
        contextToggleButton("Move objects", &g_scene.moveObjects);
        ImGui::SameLine(0.0f, 18.0f);
        ImGui::AlignTextToFramePadding();
        ImGui::TextDisabled("VIEW GUIDES");
        ImGui::SameLine(0.0f, 12.0f);
        contextToggleButton("Visual meshes", &view.showVisualMeshes);
        ImGui::SameLine();
        contextToggleButton("Mounting holes", &view.showMountingHoles);
        ImGui::SameLine();
        contextToggleButton("Conveyor flow", &view.showConveyorDirections);
        ImGui::SameLine();
        contextToggleButton("Pivot points", &view.showDragChainPivots);
        ImGui::SameLine();
        contextToggleButton("Hull overlay", &view.showHullOverlay);
        ImGui::SameLine();
        contextToggleButton("Tool gimbal", &view.showToolGimbal);
        ImGui::SameLine();
        contextToggleButton("Path preview", &view.showPathPreview);
        ImGui::SameLine(0.0f, 18.0f);
        if (ImGui::Button("Frame cell") && g_scene.viewer) {
            settleCameraTween();
            g_scene.viewer->reframeCamera();
            view.framedRobot = -1;
            view.camera = g_scene.viewer->camera();
        }
        return;
    }

    ImGui::AlignTextToFramePadding();
    ImGui::TextDisabled("PROGRAM");
    ImGui::SameLine(0.0f, 12.0f);
    ImGui::TextUnformatted(activeRobotDisplayName().c_str());
    ImGui::SameLine();
    ImGui::TextDisabled("/ %s", activeRobot().program().name.c_str());
    ImGui::SameLine(0.0f, 18.0f);
    ImGui::AlignTextToFramePadding();
    ImGui::TextDisabled("VIEW GUIDES");
    ImGui::SameLine(0.0f, 12.0f);
    contextToggleButton("Visual meshes", &view.showVisualMeshes);
    ImGui::SameLine();
    contextToggleButton("Conveyor flow", &view.showConveyorDirections);
    ImGui::SameLine();
    contextToggleButton("Program path", &view.showPathPreview);
    ImGui::SameLine();
    contextToggleButton("Simulation path", &view.showSimPath);
    ImGui::SameLine();
    contextToggleButton("Tool gimbal", &view.showToolGimbal);
    ImGui::SameLine();
    contextToggleButton("Joint axes", &view.showJointAxes);
    ImGui::SameLine(0.0f, 18.0f);
    if (ImGui::Button("Work on robot")) enterRobotView(g_scene.activeRobotIndex());
}

void drawRobotContextShelf() {
    ViewState& view = g_scene.robotView;
    ImGui::AlignTextToFramePadding();
    ImGui::TextDisabled("ROBOT CONTROL");
    ImGui::SameLine(0.0f, 12.0f);
    if (ImGui::Button("Home")) {
        activeRobot().poseController.resetHome();
        activeRobot().toolTargetPose = activeRobot().poseController.toolPose();
        activeRobot().toolWprBasePose = activeRobot().toolTargetPose;
        activeRobot().toolWprDegrees = {{0.0, 0.0, 0.0}};
    }
    ImGui::SameLine();
    if (ImGui::Button("Frame robot") && g_scene.viewer) {
        settleCameraTween();
        g_scene.viewer->reframeCameraOn(activeRobot().poseController.robotNode());
        view.camera = g_scene.viewer->camera();
        view.framedRobot = g_scene.activeRobotIndex();
    }
    ImGui::SameLine(0.0f, 18.0f);
    ImGui::AlignTextToFramePadding();
    ImGui::TextDisabled("VIEW GUIDES");
    ImGui::SameLine(0.0f, 12.0f);
    contextToggleButton("Visual meshes", &view.showVisualMeshes);
    ImGui::SameLine();
    contextToggleButton("Tool gimbal", &view.showToolGimbal);
    ImGui::SameLine();
    contextToggleButton("Joint axes", &view.showJointAxes);
    ImGui::SameLine();
    contextToggleButton("Hull overlay", &view.showHullOverlay);
    ImGui::SameLine();
    contextToggleButton("Path preview", &view.showPathPreview);
    ImGui::SameLine();
    contextToggleButton("Sim path", &view.showSimPath);
    ImGui::SameLine();
    contextToggleButton("Pivot points", &view.showDragChainPivots);
    ImGui::SameLine();
    contextToggleButton("Mounting holes", &view.showMountingHoles);
}

void drawPersistentContextBar() {
    const bool robotContext = g_scene.view.kind == ViewTarget::Kind::Robot;
    const ImVec4 background = robotContext ? kContextFrameColor
                                            : ImVec4(0.105f, 0.12f, 0.145f, 1.0f);
    ImGui::PushStyleColor(ImGuiCol_ChildBg, background);
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.94f, 0.96f, 0.99f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_TextDisabled, ImVec4(0.62f, 0.69f, 0.78f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.18f, 0.21f, 0.26f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.25f, 0.30f, 0.37f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.30f, 0.37f, 0.46f, 1.0f));
    // The application panels use a light theme. Combo boxes and their selectable rows inherit
    // those frame/header colors unless the complete control palette is overridden here, producing
    // light toolbar text on an almost-white selection as soon as the conveyor menu opens.
    ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0.15f, 0.18f, 0.22f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_FrameBgHovered, ImVec4(0.21f, 0.26f, 0.32f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_FrameBgActive, ImVec4(0.25f, 0.31f, 0.39f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_Header, ImVec4(0.23f, 0.35f, 0.49f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_HeaderHovered, ImVec4(0.28f, 0.43f, 0.61f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_HeaderActive, ImVec4(0.32f, 0.49f, 0.70f, 1.0f));
    // Tooltips are submitted while this palette is active.  Give them the same dark surface so
    // the bar's light text does not become white-on-white against the application's light popup.
    ImGui::PushStyleColor(ImGuiCol_PopupBg, ImVec4(0.13f, 0.15f, 0.19f, 0.98f));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(16.0f, 7.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(13.0f, 7.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(9.0f, 6.0f));
    ImGui::BeginChild("persistentContextBar", ImVec2(0.0f, persistentContextBarHeight()), false,
                      ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
    ImGui::SetWindowFontScale(1.22f);

    if (!robotContext) {
        const char* stationName = !g_scene.station.name.empty()
            ? g_scene.station.name.c_str() : g_scene.packageLabel.c_str();
        ImGui::AlignTextToFramePadding();
        ImGui::TextDisabled("STATION");
        ImGui::SameLine(0.0f, 10.0f);
        ImGui::TextUnformatted(stationName);
        ImGui::SameLine(0.0f, 24.0f);
        const bool executionActive = stationExecutionActive();
        stationModeButton("Layout", StationMode::Layout,
                          executionActive || g_libraryPlacement.active());
        ImGui::SameLine(0.0f, 2.0f);
        stationModeButton("Program", StationMode::Program, g_libraryPlacement.active());
        ImGui::SameLine(0.0f, 18.0f);
        ImGui::AlignTextToFramePadding();
        ImGui::TextColored(ImVec4(0.72f, 0.80f, 0.90f, 1.0f), "%s",
                           stationContextStatus().c_str());

        ImGui::SameLine();
        ImGui::SetCursorPosX(std::max(ImGui::GetCursorPosX(),
                                      ImGui::GetContentRegionMax().x - 540.0f));
        ImGui::AlignTextToFramePadding();
        ImGui::TextDisabled("SIMULATION");
        ImGui::SameLine(0.0f, 10.0f);
        drawStationExecutionCluster();
    } else {
        if (ImGui::Button("<  Station")) exitToStationView();
        ImGui::SameLine();
        ImGui::AlignTextToFramePadding();
        ImGui::Text("/ %s", activeRobotDisplayName().c_str());
        drawHardwareMarker(activeRobot());
        if (g_scene.robots.size() > 1) {
            ImGui::SameLine(0.0f, 16.0f);
            const int index = g_scene.activeRobotIndex();
            ImGui::BeginDisabled(index <= 0);
            if (ImGui::ArrowButton("##previousRobot", ImGuiDir_Left)) enterRobotView(index - 1);
            ImGui::EndDisabled();
            ImGui::SameLine();
            ImGui::BeginDisabled(index + 1 >= static_cast<int>(g_scene.robots.size()));
            if (ImGui::ArrowButton("##nextRobot", ImGuiDir_Right)) enterRobotView(index + 1);
            ImGui::EndDisabled();
        }
        ImGui::SameLine();
        ImGui::SetCursorPosX(std::max(ImGui::GetCursorPosX(),
                                      ImGui::GetContentRegionMax().x - 250.0f));
        ImGui::AlignTextToFramePadding();
        ImGui::TextDisabled("Low-level jog, program and I/O");
    }

    ImGui::Separator();
    if (robotContext) drawRobotContextShelf();
    else drawStationContextShelf();

    ImGui::EndChild();
    ImGui::PopStyleVar(3);
    ImGui::PopStyleColor(13);
}

