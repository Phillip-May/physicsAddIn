#include "ProgramPanel.h"

#include "AppState.h"
#include "ProgramEditing.h"
#include "ProgramWidgets.h"
#include "TrajectoryAndLiveRun.h"
#include "UiFormHelpers.h"

#include "imgui.h"
void drawProgramPanel() {
    if (!g_scene.viewer) {
        ImGui::TextDisabled("No robot loaded.");
        return;
    }
    const bool editing = !activeRobot().buildRunning && !activeRobot().playing;

    drawProgramList();

    drawProgramInstructionEditor(activeRobot(), 8.5f);

    if (formButton("Simulate", 2, !editing)) beginTrajectoryBuild(true);
    ImGui::SameLine();
    if (formButton("Stop", 2, editing)) {
        cancelTrajectoryBuild();
        activeRobot().playing = false;
    }

    ImGui::Spacing();
    drawProgramState(activeRobot(), editing, "robotProgramState");

    if (!activeRobot().buildStatus.empty()) {
        ImGui::Spacing();
        ImGui::TextWrapped("%s", activeRobot().buildStatus.c_str());
    }

    ImGui::Spacing();
}

