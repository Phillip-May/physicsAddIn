#include "RobotPanel.h"

#include "AppState.h"
#include "ProgramTextIo.h"
#include "StationSceneLoad.h"
#include "UiFormHelpers.h"
#include "ViewerBridge.h"
#include "WebFiles.h"

#include "imgui.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
// Joint jog, tool target and display toggles.
void drawRobotPanel() {
    if (!g_scene.viewer) {
        ImGui::TextDisabled("No robot loaded.");
        return;
    }

    ImGui::PushTextWrapPos(0.0f);
    ImGui::TextUnformatted(g_scene.packageLabel.c_str());
    ImGui::PopTextWrapPos();
    // Only shown when the scene actually holds more than one arm, so a single-robot package reads
    // exactly as it did before. Which one is chosen in the sim view.
    if (g_scene.robots.size() > 1) {
        ImGui::TextDisabled("Robot %d of %d", g_scene.activeRobotIndex() + 1,
                            static_cast<int>(g_scene.robots.size()));
    }
    ImGui::Separator();

    const OPW6RobotData* robot = activeRobot().poseController.robotData();
    std::array<double, 6> joints = activeRobot().poseController.joints();
    bool jointsEdited = false;

    // Jn label, then a bare slider, then the spin box.
    // The slider carries no value text: SliderFloat prints the number inside the groove by
    // default, which made the row read as two value boxes instead of a slider plus a field.
    for (int i = 0; i < 6; ++i) {
        const size_t index = static_cast<size_t>(i);
        float degrees = static_cast<float>(joints[index] * kRadToDeg);
        float minDeg = -180.0f;
        float maxDeg = 180.0f;
        if (robot && robot->qMin[index] < robot->qMax[index]) {
            minDeg = static_cast<float>(robot->qMin[index] * kRadToDeg);
            maxDeg = static_cast<float>(robot->qMax[index] * kRadToDeg);
        }

        ImGui::PushID(i);
        ImGui::AlignTextToFramePadding();
        ImGui::Text("J%d", i + 1);
        ImGui::SameLine(kFormLabelWidth);

        const float spinBoxWidth = 92.0f;
        const float arrowsWidth = std::floor(ImGui::GetFrameHeight() * 0.62f) + 2.0f;
        ImGui::SetNextItemWidth(std::max(40.0f,
            ImGui::GetContentRegionAvail().x - spinBoxWidth - arrowsWidth - ImGui::GetStyle().ItemSpacing.x));
        if (ImGui::SliderFloat("##jointSlider", &degrees, minDeg, maxDeg, "")) {
            joints[index] = static_cast<double>(degrees) * kDegToRad;
            jointsEdited = true;
        }
        ImGui::SameLine();
        double spinValue = static_cast<double>(degrees);
        if (nativeSpinBox("##jointSpin", &spinValue, 1.0, "%.2f deg",
                          static_cast<double>(minDeg), static_cast<double>(maxDeg), spinBoxWidth)) {
            joints[index] = spinValue * kDegToRad;
            jointsEdited = true;
        }
        ImGui::PopID();
    }

    if (jointsEdited) {
        activeRobot().poseController.setJoints(joints);
        activeRobot().toolTargetPose = activeRobot().poseController.toolPose();
        activeRobot().toolWprBasePose = activeRobot().toolTargetPose;
        activeRobot().toolWprDegrees = {{0.0, 0.0, 0.0}};
    }

    ImGui::Spacing();
    const float halfWidth = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) * 0.5f;
    if (ImGui::Button("Home", ImVec2(halfWidth, 0.0f))) {
        activeRobot().poseController.resetHome();
        activeRobot().toolTargetPose = activeRobot().poseController.toolPose();
        activeRobot().toolWprBasePose = activeRobot().toolTargetPose;
        activeRobot().toolWprDegrees = {{0.0, 0.0, 0.0}};
    }
    ImGui::SameLine();
    if (ImGui::Button("Reframe", ImVec2(halfWidth, 0.0f))) {
        settleCameraTween();
        g_scene.viewer->reframeCameraOn(activeRobot().poseController.robotNode());
        g_scene.robotView.camera = g_scene.viewer->camera();
        g_scene.robotView.framedRobot = g_scene.activeRobotIndex();
    }
    // No way out here: the context header above the dockspace carries it, and two of them would
    // only raise the question of how they differ.

    ImGui::Spacing();
    ImGui::TextUnformatted("Tool target");
    ImGui::Separator();
    double translation[3] = {activeRobot().toolTargetPose.values[3],
                            activeRobot().toolTargetPose.values[7],
                            activeRobot().toolTargetPose.values[11]};
    bool poseEdited = false;
    const char* translationLabels[3] = {"X", "Y", "Z"};
    for (int i = 0; i < 3; ++i) {
        if (nativeSpinBox(translationLabels[i], &translation[i], 1.0, "%.2f mm", 0.0, 0.0, 104.0f,
                          translationLabels[i])) poseEdited = true;
    }

    bool rotationEdited = false;
    const char* rotationLabels[3] = {"W", "P", "R"};
    for (int i = 0; i < 3; ++i) {
        if (nativeSpinBox(rotationLabels[i], &activeRobot().toolWprDegrees[static_cast<size_t>(i)], 1.0,
                          "%.2f deg", 0.0, 0.0, 104.0f, rotationLabels[i])) {
            rotationEdited = true;
        }
    }

    if (poseEdited || rotationEdited) {
        // Rotation is rebuilt from the captured base pose so the three fields stay independent.
        CadTransform target = rotationEdited
            ? composeLocalWpr(activeRobot().toolWprBasePose, activeRobot().toolWprDegrees)
            : activeRobot().toolTargetPose;
        target.values[3] = translation[0];
        target.values[7] = translation[1];
        target.values[11] = translation[2];

        std::string ikError;
        if (activeRobot().poseController.setToolPose(target, &ikError)) {
            activeRobot().toolTargetPose = activeRobot().poseController.toolPose();
            activeRobot().statusText.clear();
        } else {
            activeRobot().toolTargetPose = target;
            activeRobot().statusText = ikError;
        }
    }

    // Solution selector: every IK branch for the current target, so a different elbow/wrist
    // configuration can be chosen explicitly.
    if (!activeRobot().toolSolutions.empty()) {
        ImGui::SetNextItemWidth(-1.0f);
        const std::string preview = activeRobot().toolSolutionLabels[static_cast<size_t>(
            std::max(0, std::min(activeRobot().selectedToolSolution,
                                 static_cast<int>(activeRobot().toolSolutionLabels.size()) - 1)))];
        if (ImGui::BeginCombo("##solution", preview.c_str())) {
            for (size_t i = 0; i < activeRobot().toolSolutionLabels.size(); ++i) {
                const bool selected = static_cast<int>(i) == activeRobot().selectedToolSolution;
                if (ImGui::Selectable(activeRobot().toolSolutionLabels[i].c_str(), selected)) {
                    activeRobot().selectedToolSolution = static_cast<int>(i);
                    activeRobot().poseController.setJoints(activeRobot().toolSolutions[i]);
                    activeRobot().toolTargetPose = activeRobot().poseController.toolPose();
                }
                if (selected) ImGui::SetItemDefaultFocus();
            }
            ImGui::EndCombo();
        }
    }

    ImGui::Spacing();
    ImGui::TextUnformatted(activeRobot().collisionText.c_str());

    if (!activeRobot().statusText.empty()) {
        ImGui::Spacing();
        ImGui::TextWrapped("%s", activeRobot().statusText.c_str());
    }

    ImGui::Spacing();
    if (ImGui::Button("Open...", ImVec2(-1.0f, 0.0f))) {
#ifdef __EMSCRIPTEN__
        WebFiles::requestOpen(WebFiles::Purpose::Package, ".zip,.json");
        activeRobot().statusText = "Choose a robot package...";
#else
        const std::string packageFile = runPackageDialog();
        if (!packageFile.empty()) loadPackageIntoScene(packageFile);
#endif
    }
}

