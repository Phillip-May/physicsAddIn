#include "PlacementWindows.h"

#include "OrientationFormat.h"
#include "UiFormHelpers.h"
#include "ViewerBridge.h"

#include "imgui.h"

#include <algorithm>
#include <array>
#include <string>
void drawBasePlacementEditors(CadNode* placementNode, RobotInstance& robot) {
    if (!placementNode) return;

    const int formatIndex =
        std::max(0, std::min(robot.placementOrientationFormat, orientation::formatCount() - 1));
    const orientation::Format& format = orientation::formats()[formatIndex];

    ImGui::SetNextItemWidth(-1.0f);
    if (ImGui::BeginCombo("##orientationformat", format.label)) {
        for (int i = 0; i < orientation::formatCount(); ++i) {
            const bool selected = i == formatIndex;
            if (ImGui::Selectable(orientation::formats()[i].label, selected)) {
                // Only how the same rotation is read out. The stored placement is a matrix and is
                // untouched, so switching format cannot move the arm.
                robot.placementOrientationFormat = i;
            }
            if (selected) ImGui::SetItemDefaultFocus();
        }
        ImGui::EndCombo();
    }

    static const char* const kPositionLabels[3] = {"X", "Y", "Z"};
    const int axisOffsets[3] = {3, 7, 11};
    double position[3] = {placementNode->loc.values[axisOffsets[0]],
                          placementNode->loc.values[axisOffsets[1]],
                          placementNode->loc.values[axisOffsets[2]]};
    bool moved = vectorSpinRow("pos", kPositionLabels, position, 3, 10.0, "%.1f mm");
    if (moved) {
        for (int axis = 0; axis < 3; ++axis) placementNode->loc.values[axisOffsets[axis]] = position[axis];
    }

    std::array<double, 4> angles = orientation::valuesFromRotation(format, placementNode->loc);
    if (vectorSpinRow("rot", format.fieldLabels, angles.data(), format.fieldCount, format.step,
                      format.fieldFormat)) {
        const CadTransform rotation = orientation::rotationFromValues(format, angles);
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                placementNode->loc.values[static_cast<size_t>(row * 4 + column)] =
                    rotation.values[static_cast<size_t>(row * 4 + column)];
            }
        }
        moved = true;
    }

    if (moved) {
        // The arm's own joints have not changed, so the pose controller needs nothing; only the
        // readouts that quote a world position do.
        placementNode->needsGlobalLocUpdate = true;
        robot.readoutJointsValid = false;
        // Both path previews are held as world points, baked from the placement when they were last
        // built, so moving the arm without rebuilding them leaves the paths behind at the old stand
        // - the arm walks away from its own programme. Cheap enough to do on the drag: this is one
        // pass over the targets and the samples, and it only runs on the frames a number changed.
        refreshProgramPathPreview();
        refreshSimPathPreview();
    }
    ImGui::TextDisabled("Drag it in 3D with Move objects, in the cell panel.");
}

// The row's own way in to its placement editors. Seven numbers and a format combo is a panel, and
// inline in the tree it pushed every arm below it off the bottom of a list whose job is to show
// the arms at a glance.
void drawPlacementToggle(RobotInstance& robot) {
    ImGui::SameLine();
    if (ImGui::SmallButton("Position...")) robot.placementWindowOpen = !robot.placementWindowOpen;
}

// The placement editors in a window of their own, one per arm, opened from that arm's row.
void drawPlacementWindow(int robotIndex) {
    RobotInstance& robot = *g_scene.robots[static_cast<size_t>(robotIndex)];
    if (!robot.placementWindowOpen) return;
    CadNode* const placementNode = placementNodeFor(robot);
    if (!placementNode) {
        robot.placementWindowOpen = false;
        return;
    }

    // Titled for what it places, with the index behind ### as the identity: the visible half is a
    // node name and renaming one must not read as a different window that has forgotten its size
    // and position.
    const CadNode* const node = robot.poseController.robotNode();
    const std::string name = !placementNode->name.empty()
        ? placementNode->name
        : (node && !node->name.empty() ? node->name : "Robot " + std::to_string(robotIndex + 1));
    const std::string title = name + " position###baseposition" + std::to_string(robotIndex);

    const ImVec2 mouse = ImGui::GetIO().MousePos;
    ImGui::SetNextWindowPos(ImVec2(mouse.x + 12.0f, mouse.y), ImGuiCond_Appearing);
    // Fix the width to avoid a content/window auto-size cycle; height may auto-fit.
    ImGui::SetNextWindowSize(ImVec2(520.0f, 0.0f), ImGuiCond_FirstUseEver);
    if (ImGui::Begin(title.c_str(), &robot.placementWindowOpen, ImGuiWindowFlags_NoDocking)) {
        drawBasePlacementEditors(placementNode, robot);
    }
    ImGui::End();
}

void drawPlacementWindows() {
    for (int i = 0; i < static_cast<int>(g_scene.robots.size()); ++i) {
        if (g_scene.robots[static_cast<size_t>(i)]) drawPlacementWindow(i);
    }
}

