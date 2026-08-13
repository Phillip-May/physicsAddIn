#include "ProgramWidgets.h"

#include "AppState.h"
#include "ProgramEditing.h"
#include "RobotProgramModel.h"
#include "StationSceneLoad.h"
#include "UiFormHelpers.h"
#include "ViewerBridge.h"

#include "imgui.h"

#include <algorithm>
#include <array>
#include <string>
#include <vector>
bool drawActiveToolState(RobotInstance& robot, const char* comboId, bool editing) {
    std::vector<RobotToolChoice> choices = robotToolChoices(robot);
    OPW6RobotData* robotData = robot.poseController.robotData();
    int activeChoice = -1;
    for (int index = 0; index < static_cast<int>(choices.size()); ++index) {
        const RobotToolChoice& choice = choices[static_cast<size_t>(index)];
        if (robotData && robotData->activeTool == choice.node &&
            choice.data->activeTcpIndex == choice.tcpIndex) {
            activeChoice = index;
            break;
        }
    }
    const char* preview = activeChoice >= 0
        ? choices[static_cast<size_t>(activeChoice)].label.c_str() : "No active tool";

    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted("Active tool");
    ImGui::SetNextItemWidth(-1.0f);
    ImGui::BeginDisabled(!editing || choices.empty());
    bool changed = false;
    if (ImGui::BeginCombo(comboId, preview)) {
        for (int index = 0; index < static_cast<int>(choices.size()); ++index) {
            const bool selected = index == activeChoice;
            if (ImGui::Selectable(choices[static_cast<size_t>(index)].label.c_str(), selected)) {
                changed = activateRobotToolChoice(robot, choices[static_cast<size_t>(index)]);
            }
            if (selected) ImGui::SetItemDefaultFocus();
        }
        ImGui::EndCombo();
    }
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Select the active physical tool and TCP used by FK, IK, MoveL and path previews.");
    }
    return changed;
}

void drawProgramState(RobotInstance& robot, bool editing, const char* id) {
    ImGui::PushID(id);
    ImGui::TextUnformatted("State");
    drawActiveToolState(robot, "##activeProgramTool", editing);
    ImGui::BeginDisabled(!editing);
    nativeSpinBox("##joint", &robot.programJointSpeedDegPerSec, 1.0, "%.1f deg/s",
                  0.1, 5000.0, 130.0f, "Joint");
    nativeSpinBox("##linear", &robot.programLinearSpeedMmPerSec, 1.0, "%.1f mm/s",
                  0.1, 1000000.0, 130.0f, "Cartesian");
    nativeSpinBox("##blend", &robot.programBlendMm, 0.1, "%.2f mm",
                  -1.0, 1000.0, 130.0f, "Blend");
    ImGui::EndDisabled();
    ImGui::PopID();
}

void drawProgramInstructionTable(RobotInstance& robot, float tableHeight, bool editing) {
    // Marker rows come from the firmware run, and are -1 when no program is running.
    const int pendingRow = robot.hardware.programPendingRow();
    const int executingRow = robot.hardware.programExecutingRow();
    int deleteRow = -1;
    bool selectedRowFocused = false;
    ImGui::PushID(&robot);
    if (ImGui::BeginTable("programInstructions", 3,
                          ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                              ImGuiTableFlags_ScrollY,
                          ImVec2(0.0f, tableHeight))) {
        ImGui::TableSetupColumn("##run", ImGuiTableColumnFlags_WidthFixed, 32.0f);
        ImGui::TableSetupColumn("Instruction");
        ImGui::TableSetupColumn("Target / Value");
        ImGui::TableHeadersRow();
        for (int row = 0; row < static_cast<int>(robot.program().root->children.size()); ++row) {
            const auto& child = robot.program().root->children[static_cast<size_t>(row)];
            if (!child) continue;
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            drawProgramRunMarkers(row == pendingRow, row == executingRow);
            ImGui::TableSetColumnIndex(1);
            // PushID per row: ImGui derives a widget's identity from its label, and a program
            // is mostly repeated "MoveL" entries, so without this every such row shares one ID.
            ImGui::PushID(row);
            const std::string label = robotProgramNodeLabel(child->type);
            if (ImGui::Selectable(label.c_str(), robot.selectedInstruction == row,
                                  ImGuiSelectableFlags_SpanAllColumns)) {
                robot.selectedInstruction = row;
                if (editing) previewProgramMoveTarget(robot, row);
            }
            if (robot.selectedInstruction == row && ImGui::IsItemFocused()) {
                selectedRowFocused = true;
            }
            if (ImGui::IsItemClicked(ImGuiMouseButton_Right)) {
                robot.selectedInstruction = row;
                if (editing) previewProgramMoveTarget(robot, row);
            }
            if (ImGui::BeginPopupContextItem("instructionContext")) {
                robot.selectedInstruction = row;
                if (ImGui::MenuItem("Delete instruction", "Delete", false, editing)) {
                    deleteRow = row;
                }
                ImGui::EndPopup();
            }
            ImGui::PopID();
            ImGui::TableSetColumnIndex(2);
            ImGui::TextUnformatted(robotProgramNodeDetail(*child).c_str());
        }
        ImGui::EndTable();
    }
    if (editing && selectedRowFocused && !ImGui::GetIO().WantTextInput &&
        ImGui::IsKeyPressed(ImGuiKey_Delete, false)) {
        deleteRow = robot.selectedInstruction;
    }
    ImGui::PopID();
    if (deleteRow >= 0) deleteProgramInstruction(robot, deleteRow);
}

bool hasSelectedEditableMove(RobotInstance& robot) {
    return editableProgramMoveAt(robot, robot.selectedInstruction) != nullptr;
}

void drawSelectedMoveTargetEditor(RobotInstance& robot, bool editing) {
    RobotProgramNode* node = editableProgramMoveAt(robot, robot.selectedInstruction);
    if (!node) return;
    std::array<double, 6>* target = editableMoveTargetJointsForNode(*node);
    if (!target) return;

    ImGui::SeparatorText(node->type == RobotProgramNodeType::MoveJ
                             ? "Selected MoveJ target" : "Selected MoveL target");
    ImGui::TextDisabled("Drag the active TCP gimbal in the 3D view to edit this instruction.");

    ViewState& viewState = g_scene.activeViewState();
    if (!viewState.showToolGimbal) {
        if (ImGui::Button("Show 3D gimbal", ImVec2(-1.0f, 0.0f))) {
            viewState.showToolGimbal = true;
        }
    } else {
        const CadTransform& tcp = robot.toolTargetPose;
        ImGui::Text("TCP  X %.1f   Y %.1f   Z %.1f mm",
                    tcp.values[3], tcp.values[7], tcp.values[11]);
    }

    bool jointsChanged = false;
    ImGui::BeginDisabled(!editing);
    if (ImGui::BeginTable("selectedMoveJoints", 3, ImGuiTableFlags_SizingStretchSame)) {
        for (int joint = 0; joint < 6; ++joint) {
            ImGui::TableNextColumn();
            ImGui::PushID(joint);
            double degrees = (*target)[static_cast<size_t>(joint)] * kRadToDeg;
            ImGui::SetNextItemWidth(-1.0f);
            const std::string label = "J" + std::to_string(joint + 1);
            const std::string format = label + " %.2f deg";
            if (ImGui::DragScalar(("##" + label).c_str(), ImGuiDataType_Double, &degrees,
                                  0.25f, nullptr, nullptr, format.c_str())) {
                (*target)[static_cast<size_t>(joint)] = degrees * kDegToRad;
                jointsChanged = true;
            }
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s target", label.c_str());
            ImGui::PopID();
        }
        ImGui::EndTable();
    }
    bool* hasExternalAxis = nullptr;
    double* externalAxisMm = nullptr;
    editableMoveExternalAxisForNode(*node, &hasExternalAxis, &externalAxisMm);
    const int linkedGantry = linkedGantryIndexForRobot(liveRunArmIndexOf(robot));
    if (linkedGantry >= 0 && hasExternalAxis && externalAxisMm) {
        if (ImGui::Checkbox("Program linked J7", hasExternalAxis)) jointsChanged = true;
        ImGui::BeginDisabled(!*hasExternalAxis);
        const GantryMechanismData* data =
            g_scene.gantries[static_cast<size_t>(linkedGantry)].gantryData();
        const double minimum = data ? data->lowerLimitMm : 0.0;
        const double maximum = data ? data->upperLimitMm : 0.0;
        ImGui::SetNextItemWidth(-1.0f);
        if (ImGui::DragScalar("##linkedJ7", ImGuiDataType_Double, externalAxisMm,
                              1.0f, &minimum, &maximum, "J7 %.1f mm")) {
            jointsChanged = true;
        }
        ImGui::EndDisabled();
    }
    if (jointsChanged) {
        previewProgramMoveTarget(robot, robot.selectedInstruction);
        refreshAfterProgramEdit(robot, robot.selectedInstruction, "Move target updated");
    }

    if (formButton("Preview target", 2)) {
        previewProgramMoveTarget(robot, robot.selectedInstruction);
    }
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Pose the simulated robot at this instruction without changing the program.");
    }
    ImGui::SameLine();
    if (formButton("Capture current pose", 2)) {
        *target = robot.poseController.joints();
        if (linkedGantry >= 0 && hasExternalAxis && externalAxisMm) {
            *hasExternalAxis = true;
            *externalAxisMm = g_scene.gantries[static_cast<size_t>(linkedGantry)].positionMm();
        }
        robot.toolTargetPose = robot.poseController.toolPose();
        robot.toolWprBasePose = robot.toolTargetPose;
        robot.toolWprDegrees = {{0.0, 0.0, 0.0}};
        refreshAfterProgramEdit(robot, robot.selectedInstruction, "Move target captured");
    }
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Replace this instruction target with the robot's current pose.");
    }
    ImGui::EndDisabled();
}

// Shared by Robot view and Station > Program. Both surfaces edit the same instruction tree through
// the same insertion, selection and deletion code, so switching views cannot change semantics.
void drawProgramInstructionEditor(RobotInstance& robot, float reservedRowsAfterEditor) {
    const bool editing = !robot.buildRunning && !robot.playing && !robot.liveRun.running &&
        !robot.hardware.programActive();
    const std::string programName = robot.program().name.empty() ? "Program" : robot.program().name;
    ImGui::Text("Instructions - %s", programName.c_str());
    if (!editing) {
        ImGui::SameLine();
        ImGui::TextDisabled("(stop execution to edit)");
    }

    // Two compact action rows live below the scrolling table. The caller reserves any additional
    // rows it needs for its own controls (simulation and State in Robot view; none in Station).
    const bool selectedMove = hasSelectedEditableMove(robot);
    const float actionRowsHeight = ImGui::GetFrameHeightWithSpacing() * 2.0f;
    const float targetEditorHeight = selectedMove
        ? ImGui::GetFrameHeightWithSpacing() * 6.5f : 0.0f;
    const float reservedHeight = ImGui::GetFrameHeightWithSpacing() * reservedRowsAfterEditor;
    const float tableHeight = std::max(
        90.0f, ImGui::GetContentRegionAvail().y - actionRowsHeight - targetEditorHeight -
                   reservedHeight);
    drawProgramInstructionTable(robot, tableHeight, editing);

    drawSelectedMoveTargetEditor(robot, editing);

    const std::array<double, 6> currentPose = robot.poseController.joints();
    const int linkedGantry = linkedGantryIndexForRobot(liveRunArmIndexOf(robot));
    const bool hasLinkedGantry = linkedGantry >= 0;
    const double currentJ7 = hasLinkedGantry
        ? g_scene.gantries[static_cast<size_t>(linkedGantry)].positionMm() : 0.0;
    ImGui::BeginDisabled(!editing);
    if (formButton("+ MoveJ", 5)) {
        insertProgramInstruction(robot, RobotProgramNodeType::MoveJ,
                                 MoveJData{currentPose, hasLinkedGantry, currentJ7});
    }
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Add a joint move to the current simulated pose.");
    ImGui::SameLine();
    if (formButton("+ MoveL", 5)) {
        insertProgramInstruction(robot, RobotProgramNodeType::MoveL,
                                 MoveLData{currentPose, hasLinkedGantry, currentJ7});
    }
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Add a linear move to the current simulated pose.");
    ImGui::SameLine();
    if (formButton("+ Speed", 5)) {
        insertProgramInstruction(
            robot, RobotProgramNodeType::SetSpeed,
            SetSpeedData{robot.programJointSpeedDegPerSec * kDegToRad,
                         robot.programLinearSpeedMmPerSec});
    }
    ImGui::SameLine();
    if (formButton("+ Blend", 5)) {
        insertProgramInstruction(robot, RobotProgramNodeType::SetBlending,
                                 SetBlendingData{robot.programBlendMm});
    }
    ImGui::SameLine();
    if (formButton("+ Actuator", 5)) ImGui::OpenPopup("addActuatorInstruction");
    if (ImGui::BeginPopup("addActuatorInstruction")) {
        CadNode* activeTool = robot.poseController.robotData()
            ? robot.poseController.robotData()->activeTool : nullptr;
        std::string toolId;
        for (const StationAccessoryInstance& accessory : g_scene.station.accessories) {
            if (accessory.node == activeTool) toolId = accessory.id;
        }
        RobotToolData* tool = activeTool ? activeTool->asRobotTool() : nullptr;
        if (!tool || toolId.empty() || tool->actuators.empty()) {
            ImGui::TextDisabled("Active tool has no programmable actuators.");
        } else {
            for (const MechanismActuatorData& actuator : tool->actuators) {
                ImGui::PushID(actuator.id.c_str());
                if (ImGui::MenuItem((actuator.name + " — Open").c_str())) {
                    insertProgramInstruction(robot, RobotProgramNodeType::Actuate,
                        ActuateData{toolId, actuator.id, actuator.lowerLimit});
                }
                if (ImGui::MenuItem((actuator.name + " — Close").c_str())) {
                    insertProgramInstruction(robot, RobotProgramNodeType::Actuate,
                        ActuateData{toolId, actuator.id, actuator.upperLimit});
                }
                ImGui::PopID();
            }
        }
        ImGui::EndPopup();
    }

    const bool addTool = formButton("+ Tool", 3);
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Select the active tool and TCP for following instructions.");
    }
    if (addTool) ImGui::OpenPopup("addToolInstruction");
    if (ImGui::BeginPopup("addToolInstruction")) {
        for (const RobotToolChoice& choice : robotToolChoices(robot)) {
            std::string toolId;
            for (const StationAccessoryInstance& accessory : g_scene.station.accessories) {
                if (accessory.node == choice.node) {
                    toolId = accessory.id;
                    break;
                }
            }
            if (toolId.empty()) continue;
            if (ImGui::MenuItem(choice.label.c_str())) {
                insertProgramInstruction(robot, RobotProgramNodeType::SetTool,
                                         SetToolData{toolId, choice.tcpIndex});
            }
        }
        ImGui::EndPopup();
    }
    ImGui::SameLine();
    if (formButton("+ Stop", 3)) {
        insertProgramInstruction(robot, RobotProgramNodeType::Stop, std::monostate{});
    }
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("End this program after the preceding move, even when Station Start requests looping.");
    }
    ImGui::SameLine();

    const bool validSelection = robot.program().root && robot.selectedInstruction >= 0 &&
        robot.selectedInstruction < static_cast<int>(robot.program().root->children.size());
    if (formButton("Delete instruction", 3, !validSelection)) {
        deleteProgramInstruction(robot, robot.selectedInstruction);
    }
    ImGui::EndDisabled();
}

