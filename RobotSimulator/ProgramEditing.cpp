#include "ProgramEditing.h"
#include "StringUtil.h"

#include "FirmwareProgram.h"
#include "LiveRunDriver.h"
#include "MasteringIo.h"
#include "ProgramTextIo.h"
#include "ConveyorRuntime.h"
#include "StationSceneLoad.h"
#include "SceneMath.h"
#include "TrajectoryAndLiveRun.h"
#include "UiFormHelpers.h"
#include "ViewerBridge.h"
#include "WebFiles.h"

#include "imgui.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

std::array<double, 6>* editableMoveTargetJointsForNode(RobotProgramNode& node) {
    switch (node.type) {
    case RobotProgramNodeType::MoveJ:
        return &std::get<MoveJData>(node.data).targetJoints;
    case RobotProgramNodeType::MoveL:
        return &std::get<MoveLData>(node.data).targetJoints;
    default:
        return nullptr;
    }
}

bool editableMoveExternalAxisForNode(RobotProgramNode& node,
                                     bool** hasExternalAxis,
                                     double** positionMm) {
    if (node.type == RobotProgramNodeType::MoveJ) {
        MoveJData& move = std::get<MoveJData>(node.data);
        if (hasExternalAxis) *hasExternalAxis = &move.hasExternalAxis;
        if (positionMm) *positionMm = &move.externalAxisPositionMm;
        return true;
    }
    if (node.type == RobotProgramNodeType::MoveL) {
        MoveLData& move = std::get<MoveLData>(node.data);
        if (hasExternalAxis) *hasExternalAxis = &move.hasExternalAxis;
        if (positionMm) *positionMm = &move.externalAxisPositionMm;
        return true;
    }
    return false;
}


std::string programTextForInstruction(const RobotProgramNode& node) {
    switch (node.type) {
    case RobotProgramNodeType::MoveJ: {
        const MoveJData& move = std::get<MoveJData>(node.data);
        return formatMoveLine("MoveJ", move.targetJoints,
                              move.hasExternalAxis, move.externalAxisPositionMm);
    }
    case RobotProgramNodeType::MoveL: {
        const MoveLData& move = std::get<MoveLData>(node.data);
        return formatMoveLine("MoveL", move.targetJoints,
                              move.hasExternalAxis, move.externalAxisPositionMm);
    }
    case RobotProgramNodeType::SetSpeed: {
        const SetSpeedData& speed = std::get<SetSpeedData>(node.data);
        return strutil::format("SetSpeed %1 %2")
            .arg(programNumber(speed.jointSpeedRadPerSec * kRadToDeg))
            .arg(programNumber(speed.linearSpeedMmPerSec));
    }
    case RobotProgramNodeType::SetBlending:
        return strutil::format("SetBlending %1").arg(programNumber(std::get<SetBlendingData>(node.data).radiusMm));
    case RobotProgramNodeType::SetWeave: {
        const RobotMotionCore::WeaveParams& weave = std::get<SetWeaveData>(node.data).params;
        std::string line = "SetWeave shape=";
        line += RobotMotionCore::weaveShapeName(weave.shape);
        if (weave.rateMode == RobotMotionCore::WeaveRateMode::Frequency) {
            line += " freq=" + programNumber(weave.frequencyHz);
        } else if (weave.rateMode == RobotMotionCore::WeaveRateMode::Wavelength) {
            line += " wavelength=" + programNumber(weave.wavelengthMm);
        }
        if (weave.amplitudeLeftMm == weave.amplitudeRightMm) {
            line += " amp=" + programNumber(weave.amplitudeLeftMm);
        } else {
            line += " amp_l=" + programNumber(weave.amplitudeLeftMm);
            line += " amp_r=" + programNumber(weave.amplitudeRightMm);
        }
        if (weave.elevationMm != 0.0) line += " elev=" + programNumber(weave.elevationMm);
        if (weave.planeAngleDeg != 0.0) line += " angle=" + programNumber(weave.planeAngleDeg);
        if (weave.biasMm != 0.0) line += " bias=" + programNumber(weave.biasMm);
        if (weave.dwellLeft != 0.0) line += " dwell_l=" + programNumber(weave.dwellLeft);
        if (weave.dwellCenter != 0.0) line += " dwell_c=" + programNumber(weave.dwellCenter);
        if (weave.dwellRight != 0.0) line += " dwell_r=" + programNumber(weave.dwellRight);
        return line;
    }
    case RobotProgramNodeType::WeaveOn: {
        const int index = std::get<WeaveOnData>(node.data).scheduleIndex;
        return index >= 0 ? strutil::format("WeaveOn %1").arg(index).str() : std::string("WeaveOn");
    }
    case RobotProgramNodeType::WeaveOff:
        return std::string("WeaveOff");
    case RobotProgramNodeType::SetTool: {
        const SetToolData& tool = std::get<SetToolData>(node.data);
        return strutil::format("SetTool %1 %2").arg(tool.toolId).arg(tool.tcpIndex);
    }
    case RobotProgramNodeType::Actuate: {
        const ActuateData& actuator = std::get<ActuateData>(node.data);
        return strutil::format("Actuate %1 %2 %3")
            .arg(actuator.mechanismId)
            .arg(actuator.actuatorId)
            .arg(programNumber(actuator.position));
    }
    case RobotProgramNodeType::Stop:
        return std::string("Stop");
    case RobotProgramNodeType::Trigger: {
        const TriggerData& trigger = std::get<TriggerData>(node.data);
        std::string line = "Trigger";
        if (trigger.referenceStart) line += " ref=start";
        if (trigger.distanceMm != 0.0) line += " dist=" + programNumber(trigger.distanceMm);
        if (trigger.timeMs != 0.0) line += " time=" + programNumber(trigger.timeMs);
        line += " ShowMessage \"";
        // Escaped on the way out so a message containing a quote or a backslash reloads unchanged.
        for (char c : trigger.message) {
            if (c == '"' || c == '\\') line += '\\';
            line += c;
        }
        line += '"';
        return line;
    }
    default:
        return std::string();
    }
}

// Serialises the whole instruction list in the on-disk format. Split out from the Save
// button so the web build can hand the same text to the browser without a file to write to.
std::string programTextForCurrentProgram() {
    std::string text;
    text += "# RobotSimulatorProgram v1\n";
    text += "# Joints are stored in degrees. Linear speed and blending are millimeters.\n";
    for (const auto& child : activeRobot().program().root->children) {
        if (!child) continue;
        const std::string line = programTextForInstruction(*child);
        if (!line.empty()) text += line + "\n";
    }
    return text;
}

void saveProgramToPath(const std::string& fileName) {
    std::ofstream out(fileName);
    if (!out) {
        activeRobot().buildStatus = "Failed to open program file for writing.";
        return;
    }
    out << programTextForCurrentProgram();
    activeRobot().buildStatus = "Program saved: " + fileName;
}

int findProgramByName(const RobotInstance& robot, const std::string& name) {
    for (size_t i = 0; i < robot.programs.size(); ++i) {
        if (strutil::equalsCaseInsensitive(robot.programs[i].name, name)) return static_cast<int>(i);
    }
    return -1;
}

std::string uniqueProgramName(const RobotInstance& robot, const std::string& base) {
    if (findProgramByName(robot, base) < 0) return base;
    for (int suffix = 2; suffix < 1000; ++suffix) {
        const std::string candidate = base + " (" + std::to_string(suffix) + ")";
        if (findProgramByName(robot, candidate) < 0) return candidate;
    }
    return base;
}

// Clears whatever the arm had planned. A trajectory belongs to the program it was built from, so it
// cannot survive that program's instructions being replaced or a different one being selected.
void clearPlannedTrajectory(RobotInstance& robot) {
    robot.selectedInstruction = -1;
    robot.timelineSamples.clear();
    robot.timelineMarkers.clear();
    robot.limitReasonCounts.clear();
    robot.durationSeconds = 0.0;
    robot.elapsedSeconds = 0.0;
    robot.appliedTimelineValid = false;
    robot.playing = false;
}

void finishProgramLoad(RobotInstance& robot, int index, const char* what) {
    robot.selectedProgram = index;
    clearPlannedTrajectory(robot);
    robot.buildStatus = strutil::format("%1 '%2': %3 instruction(s)")
                            .arg(what)
                            .arg(robot.program().name)
                            .arg(robot.program().root->children.size());
    refreshProgramPathPreview();
    refreshSimPathPreview();
}

void loadProgramFromPath(const std::string& fileName) {
    auto loadedRoot = std::make_unique<RobotProgramNode>();
    loadedRoot->type = RobotProgramNodeType::Root;
    std::string errorMessage;
    if (!loadProgramTextHeadless(fileName, loadedRoot.get(), &errorMessage)) {
        activeRobot().buildStatus = errorMessage;
        return;
    }

    RobotInstance& robot = activeRobot();
    std::string name = std::filesystem::path(fileName).stem().string();
    if (name.empty()) name = "Program";

    if (findProgramByName(robot, name) < 0) {
        RobotProgram loaded;
        loaded.name = name;
        loaded.root = std::move(loadedRoot);
        robot.programs.push_back(std::move(loaded));
        finishProgramLoad(robot, static_cast<int>(robot.programs.size()) - 1, "Loaded");
        return;
    }

    robot.pendingLoadRoot = std::move(loadedRoot);
    robot.pendingLoadName = name;
    robot.pendingLoadRaise = true;
}

// The modal state a run *starts* from, which is what the editors on the program panel set. What the
// running program has since made of it is a different thing and lives in the simulator's
// LiveRunState - conflating the two is how the station row would come to show the editor's default
// while the arm was welding at a speed a SetSpeed had changed.
RobotProgramSimulator::ProgramState programStateFor(const RobotInstance& robot) {
    RobotProgramSimulator::ProgramState state;
    state.jointSpeedRadPerSec = std::max(1.0e-6, robot.programJointSpeedDegPerSec * kDegToRad);
    state.linearSpeedMmPerSec = std::max(1.0e-6, robot.programLinearSpeedMmPerSec);
    state.blendRadiusMm = robot.programBlendMm;
    return state;
}

RobotProgramSimulator::ProgramState currentProgramState() {
    return programStateFor(activeRobot());
}

// The two run markers on a program row: orange for the instruction being sent to the firmware
// next, blue for the one the robot is executing now. They differ because the firmware holds a
// lookahead queue, so commands are accepted well before they are performed.
void drawProgramRunMarkers(bool pending, bool executing) {
    const ImVec2 origin = ImGui::GetCursorScreenPos();
    const float height = ImGui::GetTextLineHeight();
    ImDrawList* drawList = ImGui::GetWindowDrawList();
    float x = origin.x + 1.0f;
    // A 10-wide triangle on a 14-pixel pitch, pending first so the pair reads left to right in
    // the order the robot gets to them.
    const auto drawArrow = [&](ImU32 color) {
        const float top = origin.y + height * 0.15f;
        const float bottom = origin.y + height * 0.85f;
        drawList->AddTriangleFilled(ImVec2(x, top), ImVec2(x, bottom),
                                    ImVec2(x + 10.0f, (top + bottom) * 0.5f), color);
        x += 14.0f;
    };
    if (pending) drawArrow(IM_COL32(245, 135, 32, 255));
    if (executing) drawArrow(IM_COL32(66, 153, 225, 255));
    // Claims the space whether or not an arrow was drawn, so rows do not shift as the run advances.
    ImGui::Dummy(ImVec2(30.0f, height));
}

void refreshAfterProgramEdit(RobotInstance& robot, int selectedInstruction,
                             const char* action) {
    clearPlannedTrajectory(robot);
    robot.selectedInstruction = selectedInstruction;
    robot.buildStatus = strutil::format("%1. Simulate to rebuild the path.").arg(action).str();
    refreshProgramPathPreview();
    refreshSimPathPreview();
}

RobotProgramNode* editableProgramMoveAt(RobotInstance& robot, int instruction) {
    RobotProgramNode* root = robot.program().root.get();
    if (!root || instruction < 0 ||
        instruction >= static_cast<int>(root->children.size())) return nullptr;
    RobotProgramNode* node = root->children[static_cast<size_t>(instruction)].get();
    return node && editableMoveTargetJointsForNode(*node) ? node : nullptr;
}

bool previewProgramMoveTarget(RobotInstance& robot, int instruction) {
    RobotProgramNode* root = robot.program().root.get();
    if (!root || instruction < 0 || instruction >= static_cast<int>(root->children.size()) ||
        !robot.poseController.isBound()) return false;

    // Tool choice is modal program state. Replaying the SetTool rows up to this move makes the
    // selected-instruction gimbal use the same TCP the live run will use at this point.
    for (int row = 0; row <= instruction; ++row) {
        const auto& preceding = root->children[static_cast<size_t>(row)];
        if (!preceding || preceding->type != RobotProgramNodeType::SetTool) continue;
        std::string toolError;
        if (!commandProgramTool(robot, std::get<SetToolData>(preceding->data), &toolError)) {
            robot.statusText = toolError;
            return false;
        }
    }

    RobotProgramNode* node = editableProgramMoveAt(robot, instruction);
    if (!node) return false;
    const std::array<double, 6>* target = editableMoveTargetJointsForNode(*node);
    if (!target) return false;

    bool* hasExternalAxis = nullptr;
    double* externalAxisMm = nullptr;
    editableMoveExternalAxisForNode(*node, &hasExternalAxis, &externalAxisMm);
    const int gantryIndex = linkedGantryIndexForRobot(liveRunArmIndexOf(robot));
    if (gantryIndex >= 0 && hasExternalAxis && externalAxisMm && *hasExternalAxis) {
        g_scene.gantries[static_cast<size_t>(gantryIndex)].setPositionMm(*externalAxisMm);
    }
    robot.poseController.setJoints(*target);
    robot.toolTargetPose = robot.poseController.toolPose();
    robot.toolWprBasePose = robot.toolTargetPose;
    robot.toolWprDegrees = {{0.0, 0.0, 0.0}};
    robot.readoutJointsValid = false;
    robot.selectedInstruction = instruction;
    return true;
}

bool editProgramMoveTargetFromWorldPose(RobotInstance& robot, int instruction,
                                        const CadTransform& worldPose,
                                        std::string* errorMessage) {
    if (robot.buildRunning || robot.playing || robot.liveRun.running ||
        robot.hardware.programActive()) {
        if (errorMessage) *errorMessage = "Stop execution before editing a move target.";
        return false;
    }
    RobotProgramNode* node = editableProgramMoveAt(robot, instruction);
    if (!node || !robot.poseController.isBound()) {
        if (errorMessage) *errorMessage = "The selected instruction is not an editable MoveJ or MoveL.";
        return false;
    }

    const CadTransform localPose =
        robot.poseController.baseWorldTransform().rigidInverse() * worldPose;
    std::string ikError;
    if (!robot.poseController.setToolPose(localPose, &ikError)) {
        if (errorMessage) *errorMessage = ikError;
        return false;
    }

    *editableMoveTargetJointsForNode(*node) = robot.poseController.joints();
    robot.toolTargetPose = robot.poseController.toolPose();
    robot.toolWprBasePose = robot.toolTargetPose;
    robot.toolWprDegrees = {{0.0, 0.0, 0.0}};
    robot.readoutJointsValid = false;
    robot.statusText.clear();
    refreshAfterProgramEdit(robot, instruction, "Move target updated");
    return true;
}

int insertProgramInstruction(RobotInstance& robot, RobotProgramNodeType type,
                             RobotProgramNodeData data) {
    RobotProgramNode* root = robot.program().root.get();
    if (!root) return -1;
    const int count = static_cast<int>(root->children.size());
    const int insertAt = robot.selectedInstruction >= 0 && robot.selectedInstruction < count
        ? robot.selectedInstruction + 1 : count;
    auto instruction = std::make_unique<RobotProgramNode>();
    instruction->type = type;
    instruction->data = std::move(data);
    instruction->parent = root;
    root->children.insert(root->children.begin() + insertAt, std::move(instruction));
    refreshAfterProgramEdit(robot, insertAt, "Instruction added");
    return insertAt;
}

bool deleteProgramInstruction(RobotInstance& robot, int index) {
    RobotProgramNode* root = robot.program().root.get();
    if (!root || index < 0 || index >= static_cast<int>(root->children.size())) return false;
    root->children.erase(root->children.begin() + index);
    const int nextSelection = root->children.empty()
        ? -1 : std::min(index, static_cast<int>(root->children.size()) - 1);
    refreshAfterProgramEdit(robot, nextSelection, "Instruction deleted");
    if (nextSelection >= 0) previewProgramMoveTarget(robot, nextSelection);
    return true;
}

// One selectable program-state tool entry. A physical tool with two TCPs contributes two entries:
// the geometry remains one station accessory, while the selected TCP changes the kinematic bind.
std::vector<RobotToolChoice> robotToolChoices(RobotInstance& robot) {
    std::vector<RobotToolChoice> choices;
    CadNode* robotNode = robot.poseController.robotNode();
    for (CadNode* toolNode : collectRobotTools(robotNode)) {
        RobotToolData* tool = toolNode ? toolNode->asRobotTool() : nullptr;
        if (!tool) continue;
        const std::string toolName = toolNode->name.empty() ? "Tool" : toolNode->name;
        if (tool->tcps.empty()) {
            choices.push_back({toolNode, tool, 0, toolName});
            continue;
        }
        for (int tcpIndex = 0; tcpIndex < static_cast<int>(tool->tcps.size()); ++tcpIndex) {
            const std::string& tcpName = tool->tcps[static_cast<size_t>(tcpIndex)].name;
            std::string label = toolName;
            if (!tcpName.empty() && !(tool->tcps.size() == 1 && tcpName == "TCP")) {
                label += " / " + tcpName;
            }
            choices.push_back({toolNode, tool, tcpIndex, std::move(label)});
        }
    }
    return choices;
}

bool activateRobotToolChoice(RobotInstance& robot, const RobotToolChoice& choice) {
    OPW6RobotData* robotData = robot.poseController.robotData();
    if (!robotData || !choice.node || !choice.data) return false;
    const int tcpIndex = choice.data->tcps.empty() ? 0 : std::max(
        0, std::min(choice.tcpIndex, static_cast<int>(choice.data->tcps.size()) - 1));
    if (robotData->activeTool == choice.node && choice.data->activeTcpIndex == tcpIndex) {
        return false;
    }

    cancelTrajectoryBuild(robot);
    robotData->activeTool = choice.node;
    choice.data->activeTcpIndex = tcpIndex;

    // Active tool belongs to the robot; active TCP belongs to the attached tool instance. Persist
    // both so saving and reopening a station does not silently return to another grasp frame.
    int robotIndex = -1;
    for (int index = 0; index < static_cast<int>(g_scene.robots.size()); ++index) {
        if (g_scene.robots[static_cast<size_t>(index)].get() == &robot) {
            robotIndex = index;
            break;
        }
    }
    if (robotIndex >= 0 && robotIndex < static_cast<int>(g_scene.station.robots.size())) {
        StationRobotInstance& stationRobot =
            g_scene.station.robots[static_cast<size_t>(robotIndex)];
        stationRobot.activeToolId = "@package";
        for (StationAccessoryInstance& accessory : g_scene.station.accessories) {
            if (accessory.node != choice.node || accessory.parentRobotId != stationRobot.id) continue;
            stationRobot.activeToolId = accessory.id;
            accessory.activeTcpIndex = tcpIndex;
            break;
        }
    }

    std::string bindError;
    if (!robot.poseController.refreshActiveToolBind(&bindError)) {
        robot.statusText = "Could not activate tool: " + bindError;
        return false;
    }

    robot.toolTargetPose = robot.poseController.toolPose();
    robot.toolWprBasePose = robot.toolTargetPose;
    robot.toolWprDegrees = {{0.0, 0.0, 0.0}};
    robot.readoutJointsValid = false;
    clearPlannedTrajectory(robot);
    robot.buildStatus = "Active tool changed. Simulate to rebuild the path.";
    robot.statusText = "Active tool: " + choice.label;
    refreshProgramPathPreview();
    refreshSimPathPreview();
    if (g_scene.viewer) g_scene.viewer->markCacheDirty();
    return true;
}

// Deep copy of an instruction tree. The payloads are plain values so the variant copies itself; the
// parent pointers are the only thing that has to be rebuilt, since a copied child pointing at the
// original's root would edit the program it was duplicated from.
std::unique_ptr<RobotProgramNode> cloneProgramNode(const RobotProgramNode& source,
                                                   RobotProgramNode* parent) {
    auto copy = std::make_unique<RobotProgramNode>();
    copy->type = source.type;
    copy->data = source.data;
    copy->parent = parent;
    for (const auto& child : source.children) {
        if (child) copy->children.push_back(cloneProgramNode(*child, copy.get()));
    }
    return copy;
}

// Moving to another program leaves the trajectory behind. The timeline belongs to the program it
// was built from, and showing one program's planned path while another's instructions are on screen
// is the kind of wrong that gets acted on.
void selectProgram(RobotInstance& robot, int index) {
    if (index < 0 || index >= static_cast<int>(robot.programs.size())) return;
    if (index == robot.selectedProgramIndex()) return;
    cancelTrajectoryBuild(robot);
    robot.selectedProgram = index;
    robot.selectedInstruction = -1;
    clearPlannedTrajectory(robot);
    refreshProgramPathPreview();
    refreshSimPathPreview();
}

void drawProgramLoadConflictModal(RobotInstance& robot) {
    if (robot.pendingLoadRaise) {
        ImGui::OpenPopup("Program name in use");
        robot.pendingLoadRaise = false;
    }
    if (!ImGui::BeginPopupModal("Program name in use", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        return;
    }

    ImGui::Text("This robot already has a program called '%s'.", robot.pendingLoadName.c_str());
    ImGui::Spacing();
    ImGui::TextWrapped("Overwrite: replace that program's instructions with the file's.");
    ImGui::TextWrapped("Load as new: keep both, and give the loaded one a numbered name.");
    ImGui::Spacing();

    const int existing = findProgramByName(robot, robot.pendingLoadName);
    if (ImGui::Button("Overwrite") && existing >= 0) {
        // Cancelled first: a build in flight is planning the tree about to be replaced.
        cancelTrajectoryBuild(robot);
        robot.programs[static_cast<size_t>(existing)].root = std::move(robot.pendingLoadRoot);
        finishProgramLoad(robot, existing, "Overwrote");
        robot.pendingLoadRoot.reset();
        ImGui::CloseCurrentPopup();
    }
    ImGui::SameLine();
    if (ImGui::Button("Load as new")) {
        RobotProgram loaded;
        loaded.name = uniqueProgramName(robot, robot.pendingLoadName);
        loaded.root = std::move(robot.pendingLoadRoot);
        robot.programs.push_back(std::move(loaded));
        finishProgramLoad(robot, static_cast<int>(robot.programs.size()) - 1, "Loaded");
        robot.pendingLoadRoot.reset();
        ImGui::CloseCurrentPopup();
    }
    ImGui::SameLine();
    if (ImGui::Button("Cancel")) {
        robot.pendingLoadRoot.reset();
        robot.buildStatus = "Load cancelled; nothing changed.";
        ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
}

// The arm's programs, above the instructions of whichever one is selected.
void drawProgramList() {
    RobotInstance& robot = activeRobot();
    const bool editing = !robot.buildRunning && !robot.playing;
    const int count = static_cast<int>(robot.programs.size());

    ImGui::TextUnformatted("Programs");
    ImGui::BeginDisabled(!editing);
    const float listHeight = ImGui::GetTextLineHeightWithSpacing() * 4.0f;
    if (ImGui::BeginListBox("##programs", ImVec2(-1.0f, listHeight))) {
        for (int i = 0; i < count; ++i) {
            const RobotProgram& entry = robot.programs[static_cast<size_t>(i)];
            ImGui::PushID(i);
            // The instruction count rides along, because it is what tells two similarly named
            // programs apart at a glance, and what says which one is still empty.
            const size_t instructions = entry.root ? entry.root->children.size() : 0;
            const std::string label = (entry.name.empty() ? "Program " + std::to_string(i + 1) : entry.name) +
                                      "   (" + std::to_string(instructions) + ")";
            if (ImGui::Selectable(label.c_str(), i == robot.selectedProgramIndex())) {
                selectProgram(robot, i);
            }
            ImGui::PopID();
        }
        ImGui::EndListBox();
    }

    if (formButton("New", 3)) {
        robot.programs.emplace_back();
        robot.programs.back().name = "Program " + std::to_string(robot.programs.size());
        selectProgram(robot, static_cast<int>(robot.programs.size()) - 1);
    }
    ImGui::SameLine();
    if (formButton("Duplicate", 3)) {
        RobotProgram copy;
        copy.name = robot.program().name + " copy";
        if (robot.program().root) copy.root = cloneProgramNode(*robot.program().root, nullptr);
        robot.programs.push_back(std::move(copy));
        selectProgram(robot, static_cast<int>(robot.programs.size()) - 1);
    }
    ImGui::SameLine();
    ImGui::BeginDisabled(count <= 1);
    if (formButton("Delete", 3)) {
        const int index = robot.selectedProgramIndex();
        // Cancelled before the vector moves: a build in flight is planning the tree about to be
        // erased out from under it.
        cancelTrajectoryBuild(robot);
        robot.programs.erase(robot.programs.begin() + index);
        robot.selectedProgram = std::max(0, index - 1);
        robot.selectedInstruction = -1;
        robot.timelineSamples.clear();
        robot.timelineMarkers.clear();
        robot.durationSeconds = 0.0;
        robot.elapsedSeconds = 0.0;
        robot.appliedTimelineValid = false;
        robot.playing = false;
        refreshProgramPathPreview();
        refreshSimPathPreview();
    }
    ImGui::EndDisabled();

    if (formButton("Save", 2)) {
#ifdef __EMSCRIPTEN__
        // No writable filesystem worth saving to, so the text goes straight to the browser,
        // which asks where to put it.
        WebFiles::saveText("program.robotprog.txt", programTextForCurrentProgram());
        robot.buildStatus = "Program handed to the browser to save.";
#else
        // Offers the program's own name, since that is what is being written.
        const std::string fileName = runFileDialog("Save Robot Program", true, kProgramFilter, L"txt",
                                                   robot.program().name);
        if (!fileName.empty()) saveProgramToPath(fileName);
#endif
    }
    ImGui::SameLine();
    if (formButton("Load", 2)) {
#ifdef __EMSCRIPTEN__
        WebFiles::requestOpen(WebFiles::Purpose::Program, ".txt");
        robot.buildStatus = "Choose a program file...";
#else
        const std::string fileName = runFileDialog("Load Robot Program", false, kProgramFilter, L"txt");
        if (!fileName.empty()) loadProgramFromPath(fileName);
#endif
    }

    char name[64] = {};
    std::snprintf(name, sizeof(name), "%s", robot.program().name.c_str());
    ImGui::SetNextItemWidth(-1.0f);
    if (ImGui::InputText("##programName", name, sizeof(name))) robot.program().name = name;
    ImGui::EndDisabled();

    // Outside the disabled block: a modal that cannot be answered is a modal you cannot get out of.
    drawProgramLoadConflictModal(robot);

    ImGui::Separator();
}

