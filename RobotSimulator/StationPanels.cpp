#include "StationPanels.h"
#include "StringUtil.h"

#include "AccessoryBuilders.h"
#include "AccessoryGeometry.h"
#include "AccessoryPropertySchema.h"
#include "AppState.h"
#include "ConveyorPhysics.h"
#include "ConveyorRuntime.h"
#include "JsonCompat.h"
#include "MasteringIo.h"
#include "PlacedMechanismSchema.h"
#include "PlacementWindows.h"
#include "ProgramEditing.h"
#include "ProgramWidgets.h"
#include "RobotLibraryPanel.h"
#include "RobotMotionCore.h"
#include "RobotProgramModel.h"
#include "RobotRuntime.h"
#include "SceneMath.h"
#include "StationParameterLinks.h"
#include "StationSceneLoad.h"
#include "TrajectoryAndLiveRun.h"
#include "UiFormHelpers.h"
#include "ViewerBridge.h"
#include "WebFiles.h"

#include "imgui.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace {
using namespace progtext;

using strutil::operator<<;

// Render one field from the shared AccessoryPropertySchema.
bool drawAccessoryField(const AccessoryField& field, TransformNodeData& parameters, CadNode* node,
                        double* previous) {
    const std::string caption = std::string(field.label) +
        (field.unit && *field.unit ? " (" + std::string(field.unit) + ")" : std::string());
    const std::string decimals = "%." + std::to_string(field.decimals) + "f";
    bool changed = false;
    if (previous) *previous = 0.0;

    switch (field.kind) {
    case AccessoryFieldKind::Number: {
        double* member = field.number(parameters);
        if (previous) *previous = *member;
        // The step and the fast step are a tenth and a whole of the schema's own step, so a field that
        // says it moves in 10 mm does.
        changed = stationPropertyInputDouble(field.key, caption.c_str(), member, field.step,
                                             field.step * 10.0, decimals.c_str(),
                                             ImGuiInputTextFlags_EnterReturnsTrue);
        break;
    }
    case AccessoryFieldKind::Integer:
        changed = stationPropertyInputInt(field.key, caption.c_str(), field.integer(parameters), 1, 10,
                                          ImGuiInputTextFlags_EnterReturnsTrue);
        break;
    case AccessoryFieldKind::Toggle:
        changed = ImGui::Checkbox(caption.c_str(), field.flag(parameters));
        break;
    case AccessoryFieldKind::Choice: {
        std::string* member = field.text(parameters);
        const char* label = member->c_str();
        for (const AccessoryFieldChoice& choice : field.choices) {
            if (*member == choice.value) { label = choice.label; break; }
        }
        ImGui::PushID(field.key);
        beginStationPropertyField(caption.c_str());
        if (ImGui::BeginCombo("##value", label)) {
            for (const AccessoryFieldChoice& choice : field.choices) {
                if (ImGui::Selectable(choice.label, *member == choice.value)) {
                    *member = choice.value;
                    changed = true;
                }
            }
            ImGui::EndCombo();
        }
        ImGui::PopID();
        break;
    }
    case AccessoryFieldKind::ItemReference: {
        // The candidates are this host's: a CadNode station's hidden accessories, resolved by their
        // stable ids. The schema says the field names something else in the cell and stops there,
        // because what "something else" is available is exactly what the two cells do not share.
        std::string* member = field.text(parameters);
        const StationAccessoryInstance* selected = conveyorSpawnPrototype(parameters);
        ImGui::PushID(field.key);
        beginStationPropertyField(caption.c_str());
        if (ImGui::BeginCombo("##value",
                              selected ? selected->name.c_str() : "Select hidden object...")) {
            for (const StationAccessoryInstance& candidate : g_scene.station.accessories) {
                if (!candidate.hidden || !candidate.node) continue;
                if (ImGui::Selectable(candidate.name.c_str(), *member == candidate.id)) {
                    *member = candidate.id;
                    changed = true;
                }
            }
            ImGui::EndCombo();
        }
        ImGui::PopID();
        break;
    }
    }
    if (field.tooltip && *field.tooltip && ImGui::IsItemHovered()) {
        ImGui::SetTooltip("%s", field.tooltip);
    }
    // The one thing a spawner's cap is worth saying out loud next to: a cap doing exactly what it was
    // asked to and a line that has broken read identically otherwise.
    if (std::string(field.key) == "maxActiveSpawns" && node) {
        const size_t active = activeSpawnCount(node);
        if (*field.integer(parameters) > 0) {
            ImGui::TextDisabled("Active products: %zu / %d", active, *field.integer(parameters));
        } else {
            ImGui::TextDisabled("Active products: %zu (unlimited)", active);
        }
    }
    return changed;
}

const char* accessoryGroupHeading(AccessoryFieldGroup group) {
    switch (group) {
    case AccessoryFieldGroup::Behaviour: return "Conveyor behavior";
    case AccessoryFieldGroup::Queue: return "Queue";
    case AccessoryFieldGroup::Dimensions: return "Dimensions";
    case AccessoryFieldGroup::Corners: return "Deck corner heights";
    case AccessoryFieldGroup::Appearance: return "Appearance";
    }
    return "";
}

} // namespace

void drawStationPropertiesPanel() {
    CadNode* node = selectedStationNode();
    if (!node) {
        ImGui::TextDisabled("Select a robot, linear rail, or accessory in the tree or 3D view.");
        return;
    }

    ImGui::TextWrapped("%s", node->name.empty() ? "Selected object" : node->name.c_str());
    ImGui::Separator();
    bool sceneChanged = false;

    switch (g_scene.stationSelectionKind) {
    case StationSelectionKind::Robot: {
        const int index = g_scene.stationSelectionIndex;
        ImGui::TextDisabled("ROBOT");
        if (index >= 0 && index < static_cast<int>(g_scene.robots.size())) {
            CadNode* placement = placementNodeFor(*g_scene.robots[index]);
            sceneChanged |= drawStationPositionEditor(placement);
            ImGui::Spacing();
            if (ImGui::Button("Work on this robot", ImVec2(-1.0f, 0.0f))) {
                enterRobotView(index);
            }
        }
        break;
    }
    case StationSelectionKind::Mechanism: {
        const int index = g_scene.stationSelectionIndex;
        ImGui::TextDisabled("LINEAR RAIL");
        if (index >= 0 && index < static_cast<int>(g_scene.gantries.size())) {
            GantryPoseController& gantry = g_scene.gantries[index];
            GantryMechanismData* data = gantry.gantryData();
            sceneChanged |= drawStationPositionEditor(node);
            const std::vector<placedmechanism::AxisField> axes = placedmechanism::axisFields(node);
            if (!axes.empty()) {
                ImGui::SeparatorText("Motion");
                double position = gantry.positionMm();
                const placedmechanism::AxisField& axis = axes.front();
                const std::string caption = axis.label + " (" + axis.unit + ")";
                if (stationPropertySliderDouble(axis.key.c_str(), caption.c_str(), &position,
                                                &axis.minimum, &axis.maximum, "%.1f")) {
                    gantry.setPositionMm(position);
                    sceneChanged = true;
                }
                if (axis.limited) {
                    ImGui::TextDisabled("Travel %.1f to %.1f %s", axis.minimum, axis.maximum,
                                        axis.unit);
                } else {
                    ImGui::TextDisabled("This package states no travel for its carriage.");
                }
                if (data) ImGui::TextDisabled("Maximum speed %.1f mm/s", data->velocityMaxMmS);
            }
        }
        break;
    }
    case StationSelectionKind::Accessory: {
        const int index = g_scene.stationSelectionIndex;
        const bool isTool = index >= 0 &&
            index < static_cast<int>(g_scene.station.accessories.size()) &&
            !g_scene.station.accessories[static_cast<size_t>(index)].parentRobotId.empty() &&
            node->asRobotTool();
        const std::string accessoryId =
            index >= 0 && index < static_cast<int>(g_scene.station.accessories.size())
            ? g_scene.station.accessories[static_cast<size_t>(index)].id : std::string();
        ImGui::TextDisabled(isTool ? "ROBOT TOOL" : "ACCESSORY");
        sceneChanged |= drawStationPositionEditor(node);
        if (isTool) {
            RobotToolData* tool = node->asRobotTool();
            ImGui::SeparatorText("Tool center point");
            const std::string& parentRobot =
                g_scene.station.accessories[static_cast<size_t>(index)].parentRobotId;
            for (size_t robotIndex = 0; robotIndex < g_scene.station.robots.size() &&
                 robotIndex < g_scene.robots.size(); ++robotIndex) {
                if (g_scene.station.robots[robotIndex].id != parentRobot ||
                    !g_scene.robots[robotIndex]) continue;
                RobotInstance& robot = *g_scene.robots[robotIndex];
                const bool editing = !robot.buildRunning && !robot.playing &&
                    !robot.liveRun.running && !robot.hardware.programActive();
                sceneChanged |= drawActiveToolState(robot, "##activeToolTcp", editing);
                break;
            }
            ImGui::TextDisabled("%zu TCPs stored in this tool package.", tool->tcps.size());
        }
        TransformNodeData* parameters = node->asTransform();
        if (parameters && parameters->hasParametricAccessory()) {
            const bool rollerConveyor =
                parameters->accessoryGenerator == "roller_conveyor";
            bool rebuild = false;
            bool startHeightEdited = false;
            bool endHeightEdited = false;
            const double persistedStartHeight =
                index >= 0 && index < static_cast<int>(g_scene.station.accessories.size())
                ? jsoncompat::fieldDouble(
                      g_scene.station.accessories[static_cast<size_t>(index)].parameters,
                      "startHeightMm", parameters->accessoryStartHeightMm)
                : parameters->accessoryStartHeightMm;
            const double persistedEndHeight =
                index >= 0 && index < static_cast<int>(g_scene.station.accessories.size())
                ? jsoncompat::fieldDouble(
                      g_scene.station.accessories[static_cast<size_t>(index)].parameters,
                      "endHeightMm", parameters->accessoryEndHeightMm)
                : parameters->accessoryEndHeightMm;
            // Every editable field, walked. The list, the labels, the units, the ranges, the choices,
            // which generators and roles have each one, whether it may be touched while a run is going
            // and what it is coupled to are all `Common/AccessoryPropertySchema` - the same table the
            // RoboDK plugin's Qt panel renders. Adding a field there adds it here.
            const bool geometryEditable =
                g_scene.stationRunState == StationRunState::Stopped;
            bool disabled = false;
            const AccessoryFieldGroup* heading = nullptr;
            for (const AccessoryField& field : accessoryPropertySchema()) {
                if (!accessoryFieldApplies(field, *parameters)) continue;
                if (!heading || *heading != field.group) {
                    if (disabled) { ImGui::EndDisabled(); disabled = false; }
                    ImGui::SeparatorText(accessoryGroupHeading(field.group));
                    if (field.group == AccessoryFieldGroup::Dimensions) {
                        ImGui::TextDisabled("Press Enter or use Apply to rebuild the model.");
                    } else if (field.group == AccessoryFieldGroup::Corners) {
                        ImGui::TextDisabled("Left/right are viewed from the start toward the end.");
                    }
                    heading = &field.group;
                }
                const bool editable = geometryEditable || field.editableWhileRunning;
                if (editable == disabled) {
                    if (disabled) ImGui::EndDisabled();
                    else ImGui::BeginDisabled(true);
                    disabled = !editable;
                }
                double previous = 0.0;
                if (!drawAccessoryField(field, *parameters, node, &previous)) continue;
                rebuild = true;
                if (field.couple) field.couple(*parameters, previous);
                const std::string key(field.key);
                // Which of the two linked endpoint heights this edit moved, for the station parameter
                // links below - a corner counts, because the schema's coupling has just moved its
                // centre with it.
                if (key == "startHeightMm" || key == "startLeftHeightMm" ||
                    key == "startRightHeightMm") {
                    startHeightEdited = true;
                } else if (key == "endHeightMm" || key == "endLeftHeightMm" ||
                           key == "endRightHeightMm") {
                    endHeightEdited = true;
                }
                // Choosing the pick-feeder role is choosing the geometry it needs: an open pocket a
                // gripper descends into has a belt surface and something to stop against.
                if (key == "role" && parameters->accessoryConveyorRole == "pick_feeder") {
                    parameters->accessoryRollerCoverEnabled = true;
                    parameters->accessoryEndStopEnabled = true;
                }
                // The two links an operator can see, shown where the value they hold is edited.
                if (!accessoryId.empty() && (key == "startHeightMm" || key == "endHeightMm")) {
                    const std::string linked = stationLinkedParameterDescription(
                        g_scene.station, accessoryId, field.key);
                    if (!linked.empty()) {
                        ImGui::TextColored(ImVec4(0.35f, 0.78f, 1.0f, 1.0f), "Linked to %s",
                                           linked.c_str());
                        ImGui::SameLine();
                        ImGui::PushID(field.key);
                        if (ImGui::SmallButton("Unlink")) {
                            removeStationParameterLinksForEndpoint(
                                g_scene.station,
                                {accessoryId, key == "startHeightMm" ? "start" : "end", field.key});
                            sceneChanged = true;
                        }
                        ImGui::PopID();
                    }
                }
            }
            if (disabled) ImGui::EndDisabled();
            if (rollerConveyor && parameters->accessoryConveyorMode == "physx") {
                ImGui::TextDisabled("Dynamic bodies collide with the generated rollers and frame.");
            }
            ImGui::BeginDisabled(!geometryEditable);
            const bool applyDimensions =
                ImGui::Button("Apply dimensions", ImVec2(-1.0f, 0.0f));
            ImGui::EndDisabled();
            if (!geometryEditable) {
                ImGui::TextDisabled("Stop the station to edit accessory geometry.");
            }
            rebuild |= applyDimensions;
            if (rebuild && rollerConveyor) {
                startHeightEdited |=
                    std::abs(parameters->accessoryStartHeightMm - persistedStartHeight) > 1.0e-9;
                endHeightEdited |=
                    std::abs(parameters->accessoryEndHeightMm - persistedEndHeight) > 1.0e-9;
            }

            std::vector<size_t> rebuildIndices;
            std::string linkError;
            const auto propagateHeight = [&](const char* parameterName, double requested,
                                             double fallback) {
                std::vector<size_t> affected;
                if (!applyStationLinkedParameter(g_scene.station, accessoryId, parameterName,
                                                 requested, &affected, &linkError,
                                                 /*rebuildModels=*/false)) {
                    if (double* address = stationAccessoryParameter(
                            g_scene.station.accessories[static_cast<size_t>(index)],
                            parameterName)) {
                        *address = fallback;
                    }
                    return;
                }
                for (size_t affectedIndex : affected) {
                    if (std::find(rebuildIndices.begin(), rebuildIndices.end(), affectedIndex) ==
                        rebuildIndices.end()) {
                        rebuildIndices.push_back(affectedIndex);
                    }
                }
            };
            if (rollerConveyor && !accessoryId.empty()) {
                if (startHeightEdited) {
                    propagateHeight("startHeightMm", parameters->accessoryStartHeightMm,
                                    persistedStartHeight);
                }
                if (endHeightEdited) {
                    propagateHeight("endHeightMm", parameters->accessoryEndHeightMm,
                                    persistedEndHeight);
                }
            }
            if (rebuild && index >= 0 &&
                index < static_cast<int>(g_scene.station.accessories.size()) &&
                std::find(rebuildIndices.begin(), rebuildIndices.end(),
                          static_cast<size_t>(index)) == rebuildIndices.end()) {
                rebuildIndices.push_back(static_cast<size_t>(index));
            }
            bool rebuiltAny = false;
            bool rebuiltConveyor = false;
            for (size_t rebuildIndex : rebuildIndices) {
                StationAccessoryInstance& accessory =
                    g_scene.station.accessories[rebuildIndex];
                if (!rebuildParametricAccessory(accessory.node, g_scene.station)) continue;
                accessory.name = accessory.node->name;
                if (const TransformNodeData* rebuilt = accessory.node->asTransform()) {
                    accessory.parameters = rebuilt->accessoryParametersJson();
                    rebuiltConveyor |= rebuilt->accessoryGenerator == "roller_conveyor";
                }
                rebuiltAny = true;
            }
            if (!linkError.empty()) {
                ImGui::TextColored(ImVec4(1.0f, 0.38f, 0.30f, 1.0f), "%s",
                                   linkError.c_str());
            }
            if (rebuiltAny) {
                if (rebuiltConveyor) rebuildConveyorRuntime();
                sceneChanged = true;
            }
        } else {
            ImGui::Spacing();
            ImGui::TextDisabled("This accessory has no editable geometry parameters.");
        }
        break;
    }
    case StationSelectionKind::None:
        break;
    }

    if (sceneChanged) {
        if (g_scene.viewer) g_scene.viewer->markCacheDirty();
        for (const std::unique_ptr<RobotInstance>& robot : g_scene.robots) {
            if (robot) robot->readoutJointsValid = false;
        }
        refreshProgramPathPreview();
        refreshSimPathPreview();
    }
}

void drawStationPanel() {
    if (!g_scene.viewer) {
        ImGui::TextDisabled("%s", g_scene.error.empty() ? "No scene loaded." : g_scene.error.c_str());
        return;
    }

    ImGui::PushTextWrapPos(0.0f);
    ImGui::TextUnformatted(g_scene.packageLabel.c_str());
    ImGui::PopTextWrapPos();
    ImGui::Separator();

    const int robotCount = static_cast<int>(g_scene.robots.size());
    const int gantryCount = static_cast<int>(g_scene.gantries.size());
    const int accessoryCount = static_cast<int>(g_scene.station.accessories.size());
    const int hiddenAccessoryCount = static_cast<int>(std::count_if(
        g_scene.station.accessories.begin(), g_scene.station.accessories.end(),
        [](const StationAccessoryInstance& accessory) { return accessory.hidden; }));
    const int visibleAccessoryCount = accessoryCount - hiddenAccessoryCount;
    ImGui::TextDisabled("%d robot%s, %d mechanism%s, %d accessor%s in this cell.", robotCount,
                        robotCount == 1 ? "" : "s", gantryCount, gantryCount == 1 ? "" : "s",
                        visibleAccessoryCount, visibleAccessoryCount == 1 ? "y" : "ies");
    ImGui::Spacing();

    const std::string& stationName = g_scene.station.name;
    const char* sceneName = !stationName.empty() ? stationName.c_str() : "AR4 sample setup";
    const float available = ImGui::GetContentRegionAvail().y;
    const float treeHeight = std::max(120.0f, std::min(available * 0.5f, available - 190.0f));
    if (ImGui::BeginChild("stationTree", ImVec2(0.0f, treeHeight),
                          ImGuiChildFlags_Borders | ImGuiChildFlags_ResizeY)) {
        if (ImGui::TreeNodeEx(sceneName, ImGuiTreeNodeFlags_DefaultOpen |
                                             ImGuiTreeNodeFlags_SpanAvailWidth)) {
            for (int i = 0; i < gantryCount; ++i) {
                GantryPoseController& gantry = g_scene.gantries[static_cast<size_t>(i)];
                GantryMechanismData* data = gantry.gantryData();
                CadNode* node = gantry.gantryNode();
                if (!data) continue;
                ImGui::PushID("gantry");
                ImGui::PushID(i);
                const std::string label = node && !node->name.empty()
                    ? node->name : "Gantry " + std::to_string(i + 1);
                ImGuiTreeNodeFlags mechanismFlags = ImGuiTreeNodeFlags_DefaultOpen |
                                                    ImGuiTreeNodeFlags_SpanAvailWidth;
                if (g_scene.stationSelectionKind == StationSelectionKind::Mechanism &&
                    g_scene.stationSelectionIndex == i) {
                    mechanismFlags |= ImGuiTreeNodeFlags_Selected;
                }
                const bool mechanismOpen = ImGui::TreeNodeEx(label.c_str(), mechanismFlags);
                if (ImGui::IsItemClicked() && !ImGui::IsItemToggledOpen()) {
                    selectStationObject(StationSelectionKind::Mechanism, i);
                }
                if (mechanismOpen) {
                    double position = gantry.positionMm();
                    // The tree's inline slider, bounded by the same shared table the properties panel
                    // uses. It read `data` directly, which meant a rail's travel was stated twice inside
                    // one file - and a slider that disagreed with the panel beside it about where a
                    // carriage may stand is the kind of thing nobody notices until a package changes.
                    const std::vector<placedmechanism::AxisField> axes =
                        placedmechanism::axisFields(node);
                    const double lower = axes.empty() ? data->lowerLimitMm : axes.front().minimum;
                    const double upper = axes.empty() ? data->upperLimitMm : axes.front().maximum;
                    if (ImGui::SliderScalar("Position", ImGuiDataType_Double, &position,
                                            &lower, &upper, "%.1f mm")) {
                        gantry.setPositionMm(position);
                        for (const std::unique_ptr<RobotInstance>& robot : g_scene.robots) {
                            if (robot) robot->readoutJointsValid = false;
                        }
                        refreshProgramPathPreview();
                        refreshSimPathPreview();
                    }
                    ImGui::SameLine();
                    if (ImGui::SmallButton("Home")) {
                        gantry.resetHome();
                        refreshProgramPathPreview();
                        refreshSimPathPreview();
                    }
                    ImGui::TextDisabled("Travel %.1f to %.1f mm", lower, upper);
                    ImGui::TreePop();
                }
                ImGui::PopID();
                ImGui::PopID();
            }
            for (int i = 0; i < accessoryCount; ++i) {
                const StationAccessoryInstance& accessory =
                    g_scene.station.accessories[static_cast<size_t>(i)];
                if (accessory.hidden || !accessory.parentRobotId.empty()) continue;
                ImGui::PushID("accessory");
                ImGui::PushID(i);
                const std::string label = !accessory.name.empty()
                    ? accessory.name : "Accessory " + std::to_string(i + 1);
                ImGuiTreeNodeFlags accessoryFlags = ImGuiTreeNodeFlags_Leaf |
                                                    ImGuiTreeNodeFlags_NoTreePushOnOpen |
                                                    ImGuiTreeNodeFlags_SpanAvailWidth;
                if (g_scene.stationSelectionKind == StationSelectionKind::Accessory &&
                    g_scene.stationSelectionIndex == i) {
                    accessoryFlags |= ImGuiTreeNodeFlags_Selected;
                }
                ImGui::TreeNodeEx(label.c_str(), accessoryFlags);
                if (ImGui::IsItemClicked()) {
                    selectStationObject(StationSelectionKind::Accessory, i);
                }
                ImGui::PopID();
                ImGui::PopID();
            }
            std::vector<std::string> hiddenFolders;
            for (const StationAccessoryInstance& accessory : g_scene.station.accessories) {
                if (!accessory.hidden) continue;
                const std::string folder = accessory.folder.empty()
                    ? "Hidden items" : accessory.folder;
                if (std::find(hiddenFolders.begin(), hiddenFolders.end(), folder) ==
                    hiddenFolders.end()) {
                    hiddenFolders.push_back(folder);
                }
            }
            for (const std::string& folder : hiddenFolders) {
                ImGui::PushID(folder.c_str());
                if (ImGui::TreeNodeEx(folder.c_str(), ImGuiTreeNodeFlags_DefaultOpen |
                                                     ImGuiTreeNodeFlags_SpanAvailWidth)) {
                    for (int i = 0; i < accessoryCount; ++i) {
                        const StationAccessoryInstance& accessory =
                            g_scene.station.accessories[static_cast<size_t>(i)];
                        const std::string accessoryFolder = accessory.folder.empty()
                            ? "Hidden items" : accessory.folder;
                        if (!accessory.hidden || accessoryFolder != folder) continue;
                        ImGui::PushID(i);
                        ImGuiTreeNodeFlags flags = ImGuiTreeNodeFlags_Leaf |
                            ImGuiTreeNodeFlags_NoTreePushOnOpen |
                            ImGuiTreeNodeFlags_SpanAvailWidth;
                        if (g_scene.stationSelectionKind == StationSelectionKind::Accessory &&
                            g_scene.stationSelectionIndex == i) {
                            flags |= ImGuiTreeNodeFlags_Selected;
                        }
                        const std::string label = !accessory.name.empty()
                            ? accessory.name : "Hidden object " + std::to_string(i + 1);
                        ImGui::TreeNodeEx(label.c_str(), flags);
                        if (ImGui::IsItemClicked()) {
                            selectStationObject(StationSelectionKind::Accessory, i);
                        }
                        ImGui::SameLine();
                        ImGui::TextDisabled("hidden");
                        ImGui::PopID();
                    }
                    ImGui::TreePop();
                }
                ImGui::PopID();
            }
            for (int i = 0; i < robotCount; ++i) {
                const RobotInstance& robot = *g_scene.robots[static_cast<size_t>(i)];
                const CadNode* node = robot.poseController.robotNode();
                ImGui::PushID(i);

                // The base frame is the arm's parent in the CAD tree, so it is the arm's parent row
                // here: the arm hangs off the frame, its placement is the frame's transform, and
                // everything the arm does is measured from it.
                RobotInstance& mutableRobotForFrame = *g_scene.robots[static_cast<size_t>(i)];
                CadNode* const frameNode = baseFrameNodeFor(robot);
                const bool hasOwnFrame = frameNode != nullptr && frameNode != node;
                bool frameOpen = true;
                if (hasOwnFrame) {
                    // AllowOverlap because SpanAvailWidth stretches this row's hit box to the right
                    // edge, over the ground the Position... button is about to be drawn on. Without
                    // it the row wins the hit test - it is submitted first, so the button is
                    // rejected as overlapped - and the button draws, highlights nothing and does
                    // nothing when pressed.
                    frameOpen = ImGui::TreeNodeEx(frameNode->name.c_str(),
                                                  ImGuiTreeNodeFlags_DefaultOpen |
                                                      ImGuiTreeNodeFlags_OpenOnArrow |
                                                      ImGuiTreeNodeFlags_SpanAvailWidth |
                                                      ImGuiTreeNodeFlags_AllowOverlap);
                    if (ImGui::IsItemClicked() && !ImGui::IsItemToggledOpen() &&
                        ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
                        mutableRobotForFrame.placementWindowOpen = true;
                    }
                    drawPlacementToggle(mutableRobotForFrame);
                }
                if (!frameOpen) {
                    ImGui::PopID();
                    continue;
                }

                // AllowOverlap for the same reason as the frame row above: this row carries the
                // Position... button when the arm has no frame of its own, and a span-the-width hit
                // box would swallow the press.
                ImGuiTreeNodeFlags flags = ImGuiTreeNodeFlags_OpenOnArrow |
                                           ImGuiTreeNodeFlags_SpanAvailWidth |
                                           ImGuiTreeNodeFlags_AllowOverlap;
                const bool robotSelected = g_scene.stationMode == StationMode::Program
                    ? i == g_scene.view.robot
                    : (g_scene.stationSelectionKind == StationSelectionKind::Robot &&
                       g_scene.stationSelectionIndex == i);
                if (robotSelected) flags |= ImGuiTreeNodeFlags_Selected;

                const std::string label = node && !node->name.empty()
                    ? node->name
                    : "Robot " + std::to_string(i + 1);
                const bool open = ImGui::TreeNodeEx(label.c_str(), flags);
                // Selection on single click, entry on double. OpenOnArrow above keeps the expander
                // out of it, so opening a robot's details is not the same gesture as opening the
                // robot.
                if (ImGui::IsItemClicked() && !ImGui::IsItemToggledOpen()) {
                    selectStationObject(StationSelectionKind::Robot, i);
                    if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) enterRobotView(i);
                }

                // An arm on the end of a cable is called out in the tree: which arms are real and
                // which are simulated is the first thing you want off a cell, and it is not
                // otherwise visible from up here. More than one row can carry it now.
                drawHardwareMarker(robot);
                // Only when there is no frame row above to carry it; see the frame block at the top
                // of this loop.
                if (!hasOwnFrame) drawPlacementToggle(mutableRobotForFrame);

                if (open) {
                    if (static_cast<size_t>(i) < g_scene.station.robots.size()) {
                        const std::string& robotId =
                            g_scene.station.robots[static_cast<size_t>(i)].id;
                        for (int toolIndex = 0; toolIndex < accessoryCount; ++toolIndex) {
                            const StationAccessoryInstance& tool =
                                g_scene.station.accessories[static_cast<size_t>(toolIndex)];
                            if (tool.parentRobotId != robotId) continue;
                            ImGui::PushID("flange-tool");
                            ImGui::PushID(toolIndex);
                            ImGuiTreeNodeFlags toolFlags = ImGuiTreeNodeFlags_Leaf |
                                ImGuiTreeNodeFlags_NoTreePushOnOpen |
                                ImGuiTreeNodeFlags_SpanAvailWidth;
                            if (g_scene.stationSelectionKind == StationSelectionKind::Accessory &&
                                g_scene.stationSelectionIndex == toolIndex) {
                                toolFlags |= ImGuiTreeNodeFlags_Selected;
                            }
                            ImGui::TreeNodeEx(tool.name.empty() ? "Flange tool" : tool.name.c_str(),
                                              toolFlags);
                            if (ImGui::IsItemClicked()) {
                                selectStationObject(StationSelectionKind::Accessory, toolIndex);
                            }
                            ImGui::SameLine();
                            ImGui::TextDisabled("tool");
                            ImGui::PopID();
                            ImGui::PopID();
                        }
                    }
                    {
                        // The main thread's copy, refreshed once a frame by applyLiveRunResults. Reading
                        // the simulator's own would be reading state the live-run thread is writing.
                        const RobotProgramSimulator::LiveRunState& live = robot.liveRun;
                        RobotInstance& mutableRobot = *g_scene.robots[static_cast<size_t>(i)];
                        if (ImGui::TreeNodeEx("State", ImGuiTreeNodeFlags_SpanAvailWidth |
                                                           ImGuiTreeNodeFlags_DefaultOpen)) {
                            const size_t instanceIndex = static_cast<size_t>(i);
                            if (instanceIndex < g_scene.station.robots.size()) {
                                const StationRobotInstance& entry = g_scene.station.robots[instanceIndex];
                                ImGui::BulletText("Data in %s", entry.configRef.empty()
                                    ? "instances/<id>/config.json (not written yet)"
                                    : entry.configRef.c_str());
                            }
                            // World, because this row is read against the cell: two arms holding the
                            // same taught pose are in two different places, and a robot-local readout
                            // would give them the same three numbers and say nothing.
                            const CadTransform tool = robot.poseController.isBound()
                                ? robot.poseController.worldToolPose() : CadTransform();
                            ImGui::BulletText("Tool %.1f, %.1f, %.1f mm", tool.values[3],
                                              tool.values[7], tool.values[11]);
                            if (robot.buildRunning) {
                                ImGui::BulletText("Planning, %zu samples", robot.buildSentSamples);
                            } else if (robot.playing) {
                                ImGui::BulletText("Running, %.2f / %.2f s", robot.elapsedSeconds,
                                                  robot.durationSeconds);
                            } else if (!robot.timelineSamples.empty()) {
                                ImGui::BulletText("%.2f s planned", robot.durationSeconds);
                            } else {
                                ImGui::BulletText("Idle");
                            }
                            ImGui::BulletText("%s", robot.collisionText.c_str());

                            // Simulate on the arm's own row, because that is the arm it plans. Pressing
                            // it selects that arm first: the build worker, the poll that collects it and
                            // the playback clock are all written around one arm at a time, and making
                            // several plan at once is a bigger change than putting the button here.
                            const bool editing = !robot.buildRunning && !robot.playing;
                            const float half =
                                (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) * 0.5f;
                            ImGui::BeginDisabled(!editing);
                            if (ImGui::Button("Simulate", ImVec2(half, 0.0f))) {
                                g_scene.view.robot = i;
                                beginTrajectoryBuild(true);
                            }
                            ImGui::EndDisabled();
                            ImGui::SameLine();
                            ImGui::BeginDisabled(editing);
                            if (ImGui::Button("Stop", ImVec2(half, 0.0f))) {
                                g_scene.view.robot = i;
                                cancelTrajectoryBuild();
                                mutableRobot.playing = false;
                            }
                            ImGui::EndDisabled();
                            if (!robot.buildStatus.empty()) {
                                ImGui::TextWrapped("%s", robot.buildStatus.c_str());
                            }
                            // The gimbal reports IK failures here, next to the arm it was dragged on.
                            if (!robot.statusText.empty()) {
                                ImGui::TextWrapped("%s", robot.statusText.c_str());
                            }

                            const int currentMode = !live.running ? 0 : (live.looping ? 2 : 1);
                            int mode = currentMode;
                            ImGui::TextUnformatted("Run program");
                            ImGui::RadioButton("Off", &mode, 0);
                            ImGui::SameLine();
                            ImGui::RadioButton("Single", &mode, 1);
                            ImGui::SameLine();
                            ImGui::RadioButton("Loop", &mode, 2);
                            if (mode != currentMode) {
                                if (mode == 0) {
                                    stopLiveRunFor(mutableRobot);
                                } else {
                                    startLiveRunFor(mutableRobot, mode == 2);
                                }
                            }

                            if (!live.status.empty()) ImGui::BulletText("%s", live.status.c_str());

                            ImGui::PushID("defaults");
                            nativeSpinBox("##joint", &mutableRobot.programJointSpeedDegPerSec, 1.0,
                                          "%.1f deg/s", 0.1, 5000.0, 130.0f, "Joint");
                            nativeSpinBox("##linear", &mutableRobot.programLinearSpeedMmPerSec, 1.0,
                                          "%.1f mm/s", 0.1, 1000000.0, 130.0f, "Linear");
                            nativeSpinBox("##blend", &mutableRobot.programBlendMm, 0.1,
                                          "%.2f mm", -1.0, 1000.0, 130.0f, "Blend");
                            ImGui::PopID();
                            if (live.running) {
                                ImGui::TextDisabled("Edits apply to the next run.");
                            }

                            if (live.running || live.completedCycles > 0) {
                                ImGui::BulletText("Instruction %d, cycle %d", live.instruction + 1,
                                                  live.completedCycles + (live.running ? 1 : 0));
                                // Into this cycle, not since the run began: on a looping run the total
                                // only says how long you have been watching.
                                ImGui::BulletText("%.2f s this cycle, %.2f s last cycle",
                                                  live.cycleElapsedSeconds, live.cycleSeconds);
                                ImGui::BulletText("Program has set joint %.1f deg/s, linear %.1f mm/s",
                                                  live.jointSpeedRadPerSec / kDegToRad,
                                                  live.linearSpeedMmPerSec);
                                ImGui::BulletText("Program has set blend %.1f mm", live.blendRadiusMm);
                                if (!live.weaveEnabled) {
                                    ImGui::BulletText("Weave off");
                                } else if (live.weaveScheduleIndex >= 0) {
                                    ImGui::BulletText("Weave schedule %d", live.weaveScheduleIndex);
                                } else {
                                    ImGui::BulletText("Weave inline, %s",
                                                      RobotMotionCore::weaveShapeName(live.weaveInline.shape));
                                }
                            }
                            ImGui::TreePop();
                        }
                    }

                    const int programCount = static_cast<int>(robot.programs.size());
                    if (ImGui::TreeNodeEx("Programs", ImGuiTreeNodeFlags_SpanAvailWidth)) {
                        for (int p = 0; p < programCount; ++p) {
                            const RobotProgram& entry = robot.programs[static_cast<size_t>(p)];
                            const bool isActive = p == robot.selectedProgramIndex();
                            ImGui::PushID(p);

                            const ImVec2 origin = ImGui::GetCursorScreenPos();
                            const float lineHeight = ImGui::GetTextLineHeight();
                            if (isActive) {
                                ImGui::GetWindowDrawList()->AddCircleFilled(
                                    ImVec2(origin.x + 6.0f, origin.y + lineHeight * 0.5f), 4.0f,
                                    IM_COL32(245, 175, 60, 255));
                            }
                            ImGui::Dummy(ImVec2(16.0f, lineHeight));
                            ImGui::SameLine();

                            const size_t count = entry.root ? entry.root->children.size() : 0;
                            const std::string label =
                                (entry.name.empty() ? "Program " + std::to_string(p + 1) : entry.name) +
                                "   (" + std::to_string(count) + ")";
                            // Selecting here makes it that arm's active program, and double-click
                            // goes in to work on it - the same gesture the robot rows use.
                            if (ImGui::Selectable(label.c_str(), isActive,
                                                  ImGuiSelectableFlags_AllowDoubleClick)) {
                                selectProgram(*g_scene.robots[static_cast<size_t>(i)], p);
                                if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) enterRobotView(i);
                            }
                            ImGui::PopID();
                        }
                        ImGui::TreePop();
                    }
                    ImGui::TreePop();
                }
                if (hasOwnFrame) ImGui::TreePop();
                ImGui::PopID();
            }
            ImGui::TreePop();
        }
    }
    ImGui::EndChild();

    if (ImGui::Button("Work on this robot", ImVec2(-1.0f, 0.0f))) enterRobotView(g_scene.view.robot);
    ImGui::TextDisabled("Or double-click it here or in the 3D view.");

    // No "bind hardware to this robot" here any more. Each arm opens its own port from its own
    // Hardware IO panel, so choosing the port for an arm and choosing which arm the hardware means
    // are the same act - there is no second decision left for the cell to make.

    // No "selected arm" section here. A control either belongs to one arm, in which case it belongs on
    // that arm's row in the tree where you can see which arm you are aiming at, or it belongs to the
    // cell, in which case it applies to all of them.

    ImGui::Spacing();
    if (ImGui::Button("Save station...", ImVec2(-1.0f, 0.0f))) saveStationToFile();
    ImGui::TextDisabled("Writes each arm's calibration to its own folder.");
    ImGui::Checkbox("Reference built-in assets", &g_scene.saveStationUsingBuiltins);
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Names shipped robots, mechanisms and accessories instead of embedding\n"
                          "copies. The saved station then needs a build that ships those assets.");
    }
    if (!activeRobot().hardwareStatus.empty() && activeRobot().hardwareStatus != "-") {
        ImGui::TextWrapped("%s", activeRobot().hardwareStatus.c_str());
    }

    // Cell-wide, because arms in a cell share a clock: running one at ten times real time and its
    // neighbour at one would show a sequence that never happens.
    ImGui::Spacing();
    const int speedIndex = clampedLiveSpeedIndex();
    if (g_scene.measuredLiveSpeedValid) {
        const double asked = kLiveSpeedFactors[speedIndex];
        const bool keepingUp = achievedRateKeepingUp(g_scene.measuredLiveSpeed, asked);
        if (keepingUp) {
            ImGui::TextDisabled("Running at %.2fx real time", g_scene.measuredLiveSpeed);
        } else {
            // The shortfall as a rate, against the rate that was asked for. Deliberately not a
            // running total of dropped simulated time - a climbing counter reads as an error tally.
            ImGui::TextColored(achievedRateCpuBoundColor(),
                               "Running at %.2fx of the %.0fx asked (CPU bound)",
                               g_scene.measuredLiveSpeed, asked);
            ImGui::TextDisabled("Same motion, less of it per second - not a coarser step.");
        }
    } else {
        ImGui::TextDisabled("Running at - (no arm is running)");
    }

    ImGui::TextDisabled("Drag to turn, middle or right drag to pan, wheel to zoom.");

    ImGui::Spacing();
    if (ImGui::Button("Open...", ImVec2(-1.0f, 0.0f))) {
#ifdef __EMSCRIPTEN__
        WebFiles::requestOpen(WebFiles::Purpose::Package, ".zip,.json");
#else
        const std::string packageFile = runPackageDialog();
        if (!packageFile.empty()) loadPackageIntoScene(packageFile);
#endif
    }
}

// The station's programming-oriented left panel.  This is deliberately an overview across the
// cell, not the low-level instruction editor already available inside Robot view: choose which arm
// and package program the station is concerned with here, simulate it from the persistent bar, and
// enter the arm only when detailed editing or hardware control is wanted.
void drawStationProgramOverviewPanel() {
    if (!g_scene.viewer || g_scene.robots.empty()) {
        ImGui::TextDisabled("No robots are available in this station.");
        return;
    }

    const char* stationName = !g_scene.station.name.empty()
        ? g_scene.station.name.c_str() : g_scene.packageLabel.c_str();
    ImGui::TextUnformatted(stationName);
    ImGui::TextDisabled("Station programming overview");
    ImGui::Separator();

    const int selectedRobot = g_scene.activeRobotIndex();
    const CadNode* selectedNode = activeRobot().poseController.robotNode();
    const char* selectedName = selectedNode && !selectedNode->name.empty()
        ? selectedNode->name.c_str() : "Robot";
    ImGui::SetNextItemWidth(-1.0f);
    if (ImGui::BeginCombo("##stationProgramRobot", selectedName)) {
        for (int i = 0; i < static_cast<int>(g_scene.robots.size()); ++i) {
            const RobotInstance& robot = *g_scene.robots[static_cast<size_t>(i)];
            const CadNode* node = robot.poseController.robotNode();
            const std::string label = node && !node->name.empty()
                ? node->name : "Robot " + std::to_string(i + 1);
            if (ImGui::Selectable(label.c_str(), i == selectedRobot)) g_scene.view.robot = i;
        }
        ImGui::EndCombo();
    }
    drawHardwareMarker(activeRobot());

    ImGui::Spacing();
    ImGui::TextUnformatted("Robot programs");
    if (ImGui::BeginChild("stationProgramList", ImVec2(0.0f, 190.0f),
                          ImGuiChildFlags_Borders | ImGuiChildFlags_ResizeY)) {
        RobotInstance& robot = activeRobot();
        for (int i = 0; i < static_cast<int>(robot.programs.size()); ++i) {
            const RobotProgram& program = robot.programs[static_cast<size_t>(i)];
            const size_t instructionCount = program.root ? program.root->children.size() : 0;
            const std::string label =
                (program.name.empty() ? "Program " + std::to_string(i + 1) : program.name) +
                "   (" + std::to_string(instructionCount) + ")";
            if (ImGui::Selectable(label.c_str(), i == robot.selectedProgramIndex(),
                                  ImGuiSelectableFlags_AllowDoubleClick)) {
                selectProgram(robot, i);
                if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
                    enterRobotView(g_scene.activeRobotIndex());
                }
            }
        }
    }
    ImGui::EndChild();

    const RobotInstance& robot = activeRobot();
    if (robot.buildRunning) {
        ImGui::Text("Planning... %zu samples", robot.buildSentSamples);
    } else if (robot.playing) {
        ImGui::Text("Simulating %.2f / %.2f s", robot.elapsedSeconds, robot.durationSeconds);
    } else if (robot.liveRun.running) {
        ImGui::Text("Running instruction %d", robot.liveRun.instruction + 1);
    } else if (!robot.timelineSamples.empty()) {
        ImGui::Text("Ready: %.2f s planned", robot.durationSeconds);
    } else {
        ImGui::TextDisabled("Stopped");
    }
    if (!robot.buildStatus.empty()) ImGui::TextWrapped("%s", robot.buildStatus.c_str());
    if (!robot.statusText.empty()) ImGui::TextWrapped("%s", robot.statusText.c_str());

    if (ImGui::Button("Work on this robot", ImVec2(-1.0f, 0.0f))) {
        enterRobotView(g_scene.activeRobotIndex());
    }
    ImGui::TextDisabled("Jogging and hardware control open in Robot view.");

    ImGui::Separator();
    const bool editing = !activeRobot().buildRunning && !activeRobot().playing &&
        !activeRobot().liveRun.running && !activeRobot().hardware.programActive();
    drawProgramState(activeRobot(), editing, "stationProgramState");
    ImGui::Separator();
    drawProgramInstructionEditor(activeRobot(), 0.0f);

}

