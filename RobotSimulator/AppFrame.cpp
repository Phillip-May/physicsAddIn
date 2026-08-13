#include "AppFrame.h"

#include "AccessoryBuilders.h"
#include "AppState.h"
#include "ContextBar.h"
#include "ConveyorRuntime.h"
#include "HardwareIoPanel.h"
#include "LibraryPlacement.h"
#include "MasteringIo.h"
#include "PlacementWindows.h"
#include "ProgramEditing.h"
#include "ProgramPanel.h"
#include "RobotLibraryPanel.h"
#include "RobotPanel.h"
#include "SimulationInfoPanel.h"
#include "StationPanels.h"
#include "StationSceneLoad.h"
#include "TimelinePanel.h"
#include "TrajectoryAndLiveRun.h"
#include "UiFormHelpers.h"
#include "ViewerBridge.h"
#include "WebFiles.h"

#include "JsonCompat.h"
#include "imgui.h"
#include "imgui_internal.h"

#include <fstream>
#include <iterator>
#include <string>
namespace {
using namespace progtext;

using strutil::operator<<;

} // namespace

#ifdef __EMSCRIPTEN__
void pollWebFileRequests() {
    std::string path;
    WebFiles::Purpose purpose = WebFiles::Purpose::None;
    if (!WebFiles::takeOpened(&path, &purpose)) return;

    const std::string fileName = path;
    switch (purpose) {
    case WebFiles::Purpose::Package:
        // Clears the "Choose a robot package..." prompt; loadPackageIntoScene only writes
        // statusText from the gimbal callback, so it would otherwise linger after the load.
        activeRobot().statusText.clear();
        loadPackageIntoScene(fileName);
        break;
    case WebFiles::Purpose::Program:
        loadProgramFromPath(fileName);
        break;
    case WebFiles::Purpose::Mastering: {
        std::ifstream file(fileName, std::ios::binary);
        const std::string bytes((std::istreambuf_iterator<char>(file)),
                                std::istreambuf_iterator<char>());
        const Json document = Json::parse(bytes, nullptr, /*allow_exceptions=*/false);
        if (document.is_discarded() || !document.is_object()) {
            activeRobot().hardwareStatus = "Load failed: not a JSON object.";
        } else {
            applyLoadedMasteringDocument(document, "the chosen file");
        }
        break;
    }
    case WebFiles::Purpose::None:
        break;
    }
}
#endif

void drawRobotSimulatorUi(const std::string& initialPackage) {
    ensureSceneLoaded(initialPackage);
#ifdef __EMSCRIPTEN__
    pollWebFileRequests();
#endif
    pollTrajectoryBuild();
    pollAllHardware();
    // Keep the browser fallback deterministic and provide the current pose seed for desktop
    // physics. This is cheap (thirteen transforms for the WS2000) and follows carriage edits made
    // by the gantry slider without introducing a second mount node.
    for (DragChainPoseController& chain : g_scene.dragChains) chain.update();
    for (DragChainPhysics& physics : g_scene.dragChainPhysics) {
        physics.step(static_cast<double>(ImGui::GetIO().DeltaTime));
    }
    stepConveyors(static_cast<double>(ImGui::GetIO().DeltaTime));
    // No live-run stepping here. It happens on the live-run thread, and its results were applied to the
    // scene before this frame opened - see applyLiveRunResults and beforeFrame in main().

    // Escape leaves a robot context, the way it closes one anywhere else. Suppressed while a text
    // field has the keyboard, where Escape means "abandon this edit" and stealing it would throw
    // the operator out of the arm they were typing a number into.
    if (g_scene.view.kind == ViewTarget::Kind::Robot && !ImGui::GetIO().WantTextInput &&
        ImGui::IsKeyPressed(ImGuiKey_Escape, false)) {
        exitToStationView();
    }
    stepCameraTween(ImGui::GetIO().DeltaTime);

    refreshPoseDerivedReadoutsIfMoved();
    applySceneViewState();

    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(viewport->WorkPos);
    ImGui::SetNextWindowSize(viewport->WorkSize);
    ImGui::SetNextWindowViewport(viewport->ID);

    // A full-viewport host window with no decoration of its own, so the dockspace fills the
    // canvas and panels can be rearranged over it.
    ImGuiWindowFlags hostFlags = ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoTitleBar |
                                 ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize |
                                 ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoBringToFrontOnFocus |
                                 ImGuiWindowFlags_NoNavFocus;
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
    ImGui::Begin("RobotSimulatorHost", nullptr, hostFlags);
    ImGui::PopStyleVar(3);

    const ImGuiID dockspaceId = ImGui::GetID("RobotSimulatorDockspace");
    static bool dockLayoutBuilt = false;
    if (!dockLayoutBuilt && ImGui::DockBuilderGetNode(dockspaceId) == nullptr) {
        dockLayoutBuilt = true;
        ImGui::DockBuilderRemoveNode(dockspaceId);
        ImGui::DockBuilderAddNode(dockspaceId, ImGuiDockNodeFlags_DockSpace | ImGuiDockNodeFlags_NoWindowMenuButton);
        ImGui::DockBuilderSetNodeSize(dockspaceId, viewport->WorkSize);

        ImGuiID remaining = dockspaceId;
        const ImGuiID leftId = ImGui::DockBuilderSplitNode(remaining, ImGuiDir_Left, 0.22f, nullptr, &remaining);
        const ImGuiID rightId = ImGui::DockBuilderSplitNode(remaining, ImGuiDir_Right, 0.28f, nullptr, &remaining);
        const ImGuiID bottomId = ImGui::DockBuilderSplitNode(remaining, ImGuiDir_Down, 0.08f, nullptr, &remaining);
        const ImGuiID analysisId = ImGui::DockBuilderSplitNode(remaining, ImGuiDir_Down, 0.42f, nullptr, &remaining);

        ImGui::DockBuilderDockWindow("Program", leftId);
        ImGui::DockBuilderDockWindow("Robot", rightId);
        ImGui::DockBuilderDockWindow("Properties", rightId);
        ImGui::DockBuilderDockWindow("Timeline", bottomId);
        ImGui::DockBuilderDockWindow("Simulation Info", analysisId);
        ImGui::DockBuilderDockWindow("Hardware IO", analysisId);
        ImGui::DockBuilderDockWindow("3D View", remaining);
        ImGui::DockBuilderDockWindow("Station", leftId);
        ImGui::DockBuilderFinish(dockspaceId);
    }
    const bool inRobotContext = g_scene.view.kind == ViewTarget::Kind::Robot;
    if (inRobotContext) {
        ImGui::GetWindowDrawList()->AddRectFilled(
            viewport->WorkPos,
            ImVec2(viewport->WorkPos.x + viewport->WorkSize.x,
                   viewport->WorkPos.y + viewport->WorkSize.y),
            ImGui::GetColorU32(kContextFrameColor));
    }
    drawPersistentContextBar();
    if (inRobotContext) ImGui::SetCursorPosX(ImGui::GetCursorPosX() + kRobotContextInset);
    // NoWindowMenuButton removes the small triangle each docked panel otherwise shows in its
    // tab strip.
    const ImVec2 dockSize = inRobotContext ? ImVec2(-kRobotContextInset, -kRobotContextInset)
                                           : ImVec2(0.0f, 0.0f);
    ImGui::DockSpace(dockspaceId, dockSize, ImGuiDockNodeFlags_NoWindowMenuButton);
    ImGui::End();

    const auto placeholder = [](const char* what) {
        ImGui::TextDisabled("%s", what);
    };

    ImGui::Begin("3D View");
    if (!g_scene.viewer) {
        placeholder(g_scene.error.empty() ? "No robot package loaded."
                                            : g_scene.error.c_str());
    } else {
        // The scene is drawn into its own framebuffer and shown as an image, which is what
        // keeps it inside the docked panel: one GL context for both the UI and the scene.
        const ImVec2 available = ImGui::GetContentRegionAvail();
        const int targetWidth = static_cast<int>(available.x);
        const int targetHeight = static_cast<int>(available.y);
        if (targetWidth > 0 && targetHeight > 0) {
            const unsigned int texture = g_scene.viewer->render(targetWidth, targetHeight);
            if (texture != 0) {
                const ImVec2 imageOrigin = ImGui::GetCursorScreenPos();
                // uv flipped vertically: GL textures start bottom-left, ImGui expects top-left.
                ImGui::Image(static_cast<ImTextureID>(texture),
                             ImVec2(static_cast<float>(targetWidth), static_cast<float>(targetHeight)),
                             ImVec2(0.0f, 1.0f), ImVec2(1.0f, 0.0f));

                // Pointer input is forwarded in image-local pixels, which is the space the
                // gimbal hit test and drag maths already work in.
                const ImVec2 mouse = ImGui::GetIO().MousePos;
                const PointI localPos(static_cast<int>(mouse.x - imageOrigin.x),
                                      static_cast<int>(mouse.y - imageOrigin.y));
                const bool hovered = ImGui::IsItemHovered();
                bool libraryPlacementConsumedWheel = false;

                if (g_scene.view.kind == ViewTarget::Kind::Sim &&
                    ImGui::BeginDragDropTarget()) {
                    const ImGuiPayload* payload = ImGui::AcceptDragDropPayload(
                        RobotLibraryPanel::kLibraryAssetPayload,
                        ImGuiDragDropFlags_AcceptBeforeDelivery |
                            ImGuiDragDropFlags_AcceptNoDrawDefaultRect);
                    if (payload && payload->Data && payload->DataSize > 1) {
                        RobotLibraryPanel::AssetRequest request;
                        if (RobotLibraryPanel::decodeAssetPayload(
                                payload->Data, payload->DataSize, &request) &&
                            beginLibraryRobotPlacement(request)) {
                            const float wheelDelta = ImGui::GetIO().MouseWheel;
                            if (wheelDelta != 0.0f) {
                                rotateLibraryRobotPlacement(wheelDelta);
                                libraryPlacementConsumedWheel = true;
                            }
                            updateLibraryRobotPlacement(localPos, targetWidth, targetHeight);
                            if (payload->Delivery) {
                                if (g_libraryPlacement.snapped()) {
                                    commitLibraryRobotPlacement();
                                } else {
                                    cancelLibraryRobotPlacement();
                                }
                            }
                        }
                    }
                    ImGui::EndDragDropTarget();
                }

                if (g_libraryPlacement.active()) {
                    const std::string previewText = g_libraryPlacement.displayName + "\n" +
                                                    g_libraryPlacement.status();
                    const ImVec2 textSize = ImGui::CalcTextSize(previewText.c_str());
                    const ImVec2 boxMin(imageOrigin.x + 12.0f, imageOrigin.y + 12.0f);
                    const ImVec2 boxMax(boxMin.x + textSize.x + 18.0f,
                                        boxMin.y + textSize.y + 14.0f);
                    ImDrawList* drawList = ImGui::GetWindowDrawList();
                    drawList->AddRectFilled(boxMin, boxMax, IM_COL32(20, 22, 24, 220), 4.0f);
                    drawList->AddRect(
                        boxMin, boxMax,
                        g_libraryPlacement.snapped() ? IM_COL32(70, 235, 115, 255)
                                                   : IM_COL32(255, 155, 45, 255),
                        4.0f, 0, 2.0f);
                    drawList->AddText(ImVec2(boxMin.x + 9.0f, boxMin.y + 7.0f),
                                      IM_COL32(245, 245, 245, 255), previewText.c_str());
                }

                // Taking hold of the camera cancels any travel in progress, in place. Letting the
                // move continue underneath a drag would have the scene pulling away from the hand
                // moving it.
                if (hovered && (ImGui::IsMouseClicked(ImGuiMouseButton_Left) ||
                                ImGui::IsMouseClicked(ImGuiMouseButton_Middle) ||
                                ImGui::IsMouseClicked(ImGuiMouseButton_Right) ||
                                (ImGui::GetIO().MouseWheel != 0.0f &&
                                 !libraryPlacementConsumedWheel))) {
                    cancelCameraTween();
                }

                // Resolve the gimbal before scene picking. Its handles are renderer overlays rather
                // than CAD nodes, so the model picker correctly reports "nothing" underneath some
                // handles; without this ordering, a precise transform press would clear the object
                // it was about to move.
                bool leftPressClaimedByGimbal = false;
                if (hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
                    leftPressClaimedByGimbal = g_scene.viewer->onPointerPressed(localPos);
                    if (leftPressClaimedByGimbal) {
                        g_scene.objectDragCandidateKind = StationSelectionKind::None;
                        g_scene.objectDragCandidateIndex = -1;
                    }
                }

                // Layout selection is the same gesture in the tree and the viewport. The picker
                // works against whole model roots, so a click on any mesh in a rail or accessory
                // selects its station instance and gives the outline/properties panel the root.
                if (hovered && g_scene.view.kind == ViewTarget::Kind::Sim &&
                    ImGui::IsMouseClicked(ImGuiMouseButton_Left) &&
                    !g_libraryPlacement.active() && !leftPressClaimedByGimbal) {
                    g_scene.objectDragCandidateKind = StationSelectionKind::None;
                    g_scene.objectDragCandidateIndex = -1;
                    struct PickTarget {
                        CadNode* node = nullptr;
                        StationSelectionKind kind = StationSelectionKind::None;
                        int index = -1;
                    };
                    std::vector<PickTarget> targets;
                    for (int i = 0; i < static_cast<int>(g_scene.robots.size()); ++i) {
                        targets.push_back({g_scene.robots[i]->poseController.robotNode(),
                                           StationSelectionKind::Robot, i});
                    }
                    if (g_scene.stationMode == StationMode::Layout) {
                        for (int i = 0; i < static_cast<int>(g_scene.gantries.size()); ++i) {
                            targets.push_back({g_scene.gantries[i].gantryNode(),
                                               StationSelectionKind::Mechanism, i});
                        }
                        for (int i = 0;
                             i < static_cast<int>(g_scene.station.accessories.size()); ++i) {
                            targets.push_back({g_scene.station.accessories[i].node,
                                               StationSelectionKind::Accessory, i});
                        }
                    }
                    std::vector<CadNode*> candidates;
                    candidates.reserve(targets.size());
                    for (const PickTarget& target : targets) {
                        if (target.node) candidates.push_back(target.node);
                    }
                    bool pickedStationObject = false;
                    if (const CadNode* picked = g_scene.viewer->pickNodeAt(localPos, candidates)) {
                        const auto hit = std::find_if(
                            targets.begin(), targets.end(),
                            [picked](const PickTarget& target) { return target.node == picked; });
                        if (hit != targets.end()) {
                            pickedStationObject = true;
                            selectStationObject(hit->kind, hit->index);
                            if (g_scene.stationMode == StationMode::Layout &&
                                g_scene.moveObjects &&
                                (hit->kind == StationSelectionKind::Mechanism ||
                                 hit->kind == StationSelectionKind::Accessory)) {
                                g_scene.objectDragCandidateKind = hit->kind;
                                g_scene.objectDragCandidateIndex = hit->index;
                            }
                            // Double-click keeps the existing fast path into low-level robot view.
                            if (hit->kind == StationSelectionKind::Robot &&
                                ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
                                enterRobotView(hit->index);
                            }
                        }
                    }
                    if (!pickedStationObject) {
                        selectStationObject(StationSelectionKind::None, -1);
                    }
                }

                if (!g_libraryPlacement.active() &&
                    g_scene.view.kind == ViewTarget::Kind::Sim &&
                    g_scene.stationMode == StationMode::Layout && g_scene.moveObjects &&
                    g_scene.objectDragCandidateKind != StationSelectionKind::None &&
                    ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
                    if (beginExistingStationObjectPlacement(
                            g_scene.objectDragCandidateKind,
                            g_scene.objectDragCandidateIndex)) {
                        // The unclaimed press began as a camera orbit. Once the model is picked up,
                        // end that orbit so the object follows a stable view under the cursor.
                        g_scene.viewer->onPointerReleased();
                    } else {
                        g_scene.objectDragCandidateKind = StationSelectionKind::None;
                        g_scene.objectDragCandidateIndex = -1;
                    }
                }

                if (g_libraryPlacement.active() && g_libraryPlacement.movingExisting &&
                    ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
                    const float wheelDelta = ImGui::GetIO().MouseWheel;
                    if (wheelDelta != 0.0f) {
                        rotateLibraryRobotPlacement(wheelDelta);
                        libraryPlacementConsumedWheel = true;
                    }
                    updateLibraryRobotPlacement(localPos, targetWidth, targetHeight);
                }
                if (ImGui::IsMouseDown(ImGuiMouseButton_Left) &&
                    !(g_libraryPlacement.active() && g_libraryPlacement.movingExisting) &&
                    g_scene.objectDragCandidateKind == StationSelectionKind::None) {
                    g_scene.viewer->onPointerMoved(localPos, true);
                }

                // Panning, on the middle and right buttons. Both, because which one means "pan" is
                // a matter of what CAD tool you came from, and neither is doing anything else here.
                // The pan direction sign and the pick-box generosity have not been visually verified.
                for (const ImGuiMouseButton panButton : {ImGuiMouseButton_Middle, ImGuiMouseButton_Right}) {
                    if (hovered && ImGui::IsMouseClicked(panButton)) g_scene.viewer->onPanPressed(localPos);
                    if (ImGui::IsMouseDown(panButton)) g_scene.viewer->onPanMoved(localPos);
                    if (ImGui::IsMouseReleased(panButton)) g_scene.viewer->onPanReleased();
                }
                if (ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
                    if (g_libraryPlacement.active() && g_libraryPlacement.movingExisting) {
                        if (g_libraryPlacement.snapped()) {
                            commitLibraryRobotPlacement();
                        } else {
                            cancelLibraryRobotPlacement();
                        }
                    }
                    g_scene.objectDragCandidateKind = StationSelectionKind::None;
                    g_scene.objectDragCandidateIndex = -1;
                    g_scene.viewer->onPointerReleased();
                }
                if (hovered && ImGui::GetIO().MouseWheel != 0.0f &&
                    !libraryPlacementConsumedWheel && !g_libraryPlacement.active()) {
                    g_scene.viewer->onScroll(static_cast<double>(ImGui::GetIO().MouseWheel));
                }
            } else {
                placeholder("Scene renderer failed to initialise; see stderr.");
            }
        }
    }
    ImGui::End();

    // Both workspaces share the scene renderer; only their panels differ.
    if (g_scene.view.kind == ViewTarget::Kind::Sim) {
        ImGui::Begin("Station");
        if (ImGui::BeginTabBar("stationWorkspaceTabs")) {
            const auto requestedFlags = [](StationPanelTab tab) {
                return g_scene.stationPanelTabSelectionPending &&
                       g_scene.stationPanelTab == tab
                    ? ImGuiTabItemFlags_SetSelected : ImGuiTabItemFlags_None;
            };
            // Keep a toolbar tab request pending until ImGui reports that tab active.
            const auto activateTab = [](StationPanelTab tab, StationMode mode) {
                if (!g_scene.stationPanelTabSelectionPending ||
                    g_scene.stationPanelTab == tab) {
                    g_scene.stationPanelTab = tab;
                    g_scene.setStationMode(mode);
                    g_scene.stationPanelTabSelectionPending = false;
                }
            };
            if (ImGui::BeginTabItem("Layout", nullptr,
                                    requestedFlags(StationPanelTab::Layout))) {
                activateTab(StationPanelTab::Layout, StationMode::Layout);
                drawStationPanel();
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("Program", nullptr,
                                    requestedFlags(StationPanelTab::Program))) {
                activateTab(StationPanelTab::Program, StationMode::Program);
                drawStationProgramOverviewPanel();
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("Library", nullptr,
                                    requestedFlags(StationPanelTab::Library))) {
                activateTab(StationPanelTab::Library, StationMode::Layout);
                g_robotLibrary.draw([](const RobotLibraryPanel::AssetRequest& request) {
                    loadPackageIntoScene(request.packagePath);
                }, &g_scene.mountingSnapScreenPercent,
                [](CadNode* root, const RobotLibraryPanel::AssetRequest&) {
                    rebuildParametricAccessory(root, g_scene.station);
                });
                ImGui::EndTabItem();
            }
            ImGui::EndTabBar();
        }
        ImGui::End();
        ImGui::Begin("Properties");
        drawStationPropertiesPanel();
        ImGui::End();
        // A library ghost requires its ImGui payload to remain alive. Existing-object movement is
        // an ordinary viewport drag and has no payload, so it is committed/cancelled by the mouse
        // release path above instead of being mistaken for a vanished Library source here.
        const ImGuiPayload* activePayload = ImGui::GetDragDropPayload();
        if (g_libraryPlacement.active() && !g_libraryPlacement.movingExisting &&
            (!activePayload ||
             !activePayload->IsDataType(RobotLibraryPanel::kLibraryAssetPayload))) {
            cancelLibraryRobotPlacement();
        }
        drawPlacementWindows();
        return;
    }

    ImGui::Begin("Program");
    if (!initialPackage.empty()) {
        ImGui::TextWrapped("%s", initialPackage.c_str());
        ImGui::Separator();
    }
    drawProgramPanel();
    ImGui::End();

    ImGui::Begin("Robot");
    drawRobotPanel();
    ImGui::End();

    ImGui::Begin("Simulation Info");
    drawSimulationInfoPanel();
    ImGui::End();

    ImGui::Begin("Hardware IO");
    drawHardwareIoPanel();
    ImGui::End();

    ImGui::Begin("Timeline");
    drawTimelinePanel();
    ImGui::End();

    drawPlacementWindows();
}

