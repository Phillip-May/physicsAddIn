#include "LibraryPlacement.h"
#include "AccessoryBuilders.h"
#include "AccessoryGeometry.h"
#include "CadNodePackage.h"
#include "DragChainPhysics.h"
#include "MasteringIo.h"
#include "MeshRobotViewer.h"
#include "SceneMath.h"
#include "StationSceneLoad.h"
#include "ViewerBridge.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iterator>
#include <memory>
#include <utility>
#include <vector>

namespace {
struct ViewerCamera final : mountingsnap::View {
    explicit ViewerCamera(const MeshRobotViewer& viewer) : viewer(viewer) {}

    bool project(const CadVec3& worldMm, double* pixelX, double* pixelY) const override {
        PointF screen;
        if (!viewer.projectWorldPoint(Vec3f(static_cast<float>(worldMm.x),
                                            static_cast<float>(worldMm.y),
                                            static_cast<float>(worldMm.z)), screen)) {
            return false;
        }
        *pixelX = screen.x();
        *pixelY = screen.y();
        return true;
    }

    bool rayThrough(double pixelX, double pixelY, CadVec3* originMm,
                    CadVec3* direction) const override {
        Vec3f origin;
        Vec3f along;
        if (!viewer.worldRayAt(PointI(static_cast<int>(pixelX), static_cast<int>(pixelY)),
                               origin, along)) {
            return false;
        }
        *originMm = CadVec3(origin.x(), origin.y(), origin.z());
        *direction = CadVec3(along.x(), along.y(), along.z());
        return true;
    }

    const MeshRobotViewer& viewer;
};

// A station's own rule about what may be mated to: everything inside a robot is off limits unless a
// tool is being placed, in which case the flange is the only thing that is on.
struct StationTargets final : mountingsnap::TargetFilter {
    bool toolPackage = false;

    bool eligible(const CadNode* node, bool insideRobot) const override {
        return toolPackage ? (insideRobot && node->mountingHoles.interfaceId == "robot_flange")
                           : !insideRobot;
    }
};

std::vector<Vec3f> asVec3f(const std::vector<CadVec3>& points) {
    std::vector<Vec3f> converted;
    converted.reserve(points.size());
    for (const CadVec3& point : points) {
        converted.emplace_back(static_cast<float>(point.x), static_cast<float>(point.y),
                               static_cast<float>(point.z));
    }
    return converted;
}

} // namespace

std::shared_ptr<CadNode> sharedSceneNode(const std::shared_ptr<CadNode>& node,
                                        const CadNode* target) {
    if (!node || !target) return {};
    if (node.get() == target) return node;
    for (const std::shared_ptr<CadNode>& child : node->children) {
        if (std::shared_ptr<CadNode> match = sharedSceneNode(child, target)) return match;
    }
    return {};
}

void removePlacementFrameFromScene() {
    if (!g_libraryPlacement.frame) return;
    if (g_libraryPlacement.movingExisting) return;
    CadNode* parent = g_libraryPlacement.frame->parent;
    if (parent) {
        parent->children.erase(
            std::remove_if(parent->children.begin(), parent->children.end(),
                           [](const std::shared_ptr<CadNode>& child) {
                               return child.get() == g_libraryPlacement.frame.get();
                           }),
            parent->children.end());
    }
    g_libraryPlacement.frame->parent = nullptr;
}

void cancelLibraryRobotPlacement() {
    if (g_scene.viewer) {
        g_scene.viewer->setPlacementPreviewRoot(nullptr, false);
        g_scene.viewer->setPlacementMountingGuides({}, {});
    }
    if (g_libraryPlacement.movingExisting && g_libraryPlacement.frame) {
        g_libraryPlacement.frame->loc = g_libraryPlacement.originalLocalPose;
        g_libraryPlacement.frame->needsGlobalLocUpdate = true;
    }
    removePlacementFrameFromScene();
    g_libraryPlacement = LibraryRobotPlacement();
    if (g_scene.viewer) g_scene.viewer->markCacheDirty();
}

bool beginLibraryRobotPlacement(const RobotLibraryPanel::AssetRequest& request) {
    const std::string variantKey = request.packagePath + "|" + request.variantId + "|" +
                                   request.parameters.dump();
    if (g_libraryPlacement.active() && g_libraryPlacement.variantKey == variantKey) return true;
    cancelLibraryRobotPlacement();
    // A cell assembled in this session has no stationSource until its first save. Requiring a
    // source file here made every asset drag fail in the new empty view (and in a loose robot
    // package) even though both already have a perfectly valid scene root to receive the asset.
    if (!g_scene.viewer || !g_scene.root) return false;

    std::string errorMessage;
    const std::string& packagePath = request.packagePath;
    std::shared_ptr<CadNode> packageRoot = loadCadNodePackage(packagePath, &errorMessage);
    if (!packageRoot) return false;
    if (TransformNodeData* transform = packageRoot->asTransform()) {
        transform->applyAccessoryParametersJson(request.parameters);
    }
    rebuildParametricAccessories(packageRoot.get(), g_scene.station);
    CadNode* robotNode = findOPW6RobotNode(packageRoot.get());
    CadNode* gantryNode = findGantryMechanismNode(packageRoot.get());
    std::string accessoryManifest;
    const bool accessoryPackage =
        readCadPackageEntry(packagePath, "accessory.json", &accessoryManifest, nullptr);
    CadNode* accessoryNode = accessoryPackage && !robotNode && !gantryNode
        ? packageRoot.get()
        : nullptr;
    const bool toolPackage = request.assetKind == "tool";
    if (toolPackage && (!accessoryNode || !packageRoot->asRobotTool())) return false;
    if (robotNode) {
        if (!validateRobotPackage(packageRoot.get(), &errorMessage)) return false;
    } else if (gantryNode) {
        GantryPoseController validation;
        if (!validation.bindToGantry(gantryNode, &errorMessage)) return false;
        for (CadNode* chainNode : collectDragChainMechanismNodes(packageRoot.get())) {
            DragChainPoseController chainValidation;
            if (!chainValidation.bindToDragChain(chainNode, &errorMessage)) return false;
        }
    } else if (!accessoryNode) {
        return false;
    }

    auto frame = std::make_shared<CadNode>();
    frame->type = CadNodeType::Transform;
    auto frameData = std::make_shared<TransformNodeData>();
    frameData->isRobotBaseFrame = robotNode != nullptr;
    frame->data = frameData;
    const std::string packageAssetName = robotNode
        ? (robotNode->name.empty() ? std::string("Robot") : robotNode->name)
        : (gantryNode
               ? (gantryNode->name.empty() ? std::string("Linear rail") : gantryNode->name)
               : (packageRoot->name.empty() ? std::string("Accessory") : packageRoot->name));
    const std::string assetName = request.displayName.empty()
        ? packageAssetName : request.displayName;
    frame->name = assetName + (robotNode ? " base" : " placement");
    frame->loc = packageRoot->loc;
    frame->parent = g_scene.root.get();
    packageRoot->loc = CadTransform();
    packageRoot->parent = frame.get();
    frame->children.push_back(packageRoot);
    g_scene.root->children.push_back(frame);

    g_libraryPlacement.packagePath = packagePath;
    g_libraryPlacement.variantKey = variantKey;
    g_libraryPlacement.displayName = assetName;
    g_libraryPlacement.frame = frame;
    g_libraryPlacement.packageRoot = packageRoot;
    g_libraryPlacement.robotNode = robotNode;
    g_libraryPlacement.gantryNode = gantryNode;
    g_libraryPlacement.accessoryNode = accessoryNode;
    std::vector<mountingsnap::Interface> sources;
    // packageRoot is the placement origin after its transform is hoisted to the preview frame.
    mountingsnap::collectPlacementSources(packageRoot.get(), &sources);
    if (sources.empty()) {
        cancelLibraryRobotPlacement();
        return false;
    }
    g_libraryPlacement.session.arm(std::move(sources), toolPackage);

    g_scene.viewer->setPlacementPreviewRoot(frame.get(), false);
    g_scene.viewer->markCacheDirty();
    return true;
}

bool beginExistingStationObjectPlacement(StationSelectionKind kind, int index) {
    if (g_libraryPlacement.active() || !g_scene.viewer || !g_scene.root || index < 0) {
        return false;
    }

    CadNode* placementNode = nullptr;
    std::string displayName;
    std::string packagePath;
    bool toolPackage = false;
    if (kind == StationSelectionKind::Accessory &&
        index < static_cast<int>(g_scene.station.accessories.size())) {
        const StationAccessoryInstance& entry = g_scene.station.accessories[index];
        placementNode = entry.node;
        displayName = entry.name;
        packagePath = entry.resolvedPackagePath;
        toolPackage = !entry.parentRobotId.empty();
    } else if (kind == StationSelectionKind::Mechanism &&
               index < static_cast<int>(g_scene.gantries.size())) {
        placementNode = g_scene.gantries[index].gantryNode();
        if (index < static_cast<int>(g_scene.station.mechanisms.size())) {
            const StationMechanismInstance& entry = g_scene.station.mechanisms[index];
            displayName = entry.name;
            packagePath = entry.resolvedPackagePath;
        }
    } else {
        // Robots retain their transform gimbal for now: moving one between fixture and carriage
        // mounting surfaces also changes its station parent, which is a separate edit from moving
        // an already-rooted rail or accessory around the floor.
        return false;
    }
    if (!placementNode) return false;

    std::shared_ptr<CadNode> placementRoot = sharedSceneNode(g_scene.root, placementNode);
    if (!placementRoot) return false;
    if (displayName.empty()) {
        displayName = placementNode->name.empty() ? std::string("Station object")
                                                  : placementNode->name;
    }

    g_libraryPlacement.movingExisting = true;
    g_libraryPlacement.existingKind = kind;
    g_libraryPlacement.existingIndex = index;
    g_libraryPlacement.packagePath = packagePath;
    g_libraryPlacement.displayName = displayName;
    g_libraryPlacement.frame = placementRoot;
    g_libraryPlacement.packageRoot = placementRoot;
    g_libraryPlacement.originalLocalPose = placementNode->loc;
    if (kind == StationSelectionKind::Accessory) {
        g_libraryPlacement.accessoryNode = placementNode;
    } else {
        g_libraryPlacement.gantryNode = placementNode;
    }

    std::vector<mountingsnap::Interface> sources;
    mountingsnap::collectPlacementSources(placementNode, &sources);
    if (sources.empty()) {
        cancelLibraryRobotPlacement();
        return false;
    }
    g_libraryPlacement.session.arm(std::move(sources), toolPackage);

    const CadTransform stands = parentWorldTransformOf(placementNode) * placementNode->loc;
    constexpr double kQuarterTurn = 0.5 * 3.14159265358979323846;
    const double yaw = std::atan2(stands.values[2], stands.values[0]);
    g_libraryPlacement.session.setQuarterTurn(
        static_cast<int>(std::llround(yaw / kQuarterTurn)));

    g_scene.viewer->setPlacementPreviewRoot(placementNode, false);
    g_scene.viewer->markCacheDirty();
    return true;
}

void rotateLibraryRobotPlacement(float wheelDelta) {
    if (!g_libraryPlacement.active()) return;
    g_libraryPlacement.session.rotated(wheelDelta);
}

void updateLibraryRobotPlacement(const PointI& cursor, int viewportWidth, int viewportHeight) {
    if (!g_libraryPlacement.active() || !g_scene.viewer) return;
    if (!g_libraryPlacement.session.armed()) return;

    // The floor's grid is put where the cursor meets the floor before the targets are read off it, so the
    // window travels with the placement instead of the cell being limited to a sheet around the origin.
    if (g_defaultFloorNode) {
        mountingsnap::recentreFloorGrid(
            g_defaultFloorNode,
            parentWorldTransformOf(g_defaultFloorNode) * g_defaultFloorNode->loc,
            ViewerCamera(*g_scene.viewer), cursor.x(), cursor.y());
    }

    StationTargets eligible;
    eligible.toolPackage = g_libraryPlacement.session.toolPackage();
    std::vector<mountingsnap::Interface> targets;
    mountingsnap::collectTargets(g_scene.root.get(), CadTransform(), /*insideRobot=*/false,
                                 g_libraryPlacement.frame.get(), eligible, &targets);

    const mountingsnap::Result& solved = g_libraryPlacement.session.moved(
        cursor.x(), cursor.y(), targets, ViewerCamera(*g_scene.viewer), viewportWidth,
        viewportHeight, g_defaultFloorNode, g_scene.mountingSnapScreenPercent);

    const CadTransform parentWorld = parentWorldTransformOf(g_libraryPlacement.frame.get());
    g_libraryPlacement.frame->loc = parentWorld.rigidInverse() * solved.worldPose;
    g_libraryPlacement.frame->needsGlobalLocUpdate = true;
    g_scene.viewer->setPlacementPreviewRoot(g_libraryPlacement.frame.get(), solved.snapped);
    g_scene.viewer->setPlacementMountingGuides(asVec3f(solved.targetGuidesMm),
                                              asVec3f(solved.sourceGuidesMm));
}

// Removes the placement ghost wrapper from the scene, keeping the package root for delivery.
void detachPlacementWrapperKeepingPackageRoot() {
    removePlacementFrameFromScene();
    g_libraryPlacement.frame->children.erase(
        std::remove_if(g_libraryPlacement.frame->children.begin(),
                       g_libraryPlacement.frame->children.end(),
                       [](const std::shared_ptr<CadNode>& child) {
                           return child.get() == g_libraryPlacement.packageRoot.get();
                       }),
        g_libraryPlacement.frame->children.end());
}

// Reparents `delivered` under `parent`, re-expressing the preview's world pose in local terms.
void deliverPlacementNodeTo(const std::shared_ptr<CadNode>& delivered, CadNode* parent) {
    delivered->parent = parent;
    parent->children.push_back(delivered);
    const CadTransform parentWorld = parentWorldTransformOf(parent) * parent->loc;
    delivered->loc = parentWorld.rigidInverse() * g_libraryPlacement.worldPose();
    delivered->needsGlobalLocUpdate = true;
}

template <typename Instances>
std::string derivePlacementInstanceId(const Instances& existing, const std::string& name) {
    StationInstanceIds ids;
    for (const auto& instance : existing) ids.reserve(instance.id);
    return ids.derive(name, static_cast<int>(existing.size()));
}

// Common tail of every delivery branch. refreshPose is false only when moving an existing
// object, which invalidates each arm's readouts instead of recomputing them immediately.
bool finishLibraryPlacement(const std::string& statusText, bool refreshPose) {
    if (g_scene.viewer) {
        g_scene.viewer->setPlacementPreviewRoot(nullptr, false);
        g_scene.viewer->setPlacementMountingGuides({}, {});
        g_scene.viewer->markCacheDirty();
    }
    g_libraryPlacement = LibraryRobotPlacement();
    if (!statusText.empty() && !g_scene.robots.empty()) activeRobot().statusText = statusText;
    if (refreshPose) {
        refreshPoseDerivedReadouts();
    } else {
        for (const std::unique_ptr<RobotInstance>& robot : g_scene.robots) {
            if (robot) robot->readoutJointsValid = false;
        }
    }
    refreshProgramPathPreview();
    refreshSimPathPreview();
    return true;
}

bool commitLibraryRobotPlacement() {
    if (!g_libraryPlacement.active() || !g_libraryPlacement.snapped() ||
        (!g_libraryPlacement.robotNode && !g_libraryPlacement.gantryNode &&
         !g_libraryPlacement.accessoryNode)) {
        cancelLibraryRobotPlacement();
        return false;
    }

    CadNode* const committedTargetNode =
        const_cast<CadNode*>(g_libraryPlacement.session.last().targetNode);
    const mountingsnap::Interface* committedSourceInterface =
        g_libraryPlacement.session.activeInterface();

    if (g_libraryPlacement.movingExisting) {
        const StationSelectionKind kind = g_libraryPlacement.existingKind;
        const int index = g_libraryPlacement.existingIndex;
        CadNode* const node = g_libraryPlacement.frame.get();
        if (kind == StationSelectionKind::Accessory &&
            index >= 0 && index < static_cast<int>(g_scene.station.accessories.size())) {
            StationAccessoryInstance& accessory = g_scene.station.accessories[index];
            accessory.placement = node->loc;
            // Moving an object explicitly breaks its previous physical seams. Rebuild only the
            // complete end interface used for this drop; cancelling the drag never reaches here
            // and therefore preserves all links.
            removeStationParameterLinksForAccessory(g_scene.station, accessory.id);
            if (committedSourceInterface) {
                createStationParameterLinksForSnap(
                    g_scene.station, accessory, *committedSourceInterface,
                    committedTargetNode);
            }
        } else if (kind == StationSelectionKind::Mechanism &&
                   index >= 0 && index < static_cast<int>(g_scene.station.mechanisms.size())) {
            g_scene.station.mechanisms[index].placement = node->loc;
        } else {
            cancelLibraryRobotPlacement();
            return false;
        }

        selectStationObject(kind, index);
        return finishLibraryPlacement(
            "Moved " + g_libraryPlacement.displayName + " using mounting-hole snap.",
            /*refreshPose=*/false);
    }

    std::string errorMessage;
    if (g_libraryPlacement.accessoryNode) {
        if (g_libraryPlacement.session.toolPackage()) {
            int robotIndex = -1;
            for (int index = 0; index < static_cast<int>(g_scene.robots.size()); ++index) {
                CadNode* robotNode = g_scene.robots[static_cast<size_t>(index)]
                    ? g_scene.robots[static_cast<size_t>(index)]->poseController.robotNode()
                    : nullptr;
                if (robotNode && committedTargetNode &&
                    libraryNodeIsDescendantOf(committedTargetNode, robotNode)) {
                    robotIndex = index;
                    break;
                }
            }
            RobotToolData* toolData = g_libraryPlacement.accessoryNode->asRobotTool();
            if (robotIndex < 0 || !toolData) {
                cancelLibraryRobotPlacement();
                return false;
            }
            CadNode* robotNode = g_scene.robots[static_cast<size_t>(robotIndex)]
                ->poseController.robotNode();
            OPW6RobotData* robotData = robotNode ? robotNode->asOPW6Robot() : nullptr;
            if (!robotData || robotIndex >= static_cast<int>(g_scene.station.robots.size())) {
                cancelLibraryRobotPlacement();
                return false;
            }

            detachPlacementWrapperKeepingPackageRoot();
            deliverPlacementNodeTo(g_libraryPlacement.packageRoot, robotNode);

            StationAccessoryInstance entry;
            entry.name = g_libraryPlacement.displayName;
            entry.id = derivePlacementInstanceId(g_scene.station.accessories, entry.name);
            entry.packageRef =
                std::filesystem::path(g_libraryPlacement.packagePath).filename().string();
            entry.resolvedPackagePath = g_libraryPlacement.packagePath;
            entry.placement = g_libraryPlacement.packageRoot->loc;
            entry.parentRobotId = g_scene.station.robots[static_cast<size_t>(robotIndex)].id;
            entry.activeTcpIndex = toolData->activeTcpIndex;
            entry.node = g_libraryPlacement.packageRoot.get();
            g_scene.station.accessories.push_back(entry);

            RobotInstance& robot = *g_scene.robots[static_cast<size_t>(robotIndex)];
            if (!robot.poseController.registerAttachedTool(
                    g_libraryPlacement.packageRoot.get(), &errorMessage)) {
                robot.statusText = "Tool attached, but robot runtime refresh failed: " + errorMessage;
            } else {
                robot.statusText = "Attached " + entry.name + " to the robot flange.";
            }
            selectStationObject(StationSelectionKind::Accessory,
                                static_cast<int>(g_scene.station.accessories.size()) - 1);
            // The attach message on the target robot is the status here, not the generic one.
            return finishLibraryPlacement(std::string(), /*refreshPose=*/true);
        }
        detachPlacementWrapperKeepingPackageRoot();
        deliverPlacementNodeTo(g_libraryPlacement.packageRoot, g_scene.root.get());

        StationAccessoryInstance entry;
        entry.name = g_libraryPlacement.displayName;
        entry.id = derivePlacementInstanceId(g_scene.station.accessories, entry.name);
        entry.packageRef =
            std::filesystem::path(g_libraryPlacement.packagePath).filename().string();
        entry.resolvedPackagePath = g_libraryPlacement.packagePath;
        entry.placement = g_libraryPlacement.packageRoot->loc;
        entry.node = g_libraryPlacement.packageRoot.get();
        if (const TransformNodeData* transform = entry.node->asTransform()) {
            if (transform->hasParametricAccessory()) {
                entry.parameters = transform->accessoryParametersJson();
            }
        }
        g_scene.station.accessories.push_back(entry);
        if (committedSourceInterface) {
            createStationParameterLinksForSnap(
                g_scene.station, g_scene.station.accessories.back(),
                *committedSourceInterface, committedTargetNode);
        }
        selectStationObject(StationSelectionKind::Accessory,
                            static_cast<int>(g_scene.station.accessories.size()) - 1);
        return finishLibraryPlacement(
            "Placed " + g_libraryPlacement.displayName + " from the Library.",
            /*refreshPose=*/true);
    }

    if (g_libraryPlacement.gantryNode) {
        GantryPoseController gantry;
        if (!gantry.bindToGantry(g_libraryPlacement.gantryNode, &errorMessage)) {
            cancelLibraryRobotPlacement();
            return false;
        }
        std::vector<DragChainPoseController> chains;
        std::vector<DragChainPhysics> physics;
        for (CadNode* chainNode : collectDragChainMechanismNodes(
                 g_libraryPlacement.packageRoot.get())) {
            chains.emplace_back();
            if (!chains.back().bindToDragChain(chainNode, &errorMessage)) {
                cancelLibraryRobotPlacement();
                return false;
            }
            physics.emplace_back();
            if (!physics.back().bind(chainNode, &errorMessage)) {
                cancelLibraryRobotPlacement();
                return false;
            }
        }

        // The wrapper exists only so robot and mechanism ghosts share one placement path. A station
        // serializes a mechanism's package root directly, so remove the wrapper at delivery and put
        // the preview's world transform on the package root itself.
        detachPlacementWrapperKeepingPackageRoot();
        deliverPlacementNodeTo(g_libraryPlacement.packageRoot, g_scene.root.get());

        StationMechanismInstance entry;
        entry.name = g_libraryPlacement.displayName;
        entry.id = derivePlacementInstanceId(g_scene.station.mechanisms, entry.name);
        entry.packageRef =
            std::filesystem::path(g_libraryPlacement.packagePath).filename().string();
        entry.resolvedPackagePath = g_libraryPlacement.packagePath;
        entry.placement = g_libraryPlacement.packageRoot->loc;
        entry.positionMm = gantry.positionMm();
        g_scene.station.mechanisms.push_back(entry);

        g_scene.gantries.push_back(std::move(gantry));
        g_scene.dragChains.insert(g_scene.dragChains.end(),
                                 std::make_move_iterator(chains.begin()),
                                 std::make_move_iterator(chains.end()));
        g_scene.dragChainPhysics.insert(g_scene.dragChainPhysics.end(),
                                        std::make_move_iterator(physics.begin()),
                                        std::make_move_iterator(physics.end()));
        selectStationObject(StationSelectionKind::Mechanism,
                            static_cast<int>(g_scene.gantries.size()) - 1);
        return finishLibraryPlacement(
            "Placed " + g_libraryPlacement.displayName + " from the Library.",
            /*refreshPose=*/true);
    }

    auto robot = std::make_unique<RobotInstance>();
    if (!robot->poseController.bindToRobot(g_libraryPlacement.robotNode, &errorMessage) ||
        !robot->collisionModel.bindToRobot(g_libraryPlacement.robotNode, &errorMessage)) {
        cancelLibraryRobotPlacement();
        return false;
    }

    CadNode* destinationParent = g_scene.root.get();
    std::string parentMechanismId;
    for (size_t i = 0; i < g_scene.gantries.size(); ++i) {
        CadNode* movingFrame = g_scene.gantries[i].movingFrame();
        if (!movingFrame ||
            !libraryNodeIsDescendantOf(committedTargetNode, movingFrame)) {
            continue;
        }
        destinationParent = movingFrame;
        if (i < g_scene.station.mechanisms.size()) {
            parentMechanismId = g_scene.station.mechanisms[i].id;
        }
        break;
    }

    removePlacementFrameFromScene();
    deliverPlacementNodeTo(g_libraryPlacement.frame, destinationParent);

    robot->poseController.resetHome();
    const OPW6RobotData* robotData = robot->poseController.robotData();
    robot->packageMotionSettings = motionSettingsFromRobotData(robotData);
    robot->packageJointLimitsValid = robotData != nullptr;
    if (robotData) {
        robot->packageJointVelocityMaxRadS = robotData->jointVelocityMaxRadS;
        robot->packageJointAccelMaxRadS2 = robotData->jointAccelerationMaxRadS2;
        robot->packageJointJerkMaxRadS3 = robotData->jointJerkMaxRadS3;
    }
    robot->motionDynamicLimitScale.fill(1.0);
    applyMotionSettingsToEditors(*robot, robot->packageMotionSettings, "Package settings loaded.");
    loadPackageProgramsInto(*robot, g_libraryPlacement.packagePath);
    robot->hardware.setRobotModelCommand(
        robotModelCommandForPackage(robot->poseController, currentRobotModelFor(*robot)));

    StationRobotInstance entry;
    entry.name = g_libraryPlacement.displayName;
    entry.id = derivePlacementInstanceId(g_scene.station.robots, entry.name);
    const std::string packageId =
        std::filesystem::path(g_libraryPlacement.packagePath).stem().string();
    entry.packageRef = resolveBuiltinRobotPath(packageId).empty()
        ? std::filesystem::path(g_libraryPlacement.packagePath).filename().string()
        : builtinRobotPackageRef(packageId);
    entry.resolvedPackagePath = g_libraryPlacement.packagePath;
    entry.placement = g_libraryPlacement.frame->loc;
    entry.parentMechanismId = parentMechanismId;
    entry.configRef = "instances/" + entry.id + "/config.json";
    g_scene.station.robots.push_back(entry);

    g_scene.robots.push_back(std::move(robot));
    g_scene.view.robot = static_cast<int>(g_scene.robots.size()) - 1;
    selectStationObject(StationSelectionKind::Robot, g_scene.view.robot);
    rebindLiveRunDriverArms();
    return finishLibraryPlacement(
        "Placed " + g_libraryPlacement.displayName + " from the Library.",
        /*refreshPose=*/true);
}

