#include "CliInspectCommands.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <vector>

#include "AccessoryBuilders.h"
#include "AccessoryGeometry.h"
#include "CadNodePackage.h"
#include "DragChainPhysics.h"
#include "JsonCompat.h"
#include "OrientationFormat.h"
#include "ProgramTextIo.h"
#include "RobotMotionCore.h"
#include "RobotRuntime.h"
#include "StationPackage.h"
#include "StationParameterLinks.h"
#include "StringUtil.h"

int validatePackageCommand(const std::vector<std::string>& args) {
    std::string errorMessage;
    std::shared_ptr<CadNode> loadedNode = loadCadNodePackage(args[2], &errorMessage);
    if (!loadedNode) {
        std::cerr << "Package validation failed: " << errorMessage << std::endl;
        return 1;
    }
    if (findOPW6RobotNode(loadedNode.get())) {
        if (!validateRobotPackage(loadedNode.get(), &errorMessage)) {
            std::cerr << "Package validation failed: " << errorMessage << std::endl;
            return 1;
        }
        RobotPoseController poseController;
        if (!poseController.bind(loadedNode.get(), &errorMessage)) {
            std::cerr << "Package pose bind failed: " << errorMessage << std::endl;
            return 1;
        }
        RobotCollisionModel collisionModel;
        if (!collisionModel.bind(loadedNode.get(), &errorMessage)) {
            std::cerr << "Package collision bind failed: " << errorMessage << std::endl;
            return 1;
        }
    } else {
        std::string accessoryManifest;
        if (!readCadPackageEntry(args[2], "accessory.json", &accessoryManifest, nullptr) ||
            findGantryMechanismNode(loadedNode.get())) {
            std::cerr << "Package validation failed: package is neither a robot nor a static "
                         "accessory." << std::endl;
            return 1;
        }
        size_t placementCenters = 0;
        size_t parameterInterfaces = 0;
        bool parameterInterfacesValid = true;
        std::function<void(const CadNode*)> countPlacementCenters = [&](const CadNode* node) {
            if (!node) return;
            if (node->mountingHoles.placementSource) {
                placementCenters += node->mountingHoles.pointsMm.size();
                for (const MountingHoleGridData& grid : node->mountingHoles.grids) {
                    placementCenters += static_cast<size_t>(grid.uCount) * grid.vCount;
                }
            }
            if (!node->mountingHoles.parameterBindings.empty()) {
                ++parameterInterfaces;
                if (node->mountingHoles.interfaceId.empty()) {
                    parameterInterfacesValid = false;
                }
                for (const auto& binding : node->mountingHoles.parameterBindings) {
                    if (binding.first.empty() || binding.second.empty()) {
                        parameterInterfacesValid = false;
                    }
                }
            }
            for (const auto& child : node->children) countPlacementCenters(child.get());
        };
        countPlacementCenters(loadedNode.get());
        if (placementCenters == 0) {
            std::cerr << "Package validation failed: accessory has no placement-source holes."
                      << std::endl;
            return 1;
        }
        if (!parameterInterfacesValid) {
            std::cerr << "Package validation failed: parameter-bound mounting interface has "
                         "no stable interface id or has an empty binding." << std::endl;
            return 1;
        }
        std::cout << "accessory=" << loadedNode->name
                  << " placement_holes=" << placementCenters
                  << " parameter_interfaces=" << parameterInterfaces << std::endl;
    }
    std::cout << "Package validation succeeded: " << args[2] << std::endl;
    return 0;
}

// The station equivalent, and the only way to check a cell without a window. Every arm is bound
// and validated, not just the first, because a station is exactly the case where the first one
// being sound says nothing about the rest.
int validateStationCommand(const std::vector<std::string>& args) {
    std::string errorMessage;
    StationDocument station;
    std::shared_ptr<CadNode> stationRoot = loadStationPackage(args[2], &station, &errorMessage);
    if (!stationRoot) {
        std::cerr << "Station load failed: " << errorMessage << std::endl;
        return 1;
    }
    const int framesInserted = ensureRobotBaseFrames(stationRoot.get());
    const std::vector<CadNode*> robotNodes = collectRobotNodes(stationRoot.get());
    const std::vector<CadNode*> gantryNodes = collectGantryMechanismNodes(stationRoot.get());
    size_t mountingHoleCenters = 0;
    size_t mountingHoleGrids = 0;
    std::function<bool(const CadNode*)> countMountingHoles = [&](const CadNode* node) {
        if (!node) return true;
        if (node->mountingHoles.pointsMm.size() >
            std::numeric_limits<size_t>::max() - mountingHoleCenters) return false;
        mountingHoleCenters += node->mountingHoles.pointsMm.size();
        for (const MountingHoleGridData& grid : node->mountingHoles.grids) {
            if (grid.vCount != 0 && grid.uCount >
                std::numeric_limits<size_t>::max() / grid.vCount) return false;
            const size_t gridCenters = static_cast<size_t>(grid.uCount) * grid.vCount;
            if (gridCenters > std::numeric_limits<size_t>::max() - mountingHoleCenters) {
                return false;
            }
            mountingHoleCenters += gridCenters;
            ++mountingHoleGrids;
        }
        for (const auto& child : node->children) {
            if (!countMountingHoles(child.get())) return false;
        }
        return true;
    };
    if (!countMountingHoles(stationRoot.get())) {
        std::cerr << "Station mounting-hole layout count overflows size_t." << std::endl;
        return 1;
    }
    std::cout << "station=" << station.name << " robots=" << robotNodes.size()
               << " mechanisms=" << gantryNodes.size()
               << " accessories=" << station.accessories.size()
               << " parameter_links=" << station.parameterLinks.size()
               << " base_frames=" << framesInserted
               << " mounting_holes=" << mountingHoleCenters
               << " mounting_hole_grids=" << mountingHoleGrids << std::endl;
    if (gantryNodes.size() != station.mechanisms.size()) {
        std::cerr << "Station listed " << station.mechanisms.size()
                  << " mechanisms but the tree holds " << gantryNodes.size() << "." << std::endl;
        return 1;
    }
    for (size_t i = 0; i < station.accessories.size(); ++i) {
        if (!station.accessories[i].node ||
            findOPW6RobotNode(station.accessories[i].node) ||
            findGantryMechanismNode(station.accessories[i].node)) {
            std::cerr << "Station accessory " << i
                      << " is missing or contains a non-static asset." << std::endl;
            return 1;
        }
    }
    std::vector<std::string> linkedEndpoints;
    const auto findInterface = [&](const CadNode* root, const std::string& interfaceId) {
        const CadNode* found = static_cast<const CadNode*>(nullptr);
        std::function<void(const CadNode*)> visit = [&](const CadNode* node) {
            if (!node || found) return;
            if (node->mountingHoles.interfaceId == interfaceId) {
                found = node;
                return;
            }
            for (const auto& child : node->children) visit(child.get());
        };
        visit(root);
        return found;
    };
    for (size_t i = 0; i < station.parameterLinks.size(); ++i) {
        const StationParameterLink& link = station.parameterLinks[i];
        const StationAccessoryInstance* a = stationAccessoryById(station,
                                                                  link.a.accessoryId);
        const StationAccessoryInstance* b = stationAccessoryById(station,
                                                                  link.b.accessoryId);
        const double* aValue = a ? stationAccessoryParameter(*a, link.a.parameter) : nullptr;
        const double* bValue = b ? stationAccessoryParameter(*b, link.b.parameter) : nullptr;
        if (!aValue || !bValue || !std::isfinite(link.scale) ||
            std::abs(link.scale) < 1.0e-12 || !std::isfinite(link.offset)) {
            std::cerr << "Station parameter link " << i
                      << " has an unresolved endpoint or invalid equation." << std::endl;
            return 1;
        }
        const CadNode* aInterface = findInterface(a->node, link.a.port);
        const CadNode* bInterface = findInterface(b->node, link.b.port);
        const auto bindingMatches = [&](const CadNode* interfaceNode,
                                        const std::string& parameterName) {
            if (!interfaceNode) return false;
            const auto binding =
                interfaceNode->mountingHoles.parameterBindings.find(link.channel);
            return binding != interfaceNode->mountingHoles.parameterBindings.end() &&
                   binding->second == parameterName;
        };
        if (!bindingMatches(aInterface, link.a.parameter) ||
            !bindingMatches(bInterface, link.b.parameter)) {
            std::cerr << "Station parameter link " << i
                      << " does not match its package interface metadata." << std::endl;
            return 1;
        }
        for (const StationParameterEndpoint* endpoint : {&link.a, &link.b}) {
            const std::string key = stationParameterEndpointKey(*endpoint);
            if (std::find(linkedEndpoints.begin(), linkedEndpoints.end(), key) !=
                linkedEndpoints.end()) {
                std::cerr << "Station parameter endpoint " << endpoint->accessoryId << "."
                          << endpoint->parameter << " is linked more than once." << std::endl;
                return 1;
            }
            linkedEndpoints.push_back(key);
        }
        const double expectedB = *aValue * link.scale + link.offset;
        if (std::abs(*bValue - expectedB) > 1.0e-6) {
            std::cerr << "Station parameter link " << i << " is open by "
                      << (*bValue - expectedB) << " mm." << std::endl;
            return 1;
        }

        const double originalA = *aValue;
        const double originalB = *bValue;
        const double delta = originalA < 4999.0 ? 1.0 : -1.0;
        std::vector<size_t> affected;
        if (!applyStationLinkedParameter(station, link.a.accessoryId, link.a.parameter,
                                         originalA + delta, &affected, &errorMessage,
                                         /*rebuildModels=*/false)) {
            std::cerr << "Station parameter link " << i
                      << " forward propagation failed: " << errorMessage << std::endl;
            return 1;
        }
        a = stationAccessoryById(station, link.a.accessoryId);
        b = stationAccessoryById(station, link.b.accessoryId);
        aValue = a ? stationAccessoryParameter(*a, link.a.parameter) : nullptr;
        bValue = b ? stationAccessoryParameter(*b, link.b.parameter) : nullptr;
        if (!aValue || !bValue ||
            std::abs(*bValue - (*aValue * link.scale + link.offset)) > 1.0e-6) {
            std::cerr << "Station parameter link " << i
                      << " did not propagate forward." << std::endl;
            return 1;
        }
        if (!applyStationLinkedParameter(station, link.b.accessoryId, link.b.parameter,
                                         originalB, &affected, &errorMessage,
                                         /*rebuildModels=*/false)) {
            std::cerr << "Station parameter link " << i
                      << " reverse propagation failed: " << errorMessage << std::endl;
            return 1;
        }
        a = stationAccessoryById(station, link.a.accessoryId);
        b = stationAccessoryById(station, link.b.accessoryId);
        aValue = a ? stationAccessoryParameter(*a, link.a.parameter) : nullptr;
        bValue = b ? stationAccessoryParameter(*b, link.b.parameter) : nullptr;
        if (!aValue || !bValue || std::abs(*aValue - originalA) > 1.0e-6 ||
            std::abs(*bValue - originalB) > 1.0e-6) {
            std::cerr << "Station parameter link " << i
                      << " did not propagate in reverse." << std::endl;
            return 1;
        }

        StationDocument snapProbe = station;
        snapProbe.parameterLinks.clear();
        StationAccessoryInstance* probeSource =
            stationAccessoryById(snapProbe, link.a.accessoryId);
        mountingsnap::Interface probeInterface;
        probeInterface.node = const_cast<CadNode*>(aInterface);
        probeInterface.interfaceId = aInterface->mountingHoles.interfaceId;
        probeInterface.parameterBindings = aInterface->mountingHoles.parameterBindings;
        const int generated = probeSource
            ? createStationParameterLinksForSnap(
                  snapProbe, *probeSource, probeInterface,
                  const_cast<CadNode*>(bInterface))
            : 0;
        if (generated != 1 || snapProbe.parameterLinks.size() != 1 ||
            snapProbe.parameterLinks.front().channel != link.channel ||
            !stationParameterEndpointEquals(snapProbe.parameterLinks.front().a, link.a) ||
            !stationParameterEndpointEquals(snapProbe.parameterLinks.front().b, link.b) ||
            snapProbe.parameterLinks.front().a.port != link.a.port ||
            snapProbe.parameterLinks.front().b.port != link.b.port) {
            std::cerr << "Station parameter link " << i
                      << " cannot be recreated from its snapped interface metadata."
                      << std::endl;
            return 1;
        }
    }
    for (size_t i = 0; i < gantryNodes.size(); ++i) {
        GantryPoseController controller;
        if (!controller.bindToGantry(gantryNodes[i], &errorMessage)) {
            std::cerr << "Station mechanism " << i << " validation failed: "
                      << errorMessage << std::endl;
            return 1;
        }
        GantryMechanismData* data = controller.gantryData();
        CadNode* movingFrame = controller.movingFrame();
        const double original = controller.positionMm();
        const std::array<double, 3> before{{movingFrame->loc.values[3],
                                            movingFrame->loc.values[7],
                                            movingFrame->loc.values[11]}};
        controller.setPositionMm(data->upperLimitMm);
        const double dx = movingFrame->loc.values[3] - before[0];
        const double dy = movingFrame->loc.values[7] - before[1];
        const double dz = movingFrame->loc.values[11] - before[2];
        const double moved = std::sqrt(dx * dx + dy * dy + dz * dz);
        const double expected = std::abs(data->upperLimitMm - original);
        controller.setPositionMm(original);
        if (std::abs(moved - expected) > 1.0e-6) {
            std::cerr << "Station mechanism " << i << " moved " << moved
                      << " mm; expected " << expected << " mm." << std::endl;
            return 1;
        }
        std::cout << "mechanism " << i << " id=" << station.mechanisms[i].id
                  << " name=" << gantryNodes[i]->name
                  << " position=" << original
                  << " travel=[" << data->lowerLimitMm << ", " << data->upperLimitMm << "]"
                  << std::endl;
    }
    const std::vector<CadNode*> chainNodes = collectDragChainMechanismNodes(stationRoot.get());
    for (size_t i = 0; i < chainNodes.size(); ++i) {
        DragChainPoseController pose;
        if (!pose.bindToDragChain(chainNodes[i], &errorMessage)) {
            std::cerr << "Station drag chain " << i << " validation failed: "
                      << errorMessage << std::endl;
            return 1;
        }
        DragChainMechanismData* data = pose.chainData();
        if (data->linkFrames.size() < 2 || !data->prototypeGeometry ||
            !data->prototypeGeometry->asMeshGeometry()) {
            std::cerr << "Station drag chain " << i
                      << " must have member instances and one mesh prototype." << std::endl;
            return 1;
        }
        CadNode* parentGantry = nullptr;
        for (CadNode* ancestor = chainNodes[i]->parent; ancestor; ancestor = ancestor->parent) {
            if (ancestor->type == CadNodeType::GantryMechanism) {
                parentGantry = ancestor;
                break;
            }
        }
        GantryPoseController reachController;
        if (!parentGantry || !reachController.bindToGantry(parentGantry, &errorMessage)) {
            std::cerr << "Station drag chain " << i << " has no usable parent gantry: "
                      << errorMessage << std::endl;
            return 1;
        }
        const double savedPosition = reachController.positionMm();
        const double lowerReach = reachController.gantryData()->lowerLimitMm;
        const double upperReach = reachController.gantryData()->upperLimitMm;
        const double reachSpan = upperReach - lowerReach;
        const std::array<double, 5> reachTestPositions{{
            lowerReach,
            lowerReach + 0.25 * reachSpan,
            lowerReach + 0.50 * reachSpan,
            lowerReach + 0.75 * reachSpan,
            upperReach}};
        bool physxActiveAtLimits = true;
        double maximumObservedJointRotation = 0.0;
        double maximumObservedVisibleBend = 0.0;
#ifdef ROBOTSIM_WITH_PHYSX
        const auto visibleBendForChain = [&](size_t* jointIndex) {
            double maximumVisibleBend = 0.0;
            if (jointIndex) *jointIndex = 0;
            for (size_t link = 1; link < data->linkFrames.size(); ++link) {
                const CadTransform& previous = data->linkFrames[link - 1]->loc;
                const CadTransform& current = data->linkFrames[link]->loc;
                const double previousLength = std::sqrt(
                    previous.values[0] * previous.values[0] +
                    previous.values[4] * previous.values[4] +
                    previous.values[8] * previous.values[8]);
                const double currentLength = std::sqrt(
                    current.values[0] * current.values[0] +
                    current.values[4] * current.values[4] +
                    current.values[8] * current.values[8]);
                if (!(previousLength > 0.0) || !(currentLength > 0.0)) continue;
                double axisDot =
                    (previous.values[0] * current.values[0] +
                     previous.values[4] * current.values[4] +
                     previous.values[8] * current.values[8]) /
                    (previousLength * currentLength);
                // A reversed terminal member's local +X points back into the chain. Compare
                // the preceding contour direction with -X so a folded-back end is correctly
                // reported near 180 degrees instead of being mistaken for a small acute bend.
                if (data->reverseFixedEndMember && link + 1 == data->linkFrames.size()) {
                    axisDot = -axisDot;
                }
                axisDot = std::max(-1.0, std::min(1.0, axisDot));
                const double visibleBend = std::acos(axisDot) * 180.0 /
                                           3.14159265358979323846;
                if (visibleBend > maximumVisibleBend) {
                    maximumVisibleBend = visibleBend;
                    if (jointIndex) *jointIndex = link;
                }
            }
            return maximumVisibleBend;
        };
#endif
        for (double reachTestPosition : reachTestPositions) {
            reachController.setPositionMm(reachTestPosition);
            pose.update();
#ifdef ROBOTSIM_WITH_PHYSX
            const CadTransform movingTarget = data->linkFrames.front()->loc;
#endif
            DragChainPhysics physics;
            if (!physics.bind(chainNodes[i], &errorMessage)) {
                std::cerr << "Station drag chain " << i << " PhysX bind failed: "
                          << errorMessage << std::endl;
                return 1;
            }
#ifdef ROBOTSIM_WITH_PHYSX
            if (physics.baseCollisionHullCount() !=
                reachController.gantryData()->baseCollisionHulls.size()) {
                std::cerr << "Station drag chain " << i << " loaded "
                          << physics.baseCollisionHullCount() << " of "
                          << reachController.gantryData()->baseCollisionHulls.size()
                          << " serialized base collision hulls." << std::endl;
                return 1;
            }
#endif
            for (int step = 0; step < 600; ++step) {
                pose.update(); // Restores the carriage target before PhysX writes member poses.
                physics.step(1.0 / 120.0);
            }
#ifdef ROBOTSIM_WITH_PHYSX
            if (!physics.isActive()) {
                std::cerr << "Station drag chain " << i
                          << " is not physically reachable at " << reachTestPosition
                          << " mm." << std::endl;
                return 1;
            }
            physxActiveAtLimits = physxActiveAtLimits && physics.isSimulated();
            const auto distanceBetween = [](const CadTransform& a, const CadTransform& b) {
                const double dx = a.values[3] - b.values[3];
                const double dy = a.values[7] - b.values[7];
                const double dz = a.values[11] - b.values[11];
                return std::sqrt(dx * dx + dy * dy + dz * dz);
            };
            for (size_t link = 1; link < data->linkFrames.size(); ++link) {
                CadTransform previousHinge = data->linkFrames[link - 1]->loc;
                previousHinge.values[3] += previousHinge.values[0] * data->pitchMm;
                previousHinge.values[7] += previousHinge.values[4] * data->pitchMm;
                previousHinge.values[11] += previousHinge.values[8] * data->pitchMm;
                CadTransform currentHinge = data->linkFrames[link]->loc;
                if (data->reverseFixedEndMember && link + 1 == data->linkFrames.size()) {
                    currentHinge.values[3] += currentHinge.values[0] * data->pitchMm;
                    currentHinge.values[7] += currentHinge.values[4] * data->pitchMm;
                    currentHinge.values[11] += currentHinge.values[8] * data->pitchMm;
                }
                if (link + 1 == data->linkFrames.size()) {
                    currentHinge.values[3] -= data->fixedEndMemberOffsetMm.x;
                    currentHinge.values[7] -= data->fixedEndMemberOffsetMm.y;
                    currentHinge.values[11] -= data->fixedEndMemberOffsetMm.z;
                }
                const double hingeError = distanceBetween(previousHinge, currentHinge);
                if (!std::isfinite(hingeError) || hingeError > 3.0) {
                    std::cerr << "Station drag chain " << i << " link hinge " << link
                              << " is separated by " << hingeError
                              << " mm after PhysX settling at "
                              << reachTestPosition << " mm." << std::endl;
                    return 1;
                }
            }
            size_t maximumJointIndex = 0;
            const double maximumJointRotation =
                physics.maxAbsJointRotationDeg(&maximumJointIndex);
            maximumObservedJointRotation = std::max(maximumObservedJointRotation,
                                                    maximumJointRotation);
            if (!std::isfinite(maximumJointRotation) ||
                maximumJointRotation > data->maxJointRotationDeg + 0.1) {
                std::cerr << "Station drag chain " << i << " PhysX joint "
                          << maximumJointIndex << " rotates " << maximumJointRotation
                          << " degrees; configured maximum is "
                          << data->maxJointRotationDeg << " degrees after settling at "
                          << reachTestPosition << " mm." << std::endl;
                return 1;
            }
            size_t maximumVisibleBendIndex = 0;
            const double maximumVisibleBend =
                visibleBendForChain(&maximumVisibleBendIndex);
            if (!std::isfinite(maximumVisibleBend) ||
                maximumVisibleBend > data->maxJointRotationDeg + 0.1) {
                std::cerr << "Station drag chain " << i << " visible link joint "
                          << maximumVisibleBendIndex << " bends " << maximumVisibleBend
                          << " degrees; configured maximum is "
                          << data->maxJointRotationDeg << " degrees after settling at "
                          << reachTestPosition << " mm." << std::endl;
                return 1;
            }
            maximumObservedVisibleBend = std::max(maximumObservedVisibleBend,
                                                  maximumVisibleBend);
            const double movingError = distanceBetween(data->linkFrames.front()->loc,
                                                       movingTarget);
            const CadTransform& last = data->linkFrames.back()->loc;
            const double fixedOffset = data->reverseFixedEndMember ? 0.0 : data->pitchMm;
            const double fixedX = last.values[3] + last.values[0] * fixedOffset -
                                  data->fixedEndMemberOffsetMm.x;
            const double fixedY = last.values[7] + last.values[4] * fixedOffset -
                                  data->fixedEndMemberOffsetMm.y;
            const double fixedZ = last.values[11] + last.values[8] * fixedOffset -
                                  data->fixedEndMemberOffsetMm.z;
            const double fixedDx = fixedX - data->fixedAnchorMm.x;
            const double fixedDy = fixedY - data->fixedAnchorMm.y;
            const double fixedDz = fixedZ - data->fixedAnchorMm.z;
            const double fixedError = std::sqrt(
                fixedDx * fixedDx + fixedDy * fixedDy + fixedDz * fixedDz);
            if (movingError > 3.0 || fixedError > 3.0) {
                std::cerr << "Station drag chain " << i << " endpoint error is moving="
                          << movingError << " mm fixed=" << fixedError
                          << " mm after PhysX settling at " << reachTestPosition << " mm."
                          << std::endl;
                return 1;
            }
#else
            physxActiveAtLimits = false;
#endif
        }
#ifdef ROBOTSIM_WITH_PHYSX
        // Exercise one persistent PhysX chain while the carriage moves. Rebinding at isolated
        // positions cannot catch a terminal joint jumping to the folded branch during a slider
        // edit, which is the failure the pivot overlay makes visible.
        reachController.setPositionMm(lowerReach);
        pose.update();
        DragChainPhysics movingPhysics;
        if (!movingPhysics.bind(chainNodes[i], &errorMessage)) {
            std::cerr << "Station drag chain " << i
                      << " moving PhysX bind failed: " << errorMessage << std::endl;
            return 1;
        }
        for (int step = 0; step < 600; ++step) {
            pose.update();
            movingPhysics.step(1.0 / 120.0);
        }
        // One step of either sweep: track the maxima and enforce the configured bend limit on
        // both the visible links and the PhysX joints. `where` finishes the error sentence.
        const auto checkChainBendAt = [&](const char* phase,
                                          const std::string& where) -> bool {
            size_t visibleJointIndex = 0;
            const double visibleBend = visibleBendForChain(&visibleJointIndex);
            size_t physxJointIndex = 0;
            const double physxRotation =
                movingPhysics.maxAbsJointRotationDeg(&physxJointIndex);
            maximumObservedVisibleBend = std::max(maximumObservedVisibleBend, visibleBend);
            maximumObservedJointRotation = std::max(maximumObservedJointRotation,
                                                     physxRotation);
            if (!std::isfinite(visibleBend) ||
                visibleBend > data->maxJointRotationDeg + 0.1) {
                std::cerr << "Station drag chain " << i << " " << phase
                          << " visible link joint " << visibleJointIndex << " bends "
                          << visibleBend << " degrees; configured maximum is "
                          << data->maxJointRotationDeg << " degrees " << where << "."
                          << std::endl;
                return false;
            }
            if (!std::isfinite(physxRotation) ||
                physxRotation > data->maxJointRotationDeg + 0.1) {
                std::cerr << "Station drag chain " << i << " " << phase << " PhysX joint "
                          << physxJointIndex << " rotates " << physxRotation
                          << " degrees; configured maximum is "
                          << data->maxJointRotationDeg << " degrees " << where << "."
                          << std::endl;
                return false;
            }
            return true;
        };
        constexpr int kTravelSteps = 120;
        for (int direction = 0; direction < 2; ++direction) {
            for (int step = 1; step <= kTravelSteps; ++step) {
                const double fraction = static_cast<double>(step) / kTravelSteps;
                const double position = direction == 0
                    ? lowerReach + fraction * reachSpan
                    : upperReach - fraction * reachSpan;
                reachController.setPositionMm(position);
                pose.update();
                movingPhysics.step(1.0 / 120.0);
                std::ostringstream where;
                where << "at " << position << " mm (direction " << direction << ", step "
                      << step << ")";
                if (!checkChainBendAt("moving", where.str())) return 1;
            }
        }
        for (int jump = 0; jump < 2; ++jump) {
            const double position = jump == 0 ? upperReach : lowerReach;
            reachController.setPositionMm(position);
            for (int step = 1; step <= 120; ++step) {
                pose.update();
                movingPhysics.step(1.0 / 120.0);
                std::ostringstream where;
                where << "after jump " << jump << " to " << position << " mm at step "
                      << step;
                if (!checkChainBendAt("jump", where.str())) return 1;
            }
        }
#endif
        reachController.setPositionMm(savedPosition);
        pose.update();
        std::cout << "drag_chain " << i << " instances=" << data->linkFrames.size()
                  << " mesh_prototypes=1 pitch_mm=" << data->pitchMm
                  << " base_collision_hulls="
                  << reachController.gantryData()->baseCollisionHulls.size()
                  << " max_joint_rotation_deg=" << data->maxJointRotationDeg
                  << " observed_joint_deg=" << maximumObservedJointRotation
                  << " observed_visible_bend_deg=" << maximumObservedVisibleBend
                  << " reach_test_mm=[" << reachTestPositions.front() << ", "
                  << reachTestPositions.back() << "] samples=" << reachTestPositions.size()
                  << " moving_samples=240 jump_samples=240"
                  << " physx=" << (physxActiveAtLimits ? "active" : "fallback") << std::endl;
    }
    if (robotNodes.size() != station.robots.size()) {
        std::cerr << "Station listed " << station.robots.size() << " robots but the tree holds "
                  << robotNodes.size() << "." << std::endl;
        return 1;
    }
    for (size_t i = 0; i < robotNodes.size(); ++i) {
        const StationRobotInstance& instance = station.robots[i];
        if (!validateRobotPackage(robotNodes[i], &errorMessage)) {
            std::cerr << "Station robot " << i << " validation failed: " << errorMessage << std::endl;
            return 1;
        }
        RobotPoseController poseController;
        RobotCollisionModel collisionModel;
        if (!poseController.bindToRobot(robotNodes[i], &errorMessage) ||
            !collisionModel.bindToRobot(robotNodes[i], &errorMessage)) {
            std::cerr << "Station robot " << i << " bind failed: " << errorMessage << std::endl;
            return 1;
        }
        // Tool selection is allowed while the robot is posed. Exercise every TCP repeatedly
        // and prove that it changes only the kinematic tool bind, never any visual/link bind:
        // a bindToRobot() on an already-posed tree would compound every joint transform on
        // every selection.
        OPW6RobotData* switchProbeRobot = poseController.robotData();
        const std::array<double, 6> switchProbeOriginalJoints = poseController.joints();
        std::array<double, 6> switchProbeJoints = switchProbeOriginalJoints;
        if (switchProbeRobot) {
            for (size_t joint = 0; joint < switchProbeJoints.size(); ++joint) {
                const double candidate = switchProbeJoints[joint] + 0.12;
                if (switchProbeRobot->qMin[joint] < switchProbeRobot->qMax[joint] &&
                    candidate <= switchProbeRobot->qMax[joint] - 1.0e-4) {
                    switchProbeJoints[joint] = candidate;
                } else if (switchProbeRobot->qMin[joint] < switchProbeRobot->qMax[joint] &&
                           switchProbeJoints[joint] - 0.12 >=
                               switchProbeRobot->qMin[joint] + 1.0e-4) {
                    switchProbeJoints[joint] -= 0.12;
                }
            }
            poseController.setJoints(switchProbeJoints);
        }
        const std::vector<CadNode*> switchProbeLinks = collectRobotLinks(robotNodes[i]);
        const std::vector<CadNode*> switchProbeTools = collectRobotTools(robotNodes[i]);
        std::vector<CadNode*> switchProbeNodes = switchProbeLinks;
        switchProbeNodes.insert(switchProbeNodes.end(), switchProbeTools.begin(),
                                switchProbeTools.end());
        std::vector<CadTransform> switchProbeLocs;
        for (CadNode* node : switchProbeNodes) {
            switchProbeLocs.push_back(node ? node->loc : CadTransform());
        }
        CadNode* originalTool = switchProbeRobot ? switchProbeRobot->activeTool : nullptr;
        std::vector<int> originalTcpIndices;
        for (CadNode* toolNode : switchProbeTools) {
            RobotToolData* tool = toolNode ? toolNode->asRobotTool() : nullptr;
            originalTcpIndices.push_back(tool ? tool->activeTcpIndex : 0);
        }
        for (int repetition = 0; repetition < 3; ++repetition) {
            for (CadNode* toolNode : switchProbeTools) {
                RobotToolData* tool = toolNode ? toolNode->asRobotTool() : nullptr;
                if (!switchProbeRobot || !tool) continue;
                switchProbeRobot->activeTool = toolNode;
                const int tcpCount = std::max(1, static_cast<int>(tool->tcps.size()));
                for (int tcpIndex = 0; tcpIndex < tcpCount; ++tcpIndex) {
                    tool->activeTcpIndex = tcpIndex;
                    if (!poseController.refreshActiveToolBind(&errorMessage)) {
                        std::cerr << "Station robot " << i
                                  << " active-tool refresh failed: " << errorMessage
                                  << std::endl;
                        return 1;
                    }
                    const CadTransform probePose = poseController.toolPose();
                    for (double value : probePose.values) {
                        if (!std::isfinite(value)) {
                            std::cerr << "Station robot " << i
                                      << " active-tool refresh produced a non-finite pose."
                                      << std::endl;
                            return 1;
                        }
                    }
                    for (size_t nodeIndex = 0; nodeIndex < switchProbeNodes.size(); ++nodeIndex) {
                        if (!switchProbeNodes[nodeIndex]) continue;
                        for (size_t valueIndex = 0; valueIndex < 12; ++valueIndex) {
                            if (switchProbeNodes[nodeIndex]->loc.values[valueIndex] !=
                                switchProbeLocs[nodeIndex].values[valueIndex]) {
                                std::cerr << "Station robot " << i
                                          << " tool switching changed robot geometry at node '"
                                          << switchProbeNodes[nodeIndex]->name << "'."
                                          << std::endl;
                                return 1;
                            }
                        }
                    }
                }
            }
        }
        for (size_t toolIndex = 0; toolIndex < switchProbeTools.size(); ++toolIndex) {
            RobotToolData* tool = switchProbeTools[toolIndex]
                ? switchProbeTools[toolIndex]->asRobotTool() : nullptr;
            if (tool) tool->activeTcpIndex = originalTcpIndices[toolIndex];
        }
        if (switchProbeRobot) switchProbeRobot->activeTool = originalTool;
        if (!poseController.refreshActiveToolBind(&errorMessage)) {
            std::cerr << "Station robot " << i
                      << " could not restore its active tool: " << errorMessage << std::endl;
            return 1;
        }
        poseController.setJoints(switchProbeOriginalJoints);
        // Off the base frame, which is the node that holds a placement once an arm hangs under
        // one. Reading the arm's own loc here would print zeroes for every arm in the cell.
        const CadNode* frameNode = robotBaseFrameNode(robotNodes[i]);
        const CadTransform& placement = frameNode->loc;
        // The TCP as well as the placement, because the two are computed by different machinery
        // and only printing them together says whether they agree. The placement is a node
        // transform read straight back out; the TCP comes through RobotKinematicSnapshot, which
        // is what path previews and the tool readout are built from. Two arms at +/-400 whose
        // TCPs came back identical is what said the snapshot was not carrying the placement.
        CadTransform toolPose = poseController.worldToolPose();
        const OPW6RobotData* robotData = poseController.robotData();
        const CadNode* activeToolNode = robotData ? robotData->activeTool : nullptr;
        const RobotToolData* activeToolData = activeToolNode
            ? activeToolNode->asRobotTool() : nullptr;
        const int activeTcpIndex = activeToolData && !activeToolData->tcps.empty()
            ? std::max(0, std::min(activeToolData->activeTcpIndex,
                                  static_cast<int>(activeToolData->tcps.size()) - 1))
            : 0;
        const std::string activeToolName = activeToolNode && !activeToolNode->name.empty()
            ? activeToolNode->name : "none";
        const std::string activeTcpName = activeToolData && !activeToolData->tcps.empty()
            ? activeToolData->tcps[static_cast<size_t>(activeTcpIndex)].name : "none";
        CadNode* parentGantry = nullptr;
        for (CadNode* ancestor = robotNodes[i]->parent; ancestor; ancestor = ancestor->parent) {
            if (ancestor->type == CadNodeType::GantryMechanism) {
                parentGantry = ancestor;
                break;
            }
        }
        if (parentGantry) {
            GantryPoseController gantry;
            if (!gantry.bindToGantry(parentGantry, &errorMessage)) {
                std::cerr << "Station robot " << i << " parent gantry failed to bind: "
                          << errorMessage << std::endl;
                return 1;
            }
            const double original = gantry.positionMm();
            const double endpoint = gantry.gantryData()->upperLimitMm;
            gantry.setPositionMm(endpoint);
            const CadTransform movedToolPose = poseController.worldToolPose();
            const double dx = movedToolPose.values[3] - toolPose.values[3];
            const double dy = movedToolPose.values[7] - toolPose.values[7];
            const double dz = movedToolPose.values[11] - toolPose.values[11];
            const double moved = std::sqrt(dx * dx + dy * dy + dz * dz);
            gantry.setPositionMm(original);
            if (std::abs(moved - std::abs(endpoint - original)) > 1.0e-6) {
                std::cerr << "Station robot " << i
                          << " did not follow its gantry carriage through the full travel."
                          << std::endl;
                return 1;
            }
            toolPose = poseController.worldToolPose();
        }
        // The programs this arm will open with, through the same loader the scene uses. Reported
        // per arm because that is the level it works at: a cell where one arm found its package's
        // programs and another did not is the failure worth seeing, and it is invisible from the
        // count of robots.
        const size_t programCount =
            probeStationInstancePrograms(instance.resolvedPackagePath, instance.programs);
        std::cout << "robot " << i << " id=" << instance.id
                  << " name=" << robotNodes[i]->name
                  << " package=" << instance.packageRef
                  << " at=(" << placement.values[3] << ", " << placement.values[7] << ", "
                  << placement.values[11] << ")"
                  << " tcp=(" << toolPose.values[3] << ", " << toolPose.values[7] << ", "
                  << toolPose.values[11] << ")"
                  << " active_tool=\"" << activeToolName << "\""
                  << " active_tcp=\"" << activeTcpName << "\""
                  << " tool_tcp_options="
                  << (activeToolData ? activeToolData->tcps.size() : 0)
                  << " config=" << (instance.config.empty() ? "none" : instance.configRef)
                  << " programs=" << programCount
                  << std::endl;
    }
    std::cout << "Station validation succeeded: " << args[2] << std::endl;
    return 0;
}

int orientationCommand(const std::vector<std::string>& args) {
    const std::string wanted = args[2];
    int sourceIndex = -1;
    for (int i = 0; i < orientation::formatCount(); ++i) {
        const std::string label = orientation::formats()[i].label;
        if (label.find(wanted) != std::string::npos) { sourceIndex = i; break; }
    }
    // Also accept a plain index, so the listing this prints on failure is directly usable.
    if (sourceIndex < 0 && !wanted.empty() && wanted.find_first_not_of("0123456789") == std::string::npos) {
        const int parsed = std::atoi(wanted.c_str());
        if (parsed >= 0 && parsed < orientation::formatCount()) sourceIndex = parsed;
    }
    if (sourceIndex < 0) {
        std::cerr << "No orientation format matches '" << wanted << "'. Available:" << std::endl;
        for (int i = 0; i < orientation::formatCount(); ++i) {
            std::cerr << "  " << i << "  " << orientation::formats()[i].label << std::endl;
        }
        return 2;
    }

    const orientation::Format& source = orientation::formats()[sourceIndex];
    std::array<double, 4> values{{0.0, 0.0, 0.0, 0.0}};
    const int given = std::min(static_cast<int>(args.size()) - 3, 4);
    for (int i = 0; i < given; ++i) values[static_cast<size_t>(i)] = std::atof(args[static_cast<size_t>(3 + i)].c_str());
    if (given < source.fieldCount) {
        std::cerr << "'" << source.label << "' needs " << source.fieldCount << " values, got "
                  << given << "." << std::endl;
        return 2;
    }

    const CadTransform rotation = orientation::rotationFromValues(source, values);
    std::printf("from  %s\n", source.label);
    std::printf("rotation matrix (row major, rotation part only)\n");
    for (int row = 0; row < 3; ++row) {
        std::printf("  %10.6f %10.6f %10.6f\n",
                    rotation.values[static_cast<size_t>(row * 4 + 0)],
                    rotation.values[static_cast<size_t>(row * 4 + 1)],
                    rotation.values[static_cast<size_t>(row * 4 + 2)]);
    }
    for (int i = 0; i < orientation::formatCount(); ++i) {
        const orientation::Format& format = orientation::formats()[i];
        const std::array<double, 4> shown = orientation::valuesFromRotation(format, rotation);
        std::printf("  %-38s", format.label);
        for (int f = 0; f < format.fieldCount; ++f) {
            std::printf(" %11.5f", shown[static_cast<size_t>(f)]);
        }
        std::printf("\n");
    }
    return 0;
}

int listRobotsCommand(const std::vector<std::string>& args) {
    (void)args;
    const std::vector<BuiltinRobot> robots = listBuiltinRobots(true);
    std::cout << "catalogue=" << builtinRobotCatalogueRoot() << " robots=" << robots.size()
              << std::endl;
    for (const BuiltinRobot& robot : robots) {
        std::cout << "  " << builtinRobotPackageRef(robot.id)
                  << "  name=" << (robot.name.empty() ? std::string("-") : robot.name)
                  << "  resolves=" << (robot.resolves ? "yes" : "no") << std::endl;
    }
    // Not an error. An empty catalogue is a real answer, and the caller can see the root above.
    return 0;
}

// Loads a station and writes it out again. Useful in its own right - it is how a loose station
// becomes a self-contained archive - and it is the only way to exercise the save path without a
// window, which matters because that path decides whether a cell's calibration survives.
int resaveStationCommand(const std::vector<std::string>& args) {
    const bool useBuiltins =
        std::find(args.begin() + 4, args.end(), std::string("--builtins")) != args.end();
    std::string errorMessage;
    StationDocument station;
    if (!loadStationPackage(args[2], &station, &errorMessage)) {
        std::cerr << "Station load failed: " << errorMessage << std::endl;
        return 1;
    }
    if (useBuiltins) {
        std::vector<std::string> promoted;
        promoteBuiltinPackageRefs(&station, &promoted);
        for (const std::string& line : promoted) std::cout << "builtin: " << line << std::endl;
    }
    if (!saveStationPackage(args[3], station, args[2], &errorMessage)) {
        std::cerr << "Station save failed: " << errorMessage << std::endl;
        return 1;
    }
    std::cout << "Station resaved: " << args[3] << " robots=" << station.robots.size()
              << " mechanisms=" << station.mechanisms.size()
              << " accessories=" << station.accessories.size()
              << " parameter_links=" << station.parameterLinks.size() << std::endl;
    return 0;
}

int dumpAxesCommand(const std::vector<std::string>& args) {
    std::string errorMessage;
    std::shared_ptr<CadNode> loadedNode = loadCadNodePackage(args[2], &errorMessage);
    if (!loadedNode) {
        std::cerr << "Package load failed: " << errorMessage << std::endl;
        return 1;
    }
    for (const std::string& line : describeRobotJointAxes(loadedNode.get())) {
        std::cout << line << std::endl;
    }
    return 0;
}

int toolPoseCommand(const std::vector<std::string>& args) {
    if (args.size() < 9) {
        std::cerr << "Usage: RobotSimulator --tool-pose <package> j1 j2 j3 j4 j5 j6" << std::endl;
        return 2;
    }
    std::string errorMessage;
    std::shared_ptr<CadNode> loadedNode = loadCadNodePackage(args[2], &errorMessage);
    if (!loadedNode || !validateRobotPackage(loadedNode.get(), &errorMessage)) {
        std::cerr << "Package validation failed: " << errorMessage << std::endl;
        return 1;
    }
    RobotPoseController poseController;
    if (!poseController.bind(loadedNode.get(), &errorMessage)) {
        std::cerr << "Package pose bind failed: " << errorMessage << std::endl;
        return 1;
    }
    std::array<double, 6> q{};
    for (size_t joint = 0; joint < q.size(); ++joint) {
        double degrees = 0.0;
        if (!strutil::parseDouble(args[3 + joint], &degrees) || !std::isfinite(degrees)) {
            std::cerr << "Invalid joint degree value: " << args[3 + joint] << std::endl;
            return 1;
        }
        q[joint] = degrees * kDegToRad;
    }
    poseController.setJoints(q);
    const CadTransform tcp = poseController.toolPose();
    // Seventeen significant digits round-trips a double exactly, so whatever a reader disagrees with this
    // by is a disagreement about the kinematics and never about the printing.
    std::cout << "tcp";
    for (int cell = 0; cell < 12; ++cell) {
        std::cout << ' ' << std::setprecision(17) << tcp.values[cell];
    }
    std::cout << std::endl;
    return 0;
}

// Prints the load_robot_model command a package produces, and its length. Exists because that
// command is the difference between Run program working and the firmware answering
// model_not_loaded, and because the firmware parses it out of a fixed 1536-byte buffer - both
// worth being able to check without a robot attached.
int dumpRobotModelCommand(const std::vector<std::string>& args) {
    std::string errorMessage;
    std::shared_ptr<CadNode> loadedNode = loadCadNodePackage(args[2], &errorMessage);
    if (!loadedNode || !validateRobotPackage(loadedNode.get(), &errorMessage)) {
        std::cerr << "Package validation failed: " << errorMessage << std::endl;
        return 1;
    }
    RobotPoseController poseController;
    if (!poseController.bind(loadedNode.get(), &errorMessage)) {
        std::cerr << "Package pose bind failed: " << errorMessage << std::endl;
        return 1;
    }
    const Json command = robotModelCommandForPackage(poseController, poseController.motionModel());
    if (command.empty()) {
        std::cerr << "Robot model is not valid for this package." << std::endl;
        return 1;
    }
    const std::string text = command.dump();
    std::cout << text << std::endl;
    // 1535, not 1536: the firmware's read loop keeps a byte for the terminator, at
    // "used < sizeof(message.text) - 1", and drops the whole line once that is reached.
    std::cout << "bytes=" << text.size() << " limit=1535" << std::endl;
    return text.size() <= 1535 ? 0 : 1;
}

int stationIkPositionCommand(const std::vector<std::string>& args) {
    std::string errorMessage;
    StationDocument document;
    std::shared_ptr<CadNode> loadedNode = loadStationPackage(args[2], &document, &errorMessage);
    const std::vector<CadNode*> robots = collectRobotNodes(loadedNode.get());
    const int robotIndex = strutil::parseIntOr(args[3], -1);
    const int tcpIndex = strutil::parseIntOr(args[4], -1);
    if (!loadedNode || robotIndex < 0 || robotIndex >= static_cast<int>(robots.size())) {
        std::cerr << "Station/robot load failed: " << errorMessage << std::endl;
        return 1;
    }
    double worldX = 0.0, worldY = 0.0, worldZ = 0.0;
    if (!strutil::parseDouble(args[5], &worldX) || !strutil::parseDouble(args[6], &worldY) ||
        !strutil::parseDouble(args[7], &worldZ)) return 1;
    RobotPoseController controller;
    if (!controller.bindToRobot(robots[static_cast<size_t>(robotIndex)], &errorMessage)) {
        std::cerr << errorMessage << std::endl;
        return 1;
    }
    OPW6RobotData* robotData = controller.robotData();
    RobotToolData* tool = robotData && robotData->activeTool
        ? robotData->activeTool->asRobotTool() : nullptr;
    if (tool && tcpIndex >= 0 && tcpIndex < static_cast<int>(tool->tcps.size())) {
        tool->activeTcpIndex = tcpIndex;
        if (!controller.refreshActiveToolBind(&errorMessage)) {
            std::cerr << errorMessage << std::endl;
            return 1;
        }
    }
    // Optional six-joint seed (degrees) preserves the orientation and OPW branch of a taught
    // pose while retouching only its TCP position.  With no seed this remains the historical
    // home-oriented query.
    if (args.size() >= 14) {
        std::array<double, 6> seededJoints{};
        for (size_t joint = 0; joint < seededJoints.size(); ++joint) {
            double degrees = 0.0;
            if (!strutil::parseDouble(args[8 + joint], &degrees)) {
                std::cerr << "Invalid IK seed joint " << (joint + 1) << std::endl;
                return 1;
            }
            seededJoints[joint] = degrees * kDegToRad;
        }
        controller.setJoints(seededJoints);
    }
    CadTransform robotWorld;
    std::vector<const CadNode*> ancestry;
    for (const CadNode* node = robots[static_cast<size_t>(robotIndex)]; node;
         node = node->parent) ancestry.push_back(node);
    for (auto it = ancestry.rbegin(); it != ancestry.rend(); ++it) robotWorld = robotWorld * (*it)->loc;
    CadTransform targetWorld = robotWorld * controller.toolPose();
    targetWorld.values[3] = worldX;
    targetWorld.values[7] = worldY;
    targetWorld.values[11] = worldZ;
    const CadTransform target = robotWorld.rigidInverse() * targetWorld;
    std::array<std::array<double, 6>, 12> solutions{};
    const int count = controller.toolPoseSolutions(target, solutions.data(),
                                                   static_cast<int>(solutions.size()),
                                                   &errorMessage);
    if (count <= 0) {
        std::cerr << "No IK solution: " << errorMessage << std::endl;
        return 1;
    }
    const std::array<double, 6> home = controller.joints();
    std::sort(solutions.begin(), solutions.begin() + count,
              [&](const auto& a, const auto& b) {
                  double da = 0.0, db = 0.0;
                  for (int j = 0; j < 6; ++j) {
                      da += std::abs(a[static_cast<size_t>(j)] - home[static_cast<size_t>(j)]);
                      db += std::abs(b[static_cast<size_t>(j)] - home[static_cast<size_t>(j)]);
                  }
                  return da < db;
              });
    for (int solution = 0; solution < count; ++solution) {
        std::cout << "solution " << solution;
        for (double value : solutions[static_cast<size_t>(solution)]) {
            std::cout << ' ' << value * kRadToDeg;
        }
        std::cout << std::endl;
    }
    return 0;
}

int ikSmokeCommand(const std::vector<std::string>& args) {
    std::string errorMessage;
    std::shared_ptr<CadNode> loadedNode = loadCadNodePackage(args[2], &errorMessage);
    if (!loadedNode || !validateRobotPackage(loadedNode.get(), &errorMessage)) {
        std::cerr << "Package validation failed: " << errorMessage << std::endl;
        return 1;
    }
    RobotPoseController poseController;
    if (!poseController.bind(loadedNode.get(), &errorMessage)) {
        std::cerr << "Package pose bind failed: " << errorMessage << std::endl;
        return 1;
    }
    const CadTransform homeTarget = poseController.toolPose();
    const struct {
        const char* name;
        int offset;
        double delta;
    } targets[] = {
        {"+X", 3, 50.0},
        {"-X", 3, -50.0},
        {"+Y", 7, 50.0},
        {"-Y", 7, -50.0},
        {"+Z", 11, 50.0},
        {"-Z", 11, -50.0},
    };
    for (const auto& testTarget : targets) {
        poseController.resetHome();
        CadTransform target = homeTarget;
        target.values[static_cast<size_t>(testTarget.offset)] += testTarget.delta;
        if (!poseController.setToolPose(target, &errorMessage)) {
            std::cerr << "IK smoke failed for " << testTarget.name << ": " << errorMessage << std::endl;
            return 1;
        }
        const CadTransform solved = poseController.toolPose();
        std::cout << "IK smoke " << testTarget.name << " TCP=("
                  << solved.values[3] << ", "
                  << solved.values[7] << ", "
                  << solved.values[11] << ") joints_deg=(";
        const std::array<double, 6>& solvedJoints = poseController.joints();
        for (int i = 0; i < 6; ++i) {
            if (i) std::cout << ", ";
            std::cout << solvedJoints[static_cast<size_t>(i)] * kRadToDeg;
        }
        std::cout << ")" << std::endl;
    }
    for (int axis = 0; axis < 3; ++axis) {
        poseController.resetHome();
        constexpr double kRotationSmokeRad = 10.0 * kDegToRad;
        const CadTransform target = withLocalRotation(homeTarget, axis, kRotationSmokeRad);
        if (!poseController.setToolPose(target, &errorMessage)) {
            std::cerr << "IK smoke failed for R" << static_cast<char>('X' + axis)
                      << ": " << errorMessage << std::endl;
            return 1;
        }
        std::cout << "IK smoke R" << static_cast<char>('X' + axis)
                  << " succeeded" << std::endl;
    }
    std::cout << "IK smoke succeeded." << std::endl;
    return 0;
}

int bakeHullsCommand(const std::vector<std::string>& args) {
    std::string errorMessage;
    std::shared_ptr<CadNode> loadedNode = loadCadNodePackage(args[2], &errorMessage);
    if (!loadedNode ||
        !bakeRobotCollisionHulls(loadedNode.get(), &errorMessage) ||
        !saveCadNodePackage(args[3], *loadedNode, args[2], &errorMessage)) {
        std::cerr << "Hull baking failed: " << errorMessage << std::endl;
        return 1;
    }
    std::cout << "Hull baking succeeded: " << args[3] << std::endl;
    return 0;
}

