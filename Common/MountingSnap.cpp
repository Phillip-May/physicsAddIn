#include "MountingSnap.h"

#include "StringUtil.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <utility>

namespace mountingsnap {
namespace {

CadVec3 subtract(const CadVec3& a, const CadVec3& b) {
    return CadVec3(a.x - b.x, a.y - b.y, a.z - b.z);
}

double screenDistance(double ax, double ay, double bx, double by) {
    const double dx = ax - bx;
    const double dy = ay - by;
    return std::sqrt(dx * dx + dy * dy);
}

CadTransform rotationOnly(const CadTransform& transform) {
    CadTransform rotation = transform;
    rotation.values[3] = 0.0;
    rotation.values[7] = 0.0;
    rotation.values[11] = 0.0;
    return rotation;
}

CadTransform yawQuarterTurn(int quarterTurns) {
    const double angle = static_cast<double>(quarterTurns) * 0.5 * 3.14159265358979323846;
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    CadTransform rotation;
    rotation.values = {{c, 0.0, s, 0.0,
                        0.0, 1.0, 0.0, 0.0,
                        -s, 0.0, c, 0.0}};
    return rotation;
}

CadTransform interfaceMatingRotation(bool mateOpposite) {
    CadTransform rotation;
    if (mateOpposite) {
        // Half-turn about interface-local X: keeps the vertical edge direction consistent while
        // making the two outward normals face each other across an end-to-end joint.
        rotation.values = {{1.0, 0.0, 0.0, 0.0,
                            0.0, -1.0, 0.0, 0.0,
                            0.0, 0.0, -1.0, 0.0}};
    }
    return rotation;
}

void appendPoints(const CadNode* node, const CadTransform& transform,
                  std::vector<CadVec3>* points) {
    constexpr size_t kHoleLimit = 250000;
    for (const CadVec3& point : node->mountingHoles.pointsMm) {
        if (points->size() >= kHoleLimit) return;
        points->push_back(transform * point);
    }
    for (const MountingHoleGridData& grid : node->mountingHoles.grids) {
        for (uint32_t u = 0; u < grid.uCount; ++u) {
            for (uint32_t v = 0; v < grid.vCount; ++v) {
                if (points->size() >= kHoleLimit) return;
                points->push_back(transform *
                                  CadVec3(grid.originMm.x + grid.uStepMm.x * u + grid.vStepMm.x * v,
                                          grid.originMm.y + grid.uStepMm.y * u + grid.vStepMm.y * v,
                                          grid.originMm.z + grid.uStepMm.z * u + grid.vStepMm.z * v));
            }
        }
    }
}

Interface interfaceAt(const CadNode* node, const CadTransform& transform) {
    Interface interface;
    interface.node = const_cast<CadNode*>(node);
    interface.transform = transform;
    interface.mateOpposite = node->mountingHoles.mateOpposite;
    interface.interfaceId = node->mountingHoles.interfaceId;
    interface.parameterBindings = node->mountingHoles.parameterBindings;
    appendPoints(node, transform, &interface.pointsMm);
    return interface;
}

CadVec3 centreOf(const std::vector<CadVec3>& points) {
    CadVec3 centre;
    for (const CadVec3& point : points) {
        centre = CadVec3(centre.x + point.x, centre.y + point.y, centre.z + point.z);
    }
    return points.empty() ? centre : scaled(centre, 1.0 / static_cast<double>(points.size()));
}

// How a source pattern is turned to meet a target one. The rotation only: which hole lands on which is
// what the anchor pair below decides.
CadTransform matingAlignment(const Interface& source, const Interface& target, int quarterTurn) {
    return rotationOnly(target.transform) * yawQuarterTurn(quarterTurn) *
           interfaceMatingRotation(source.mateOpposite) *
           rotationOnly(source.transform).rigidInverse();
}

// That rotation, slid so one named source point lands exactly on one named target point.
CadTransform anchoredAt(const CadTransform& rotation, const CadVec3& sourcePoint,
                        const CadVec3& targetPoint) {
    CadTransform pose = rotation;
    const CadVec3 rotated = rotation * sourcePoint;
    pose.values[3] = targetPoint.x - rotated.x;
    pose.values[7] = targetPoint.y - rotated.y;
    pose.values[11] = targetPoint.z - rotated.z;
    return pose;
}

int countMatchedHoles(const CadTransform& pose, const std::vector<CadVec3>& sourcePoints,
                      const std::vector<CadVec3>& targetPoints, double* totalErrorMm,
                      double* worstErrorMm) {
    constexpr double kInterfaceWorldToleranceMm = 0.5;
    std::vector<bool> targetUsed(targetPoints.size(), false);
    int matches = 0;
    if (totalErrorMm) *totalErrorMm = 0.0;
    if (worstErrorMm) *worstErrorMm = 0.0;
    for (const CadVec3& sourcePoint : sourcePoints) {
        const CadVec3 transformedSource = pose * sourcePoint;
        size_t closest = targetPoints.size();
        double closestDistance = kInterfaceWorldToleranceMm;
        for (size_t targetIndex = 0; targetIndex < targetPoints.size(); ++targetIndex) {
            if (targetUsed[targetIndex]) continue;
            const double distance = lengthOf(subtract(transformedSource, targetPoints[targetIndex]));
            if (distance <= closestDistance) {
                closestDistance = distance;
                closest = targetIndex;
            }
        }
        if (closest >= targetPoints.size()) continue;
        targetUsed[closest] = true;
        ++matches;
        if (totalErrorMm) *totalErrorMm += closestDistance;
        if (worstErrorMm) *worstErrorMm = std::max(*worstErrorMm, closestDistance);
    }
    return matches;
}

} // namespace

void collectInterfaces(const CadNode* node, const CadTransform& parentTransform,
                       bool placementSourcesOnly, std::vector<Interface>* interfaces) {
    if (!node || !interfaces) return;
    const CadTransform transform = parentTransform * node->loc;
    if ((!placementSourcesOnly || node->mountingHoles.placementSource) &&
        !node->mountingHoles.empty()) {
        Interface interface = interfaceAt(node, transform);
        if (!interface.pointsMm.empty()) interfaces->push_back(std::move(interface));
    }
    for (const std::shared_ptr<CadNode>& child : node->children) {
        collectInterfaces(child.get(), transform, placementSourcesOnly, interfaces);
    }
}

void collectLocalInterfaces(const CadNode* root, bool placementSourcesOnly,
                            std::vector<Interface>* interfaces) {
    if (!root || !interfaces) return;
    const CadTransform identity;
    if ((!placementSourcesOnly || root->mountingHoles.placementSource) &&
        !root->mountingHoles.empty()) {
        Interface interface = interfaceAt(root, identity);
        if (!interface.pointsMm.empty()) interfaces->push_back(std::move(interface));
    }
    for (const std::shared_ptr<CadNode>& child : root->children) {
        collectInterfaces(child.get(), identity, placementSourcesOnly, interfaces);
    }
}

namespace {

bool holdsRobot(const CadNode* node) {
    if (!node) return false;
    if (node->type == CadNodeType::OPW6Robot) return true;
    for (const std::shared_ptr<CadNode>& child : node->children) {
        if (holdsRobot(child.get())) return true;
    }
    return false;
}

} // namespace

void collectPlacementSources(const CadNode* root, std::vector<Interface>* interfaces) {
    if (!root || !interfaces) return;
    collectLocalInterfaces(root, /*placementSourcesOnly=*/true, interfaces);
    if (!interfaces->empty() || !holdsRobot(root)) return;
    collectLocalInterfaces(root, /*placementSourcesOnly=*/false, interfaces);
}

void collectTargets(CadNode* node, const CadTransform& parentTransform, bool insideRobot,
                    const CadNode* exclude, const TargetFilter& filter,
                    std::vector<Interface>* targets) {
    if (!node || !targets || !node->visible || node == exclude) return;
    const CadTransform worldTransform = parentTransform * node->loc;
    insideRobot = insideRobot || node->type == CadNodeType::OPW6Robot;
    if (filter.eligible(node, insideRobot) && !node->mountingHoles.empty()) {
        Interface target = interfaceAt(node, worldTransform);
        if (!target.pointsMm.empty()) targets->push_back(std::move(target));
    }
    for (const std::shared_ptr<CadNode>& child : node->children) {
        collectTargets(child.get(), worldTransform, insideRobot, exclude, filter, targets);
    }
}

int rotationQuarterStep(const Interface& interface) {
    if (interface.pointsMm.size() <= 1) return 1;
    const CadTransform interfaceInverse = interface.transform.rigidInverse();
    const CadVec3 first = interfaceInverse * interface.pointsMm.front();
    double minimum[3] = {first.x, first.y, first.z};
    double maximum[3] = {first.x, first.y, first.z};
    for (const CadVec3& point : interface.pointsMm) {
        const CadVec3 local = interfaceInverse * point;
        const double values[3] = {local.x, local.y, local.z};
        for (int axis = 0; axis < 3; ++axis) {
            minimum[axis] = std::min(minimum[axis], values[axis]);
            maximum[axis] = std::max(maximum[axis], values[axis]);
        }
    }
    std::array<double, 3> extents{{maximum[0] - minimum[0],
                                   maximum[1] - minimum[1],
                                   maximum[2] - minimum[2]}};
    std::sort(extents.begin(), extents.end(), std::greater<double>());
    if (extents[1] < 1.0e-3) return 2; // A line pattern only maps onto itself after 180 degrees.
    return extents[0] / extents[1] <= 1.05 ? 1 : 2;
}

Mate mate(const Interface& source, const Interface& target, int quarterTurn) {
    Mate best;
    best.requiredHoles = static_cast<int>(source.pointsMm.size());
    if (source.pointsMm.empty() || target.pointsMm.empty()) return best;
    const CadTransform alignedRotation = matingAlignment(source, target, quarterTurn);
    // Every anchor pair, because which hole lands on which is the only freedom the rotation leaves and
    // there is no cursor here to prefer one. The best is the most complete, then the tightest.
    for (const CadVec3& onto : target.pointsMm) {
        for (const CadVec3& anchor : source.pointsMm) {
            const CadTransform pose = anchoredAt(alignedRotation, anchor, onto);
            double totalError = 0.0;
            double worstError = 0.0;
            const int matches = countMatchedHoles(pose, source.pointsMm, target.pointsMm,
                                                  &totalError, &worstError);
            if (matches < best.matchedHoles ||
                (matches == best.matchedHoles && worstError >= best.worstErrorMm &&
                 best.matchedHoles > 0)) {
                continue;
            }
            best.worldPose = pose;
            best.matchedHoles = matches;
            best.worstErrorMm = worstError;
        }
    }
    best.complete = best.matchedHoles >= best.requiredHoles && best.requiredHoles > 0;
    return best;
}

MountingHoleGridData floorGridWindow(const CadVec3& centreMm, double halfExtentMm, double pitchMm) {
    MountingHoleGridData grid;
    const double pitch = pitchMm > 0.0 ? pitchMm : kFloorGridPitchMm;
    // At least one step either side, or a window with a single point in it would offer nothing to snap to
    // at all and the floor would silently stop working.
    const double steps = std::max(1.0, std::floor(std::max(pitch, halfExtentMm) / pitch));
    // Rounded to whole steps: this is the line that makes the window a view of one fixed lattice rather
    // than a patch that travels with the cursor.
    const double centreU = std::round(centreMm.x / pitch) * pitch;
    const double centreV = std::round(centreMm.z / pitch) * pitch;
    grid.originMm = CadVec3(centreU - steps * pitch, 0.0, centreV - steps * pitch);
    grid.uStepMm = CadVec3(pitch, 0.0, 0.0);
    grid.vStepMm = CadVec3(0.0, 0.0, pitch);
    grid.uCount = static_cast<uint32_t>(steps * 2.0 + 1.0);
    grid.vCount = grid.uCount;
    return grid;
}

bool recentreFloorGrid(CadNode* floor, const CadTransform& floorToWorld, const View& view,
                       double cursorX, double cursorY, double halfExtentMm, double pitchMm) {
    if (!floor || floor->mountingHoles.grids.empty()) return false;
    CadVec3 rayOrigin;
    CadVec3 rayDirection;
    if (!view.rayThrough(cursorX, cursorY, &rayOrigin, &rayDirection)) return false;
    const CadVec3 planePoint = floorToWorld * CadVec3(0.0, 0.0, 0.0);
    const CadVec3 planeNormal = normalized(rotate(floorToWorld, CadVec3(0.0, 1.0, 0.0)));
    const double denominator = dot(rayDirection, planeNormal);
    if (std::abs(denominator) < 1.0e-6) return false;
    const double distance = dot(subtract(planePoint, rayOrigin), planeNormal) / denominator;
    // Behind the eye, which is the floor seen from underneath.
    if (!(distance > 0.0)) return false;
    const CadVec3 hit(rayOrigin.x + rayDirection.x * distance,
                      rayOrigin.y + rayDirection.y * distance,
                      rayOrigin.z + rayDirection.z * distance);
    floor->mountingHoles.grids.front() =
        floorGridWindow(floorToWorld.rigidInverse() * hit, halfExtentMm, pitchMm);
    return true;
}

Result solve(const Request& request, const View& view) {
    Result result;
    if (!request.sourceInterfaces || request.sourceInterfaces->empty()) return result;
    const std::vector<Interface>& sourceInterfaces = *request.sourceInterfaces;
    static const std::vector<Interface> kNoTargets;
    const std::vector<Interface>& targets = request.targets ? *request.targets : kNoTargets;

    const double thresholdPixels = std::max(
        2.0, static_cast<double>(std::min(request.viewportWidthPx, request.viewportHeightPx)) *
                 request.snapScreenPercent / 100.0);

    int bestMatches = 0;
    int bestRequiredMatches = 1;
    int bestSourceInterface = 0;
    bool bestSatisfiesRequirement = false;
    double bestError = std::numeric_limits<double>::max();
    double bestCentreDistance = std::numeric_limits<double>::max();
    CadTransform bestPose;
    CadNode* bestNode = nullptr;
    bool bestDirectionalMate = false;
    bool incompleteDirectionalMateNearby = false;
    int incompleteDirectionalMatches = 0;
    int incompleteDirectionalRequired = 0;
    double incompleteDirectionalDistance = std::numeric_limits<double>::max();

    // Free dragging is driven by the object's ordinary base/feet interface. End interfaces are
    // still evaluated for snapping, but must not turn the conveyor upright merely because the
    // cursor passes close to another conveyor's vertical end pattern.
    int dragSourceInterface = 0;
    for (size_t index = 0; index < sourceInterfaces.size(); ++index) {
        if (!sourceInterfaces[index].mateOpposite) {
            dragSourceInterface = static_cast<int>(index);
            break;
        }
    }
    const Interface& dragInterface = sourceInterfaces[static_cast<size_t>(dragSourceInterface)];
    const CadVec3 dragSourceCentre = centreOf(dragInterface.pointsMm);

    // The closest compatible surface supplies the unsnapped preview plane. Compute this before
    // evaluating snap candidates so every interface is measured from the same current drag pose.
    const Interface* nearestSurface = nullptr;
    CadVec3 nearestSurfacePoint;
    double nearestSurfacePixels = std::numeric_limits<double>::max();
    for (const Interface& surface : targets) {
        if (surface.mateOpposite != dragInterface.mateOpposite) continue;
        for (const CadVec3& targetPoint : surface.pointsMm) {
            double targetX = 0.0;
            double targetY = 0.0;
            if (!view.project(targetPoint, &targetX, &targetY)) continue;
            const double cursorDistance =
                screenDistance(request.cursorX, request.cursorY, targetX, targetY);
            if (cursorDistance < nearestSurfacePixels) {
                nearestSurfacePixels = cursorDistance;
                nearestSurface = &surface;
                nearestSurfacePoint = targetPoint;
            }
        }
    }

    const CadTransform dragSourceRotation = rotationOnly(dragInterface.transform);
    CadTransform freePose = yawQuarterTurn(request.quarterTurn) * dragSourceRotation.rigidInverse();
    CadVec3 planePoint;
    CadVec3 planeNormal(0.0, 1.0, 0.0);
    if (nearestSurface) {
        freePose = rotationOnly(nearestSurface->transform) *
                   yawQuarterTurn(request.quarterTurn) *
                   interfaceMatingRotation(request.toolPackage && dragInterface.mateOpposite) *
                   dragSourceRotation.rigidInverse();
        planePoint = nearestSurfacePoint;
        planeNormal = normalized(rotate(nearestSurface->transform, CadVec3(0.0, 1.0, 0.0)));
    }
    CadVec3 rayOrigin;
    CadVec3 rayDirection;
    if (view.rayThrough(request.cursorX, request.cursorY, &rayOrigin, &rayDirection)) {
        const double denominator = dot(rayDirection, planeNormal);
        if (std::abs(denominator) > 1.0e-5) {
            const double distance = dot(subtract(planePoint, rayOrigin), planeNormal) / denominator;
            if (distance > 0.0) {
                const CadVec3 intersection = CadVec3(rayOrigin.x + rayDirection.x * distance,
                                                     rayOrigin.y + rayDirection.y * distance,
                                                     rayOrigin.z + rayDirection.z * distance);
                const CadVec3 rotatedCentre = freePose * dragSourceCentre;
                freePose.values[3] = intersection.x - rotatedCentre.x;
                freePose.values[7] = intersection.y - rotatedCentre.y;
                freePose.values[11] = intersection.z - rotatedCentre.z;
            }
        }
    }

    const auto requiredMatchesFor = [&](const CadNode* targetNode, size_t sourcePointCount) {
        if (targetNode == request.floor || sourcePointCount <= 1) return 1;
        return static_cast<int>(sourcePointCount);
    };

    for (const Interface& surface : targets) {
        std::vector<double> targetScreenX(surface.pointsMm.size(), 0.0);
        std::vector<double> targetScreenY(surface.pointsMm.size(), 0.0);
        std::vector<bool> targetProjected(surface.pointsMm.size(), false);
        for (size_t targetIndex = 0; targetIndex < surface.pointsMm.size(); ++targetIndex) {
            targetProjected[targetIndex] = view.project(surface.pointsMm[targetIndex],
                                                        &targetScreenX[targetIndex],
                                                        &targetScreenY[targetIndex]);
        }

        for (size_t sourceInterfaceIndex = 0; sourceInterfaceIndex < sourceInterfaces.size();
             ++sourceInterfaceIndex) {
            const Interface& sourceInterface = sourceInterfaces[sourceInterfaceIndex];
            if (request.retainSnapAfterRotation &&
                (surface.node != request.retainedTarget ||
                 static_cast<int>(sourceInterfaceIndex) != request.retainedSourceInterface)) {
                continue;
            }
            // Ordinary mounting patterns mate to ordinary patterns, while directional conveyor
            // ends only mate to another directional end. This prevents a leg or floor pattern
            // from winning on one coincident point at the seam.
            if (surface.mateOpposite != sourceInterface.mateOpposite) continue;
            if (surface.node == request.floor && sourceInterface.mateOpposite) continue;
            const std::vector<CadVec3>& sourcePoints = sourceInterface.pointsMm;
            if (sourcePoints.empty()) continue;
            const CadVec3 sourceCentre = centreOf(sourcePoints);
            const CadTransform alignedRotation =
                matingAlignment(sourceInterface, surface, request.quarterTurn);

            for (size_t anchorIndex = 0; anchorIndex < surface.pointsMm.size(); ++anchorIndex) {
                if (!targetProjected[anchorIndex]) continue;
                for (size_t sourceAnchor = 0; sourceAnchor < sourcePoints.size(); ++sourceAnchor) {
                    const CadTransform pose = anchoredAt(alignedRotation,
                                                         sourcePoints[sourceAnchor],
                                                         surface.pointsMm[anchorIndex]);

                    // The drag cursor represents the centre of the object being placed. An
                    // anchor-hole pair can be some distance from it (especially on a large
                    // pedestal), so deciding snap reach from that anchor makes the preview jump
                    // sideways. Test the resulting interface centre instead: the complete pattern
                    // may pull the ghost only by the percentage the host asked for.
                    double currentX = 0.0, currentY = 0.0, snappedX = 0.0, snappedY = 0.0;
                    if (!view.project(freePose * sourceCentre, &currentX, &currentY) ||
                        !view.project(pose * sourceCentre, &snappedX, &snappedY)) {
                        continue;
                    }
                    // Acquisition follows the interface being moved, not the mouse. A conveyor is
                    // normally grabbed around its middle, so its end can be correctly beside the
                    // other conveyor while the cursor remains half a conveyor length away.
                    const double centreDistance =
                        screenDistance(currentX, currentY, snappedX, snappedY);
                    // End connections are visually large objects with a small four-point pattern;
                    // give their interface centres twice the ordinary hole tolerance to engage.
                    // The pattern itself must still pass the complete-pattern test below.
                    const double acquisitionPixels = sourceInterface.mateOpposite
                        ? thresholdPixels * 2.0 : thresholdPixels;
                    const bool retainedMate = request.retainSnapAfterRotation &&
                        surface.node == request.retainedTarget &&
                        static_cast<int>(sourceInterfaceIndex) == request.retainedSourceInterface;
                    if (!retainedMate && centreDistance > acquisitionPixels) continue;

                    // The anchor pair is exact by construction. On the generic floor one match is
                    // the complete rule, so stop there instead of turning a 41x41 grid into an
                    // O(n^2) search every frame. Model-specific mating surfaces still evaluate the
                    // complete pattern below.
                    int matches = 1;
                    double totalError = 0.0;
                    if (surface.node != request.floor) {
                        matches = countMatchedHoles(pose, sourcePoints, surface.pointsMm,
                                                    &totalError, nullptr);
                    }
                    const double averageError = matches > 0
                        ? totalError / matches : std::numeric_limits<double>::max();
                    const int requiredMatches =
                        requiredMatchesFor(surface.node, sourcePoints.size());
                    const bool satisfiesRequirement = matches >= requiredMatches;
                    if (sourceInterface.mateOpposite && !satisfiesRequirement &&
                        (retainedMate || centreDistance <= acquisitionPixels) &&
                        (matches > incompleteDirectionalMatches ||
                         (matches == incompleteDirectionalMatches &&
                          centreDistance < incompleteDirectionalDistance))) {
                        incompleteDirectionalMateNearby = true;
                        incompleteDirectionalMatches = matches;
                        incompleteDirectionalRequired = requiredMatches;
                        incompleteDirectionalDistance = centreDistance;
                    }
                    if ((satisfiesRequirement && !bestSatisfiesRequirement) ||
                        (satisfiesRequirement == bestSatisfiesRequirement &&
                         matches > bestMatches) ||
                        (satisfiesRequirement == bestSatisfiesRequirement &&
                         matches == bestMatches && centreDistance < bestCentreDistance) ||
                        (satisfiesRequirement == bestSatisfiesRequirement &&
                         matches == bestMatches && centreDistance == bestCentreDistance &&
                         averageError < bestError)) {
                        bestMatches = matches;
                        bestRequiredMatches = requiredMatches;
                        bestSourceInterface = static_cast<int>(sourceInterfaceIndex);
                        bestSatisfiesRequirement = satisfiesRequirement;
                        bestError = averageError;
                        bestCentreDistance = centreDistance;
                        bestPose = pose;
                        bestNode = surface.node;
                        bestDirectionalMate = sourceInterface.mateOpposite;
                    }
                }
            }
        }
    }

    // A nearby directional interface owns the placement decision. Falling back to the generic
    // floor after rejecting a 2/4 or 3/4 end pattern made the ghost green and allowed exactly the
    // half-connected conveyor pose the complete-pattern rule was meant to prevent.
    if (incompleteDirectionalMateNearby && !bestDirectionalMate) {
        bestSatisfiesRequirement = false;
        bestMatches = 0;
        bestNode = nullptr;
    }

    result.sourceInterface = bestSatisfiesRequirement ? bestSourceInterface : dragSourceInterface;
    const size_t activeIndex = static_cast<size_t>(std::max(
        0, std::min(result.sourceInterface, static_cast<int>(sourceInterfaces.size()) - 1)));
    const std::vector<CadVec3>& sourcePoints = sourceInterfaces[activeIndex].pointsMm;
    const CadVec3 sourceCentre = centreOf(sourcePoints);
    result.requiredHoles = bestNode ? bestRequiredMatches
                                    : requiredMatchesFor(nullptr, sourcePoints.size());
    result.snapped = bestSatisfiesRequirement;
    result.matchedHoles = result.snapped ? bestMatches : 0;
    result.targetNode = result.snapped ? bestNode : nullptr;

    if (result.snapped) {
        result.worldPose = bestPose;
        result.status = strutil::format("Rotation %1 deg | Snapped: %2/%3 holes (%4% tolerance)")
                            .arg(request.quarterTurn * 90)
                            .arg(bestMatches)
                            .arg(sourcePoints.size())
                            .arg(request.snapScreenPercent, 0, 'f', 1)
                            .str();
    } else {
        result.worldPose = freePose;
        if (incompleteDirectionalMateNearby) {
            result.status =
                strutil::format("Rotation %1 deg | End incomplete: %2/%3 holes; align all points")
                    .arg(request.quarterTurn * 90)
                    .arg(incompleteDirectionalMatches)
                    .arg(incompleteDirectionalRequired)
                    .str();
        } else {
            result.status = strutil::format("Rotation %1 deg | Align all %2 mounting holes")
                                .arg(request.quarterTurn * 90)
                                .arg(result.requiredHoles)
                                .str();
        }
    }

    // Keep the placement guides local to the cursor and the ghost. A complete fixture table can
    // hold thousands of points; showing the nearby patch makes the snap grid legible and avoids
    // turning on a whole-scene mounting-hole overlay just to place one asset.
    result.sourceGuidesMm.reserve(sourcePoints.size());
    for (const CadVec3& sourcePoint : sourcePoints) {
        result.sourceGuidesMm.push_back(result.worldPose * sourcePoint);
    }
    const Interface* guideSurface = nearestSurface;
    if (bestNode) {
        const auto match = std::find_if(
            targets.begin(), targets.end(),
            [bestNode](const Interface& surface) { return surface.node == bestNode; });
        if (match != targets.end()) guideSurface = &*match;
    }
    double guideCentreX = 0.0;
    double guideCentreY = 0.0;
    if (guideSurface &&
        view.project(result.worldPose * sourceCentre, &guideCentreX, &guideCentreY)) {
        const double guideRadiusPixels = std::max(
            thresholdPixels * 4.0,
            static_cast<double>(std::min(request.viewportWidthPx, request.viewportHeightPx)) * 0.14);
        constexpr size_t kMaximumLocalGuides = 256;
        for (const CadVec3& targetPoint : guideSurface->pointsMm) {
            double targetX = 0.0;
            double targetY = 0.0;
            if (!view.project(targetPoint, &targetX, &targetY) ||
                screenDistance(guideCentreX, guideCentreY, targetX, targetY) > guideRadiusPixels) {
                continue;
            }
            result.targetGuidesMm.push_back(targetPoint);
            if (result.targetGuidesMm.size() >= kMaximumLocalGuides) break;
        }
    }
    return result;
}

} // namespace mountingsnap
