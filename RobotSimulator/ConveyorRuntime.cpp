#include "ConveyorRuntime.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "AccessoryBuilders.h"
#include "ConveyorCore.h"
#include "MasteringIo.h"
#include "SceneConveyorHost.h"
#include "StringUtil.h"
#include "UnitsMath.h"

using strutil::operator<<;

TransformNodeData* conveyorParameters(CadNode* node) {
    TransformNodeData* parameters = node ? node->asTransform() : nullptr;
    return parameters && parameters->accessoryGenerator == "roller_conveyor"
        ? parameters : nullptr;
}

const TransformNodeData* conveyorParameters(const CadNode* node) {
    const TransformNodeData* parameters = node ? node->asTransform() : nullptr;
    return parameters && parameters->accessoryGenerator == "roller_conveyor"
        ? parameters : nullptr;
}

ConveyorSimulationMode resolvedConveyorMode(const TransformNodeData& parameters) {
    ConveyorSimulationMode requested = g_scene.defaultConveyorMode;
    if (parameters.accessoryConveyorMode == "logical") {
        requested = ConveyorSimulationMode::Logical;
    }
    if (parameters.accessoryConveyorMode == "physx") {
        requested = ConveyorSimulationMode::PhysX;
    }
    return requested == ConveyorSimulationMode::PhysX && !ConveyorPhysics::backendAvailable()
        ? ConveyorSimulationMode::Logical : requested;
}

CadVec3 conveyorNormalized(const CadVec3& vector) {
    const double length = std::sqrt(vector.x * vector.x + vector.y * vector.y +
                                    vector.z * vector.z);
    return length > 1.0e-9 ? CadVec3(vector.x / length, vector.y / length, vector.z / length)
                           : CadVec3();
}

double conveyorPointDistance(const CadVec3& a, const CadVec3& b) {
    return std::sqrt((a.x - b.x) * (a.x - b.x) +
                     (a.y - b.y) * (a.y - b.y) +
                     (a.z - b.z) * (a.z - b.z));
}

void placeConveyorWorkpiece(ConveyorWorkpiece& workpiece) {
    TransformNodeData* parameters = conveyorParameters(workpiece.conveyor);
    if (!parameters || !workpiece.node) return;
    const ConveyorPathPose pose = conveyorPathPoseAt(*parameters, workpiece.progress);
    workpiece.node->loc = CadTransform();
    workpiece.node->loc.values[3] = pose.x;
    workpiece.node->loc.values[7] = pose.y + conveyorSurfaceOffsetMm(*parameters) +
                                    workpiece.heightMm * 0.5;
    workpiece.node->loc.values[11] = pose.z;
    workpiece.node->needsGlobalLocUpdate = true;
}

void detachConveyorWorkpiece(const ConveyorWorkpiece& workpiece) {
    if (!workpiece.node || !workpiece.node->parent) return;
    auto& children = workpiece.node->parent->children;
    children.erase(std::remove(children.begin(), children.end(), workpiece.node), children.end());
    workpiece.node->parent = nullptr;
}

void clearConveyorWorkpieces() {
    for (const ConveyorWorkpiece& workpiece : g_scene.conveyorWorkpieces) {
        detachConveyorWorkpiece(workpiece);
    }
    g_scene.conveyorWorkpieces.clear();
    g_scene.conveyorSpawners.clear();
    resetSceneConveyorCore();
    g_scene.conveyorPhysics.reset();
    conveyorMarkSceneDirty();
}

bool conveyorSpawnAreaClear(CadNode* conveyor, const TransformNodeData& parameters) {
    // Match the dark enclosure drawn over the first half of a spawner conveyor. Production is
    // inhibited only while an existing workpiece overlaps that visible volume; downstream queue
    // length and feeder occupancy are deliberately irrelevant.
    const ConveyorPathPose a = conveyorPathPoseAt(parameters, 0.0);
    const ConveyorPathPose b = conveyorPathPoseAt(parameters, 0.5);
    CadVec3 forward = conveyorNormalized(CadVec3(b.x - a.x, b.y - a.y, b.z - a.z));
    const double horizontal = std::max(
        1.0e-6, std::sqrt(forward.x * forward.x + forward.z * forward.z));
    const CadVec3 side(-forward.z / horizontal, 0.0, forward.x / horizontal);
    const CadVec3 up = conveyorNormalized(CadVec3(
        side.y * forward.z - side.z * forward.y,
        side.z * forward.x - side.x * forward.z,
        side.x * forward.y - side.y * forward.x));
    constexpr double kEnclosureHeightMm = 360.0;
    const double enclosureWidthMm = std::max(180.0, parameters.accessoryWidthMm - 80.0);
    const double surfaceOffsetMm = conveyorSurfaceOffsetMm(parameters);
    const CadVec3 center(
        (a.x + b.x) * 0.5 + up.x * (surfaceOffsetMm + kEnclosureHeightMm * 0.5),
        (a.y + b.y) * 0.5 + up.y * (surfaceOffsetMm + kEnclosureHeightMm * 0.5),
        (a.z + b.z) * 0.5 + up.z * (surfaceOffsetMm + kEnclosureHeightMm * 0.5));
    const double halfForward = conveyorPointDistance(
        CadVec3(a.x, a.y, a.z), CadVec3(b.x, b.y, b.z)) * 0.5;
    const double halfSide = enclosureWidthMm * 0.5;
    const double halfUp = kEnclosureHeightMm * 0.5;
    const CadTransform conveyorWorld = parentWorldTransformOf(conveyor) * conveyor->loc;
    const CadTransform worldToConveyor = conveyorWorld.rigidInverse();
    for (const ConveyorWorkpiece& piece : g_scene.conveyorWorkpieces) {
        if (!piece.node) continue;
        CadTransform pieceWorld;
        if (!piece.physicsBody ||
            !g_scene.conveyorPhysics.bodyPose(piece.physicsBody, &pieceWorld)) {
            pieceWorld = parentWorldTransformOf(piece.node.get()) * piece.node->loc;
        }
        const CadTransform pieceLocal = worldToConveyor * pieceWorld;
        const CadVec3 local(pieceLocal.values[3], pieceLocal.values[7], pieceLocal.values[11]);
        const CadVec3 delta(local.x - center.x, local.y - center.y, local.z - center.z);
        const double footprintRadius = std::max(1.0, piece.radiusMm);
        const double verticalRadius = std::max(1.0, piece.halfExtentsMm.y);
        if (std::abs(delta.x * forward.x + delta.y * forward.y + delta.z * forward.z) <=
                halfForward + footprintRadius &&
            std::abs(delta.x * side.x + delta.y * side.y + delta.z * side.z) <=
                halfSide + footprintRadius &&
            std::abs(delta.x * up.x + delta.y * up.y + delta.z * up.z) <=
                halfUp + verticalRadius) {
            return false;
        }
    }
    return true;
}

std::shared_ptr<CadNode> cloneCadNodeTreeSharedGeometry(const CadNode* source) {
    if (!source) return nullptr;
    auto clone = std::make_shared<CadNode>();
    clone->name = source->name;
    clone->color = source->color;
    clone->loc = source->loc;
    clone->visible = source->visible;
    clone->excludedFromDecomposition = source->excludedFromDecomposition;
    clone->mountingHoles = source->mountingHoles;
    clone->type = source->type;
    // Workpiece prototypes are immutable while a station runs. Sharing the loaded mesh payloads
    // avoids duplicating vertex buffers for every spawned object; transforms and visibility still
    // belong to each clone's CadNode tree.
    clone->data = source->data;
    for (const std::shared_ptr<CadNode>& child : source->children) {
        std::shared_ptr<CadNode> childClone = cloneCadNodeTreeSharedGeometry(child.get());
        if (!childClone) continue;
        childClone->parent = clone.get();
        clone->children.push_back(std::move(childClone));
    }
    return clone;
}

void includeCadNodeBounds(const CadNode* node, const CadTransform& parent,
                          CadVec3& minimum, CadVec3& maximum, bool& haveBounds) {
    if (!node) return;
    const CadTransform transform = parent * node->loc;
    if (const MeshGeometryData* mesh = node->asMeshGeometry(); mesh && mesh->loaded) {
        for (int xi = 0; xi < 2; ++xi) {
            for (int yi = 0; yi < 2; ++yi) {
                for (int zi = 0; zi < 2; ++zi) {
                    const CadVec3 corner(mesh->bounds[xi ? 3 : 0],
                                         mesh->bounds[yi ? 4 : 1],
                                         mesh->bounds[zi ? 5 : 2]);
                    const CadVec3 point = transform * corner;
                    if (!haveBounds) {
                        minimum = maximum = point;
                        haveBounds = true;
                    } else {
                        minimum.x = std::min(minimum.x, point.x);
                        minimum.y = std::min(minimum.y, point.y);
                        minimum.z = std::min(minimum.z, point.z);
                        maximum.x = std::max(maximum.x, point.x);
                        maximum.y = std::max(maximum.y, point.y);
                        maximum.z = std::max(maximum.z, point.z);
                    }
                }
            }
        }
    }
    for (const std::shared_ptr<CadNode>& child : node->children) {
        includeCadNodeBounds(child.get(), transform, minimum, maximum, haveBounds);
    }
}

CadTransform nodeWorldTransform(const CadNode* node) {
    return node ? parentWorldTransformOf(node) * node->loc : CadTransform();
}

CadTransform actuatorTcpWorld(const ToolActuatorRuntime& runtime) {
    if (!runtime.tool || !runtime.actuator) return CadTransform();
    const RobotToolData* tool = runtime.tool->asRobotTool();
    const int tcp = runtime.actuator->tcpIndex;
    if (!tool || tcp < 0 || tcp >= static_cast<int>(tool->tcps.size())) {
        return nodeWorldTransform(runtime.tool);
    }
    return nodeWorldTransform(runtime.tool) * tool->tcps[static_cast<size_t>(tcp)].loc;
}

CadNode* nearestConveyorForRelease(const CadVec3& worldPoint, double* progress);

ConveyorWorkpiece* nearestWorkpieceTo(const CadTransform& worldPose, double radiusMm) {
    ConveyorWorkpiece* nearest = nullptr;
    double nearestDistance = radiusMm;
    for (ConveyorWorkpiece& piece : g_scene.conveyorWorkpieces) {
        if (!piece.node || piece.graspTool) continue;
        CadTransform world;
        if (!piece.physicsBody || !g_scene.conveyorPhysics.bodyPose(piece.physicsBody, &world)) {
            world = nodeWorldTransform(piece.node.get());
        }
        const double dx = world.values[3] - worldPose.values[3];
        const double dy = world.values[7] - worldPose.values[7];
        const double dz = world.values[11] - worldPose.values[11];
        const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (distance <= nearestDistance) {
            nearest = &piece;
            nearestDistance = distance;
        }
    }
    return nearest;
}

void attachLogicalWorkpiece(ToolActuatorRuntime& runtime) {
    if (!runtime.tool || !runtime.actuator) return;
    const CadTransform tcpWorld = actuatorTcpWorld(runtime);
    ConveyorWorkpiece* nearest = nearestWorkpieceTo(tcpWorld, runtime.actuator->captureRadiusMm);
    if (!nearest) {
        g_scene.stationSimulationStatus = runtime.actuator->name +
            " closed without a workpiece inside its capture volume.";
        return;
    }
    CadTransform world;
    if (!nearest->physicsBody || !g_scene.conveyorPhysics.bodyPose(nearest->physicsBody, &world)) {
        world = nodeWorldTransform(nearest->node.get());
    }
    nearest->sourceConveyor = nearest->conveyor;
    if (nearest->physicsBody) {
        g_scene.conveyorPhysics.removeBody(nearest->physicsBody);
        nearest->physicsBody = 0;
    }
    detachConveyorWorkpiece(*nearest);
    nearest->node->parent = runtime.tool;
    nearest->node->loc = nodeWorldTransform(runtime.tool).rigidInverse() * world;
    nearest->node->needsGlobalLocUpdate = true;
    runtime.tool->children.push_back(nearest->node);
    nearest->graspTool = runtime.tool;
    nearest->graspActuatorId = runtime.actuator->id;
    nearest->logicalAttachment = true;
    g_scene.stationSimulationStatus = runtime.actuator->name + " logically grasped " +
        nearest->node->name + ".";
    conveyorMarkSceneDirty();
}

void releaseLogicalWorkpiece(ToolActuatorRuntime& runtime) {
    for (ConveyorWorkpiece& piece : g_scene.conveyorWorkpieces) {
        if (!piece.logicalAttachment || piece.graspTool != runtime.tool ||
            piece.graspActuatorId != runtime.actuator->id || !piece.node) continue;
        const CadTransform world = nodeWorldTransform(piece.node.get());
        detachConveyorWorkpiece(piece);
        double destinationProgress = 0.0;
        CadNode* destination = nearestConveyorForRelease(
            CadVec3(world.values[3], world.values[7], world.values[11]),
            &destinationProgress);
        if (!destination) destination = g_scene.root.get();
        piece.node->parent = destination;
        piece.node->loc = nodeWorldTransform(destination).rigidInverse() * world;
        piece.node->needsGlobalLocUpdate = true;
        destination->children.push_back(piece.node);
        piece.conveyor = conveyorParameters(destination) ? destination : nullptr;
        piece.sourceConveyor = piece.conveyor;
        piece.progress = destinationProgress;
        piece.forward = true;
        piece.graspTool = nullptr;
        piece.graspActuatorId.clear();
        piece.logicalAttachment = false;
        if (piece.conveyor) {
            if (TransformNodeData* parameters = conveyorParameters(piece.conveyor);
                parameters && resolvedConveyorMode(*parameters) == ConveyorSimulationMode::PhysX) {
                std::string error;
                piece.physicsBody = g_scene.conveyorPhysics.addBox(
                    world, piece.halfExtentsMm, &error);
                if (!piece.physicsBody && !error.empty()) g_scene.stationSimulationStatus = error;
            }
        }
        conveyorMarkSceneDirty();
        return;
    }
}

void applyToolActuatorPosition(ToolActuatorRuntime& runtime, double requestedPosition) {
    if (!runtime.actuator) return;
    MechanismActuatorData& actuator = *runtime.actuator;
    actuator.position = std::max(actuator.lowerLimit,
                                 std::min(actuator.upperLimit, requestedPosition));
    for (size_t index = 0; index < actuator.bindings.size() &&
                           index < runtime.bindingBaseLocs.size(); ++index) {
        MechanismActuatorBinding& binding = actuator.bindings[index];
        if (!binding.node) continue;
        binding.node->loc = runtime.bindingBaseLocs[index];
        const double offset = binding.mmPerUnit * actuator.position;
        binding.node->loc.values[3] += binding.translationAxis.x * offset;
        binding.node->loc.values[7] += binding.translationAxis.y * offset;
        binding.node->loc.values[11] += binding.translationAxis.z * offset;
        binding.node->needsGlobalLocUpdate = true;
    }
    const double span = std::max(1.0e-9, actuator.upperLimit - actuator.lowerLimit);
    const bool closed = (actuator.position - actuator.lowerLimit) / span >= 0.8;
    const bool logicalInteraction = actuator.interaction == "logical-grasp" ||
        (actuator.interaction == "physics-grasp" && !ConveyorPhysics::backendAvailable());
    if (logicalInteraction) {
        if (closed && !runtime.wasClosed) attachLogicalWorkpiece(runtime);
        if (!closed && runtime.wasClosed) releaseLogicalWorkpiece(runtime);
    } else if (actuator.interaction == "physics-grasp") {
        if (closed && !runtime.wasClosed) {
            ConveyorWorkpiece* nearest = nearestWorkpieceTo(
                actuatorTcpWorld(runtime), actuator.captureRadiusMm);
            runtime.physicsGraspBody = nearest ? nearest->physicsBody : 0;
            runtime.hasPhysicsGraspTcpRelative = false;
            if (runtime.physicsGraspBody) {
                CadTransform bodyWorld;
                const CadTransform tcpWorld = actuatorTcpWorld(runtime);
                if (g_scene.conveyorPhysics.bodyPose(runtime.physicsGraspBody, &bodyWorld)) {
                    runtime.physicsGraspTcpRelative = tcpWorld.rigidInverse() * bodyWorld;
                    runtime.hasPhysicsGraspTcpRelative = true;
                }
            }
            g_scene.stationSimulationStatus = runtime.physicsGraspBody
                ? actuator.name + " established a PhysX contact grasp."
                : actuator.name + " closed without a physical workpiece inside its capture volume.";
        }
        if (!closed && runtime.physicsGraspBody) {
            CadTransform bodyWorld;
            if (g_scene.conveyorPhysics.bodyPose(runtime.physicsGraspBody, &bodyWorld)) {
                for (ConveyorWorkpiece& piece : g_scene.conveyorWorkpieces) {
                    if (piece.physicsBody != runtime.physicsGraspBody || !piece.node) continue;
                    double destinationProgress = 0.0;
                    CadNode* destination = nearestConveyorForRelease(
                        CadVec3(bodyWorld.values[3], bodyWorld.values[7], bodyWorld.values[11]),
                        &destinationProgress);
                    if (destination && destination != piece.node->parent) {
                        detachConveyorWorkpiece(piece);
                        piece.node->parent = destination;
                        piece.node->loc = nodeWorldTransform(destination).rigidInverse() * bodyWorld;
                        piece.node->needsGlobalLocUpdate = true;
                        destination->children.push_back(piece.node);
                    }
                    if (destination) {
                        piece.conveyor = destination;
                        piece.sourceConveyor = destination;
                        piece.progress = destinationProgress;
                        piece.forward = true;
                    }
                    break;
                }
            }
            g_scene.conveyorPhysics.clearGripForce(runtime.physicsGraspBody);
            runtime.physicsGraspBody = 0;
            runtime.hasPhysicsGraspTcpRelative = false;
            runtime.hasPreviousForceTarget = false;
        }
    }
    runtime.wasClosed = closed;
    conveyorMarkSceneDirty();
}

void rebuildToolActuatorRuntime() {
    g_scene.toolActuators.clear();
    for (StationAccessoryInstance& accessory : g_scene.station.accessories) {
        RobotToolData* toolData = accessory.node ? accessory.node->asRobotTool() : nullptr;
        if (!toolData) continue;
        for (MechanismActuatorData& actuator : toolData->actuators) {
            ToolActuatorRuntime runtime;
            runtime.tool = accessory.node;
            runtime.actuator = &actuator;
            runtime.wasClosed = actuator.position >= actuator.upperLimit - 1.0e-6;
            for (MechanismActuatorBinding& binding : actuator.bindings) {
                CadTransform base = binding.node ? binding.node->loc : CadTransform();
                const double offset = binding.mmPerUnit * actuator.position;
                base.values[3] -= binding.translationAxis.x * offset;
                base.values[7] -= binding.translationAxis.y * offset;
                base.values[11] -= binding.translationAxis.z * offset;
                runtime.bindingBaseLocs.push_back(base);
                runtime.physicsJawBodies.push_back(0);
            }
            g_scene.toolActuators.push_back(std::move(runtime));
        }
    }
}

ToolActuatorRuntime* findToolActuatorRuntime(const std::string& mechanismId,
                                             const std::string& actuatorId) {
    CadNode* requestedTool = nullptr;
    for (StationAccessoryInstance& accessory : g_scene.station.accessories) {
        if (accessory.id == mechanismId) requestedTool = accessory.node;
    }
    for (ToolActuatorRuntime& runtime : g_scene.toolActuators) {
        if (runtime.tool == requestedTool && runtime.actuator &&
            runtime.actuator->id == actuatorId) return &runtime;
    }
    return nullptr;
}

void commandToolActuator(const ActuateData& command) {
    ToolActuatorRuntime* runtime = findToolActuatorRuntime(command.mechanismId,
                                                           command.actuatorId);
    if (!runtime) {
        g_scene.stationSimulationStatus = "Actuator not found: " + command.mechanismId +
            " / " + command.actuatorId;
        return;
    }
    applyToolActuatorPosition(*runtime, command.position);
}

bool commandProgramTool(RobotInstance& robot, const SetToolData& command,
                        std::string* errorMessage) {
    int robotIndex = -1;
    for (int index = 0; index < static_cast<int>(g_scene.robots.size()); ++index) {
        if (g_scene.robots[static_cast<size_t>(index)].get() == &robot) {
            robotIndex = index;
            break;
        }
    }
    if (robotIndex < 0 || robotIndex >= static_cast<int>(g_scene.station.robots.size())) {
        if (errorMessage) *errorMessage = "SetTool could not resolve the program robot.";
        return false;
    }

    StationRobotInstance& stationRobot = g_scene.station.robots[static_cast<size_t>(robotIndex)];
    StationAccessoryInstance* selected = nullptr;
    for (StationAccessoryInstance& accessory : g_scene.station.accessories) {
        if (accessory.id == command.toolId && accessory.parentRobotId == stationRobot.id) {
            selected = &accessory;
            break;
        }
    }
    RobotToolData* tool = selected && selected->node ? selected->node->asRobotTool() : nullptr;
    OPW6RobotData* robotData = robot.poseController.robotData();
    if (!selected || !tool || !robotData) {
        if (errorMessage) *errorMessage = "SetTool references an unattached robot tool: " +
            command.toolId;
        return false;
    }
    if (command.tcpIndex < 0 || command.tcpIndex >= static_cast<int>(tool->tcps.size())) {
        if (errorMessage) *errorMessage = strutil::format("SetTool TCP %1 is out of range for %2.")
            .arg(command.tcpIndex).arg(command.toolId).str();
        return false;
    }

    robotData->activeTool = selected->node;
    tool->activeTcpIndex = command.tcpIndex;
    selected->activeTcpIndex = command.tcpIndex;
    stationRobot.activeToolId = command.toolId;
    std::string bindError;
    if (!robot.poseController.refreshActiveToolBind(&bindError)) {
        if (errorMessage) *errorMessage = "SetTool failed: " + bindError;
        return false;
    }
    robot.toolTargetPose = robot.poseController.toolPose();
    robot.toolWprBasePose = robot.toolTargetPose;
    robot.toolWprDegrees = {{0.0, 0.0, 0.0}};
    robot.readoutJointsValid = false;
    conveyorMarkSceneDirty();
    return true;
}

void updatePhysicsToolJaws() {
    if (!g_scene.conveyorPhysics.isActive()) return;
    for (ToolActuatorRuntime& runtime : g_scene.toolActuators) {
        if (!runtime.actuator || runtime.actuator->interaction != "physics-grasp") continue;
        for (size_t index = 0; index < runtime.actuator->bindings.size(); ++index) {
            MechanismActuatorBinding& binding = runtime.actuator->bindings[index];
            if (!binding.node) continue;
            const MeshGeometryData* mesh = binding.node->asMeshGeometry();
            if (!mesh || !mesh->loaded) continue;
            const CadTransform pose = nodeWorldTransform(binding.node);
            ConveyorPhysics::BodyHandle& handle = runtime.physicsJawBodies[index];
            if (!handle) {
                std::string error;
                std::vector<CadVec3> vertices;
                vertices.reserve(mesh->vertices.size() / 3);
                for (size_t vertex = 0; vertex + 2 < mesh->vertices.size(); vertex += 3) {
                    vertices.emplace_back(mesh->vertices[vertex], mesh->vertices[vertex + 1],
                                          mesh->vertices[vertex + 2]);
                }
                handle = g_scene.conveyorPhysics.addKinematicConvex(pose, vertices, &error);
                if (!handle && !error.empty()) g_scene.stationSimulationStatus = error;
            } else {
                g_scene.conveyorPhysics.setKinematicPose(handle, pose);
            }
        }
    }
}

void applyPhysicsToolForces(double seconds) {
    for (ToolActuatorRuntime& runtime : g_scene.toolActuators) {
        if (!runtime.actuator || !runtime.wasClosed || !runtime.physicsGraspBody ||
            runtime.actuator->interaction != "physics-grasp") continue;
        CadTransform body;
        if (!g_scene.conveyorPhysics.bodyPose(runtime.physicsGraspBody, &body)) continue;
        const CadTransform tcpWorld = actuatorTcpWorld(runtime);
        const CadTransform target = runtime.hasPhysicsGraspTcpRelative
            ? tcpWorld * runtime.physicsGraspTcpRelative : tcpWorld;
        CadVec3 targetVelocity;
        if (runtime.hasPreviousForceTarget && seconds > 1.0e-9) {
            targetVelocity = CadVec3(
                (target.values[3] - runtime.previousForceTarget.values[3]) / seconds,
                (target.values[7] - runtime.previousForceTarget.values[7]) / seconds,
                (target.values[11] - runtime.previousForceTarget.values[11]) / seconds);
            // A program snapshot can cross an instruction boundary in one fixed tick. Do not turn
            // that sampling discontinuity into an unbounded feed-forward impulse on the product.
            constexpr double kMaximumGripTargetSpeedMmS = 1000.0;
            const double targetSpeed = std::sqrt(
                targetVelocity.x * targetVelocity.x + targetVelocity.y * targetVelocity.y +
                targetVelocity.z * targetVelocity.z);
            if (targetSpeed > kMaximumGripTargetSpeedMmS) {
                const double scale = kMaximumGripTargetSpeedMmS / targetSpeed;
                targetVelocity.x *= scale;
                targetVelocity.y *= scale;
                targetVelocity.z *= scale;
            }
        }
        runtime.previousForceTarget = target;
        runtime.hasPreviousForceTarget = true;
        // The capture radius is an acquisition test, not a break-away distance. Once opposing jaws
        // have acquired a body, keep advancing the pad-force target until the actuator opens. The
        // configured effort limit remains the physical bound; abandoning target updates here leaves
        // a stale force target behind and makes an otherwise held part appear to slip catastrophically.
        g_scene.conveyorPhysics.applyGripForce(
            runtime.physicsGraspBody, target, targetVelocity,
            // The generated 30 mm workpiece is clamped to a 0.1 kg minimum mass. These values are
            // close to critical damping at that mass, while effortLimit still caps the pad force.
            /*stiffnessNPerM=*/550.0, /*dampingNsPerM=*/15.0,
            std::max(1.0, runtime.actuator->effortLimit));
    }
}

const StationAccessoryInstance* conveyorSpawnPrototype(const TransformNodeData& parameters) {
    if (parameters.accessorySpawnObjectId.empty()) return nullptr;
    for (const StationAccessoryInstance& accessory : g_scene.station.accessories) {
        if (accessory.id == parameters.accessorySpawnObjectId && accessory.hidden && accessory.node) {
            return &accessory;
        }
    }
    return nullptr;
}

size_t activeSpawnCount(CadNode* spawner) {
    return static_cast<size_t>(std::count_if(
        g_scene.conveyorWorkpieces.begin(), g_scene.conveyorWorkpieces.end(),
        [&](const ConveyorWorkpiece& piece) { return piece.originSpawner == spawner; }));
}

void spawnConveyorWorkpiece(CadNode* conveyor, double requestedProgress) {
    TransformNodeData* parameters = conveyorParameters(conveyor);
    if (!parameters) return;
    if (parameters->accessoryConveyorRole == "spawner" &&
        parameters->accessoryMaxActiveSpawns > 0 &&
        activeSpawnCount(conveyor) >=
            static_cast<size_t>(parameters->accessoryMaxActiveSpawns)) {
        return;
    }
    const StationAccessoryInstance* prototype = conveyorSpawnPrototype(*parameters);
    if (!prototype) {
        g_scene.stationSimulationStatus = parameters->accessorySpawnObjectId.empty()
            ? "Spawner has no source object. Select one from the Hidden items folder."
            : "Spawner source object '" + parameters->accessorySpawnObjectId +
                  "' is missing or is not hidden.";
        return;
    }
    const uint64_t workpieceId = g_scene.nextWorkpieceId++;
    auto node = std::make_shared<CadNode>();
    node->name = "Spawned " + (!prototype->name.empty()
        ? prototype->name : std::string("object")) + " " +
        std::to_string(workpieceId);
    node->type = CadNodeType::Custom;
    std::shared_ptr<CadNode> visual = cloneCadNodeTreeSharedGeometry(prototype->node);
    visual->loc = CadTransform(); // discard the prototype's hidden station placement
    visual->setVisibleRecursive(true);

    CadVec3 minimum, maximum;
    bool haveBounds = false;
    includeCadNodeBounds(visual.get(), CadTransform(), minimum, maximum, haveBounds);
    if (!haveBounds) return;
    const CadVec3 center((minimum.x + maximum.x) * 0.5,
                         (minimum.y + maximum.y) * 0.5,
                         (minimum.z + maximum.z) * 0.5);
    if (requestedProgress < 0.0 && !conveyorSpawnAreaClear(conveyor, *parameters)) return;
    visual->loc.values[3] -= center.x;
    visual->loc.values[7] -= center.y;
    visual->loc.values[11] -= center.z;
    visual->parent = node.get();
    node->children.push_back(visual);
    node->parent = conveyor;
    conveyor->children.push_back(node);

    ConveyorWorkpiece workpiece;
    workpiece.id = workpieceId;
    workpiece.node = std::move(node);
    workpiece.conveyor = conveyor;
    if (parameters->accessoryConveyorRole == "spawner") {
        workpiece.originSpawner = conveyor;
    }
    const bool physical = resolvedConveyorMode(*parameters) == ConveyorSimulationMode::PhysX;
    if (requestedProgress >= 0.0) {
        workpiece.progress = std::max(0.0, std::min(1.0, requestedProgress));
    } else if (physical) {
        const double pathLength = conveyorPathLengthMm(*parameters);
        const double firstRollerDistance = conveyorRollerInsetMm(*parameters);
        // Drop between rollers two and three instead of balancing at the end of the deck. Keeping
        // one complete roller bay behind the footprint avoids an artificial edge-tip on coarse
        // pitches. Rotation remains completely unconstrained after contact.
        const double supportedDropDistance = std::min(
            pathLength - firstRollerDistance,
            firstRollerDistance + parameters->accessoryRollerPitchMm * 1.5);
        workpiece.progress = supportedDropDistance / pathLength;
    } else {
        workpiece.progress = 0.0;
    }
    workpiece.forward = true;
    workpiece.halfExtentsMm = CadVec3(
        std::max(1.0, (maximum.x - minimum.x) * 0.5),
        std::max(1.0, (maximum.y - minimum.y) * 0.5),
        std::max(1.0, (maximum.z - minimum.z) * 0.5));
    workpiece.radiusMm = std::hypot(workpiece.halfExtentsMm.x,
                                    workpiece.halfExtentsMm.z);
    workpiece.heightMm = workpiece.halfExtentsMm.y * 2.0;
    placeConveyorWorkpiece(workpiece);
    if (physical) {
        // Start clear of the contact offset and let gravity establish the first roller contacts.
        // This is a physical drop, unlike logical mode's exact path placement.
        workpiece.node->loc.values[7] += 25.0;
        const CadTransform conveyorWorld = parentWorldTransformOf(conveyor) * conveyor->loc;
        const CadTransform worldPose = conveyorWorld * workpiece.node->loc;
        std::string physicsError;
        workpiece.physicsBody = g_scene.conveyorPhysics.addBox(
            worldPose, workpiece.halfExtentsMm, &physicsError);
        if (!workpiece.physicsBody) {
            detachConveyorWorkpiece(workpiece);
            g_scene.stationSimulationStatus = physicsError;
            return;
        }
    }
    g_scene.conveyorWorkpieces.push_back(std::move(workpiece));
    conveyorMarkSceneDirty();
}

ConveyorTransferTarget nextConveyor(CadNode* current, bool leavingForward) {
    std::vector<conveyorcore::Lane> lanes;
    std::vector<CadNode*> nodes;
    lanes.reserve(g_scene.station.accessories.size());
    nodes.reserve(g_scene.station.accessories.size());
    // Every accessory, conveyor or not, so an index into `lanes` indexes `nodes` too. A non-conveyor
    // carries a null parameter block and the search skips it.
    for (const StationAccessoryInstance& entry : g_scene.station.accessories) {
        conveyorcore::Lane lane;
        lane.parameters = conveyorParameters(entry.node);
        if (lane.parameters) {
            lane.world = parentWorldTransformOf(entry.node) * entry.node->loc;
            lane.mode = resolvedConveyorMode(*lane.parameters) == ConveyorSimulationMode::PhysX
                ? conveyorcore::Mode::Physx
                : conveyorcore::Mode::Logical;
        }
        lanes.push_back(lane);
        nodes.push_back(entry.node);
    }
    const TransformNodeData* currentParameters = conveyorParameters(current);
    if (!currentParameters) return {};
    const auto existing = std::find(nodes.begin(), nodes.end(), current);
    size_t from = static_cast<size_t>(std::distance(nodes.begin(), existing));
    if (existing == nodes.end()) {
        conveyorcore::Lane lane;
        lane.parameters = currentParameters;
        lane.world = parentWorldTransformOf(current) * current->loc;
        lane.mode = resolvedConveyorMode(*currentParameters) == ConveyorSimulationMode::PhysX
            ? conveyorcore::Mode::Physx
            : conveyorcore::Mode::Logical;
        from = lanes.size();
        lanes.push_back(lane);
        nodes.push_back(current);
    }

    const conveyorcore::Handover found = conveyorcore::nextLane(lanes, from, leavingForward);
    ConveyorTransferTarget best;
    if (!found.valid()) return best;
    best.conveyor = nodes[static_cast<size_t>(found.lane)];
    best.forward = found.forward;
    return best;
}

void reparentConveyorWorkpiece(ConveyorWorkpiece& workpiece, CadNode* destination) {
    if (!workpiece.node || !destination || workpiece.conveyor == destination) return;
    detachConveyorWorkpiece(workpiece);
    workpiece.conveyor = destination;
    workpiece.node->parent = destination;
    destination->children.push_back(workpiece.node);
}

CadVec3 conveyorTangentAt(CadNode* conveyor, double progress, bool forward);

void appendPhysicsCollisionMeshes(CadNode* conveyor,
                                  std::vector<ConveyorPhysics::CollisionMesh>& meshes) {
    if (!conveyor) return;
    // These are the generated frame, feet and rollers used by rebuildRollerConveyor. Cooking these
    // exact buffers keeps rendering and collision on one geometry source of truth.
    for (const std::shared_ptr<CadNode>& child : conveyor->children) {
        const MeshGeometryData* geometry = child ? child->asMeshGeometry() : nullptr;
        if (!geometry || geometry->vertices.empty() || geometry->indices.empty()) continue;
        // Role enclosures are operator-facing machine markers. Spawned products intentionally
        // travel through the hidden half of the conveyor, so these visual meshes are not collision.
        if (child->name == "Conveyor role enclosure" ||
            child->name == "Spawner product and infinity markings" ||
            child->name == "Deleter X markings") {
            continue;
        }
        const bool rollers = conveyor->children.size() > 2 &&
                             child.get() == conveyor->children[2].get();
        if (rollers) {
            constexpr size_t kRollerVertices = 42; // 2 * 20 ring vertices + 2 cap centers.
            constexpr size_t kRollerIndices = 240; // 20 side quads + two 20-triangle caps.
            const size_t rollerCount = geometry->vertices.size() / (kRollerVertices * 3);
            if (rollerCount >= 2 && geometry->vertices.size() == rollerCount * kRollerVertices * 3 &&
                geometry->indices.size() == rollerCount * kRollerIndices) {
                const TransformNodeData* parameters = conveyorParameters(conveyor);
                const double pathLength = conveyorPathLengthMm(*parameters);
                const double rollerInset = conveyorRollerInsetMm(*parameters);
                const double rollerRun = std::max(1.0, pathLength - rollerInset * 2.0);
                for (size_t roller = 0; roller < rollerCount; ++roller) {
                    const size_t vertexStart = roller * kRollerVertices;
                    const size_t indexStart = roller * kRollerIndices;
                    ConveyorPhysics::CollisionMesh collision;
                    collision.pose = parentWorldTransformOf(child.get()) * child->loc;
                    collision.verticesMm.assign(
                        geometry->vertices.begin() + vertexStart * 3,
                        geometry->vertices.begin() + (vertexStart + kRollerVertices) * 3);
                    collision.indices.reserve(kRollerIndices);
                    for (size_t index = 0; index < kRollerIndices; ++index) {
                        collision.indices.push_back(
                            geometry->indices[indexStart + index] -
                            static_cast<uint32_t>(vertexStart));
                    }
                    const double distance = rollerInset + rollerRun *
                        static_cast<double>(roller) / static_cast<double>(rollerCount - 1);
                    const ConveyorPathPose rollerPose = conveyorPathPoseAt(
                        *parameters, distance / pathLength);
                    const double rollerRadius = conveyorRollerRadiusMm(*parameters);
                    const CadVec3 centerLocal(rollerPose.x,
                                              rollerPose.y - rollerRadius,
                                              rollerPose.z);
                    collision.surfaceCenterMm = collision.pose * centerLocal;
                    const double rollerHalfWidth = parameters->accessoryWidthMm * 0.5 - 45.0;
                    const double rightDrop = conveyorRightDropAt(
                        *parameters, distance / pathLength);
                    const CadVec3 axis = conveyorNormalized(rotate(
                        collision.pose,
                        CadVec3(-std::sin(rollerPose.heading),
                                rollerHalfWidth > 1.0e-6
                                    ? -rightDrop / (rollerHalfWidth * 2.0) : 0.0,
                                std::cos(rollerPose.heading))));
                    const double speed = parameters->accessoryConveyorSpeedMmS;
                    const double angularSpeed = speed / rollerRadius;
                    collision.surfaceAngularVelocityRadS = CadVec3(
                        -axis.x * angularSpeed,
                        -axis.y * angularSpeed,
                        -axis.z * angularSpeed);
                    meshes.push_back(std::move(collision));
                }
                continue;
            }
        }
        const bool cover = child->name == "Black roller cover";
        if (cover) {
            constexpr size_t kCoverVertices = 4;
            constexpr size_t kCoverIndices = 6;
            const size_t segmentCount = geometry->vertices.size() / (kCoverVertices * 3);
            if (segmentCount >= 1 &&
                geometry->vertices.size() == segmentCount * kCoverVertices * 3 &&
                geometry->indices.size() == segmentCount * kCoverIndices) {
                const TransformNodeData* parameters = conveyorParameters(conveyor);
                const bool banked =
                    std::abs(parameters->accessoryStartLeftHeightMm -
                             parameters->accessoryStartRightHeightMm) > 0.01 ||
                    std::abs(parameters->accessoryEndLeftHeightMm -
                             parameters->accessoryEndRightHeightMm) > 0.01;
                const bool constantDirection =
                    std::abs(parameters->accessoryTurnAngleDeg) < 0.001 && !banked;
                const bool pickFeeder =
                    parameters->accessoryConveyorRole == "pick_feeder";
                if (pickFeeder) {
                    const CadTransform coverWorld =
                        parentWorldTransformOf(child.get()) * child->loc;

                    // Passive steel follows the same segmented profile as the visible belt. In the
                    // final pickup pocket this becomes the centre support tongue instead of an
                    // invisible full-width slab across the two finger channels.
                    constexpr double kBedHalfThicknessMm = 30.0;
                    const double bedTopOffset = conveyorSurfaceOffsetMm(*parameters) - 1.0;
                    for (size_t segment = 0; segment < segmentCount; ++segment) {
                        const double ta = static_cast<double>(segment) / segmentCount;
                        const double tb = static_cast<double>(segment + 1) / segmentCount;
                        const ConveyorPathPose a = conveyorPathPoseAt(*parameters, ta);
                        const ConveyorPathPose b = conveyorPathPoseAt(*parameters, tb);
                        const CadVec3 forward = conveyorNormalized(CadVec3(
                            b.x - a.x, b.y - a.y, b.z - a.z));
                        const CadVec3 side = conveyorNormalized(CadVec3(
                            -std::sin((a.heading + b.heading) * 0.5), 0.0,
                            std::cos((a.heading + b.heading) * 0.5)));
                        const CadVec3 up = conveyorNormalized(CadVec3(
                            side.y * forward.z - side.z * forward.y,
                            side.z * forward.x - side.x * forward.z,
                            side.x * forward.y - side.y * forward.x));
                        CadTransform localBed;
                        localBed.values = {{forward.x, up.x, side.x,
                            (a.x + b.x) * 0.5 + up.x * (bedTopOffset - kBedHalfThicknessMm),
                            forward.y, up.y, side.y,
                            (a.y + b.y) * 0.5 + up.y * (bedTopOffset - kBedHalfThicknessMm),
                            forward.z, up.z, side.z,
                            (a.z + b.z) * 0.5 + up.z * (bedTopOffset - kBedHalfThicknessMm)}};
                        ConveyorPhysics::CollisionMesh passiveBed;
                        passiveBed.pose = coverWorld * localBed;
                        passiveBed.boxHalfExtentsMm = CadVec3(
                            conveyorPointDistance(CadVec3(a.x, a.y, a.z),
                                                  CadVec3(b.x, b.y, b.z)) * 0.5 + 2.0,
                            kBedHalfThicknessMm,
                            (robotPickFeederDeckHalfWidthAt(*parameters, ta) +
                             robotPickFeederDeckHalfWidthAt(*parameters, tb)) * 0.5);
                        meshes.push_back(std::move(passiveBed));
                    }

                    for (size_t segment = 0; segment < segmentCount; ++segment) {
                        const double ta = static_cast<double>(segment) / segmentCount;
                        const double tb = static_cast<double>(segment + 1) / segmentCount;
                        const double progress = (ta + tb) * 0.5;
                        const ConveyorPathPose a = conveyorPathPoseAt(*parameters, ta);
                        const ConveyorPathPose b = conveyorPathPoseAt(*parameters, tb);
                        const CadVec3 forward = conveyorNormalized(CadVec3(
                            b.x - a.x, b.y - a.y, b.z - a.z));
                        const CadVec3 side = conveyorNormalized(CadVec3(
                            -std::sin((a.heading + b.heading) * 0.5), 0.0,
                            std::cos((a.heading + b.heading) * 0.5)));
                        const CadVec3 up = conveyorNormalized(CadVec3(
                            side.y * forward.z - side.z * forward.y,
                            side.z * forward.x - side.x * forward.z,
                            side.x * forward.y - side.y * forward.x));
                        constexpr double halfThicknessMm = 20.0;
                        const double surfaceOffset = conveyorSurfaceOffsetMm(*parameters);
                        CadTransform localPatch;
                        localPatch.values = {{forward.x, up.x, side.x,
                            (a.x + b.x) * 0.5 + up.x * (surfaceOffset - halfThicknessMm),
                            forward.y, up.y, side.y,
                            (a.y + b.y) * 0.5 + up.y * (surfaceOffset - halfThicknessMm),
                            forward.z, up.z, side.z,
                            (a.z + b.z) * 0.5 + up.z * (surfaceOffset - halfThicknessMm)}};
                        ConveyorPhysics::CollisionMesh patch;
                        patch.pose = coverWorld * localPatch;
                        patch.boxHalfExtentsMm = CadVec3(
                            conveyorPointDistance(
                                CadVec3(a.x, a.y, a.z), CadVec3(b.x, b.y, b.z)) * 0.5 + 1.0,
                            halfThicknessMm,
                            (robotPickFeederDeckHalfWidthAt(*parameters, ta) +
                             robotPickFeederDeckHalfWidthAt(*parameters, tb)) * 0.5);
                        double driveScale = 1.0;
                        if (progress >= 0.90) {
                            driveScale = 0.05;
                        } else if (progress > 0.70) {
                            const double blend = (progress - 0.70) / 0.20;
                            driveScale = 1.0 - blend * 0.95;
                        }
                        const CadVec3 tangent = conveyorTangentAt(conveyor, progress, true);
                        const double speed = parameters->accessoryConveyorSpeedMmS * driveScale;
                        patch.surfaceLinearVelocityMmS = CadVec3(
                            tangent.x * speed, tangent.y * speed, tangent.z * speed);
                        meshes.push_back(std::move(patch));
                    }
                    continue;
                }
                if (constantDirection) {
                    const ConveyorPathPose a = conveyorPathPoseAt(*parameters, 0.0);
                    const ConveyorPathPose b = conveyorPathPoseAt(*parameters, 1.0);
                    CadVec3 forward(b.x - a.x, b.y - a.y, b.z - a.z);
                    const double length = std::sqrt(forward.x * forward.x + forward.y * forward.y +
                                                    forward.z * forward.z);
                    if (length > 1.0e-6) {
                        forward = CadVec3(forward.x / length, forward.y / length,
                                          forward.z / length);
                        const double horizontal = std::max(
                            1.0e-6, std::sqrt(forward.x * forward.x + forward.z * forward.z));
                        const CadVec3 side(-forward.z / horizontal, 0.0,
                                           forward.x / horizontal);
                        const CadVec3 up(side.y * forward.z - side.z * forward.y,
                                         side.z * forward.x - side.x * forward.z,
                                         side.x * forward.y - side.y * forward.x);
                        // Ordinary constant-direction covers remain a single inexpensive native
                        // slab. Pick feeders took the segmented deceleration path above.
                        constexpr double halfThicknessMm = 1.0;
                        const double surfaceOffset = conveyorSurfaceOffsetMm(*parameters);
                        const CadVec3 center((a.x + b.x) * 0.5 - up.x * halfThicknessMm +
                                                 up.x * surfaceOffset,
                                             (a.y + b.y) * 0.5 - up.y * halfThicknessMm +
                                                 up.y * surfaceOffset,
                                             (a.z + b.z) * 0.5 - up.z * halfThicknessMm +
                                                 up.z * surfaceOffset);
                        CadTransform localBox;
                        localBox.values = {{forward.x, up.x, side.x, center.x,
                                            forward.y, up.y, side.y, center.y,
                                            forward.z, up.z, side.z, center.z}};
                        ConveyorPhysics::CollisionMesh collision;
                        collision.pose = (parentWorldTransformOf(child.get()) * child->loc) *
                                         localBox;
                        collision.boxHalfExtentsMm = CadVec3(
                            length * 0.5, halfThicknessMm,
                            std::max(20.0, parameters->accessoryWidthMm * 0.5 - 37.0));
                        const CadVec3 tangent = conveyorTangentAt(conveyor, 0.5, true);
                        const double speed = parameters->accessoryConveyorSpeedMmS;
                        collision.surfaceLinearVelocityMmS = CadVec3(
                            tangent.x * speed, tangent.y * speed, tangent.z * speed);
                        meshes.push_back(std::move(collision));
                        continue;
                    }
                }
                const double renderSegmentLength =
                    conveyorPathLengthMm(*parameters) / static_cast<double>(segmentCount);
                // A straight or inclined belt has one target velocity everywhere, so its cover is
                // one native box. Curves retain local tangents as short oriented box chords,
                // grouped over 120 mm. Keeping the render mesh out of the powered collision path
                // avoids the costly convex/triangle contact pairs produced by each curved patch.
                const size_t segmentsPerCollision = constantDirection
                    ? segmentCount
                    : std::max<size_t>(1, static_cast<size_t>(
                          std::floor(120.0 / std::max(1.0, renderSegmentLength))));
                for (size_t segment = 0; segment < segmentCount;
                     segment += segmentsPerCollision) {
                    const size_t groupSegments =
                        std::min(segmentsPerCollision, segmentCount - segment);
                    const double startProgress = static_cast<double>(segment) /
                                                 static_cast<double>(segmentCount);
                    const double endProgress = static_cast<double>(segment + groupSegments) /
                                               static_cast<double>(segmentCount);
                    const ConveyorPathPose a = conveyorPathPoseAt(*parameters, startProgress);
                    const ConveyorPathPose b = conveyorPathPoseAt(*parameters, endProgress);
                    CadVec3 forward(b.x - a.x, b.y - a.y, b.z - a.z);
                    const double chordLength = std::sqrt(forward.x * forward.x +
                                                         forward.y * forward.y +
                                                         forward.z * forward.z);
                    if (chordLength > 1.0e-6) {
                        forward = CadVec3(forward.x / chordLength, forward.y / chordLength,
                                          forward.z / chordLength);
                        const double progress = (startProgress + endProgress) * 0.5;
                        const ConveyorPathPose midpoint = conveyorPathPoseAt(*parameters, progress);
                        const double crossSlope =
                            (conveyorCornerHeightAt(*parameters, progress, true) -
                             conveyorCornerHeightAt(*parameters, progress, false)) /
                            std::max(1.0, parameters->accessoryWidthMm);
                        const CadVec3 rawSide(-std::sin(midpoint.heading), crossSlope,
                                              std::cos(midpoint.heading));
                        const double sideAlongForward = rawSide.x * forward.x +
                                                        rawSide.y * forward.y +
                                                        rawSide.z * forward.z;
                        const CadVec3 side = conveyorNormalized(CadVec3(
                            rawSide.x - forward.x * sideAlongForward,
                            rawSide.y - forward.y * sideAlongForward,
                            rawSide.z - forward.z * sideAlongForward));
                        const CadVec3 up = conveyorNormalized(CadVec3(
                            side.y * forward.z - side.z * forward.y,
                            side.z * forward.x - side.x * forward.z,
                            side.x * forward.y - side.y * forward.x));
                        // Curved products cross a new oriented chord every ~120 mm. Give the
                        // collision slab real depth below its unchanged top surface so a tilted
                        // a fast dynamic workpiece cannot tunnel through a 2 mm mathematical skin at a seam.
                        constexpr double halfThicknessMm = 10.0;
                        const double surfaceOffset = conveyorSurfaceOffsetMm(*parameters);
                        const CadVec3 center((a.x + b.x) * 0.5 +
                                                 up.x * (surfaceOffset - halfThicknessMm),
                                             (a.y + b.y) * 0.5 +
                                                 up.y * (surfaceOffset - halfThicknessMm),
                                             (a.z + b.z) * 0.5 +
                                                 up.z * (surfaceOffset - halfThicknessMm));
                        CadTransform localBox;
                        localBox.values = {{forward.x, up.x, side.x, center.x,
                                            forward.y, up.y, side.y, center.y,
                                            forward.z, up.z, side.z, center.z}};
                        ConveyorPhysics::CollisionMesh collision;
                        collision.pose = (parentWorldTransformOf(child.get()) * child->loc) *
                                         localBox;
                        const double halfWidth = std::max(
                            20.0, parameters->accessoryWidthMm * 0.5 - 37.0);
                        // Adjacent tangent-aligned chords otherwise open a wedge on the inside of
                        // a curve. Extend each end to the intersection of the two tangent planes;
                        // the visible cover is continuous, so its collision surface must be too.
                        const double headingDelta = std::min(
                            1.2, std::abs(static_cast<double>(b.heading - a.heading)));
                        const double seamOverlap = halfWidth * std::tan(headingDelta * 0.5) + 2.0;
                        collision.boxHalfExtentsMm = CadVec3(
                            chordLength * 0.5 + seamOverlap, halfThicknessMm, halfWidth);
                        const CadVec3 tangent = conveyorTangentAt(conveyor, progress, true);
                        const double speed = parameters->accessoryConveyorSpeedMmS;
                        const CadTransform conveyorWorld =
                            parentWorldTransformOf(child.get()) * child->loc;
                        const CadVec3 worldSide = conveyorNormalized(
                            rotate(conveyorWorld, side));
                        const double lateralLimit = speed * 0.75;
                        const double lateralSpeed = std::max(
                            -lateralLimit, std::min(lateralLimit, -crossSlope * 1000.0));
                        collision.surfaceLinearVelocityMmS = CadVec3(
                            tangent.x * speed + worldSide.x * lateralSpeed,
                            tangent.y * speed + worldSide.y * lateralSpeed,
                            tangent.z * speed + worldSide.z * lateralSpeed);
                        meshes.push_back(std::move(collision));
                        continue;
                    }
                    const size_t vertexStart = segment * kCoverVertices;
                    const size_t vertexCount = groupSegments * kCoverVertices;
                    const size_t indexStart = segment * kCoverIndices;
                    const size_t indexCount = groupSegments * kCoverIndices;
                    ConveyorPhysics::CollisionMesh collision;
                    collision.pose = parentWorldTransformOf(child.get()) * child->loc;
                    collision.verticesMm.assign(
                        geometry->vertices.begin() + vertexStart * 3,
                        geometry->vertices.begin() + (vertexStart + vertexCount) * 3);
                    collision.indices.reserve(indexCount);
                    for (size_t index = 0; index < indexCount; ++index) {
                        collision.indices.push_back(
                            geometry->indices[indexStart + index] -
                            static_cast<uint32_t>(vertexStart));
                    }
                    const double progress =
                        (static_cast<double>(segment) +
                         static_cast<double>(groupSegments) * 0.5) /
                        static_cast<double>(segmentCount);
                    const CadVec3 tangent = conveyorTangentAt(conveyor, progress, true);
                    const double speed = parameters->accessoryConveyorSpeedMmS;
                    collision.surfaceLinearVelocityMmS = CadVec3(
                        tangent.x * speed, tangent.y * speed, tangent.z * speed);
                    meshes.push_back(std::move(collision));
                }
                continue;
            }
        }
        const bool frame = !conveyor->children.empty() &&
                           child.get() == conveyor->children[0].get();
        if (frame) {
            const TransformNodeData* parameters = conveyorParameters(conveyor);
            if (parameters && parameters->accessoryConveyorRole == "pick_feeder") {
                // The feeder has one collision representation only - native boxes generated from
                // the same profile as its rendered geometry. A second overlaid set with different
                // dimensions makes contacts bounce between the two surfaces.
                const double pathLength = conveyorPathLengthMm(*parameters);
                const int guideSegments = std::max(
                    1, static_cast<int>(std::min(4096.0, std::ceil(pathLength / 30.0))));
                const CadTransform feederWorld =
                    parentWorldTransformOf(child.get()) * child->loc;
                for (int segment = 0; segment < guideSegments; ++segment) {
                    const double ta = static_cast<double>(segment) / guideSegments;
                    const double tb = static_cast<double>(segment + 1) / guideSegments;
                    const double midpoint = (ta + tb) * 0.5;
                    const ConveyorPathPose a = conveyorPathPoseAt(*parameters, ta);
                    const ConveyorPathPose b = conveyorPathPoseAt(*parameters, tb);
                    const double offsetA =
                        robotPickFeederWallCenterOffsetAt(*parameters, ta);
                    const double offsetB =
                        robotPickFeederWallCenterOffsetAt(*parameters, tb);
                    const double guideHeight = robotPickFeederWallHeightAt(midpoint);
                    if (guideHeight <= 0.5) continue;
                    const double centerAboveDeck = guideHeight * 0.5;
                    for (double sign : {-1.0, 1.0}) {
                        const CadVec3 p0(
                            a.x - std::sin(a.heading) * offsetA * sign,
                            a.y + conveyorSideHeightOffsetAt(
                                      *parameters, ta, sign, offsetA) + centerAboveDeck,
                            a.z + std::cos(a.heading) * offsetA * sign);
                        const CadVec3 p1(
                            b.x - std::sin(b.heading) * offsetB * sign,
                            b.y + conveyorSideHeightOffsetAt(
                                      *parameters, tb, sign, offsetB) + centerAboveDeck,
                            b.z + std::cos(b.heading) * offsetB * sign);
                        const CadVec3 guideForward = conveyorNormalized(CadVec3(
                            p1.x - p0.x, p1.y - p0.y, p1.z - p0.z));
                        const CadVec3 guideUp(0.0, 1.0, 0.0);
                        const CadVec3 guideSide = conveyorNormalized(CadVec3(
                            guideForward.z, 0.0, -guideForward.x));
                        const double guideLength = conveyorPointDistance(p0, p1);
                        CadTransform localGuide;
                        localGuide.values = {{guideForward.x, guideUp.x, guideSide.x,
                                              (p0.x + p1.x) * 0.5,
                                              guideForward.y, guideUp.y, guideSide.y,
                                              (p0.y + p1.y) * 0.5,
                                              guideForward.z, guideUp.z, guideSide.z,
                                              (p0.z + p1.z) * 0.5}};
                        ConveyorPhysics::CollisionMesh solidGuide;
                        solidGuide.pose = feederWorld * localGuide;
                        solidGuide.boxHalfExtentsMm = CadVec3(
                            guideLength * 0.5, guideHeight * 0.5, 4.0);
                        meshes.push_back(std::move(solidGuide));
                    }
                }

                // Match the visible terminal wall, including the deck-following chute width.
                const ConveyorPathPose end = conveyorPathPoseAt(*parameters, 1.0);
                const ConveyorPathPose before = conveyorPathPoseAt(*parameters, 0.99);
                const CadVec3 forward = conveyorNormalized(CadVec3(
                    end.x - before.x, end.y - before.y, end.z - before.z));
                const CadVec3 side = conveyorNormalized(CadVec3(
                    -std::sin(end.heading), 0.0, std::cos(end.heading)));
                const CadVec3 up = conveyorNormalized(CadVec3(
                    side.y * forward.z - side.z * forward.y,
                    side.z * forward.x - side.x * forward.z,
                    side.x * forward.y - side.y * forward.x));
                const double stopHalfWidth =
                    robotPickFeederWallCenterOffsetAt(*parameters, 1.0);
                constexpr double kStopHalfThicknessMm = 15.0;
                const auto appendStopBox = [&](double sideCenter, double sideHalfExtent,
                                               double height) {
                    CadTransform local;
                    local.values = {{forward.x, up.x, side.x,
                                      end.x + up.x * height * 0.5 + side.x * sideCenter,
                                      forward.y, up.y, side.y,
                                      end.y + up.y * height * 0.5 + side.y * sideCenter,
                                      forward.z, up.z, side.z,
                                      end.z + up.z * height * 0.5 + side.z * sideCenter}};
                    ConveyorPhysics::CollisionMesh box;
                    box.pose = feederWorld * local;
                    box.boxHalfExtentsMm = CadVec3(
                        kStopHalfThicknessMm, height * 0.5, sideHalfExtent);
                    meshes.push_back(std::move(box));
                };
                appendStopBox(
                    0.0, stopHalfWidth, kRobotPickFeederFullWallHeightMm);
                continue;
            }
            const double pathLength = conveyorPathLengthMm(*parameters);
            const size_t pathSegments = static_cast<size_t>(std::max(
                1, static_cast<int>(std::min(4096.0, std::ceil(pathLength / 120.0)))));
            constexpr size_t kBoxVertices = 8;
            constexpr size_t kBoxIndices = 36;
            const size_t boxesPerSegment = parameters->accessorySupportBracesEnabled ? 4 : 2;
            const size_t boxesBeforeTies = pathSegments * boxesPerSegment;
            const size_t tieVertexStart = boxesBeforeTies * kBoxVertices;
            const size_t tieIndexStart = boxesBeforeTies * kBoxIndices;
            if (geometry->vertices.size() >= (tieVertexStart + 2 * kBoxVertices) * 3 &&
                geometry->indices.size() >= tieIndexStart + 2 * kBoxIndices) {
                ConveyorPhysics::CollisionMesh fixedFrame;
                fixedFrame.pose = parentWorldTransformOf(child.get()) * child->loc;
                fixedFrame.verticesMm = geometry->vertices;
                fixedFrame.indices.reserve(geometry->indices.size() - 2 * kBoxIndices);
                fixedFrame.indices.insert(fixedFrame.indices.end(),
                                          geometry->indices.begin(),
                                          geometry->indices.begin() + tieIndexStart);
                fixedFrame.indices.insert(fixedFrame.indices.end(),
                                          geometry->indices.begin() +
                                              tieIndexStart + 2 * kBoxIndices,
                                          geometry->indices.end());
                meshes.push_back(std::move(fixedFrame));

                for (size_t tie = 0; tie < 2; ++tie) {
                    const size_t vertexStart = tieVertexStart + tie * kBoxVertices;
                    const auto vertexAt = [&](size_t vertex) {
                        const size_t offset = (vertexStart + vertex) * 3;
                        return CadVec3(geometry->vertices[offset],
                                       geometry->vertices[offset + 1],
                                       geometry->vertices[offset + 2]);
                    };
                    const CadVec3 v0 = vertexAt(0);
                    const CadVec3 v1 = vertexAt(1);
                    const CadVec3 v2 = vertexAt(2);
                    const CadVec3 v4 = vertexAt(4);
                    const CadVec3 v7 = vertexAt(7);
                    const auto normalizedEdge = [](const CadVec3& from, const CadVec3& to,
                                                   double* length) {
                        CadVec3 edge(to.x - from.x, to.y - from.y, to.z - from.z);
                        *length = std::sqrt(edge.x * edge.x + edge.y * edge.y + edge.z * edge.z);
                        if (*length > 1.0e-9) {
                            edge = CadVec3(edge.x / *length, edge.y / *length,
                                           edge.z / *length);
                        }
                        return edge;
                    };
                    double forwardLength = 0.0;
                    double upLength = 0.0;
                    double sideLength = 0.0;
                    const CadVec3 forward = normalizedEdge(v0, v1, &forwardLength);
                    const CadVec3 up = normalizedEdge(v0, v2, &upLength);
                    const CadVec3 side = normalizedEdge(v0, v4, &sideLength);
                    const CadVec3 center((v0.x + v7.x) * 0.5,
                                         (v0.y + v7.y) * 0.5,
                                         (v0.z + v7.z) * 0.5);
                    CadTransform localBox;
                    localBox.values = {{forward.x, up.x, side.x, center.x,
                                        forward.y, up.y, side.y, center.y,
                                        forward.z, up.z, side.z, center.z}};
                    ConveyorPhysics::CollisionMesh transfer;
                    transfer.pose = (parentWorldTransformOf(child.get()) * child->loc) * localBox;
                    transfer.boxHalfExtentsMm = CadVec3(
                        forwardLength * 0.5, upLength * 0.5, sideLength * 0.5);
                    const double endpointProgress = tie == 0 ? 0.0 : 1.0;
                    const CadVec3 tangent = conveyorTangentAt(
                        conveyor, endpointProgress, true);
                    const double speed = parameters->accessoryConveyorSpeedMmS;
                    transfer.surfaceLinearVelocityMmS = CadVec3(
                        tangent.x * speed, tangent.y * speed, tangent.z * speed);
                    meshes.push_back(std::move(transfer));
                }
                continue;
            }
        }
        ConveyorPhysics::CollisionMesh collision;
        collision.pose = parentWorldTransformOf(child.get()) * child->loc;
        collision.verticesMm = geometry->vertices;
        collision.indices = geometry->indices;
        meshes.push_back(std::move(collision));
    }
}

void appendFloorPhysicsCollision(std::vector<ConveyorPhysics::CollisionMesh>& meshes) {
    if (!g_defaultFloorNode || g_defaultFloorNode->children.empty()) return;
    CadNode* visual = g_defaultFloorNode->children.front().get();
    const MeshGeometryData* geometry = visual ? visual->asMeshGeometry() : nullptr;
    if (!geometry || !geometry->loaded) return;
    const double sizeX = geometry->bounds[3] - geometry->bounds[0];
    const double sizeY = geometry->bounds[4] - geometry->bounds[1];
    const double sizeZ = geometry->bounds[5] - geometry->bounds[2];
    if (!(sizeX > 0.0 && sizeY > 0.0 && sizeZ > 0.0)) return;
    CadTransform center;
    center.values[3] = (geometry->bounds[0] + geometry->bounds[3]) * 0.5;
    center.values[7] = (geometry->bounds[1] + geometry->bounds[4]) * 0.5;
    center.values[11] = (geometry->bounds[2] + geometry->bounds[5]) * 0.5;
    ConveyorPhysics::CollisionMesh floor;
    floor.pose = (parentWorldTransformOf(visual) * visual->loc) * center;
    floor.boxHalfExtentsMm = CadVec3(sizeX * 0.5, sizeY * 0.5, sizeZ * 0.5);
    meshes.push_back(std::move(floor));
}

double closestConveyorProgress(CadNode* conveyor, const CadVec3& worldPoint) {
    const TransformNodeData* parameters = conveyorParameters(conveyor);
    if (!parameters) return 0.0;
    return closestConveyorProgress(*parameters, parentWorldTransformOf(conveyor) * conveyor->loc,
                                   worldPoint);
}

CadNode* nearestConveyorForRelease(const CadVec3& worldPoint, double* progress) {
    CadNode* best = nullptr;
    double bestProgress = 0.0;
    double bestDistanceSquared = std::numeric_limits<double>::max();
    for (const StationAccessoryInstance& accessory : g_scene.station.accessories) {
        CadNode* conveyor = accessory.node;
        const TransformNodeData* parameters = conveyorParameters(conveyor);
        if (!parameters || accessory.hidden) continue;
        const double candidateProgress = closestConveyorProgress(conveyor, worldPoint);
        const CadTransform world = parentWorldTransformOf(conveyor) * conveyor->loc;
        const CadVec3 pathPoint = conveyorTransformPoint(
            world, conveyorPathPoseAt(*parameters, candidateProgress));
        const double dx = worldPoint.x - pathPoint.x;
        const double dy = worldPoint.y - pathPoint.y;
        const double dz = worldPoint.z - pathPoint.z;
        const double distanceSquared = dx * dx + dy * dy + dz * dz;
        const double captureDistance = std::max(500.0, parameters->accessoryWidthMm * 0.75);
        if (distanceSquared <= captureDistance * captureDistance &&
            distanceSquared < bestDistanceSquared) {
            best = conveyor;
            bestProgress = candidateProgress;
            bestDistanceSquared = distanceSquared;
        }
    }
    if (progress) *progress = bestProgress;
    return best;
}

CadVec3 conveyorTangentAt(CadNode* conveyor, double progress, bool forward) {
    const TransformNodeData* parameters = conveyorParameters(conveyor);
    if (!parameters) return CadVec3();
    return conveyorTangentAt(*parameters, parentWorldTransformOf(conveyor) * conveyor->loc, progress,
                             forward);
}

bool physicalBodyReachedEnd(const ConveyorWorkpiece& piece, const CadVec3& center,
                            double travelAllowanceMm) {
    const TransformNodeData* parameters = conveyorParameters(piece.conveyor);
    if (!parameters) return false;
    if (piece.forward ? piece.progress < 0.90 : piece.progress > 0.10) return false;
    const double endpointProgress = piece.forward ? 1.0 : 0.0;
    const CadTransform world = parentWorldTransformOf(piece.conveyor) * piece.conveyor->loc;
    const CadVec3 endpoint = conveyorTransformPoint(
        world, conveyorPathPoseAt(*parameters, endpointProgress));
    const CadVec3 outward = conveyorTangentAt(piece.conveyor, endpointProgress, piece.forward);
    const CadVec3 offset(center.x - endpoint.x, center.y - endpoint.y, center.z - endpoint.z);
    const double crossed = offset.x * outward.x + offset.y * outward.y + offset.z * outward.z;
    const double distance = std::sqrt(offset.x * offset.x + offset.y * offset.y + offset.z * offset.z);
    return crossed >= 0.0 && distance <= std::max(parameters->accessoryWidthMm * 0.65,
                                                   travelAllowanceMm + piece.radiusMm);
}

bool deleterVolumeIntersects(const ConveyorWorkpiece& piece,
                             const CadVec3& currentCenter) {
    const TransformNodeData* parameters = conveyorParameters(piece.conveyor);
    if (!parameters || parameters->accessoryConveyorRole != "deleter") return false;
    constexpr double enclosureHeightMm = 360.0;
    const ConveyorPathPose a = conveyorPathPoseAt(*parameters, 0.5);
    const ConveyorPathPose b = conveyorPathPoseAt(*parameters, 1.0);
    CadVec3 forward(b.x - a.x, b.y - a.y, b.z - a.z);
    const double forwardLength = std::sqrt(forward.x * forward.x + forward.y * forward.y +
                                           forward.z * forward.z);
    if (forwardLength < 1.0e-6) return false;
    forward = CadVec3(forward.x / forwardLength, forward.y / forwardLength,
                      forward.z / forwardLength);
    const double horizontal = std::max(
        1.0e-6, std::sqrt(forward.x * forward.x + forward.z * forward.z));
    const CadVec3 side(-forward.z / horizontal, 0.0, forward.x / horizontal);
    const CadVec3 up(side.y * forward.z - side.z * forward.y,
                     side.z * forward.x - side.x * forward.z,
                     side.x * forward.y - side.y * forward.x);
    const double surfaceOffset = conveyorSurfaceOffsetMm(*parameters);
    const CadVec3 localCenter(
        (a.x + b.x) * 0.5 + up.x * (surfaceOffset + enclosureHeightMm * 0.5),
        (a.y + b.y) * 0.5 + up.y * (surfaceOffset + enclosureHeightMm * 0.5),
        (a.z + b.z) * 0.5 + up.z * (surfaceOffset + enclosureHeightMm * 0.5));
    const CadTransform world = parentWorldTransformOf(piece.conveyor) * piece.conveyor->loc;
    const CadVec3 center = world * localCenter;
    const CadVec3 worldForward = conveyorNormalized(rotate(world, forward));
    const CadVec3 worldUp = conveyorNormalized(rotate(world, up));
    const CadVec3 worldSide = conveyorNormalized(rotate(world, side));
    const CadVec3 offset(currentCenter.x - center.x, currentCenter.y - center.y,
                         currentCenter.z - center.z);
    // Sweep the previous center to the current center through the enclosure expanded by the
    // workpiece's horizontal bounding sphere. This cannot tunnel when many 60 Hz steps complete between GUI
    // frames, and deleting on first physical overlap matches the visible machine block.
    const double productExtent = std::sqrt(
        piece.radiusMm * piece.radiusMm + piece.heightMm * piece.heightMm * 0.25);
    const std::array<CadVec3, 3> axes{{worldForward, worldUp, worldSide}};
    const std::array<double, 3> halfExtents{{
        forwardLength * 0.5 + productExtent,
        enclosureHeightMm * 0.5 + productExtent,
        std::max(180.0, parameters->accessoryWidthMm - 80.0) * 0.5 + productExtent}};
    const CadVec3 start = piece.hasPreviousPhysicalCenter
        ? CadVec3(piece.previousPhysicalCenter.x - center.x,
                  piece.previousPhysicalCenter.y - center.y,
                  piece.previousPhysicalCenter.z - center.z)
        : offset;
    double minimumT = 0.0;
    double maximumT = 1.0;
    for (size_t axisIndex = 0; axisIndex < axes.size(); ++axisIndex) {
        const CadVec3& axis = axes[axisIndex];
        const double startProjection = start.x * axis.x + start.y * axis.y + start.z * axis.z;
        const double endProjection = offset.x * axis.x + offset.y * axis.y + offset.z * axis.z;
        const double delta = endProjection - startProjection;
        if (std::abs(delta) < 1.0e-9) {
            if (std::abs(startProjection) > halfExtents[axisIndex]) return false;
            continue;
        }
        double enter = (-halfExtents[axisIndex] - startProjection) / delta;
        double leave = (halfExtents[axisIndex] - startProjection) / delta;
        if (enter > leave) std::swap(enter, leave);
        minimumT = std::max(minimumT, enter);
        maximumT = std::min(maximumT, leave);
        if (minimumT > maximumT) return false;
    }
    return true;
}

void rebuildConveyorRuntime() {
    clearConveyorWorkpieces();
    g_scene.conveyorTimeAccumulatorSeconds = 0.0;
    std::vector<ConveyorPhysics::CollisionMesh> physicsMeshes;
    for (const StationAccessoryInstance& entry : g_scene.station.accessories) {
        TransformNodeData* parameters = conveyorParameters(entry.node);
        if (!parameters) continue;
        if (resolvedConveyorMode(*parameters) == ConveyorSimulationMode::PhysX) {
            appendPhysicsCollisionMeshes(entry.node, physicsMeshes);
        }
        if (parameters->accessoryConveyorRole == "spawner") {
            ConveyorSpawnerRuntime spawner;
            spawner.conveyor = entry.node;
            g_scene.conveyorSpawners.push_back(spawner);
            // The countdown itself is the core's. It emits immediately on Start and uses the
            // configured cadence from there on.
            addSceneConveyorSpawner(entry.node);
        }
    }
    if (!physicsMeshes.empty()) appendFloorPhysicsCollision(physicsMeshes);
    std::string physicsError;
    if (!g_scene.conveyorPhysics.reset(physicsMeshes, &physicsError)) {
        g_scene.stationSimulationStatus = physicsError;
    } else {
        rebuildToolActuatorRuntime();
        updatePhysicsToolJaws();
        g_scene.stationSimulationStatus.clear();
        for (const StationAccessoryInstance& entry : g_scene.station.accessories) {
            TransformNodeData* parameters = conveyorParameters(entry.node);
            if (!parameters || parameters->accessoryInitialWorkpieceCount <= 0 ||
                parameters->accessorySpawnObjectId.empty()) continue;
            const double pathLength = conveyorPathLengthMm(*parameters);
            for (int index = 0; index < parameters->accessoryInitialWorkpieceCount; ++index) {
                const double fromEnd = parameters->accessoryInitialWorkpieceEndInsetMm +
                    parameters->accessoryInitialWorkpieceSpacingMm * index;
                spawnConveyorWorkpiece(entry.node, 1.0 - fromEnd / pathLength);
            }
        }
    }
}

void stepConveyorTick(double seconds, bool stepPhysics) {
    stepSceneConveyorRules(seconds);
    updatePhysicsToolJaws();
    if (stepPhysics) {
        applyPhysicsToolForces(seconds);
        g_scene.conveyorPhysics.step(seconds);
    }
    for (ConveyorWorkpiece& piece : g_scene.conveyorWorkpieces) {
        if (!piece.physicsBody || !piece.conveyor || !piece.node) continue;
        CadTransform worldPose;
        if (!g_scene.conveyorPhysics.bodyPose(piece.physicsBody, &worldPose)) continue;
        const CadTransform conveyorWorld = parentWorldTransformOf(piece.conveyor) * piece.conveyor->loc;
        piece.node->loc = conveyorWorld.rigidInverse() * worldPose;
        piece.node->needsGlobalLocUpdate = true;
    }
}

void stepConveyors(double frameSeconds) {
    if (g_scene.stationRunState != StationRunState::Running || !(frameSeconds > 0.0)) return;
    const int speedIndex = std::max(0, std::min(g_scene.liveSpeedIndex, kLiveSpeedCount - 1));
    const double speedFactor = kLiveSpeedFactors[speedIndex];
    constexpr double kFixedStepSeconds = ConveyorPhysics::fixedStepSeconds();
    if (g_scene.conveyorPhysics.asyncActive()) {
        g_scene.conveyorPhysics.setAsyncSpeedFactor(speedFactor);
        const uint64_t completedSteps = g_scene.conveyorPhysics.consumeCompletedAsyncSteps();
        if (completedSteps > 0) {
            // PhysX has already advanced these fixed ticks on its worker. Apply timers, routing,
            // and the newest immutable pose snapshot once, regardless of how many ticks completed.
            stepConveyorTick(static_cast<double>(completedSteps) * kFixedStepSeconds, false);
        }
        return;
    }
    // Forty-eight substeps sustains 10x at ordinary interactive frame rates without allowing the
    // deliberately extreme 100x notch to freeze the UI. If the request exceeds that budget, excess
    // wall-clock debt is discarded; every backend still advances by the same stable amount.
    constexpr int kMaxSubstepsPerFrame = 48;
    constexpr double kMaximumAccumulatedSeconds =
        kFixedStepSeconds * static_cast<double>(kMaxSubstepsPerFrame);
    const double requestedSeconds = std::min(0.1, frameSeconds) * speedFactor;
    g_scene.conveyorTimeAccumulatorSeconds = std::min(
        kMaximumAccumulatedSeconds,
        g_scene.conveyorTimeAccumulatorSeconds + requestedSeconds);

    int substeps = 0;
    while (g_scene.conveyorTimeAccumulatorSeconds + 1.0e-12 >= kFixedStepSeconds &&
           substeps < kMaxSubstepsPerFrame) {
        stepConveyorTick(kFixedStepSeconds);
        g_scene.conveyorTimeAccumulatorSeconds -= kFixedStepSeconds;
        ++substeps;
    }
    // Do not let floating-point subtraction leave a tiny negative remainder that delays the next
    // frame by a tick.
    g_scene.conveyorTimeAccumulatorSeconds =
        std::max(0.0, g_scene.conveyorTimeAccumulatorSeconds);
}

