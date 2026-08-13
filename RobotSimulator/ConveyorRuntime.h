#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "AccessoryGeometry.h"
#include "AppState.h"
#include "CadNode.h"
#include "ConveyorPhysics.h"
#include "RobotProgramModel.h"
#include "StationPackage.h"

// Defined beside the scene state: the runtime below reports scene-graph changes through this
// hook instead of touching the renderer itself.
void conveyorMarkSceneDirty();
uint64_t conveyorSceneDirtyMarkCount();

TransformNodeData* conveyorParameters(CadNode* node);
const TransformNodeData* conveyorParameters(const CadNode* node);
ConveyorSimulationMode resolvedConveyorMode(const TransformNodeData& parameters);
CadTransform nodeWorldTransform(const CadNode* node);
CadTransform actuatorTcpWorld(const ToolActuatorRuntime& runtime);
void detachConveyorWorkpiece(const ConveyorWorkpiece& workpiece);
void clearConveyorWorkpieces();
std::shared_ptr<CadNode> cloneCadNodeTreeSharedGeometry(const CadNode* source);
void includeCadNodeBounds(const CadNode* node, const CadTransform& parent,
                          CadVec3& minimum, CadVec3& maximum, bool& haveBounds);
void commandToolActuator(const ActuateData& command);
bool commandProgramTool(RobotInstance& robot, const SetToolData& command,
                        std::string* errorMessage = nullptr);
void updatePhysicsToolJaws();
void applyPhysicsToolForces(double seconds);
void rebuildToolActuatorRuntime();
ToolActuatorRuntime* findToolActuatorRuntime(const std::string& mechanismId,
                                             const std::string& actuatorId);
const StationAccessoryInstance* conveyorSpawnPrototype(const TransformNodeData& parameters);
size_t activeSpawnCount(CadNode* spawner);
void spawnConveyorWorkpiece(CadNode* conveyor, double requestedProgress = -1.0);

struct ConveyorTransferTarget {
    CadNode* conveyor = nullptr;
    bool forward = true;
};
void placeConveyorWorkpiece(ConveyorWorkpiece& workpiece);
double closestConveyorProgress(CadNode* conveyor, const CadVec3& worldPoint);
ConveyorTransferTarget nextConveyor(CadNode* current, bool leavingForward);
void reparentConveyorWorkpiece(ConveyorWorkpiece& workpiece, CadNode* destination);
bool physicalBodyReachedEnd(const ConveyorWorkpiece& piece, const CadVec3& center,
                            double travelAllowanceMm = 0.0);
bool deleterVolumeIntersects(const ConveyorWorkpiece& piece, const CadVec3& currentCenter);
void appendPhysicsCollisionMeshes(CadNode* conveyor,
                                  std::vector<ConveyorPhysics::CollisionMesh>& meshes);
void appendFloorPhysicsCollision(std::vector<ConveyorPhysics::CollisionMesh>& meshes);
void rebuildConveyorRuntime();
void stepConveyorTick(double seconds, bool stepPhysics = true);
void stepConveyors(double frameSeconds);
